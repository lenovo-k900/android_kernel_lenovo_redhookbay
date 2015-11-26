/*
 * Support for OmniVision OV2722 1080p HD camera sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "ov2722.h"
#define to_ov2722_sensor(sd) container_of(sd, struct ov2722_device, sd)
#define COMBINE(x, y)	(((0xff & x) << 8 ) | (0xff & y))

static int debug = 2;
static int real_model_id;
static int R_g, G_g, B_g;
static int opt_flag = 0;
typedef unsigned int sensor_register;
struct sensor_mode_data {
	sensor_register coarse_integration_time_min;
	sensor_register coarse_integration_time_max_margin;
	sensor_register fine_integration_time_min;
	sensor_register fine_integration_time_max_margin;
	sensor_register fine_integration_time_def;
	sensor_register frame_length_lines;
	sensor_register line_length_pck;
	sensor_register read_mode;
    int vt_pix_clk_freq_mhz;
    unsigned int crop_horizontal_start;
    unsigned int crop_vertical_start;
    unsigned int crop_horizontal_end;
    unsigned int crop_vertical_end;
    unsigned int output_width;
    unsigned int output_height;
    uint8_t binning_factor_x;
    uint8_t binning_factor_y;

};

module_param_call(real_model_id, NULL, param_get_int,&real_model_id,S_IRUGO);
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "ov2722 debug output");

#define lilp3_debug(level, fmt, arg...) \
  do{ \
	  if (level > debug) \
		printk( fmt , ## arg); \
	}while(0)

static int ov2722_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val);
static int ov2722_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val);
static int ov2722_init(struct v4l2_subdev *sd, u32 val);

/*
 * i2c read/write stuff
 */
static int ov2722_read(struct i2c_client *c, u16 reg, u32 * value)
{
	int ret;
	int i;
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	u8 ret_val = 0;
	*value = 0;

	/* Read needs two message to go */
	memset(&msg, 0, sizeof(msg));
	msgbuf[0] = 0;
	msgbuf[1] = 0;
	i = 0;

	msgbuf[i++] = reg >> 8;
	msgbuf[i++] = reg & 0xff;
	msg[0].addr = c->addr;
	msg[0].buf = msgbuf;
	msg[0].len = i;

	msg[1].addr = c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &ret_val;
	msg[1].len = 1;

	ret = i2c_transfer(c->adapter, &msg[0], 2);
	*value = ret_val;

	ret = (ret == 2) ? 0 : -EIO;

	if (ret < 0)
		v4l2_err(c, "slave addr = 0x%x, reg = 0x%x, val = 0x%x -%s\n",
	       c->addr, reg, ret_val,  "Read failed");

	return ret;
}

static int ov2722_write(struct i2c_client *c, u16 reg, u8 value)
{
	int i;
	struct i2c_msg msg;
	u8 msgbuf[3];
	int ret;

	/* Writing only needs one message */
	memset(&msg, 0, sizeof(msg));
	i = 0;
	msgbuf[i++] = reg >> 8;
	msgbuf[i++] = reg;
	msgbuf[i++] = value;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.buf = msgbuf;
	msg.len = i;

	ret = i2c_transfer(c->adapter, &msg, 1);
	ret = (ret == 1) ? 0 : -EIO;

	if (ret < 0)
		dev_err(&c->dev, "slave addr = 0x%x, reg = 0x%x -%s\n",
	       c->addr, reg,  "Write failed");

	return ret;
}

static int ov2722_write_regs(struct i2c_client *c,
	const struct s_register_setting *regs)
{
	const struct s_register_setting *p = regs;
	int ret = 0;
	u32 val = 0;

	while (!is_last_reg_setting(*p)) {
		ret = ov2722_write(c, p->reg, p->val);
		if (ret < 0)
			goto  err;
		ret = ov2722_read(c, p->reg, &val);

		p++;

		udelay(150);
	}

err:
	return ret;
}
static int RG_Ratio_Typical=0x121;
static int BG_Ratio_Typical=0x13F;

// index: index of otp group. (0, 1, 2)
// return:0, group index is empty
//1, group index has invalid data
//2, group index has valid data
int check_otp(struct v4l2_subdev *sd,int index)
{
	int temp, i;
	int address;
	struct i2c_client *client;
	client = v4l2_get_subdevdata(sd);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	// read otp into buffer
	ov2722_write(client,0x3d81, 0x01);
	msleep(10);
	address = 0x3d05 + index*9;
	ov2722_read(client,address,&temp);
	// disable otp read
	ov2722_write(client,0x3d81, 0x00);
	// clear otp buffer
	for (i=0;i<32;i++) {
		ov2722_write(client,0x3d00 + i, 0x00);
	}
	lilp3_debug(LILP3_INFO,"lilp3, check_otp temp=0x%x\n", temp);

	if (!temp) {
		return 0;
	}
	else if ((!(temp & 0x80)) && (temp&0x7f)) {
		return 2;
	}
	else {
		return 1;
	}
}
// index: index of otp group. (0, 1, 2)
// return:0,
int read_otp(struct v4l2_subdev *sd,int index, struct otp_struct * otp_ptr)
{
	int i;
	int address;
	int lsb;
	int temp;
	struct i2c_client *client;
	client = v4l2_get_subdevdata(sd);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	// read otp into buffer
	ov2722_write(client,0x3d81, 0x01);
	msleep(10);
	address = 0x3d05 + index*9;
	ov2722_read(client,address,&temp);
	
	(*otp_ptr).customer_id = (temp & 0x7f);
	ov2722_read(client,address,&temp);

	
	(*otp_ptr).module_integrator_id =temp;
	ov2722_read(client,address + 1, &temp);
	(*otp_ptr).lens_id = temp;
	ov2722_read(client,address + 6,&temp);
	lsb = temp;
	ov2722_read(client,address + 2,&temp);
	(*otp_ptr).rg_ratio = (temp<<2) | ((lsb>>6) & 0x03);
	ov2722_read(client,address + 3, &temp);
	(*otp_ptr).bg_ratio = (temp<<2) | ((lsb>>4) & 0x03);
	ov2722_read(client,address + 7,&temp);
	(*otp_ptr).light_rg = (temp<<2) | ((lsb>>2) & 0x03);
	ov2722_read(client,address + 8,&temp);
	(*otp_ptr).light_bg = (temp<<2) | (lsb & 0x03);
	ov2722_read(client,address + 4,&temp);
	(*otp_ptr).user_data[0] = temp;
	ov2722_read(client,address + 5,&temp);
	(*otp_ptr).user_data[1] = temp;
	// disable otp read
	ov2722_write(client,0x3d81, 0x00);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d customer_id=0x%x\n",__func__,__LINE__,(*otp_ptr).customer_id );
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d module_integrator_id=0x%x\n",__func__,__LINE__,(*otp_ptr).module_integrator_id );
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d lens_id=0x%x\n",__func__,__LINE__,(*otp_ptr).lens_id );
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d rg_ratio=0x%x\n",__func__,__LINE__,(*otp_ptr).rg_ratio );
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d bg_ratio=0x%x\n",__func__,__LINE__,(*otp_ptr).bg_ratio );
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d light_rg=0x%x\n",__func__,__LINE__,(*otp_ptr).light_rg );
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d light_bg=0x%x\n",__func__,__LINE__,(*otp_ptr).light_bg );

	// clear otp buffer
	for (i=0;i<32;i++) {
		ov2722_write(client,0x3d00 + i, 0x00);
	}
	return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int update_awb_gain(struct v4l2_subdev *sd,int R_gain, int G_gain, int B_gain)
{
	struct i2c_client *client;
	client = v4l2_get_subdevdata(sd);
	lilp3_debug(LILP3_INFO,"lilp3, %s : R_gain = %x, G_gain = %x, B_gain = %x, %d\n",__func__,R_gain,G_gain,B_gain,__LINE__);


	if (R_gain>0x400) {
		ov2722_write(client,0x5186, R_gain>>8);
		ov2722_write(client,0x5187, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
		ov2722_write(client,0x5188, G_gain>>8);
		ov2722_write(client,0x5189, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
		ov2722_write(client,0x518a, B_gain>>8);
		ov2722_write(client,0x518b, B_gain & 0x00ff);
	}
	return 0;
}
// call this function after OV2722 initialization
// return value: 0 update success
//1, no OTP
int update_otp(struct v4l2_subdev *sd)
{
	int i;
	int otp_index;
	int temp;
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg,bg;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	for(i=0;i<3;i++) {
		temp = check_otp(sd,i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	lilp3_debug(LILP3_INFO,"update_otp i=%d\n", i);
	if (i==3) {
		// no valid wb OTP data
		return 1;
	}
	read_otp(sd,otp_index, &current_otp);

	if(current_otp.light_rg==0) {
		// no light source information in OTP
		rg = current_otp.rg_ratio;
	}
	else {
		// light source information found in OTP
		rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
	}
	if(current_otp.light_bg==0) {
		// no light source information in OTP
		bg = current_otp.bg_ratio;
	}
	else {
		// light source information found in OTP
		bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
	}
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical) {
		if (rg< RG_Ratio_Typical) {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else {
		if (rg < RG_Ratio_Typical) {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}
	lilp3_debug(LILP3_INFO,"lilp3, %s : R_gain = %x, G_gain = %x, B_gain = %x, %d\n",__func__,R_gain,G_gain,B_gain,__LINE__);

	R_g = R_gain;
	G_g = G_gain;
	B_g = B_gain;
	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct ov2722_device *dev;
	struct i2c_client *client;
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;
	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 1);
		if (ret)
			goto fail_power;
	}
	udelay(150);

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ov2722_device *dev;
	struct i2c_client *client;
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 0);
		if (ret)
			dev_err(&client->dev, "gpio failed 2\n");
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");
	return ret;
}

static int ov2722_s_power(struct v4l2_subdev *sd, int power)
{
#if 0
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	if (power == 0)
		return power_down(sd);
	else{
		if (power_up(sd))
			return -EINVAL;

		return 0;
	}
#endif
	int ret = 0;
	lilp3_debug(LILP3_INFO,"lilp3, %s : power = %d, %d\n",__func__,power,__LINE__);
	if (power == 0) {
		ret = power_down(sd);
	} else {
		ret = power_up(sd);
		if (ret)
			return ret;

		ret = ov2722_init(sd, 0);
	}
	return ret;
}

/*
 * ov2722 chip id get
 */
static int
ov2722_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV2722, 0);

	return 0;
}

/* Return value of the specified register, first try getting it from
 * the register list and if not found, get from the sensor via i2c.
 */
static int ov2722_get_register(struct v4l2_subdev *sd, int reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//const struct s_register_setting *next;
	u32 val;
#if 0
	/* Try if the values is in the register list */
	for (next = reglist; next->type != OV2722_TOK_TERM; next++) {
		if (next->type != OV8830_8BIT) {
			v4l2_err(sd, "only 8-bit registers supported\n");
			return -ENXIO;
		}
		if (next->reg.sreg == reg)
			return next->val;
	}
#endif
	/* If not, read from sensor */
	if (ov2722_read(client, reg, &val)) {
		v4l2_err(sd, "failed to read register 0x%04X\n", reg);
		return -EIO;
	}

	return val;
}

/*
 * ov2722 control query
 */
static int ov2722_t_gain(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *client;
	struct ov2722_device *dev;
	u32 reg_v;
	u32 reg_v2;
	int ret;

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);

	//mutex_lock(&dev->power_lock);
	/* enable manual exp/gain */
	ret = ov2722_read(client, OV2722_AEC_MANUAL_CTRL, &reg_v);
	if (ret < 0)
		goto  err;

	reg_v |= 0x07;
	ret = ov2722_write(client, OV2722_AEC_MANUAL_CTRL, reg_v);
	if (ret < 0)
		goto  err;
	ret = ov2722_read(client, OV2722_AEC_MANUAL_CTRL, &reg_v);
	if (ret < 0)
		goto  err;

	/* set gain */
	ret = ov2722_write(client, OV2722_AGC_ADJ_L, value & 0xff);
	if (ret < 0)
		goto  err;
	ret = ov2722_write(client, OV2722_AGC_ADJ_H, (value >> 8) & 0x03);
	if (ret < 0)
		goto  err;
	ret = ov2722_read(client, OV2722_AGC_ADJ_L, &reg_v);
	if (ret < 0)
		goto  err;
	ret = ov2722_read(client, OV2722_AGC_ADJ_H, &reg_v2);
	if (ret < 0)
		goto  err;

err:
	//mutex_unlock(&dev->power_lock);
	return ret;
}

static int ov2722_t_exposure(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *client;
	struct ov2722_device *dev;
	u32 reg_v;
	int ret;
	u32 fl_lines;

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);
	fl_lines = dev->lines_per_frame;
	lilp3_debug(LILP3_INFO,"=============>%s value=%d, fl_lines=%d\n",__func__, value, fl_lines);
	/*adjust frame rate*/
	if(value > (dev->lines_per_frame-14))
		fl_lines = value+14;
	lilp3_debug(LILP3_INFO,"=============>%s fl_lines=%d\n",__func__,  fl_lines);

	ret = ov2722_write(client,OV2722_TIMING_VTS_H, (u8)(fl_lines >>8));
	if (ret < 0)
		goto  err;

	ret = ov2722_write(client,OV2722_TIMING_VTS_L, (u8)(fl_lines & 0x00FF));	
	if (ret < 0)
		goto  err;	 



	//mutex_lock(&dev->power_lock);
	/* enable manual exp/gain */
	ret = ov2722_read(client, OV2722_AEC_MANUAL_CTRL, &reg_v);
	if (ret < 0)
		goto  err;

	reg_v |= 0x07;
	ret = ov2722_write(client, OV2722_AEC_MANUAL_CTRL, reg_v);
	if (ret < 0)
		goto  err;
	ret = ov2722_read(client, OV2722_AEC_MANUAL_CTRL, &reg_v);
	if (ret < 0)
		goto  err;

	
	ret = ov2722_write(client, OV2722_AEC_PK_EXPO_L, (value & 0x0f) << 4);
	if (ret < 0)
		goto  err;
	ret = ov2722_write(client, OV2722_AEC_PK_EXPO_M, (value >> 4) & 0xff);
	if (ret < 0)
		goto  err;
	ret = ov2722_write(client, OV2722_AEC_PK_EXPO_H, (value >> 12) & 0xff);

	dev->exposure = value;

err:
	//mutex_unlock(&dev->power_lock);
	return ret;
}
#if 0
static int ov2722_v_flip(struct v4l2_subdev *sd, s32 value)
{
	return -ENXIO;
}
#endif

static int ov2722_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct ov2722_device *dev = NULL;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	dev=to_ov2722_sensor(sd);
	*value = dev->exposure;
	return 0;
}


static int ov2722_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV2722_FOCAL_LENGTH_NUM << 16) | OV2722_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov2722_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for ov2722*/
	*val = (OV2722_F_NUMBER_DEFAULT_NUM << 16) | OV2722_F_NUMBER_DEM;
	return 0;
}

static int ov2722_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV2722_F_NUMBER_DEFAULT_NUM << 24) |
		(OV2722_F_NUMBER_DEM << 16) |
		(OV2722_F_NUMBER_DEFAULT_NUM << 8) | OV2722_F_NUMBER_DEM;
	return 0;
}

static int ov2722_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	//struct ov2722_device *dev = to_ov2722_sensor(sd);
	int r = ov2722_get_register(sd, OV2722_TIMING_X_INC);

	if (r < 0)
		return r;

	*val = fls((r >> 4) + (r & 0xF)) - 2;

	return 0;
}

static int ov2722_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	//struct ov2722_device *dev = to_ov2722_sensor(sd);
	int r = ov2722_get_register(sd, OV2722_TIMING_Y_INC);

	if (r < 0)
		return r;

	*val = fls((r >> 4) + (r & 0xF)) - 2;

	return 0;
}


struct ov2722_control ov2722_controls[] = {
#if 0
	{
	 .qc = {
		.id = V4L2_CID_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "global gain",
		.minimum = 0x0,
		.maximum = 0x1ff,
		.step = 0x01,
		.default_value = 0x00,
		.flags = 0,
		},
	 .tweak = ov2722_t_gain,
	 },
	{
	 .qc = {
		.id = V4L2_CID_EXPOSURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "exposure",
		.minimum = 0x0,
		.maximum = 0xFFFF,
		.step = 0x01,
		.default_value = 0x00,
		.flags = 0,
		},
	 .tweak = ov2722_t_exposure,
	 },
#endif
			{
				.qc = {
					.id = V4L2_CID_EXPOSURE_ABSOLUTE,
					.type = V4L2_CTRL_TYPE_INTEGER,
					.name = "exposure",
					.minimum = 0x0,
					.maximum = 0xffff,
					.step = 0x01,
					.default_value = 0x00,
					.flags = 0,
				},
				.query = ov2722_q_exposure,
			},

			{
				.qc = {
					.id = V4L2_CID_FOCAL_ABSOLUTE,
					.type = V4L2_CTRL_TYPE_INTEGER,
					.name = "focal length",
					.minimum = OV2722_FOCAL_LENGTH_DEFAULT,
					.maximum = OV2722_FOCAL_LENGTH_DEFAULT,
					.step = 0x01,
					.default_value = OV2722_FOCAL_LENGTH_DEFAULT,
					.flags = 0,
				},
				.query = ov2722_g_focal,
			},
			{
				.qc = {
					.id = V4L2_CID_FNUMBER_ABSOLUTE,
					.type = V4L2_CTRL_TYPE_INTEGER,
					.name = "f-number",
					.minimum = OV2722_F_NUMBER_DEFAULT,
					.maximum = OV2722_F_NUMBER_DEFAULT,
					.step = 0x01,
					.default_value = OV2722_F_NUMBER_DEFAULT,
					.flags = 0,
				},
				.query = ov2722_g_fnumber,
			},
			{
				.qc = {
					.id = V4L2_CID_FNUMBER_RANGE,
					.type = V4L2_CTRL_TYPE_INTEGER,
					.name = "f-number range",
					.minimum = OV2722_F_NUMBER_RANGE,
					.maximum =	OV2722_F_NUMBER_RANGE,
					.step = 0x01,
					.default_value = OV2722_F_NUMBER_RANGE,
					.flags = 0,
				},
				.query = ov2722_g_fnumber_range,
			},
			{
				.qc = {
					.id = V4L2_CID_BIN_FACTOR_HORZ,
					.type = V4L2_CTRL_TYPE_INTEGER,
					.name = "horizontal binning factor",
					.minimum = 0,
					.maximum = OV2722_BIN_FACTOR_MAX,
					.step = 1,
					.default_value = 0,
					.flags = V4L2_CTRL_FLAG_READ_ONLY,
				},
				.query = ov2722_g_bin_factor_x,
			},
			{
				.qc = {
					.id = V4L2_CID_BIN_FACTOR_VERT,
					.type = V4L2_CTRL_TYPE_INTEGER,
					.name = "vertical binning factor",
					.minimum = 0,
					.maximum = OV2722_BIN_FACTOR_MAX,
					.step = 1,
					.default_value = 0,
					.flags = V4L2_CTRL_FLAG_READ_ONLY,
				},
				.query = ov2722_g_bin_factor_y,
			},

};
#define N_CONTROLS (ARRAY_SIZE(ov2722_controls))

static struct ov2722_control *ov2722_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov2722_controls[i].qc.id == id)
			return ov2722_controls + i;
	return NULL;
}

static int
ov2722_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov2722_control *ctrl = ov2722_find_control(qc->id);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : ctrl->id = %d\n",__func__,__LINE__);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

/*
 * ov2722 control set/get
 */
static int ov2722_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov2722_control *o_ctrl = ov2722_find_control(ctrl->id);
	int ret;

	lilp3_debug(LILP3_DEBUG,"lilp3, %s : ctrl->id = %d, V4L2_CID_BIN_FACTOR_HORZ = %d, V4L2_CID_BIN_FACTOR_VERT = %d, %d\n",__func__,ctrl->id,V4L2_CID_BIN_FACTOR_HORZ,V4L2_CID_BIN_FACTOR_VERT,__LINE__);

	if (!o_ctrl)
		return -EINVAL;

	ret = o_ctrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov2722_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov2722_control *octrl = ov2722_find_control(ctrl->id);
	int ret;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

struct ov2722_format ov2722_formats[] = {
	{
	 .desc = "RGB Bayer Format",
	 .regs = NULL,
	 },
};

struct ov2722_resolution ov2722_res_preview[] = {
	{
	 .desc = "1080P",
	 .width = 1936,
	 .height = 1096,
	 .fps = 24,
	 .used = 0,
	 .pixels_per_line = 2152,
	 .lines_per_frame = 1376,
	 .regs = (struct s_register_setting *)ov2722_1080P_24fps,
	 .skip_frames = 3,
	 },
	{
	 .desc = "FullHD",
	 .width = 1824,
	 .height = 1092,
	 .fps = 24,
	 .used = 0,
	 .pixels_per_line = 2152,
	 .lines_per_frame = 1376,
	 .regs = (struct s_register_setting *)ov2722_FullHD_PREVIEW_24fps,
	 .skip_frames = 3,
	 },
	 {
	  .desc = "1456_1092",
	  .width = 1456,
	  .height = 1092,
	  .fps = 24,
	  .used = 0,
	  .pixels_per_line = 2152,
	  .lines_per_frame = 1376,
	  .regs = (struct s_register_setting *)ov2722_1456_1092_24fps,
	  .skip_frames = 3,
	  },

	{
	.desc = "UXGA_PREVIEW",
	.width = 1336,
	.height = 1092,
	.fps = 24,
	.used = 0,
	.pixels_per_line = 2144,
	.lines_per_frame = 1120,
	.regs = (struct s_register_setting *)ov2722_UXGA_PREVIEW_24fps,
	.skip_frames = 3,
	 },
};

#define N_RES_PREVIEW (ARRAY_SIZE(ov2722_res_preview))


struct ov2722_resolution ov2722_res_still[] = {
	{
	 .desc = "1080P",
	 .width = 1936,
	 .height = 1096,
	 .fps = 24,
	 .used = 0,
	 .pixels_per_line = 2152,
	 .lines_per_frame = 1376,
	 .regs = (struct s_register_setting *)ov2722_1080P_24fps,
	 .skip_frames = 3,
	 },
	{
	 .desc = "FullHD",
	 .width = 1824,
	 .height = 1092,
	 .fps = 24,
	 .used = 0,
	 .pixels_per_line = 2152,
	 .lines_per_frame = 1376,
	 .regs = (struct s_register_setting *)ov2722_FullHD_PREVIEW_24fps,
	 .skip_frames = 3,
	 },
	 {
	  .desc = "1456_1092",
	  .width = 1456,
	  .height = 1092,
	  .fps = 24,
	  .used = 0,
	  .pixels_per_line = 2152,
	  .lines_per_frame = 1376,
	  .regs = (struct s_register_setting *)ov2722_1456_1092_24fps,
	  .skip_frames = 3,
	  },

	{
	.desc = "UXGA_PREVIEW",
	.width = 1336,
	.height = 1092,
	.fps = 24,
	.used = 0,
	.pixels_per_line = 2144,
	.lines_per_frame = 1120,
	.regs = (struct s_register_setting *)ov2722_UXGA_PREVIEW_24fps,
	.skip_frames = 3,
	 },
};

#define N_RES_STILL (ARRAY_SIZE(ov2722_res_still))


struct ov2722_resolution ov2722_res_video[] = {
	{
	 .desc = "1080P",
	 .width = 1936,
	 .height = 1096,
	 .fps = 24,
	 .used = 0,
	 .pixels_per_line = 2152,
	 .lines_per_frame = 1376,
	 .regs = (struct s_register_setting *)ov2722_1080P_24fps,
	 .skip_frames = 3,
	 },
	{
	 .desc = "FullHD",
	 .width = 1824,
	 .height = 1092,
	 .fps = 24,
	 .used = 0,
	 .pixels_per_line = 2152,
	 .lines_per_frame = 1376,
	 .regs = (struct s_register_setting *)ov2722_FullHD_PREVIEW_24fps,
	 .skip_frames = 3,
	 },
	 {
	  .desc = "1456_1092",
	  .width = 1456,
	  .height = 1092,
	  .fps = 24,
	  .used = 0,
	  .pixels_per_line = 2152,
	  .lines_per_frame = 1376,
	  .regs = (struct s_register_setting *)ov2722_1456_1092_24fps,
	  .skip_frames = 3,
	  },

	{
	.desc = "UXGA_PREVIEW",
	.width = 1336,
	.height = 1092,
	.fps = 24,
	.used = 0,
	.pixels_per_line = 2144,
	.lines_per_frame = 1120,
	.regs = (struct s_register_setting *)ov2722_UXGA_PREVIEW_24fps,
	.skip_frames = 3,
	 },
#if 0
	{
	 .desc = "720P",
	 .width = 1296,
	 .height = 736,
	 .fps = 30,
	 .used = 0,
	 .regs = (struct s_register_setting *)ov2722_720P_30fps,
	 .skip_frames = 3,
	 },
#endif
};

#define N_RES_VIDEO (ARRAY_SIZE(ov2722_res_video))

static struct ov2722_resolution *ov2722_res = ov2722_res_preview;
static int N_RES = N_RES_PREVIEW;


#if 0
static int distance(struct ov2722_resolution *res, u32 w, u32 h)
{
	int ret;
	if (res->width < w || res->height < h)
		return -1;

	ret = ((res->width - w) + (res->height - h));
	return ret;
}
#endif

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
/* tune this value so that the DVS resolutions get selected properly,
 * but make sure 16:9 does not match 4:3.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 600
#define LARGEST_ALLOWED_RATIO_MISMATCH2 742

static int distance(struct ov2722_resolution const *res, const u32 w,
				const u32 h)
{
	unsigned int w_ratio = ((res->width<<13)/w);
	unsigned int h_ratio = ((res->height<<13)/h);
	int match   = abs(((w_ratio<<13)/h_ratio) - ((int)8192));
	int match2	= abs((res->width<<13)/res->height-(w<<13)/h);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : res->width = %d, res->height = %d, match2 = %d, %d\n",__func__,match2,res->width,res->height,__LINE__);

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)
		|| (match > LARGEST_ALLOWED_RATIO_MISMATCH) || (match2 >=LARGEST_ALLOWED_RATIO_MISMATCH2))
		return -1;

	return w_ratio + h_ratio;
}

static int nearest_resolution_index(int w, int h, u32 pixelformat)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = 0xffff;
	struct ov2722_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &(ov2722_res[i]);
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	if (idx == -1)
		return -1;

	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ov2722_res[i].width)
			continue;
		if (h != ov2722_res[i].height)
			continue;

		return i;
	}

	return -1;
}

static int ov2722_try_mbus_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ov2722_device *dev;
	struct i2c_client *client;
	int idx;
	lilp3_debug(LILP3_INFO,"lilp3, %s : nearest_resolution_index: fmt->width =%d, fmt->height =%d, %d\n",__func__,fmt->width,fmt->height,__LINE__);

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);

	if (!fmt)
		return -EINVAL;
        if((fmt->width == 336) && (fmt->height == 256))
        {
		idx = nearest_resolution_index(640,
					480,
					0);
	}else {
		idx = nearest_resolution_index(fmt->width,
					fmt->height,
					0);
        }
	if (idx == -1)
		idx = 0;

	fmt->width = ov2722_res[idx].width;
	fmt->height = ov2722_res[idx].height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	lilp3_debug(LILP3_INFO,"lilp3, %s : idx = %d, width = %d, height =%d, %d\n",__func__,idx,ov2722_res[idx].width,ov2722_res[idx].height,__LINE__);
	return 0;
}

static int ov2722_get_intg_factor(struct v4l2_subdev *sd,
				  struct camera_mipi_info *info)
{
	const int ext_clk = 19200000; /* MHz */
	struct i2c_client *client;
    s32 bin_x, bin_y;
	u32 data[2];
	struct atomisp_sensor_mode_data *m = &info->data;
	//struct ov2722_device *dev = to_ov2722_sensor(sd);
	int pll2_prediv;
	int pll2_multiplier;
	int pll2_divs;
	//int pll2_seld5;
	int t1, t2, t3;
	int sclk;
	int div1, div2, div3;
	int div3_f;
	int line_pck;
	int frame_length;
	int temp1,temp2;
	client = v4l2_get_subdevdata(sd);
	memset(&info->data, 0, sizeof(info->data));

	pll2_prediv     = ov2722_get_register(sd, OV2722_PLL2_PREDIV);
	pll2_multiplier = ov2722_get_register(sd, OV2722_PLL2_MULTIPLIER);
	pll2_divs       = ov2722_get_register(sd, OV2722_PLL2_DIVS);

	lilp3_debug(LILP3_DEBUG,"1: pll2_prediv = %d pll2_multiplier = %d pll2_divs = %d\n",pll2_prediv,pll2_multiplier,pll2_divs);

	if (pll2_prediv < 0 || pll2_multiplier < 0 ||
	    pll2_divs < 0 )
	    return -EIO;

	pll2_prediv =(pll2_prediv>>4) &0x07 ;
	pll2_multiplier &= 0x7F;
	pll2_divs &= 0x03;
	lilp3_debug(LILP3_DEBUG,"2: pll2_prediv = %d pll2_multiplier = %d pll2_divs = %d\n",pll2_prediv,pll2_multiplier,pll2_divs);


	if (pll2_prediv <= 0)
	    return -EIO;

	t1 = ext_clk / pll2_prediv;
	t2 = t1 * pll2_multiplier;
	t3 = t2 * pll2_divs;

	div1 = ov2722_get_register(sd, OV2722_PLL2_SCALE_DIVM);
	div2 = ov2722_get_register(sd, OV2722_PLL2_SYS_DIV);
	div3 = ov2722_get_register(sd, OV2722_PLL2_BIT_DIV);
	lilp3_debug(LILP3_DEBUG,"1: div1=%d div2=%d div3=%d\n",div1,div2,div3);

	div1 = (div1>>4)+1;
	div2 = (div2 >>7)+1;
	div3 = div3 & 0x0f;
	lilp3_debug(LILP3_DEBUG,"2: div1=%d div2=%d div3=%d\n",div1,div2,div3);

	if(div3 <9) div3_f =2;
	else if(div3 ==9) div3_f =4;
	else div3_f=5;

	lilp3_debug(LILP3_DEBUG,"3: div1=%d div2=%d div3_f=%d\n",div1,div2,div3_f);
	sclk = (int)(t3 /(div1*div2*div3_f/2));

	m->vt_pix_clk_freq_mhz = sclk/2;

	ov2722_read(client,OV2722_TIMING_HTS_H,&temp1);
	ov2722_read(client,OV2722_TIMING_HTS_L,&temp2);
	line_pck = (temp1<< 8) + temp2 ;
	lilp3_debug(LILP3_DEBUG,"line_pck: temp1 = %d temp2 = %d\n",temp1,temp2);
	ov2722_read(client,OV2722_TIMING_VTS_H,&temp1);
	ov2722_read(client,OV2722_TIMING_VTS_L,&temp2);
	frame_length = (temp1<< 8) + temp2 ;
	lilp3_debug(LILP3_DEBUG,"frame_length: temp1 = %d temp2 = %d\n",temp1,temp2);

	/* HTS and VTS */
	m->frame_length_lines = frame_length;
	m->line_length_pck = line_pck;

	m->coarse_integration_time_min = 0;
	m->coarse_integration_time_max_margin = 14;

	/* OV Sensor do not use fine integration time. */
	m->fine_integration_time_min = 0;
	m->fine_integration_time_max_margin = 0;

	/*
	 * read_mode inicate whether binning is used for calculating
	 * the correct exposure value from the user side. So adapt the
	 * read mode values accordingly.
	 */
	lilp3_debug(LILP3_INFO,"sclk=%d line_pck=%d frame_length=%d\n",sclk,line_pck,frame_length);
	//m->read_mode = ov2722_res[dev->fmt_idx].bin_factor_x ?
	//	     OV8830_READ_MODE_BINNING_ON : OV8830_READ_MODE_BINNING_OFF;
	memset(data, 0, 2 * sizeof(u32));

	ov2722_read(client, OV2722_CROP_X_START_HIGH, &temp1);
	ov2722_read(client, OV2722_CROP_X_START_LOW, &temp2);
	m->crop_horizontal_start = (temp1<< 8) + temp2;

	ov2722_read(client, OV2722_CROP_Y_START_HIGH, &temp1);
	ov2722_read(client, OV2722_CROP_Y_START_LOW, &temp2);
	m->crop_vertical_start = (temp1<< 8) + temp2;

	//ov2722_read(client, OV2722_CROP_X_END_HIGH, &temp1);
	//ov2722_read(client, OV2722_CROP_X_END_LOW, &temp2);
	//m->crop_horizontal_end = (temp1<< 8) + temp2;

	//ov2722_read(client, OV2722_CROP_Y_END_HIGH, &temp1);
	//ov2722_read(client, OV2722_CROP_Y_END_LOW, &temp2);
	//m->crop_vertical_end = (temp1<< 8) + temp2;

	ov2722_read(client, OV2722_OUTPUT_WIDTH_HIGH, &temp1);
	ov2722_read(client, OV2722_OUTPUT_WIDTH_LOW, &temp2);
	m->output_width = (temp1<< 8) + temp2;

	ov2722_read(client, OV2722_OUTPUT_HEIGHT_HIGH, &temp1);
	ov2722_read(client, OV2722_OUTPUT_HEIGHT_LOW, &temp2);
	m->output_height = (temp1<< 8) + temp2;
	
	m->crop_horizontal_end = m->crop_horizontal_start+m->output_width;
	m->crop_vertical_end = m->crop_vertical_start+m->output_height;
	
	ov2722_g_bin_factor_y(sd, &bin_y);
	ov2722_g_bin_factor_y(sd, &bin_x);
	m->binning_factor_x = bin_x + 1;
	m->binning_factor_y = bin_y + 1;
	return 0;
}

static int ov2722_s_mbus_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	const struct s_register_setting *ov2722_def_reg;
	struct camera_mipi_info *ov2722_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	ov2722_info = v4l2_get_subdev_hostdata(sd);
	if (ov2722_info == NULL)
		return -EINVAL;

	ret = ov2722_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);
	lilp3_debug(LILP3_INFO,"lilp3, %s: get_resolution_index : dev->fmt_idx = %d, fmt->width = %d, fmt->height = %d, %d\n",__func__,dev->fmt_idx,fmt->width, fmt->height,__LINE__);

	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	ov2722_def_reg = ov2722_res[dev->fmt_idx].regs;
	ret = ov2722_write_regs(client, ov2722_def_reg);
	if (ret){
		v4l2_err(sd, "I2C write fail\n");
		return -EINVAL;
	}
	dev->fps = ov2722_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ov2722_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ov2722_res[dev->fmt_idx].lines_per_frame;
#if 1
	ret = ov2722_get_intg_factor(sd, ov2722_info);
	if (ret) {
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}

	/* restore exposure, gain settings */
	if (dev->exposure) {
	    ret = ov2722_t_exposure(sd, dev->exposure);
	    if (ret)
	        v4l2_warn(sd, "failed to set exposure time\n");
	    ret = ov2722_t_gain(sd, dev->gain);
	        if (ret)
	        v4l2_warn(sd, "failed to set exposure time\n");
	}
#endif

	return 0;
}

static int ov2722_g_mbus_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : dev->fmt_idx = %d, fmt->width = %d, fmt->height = %d, %d\n",__func__,dev->fmt_idx,ov2722_res[dev->fmt_idx].width,ov2722_res[dev->fmt_idx].height,__LINE__);

	if (!fmt)
		return -EINVAL;

	fmt->width = ov2722_res[dev->fmt_idx].width;
	fmt->height = ov2722_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov2722_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 high, low;
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	/*
	 * 1. i2c check
	 */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/*
	 * 3. check sensor chip ID
	 */
	ret = ov2722_read(client, OV2722_SC_CMMN_CHIP_ID_H, &high);
	if (ret) {
		v4l2_err(client, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov2722_read(client, OV2722_SC_CMMN_CHIP_ID_L, &low);
	*id = ((((u16) high) << 8) | (u16) low);
	v4l2_info(client, "sensor_id = 0x%x\n", *id);
	real_model_id = *id;

	if (*id != OV2722_ID) {
		v4l2_err(client, "sensor ID error\n");
		return -ENODEV;
	}

	ov2722_read(client, OV2722_SC_CMMN_SUB_ID, &high);
	*revision = (u8) high & 0x0f;
	v4l2_info(client, "sensor_revision = 0x%x\n", *revision);

	v4l2_info(client, "detect ov2722 success\n");

	return 0;
}
#if 0
static int ov2722_stream_init(struct v4l2_subdev *sd, int enable)
{
	struct ov2722_device *dev;
	struct i2c_client *client;
	int ret = 0;
	struct s_register_setting *ov2722_def_reg;

	u8 sensor_revision;
	u16 sensor_id;

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);
	if (enable) {
		ret = ov2722_s_power(sd, 1);
		if (ret) {
			v4l2_err(client, "ov2722 power-up err.\n");
			return ret;
		}
		/*
		 * config & detect sensor
		 */
		ret = ov2722_detect(client, &sensor_id, &sensor_revision);
		if (ret) {
			v4l2_err(client, "ov2722_detect err.\n");
			return ret;
		}

		dev->sensor_id = sensor_id;
		dev->sensor_revision = sensor_revision;

		/*
		 * sw reset sensor
		 */
		ov2722_write(client, (u32) OV2722_SW_RESET, 0x01);
		v4l2_info(client, "sw reset, fmt_idx %d\n", dev->fmt_idx);

		udelay(150);

		ov2722_def_reg = ov2722_res[dev->fmt_idx].regs;
		ov2722_write_regs(client, ov2722_def_reg);
	} else {
		ret = ov2722_s_power(sd, 0);
		if (ret) {
			v4l2_err(client, "ov2722 power-down err.\n");
			return ret;
		}

	}
	return 0;
}
#endif
/*
 * ov2722 stream on/off
 */
static int ov2722_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret=0;
#if 0
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : enable = %d,  %d\n",__func__,enable,__LINE__);
	ov2722_read(client, 0x3018, &high);

	lilp3_debug(LILP3_INFO,"lilp3, %s : 0x3018 = %x  %d\n",__func__,high,__LINE__);
	if((high & 0x04) == 0x04){
		lilp3_debug(LILP3_INFO,"lilp3, %s : 0x3018 = %x  %d\n",__func__,high&(~0x04),__LINE__);
		//ov2722_write(client, (u32) OV2722_MIPI_CTRL, ~(high&0x04));
		ov2722_write(client, (u32) OV2722_MIPI_CTRL, high&(~0x04));
	}

	ret = ov2722_write(client, (u32) OV2722_SW_STREAM, enable ? 1 : 0);
	if (ret != 0) {
		v4l2_err(client, "failed to set streaming\n");
		return ret;
	}
	mdelay(100);
	if(enable == 1)
	{
			high= high | 0x04;
			ov2722_write(client, (u32)0x3018, high);
			update_otp(sd);
	}
	ov2722_read(client, 0x3018, &high);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : 0x3018 = %x  %d\n",__func__,high,__LINE__);

	dev->streaming = enable;

	/* restore settings */
	ov2722_res = ov2722_res_preview;
	N_RES = N_RES_PREVIEW;

	return 0;
#endif
	if (enable) {
		ret = ov2722_write(client, (u32) OV2722_SW_STREAM, 0x01);
		if (ret < 0)
			goto err;
		msleep(1);
		if(!opt_flag){
			update_otp(sd);
			update_awb_gain(sd,R_g, G_g, B_g);
			msleep(1);
			opt_flag = 1;
		}
	} else {
		ret = ov2722_write(client, (u32) OV2722_SW_STREAM, 0x00);
		if (ret < 0)
			goto err;
		msleep(10);
	}

err:
	return ret;
}

static int
ov2722_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : dev->run_mode = %x, %d\n",__func__,dev->run_mode,__LINE__);

	dev->run_mode = param->parm.capture.capturemode;

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ov2722_res = ov2722_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		ov2722_res = ov2722_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		ov2722_res = ov2722_res_preview;
		N_RES = N_RES_PREVIEW;
	}

	return 0;
}

static int ov2722_g_priv_int_data(struct v4l2_subdev *sd,
				struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct ov2722_device *dev = container_of(sd, struct ov2722_device, sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	ret = copy_to_user(to, &current_otp, read_size);
	if (ret) {
		v4l2_err(client, "%s: failed to copy OTP data to user\n", __func__);
		return -EFAULT;
	}

out:
	priv->size = sizeof(struct otp_struct);

	return 0;
}

/*
 * ov2722 enum frame size, frame intervals
 */
static int ov2722_enum_framesizes(struct v4l2_subdev *sd,
	struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov2722_res[index].width;
	fsize->discrete.height = ov2722_res[index].height;
	fsize->reserved[0] = ov2722_res[index].used;

	return 0;
}

static int ov2722_enum_frameintervals(struct v4l2_subdev *sd,
	struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (index >= N_RES)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = ov2722_res[index].width;
	fival->height = ov2722_res[index].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov2722_res[index].fps;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2722_g_register(struct v4l2_subdev *sd,
	struct v4l2_dbg_register *reg)
{
	return 0;
}

static int ov2722_s_register(struct v4l2_subdev *sd,
	struct v4l2_dbg_register *reg)
{
	return 0;
}
#endif
#if 0
static int ov2722_enum_fmt(struct v4l2_subdev *sd,
	struct v4l2_fmtdesc *fmtdesc)
{
	if (!fmtdesc)
		return -EINVAL;

	if (fmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(sd, "unsupported buffer type.\n");
		return -EINVAL;
	}

	return 0;
}
#endif
static int ov2722_enum_mbus_fmt(struct v4l2_subdev *sd,
	unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	*code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}
#if 0
static int ov2722_log_status(struct v4l2_subdev *sd)
{
	struct ov2722_device *dev;
	struct i2c_client *client;

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);

	/*
	 * TBD
	 */

	return 0;
}

static int ov2722_initdevice(struct i2c_client *c)
{
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	return 0;
}
#endif

static int ov2722_s_config(struct v4l2_subdev *sd,
	int irq, void *pdata)
{
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			v4l2_err(client, "ov2722 platform init err\n");
			return ret;
		}
	}
	ret = ov2722_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "ov2722 power-up err.\n");
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ov2722_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ov2722_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	/* power off sensor */
	ret = ov2722_s_power(sd, 0);
	if (ret) {
		v4l2_err(client, "ov2722 power-down err.\n");
		return ret;
	}

	return 0;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	ov2722_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;

}
#if 0
static int ov2722_g_parm(struct v4l2_subdev *sd,
	struct v4l2_streamparm *param)
{
	struct ov2722_device *dev;
	struct i2c_client *client;

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);

	if (!param)
		return -EINVAL;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(sd,  "unsupported buffer type.\n");
		return -EINVAL;
	}

	/* support nothing */
	memset(&param->parm, 0, sizeof(struct v4l2_captureparm));

	return 0;
}
#endif
#if 0
static int ov2722_s_parm(struct v4l2_subdev *sd,
	struct v4l2_streamparm *param)
{
	struct ov2722_device *dev;
	struct i2c_client *client;

	client = v4l2_get_subdevdata(sd);
	dev = container_of(sd, struct ov2722_device, sd);


	if (!param)
		return -EINVAL;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(sd,  "unsupported buffer type.\n");
		return -EINVAL;
	}
#if 0
	/* we don't support frame rate control in driver. */
	if (param->parm.capture.capability | V4L2_CAP_TIMEPERFRAME) {
		v4l2_err(sd,   "time per frame is not supported.\n");
		return -EINVAL;
	}
#endif
	return 0;
}
#endif

static int ov2722_s_exposure(struct v4l2_subdev *sd,
			      struct atomisp_exposure *exposure)
{
	int ret;
	int exp = exposure->integration_time[0];
	int gain = exposure->gain[0];
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : exp = %d, gain = %d ,%d\n",__func__,exp,gain,__LINE__);

	ret = ov2722_t_exposure(sd, exp);
	if (ret)
		v4l2_warn(sd, "failed to set exposure time\n");
	ret = ov2722_t_gain(sd, gain);
	if (ret)
		v4l2_warn(sd, "failed to set exposure time\n");

	return ret;
}


static long ov2722_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ov2722_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return ov2722_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int ov2722_init_registers(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/*
	 * sw reset sensor
	 */
	ret = ov2722_write(client, (u32) OV2722_SW_RESET, 0x01);
	v4l2_info(client, "sw reset\n");

	udelay(150);

	ret |= ov2722_write_regs(client, ov2722_BasicSettings);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : ov2722_write_regs : ret = %d, %d\n",__func__,ret,__LINE__);

	update_awb_gain(sd,R_g, G_g, B_g);
	msleep(1);

	return ret;
}

static int __ov2722_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/* set inital registers */
	ret = ov2722_init_registers(sd);
#if 0
	/* restore settings */
	ov8830_res = ov8830_res_preview;
	N_RES = N_RES_PREVIEW;
#endif
	return ret;
}


static int ov2722_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/* set inital registers */
	ret = __ov2722_init(sd, val);

	/* restore settings */
	ov2722_res = ov2722_res_preview;
	N_RES = N_RES_PREVIEW;

	return ret;
}


#define MAX_FMTS 1
static int ov2722_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_mbus_code_enum *code)
{
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	if (code->index >= MAX_FMTS)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov2722_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov2722_res[index].width;
	fse->min_height = ov2722_res[index].height;
	fse->max_width = ov2722_res[index].width;
	fse->max_height = ov2722_res[index].height;

	return 0;

}
#if 0
static int ov2722_enum_frame_ival(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client;
	client = v4l2_get_subdevdata(sd);

	return -EINVAL;
}
#endif
static struct v4l2_mbus_framefmt *
__ov2722_get_pad_format(struct ov2722_device *sensor,
	struct v4l2_subdev_fh *fh, unsigned int pad,
	enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client;
	client = v4l2_get_subdevdata(&sensor->sd);
	lilp3_debug(LILP3_DEBUG,"lilp3,%s : %d\n",__func__,__LINE__);
	if (pad != 0) {
		v4l2_err(client, "__ov2722_get_pad_format err. pad %x\n", pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int ov2722_get_pad_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *fmt)
{
	struct ov2722_device *snr = container_of(sd, struct ov2722_device, sd);
	struct v4l2_mbus_framefmt *format;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	format = __ov2722_get_pad_format(snr, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int ov2722_set_pad_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *fmt)
{
	struct ov2722_device *snr = container_of(sd, struct ov2722_device, sd);
	struct v4l2_mbus_framefmt *format;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	format = __ov2722_get_pad_format(snr, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int
ov2722_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;

	/*
	 * if no specific information to calculate the fps,
	 * just used the value in sensor settings
	 */
	if (!dev->pixels_per_line || !dev->lines_per_frame) {
		interval->interval.numerator = 1;
		interval->interval.denominator = dev->fps;
		return 0;
	}

	/*
	 * DS: if coarse_integration_time is set larger than
	 * lines_per_frame the frame_size will be expanded to
	 * coarse_integration_time+1
	 */
	if (dev->exposure > dev->lines_per_frame) {
		if (dev->exposure == 0xFFFF) {
			/*
			 * we can not add 1 according to ds, as this will
			 * cause over flow
			 */
			v4l2_warn(client, "%s: abnormal exposure:0x%x\n",
				  __func__, dev->exposure);
			lines_per_frame = dev->exposure;
		} else
			lines_per_frame = dev->exposure + 1;
	} else
		lines_per_frame = dev->lines_per_frame;

	interval->interval.numerator = dev->pixels_per_line *
					lines_per_frame;
	interval->interval.denominator = OV2722_MCLK * 1000000;

	return 0;
}

static int ov2722_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	*frames = ov2722_res[dev->fmt_idx].skip_frames;

	return 0;
}

static const struct v4l2_subdev_video_ops ov2722_video_ops = {
	.try_mbus_fmt = ov2722_try_mbus_fmt,
	.s_mbus_fmt = ov2722_s_mbus_fmt,
	.g_mbus_fmt = ov2722_g_mbus_fmt,
	.s_stream = ov2722_s_stream,
	.enum_framesizes = ov2722_enum_framesizes,
	.enum_frameintervals = ov2722_enum_frameintervals,
	.enum_mbus_fmt = ov2722_enum_mbus_fmt,
	.s_parm = ov2722_s_parm,
	.g_frame_interval = ov2722_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov2722_core_ops = {
	.g_chip_ident = ov2722_g_chip_ident,
	.queryctrl = ov2722_queryctrl,
	.g_ctrl = ov2722_g_ctrl,
	.s_ctrl = ov2722_s_ctrl,
	.s_power = ov2722_s_power,
	.ioctl = ov2722_ioctl,
	.init = ov2722_init,
};

static const struct v4l2_subdev_sensor_ops ov2722_sensor_ops = {
	.g_skip_frames	= ov2722_g_skip_frames,
};

static const struct v4l2_subdev_pad_ops ov2722_pad_ops = {
	.enum_mbus_code = ov2722_enum_mbus_code,
	.enum_frame_size = ov2722_enum_frame_size,
	.get_fmt = ov2722_get_pad_format,
	.set_fmt = ov2722_set_pad_format,
};

static const struct v4l2_subdev_ops ov2722_ops = {
	.core = &ov2722_core_ops,
	.video = &ov2722_video_ops,
	.pad = &ov2722_pad_ops,
	.sensor = &ov2722_sensor_ops,
};

static const struct media_entity_operations ov2722_entity_ops = {
	.link_setup = NULL,//.set_power = v4l2_subdev_set_power,
};

static int ov2722_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2722_device *dev = to_ov2722_sensor(sd);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static int ov2722_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct ov2722_device *dev;
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : Entry %d\n",__func__,__LINE__);
	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&dev->sd, client, &ov2722_ops);
	if (client->dev.platform_data) {
		ret = ov2722_s_config(&dev->sd, client->irq,
					   client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	/*TODO add format code here*/
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &ov2722_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ov2722_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;
	lilp3_debug(LILP3_INFO,"lilp3, %s : End %d\n",__func__,__LINE__);

	return 0;
}

static const struct i2c_device_id ov2722_id[] = {
	{OV2722_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov2722_id);

static struct i2c_driver ov2722_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV2722_NAME,
	},
	.probe = ov2722_probe,
	.remove = ov2722_remove,
	.id_table = ov2722_id,
};

static __init int ov2722_init_mod(void)
{
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	return i2c_add_driver(&ov2722_driver);
}

static __exit void ov2722_exit_mod(void)
{
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	i2c_del_driver(&ov2722_driver);
}

module_init(ov2722_init_mod);
module_exit(ov2722_exit_mod);

MODULE_DESCRIPTION("A low-level driver for Omnivision OV2722 sensors");
MODULE_LICENSE("GPL");
