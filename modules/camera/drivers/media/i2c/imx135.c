/*
 * Support for Sony imx135 1080p HD camera sensor.
 *
 * Copyright (c) 2011 Intel Corporation. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/proc_fs.h>

#include "imx135.h"
#define IMX135_BIN_FACTOR_MAX	2

#define to_imx135_sensor(sd) container_of(sd, struct imx135_device, sd)

#define HOME_POS 255

/* divides a by b using half up rounding and div/0 prevention
 * (result is 0 if b == 0) */
#define divsave_rounded(a, b)	(((b) != 0) ? (((a)+((b)>>1))/(b)) : (-1))

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
};

/*
 * TODO: use debug parameter to actually define when debug messages should
 * be printed.
 */
static int debug = 2;
static u16 real_model_id;
int otp_size=0;

module_param(debug, int, 0644);
module_param_call(real_model_id, NULL, param_get_int,&real_model_id,S_IRUGO);

MODULE_PARM_DESC(debug, "Enable debug messages");
//#define IMX135_DEBUG_AF 1
//#define IMX135_DEBUG_NR 1
#ifdef IMX135_DEBUG_NR
int nr_onoff=0xFF;
int nr_type=0;
int nr_index=0;
static const struct file_operations proc_nr_operations;

struct v4l2_subdev *imx135_sd_nr=NULL;

#endif
#ifdef IMX135_DEBUG_AF
static const struct file_operations proc_af_operations;

struct v4l2_subdev *imx135_sd=NULL;
static int af_stop=0;

#endif
static int DW9719_t_focus_abs(struct v4l2_subdev *sd, s32 value);
static int imx135_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val);
static int imx135_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val);

#define lilp3_debug(level, fmt, arg...) \
  do{ \
	  if (level > debug) \
		printk( fmt , ## arg); \
	}while(0)

static struct imx135_resolution imx135_res_preview[] = {
	{
		.desc = "IMX135_PREVIEW_1336_1104_30fps",
		.width = 1040,
		.height = 784,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 10000,
		.lines_per_frame = 2040,
		.regs = imx135_1M_STILL_37fps,
	},

	{
		.desc = "IMX135_PREVIEW_1336_1104_30fps",
		.width = 1336,
		.height = 1104,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 6000,
		.lines_per_frame = 2040,
		.regs = imx135_1336x1104_30fps,
	},	
	{
		 .desc =	"MODE1920x1080" ,
		 .width =	1936	,
		 .height =	1104	,
		 .fps = 	22	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_1936x1104_30fps
	},
	{
		.desc = "IMX135_PREVIEW_2008_1208_30fps",
		.width = 2008,
		.height = 1208,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 10000,
		.lines_per_frame = 1960,
		.regs = imx135_2008x1208_30fps,
	},

	{
		 .desc = "IMX135_PREVIEW_2056x1160",
		 .width = 2056,
		 .height = 1160,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 10000,
		 .lines_per_frame = 1960,
		 .regs = imx135_2056x1160_30fps ,
	},	
	{
		 .desc = "IMX135_PREVIEW_2104x1560",
		 .width = 2104,
		 .height = 1560,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 10000,
		 .lines_per_frame = 2220,
		 .regs = imx135_2104x1560_30fps	,
	},
	
	{
		.desc = "IMX135_PREVIEW_3280x2464",
		.width = 2064,
		.height = 1552,
		.fps = 19,
		.used = 0,
		.pixels_per_line = 10000,
		.lines_per_frame = 3800,
		.regs = imx135_3M_STILL_30fps ,
	},

	{
		.desc = "IMX135_PREVIEW_3280x2464",
		.width = 2576,
		.height = 1936,
		.fps = 22,
		.used = 0,
		.pixels_per_line = 10000,
		.lines_per_frame = 3800,
		.regs = imx135_5M_STILL_19fps ,
	},
        {
                .desc = "IMX135_PREVIEW_4112x2320",
                .width = 4096,
                .height = 2304,
                .fps = 21,
                .used = 0,
                .pixels_per_line = 4572,
                .lines_per_frame = 2338,
                .regs = imx135_4112x2320_21fps ,
        },
	{
		.desc = "IMX135_PREVIEW_3280x2464",
		.width = 3280,
		.height = 2464,
		.fps = 22,
		.used = 0,
		.pixels_per_line = 10000,
		.lines_per_frame = 3800,
		.regs = imx135_8M_STILL_25fps ,
	},
	{
		.desc = "IMX135_PREVIEW_1632x1224",
		.width = 1632,
		.height = 1224,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 4572,
		.lines_per_frame = 1960,
		.regs = imx135_PREVIEW_1632x1224_30fps ,
	},
	 {
		 .desc = "STILL_13M_7fps",
		 .width = 4208,
		 .height = 3120,
		 .fps = 21,
		 .used = 0,
		 .pixels_per_line = 7168,
		 .lines_per_frame = 3168,
		 .regs = imx135_4208_3120_24fps,
	 },
};

#define N_RES_PREVIEW (ARRAY_SIZE(imx135_res_preview))

static struct imx135_resolution imx135_res_still[] = {
	{
		.desc = "IMX135_PREVIEW_1336_1104_30fps",
		.width = 1040,
		.height = 784,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 4696,
		.lines_per_frame = 1542,
		.regs = imx135_1M_STILL_37fps,
	},
	{
		.desc = "IMX135_PREVIEW_1632x1224",
		.width = 1632,
		.height = 1224,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 4572,
		.lines_per_frame = 1960,
		.regs = imx135_PREVIEW_1632x1224_30fps ,
	},
	{
		.desc = "IMX135_PREVIEW_1336_1104_30fps",
		.width = 1336,
		.height = 1104,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 10000,
		.lines_per_frame = 2040,
		.regs = imx135_1336x1104_30fps,
	},

        {
                .desc = "imx135_1080P_STILL_15fps",
                .width = 1936,
                .height = 1104,
                .fps = 15,
                .used = 0,
                .pixels_per_line = 6100,
                .lines_per_frame = 3100,
                .regs = imx135_1080P_STILL_15fps,
        },

        {
                .desc = "imx135_2M_STILL_15fps",
                .width = 1640,
                .height = 1232,
                .fps = 15,
                .used = 0,
                .pixels_per_line = 6100,
                .lines_per_frame = 3100,
                .regs = imx135_2M_STILL_15fps,
        },

	 {
		 .desc = "STILL_2008_1208_30fps",
		 .width = 2008,
		 .height = 1208,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 10000,
		 .lines_per_frame = 2220,
		 .regs = imx135_2008x1208_30fps,
	 },

	 {
		 .desc = "STILL_4016_2416_30fps",
		 .width = 4016,
		 .height = 2416,
		 .fps = 10,
		 .used = 0,
		 .pixels_per_line = 9000,
		 .lines_per_frame = 3100,
		 .regs = imx135_4016x2416_30fps,
	 },

	 {
		 .desc = "STILL_2056_1160_30fps",
		 .width = 2056,
		 .height = 1160,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 6000,
		 .lines_per_frame = 2220,
		 .regs = imx135_2056x1160_30fps,
	 },

         {
                 .desc = "STILL_3M_10fps",
                 .width = 2064,
                 .height = 1552,
                 .fps = 10,
                 .used = 0,
                 .pixels_per_line = 9000,
                 .lines_per_frame = 3100,
                 .regs = imx135_3M_STILL_10fps,
         },

	 {
		 .desc = "STILL_2104_1560_30fps",
		 .width = 2104,
		 .height = 1560,
		 .fps = 30,
		 .used = 0,
		 .pixels_per_line = 10000,
		 .lines_per_frame = 2220,
		 .regs = imx135_2104x1560_30fps,
	 },

         {
                 .desc = "IMX135_STILL_5M",
                 .width = 2576,
                 .height = 1936,
                 .fps = 10,
                 .used = 0,
                 .pixels_per_line = 10509,
                 .lines_per_frame = 3800,
                 .regs = imx135_5M_STILL_10fps,
         },

         {
                 .desc = "IMX135_STILL_2896_1744",
                 .width = 2896,
                 .height = 1744,
                 .fps = 10,
                 .used = 0,
                 .pixels_per_line = 9000,
                 .lines_per_frame = 3100,
                 .regs = imx135_2896_1744_10fps,
         },

        {
                .desc = "IMX135_STILL_6M",
                .width = 3280,
                .height = 1852,
                .fps = 10,
                .used = 0,
                .pixels_per_line = 9000,
                .lines_per_frame = 3100,
                .regs = imx135_6M_STILL_10fps ,
        },

        {
                .desc = "IMX135_STILL_3600x2032",
                .width = 3600,
                .height = 2032,
                .fps = 10,
                .used = 0,
                .pixels_per_line = 9000,
                .lines_per_frame = 3100,
                .regs = imx135_3600_2032_10fps ,
        },

        {
                .desc = "IMX135_STILL_3536x2128",
                .width = 3536,
                .height = 2128,
                .fps = 10,
                .used = 0,
                .pixels_per_line = 9000,
                .lines_per_frame = 3100,
                .regs = imx135_3536_2118_10fps,
        },

	 {
		.desc = "IMX135_PREVIEW_3280x2464",
		.width = 3280,
		.height = 2464,
		.fps = 10,
		.used = 0,
		.pixels_per_line = 10509,
		.lines_per_frame = 3800,
		.regs = imx135_8M_STILL_15fps ,
	 },

        {
                .desc = "IMX135_STILL_3600x2704",
                .width = 3600,
                .height = 2704,
                .fps = 10,
                .used = 0,
                .pixels_per_line = 9000,
                .lines_per_frame = 3100,
                .regs = imx135_3600_2704_10fps,
        },

	 {
		 .desc = "STILL_4112_2320_30fps",
		 .width = 4112,
		 .height = 2320,
		 .fps = 9,
		 .used = 0,
		 .pixels_per_line = 15000,
		 .lines_per_frame = 2914,
		 .regs = imx135_4112x2320_30fps,
	 },

	 {
		 .desc = "STILL_13M_7fps",
		 .width = 4208,
		 .height = 3120,
		 .fps = 7,
		 .used = 0,
		 .pixels_per_line = 10000,
		 .lines_per_frame = 4208,
		 .regs = imx135_4208_3120_7fps,
	 },

};

#define N_RES_STILL (ARRAY_SIZE(imx135_res_still))

static struct imx135_resolution imx135_res_video[] = {
	{
		  .desc =	 "MODE192x160" ,
		  .width =	 192	 ,
		  .height =  160	 ,
		  .fps =	 30  ,
		  .used =	 0	 ,
		  .pixels_per_line = 4572,
		  .lines_per_frame = 2040,
		  .regs =	 imx135_192x160_30fps
	},

	{
		  .desc =	 "MODE352x288" ,
		  .width =	 368	 ,
		  .height =  304	 ,
		  .fps =	 30  ,
		  .used =	 0	 ,
		  .pixels_per_line = 4572,
		  .lines_per_frame = 2040,
		  .regs =	 imx135_368x304_30fps
	},

	{
		 .desc =	"MODE656x496" ,
		 .width =	656	,
		 .height =	496	,
		 .fps = 	30	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_656x496_30fps
	},

	{
		 .desc =	"MODE736x496" ,
		 .width =	736	,
		 .height =	496	,
		 .fps = 	30	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_736x496_30fps
	},

	{
		 .desc =	"MODE896x736" ,
		 .width =	896	,
		 .height =	736	,
		 .fps = 	30	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_896x736_STILL_37fps
	},

	{
		 .desc =	"MODE976x736" ,
		 .width =	976	,
		 .height =	736	,
		 .fps = 	30	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_976x736_STILL_37fps
	},

	{
		 .desc =	"MODE1216x736" ,
		 .width =	1216	,
		 .height =	736	,
		 .fps = 	24	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_1216x736_STILL_37fps
	},

	{
		 .desc =	"MODE336x256" ,
		 .width =	336	,
		 .height =	256	,
		 .fps = 	30	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2040,
		 .regs =	imx135_336x256_30fps
	},
	{
		 .desc =	"MODE816x496" ,
		 .width =	816	,
		 .height =	496	,
		 .fps = 	30	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2040,
		 .regs =	imx135_816x496_30fps
	},
	{
		 .desc =	"MODE1296x736" ,
		 .width =	1296	,
		 .height =	736	,
		 .fps = 	25	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_1296x736_30fps
	},

	{
		.desc = "IMX135_PREVIEW_1632x1224",
		.width = 1632,
		.height = 1224,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 4572,
		.lines_per_frame = 1960,
		.regs = imx135_video_1632x1224_30fps ,
	},
	{
		 .desc =	"MODE1920x1080" ,
		 .width =	1936	,
		 .height =	1104	,
		 .fps = 	25	,
		 .used =	0	,
		 .pixels_per_line = 4572,
		 .lines_per_frame = 2334,
		 .regs =	imx135_video_1936x1104_24fps,//imx135_1936x1104_30fps
	},

};

#define N_RES_VIDEO (ARRAY_SIZE(imx135_res_video))

static struct imx135_resolution imx135_res_hdr_movie[] = {
	{
		 .desc =	"1920x1080_hdr_30fps"	,
		 .width =	1920	,
		 .height =	1080	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 5166,
		 .lines_per_frame = 2320,
		 .regs =	imx135_1920x1080_HDR_30fps	,
	},

	{
		 .desc =	"2104x1560_hdr_30fps"	,
		 .width =	2104	,
		 .height =	1560	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 10000,
		 .lines_per_frame = 2220,
		 .regs =	imx135_2104x1560_HDR_30fps	,
	},

};

#define N_RES_HDR_MOVIE (ARRAY_SIZE(imx135_res_hdr_movie))


static struct imx135_resolution *imx135_res = imx135_res_preview;
static int N_RES = N_RES_PREVIEW;
static int
imx135_read_reg_byte(struct i2c_client *client, u16 len, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u16 data[IMX135_SHORT_MAX];
	int err;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > IMX135_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
	if (len == IMX135_8BIT) {
		*val = (u8)data[0];
	} 

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
imx135_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[IMX135_SHORT_MAX];
	int err, i;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > IMX135_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
	if (len == IMX135_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
imx135_read_reg_bytes(struct i2c_client *client, u16 len, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u16 data[IMX135_SHORT_MAX];
	int err;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > IMX135_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
		//*val = (u8)data[0];
	memcpy(val,(u8 *)data, len);

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int imx135_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (ret == num_msg)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_err(&client->dev, "retrying i2c write transfer... %d\n",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret;
}

static int
imx135_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != IMX135_8BIT && data_length != IMX135_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == IMX135_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* IMX135_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu(val);
	}

	ret = imx135_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


/**
 * imx135_rmw_reg - Read/Modify/Write a value to a register in the sensor
 * device
 * @client: i2c driver client structure
 * @data_length: 8/16-bits length
 * @reg: register address
 * @mask: masked out bits
 * @set: bits set
 *
 * Read/modify/write a value to a register in the  sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int imx135_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
			   u16 mask, u16 set)
{
	int err;
	u16 val;

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	if (data_length == IMX135_8BIT && mask & ~0xff)
		return -EINVAL;

	err = imx135_read_reg(client, data_length, reg, &val);
	if (err) {
		v4l2_err(client, "imx135_rmw_reg error exit, read failed\n");
		return -EINVAL;
	}

	val &= ~mask;

	/*
	 * Perform the OR function if the @set exists.
	 * Shift @set value to target bit location. @set should set only
	 * bits included in @mask.
	 *
	 * REVISIT: This function expects @set to be non-shifted. Its shift
	 * value is then defined to be equal to mask's LSB position.
	 * How about to inform values in their right offset position and avoid
	 * this unneeded shift operation?
	 */
	set <<= ffs(mask) - 1;
	val |= set & mask;

	err = imx135_write_reg(client, data_length, reg, val);
	if (err) {
		v4l2_err(client, "imx135_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


/*
 * imx135_write_reg_array - Initializes a list of MT9M114 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __imx135_flush_reg_array, __imx135_buf_reg_array() and
 * __imx135_write_reg_is_consecutive() are internal functions to
 * imx135_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __imx135_flush_reg_array(struct i2c_client *client,
				     struct imx135_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return imx135_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __imx135_buf_reg_array(struct i2c_client *client,
				   struct imx135_write_ctrl *ctrl,
				   const struct imx135_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case IMX135_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case IMX135_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg.sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= IMX135_MAX_WRITE_BUF_SIZE)
		__imx135_flush_reg_array(client, ctrl);

	return 0;
}

static int
__imx135_write_reg_is_consecutive(struct i2c_client *client,
				   struct imx135_write_ctrl *ctrl,
				   const struct imx135_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int imx135_write_reg_array(struct i2c_client *client,
				   const struct imx135_reg *reglist)
{
	const struct imx135_reg *next = reglist;
	struct imx135_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != IMX135_TOK_TERM; next++) {
		switch (next->type & IMX135_TOK_MASK) {
		case IMX135_TOK_DELAY:
			err = __imx135_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		case IMX135_RMW:
			err = __imx135_flush_reg_array(client, &ctrl);
			err |= imx135_rmw_reg(client,
					       next->type & ~IMX135_RMW,
					       next->reg.sreg, next->val,
					       next->val2);
			if (err) {
				v4l2_err(client, "%s: rwm error, "
						"aborted\n", __func__);
				return err;
			}
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__imx135_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __imx135_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __imx135_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __imx135_flush_reg_array(client, &ctrl);
}

static int DW9719_write8(struct v4l2_subdev *sd, int reg, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9719_device *dev = to_dw9719_device(sd);
	struct i2c_msg msg;

	memset(&msg, 0 , sizeof(msg));
	msg.addr = DW9719_I2C_ADDR;
	msg.len = 2;
	msg.buf = dev->buffer;
	msg.buf[0] = reg;
	msg.buf[1] = val;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int DW9719_write16(struct v4l2_subdev *sd, int reg, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9719_device *dev = to_dw9719_device(sd);
	struct i2c_msg msg;

	memset(&msg, 0 , sizeof(msg));
	msg.addr = DW9719_I2C_ADDR;
	msg.len = 3;
	msg.buf = dev->buffer;
	msg.buf[0] = reg;
	msg.buf[1] = val >> 8;
	msg.buf[2] = val & 0xFF;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int DW9719_read8(struct v4l2_subdev *sd, int reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9719_device *dev = to_dw9719_device(sd);
	struct i2c_msg msg[2];
	int r;

	memset(msg, 0 , sizeof(msg));
	msg[0].addr = DW9719_I2C_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = dev->buffer;
	msg[0].buf[0] = reg;

	msg[1].addr = DW9719_I2C_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = dev->buffer;

	r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (r != ARRAY_SIZE(msg))
		return -EIO;

	return dev->buffer[0];
}
static int DW9719_busy_check(struct v4l2_subdev *sd)
{
	int wait_loop=50;
	char busy_status=0;
	int ret_val=0;
	while(wait_loop--)
	{
		busy_status = DW9719_read8(sd, DW9719_STATUS);
		lilp3_debug(LILP3_DEBUG,"lilp3, %s :busy_status=0x%x ,%d\n",__func__,busy_status,__LINE__);
		if(busy_status & 0x01)
		{
			usleep_range(10,20);
			continue;
		}
		else
			break;
	}
	if(wait_loop==0)
		ret_val=-1;

	return ret_val;
		
		
}
static int DW9719_init(struct v4l2_subdev *sd)
{
	struct dw9719_device *dev = to_dw9719_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	dev->platform_data = camera_get_af_platform_data();
	if (!dev->platform_data) {
		v4l2_err(client, "failed to get platform data\n");
		return -ENXIO;
	}
	return 0;
}

static int DW9719_power_up(struct v4l2_subdev *sd)
{
	/* Transition time required from shutdown to standby state */
	const int WAKEUP_DELAY_US = 200;
	//const int DEFAULT_CONTROL_VAL = 0x02;
	struct dw9719_device *dev = to_dw9719_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int r;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	/* Enable power */
	r = dev->platform_data->power_ctrl(sd, 1);
	if (r)
		return r;

	udelay(1);		/* Wait for VBAT to stabilize */

	/* jiggle SCL pin to wake up device  software reset*/
	DW9719_write8(sd, DW9719_CONTROL, 1);

	usleep_range(WAKEUP_DELAY_US, WAKEUP_DELAY_US * 10);
#if 0
	/* Reset device */
	r = DW9719_write8(sd, DW9719_CONTROL, 1);
	if (r < 0)
		goto fail_powerdown;
#else
	/* Reset device */
	r = DW9719_write8(sd, DW9719_CONTROL, 2);
	if (r < 0)
		goto fail_powerdown;	
	/* set to SAC3 mode */
	r = DW9719_write8(sd, DW9719_MODE, 0x40);
	if (r < 0)
		goto fail_powerdown;	

	/* resonance mode */
	r = DW9719_write8(sd, DW9719_VCM_RESONANCE, 0x04);
	if (r < 0)
		goto fail_powerdown;	

	
#endif
	r = DW9719_busy_check(sd);
	if(r<0)
		lilp3_debug(LILP3_INFO,"lilp3, %s :error!!! Af is busy always r = %x, %d\n",__func__,r,__LINE__);
	/* Detect device */
	r = DW9719_read8(sd, 0x00);
	lilp3_debug(LILP3_INFO,"lilp3, %s : Detect AF r = %x, %d\n",__func__,r,__LINE__);
	if (r < 0)
		goto fail_powerdown;
	if (r != 0xf1) {
		r = -ENXIO;
		goto fail_powerdown;
	}

	dev->focus = DW9719_MAX_FOCUS_POS;
	dev->initialized = true;
	
	/* vcm default position mode */
	r = DW9719_t_focus_abs(sd,dev->focus);
	if (r < 0)
		goto fail_powerdown;

	v4l2_info(client, "detected DW9719\n");
#if 0
	{
		int lsb=0;
		int msb=0;
		int i = 10;
		while(i--){
		lsb= (20 & 0x0f) << 4;
		msb=(0x03ff & 20)>>4;
		DW9719_write8(sd, 0x03, msb);
		DW9719_write8(sd, 0x04, lsb);
		msleep(50);

		lsb= ( 500 & 0x0f) << 4;
		msb=(0x03ff & 500)>>4;
		DW9719_write8(sd, 0x03, msb);
		DW9719_write8(sd, 0x04, lsb);
		lilp3_debug(LILP3_DEBUG,"lilp3, %s : i = %d\n",__func__,i);
		msleep(50);
		}
	}
#endif
	return 0;

fail_powerdown:
	dev->platform_data->power_ctrl(sd, 0);
	return r;
}

static int DW9719_power_down(struct v4l2_subdev *sd)
{

	struct dw9719_device *dev = to_dw9719_device(sd);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	return dev->platform_data->power_ctrl(sd, 0);
}

static int DW9719_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct dw9719_device *dev = to_dw9719_device(sd);
	int r;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d, val=%d\n",__func__,__LINE__, value);

	if (!dev->initialized)
		return -ENODEV;
	r = DW9719_busy_check(sd);
	if(r<0)
	{
		lilp3_debug(LILP3_INFO,"lilp3, %s :error!!! Af is busy always r = %x, %d\n",__func__,r,__LINE__);
		return r;
	}

	value = clamp(value, 0, DW9719_MAX_FOCUS_POS);
	r = DW9719_write16(sd, DW9719_VCM_CURRENT,
			   DW9719_MAX_FOCUS_POS - value);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : val = %x, r = %d, %d\n",__func__,value,r,__LINE__);
	if (r < 0)
		return r;

	getnstimeofday(&dev->focus_time);
	dev->focus = value;
	return 0;
}

static int DW9719_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{


	struct dw9719_device *dev = to_dw9719_device(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	*value = dev->focus;
	return 0;
}

static int DW9719_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct dw9719_device *dev = to_dw9719_device(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d , val=%d\n",__func__,__LINE__, value);

	return DW9719_t_focus_abs(sd, dev->focus + value);
}

static int DW9719_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	static const struct timespec move_time = {
		/* The time required for focus motor to move the lens */
		.tv_sec = 0,
		.tv_nsec = 60000000
	};
	struct dw9719_device *dev = to_dw9719_device(sd);
	struct timespec current_time, finish_time, delta_time;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	getnstimeofday(&current_time);
	finish_time = timespec_add(dev->focus_time, move_time);
	delta_time = timespec_sub(current_time, finish_time);
	if (delta_time.tv_sec >= 0 && delta_time.tv_nsec >= 0) {
		/* VCM motor is not moving */
		*value = ATOMISP_FOCUS_HP_COMPLETE |
			 ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
	} else {
		/* VCM motor is still moving */
		*value = ATOMISP_FOCUS_STATUS_MOVING |
			 ATOMISP_FOCUS_HP_IN_PROGRESS;
	}
	return 0;
}

/* Start group hold for the following register writes */
static int imx135_grouphold_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const int group = 0;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	return imx135_write_reg(client, IMX135_8BIT,
				IMX135_GROUP_ACCESS,
				group | IMX135_GROUP_ACCESS_HOLD_START);
}

/* End group hold and quick launch it */
static int imx135_grouphold_launch(struct v4l2_subdev *sd)
{

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const int group = 0;
	int ret;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/* End group */
	ret = imx135_write_reg(client, IMX135_8BIT,
			       IMX135_GROUP_ACCESS,
			       group | IMX135_GROUP_ACCESS_HOLD_END);
	if (ret)
		return ret;

	/* Delay launch group (during next vertical blanking) */
	return ret;
	//return imx135_write_reg(client, IMX135_8BIT,
	//			IMX135_GROUP_ACCESS,
	//			group | IMX135_GROUP_ACCESS_DELAY_LAUNCH);
}

/*
 * Read EEPROM data from the le24l042cs chip and store
 * it into a kmalloced buffer. On error return NULL.
 * The caller must kfree the buffer when no more needed.
 * @size: set to the size of the returned EEPROM data.
 */
 #if 0
static void *le24l042cs_read(struct i2c_client *client, int *size)
{
	static const unsigned int LE24L042CS_I2C_ADDR = 0xA0 >> 1;
	static const unsigned int LE24L042CS_EEPROM_SIZE = 1280;
	static const unsigned int MAX_READ_SIZE = IMX135_MAX_WRITE_BUF_SIZE;
	struct i2c_msg msg[2];
	int addr;
	char *buffer;

	buffer = kmalloc(LE24L042CS_EEPROM_SIZE, GFP_KERNEL);
	if (!buffer)
		return NULL;

	memset(msg, 0, sizeof(msg));
	for (addr = 0; addr < LE24L042CS_EEPROM_SIZE; addr += MAX_READ_SIZE) {
		unsigned int i2c_addr = LE24L042CS_I2C_ADDR;
		unsigned char addr_buf;
		int r;

		i2c_addr |= (addr >> 8) & 1;
		addr_buf = addr & 0xFF;

		msg[0].addr = i2c_addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &addr_buf;

		msg[1].addr = i2c_addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = min(MAX_READ_SIZE, LE24L042CS_EEPROM_SIZE - addr);
		msg[1].buf = &buffer[addr];

		r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (r != ARRAY_SIZE(msg)) {
			kfree(buffer);
			dev_err(&client->dev, "read failed at 0x%03x\n", addr);
			return NULL;
		}
	}

	if (size)
		*size = LE24L042CS_EEPROM_SIZE;
	return buffer;
}
 #endif
#ifdef IMX135_DEBUG_AF
int imx135_strlen(char *string)
{
	int i=0;
	char *p=string;
	while(*p++)
		i++;
	return i;

}

int imx135_strtoint(char *string)
{
	int ret_val=0;
	int i=0;
	int strlen=imx135_strlen(string);
	char *p=string;
	printk("=====================imx135_strtoint strlen=%d\n", strlen);
	for(i=0;i<strlen;i++)
	{
	printk("p[%d]=%d ,", i, *(p+i));
		if(*(p+i)<'0' || *(p+i)>'9')
			return ret_val;
		ret_val=ret_val*10+(*(p+i)-'0');
		
		if(i>64)
		{
			printk("the string is too long %s\n",__func__);
			break;
		}
	}
	printk("\n ret_val=%d\n", ret_val);
	return ret_val;

}

static ssize_t proc_af_write (struct file *file, const char __user *buf, size_t nbytes, loff_t *ppos)
{
	char string[64];
	int cur_val=0;
	int i=0;
	copy_from_user(string, buf,10);
	printk("=====================>af val=%s\n", string);
	nbytes=imx135_strlen(string);
	cur_val=imx135_strtoint(string);
	printk("=====================>cur_val=%d\n", cur_val);
	if(cur_val==0)
		af_stop=0;
	else if(cur_val<=1023 && cur_val>0)
		af_stop=1;
	printk("=========================>%s,af_stop=%d\n",__func__, af_stop);
	if(cur_val>1023 || cur_val<=0)
		return nbytes;
	DW9719_write16(imx135_sd, DW9719_VCM_CURRENT,
			   cur_val);
	i = DW9719_read8(imx135_sd, 0x03);
	lilp3_debug(LILP3_INFO,"lilp3, %s : af msb = %x, %d\n",__func__,i,__LINE__);

	i = DW9719_read8(imx135_sd, 0x04);
	lilp3_debug(LILP3_INFO,"lilp3, %s : af lsb = %x, %d\n",__func__,i,__LINE__);
	
	return nbytes;
}


static const struct file_operations proc_af_operations = {
	.owner	= THIS_MODULE,
	.write	= proc_af_write,
};
#endif
#ifdef IMX135_DEBUG_NR
static ssize_t proc_nr_write (struct file *file, const char __user *buf, size_t nbytes, loff_t *ppos)
{
	char string[64];
	int cur_val=0;
	int i=0;
	copy_from_user(string, buf,10);
	printk("=====================>af val=%s\n", string);
	nbytes=imx135_strlen(string);
	cur_val=imx135_strtoint(string);
	
	printk("=====================>cur_val=%d\n", cur_val);

	switch(cur_val)
	{
		case 00:
			nr_onoff=0xFF;
			nr_type=00;
			nr_index=0;
			break;
		case 01:
			nr_onoff=0xF9;
			nr_type=01;	
			nr_index=1;
			break;
		case 02:
			nr_onoff=0xF9;
			nr_type=02;	
			nr_index=2;
			

			break;
		case 03:
			nr_onoff=0xF9;
			nr_type=03;	
			nr_index=3;

			break;
		case 10:
			nr_onoff=0xF3;
			nr_type=10;	
			nr_index=4;

			break;
		case 11:
			nr_onoff=0xF1;
			nr_type=11;
			nr_index=5;


			break;
		case 12:
			nr_onoff=0xF1;
			nr_type=12;
			nr_index=6;

			break;
		case 13:
			nr_onoff=0xF1;
			nr_type=13;
			nr_index=7;


			break;
		case 20:
			nr_onoff=0xF3;
			nr_type=20;
			nr_index=8;

			break;
		case 21:
			nr_onoff=0xF1;
			nr_type=21;
			nr_index=9;

			break;
		case 22:
			nr_onoff=0xF1;
			nr_type=22;
			nr_index=10;

			break;
		case 23:
			nr_onoff=0xF1;
			nr_type=23;
			nr_index=11;

			break;
		case 30:
			nr_onoff=0xF3;
			nr_type=30;
			nr_index=12;

			break;
		case 31:
			nr_onoff=0xF1;
			nr_type=30;
			nr_index=13;

			break;
		case 32:
			nr_onoff=0xF1;
			nr_type=32;
			nr_index=14;

			break;
		case 33:
			nr_onoff=0xF1;
			nr_type=33;
			nr_index=15;
			break;
		default:
			nr_onoff=0xFF;
			nr_type=00;
			nr_index=0;

			break;

	}
	printk(" %s : %d nr_onoff=%d, nr_type=%d,nr_index=%d\n"
		,__func__,__LINE__,
		nr_onoff,nr_type,nr_index);

	return nbytes;
}

static const struct file_operations proc_nr_operations = {
	.owner	= THIS_MODULE,
	.write	= proc_nr_write,
};
#endif
static int imx135_read_otp_reg_array(struct i2c_client *client, u16 size, u16 addr,
				  void *data)
{
	u8 *buf = data;
	u16 index;
	int ret = 0;
	u16 one_read_size=IMX135_OTP_READ_ONETIME;

	for (index = 0; index + one_read_size <= size;
	     index += one_read_size) {
		ret = imx135_read_reg_bytes(client, one_read_size, addr + index,
				       (u8 *)&buf[index]);
		if (ret)
			return ret;
	}

	if (size - index > 0)
		{
		lilp3_debug(LILP3_INFO,"=================size=%d, index=%d\n", size, index);
		ret = imx135_read_reg_bytes(client, size - index, addr + index,
				       (u8 *)&buf[index]);
		}

	return ret;
}

static unsigned long
imx135_otp_sum(struct v4l2_subdev *sd, u8 *buf, u16 start, u16 end)
{
	unsigned long sum = 0;
	u16 i;

	for (i = start; i <= end; i++)
		sum += buf[i];

	return sum;
}

static int imx135_otp_checksum(struct v4l2_subdev *sd, u8 *buf)
{
	unsigned long sum;
	u16 checksum;
	int i;
	int zero_flag = 1;
	u16 checksum_reg=0;
	u16 checksum_low=0;
	u16 checksum_high=0;
	u8 major_version=0;
	u8 minor_version =0;
	int list_len=0;
	int intel_otp_end=0;
	struct imx135_otp_checksum_format *list;
	
	major_version=(u8)*(buf+IMX135_OTP_VERSION_ADDR);
	minor_version=(u8)*(buf+IMX135_OTP_VERSION_ADDR+1);
	printk("===================>%s major_version=%d, minor_version=%d\n", __func__, major_version, minor_version);
	if(major_version >= IMX135_OTP_MAJOR_VERSION_PVT
		&& minor_version >= IMX135_OTP_MINOR_VERSION_PVT)
	{
		checksum_high=(u16)*(buf+IMX135_OTP_CHECKSUM_ADDR_PVT);
		checksum_low=(u16)*(buf+IMX135_OTP_CHECKSUM_ADDR_PVT+1);
		list=imx135_otp_checksum_list_pvt;
		list_len=ARRAY_SIZE(imx135_otp_checksum_list_pvt);
		intel_otp_end=IMX135_OTP_INTEL_END_PVT;

	}
	else
	{
		checksum_high=(u16)*(buf+IMX135_OTP_CHECKSUM_ADDR_DVT);
		checksum_low=(u16)*(buf+IMX135_OTP_CHECKSUM_ADDR_DVT+1);
		list=imx135_otp_checksum_list_dvt;
		list_len=ARRAY_SIZE(imx135_otp_checksum_list_dvt);
		intel_otp_end=IMX135_OTP_INTEL_END_DVT;
	
	}
	otp_size=intel_otp_end-IMX135_OTP_INTEL_START+1;

	checksum_reg=(checksum_low & 0x00ff)|(((checksum_high & 0x00ff)<<8)&0xff00);
	printk("===============>%s checksum_low=0x%x, checksum_high=0x%x, checksum_reg=0x%x\n",
		__func__,checksum_low, checksum_high,checksum_reg);
	for (i = 0; i < list_len; i++) {
		sum = imx135_otp_sum(sd, buf, list[i].start, list[i].end);
		checksum = sum % IMX135_OTP_MOD_CHECKSUM;
	lilp3_debug(LILP3_INFO,"===============>sum=%ld, checksum=0x%x\n", sum, checksum);
		if (checksum_reg != checksum)
			return -EINVAL;
		/*
		 * Checksum must fail if whole data is 0.
		 * Clear zero_flag if data != 0 is found.
		 */
		if (unlikely(zero_flag && (sum > 0)))
			zero_flag = 0;
	}

	return !zero_flag ? 0 : -EINVAL;
}
static int imx135_HDR_set_atr_onoff(struct v4l2_subdev *sd, u8 onoff)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret=0;
	lilp3_debug(LILP3_DEBUG,"imx135 %s\n", __func__);


	if(onoff==0)
	{
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ATR_OFF1,0x00);
		if (ret) {
			v4l2_err(client, "%s: failed to write atr off1\n",
				 __func__);
			return ret;
		}

		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ATR_OFF2,0x00);
		if (ret) {
			v4l2_err(client, "%s: failed to write atr off2\n",
				 __func__);
			return ret;
		}

		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ATR_TRIGGER,0x01);
		if (ret) {
			v4l2_err(client, "%s: failed to write atr trigger off\n",
				 __func__);
			return ret;
		}

	}
	else if (onoff==1)
	{

		

		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ATR_TRIGGER,0x00);
		if (ret) {
			v4l2_err(client, "%s: failed to write atr trigger on\n",
				 __func__);
			return ret;
		}

	}	
	return 0;
}
static int imx135_HDR_set_curve(struct v4l2_subdev *sd, struct HDR_curve_parm curve_parm)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret=0;

	lilp3_debug(LILP3_DEBUG,"imx135 %s\n", __func__);

	if(curve_parm.Curve_mode==2) //atr mode
	{
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_MODE_SETTING, 2);
		if (ret) {
			v4l2_err(client, "%s: failed to write tc mode setting :atr on\n",
				 __func__);
			return ret;
		}
	    //set the gamma points
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_NOISE, (u8)((curve_parm.tc_out_noise >>8)& 0x3f));
		if (ret) {
			v4l2_err(client, "%s: failed to write tc noise 1 \n",
				 __func__);
			return ret;
		}	
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_NOISE+1, (u8)(curve_parm.tc_out_noise & 0x00ff));
		if (ret) {
			v4l2_err(client, "%s: failed to write tc noise 2 \n",
				 __func__);
			return ret;
		}	
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_MID, (u8)((curve_parm.tc_out_mid >>8)& 0x3f));
		if (ret) {
			v4l2_err(client, "%s: failed to write tc mid 1 \n",
				 __func__);
			return ret;
		}	
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_MID+1, (u8)(curve_parm.tc_out_mid& 0x00ff));
		if (ret) {
			v4l2_err(client, "%s: failed to write tc mid 2\n",
				 __func__);
			return ret;
		}
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_SAT, (u8)((curve_parm.tc_out_sat >>8)& 0x3f));
		if (ret) {
			v4l2_err(client, "%s: failed to write tc sat 1 \n",
				 __func__);
			return ret;
		}	
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_SAT+1, (u8)(curve_parm.tc_out_sat & 0x00ff));
		if (ret) {
			v4l2_err(client, "%s: failed to write tc sat 2\n",
				 __func__);
			return ret;
		}

	}
	else if(curve_parm.Curve_mode==1) //fix mode
	{
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_TC_MODE_SETTING, 1);
		if (ret) {
			v4l2_err(client, "%s: failed to write tc mode setting :ifx\n",
				 __func__);
			return ret;
		}
	}	
	return 0;
}

static int imx135_HDR_set_wb_coef(struct v4l2_subdev *sd, 
										struct WB_coefficient WBC_parm)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 gain_u=0;
	u8 gain_l=0;
	int ret=0;
	lilp3_debug(LILP3_DEBUG,"imx135 %s\n", __func__);

	gain_u=(u8)((WBC_parm.abs_gain_r>>8) & 0x00ff);
	gain_l=(u8)(WBC_parm.abs_gain_r & 0x00ff);
	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ABS_GAIN_R,gain_u);
	if (ret) {
		v4l2_err(client, "%s: failed to write gain u\n",
			 __func__);
		return ret;
	}

	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ABS_GAIN_R+1,gain_l);
		if (ret) {
		v4l2_err(client, "%s: failed to write gain l\n",
			 __func__);
		return ret;
	}
	gain_u=(u8)((WBC_parm.abs_gain_b>>8) & 0x00ff);
	gain_l=(u8)(WBC_parm.abs_gain_b & 0x00ff);
	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ABS_GAIN_B,gain_u);
		if (ret) {
		v4l2_err(client, "%s: failed to write gain b u\n",
			 __func__);
		return ret;
	}
	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ABS_GAIN_B+1,gain_l);
	if (ret) {
		v4l2_err(client, "%s: failed to write gain b l\n",
			 __func__);
		return ret;
	}


	
	return 0;
}
static int imx135_HDR_set_mode(struct v4l2_subdev *sd, struct HDR_mode_setting hdr_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret=0;
	u16 bl=64;
	u16 wb_lmt=0;
	lilp3_debug(LILP3_DEBUG,"imx135 %s\n", __func__);
	if(hdr_mode.HDR_mode==0) //auto mode
	{
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_WD_DIRECT_MODE,0x00);
		if (ret) {
			v4l2_err(client, "%s: failed to write wd direct mode\n",
				 __func__);
			return ret;
		}

		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_WD_INTEG_RATIO,hdr_mode.exp_ratio>>1);
		if (ret) {
			v4l2_err(client, "%s: failed to write exposure ratio\n",
				 __func__);
			return ret;
		}
		wb_lmt=(1023-bl)/hdr_mode.exp_ratio;
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_WB_LMT,(u8)(wb_lmt>>8));
		if (ret) {
			v4l2_err(client, "%s: failed to write wb lmt high\n",
				 __func__);
			return ret;
		}		
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_WB_LMT+1,(u8)(wb_lmt & 0x00ff));
		if (ret) {
			v4l2_err(client, "%s: failed to write wb lmt low\n",
				 __func__);
			return ret;
		}	
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_AE_SAT,(u8)(wb_lmt>>8));
		if (ret) {
			v4l2_err(client, "%s: failed to write wb lmt high\n",
				 __func__);
			return ret;
		}		
		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_AE_SAT+1,(u8)(wb_lmt & 0x00ff));
		if (ret) {
			v4l2_err(client, "%s: failed to write wb lmt low\n",
				 __func__);
			return ret;
		}	

		
	}
	else if(hdr_mode.HDR_mode==1)
	{

		ret=imx135_write_reg(client, IMX135_8BIT,IMX135_ATR_TRIGGER,0x00);
		if (ret) {
			v4l2_err(client, "%s: failed to write atr trigger on\n",
				 __func__);
			return ret;
		}

	}	
	return 0;


}

static int imx135_HDR_set_lsc(struct v4l2_subdev *sd, 
										u8 *hdr_lsc_val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int lsc_size=0;
	int i=0;
	int ret=0;
	lilp3_debug(LILP3_DEBUG,"imx135 %s\n", __func__);
	lsc_size=IMX135_HDR_LSC_END-IMX135_HDR_LSC_START+1;
	for(i=0;i<IMX135_HDR_LSC_END-IMX135_HDR_LSC_START+1;i++);
		imx135_write_reg(client, IMX135_8BIT,IMX135_HDR_LSC_START+i,*(hdr_lsc_val+i));

	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_EN_LSC,0x1F);
	if (ret) {
		v4l2_err(client, "%s: failed to write en lsc\n",
			 __func__);
		return ret;
	}
	
	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_LSC_ENABLE,0x01);
	if (ret) {
		v4l2_err(client, "%s: failed to write lsc enable\n",
			 __func__);
		return ret;
	}

	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_RAM_SEL_TOGGLE,0x01);
	if (ret) {
		v4l2_err(client, "%s: failed to write ram sel toggle\n",
			 __func__);
		return ret;
	}

	return 0;
}

static int imx135_HDR_stats_len_set(struct v4l2_subdev *sd, 
										u8 stats_len_set)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret=0;
	lilp3_debug(LILP3_DEBUG,"imx135 %s\n", __func__);
	switch(stats_len_set)
	{
		case 0:
			stats_len = 16;
			break;
		case 1:
			stats_len = 8;
			break;
		case 2:
			stats_len = 4;
			break;
		case 3:
			stats_len = 1;
			break;
		default:
			break;
	}

	
	ret=imx135_write_reg(client, IMX135_8BIT,IMX135_STATS_MODE,stats_len_set);
	if (ret) {
		v4l2_err(client, "%s: failed to write stats len\n",
			 __func__);
		return ret;
	}
	
	

	return 0;
}

static int imx135_HDR_parm_setting(struct v4l2_subdev *sd,
										struct atomisp_HDR_parm *hdr)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	//u8 __user *from = hdr->LSC_param;
	u8 *hdr_lsc_parm=NULL;
	
	lilp3_debug(LILP3_DEBUG,"imx135 %s hdr_setting_type=0x%x\n", __func__,hdr->HDR_setting_type );
	if(hdr->HDR_setting_type & ATR_ONOFF_SETTING)
	{
		imx135_HDR_set_atr_onoff(sd, hdr->ATR_onoff);
	}
	if(hdr->HDR_setting_type & LSC_SETTING)
	{
		//hdr_lsc_parm=kmalloc(IMX135_HDR_LSC_END-IMX135_HDR_LSC_START+1, GFP_KERNEL);
		//copy_from_user(hdr_lsc_parm, from, IMX135_HDR_LSC_END-IMX135_HDR_LSC_START+1);
		if(hdr_lsc_parm)
			imx135_HDR_set_lsc(sd,hdr_lsc_parm);
		
	}
	if(hdr->HDR_setting_type & WBC_SETTING)
	{
		imx135_HDR_set_wb_coef(sd,hdr->WBC_parm);
	}
	if(hdr->HDR_setting_type & HDR_MODE_SETTING)
	{
		imx135_HDR_set_mode(sd, hdr->HDR_mode);
	}
	if(hdr->HDR_setting_type & CURVE_MODE_SETTING)
	{
		imx135_HDR_set_curve(sd,hdr->Curve_parm);
	}
	if(hdr->HDR_setting_type & STATS_DATA_LEN)
	{
		imx135_HDR_stats_len_set(sd,hdr->stats_len_set);
	}
	return 0;
	


}

static int imx135_HDR_stats_read(struct v4l2_subdev *sd,
										struct atomisp_HDR_parm *hdr_parm)


{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	//u16 __user *to = priv->data;
	int i=0;
	//int temp_val=0;
	u8 u_value=0;
	u8 l_value=0;
	int j=0;
	//printk("imx135 %s\n", __func__);
#if 0	//only 4*4
		HDR_stats_y=(u16)kmalloc(stats_len*stats_len*2, GFP_KERNEL);

		for(i=0;i<stats_len*stats_len;i++)
		{
	
			   u_value=0;
			   l_value=0;
			   *(HDR_stats_y+i)=0;
			   ret = imx135_read_reg_byte(client, IMX135_8BIT,
					          HDR_reg[i], &u_value);

				if (ret) {
					v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
						__func__,HDR_reg[i] );
					return ret;
				}
			   
			   ret = imx135_read_reg_byte(client, IMX135_8BIT,
					           HDR_reg[i]+1, &l_value);

				if (ret) {
					v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
						__func__,HDR_reg[i]+1 );
					return ret;
				}	
				

			   
				printk("{u_value[%d]=0x%x,l_value[%d]=0x%x,",i,u_value,i, l_value );
				
				*(HDR_stats_y+i) |= l_value;
				*(HDR_stats_y+i) |= (u16)u_value<<8;
				printk("y[%d]=0x%x} \n",i, *(HDR_stats_y+i));
				
		}
		ret = copy_to_user(to, HDR_stats_y, stats_len*stats_len*2);
		priv->size=stats_len*stats_len*2;
		if (ret) {
			v4l2_err(client, "%s: failed to copy HDR stats data to user\n",
				 __func__);
			kfree(HDR_stats_y);
			return -EFAULT;
		}
		kfree(HDR_stats_y);
#else
		//HDR_stats_y=kmalloc(stats_len*stats_len*2, GFP_KERNEL);
		memset(HDR_stats_y, 0, sizeof(HDR_stats_y));

		for(i=0;i<stats_len;i++)
			for(j=0;j<stats_len;j++)
			{

				   u_value=0;
				   l_value=0;
				   //*(HDR_stats_y+i*stats_len+j)=0;
				   ret = imx135_read_reg_byte(client, IMX135_8BIT,
								  0x5000+i*0x80+j*8, &u_value);

					if (ret) {
						v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
							__func__,0x5000+i*0x80+j*8 );
						//kfree(HDR_stats_y);
						return ret;
					}
				   
				   ret = imx135_read_reg_byte(client, IMX135_8BIT,
								   0x5000+i*0x80+j*8+1, &l_value);

					if (ret) {
						v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
							__func__,0x5000+i*0x80+j*8+1 );
						//kfree(HDR_stats_y);
						return ret;
					}	
					

				   
					//printk("{u_value=0x%x,l_value=0x%x,",u_value, l_value );
					HDR_stats_y[i*stats_len+j] |= l_value;
					HDR_stats_y[i*stats_len+j] |= (u16)u_value<<8;					

					//printk("y[%d]=0x%x} \n",i*stats_len+j, HDR_stats_y[i*stats_len+j]);
					
			}
		//ret = copy_to_user(to, HDR_stats_y, stats_len*stats_len*2);
		//priv->size=stats_len*stats_len*2;
		hdr_parm->sensor_stats = &HDR_stats_y[0];
		hdr_parm->stats_len_set = stats_len;
		
		if (ret) {
			v4l2_err(client, "%s: failed to copy HDR stats data to user\n",
				 __func__);
			//kfree(HDR_stats_y);
			return -EFAULT;
		}
		//kfree(HDR_stats_y);


#endif

	
		return 0;


}
#if 0
static int imx135_HDR_stats_read_test(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int i=0;
	int temp_val=0;
	u8 u_value=0;
	u8 l_value=0;
	int j=0;
	void *p;
	printk("===============================>imx135 %s\n", __func__);
#if 0	//only 4*4
		HDR_stats_y=(u16)kmalloc(stats_len*stats_len*2, GFP_KERNEL);

		for(i=0;i<stats_len*stats_len;i++)
		{
	
			   u_value=0;
			   l_value=0;
			   *(HDR_stats_y+i)=0;
			   ret = imx135_read_reg_byte(client, IMX135_8BIT,
					          HDR_reg[i], &u_value);

				if (ret) {
					v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
						__func__,HDR_reg[i] );
					return ret;
				}
			   
			   ret = imx135_read_reg_byte(client, IMX135_8BIT,
					           HDR_reg[i]+1, &l_value);

				if (ret) {
					v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
						__func__,HDR_reg[i]+1 );
					return ret;
				}	
				

			   
				printk("{u_value[%d]=0x%x,l_value[%d]=0x%x,",i,u_value,i, l_value );
				
				*(HDR_stats_y+i) |= l_value;
				*(HDR_stats_y+i) |= (u16)u_value<<8;
				printk("y[%d]=0x%x} \n",i, *(HDR_stats_y+i));
				
		}
		ret = copy_to_user(to, HDR_stats_y, stats_len*stats_len*2);
		priv->size=stats_len*stats_len*2;
		if (ret) {
			v4l2_err(client, "%s: failed to copy HDR stats data to user\n",
				 __func__);
			kfree(HDR_stats_y);
			return -EFAULT;
		}
		kfree(HDR_stats_y);
#else
		//HDR_stats_y=kmalloc(stats_len*stats_len*2, GFP_KERNEL);
		memset(HDR_stats_y, 0, sizeof(HDR_stats_y));

		for(i=0;i<stats_len;i++)
			for(j=0;j<stats_len;j++)
			{

				   u_value=0;
				   l_value=0;
				   //*(HDR_stats_y+i*stats_len+j)=0;
				   ret = imx135_read_reg_byte(client, IMX135_8BIT,
								  0x5000+i*0x80+j*8, &u_value);

					if (ret) {
						v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
							__func__,0x5000+i*0x80+j*8 );
						//kfree(HDR_stats_y);
						return ret;
					}
				   
				   ret = imx135_read_reg_byte(client, IMX135_8BIT,
								   0x5000+i*0x80+j*8+1, &l_value);

					if (ret) {
						v4l2_err(client, "%s: failed to read HDR stats data, reg=0x%x\n", 
							__func__,0x5000+i*0x80+j*8+1 );
						//kfree(HDR_stats_y);
						return ret;
					}	
					

				   
					printk("{u_value=0x%x,l_value=0x%x,",u_value, l_value );
					HDR_stats_y[i*stats_len+j] |= l_value;
					HDR_stats_y[i*stats_len+j] |= (u16)u_value<<8;					
					//*(HDR_stats_y+i*stats_len+j) |= l_value;
					//*(HDR_stats_y+i*stats_len+j) |= (u16)u_value<<8;
					printk("y[%d]=0x%x} \n",i*stats_len+j, HDR_stats_y[i*stats_len+j]);
					
			}


#endif
	
		return 0;


}
#endif
static int
__imx135_otp_read(struct v4l2_subdev *sd, void *buf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int retry = 100;
	u16 ready;
	int i=0;
	//int temp_val=0;
	for(i=0;i<IMX135_OTP_PAGE_MAX;i++)
	{

		/*set page NO.*/
		ret = imx135_write_reg(client, IMX135_8BIT,
			       IMX135_OTP_PAGE_REG, i & 0xFF);
		if (ret) {
			v4l2_err(client, "%s: failed to prepare OTP page %d\n",
				 __func__,i);
			return ret;
		}

		/*set read mode*/
		ret = imx135_write_reg(client, IMX135_8BIT,
			       IMX135_OTP_MODE_REG, IMX135_OTP_MODE_READ & 0xFF);
		if (ret) {
			v4l2_err(client, "%s: failed to set OTP reading mode page=%d\n",
				 __func__,i);
			return ret;
		}

		do {
			ret = imx135_read_reg(client, IMX135_8BIT,
					       IMX135_OTP_READY_REG, &ready);
			if (ret) {
				v4l2_err(client, "%s: failed to read OTP memory "
						 "status\n", __func__);
				return ret;
			}
			if (ready & IMX135_OTP_READY_REG_OK)
				break;
			msleep(1);
		} while (--retry);

		if (!retry) {
			v4l2_err(client, "%s: OTP memory read timeout.\n", __func__);
			return -ETIMEDOUT;
		}

		if (!(ready & IMX135_OTP_READY_REG_OK)) {
			v4l2_err(client, "%s: OTP memory was initialized with error\n",
				  __func__);
			return -EIO;
		}
		ret = imx135_read_otp_reg_array(client, IMX135_OTP_PAGE_SIZE,
					     IMX135_OTP_START_ADDR, buf+i*IMX135_OTP_PAGE_SIZE);
		if (ret) {
			v4l2_err(client, "%s: failed to read OTP data\n", __func__);
			return ret;
		}

	}
#if 0 //debug the otp data

	lilp3_debug(LILP3_DEBUG,"\n");

	for(i=0;i<1280;i++)
	{
		temp_val=*(u8 *)(buf+i);
		if(temp_val<0x10)
		{
			lilp3_debug(LILP3_DEBUG,"0x0%X, ", temp_val);
		}
		else
		{
			lilp3_debug(LILP3_DEBUG,"0x%2X, ", temp_val );
		}

		if(i!=0 && (i+1)%16==0)
		{
			lilp3_debug(LILP3_DEBUG,"\n");
		}
	}
#endif

	if (IMX135_OTP_CHECKSUM)
	{
		ret = imx135_otp_checksum(sd, buf);
		if (ret)
		{
			v4l2_err(client, "%s: OTP checksum failed\n", __func__);
			return ret;
		}
		printk("============> %s OTP checksum successfull\n", __func__);
	}
	return 0;
}

static void *imx135_otp_read(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	void *buf;
	int ret;

	buf = kmalloc(IMX135_OTP_DATA_SIZE, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	/*
	 * Try all banks in reverse order and return after first success.
	 * Last used bank has most up-to-date data.
	 */
	ret = __imx135_otp_read(sd,buf);

	/* Driver has failed to find valid data */
	if (ret) {
		v4l2_err(client, "%s: sensor found no valid OTP data\n",
			  __func__);
		kfree(buf);
		return ERR_PTR(ret);
	}

	return buf;
}


static int imx135_g_priv_int_data(struct v4l2_subdev *sd,
				   struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx135_device *dev = to_imx135_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;
	/*
	u8 major_version=0;
	u8 minor_version =0;
	int intel_otp_end=0;
	
	major_version=(u8)*((u8 *)dev->otp_data+IMX135_OTP_VERSION_ADDR);
	minor_version=(u8)*((u8 *)dev->otp_data+IMX135_OTP_VERSION_ADDR+1);
	if(major_version == IMX135_OTP_MAJOR_VERSION_PVT 
		&& minor_version==IMX135_OTP_MINOR_VERSION_PVT)
	{
		intel_otp_end=IMX135_OTP_INTEL_END_PVT;

	}
	else
	{
		intel_otp_end=IMX135_OTP_INTEL_END_DVT;
	}
*/
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : read_size = %x ,%d\n",__func__,read_size,__LINE__);
	/* No OTP data available on sensor */
	if (!dev->otp_data)
		return -EIO;

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	/* Correct read_size value only if bigger than maximum */
	if (read_size > otp_size)
		read_size = otp_size;
	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		v4l2_err(client, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}

out:
	/* Return correct size */
	priv->size = otp_size;//IMX135_OTP_DATA_SIZE ;

	return 0;
}

static int imx135_set_exposure(struct v4l2_subdev *sd, int exposure, int gain, int digitgain)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx135_device *dev = to_imx135_sensor(sd);
	int ret;

	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d, exp=%d,gain=%d,digitgain=%d\n",__func__,__LINE__, exposure, gain, digitgain);

	ret = imx135_grouphold_start(sd);
	if (ret)
		goto out;

	/* set exposure time */
	ret = imx135_write_reg(client, IMX135_8BIT,
				IMX135_COARSE_INTG_TIME, (exposure >> 8) & 0xFF);
	if (ret)
		goto out;

	ret = imx135_write_reg(client, IMX135_8BIT,
				IMX135_COARSE_INTG_TIME+1, exposure & 0xFF);
	if (ret)
		goto out;

	/* set analog gain */
	ret = imx135_write_reg(client, IMX135_8BIT,
				IMX135_AGC_ADJ, gain);
	if (ret)
		goto out;

	/* set short analog gain for HDR movie mode and there is not enfluence on normal mode */
	ret = imx135_write_reg(client, IMX135_8BIT,
				IMX135_SHORT_AGC_GAIN, gain);
	if (ret)
		goto out;

	

	/* set digital gain for channel 0*/
	ret = imx135_write_reg(client, IMX135_8BIT,
				IMX135_DGC_ADJ, (digitgain >> 8) & 0xFF);
	if (ret)
		goto out;

	ret = imx135_write_reg(client, IMX135_8BIT,
				IMX135_DGC_ADJ+1, digitgain & 0xFF);
	if (ret)
		goto out;

	/* set digital gain for channel 1*/
	ret = imx135_write_reg(client, IMX135_8BIT,
		           IMX135_DGC_ADJ+2, (digitgain >> 8) & 0xFF);
	if (ret)
		goto out;

	ret = imx135_write_reg(client, IMX135_8BIT,
		           IMX135_DGC_ADJ+3, digitgain & 0xFF);
	if (ret)
		goto out;

	/* set digital gain for channel 2*/
	ret = imx135_write_reg(client, IMX135_8BIT,
		           IMX135_DGC_ADJ+4, (digitgain >> 8) & 0xFF);
	if (ret)
		goto out;

	ret = imx135_write_reg(client, IMX135_8BIT,
		           IMX135_DGC_ADJ+5, digitgain & 0xFF);
	if (ret)
		goto out;

	/* set digital gain for channel 3*/
	ret = imx135_write_reg(client, IMX135_8BIT,
		           IMX135_DGC_ADJ+6, (digitgain >> 8) & 0xFF);
	if (ret)
		goto out;

	ret = imx135_write_reg(client, IMX135_8BIT,
		           IMX135_DGC_ADJ+7, digitgain & 0xFF);
 	if (ret)
 		goto out;

	ret = imx135_grouphold_launch(sd);
	if (ret)
		goto out;

	dev->gain     = gain;
	dev->digitgain = digitgain;
	dev->exposure = exposure;
out:
	return ret;
}

static int imx135_s_exposure(struct v4l2_subdev *sd,
			      struct atomisp_exposure *exposure)
{


	int exp = exposure->integration_time[0];
	int gain = exposure->gain[0];
	int digitgain = exposure->gain[1];
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	return imx135_set_exposure(sd, exp, gain, digitgain);
}

static long imx135_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return imx135_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return imx135_g_priv_int_data(sd, arg);
	case ATOMISP_IOC_G_SENSOR_HDR_STATS_DATA:
		return imx135_HDR_stats_read(sd, arg);
	case ATOMISP_IOC_S_ISP_HDR_SETTING:
		return imx135_HDR_parm_setting(sd,arg);
	default:
		return -EINVAL;
	}
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);
	return 0;
}

static int imx135_init_registers(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	//lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	lilp3_debug(LILP3_INFO,"lilp3,begin %s client->addr = %x, %d\n",__func__,client->addr,__LINE__);
	ret = imx135_write_reg_array(client, imx135_SwReset);
	//lilp3_debug(LILP3_DEBUG,"after imx135_SwReset \n");
	//ret |= imx135_write_reg_array(client, imx135_PLL192MHz);
	//lilp3_debug(LILP3_DEBUG,"after imx135_PLL192MHz \n");
	ret |= imx135_write_reg_array(client, imx135_BasicSettings);
	//lilp3_debug(LILP3_DEBUG,"after imx135_BasicSettings \n");
	//ret |= imx135_write_reg_array(client, imx135_MIPI_Settings_684Mbps);
	//lilp3_debug(LILP3_DEBUG,"after imx135_MIPI_Settings_684Mbps \n");
	ret |= imx135_write_reg_array(client, imx135_PREVIEW_848x616_30fps);
	//lilp3_debug(LILP3_DEBUG,"after imx135_PREVIEW_848x616_30fps \n");

	return ret;
}

static int __imx135_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	/* set inital registers */
	ret = imx135_init_registers(sd);

	/* restore settings */
	imx135_res = imx135_res_preview;
	N_RES = N_RES_PREVIEW;

	return ret;
}


static int imx135_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/* set inital registers */
	ret = __imx135_init(sd, val);
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return ret;
}

static void imx135_uninit(struct v4l2_subdev *sd)
{

	struct imx135_device *dev = NULL;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	dev=to_imx135_sensor(sd);

	dev->exposure = 0;
	dev->gain     = 0;
	dev->digitgain= 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx135_device *dev = to_imx135_sensor(sd);
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	lilp3_debug(LILP3_INFO,"lilp3, %s : gpio_ctrl, ret=%d, %d\n",__func__,ret,__LINE__);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* Enable power */
#ifdef IMX135_DEBUG_NR
	nr_index=0;
	nr_type =0;
	nr_onoff=0xFF;
#endif
	ret = dev->platform_data->power_ctrl(sd, 1);
	lilp3_debug(LILP3_INFO,"lilp3, %s : power_ctrl, ret=%d, %d\n",__func__,ret,__LINE__);
	if (ret)
		goto fail_power;
	msleep(2);
	/* Enable clock */
		
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	lilp3_debug(LILP3_INFO,"lilp3, %s : flisclk_ctrl, ret=%d, %d\n",__func__,ret,__LINE__);
	if (ret)
		goto fail_clk;
	msleep(1);
	/* Release reset */	
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	lilp3_debug(LILP3_INFO,"lilp3, %s : gpio_ctrl, ret=%d, %d\n",__func__,ret,__LINE__);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* Minumum delay is 8192 clock cycles before first i2c transaction,
	 * which is 1.37 ms at the lowest allowed clock rate 6 MHz */
	msleep(1);
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
	struct imx135_device *dev = to_imx135_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
	udelay(1);
	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int imx135_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	int ret, r;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E==============================\n", __func__);
	lilp3_debug(LILP3_INFO,"lilp3, %s : on = %d, %d\n",__func__,on,__LINE__);
	if (on == 0) {
		imx135_uninit(sd);
		ret = power_down(sd);
		r = DW9719_power_down(sd);
		if (ret == 0)
			ret = r;
		dev->power = 0;
	} else {
		DW9719_power_up(sd);
		ret = power_up(sd);
		lilp3_debug(LILP3_INFO,"lilp3, %s : power_up, ret = %d, %d\n",__func__,ret,__LINE__);
		if (ret)
			return ret;

		dev->power = 1;
		/* init motor initial position */
		//lilp3_debug(LILP3_DEBUG,"=========imx135_s_power before init \n");
		ret = __imx135_init(sd, 0);
		lilp3_debug(LILP3_INFO,"lilp3, %s : __imx135_init, ret = %d, %d\n",__func__,ret,__LINE__);
	}
	//lilp3_debug(LILP3_DEBUG,"imx135_s_power=============exit\n");
	return ret;
}

static int imx135_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_IMX135, 0);
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

/* Return value of the specified register, first try getting it from
 * the register list and if not found, get from the sensor via i2c.
 */
static int imx135_get_register(struct v4l2_subdev *sd, int reg,
			       const struct imx135_reg *reglist)
{


	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct imx135_reg *next;
	u16 val;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/* Try if the values is in the register list */
	for (next = reglist; next->type != IMX135_TOK_TERM; next++) {
		if (next->type != IMX135_8BIT) {
			v4l2_err(sd, "only 8-bit registers supported\n");
			return -ENXIO;
		}
		if (next->reg.sreg == reg)
			return next->val;
	}

	/* If not, read from sensor */
	if (imx135_read_reg(client, IMX135_8BIT, reg, &val)) {
		v4l2_err(sd, "failed to read register 0x%04X\n", reg);
		return -EIO;
	}

	return val;
}

static int imx135_get_intg_factor(struct v4l2_subdev *sd,
				  struct camera_mipi_info *info,
				  const struct imx135_reg *reglist)
{

#if 0
	const int ext_clk = 19200000; /* MHz */
	struct sensor_mode_data *m = (struct sensor_mode_data *)&info->data;
	int pll2_prediv;
	int pll2_multiplier;
	int pll2_divs;
	int pll2_seld5;
	int timing_vts_hi;
	int timing_vts_lo;
	int timing_hts_hi;
	int timing_hts_lo;
	int t1, t2, t3;
	int sclk;

	memset(&info->data, 0, sizeof(info->data));

	timing_vts_hi = imx135_get_register(sd, IMX135_TIMING_VTS, reglist);
	timing_vts_lo = imx135_get_register(sd, IMX135_TIMING_VTS + 1, reglist);
	timing_hts_hi = imx135_get_register(sd, IMX135_TIMING_HTS, reglist);
	timing_hts_lo = imx135_get_register(sd, IMX135_TIMING_HTS + 1, reglist);

	pll2_prediv     = imx135_get_register(sd, IMX135_PLL_PLL10, reglist);
	pll2_multiplier = imx135_get_register(sd, IMX135_PLL_PLL11, reglist);
	pll2_divs       = imx135_get_register(sd, IMX135_PLL_PLL12, reglist);
	pll2_seld5      = imx135_get_register(sd, IMX135_PLL_PLL13, reglist);

	if (timing_vts_hi < 0 || timing_vts_lo < 0 ||
	    timing_hts_hi < 0 || timing_vts_lo < 0 ||
	    pll2_prediv < 0 || pll2_multiplier < 0 ||
	    pll2_divs < 0 || pll2_seld5 < 0)
		return -EIO;

	pll2_prediv &= 0x07;
	pll2_multiplier &= 0x3F;
	pll2_divs = (pll2_divs & 0x0F) + 1;
	pll2_seld5 &= 0x03;

	if (pll2_prediv <= 0)
		return -EIO;

	t1 = ext_clk / pll2_prediv;
	t2 = t1 * pll2_multiplier;
	t3 = t2 / pll2_divs;
	sclk = t3;
	if (pll2_seld5 == 0)
		sclk = t3;
	else if (pll2_seld5 == 3)
		sclk = t3 * 2 / 5;
	else
		sclk = t3 / pll2_seld5;
	m->vt_pix_clk_freq_mhz = sclk;

	m->frame_length_lines = (timing_vts_hi  << 8) | timing_vts_lo;
	m->line_length_pck = (timing_hts_hi << 8) | timing_hts_lo;

	return 0;
#endif
   // struct i2c_client *client = v4l2_get_subdevdata(sd);
	//sensor_register	vt_pix_clk_div;
	s32 bin_x, bin_y;
    /* TODO: this should not be a constant but should be set by a call to
     * MSIC's driver to get the ext_clk that MSIC supllies to the sensor.
     */
	struct atomisp_sensor_mode_data buf;
	//const struct imx135_resolution *next = reglist;
	int vt_pix_clk_freq_mhz= 0;
	u32 data[2];

	sensor_register coarse_integration_time_min;
	sensor_register coarse_integration_time_max_margin;
	sensor_register fine_integration_time_min;
	sensor_register fine_integration_time_max_margin;
	sensor_register frame_length_lines;
	sensor_register line_length_pck;
	sensor_register read_mode;

    int ext_clock=19200000;
	int prepllck_op_div=0;
	int pll_op_mpy=0;
	int vtsyck_div=0;
	int vtpxck_div=0;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (info == NULL)
		return -EINVAL;

	prepllck_op_div=imx135_get_register(sd,0x0305,reglist);
	lilp3_debug(LILP3_DEBUG,"prepllck_op_div=%d\n", prepllck_op_div);
	pll_op_mpy=imx135_get_register(sd,0x030C,reglist);
	lilp3_debug(LILP3_DEBUG,"pll_op_mpy_high=%d\n", pll_op_mpy);
	pll_op_mpy=(pll_op_mpy & 0x07)<<8 | imx135_get_register(sd,0x030D,reglist);
	lilp3_debug(LILP3_DEBUG,"pll_op_mpy_low=%d\n", imx135_get_register(sd,0x030D,reglist));
	lilp3_debug(LILP3_DEBUG,"pll_op_mpy=%d\n",pll_op_mpy);
	vtsyck_div = imx135_get_register(sd,0x0303,reglist)&0x03;
	lilp3_debug(LILP3_DEBUG,"vtsyck_div=%d\n", vtsyck_div);
	vtpxck_div = imx135_get_register(sd,0x0301,reglist)&0x1f;
	lilp3_debug(LILP3_DEBUG,"vtpxck_div=%d\n", vtpxck_div);

	vt_pix_clk_freq_mhz =((ext_clock)/(prepllck_op_div*vtsyck_div*vtpxck_div))*pll_op_mpy;

#define COMBINE(x, y)	(((0xff & x) << 8 ) | (0xff & y))
#define IMX135_FRAME_LENGTH_LINES_HIGH	0x0340
#define IMX135_FRAME_LENGTH_LINES_LOW	0x0341
#define IMX135_LINE_LENGTH_PCK_HIGH	0x0342
#define IMX135_LINE_LENGTH_PCK_LOW	0x0343
#define IMX135_COARSE_INTE_MIN_HIGH	0x1004
#define IMX135_COARSE_INTE_MIN_LOW	0x1005
#define IMX135_COARSE_INTE_MAX_MARG_HIGH	0x1006
#define IMX135_COARSE_INTE_MAX_MARG_LOW		0x1007

	memset(data, 0, 2 * sizeof(u32));
	data[0]=imx135_get_register(sd, IMX135_FRAME_LENGTH_LINES_HIGH, reglist);

	data[1]=imx135_get_register(sd, IMX135_FRAME_LENGTH_LINES_LOW, reglist);

	frame_length_lines = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_LINE_LENGTH_PCK_HIGH, reglist);
	
	data[1]=imx135_get_register(sd, IMX135_LINE_LENGTH_PCK_LOW, reglist);
	
	line_length_pck = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_COARSE_INTE_MIN_HIGH, reglist);
	
	data[1]=imx135_get_register(sd, IMX135_COARSE_INTE_MIN_LOW, reglist);
	
	coarse_integration_time_min = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_COARSE_INTE_MAX_MARG_HIGH, reglist);
	
	data[1]=imx135_get_register(sd, IMX135_COARSE_INTE_MAX_MARG_LOW, reglist);
	
	coarse_integration_time_max_margin = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_CROP_X_START_HIGH, reglist);
	data[1]=imx135_get_register(sd, IMX135_CROP_X_START_LOW, reglist);
	buf.crop_horizontal_start = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_CROP_Y_START_HIGH, reglist);
	data[1]=imx135_get_register(sd, IMX135_CROP_Y_START_LOW, reglist);
	buf.crop_vertical_start = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_CROP_X_END_HIGH, reglist);
	data[1]=imx135_get_register(sd, IMX135_CROP_X_END_LOW, reglist);
	buf.crop_horizontal_end = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_CROP_Y_END_HIGH, reglist);
	data[1]=imx135_get_register(sd, IMX135_CROP_Y_END_LOW, reglist);
	buf.crop_vertical_end = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_OUTPUT_WIDTH_HIGH, reglist);
	data[1]=imx135_get_register(sd, IMX135_OUTPUT_WIDTH_LOW, reglist);
	buf.output_width = COMBINE(data[0], data[1]);

	data[0]=imx135_get_register(sd, IMX135_OUTPUT_HEIGHT_HIGH, reglist);
	data[1]=imx135_get_register(sd, IMX135_OUTPUT_HEIGHT_LOW, reglist);
	buf.output_height = COMBINE(data[0], data[1]);

	imx135_g_bin_factor_y(sd, &bin_y);
	imx135_g_bin_factor_y(sd, &bin_x);
	buf.binning_factor_x = bin_x + 1;
	buf.binning_factor_y = bin_y + 1;

	/*There is no such fine integration time items*/
	fine_integration_time_min = 0;
	fine_integration_time_max_margin = 0;


	read_mode = 0;

	buf.coarse_integration_time_min = coarse_integration_time_min;
	buf.coarse_integration_time_max_margin = coarse_integration_time_max_margin;
	buf.fine_integration_time_min = fine_integration_time_min;
	buf.fine_integration_time_max_margin = fine_integration_time_max_margin;
	buf.vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz*2;
	buf.line_length_pck = line_length_pck;
	buf.frame_length_lines = frame_length_lines;
	buf.read_mode = read_mode;
	lilp3_debug(LILP3_DEBUG,"lilp3, vt_pix_clk_freq_mhz=%d\n", buf.vt_pix_clk_freq_mhz);
	lilp3_debug(LILP3_DEBUG,"lilp3, line_length_pck=%d\n", buf.line_length_pck);
	lilp3_debug(LILP3_DEBUG,"lilp3, frame_length_lines=%d\n", buf.frame_length_lines);
	lilp3_debug(LILP3_DEBUG,"lilp3, read_mode=%d\n", buf.read_mode);

	memcpy(&info->data, &buf, sizeof(buf));

	return 0;

}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int imx135_q_exposure(struct v4l2_subdev *sd, s32 *value)
{


	struct imx135_device *dev = NULL;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	dev=to_imx135_sensor(sd);
	*value = dev->exposure;
	return 0;
}

static int imx135_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	return imx135_write_reg(client, IMX135_16BIT, 0x3070, value);
}

static int imx135_v_flip(struct v4l2_subdev *sd, s32 value)
{
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	return -ENXIO;
}

static int imx135_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	*val = (IMX135_FOCAL_LENGTH_NUM << 16) | IMX135_FOCAL_LENGTH_DEM;
	return 0;
}

static int imx135_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	/*const f number for imx135*/
	*val = (IMX135_F_NUMBER_DEFAULT_NUM << 16) | IMX135_F_NUMBER_DEM;
	return 0;
}

static int imx135_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	*val = (IMX135_F_NUMBER_DEFAULT_NUM << 24) |
		(IMX135_F_NUMBER_DEM << 16) |
		(IMX135_F_NUMBER_DEFAULT_NUM << 8) | IMX135_F_NUMBER_DEM;
	return 0;
}

static int imx135_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{


	struct imx135_device *dev = to_imx135_sensor(sd);
	int factor_enable=0;
	int r =0;

	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	r=imx135_get_register(sd, IMX135_TIMING_X_INC,
		imx135_res[dev->fmt_idx].regs);

    factor_enable =  imx135_get_register(sd, IMX135_BINNING_ENABLE,
		imx135_res[dev->fmt_idx].regs);

	if (r < 0)
		return r;
	if(r==0x11 || factor_enable==0)
	{
		*val = 0;
		return 0;
	}
	if(r==0x22)
	{
		*val = 1;
		return 0;
	}
	if(r==0x44)
	{
		*val = 2;
		return 0;
	}

	return 0;
}

static int imx135_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{

	struct imx135_device *dev = to_imx135_sensor(sd);
	int factor_enable=0;
	int r = 0;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	r=imx135_get_register(sd, IMX135_TIMING_Y_INC,
		imx135_res[dev->fmt_idx].regs);


    factor_enable =  imx135_get_register(sd, IMX135_BINNING_ENABLE,
		imx135_res[dev->fmt_idx].regs);

	if (r < 0)
		return r;

	if(r==0x11 || factor_enable==0)
	{
		*val = 0;
		return 0;
	}
	if(r==0x22)
	{
		*val = 1;
		return 0;
	}
	if(r==0x44)
	{
		*val = 2;
		return 0;
	}

	return 0;
}

static struct imx135_control imx135_controls[] = {
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
		.query = imx135_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = imx135_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = imx135_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = DW9719_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = DW9719_t_focus_abs,
		.query = DW9719_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = -DW9719_MAX_FOCUS_POS,
			.maximum = DW9719_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = DW9719_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.minimum = 0,
			.maximum = 100, /* allow enum to grow in the future */
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = DW9719_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = IMX135_FOCAL_LENGTH_DEFAULT,
			.maximum = IMX135_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = IMX135_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = imx135_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = IMX135_F_NUMBER_DEFAULT,
			.maximum = IMX135_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = IMX135_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = imx135_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = IMX135_F_NUMBER_RANGE,
			.maximum =  IMX135_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = IMX135_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = imx135_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = IMX135_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_READ_ONLY,
		},
		.query = imx135_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = IMX135_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_READ_ONLY,
		},
		.query = imx135_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(imx135_controls))

static struct imx135_control *imx135_find_control(u32 id)
{
	int i;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	for (i = 0; i < N_CONTROLS; i++)
		if (imx135_controls[i].qc.id == id)
			return &imx135_controls[i];
	return NULL;
}

static int imx135_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct imx135_control *ctrl = imx135_find_control(qc->id);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (ctrl == NULL)
		return -EINVAL;

	*qc = ctrl->qc;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

/* imx135 control set/get */
static int imx135_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct imx135_control *s_ctrl;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (!ctrl)
		return -EINVAL;

	s_ctrl = imx135_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return s_ctrl->query(sd, &ctrl->value);
}

static int imx135_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct imx135_control *octrl = imx135_find_control(ctrl->id);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return octrl->tweak(sd, ctrl->value);
}

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
#define LARGEST_ALLOWED_RATIO_MISMATCH2 819
static int distance(struct imx135_resolution const *res, const u32 w,
				const u32 h)
{
	unsigned int w_ratio = ((res->width<<13)/w);
	unsigned int h_ratio = ((res->height<<13)/h);

	int match   = abs(((w_ratio<<13)/h_ratio) - ((int)8192));
	int match2  = abs((res->width<<13)/res->height-(w<<13)/h);
	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)
		|| (match > LARGEST_ALLOWED_RATIO_MISMATCH) || (match2 >=LARGEST_ALLOWED_RATIO_MISMATCH2))
		return -1;

	return w_ratio + h_ratio;
}

/*
 * Returns the nearest higher resolution index.
 * @w: width
 * @h: height
 * matching is done based on enveloping resolution and
 * aspect ratio. If the aspect ratio cannot be matched
 * to any index, -1 is returned.
 */
#define PIC_INDEX_1920_1080 4 //need delete after ZSL is OK
#define PIC_INDEX_2048_1536 6 //need delete after ZSL is OK
#define PREV_INDEX_2064_1552 6
#define PIC_INDEX_3600_2704 16
#define PIC_INDEX_3600_2032 13
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct imx135_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &imx135_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}
	/*workaround for 1080p and 2048_1536 capture workaround, need delete once ZSL is OK begin*/
/*	if((((w==1920) && (h==1080))||((w==1936)&&(h=1096)))
		&&((int)imx135_res ==(int)imx135_res_still))
		idx=PIC_INDEX_1920_1080;
	if((((w==2048)&&(h==1536))||((w==2064)&&(h==1552)))
		&&((int)imx135_res ==(int)imx135_res_still))
		idx=PIC_INDEX_2048_1536;	
		*/
		if(((w>=960 && w<=980)&&(h>=720 && h<=740))
			&&((int)imx135_res ==(int)imx135_res_preview))
			idx=PREV_INDEX_2064_1552;
		if(((w>=3584 && w<=3600)&&(h>=2688 && h<=2704))
			&&((int)imx135_res ==(int)imx135_res_still))
			idx=PIC_INDEX_3600_2704;
		if(((w>=3584 && w<=3600)&&(h>=2016 && h<=2032))
			&&((int)imx135_res ==(int)imx135_res_still))
			idx=PIC_INDEX_3600_2032;
	
	/*workaround for 1080p and 2048_1536 capture workaround, need delete once ZSL is OK end*/

	
	//lilp3_debug(LILP3_DEBUG,"=====================================>xhx idx=%d w=%d, h=%d", idx, w, h);
	lilp3_debug(LILP3_INFO,"lilp3, %s : idx = %d w = %d, h = %d, %d\n",__func__,idx, w, h,__LINE__);
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		lilp3_debug(LILP3_INFO,"lilp3, %s w=%d, h=%d, width[%d]=%d, height[%d]=%d\n",__func__,
			w, h,
			i,
			imx135_res[i].width,
			i,
			imx135_res[i].height);
		if (w != imx135_res[i].width)
			continue;
		if (h != imx135_res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int imx135_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int idx;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_INFO,"lilp3, %s : fmt->width = %d, fmt->height = %d, %d\n",__func__,fmt->width,fmt->height,__LINE__);

	if (!fmt)
		return -EINVAL;

	if ((fmt->width > IMX135_RES_WIDTH_MAX) || (fmt->height > IMX135_RES_HEIGHT_MAX)) {
		fmt->width = IMX135_RES_WIDTH_MAX;
		fmt->height = IMX135_RES_HEIGHT_MAX;
	} else {
		idx = nearest_resolution_index(fmt->width, fmt->height);
		lilp3_debug(LILP3_INFO,"lilp3, %s : nearest_resolution_index : idx = %d, width = %d, height = %d, %d\n",__func__,idx,imx135_res[idx].width,imx135_res[idx].height,__LINE__);

		/*
		 * nearest_resolution_index() doesn't return smaller resolutions.
		 * If it fails, it means the requested resolution is higher than we
		 * can support. Fallback to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = N_RES - 1;

		fmt->width = imx135_res[idx].width;
		fmt->height = imx135_res[idx].height;
	}

	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}
int readout_nr_registers(struct v4l2_subdev *sd)
{
	int ret=0;
	u8 val=0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

   lilp3_debug(LILP3_INFO,"=======================%s readout the nr registers\n", __func__);
   ret = imx135_read_reg_byte(client, IMX135_8BIT,0x4203 , &val);

	if (ret) {
		printk("%s:  ret=%d]\n", __func__,ret );

		return ret;
	} 
	lilp3_debug(LILP3_INFO,"%s:  [0x4203]=[0x%x]\n", __func__,val );

    ret = imx135_read_reg_byte(client, IMX135_8BIT,0x4216 , &val);
	if (ret) {
		printk("%s:  ret=%d]\n", __func__,ret );

		return ret;
	} 
	lilp3_debug(LILP3_INFO,"%s:  [0x4216]=[0x%x]\n", __func__,val );
	
	ret = imx135_read_reg_byte(client, IMX135_8BIT,0x4217 , &val);
	if (ret) {
		
		printk("%s:  ret=%d]\n", __func__,ret );
		

		return ret;
	} 
	lilp3_debug(LILP3_INFO,"%s:  [0x4217]=[0x%x]\n", __func__,val );

	ret = imx135_read_reg_byte(client, IMX135_8BIT,0x428B , &val);
	if (ret) {
		
		printk( "%s:  ret=%d]\n", __func__,ret );
		

		return ret;
	} 
	lilp3_debug(LILP3_INFO,"%s:  [0x428B]=[0x%x]\n", __func__,val );


	ret = imx135_read_reg_byte(client, IMX135_8BIT,0x428F , &val);
	if (ret) {
		
		printk("%s:  ret=%d]\n", __func__,ret );
		

		return ret;
	} 
	lilp3_debug(LILP3_INFO,"%s:  [0x428F]=[0x%x]\n", __func__,val );


	 ret = imx135_read_reg_byte(client, IMX135_8BIT,0x4298 , &val);
	 if (ret) {
		
		printk("%s:  ret=%d]\n", __func__,ret );
		return ret;
	 } 
	lilp3_debug(LILP3_INFO,"%s:	[0x4298]=[0x%x]\n", __func__,val );

	 
	 ret = imx135_read_reg_byte(client, IMX135_8BIT,0x429A , &val);
	 if (ret) {
		
		printk("%s:  ret=%d]\n", __func__,ret );
		
	 
		 return ret;
	 } 
	 lilp3_debug(LILP3_INFO,"%s:  [0x429A]=[0x%x]\n", __func__,val );
	return ret;


}



#define ANR1 	\
				{ IMX135_8BIT, { 0x4216 }, 0x00 }, \
				{ IMX135_8BIT, { 0x4217 }, 0x08 },
				
#define ANR2	\
				{ IMX135_8BIT, { 0x4216 }, 0x08 }, \
				{ IMX135_8BIT, { 0x4217 }, 0x08 },
				
#define ANR3	\
				{ IMX135_8BIT, { 0x4216 }, 0x08 }, \
				{ IMX135_8BIT, { 0x4217 }, 0x10 },
				
#define CNR1	\
				{ IMX135_8BIT, { 0x428B }, 0x0F }, \
  				{ IMX135_8BIT, { 0x428F }, 0x0F }, \
  				{ IMX135_8BIT, { 0x4298 }, 0x0E }, \
  				{ IMX135_8BIT, { 0x429A }, 0x0E },
  				
#define CNR2	\
				{ IMX135_8BIT, { 0x428B }, 0x3F }, \
				{ IMX135_8BIT, { 0x428F }, 0x3F }, \
				{ IMX135_8BIT, { 0x4298 }, 0x3E }, \
				{ IMX135_8BIT, { 0x429A }, 0x3E },
				
#define CNR3	\
				{ IMX135_8BIT, { 0x428B }, 0x7F }, \
				{ IMX135_8BIT, { 0x428F }, 0x7F }, \
				{ IMX135_8BIT, { 0x4298 }, 0x7E }, \
				{ IMX135_8BIT, { 0x429A }, 0x7E },



struct imx135_reg imx135_anr0_cnr1[]={
	CNR1
	
	{ IMX135_TOK_TERM, {0}, 0}

};
struct imx135_reg imx135_anr0_cnr2[]=
{
	CNR2
	{ IMX135_TOK_TERM, {0}, 0}

};

struct imx135_reg imx135_anr0_cnr3[]=
{
	CNR3
	{ IMX135_TOK_TERM, {0}, 0}

};
struct imx135_reg imx135_anr1_cnr0[]=
{
	ANR1
	{ IMX135_TOK_TERM, {0}, 0}

};
struct imx135_reg imx135_anr1_cnr1[]=
{
	ANR1
	CNR1
	{ IMX135_TOK_TERM, {0}, 0}

};

struct imx135_reg imx135_anr1_cnr2[]=
{
	ANR1
	CNR2
	{ IMX135_TOK_TERM, {0}, 0}

};
struct imx135_reg imx135_anr1_cnr3[]=
{
	ANR1
	CNR3
	{ IMX135_TOK_TERM, {0}, 0}
};
struct imx135_reg imx135_anr2_cnr0[]=
{
	ANR2
	{ IMX135_TOK_TERM, {0}, 0}
};
struct imx135_reg imx135_anr2_cnr1[]=
{
	ANR2
	CNR1
	{ IMX135_TOK_TERM, {0}, 0}

};
struct imx135_reg imx135_anr2_cnr2[]=
{
	ANR2
	CNR2
	{ IMX135_TOK_TERM, {0}, 0}

};
struct imx135_reg imx135_anr2_cnr3[]=
{
	ANR2
	CNR3
	{ IMX135_TOK_TERM, {0}, 0}
};
struct imx135_reg imx135_anr3_cnr0[]=
{
	ANR3
	{ IMX135_TOK_TERM, {0}, 0}
};

struct imx135_reg imx135_anr3_cnr1[]=
{
	ANR3
	CNR1
	{ IMX135_TOK_TERM, {0}, 0}
};
struct imx135_reg imx135_anr3_cnr2[]=
{
	ANR3
	CNR2
	{ IMX135_TOK_TERM, {0}, 0}
};
struct imx135_reg imx135_anr3_cnr3[]=
{
	ANR3
	CNR3
	{ IMX135_TOK_TERM, {0}, 0}
};

struct imx135_reg *imx135_anr_cnr_array[]=
{
	imx135_anr0_cnr1,
	imx135_anr0_cnr2,
	imx135_anr0_cnr3,
	imx135_anr1_cnr0,
	imx135_anr1_cnr1,
	imx135_anr1_cnr2,
	imx135_anr1_cnr3,
	imx135_anr2_cnr0,
	imx135_anr2_cnr1,
	imx135_anr2_cnr2,
	imx135_anr2_cnr3,
	imx135_anr3_cnr0,
	imx135_anr3_cnr1,
	imx135_anr3_cnr2,
	imx135_anr3_cnr3


};
#ifdef IMX135_DEBUG_NR
int nr_process_after_reg_array(struct v4l2_subdev *sd)
{
	
	//struct imx135_device *dev = to_imx135_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	printk("lilp3, %s : %d nr_onoff=%d, nr_type=%d,nr_index=%d\n"
		,__func__,__LINE__,
		nr_onoff,nr_type,nr_index);
	
	ret=imx135_write_reg(client, IMX135_8BIT, 0x4203, nr_onoff);

	if (ret != 0) {
		v4l2_err(client, "failed to set streaming\n");
		return ret;
	}
	//nr_onoff=0xFF;
	if(nr_index!=0)
	{
		ret = imx135_write_reg_array(client, imx135_anr_cnr_array[nr_index-1]);
		if (ret)
			return -EINVAL;	
	}
	readout_nr_registers(sd);

	//nr_index=0;
	//nr_type=0;
	return 0;
}

#endif

static int imx135_denoise_setting(struct v4l2_subdev *sd,int denoise_selection, int denoise_index)
{

	//struct imx135_device *dev = to_imx135_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	
	lilp3_debug(LILP3_INFO,"%s : %d denoise_selection=%d, nr_index=%d\n",__func__,__LINE__,denoise_selection,denoise_index);
	ret = imx135_grouphold_start(sd);
	if (ret != 0) {
		v4l2_err(client, "failed to set hold\n");
		return ret;
	}	

	switch(denoise_selection)
	{
		case 0xFF:
			lilp3_debug(LILP3_INFO,"%s disable all\n", __func__);
			ret=imx135_write_reg(client, IMX135_8BIT, 0x4203, denoise_selection);
			if (ret != 0) {
				v4l2_err(client, "failed to set streaming\n");
				return ret;
			}		
			
			return ret;
		case 0xC1:
			lilp3_debug(LILP3_INFO,"%s all on\n", __func__);

		case 0xC3:
			lilp3_debug(LILP3_INFO,"%s DLC/ARNR ON\n", __func__);

		case 0xC9:
			lilp3_debug(LILP3_INFO,"%s DLC/LFNR ON\n", __func__);
		case 0xF1:
			lilp3_debug(LILP3_INFO,"%s ARNR/LFNR ON\n", __func__);

		case 0xCB:

			lilp3_debug(LILP3_INFO,"%s DLC ON\n", __func__);

		case 0xF3:		
			lilp3_debug(LILP3_INFO,"%s ARNR ON\n", __func__);

		case 0xF9:
			lilp3_debug(LILP3_INFO,"%s LFNR ON\n", __func__);
			ret=imx135_write_reg(client, IMX135_8BIT, 0x4203, denoise_selection);
			if (ret != 0) {
				v4l2_err(client, "failed to set streaming\n");
				return ret;
			}	

			break;
		default:
			printk("%s invalid denoise selection\n", __func__);

			break;


		
	}

	if(denoise_index!=0)
	{
		ret = imx135_write_reg_array(client, imx135_anr_cnr_array[denoise_index-1]);
		if (ret)
			return -EINVAL;	
	}
	ret=imx135_grouphold_launch(sd);
	if (ret != 0) {
		v4l2_err(client, "failed to set hold\n");
		return ret;
	}		
	readout_nr_registers(sd);
	return ret;

}
static int imx135_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	const struct imx135_reg *imx135_def_reg;
	struct camera_mipi_info *imx135_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	imx135_info = v4l2_get_subdev_hostdata(sd);
	if (imx135_info == NULL)
		return -EINVAL;

	ret = imx135_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}
	//lilp3_debug(LILP3_DEBUG,"=========s_mbus_fmt width=%d, height=%d\n", fmt->width, fmt->height);
	lilp3_debug(LILP3_INFO,"lilp3, %s : fmt->width = %d, fmt->height = %d, %d\n",__func__,fmt->width,fmt->height,__LINE__);
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);
    //lilp3_debug(LILP3_DEBUG,"=========s_mbus_fmt, idx=%d\n",dev->fmt_idx);
	lilp3_debug(LILP3_INFO,"lilp3, %s : get_resolution_index : idx = %d, width = %d, height = %d, %d\n",__func__,dev->fmt_idx,imx135_res[dev->fmt_idx].width,imx135_res[dev->fmt_idx].height,__LINE__);
	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}
	printk("%s width=%d,height=%d index=%d\n",__func__,
		fmt->width,fmt->height,  dev->fmt_idx);
	imx135_def_reg = imx135_res[dev->fmt_idx].regs;
	ret = imx135_write_reg_array(client, imx135_def_reg);
	if (ret)
		return -EINVAL;

#ifdef IMX135_DEBUG_NR
if(imx135_res[dev->fmt_idx].width>=4200)
	nr_process_after_reg_array(sd);

#else
	/*use arnr2+cnr1*/
	if(imx135_res==imx135_res_still)
		imx135_denoise_setting(sd,0xF1,9);
	/*use arnr2+cnr1*/


#endif


	dev->fps = imx135_res[dev->fmt_idx].fps;
	dev->pixels_per_line = imx135_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = imx135_res[dev->fmt_idx].lines_per_frame;

	ret = imx135_get_intg_factor(sd, imx135_info, imx135_PLL192MHz);
	if (ret) {
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : dev->exposure = %d, %d\n",__func__,dev->exposure,__LINE__);
	/* restore exposure, gain settings */
	if (dev->exposure) {
		ret = imx135_set_exposure(sd, dev->exposure, dev->gain, dev->digitgain);
		if (ret)
			v4l2_warn(sd, "failed to set exposure time\n");
	}
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static int imx135_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);

	if (!fmt)
		return -EINVAL;

	fmt->width = imx135_res[dev->fmt_idx].width;
	fmt->height = imx135_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static int imx135_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (imx135_read_reg(client, IMX135_8BIT, 0x0016,
			     &high)) {
		v4l2_err(client, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	if (imx135_read_reg(client, IMX135_8BIT, 0x0017,
			     &low)) {
		v4l2_err(client, "sensor_id_low = 0x%x\n", high);
		return -ENODEV;
	}
	*id = (((u8) high) << 8) | (u8) low;
	v4l2_info(client, "sensor_id = 0x%x\n", *id);
	real_model_id = *id;

	/* Reco settings changes this 0x2a88 from init registers*/
	if (*id != 0x0135) {
		v4l2_err(client, "sensor ID error\n");
		return -ENODEV;
	}

	v4l2_info(client, "detect imx135 success\n");

	/* REVISIT: HACK: Driver is currently forcing revision to 0 */
	*revision = 0;

	return 0;
}

/*
 * imx135 stream on/off
 */
static int imx135_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E enable=%d\n", __func__, enable);
	int ret = 0;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : enable = %d, %d\n",__func__,enable,__LINE__);
	ret=imx135_write_reg(client, IMX135_8BIT, 0x0100, enable ? 1 : 0);
	if (ret != 0) {
		v4l2_err(client, "failed to set streaming\n");
		return ret;
	}

	dev->streaming = enable;

	/* restore settings */
	imx135_res = imx135_res_preview;
	N_RES = N_RES_PREVIEW;
	if(dev->run_mode == CI_MODE_VIDEO) {
		imx135_res = imx135_res_video;
		N_RES = N_RES_VIDEO;
	}
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

/*
 * imx135 enum frame size, frame intervals
 */
static int imx135_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = imx135_res[index].width;
	fsize->discrete.height = imx135_res[index].height;
	fsize->reserved[0] = imx135_res[index].used;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static int imx135_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (index >= N_RES)
		return -EINVAL;

	/* since the isp will donwscale the resolution to the right size, find the nearest one that will allow the isp to do so
	 * important to ensure that the resolution requested is padded correctly by the requester, which is the atomisp driver in this case.
	 */
	index = nearest_resolution_index(fival->width, fival->height);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d index=%d w:%d h:%d\n",__func__,__LINE__,index, fival->width, fival->height);

	if (-1 == index)
		index = N_RES - 1;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
/*	fival->width = imx135_res[index].width;
	fival->height = imx135_res[index].height; */
	fival->discrete.numerator = 1;
	fival->discrete.denominator = imx135_res[index].fps;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static int imx135_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	*code = V4L2_MBUS_FMT_SGRBG10_1X10;
	return 0;
}

static int imx135_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;
	void *otp_data;

	if (pdata == NULL)
		return -ENODEV;
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	dev->platform_data = pdata;
	//lilp3_debug(LILP3_DEBUG,"====================>xhx imx135_s_config\n");
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			v4l2_err(client, "imx135 platform init err\n");
			return ret;
		}
	}
	ret = imx135_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "imx135 power-up err.\n");
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = imx135_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "imx135_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;
	/* Read sensor's OTP data */
	otp_data = imx135_otp_read(sd);
	if (!IS_ERR(otp_data))
		dev->otp_data = otp_data+IMX135_OTP_INTEL_START_ADDR;

	/* power off sensor */
	ret = imx135_s_power(sd, 0);
	if (ret) {
		v4l2_err(client, "imx135 power-down err.\n");
		return ret;
	}

	return 0;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	imx135_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
imx135_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static int
imx135_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = imx135_res[index].width;
	fse->min_height = imx135_res[index].height;
	fse->max_width = imx135_res[index].width;
	fse->max_height = imx135_res[index].height;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static struct v4l2_mbus_framefmt *
__imx135_get_pad_format(struct imx135_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);

	if (pad != 0) {
		v4l2_err(client, "%s err. pad %x\n", __func__, pad);
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
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);
}

static int
imx135_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{

	struct imx135_device *dev =NULL;
	struct v4l2_mbus_framefmt *format = NULL;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	dev=to_imx135_sensor(sd);
	format=__imx135_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
imx135_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{

	struct imx135_device *dev =NULL;

	struct v4l2_mbus_framefmt *format = NULL;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	dev=to_imx135_sensor(sd);
	format=__imx135_get_pad_format(dev, fh, fmt->pad, fmt->which);
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);

	if (format == NULL)
		return -EINVAL;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static int
imx135_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d mode:%d\n",__func__,__LINE__, dev->run_mode);

	dev->run_mode = param->parm.capture.capturemode;

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		printk("CI_MODE_VIDEO \n");
		imx135_res = imx135_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		printk("CI_MODE_STILL_CAPTURE \n");
		imx135_res = imx135_res_still;
		N_RES = N_RES_STILL;
		break;
	case CI_MODE_HDR_MOVIE:
		printk("CI_MODE_HDR_MOVIE \n");
		imx135_res = imx135_res_hdr_movie;
		N_RES = N_RES_HDR_MOVIE;
		break;
	default:
		printk("CI_MODE_PREVIEW \n");
		imx135_res = imx135_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);
	return 0;
}

static int
imx135_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct imx135_device *dev = to_imx135_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;
	//lilp3_debug(LILP3_DEBUG,"=========>%s E\n", __func__);
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);

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
	interval->interval.denominator = IMX135_MCLK * 1000000;
	//lilp3_debug(LILP3_DEBUG,"=========>%s X\n", __func__);

	return 0;
}

static const struct v4l2_subdev_video_ops imx135_video_ops = {
	.s_stream = imx135_s_stream,
	.enum_framesizes = imx135_enum_framesizes,
	.enum_frameintervals = imx135_enum_frameintervals,
	.enum_mbus_fmt = imx135_enum_mbus_fmt,
	.try_mbus_fmt = imx135_try_mbus_fmt,
	.g_mbus_fmt = imx135_g_mbus_fmt,
	.s_mbus_fmt = imx135_s_mbus_fmt,
	.s_parm = imx135_s_parm,
	.g_frame_interval = imx135_g_frame_interval,
};

static const struct v4l2_subdev_core_ops imx135_core_ops = {
	.g_chip_ident = imx135_g_chip_ident,
	.queryctrl = imx135_queryctrl,
	.g_ctrl = imx135_g_ctrl,
	.s_ctrl = imx135_s_ctrl,
	.s_power = imx135_s_power,
	.ioctl = imx135_ioctl,
	.init = imx135_init,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops imx135_pad_ops = {
	.enum_mbus_code = imx135_enum_mbus_code,
	.enum_frame_size = imx135_enum_frame_size,
	.get_fmt = imx135_get_pad_format,
	.set_fmt = imx135_set_pad_format,
};

static const struct v4l2_subdev_ops imx135_ops = {
	.core = &imx135_core_ops,
	.video = &imx135_video_ops,
	.pad = &imx135_pad_ops,
};

static const struct media_entity_operations imx135_entity_ops = {
	.link_setup = NULL,
};

static int imx135_remove(struct i2c_client *client)
{

	struct v4l2_subdev *sd = NULL;
	struct imx135_device *dev = NULL;
	lilp3_debug(LILP3_DEBUG,"lilp3, %s : %d\n",__func__,__LINE__);
	sd=i2c_get_clientdata(client);
	dev=to_imx135_sensor(sd);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev->otp_data);
	kfree(dev);

	return 0;
}

static int imx135_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct imx135_device *dev;
	int ret;
	lilp3_debug(LILP3_INFO,"lilp3, %s : Entry %d\n",__func__,__LINE__);
	//lilp3_debug(LILP3_DEBUG,"============================>xhx imx135_probe\n");
	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	dev->fmt_idx = 0;
	//lilp3_debug(LILP3_DEBUG,"===> imx135 sd:%x\n", (int)&(dev->sd));
	v4l2_i2c_subdev_init(&(dev->sd), client, &imx135_ops);

	ret = DW9719_init(&dev->sd);
	if (ret < 0)
		goto out_free;
	//lilp3_debug(LILP3_DEBUG,"=========================>xhx before platform_data init\n");
	if (client->dev.platform_data) {
		ret = imx135_s_config(&dev->sd, client->irq,
				      client->dev.platform_data);
		if (ret)
			goto out_free;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &imx135_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	dev->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;

	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		imx135_remove(client);
		return ret;
	}
#ifdef IMX135_DEBUG_AF
	proc_create_data("af", S_IFREG | S_IWUGO | S_IWUSR, NULL, &proc_af_operations, NULL);
	imx135_sd=&dev->sd;
#endif
#ifdef IMX135_DEBUG_NR
proc_create_data("nr", S_IFREG | S_IWUGO | S_IWUSR, NULL, &proc_nr_operations, NULL);
imx135_sd_nr=&dev->sd;

#endif
	lilp3_debug(LILP3_INFO,"lilp3, %s : Exit %d\n",__func__,__LINE__);
	return 0;

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	lilp3_debug(LILP3_INFO,"lilp3, %s : ERR free dev %d\n",__func__,__LINE__);
	return ret;
}

static const struct i2c_device_id imx135_id[] = {
	{IMX135_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, imx135_id);

static struct i2c_driver imx135_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = IMX135_NAME,
	},
	.probe = imx135_probe,
	.remove = imx135_remove,
	.id_table = imx135_id,
};

static __init int imx135_init_mod(void)
{
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	return i2c_add_driver(&imx135_driver);
}

static __exit void imx135_exit_mod(void)
{
	lilp3_debug(LILP3_INFO,"lilp3, %s : %d\n",__func__,__LINE__);
	i2c_del_driver(&imx135_driver);
}

module_init(imx135_init_mod);
module_exit(imx135_exit_mod);

MODULE_DESCRIPTION("A low-level driver for Sony IMX135 sensors");
MODULE_LICENSE("GPL");
