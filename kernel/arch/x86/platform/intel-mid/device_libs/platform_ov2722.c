/*
 * platform_ov2722.c: ov2722 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_ov2722.h"
#include "platform_mt9e013.h"


#define VPROG1_VAL 2800000
static int gp_camera1_reset;
static int gp_camera1_power_down;
static int camera_vprog1_on;


static struct regulator *vprog1_reg;


/*
 * MFLD PR2 secondary camera sensor - OV2722 platform data
 */
static int ov2722_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
#if 0
	int ret;
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
#ifdef CONFIG_BOARD_CTP
		gpio_set_value(camera_reset, 0);
		msleep(60);
#endif
		gpio_set_value(camera_reset, 1);
	} else
		gpio_set_value(camera_reset, 0);

	return 0;
#endif
     int ret;
	 //printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);

     if (gp_camera1_power_down < 0) {
         ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
                     GPIOF_DIR_OUT, 1);
         if (ret < 0)
             return ret;
         gp_camera1_power_down = ret;
     }

     if (gp_camera1_reset < 0) {
         ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
                     GPIOF_DIR_OUT, 1);
         if (ret < 0)
             return ret;
         gp_camera1_reset = ret;
     }

     if (flag) {
         gpio_set_value(gp_camera1_power_down, 0);
         gpio_set_value(gp_camera1_reset, 0);
         msleep(50);
         gpio_set_value(gp_camera1_reset, 1);
     } else {
         gpio_set_value(gp_camera1_reset, 0);
         gpio_set_value(gp_camera1_power_down, 0);
     }

     return 0;

}

static int ov2722_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int mt9e013_reset_value;
static int ov2722_power_ctrl(struct v4l2_subdev *sd, int flag)
{
#if 0
#ifdef CONFIG_BOARD_CTP
	int reg_err;
#endif
#ifndef CONFIG_BOARD_CTP
	int ret;
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);

	/* Note here, there maybe a workaround to avoid I2C SDA issue */
	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
#ifndef CONFIG_BOARD_REDRIDGE
		if (ret < 0)
			return ret;
#endif
		camera_power_down = ret;
	}

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}
#endif
	if (flag) {
#ifndef CONFIG_BOARD_CTP
		if (!mt9e013_reset_value) {
			if (mt9e013_reset)
				mt9e013_reset(sd);
			mt9e013_reset_value = 1;
		}
#ifdef CONFIG_BOARD_REDRIDGE
		gpio_direction_output(camera_reset, 0);
#endif
		gpio_set_value(camera_reset, 0);
#endif
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT, "Failed to enable regulator vprog1\n");
				return reg_err;
			}
#else
			intel_scu_ipc_msic_vprog1(1);
#endif
		}
#ifndef CONFIG_BOARD_CTP
#ifdef CONFIG_BOARD_REDRIDGE
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 1);
#else
		gpio_set_value(camera_power_down, 1);
#endif
#endif
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT, "Failed to disable regulator vprog1\n");
				return reg_err;
			}
#else
			intel_scu_ipc_msic_vprog1(0);
#endif
		}
#ifndef CONFIG_BOARD_CTP
#ifdef CONFIG_BOARD_REDRIDGE
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 0);
#else
		gpio_set_value(camera_power_down, 0);
#endif

		mt9e013_reset_value = 0;
#endif
	}

	return 0;
#endif
#ifdef CONFIG_BOARD_CTP
	int reg_err;
#endif
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);
	if (flag) {
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT, "Failed to enable regulator vprog1\n");
				return reg_err;
#else
			intel_scu_ipc_msic_vprog1(0);
#endif
			}
		}
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
#ifdef CONFIG_BOARD_CTP
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT, "Failed to disable regulator vprog1\n");
				return reg_err;
			}
#else
			intel_scu_ipc_msic_vprog1(1);
#endif
		}
	}
	return 0;
}

static int ov2722_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);	
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov2722_platform_init(struct i2c_client *client)
{
	int ret;
	//printk("lilp3, %s : %d\n",__func__,__LINE__);

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	return ret;
}

static int ov2722_platform_deinit(void)
{
	//printk("lilp3, %s : %d\n",__func__,__LINE__);
	regulator_put(vprog1_reg);
}

static struct camera_sensor_platform_data ov2722_sensor_platform_data = {
	.gpio_ctrl	= ov2722_gpio_ctrl,
	.flisclk_ctrl	= ov2722_flisclk_ctrl,
	.power_ctrl	= ov2722_power_ctrl,
	.csi_cfg	= ov2722_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init = ov2722_platform_init,
	.platform_deinit = ov2722_platform_deinit,
#endif
};

void *ov2722_platform_data(void *info)
{
	gp_camera1_reset = -1;
	gp_camera1_power_down = -1;

	return &ov2722_sensor_platform_data;
}

