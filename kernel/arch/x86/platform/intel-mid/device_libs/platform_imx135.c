/*
 * platform_imx135.c: imx135 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_imx135.h"


static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;

static struct regulator *vprog1_reg;
#define VPROG1_VAL 2800000
/*
 * CLV PR0 primary camera sensor - IMX135 platform data
 */

static int imx135_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {

		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int imx135_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int imx135_power_ctrl(struct v4l2_subdev *sd, int flag)
{
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

static int imx135_csi_configure(struct v4l2_subdev *sd, int flag)
{
	//printk("lilp3, %s : flag = %d, %d\n",__func__,flag,__LINE__);
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static int imx135_platform_init(struct i2c_client *client)
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

static int imx135_platform_deinit(void)
{
	//printk("lilp3, %s : %d\n",__func__,__LINE__);
	regulator_put(vprog1_reg);
}
static struct camera_sensor_platform_data imx135_sensor_platform_data = {
	.gpio_ctrl      = imx135_gpio_ctrl,
	.flisclk_ctrl   = imx135_flisclk_ctrl,
	.power_ctrl     = imx135_power_ctrl,
	.csi_cfg        = imx135_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init = imx135_platform_init,
	.platform_deinit = imx135_platform_deinit,
#endif
};

void *imx135_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &imx135_sensor_platform_data;
}
