/*
 * platform_s3202.c: s3202 platform data initilization file
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
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
/*#include <linux/rmi.h>*/
/*#include <linux/rmi_platformdata.h>*/
#include <linux/input/synaptics_dsx.h>
#include <asm/intel-mid.h>
#include <linux/input.h>
#include <linux/delay.h>
/*#include "platform_s3202.h"*/

#define MRST_IRQ_OFFSET 0x100
#define RMI_INT_GPIO 62

#define rmi_reset_gpio (58)

static int rmi_touchpad_gpio_setup(unsigned interrupt_gpio, bool configure)
{
	int err = 0;
	if(configure == true){
		err = gpio_request(interrupt_gpio,"rmi_int");	
		if(err < 0){
			printk("request gpio rmi_int err.\n");
			return err;
		}	
		err = gpio_direction_input(interrupt_gpio);
		if(err < 0){
			printk("set rmi_int direction err.\n");
			return err;
		}

		err = gpio_request(rmi_reset_gpio, "s3408-reset");
		if (err < 0){
			printk(KERN_ERR "Failed to request GPIO%d (MaxTouch-reset) err=%d\n",
				rmi_reset_gpio, err);
			return err;
		}

		err = gpio_direction_output(rmi_reset_gpio, 0);
		if (err){
			printk(KERN_ERR "Failed to change direction, err=%d\n", err);
			return err;
		}

		/* maXTouch wants 40mSec minimum after reset to get organized */
		gpio_set_value(rmi_reset_gpio, 1);
		msleep(40);

		return err;
	}else{
		gpio_free(interrupt_gpio);	
		gpio_free(rmi_reset_gpio);	
		printk("s3408 free requested gpios.\n");
	}
}
static unsigned char rmi4_f1a_button_codes[] = {KEY_MENU,KEY_HOMEPAGE,KEY_BACK};

static struct synaptics_rmi_f1a_button_map rmi4_f1a_button_map = {
	.nbuttons = ARRAY_SIZE(rmi4_f1a_button_codes),
	.map = rmi4_f1a_button_codes,
};

static struct synaptics_dsx_platform_data s3408_i2c_platform_data = {
	/*.regulator_en = 0,*/
	.gpio = RMI_INT_GPIO,
	.gpio_config = rmi_touchpad_gpio_setup,
	.f1a_button_map = &rmi4_f1a_button_map,
	
};
struct i2c_board_info s3408_i2c_info[] __initdata = {   
		{
				I2C_BOARD_INFO("synaptics_dsx_i2c", 0x20),
			 	.platform_data = &s3408_i2c_platform_data,
				.irq = RMI_INT_GPIO + MRST_IRQ_OFFSET,
		},
		{
				I2C_BOARD_INFO("synaptics_dsx_i2c", 0x38),
			 	.platform_data = &s3408_i2c_platform_data,
				.irq = RMI_INT_GPIO + MRST_IRQ_OFFSET,
		},
};   
