/*
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>


static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4330";

static bool previous;

static int bluetooth_set_power(void *data, bool blocked)
{
	int ret = 0;
	int (*power_control)(int enable);
	power_control = data;
	if(previous != blocked)
	   ret = (*power_control)(!blocked);
	if(!ret)
	   previous = blocked;
	return ret;
}

static struct rfkill_ops rfkill_bluetooth_ops = {
	.set_block = bluetooth_set_power,
};

static int bcm4330_rfkill_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "-->%s\n", __func__);

	int rc = 0;
	bool default_state = true;

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
	   &rfkill_bluetooth_ops, pdev->dev.platform_data);
	if (!bt_rfk) {
	 rc = -ENOMEM;
	 goto err_rfkill_alloc;
	}

	/* userspace cannot take exclusive control */
	rfkill_init_sw_state(bt_rfk, 0);
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	rfkill_set_sw_state(bt_rfk, 1);
	bluetooth_set_power((void*)(pdev->dev.platform_data), default_state);
	

	printk(KERN_INFO "<--%s\n", __func__);
	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
err_gpio_shutdown:
err_gpio_reset:
	return rc;
}

static int bcm4330_rfkill_remove(struct platform_device *dev)
{
	printk(KERN_INFO "-->%s\n", __func__);
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	printk(KERN_INFO "<--%s\n", __func__);
	return 0;
}

static struct platform_driver bcm4330_rfkill_platform_driver = {
	.probe  = bcm4330_rfkill_probe,
	.remove = bcm4330_rfkill_remove,
	.driver = {
		.name = "bcm4330_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init bcm4330_rfkill_init(void)
{
	printk(KERN_INFO "-->%s\n", __func__);

	return platform_driver_register(&bcm4330_rfkill_platform_driver);
}

static void __exit bcm4330_rfkill_exit(void)
{
	printk(KERN_INFO "-->%s\n", __func__);
	platform_driver_unregister(&bcm4330_rfkill_platform_driver);
}

late_initcall(bcm4330_rfkill_init);
module_exit(bcm4330_rfkill_exit);
MODULE_DESCRIPTION("bluetooth rfkill");
MODULE_AUTHOR("lenovo");
MODULE_LICENSE("GPL");

