/*
 * Arizona-i2c.c  --  Arizona I2C bus interface
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/mfd/arizona/core.h>

#include "arizona.h"

#define GPIO_ARIZONA	(32)
int arizona_gpio_to_irq_i2c(void)
{
    int ret;

    ret = gpio_request(GPIO_ARIZONA, "arizona_irq_i2c");
    if (ret) {
        printk("###enter %s, in line %d\n", __func__, __LINE__);
    }
    gpio_direction_input(GPIO_ARIZONA);
    gpio_to_irq(GPIO_ARIZONA);

    return ret;
}

static __devinit int arizona_i2c_probe(struct i2c_client *i2c,
					  const struct i2c_device_id *id)
{
	struct arizona *arizona;
	const struct regmap_config *regmap_config;
	int ret;

	switch (id->driver_data) {
#ifdef CONFIG_MFD_WM5102
	case WM5102:
		regmap_config = &wm5102_i2c_regmap;
		break;
#endif
#ifdef CONFIG_MFD_WM5110
	case WM5110:
		regmap_config = &wm5110_i2c_regmap;
		break;
#endif
	default:
		dev_err(&i2c->dev, "Unknown device type %ld\n",
			id->driver_data);
		return -EINVAL;
	}

	arizona = devm_kzalloc(&i2c->dev, sizeof(*arizona), GFP_KERNEL);
	if (arizona == NULL)
		return -ENOMEM;

	arizona->regmap = devm_regmap_init_i2c(i2c, regmap_config);
	if (IS_ERR(arizona->regmap)) {
		ret = PTR_ERR(arizona->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	arizona->type = id->driver_data;
	arizona->dev = &i2c->dev;
	//	arizona->irq = i2c->irq;
    arizona->irq = arizona_gpio_to_irq_i2c();

	return arizona_dev_init(arizona);
}

static int __devexit arizona_i2c_remove(struct i2c_client *i2c)
{
	struct arizona *arizona = dev_get_drvdata(&i2c->dev);
	arizona_dev_exit(arizona);
	return 0;
}

static const struct i2c_device_id arizona_i2c_id[] = {
	{ "wm5102", WM5102 },
	{ "wm5110", WM5110 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, arizona_i2c_id);

static struct i2c_driver arizona_i2c_driver = {
	.driver = {
		.name	= "arizona",
		.owner	= THIS_MODULE,
		.pm	= &arizona_pm_ops,
	},
	.probe		= arizona_i2c_probe,
	.remove		= __devexit_p(arizona_i2c_remove),
	.id_table	= arizona_i2c_id,
};

static __init int arizona_i2c_init(void)
{
	return i2c_register_driver(THIS_MODULE, &arizona_i2c_driver);
}
module_init(arizona_i2c_init);

static __exit void arizona_i2c_exit(void)
{
	return i2c_del_driver(&arizona_i2c_driver);
}
module_exit(arizona_i2c_exit);

MODULE_DESCRIPTION("Arizona I2C bus interface");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
