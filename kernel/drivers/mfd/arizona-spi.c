/*
 * arizona-spi.c  --  Arizona SPI bus interface
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
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/mfd/arizona/core.h>

#include "arizona.h"
#include <linux/gpio.h>

struct arizona *arizona_global;

#define GPIO_ARIZONA_SPI  (32)
int arizona_gpio_to_irq_spi()
{
    int ret;

    ret = gpio_request(GPIO_ARIZONA_SPI, "arizona_irq");
    if (ret) {
        printk("###enter %s, in line %d\n", __func__, __LINE__);
    }
    gpio_direction_input(GPIO_ARIZONA_SPI);
    ret = gpio_to_irq(GPIO_ARIZONA_SPI);
    printk(KERN_DEBUG "###enter %s, in line %d, wm5102.irq = %d\n", __func__, __LINE__, ret);

    return ret;
}

static int __devinit arizona_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct arizona *arizona;
	const struct regmap_config *regmap_config;
	int ret;

	switch (id->driver_data) {
#ifdef CONFIG_MFD_WM5102
	case WM5102:
		regmap_config = &wm5102_spi_regmap;
		break;
#endif
#ifdef CONFIG_MFD_WM5110
	case WM5110:
		regmap_config = &wm5110_spi_regmap;
		break;
#endif
	default:
		dev_err(&spi->dev, "Unknown device type %ld\n",
			id->driver_data);
		return -EINVAL;
	}

	arizona = devm_kzalloc(&spi->dev, sizeof(*arizona), GFP_KERNEL);
	if (arizona == NULL)
		return -ENOMEM;

    //spi->bits_per_word = 16;
    printk("%s,spi work in 8-bit mode\n",__func__);
    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0;
    spi_setup(spi);

	arizona->regmap = devm_regmap_init_spi(spi, regmap_config);
	if (IS_ERR(arizona->regmap)) {
		ret = PTR_ERR(arizona->regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	arizona->type = id->driver_data;
	arizona->dev = &spi->dev;
//	arizona->irq = spi->irq;
	arizona->irq = arizona_gpio_to_irq_spi();
    arizona_global = arizona;

    printk("enter %s, in line %d\n", __func__, __LINE__);
	return arizona_dev_init(arizona);
}

static int __devexit arizona_spi_remove(struct spi_device *spi)
{
	struct arizona *arizona = dev_get_drvdata(&spi->dev);
	arizona_dev_exit(arizona);
	return 0;
}

static const struct spi_device_id arizona_spi_ids[] = {
	{ "wm5102", WM5102 },
	{ "wm5110", WM5110 },
	{ },
};
MODULE_DEVICE_TABLE(spi, arizona_spi_ids);

static struct spi_driver arizona_spi_driver = {
	.driver = {
		.name	= "arizona",
		.owner	= THIS_MODULE,
		.pm	= &arizona_pm_ops,
	},
	.probe		= arizona_spi_probe,
	.remove		= __devexit_p(arizona_spi_remove),
	.id_table	= arizona_spi_ids,
};

static __init int arizona_spi_init(void)
{
	return spi_register_driver(&arizona_spi_driver);
}
module_init(arizona_spi_init);

static __exit void arizona_spi_exit(void)
{
	return spi_unregister_driver(&arizona_spi_driver);
}
module_exit(arizona_spi_exit);

MODULE_DESCRIPTION("Arizona SPI bus interface");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
