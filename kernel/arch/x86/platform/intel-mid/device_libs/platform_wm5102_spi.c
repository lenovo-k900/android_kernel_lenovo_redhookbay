/*
 * platform_wm5102_spi.c: wm5102 platform data initilization file
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
#include <linux/spi/spi.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>

#include <linux/mfd/arizona/pdata.h>
#include <linux/spi/spi-dw.h>
#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>

extern struct dw_spi_chip;
static struct dw_spi_chip spi_device_wm5102; 
extern void lnw_gpio_set_alt(int gpio, int alt);

// spi chip select is low active
#define GPIO_AON_18 18  //cs
#define GPIO_AON_23 23  //clk
#define GPIO_AON_22 22  //sdi
#define GPIO_AON_21 21  //sdo

static void spi_1_ss2_toggle(u32 command)
{
   u32 value = 0;

   lnw_gpio_set_alt(GPIO_AON_18, 0);
     
	switch (command){
	case SPI_DW_DEASSERT:
                value = 1;
		break;
	case SPI_DW_ASSERT:
                value = 0;
		break;
	default:
		printk("%s-command is invalid=%d\n",__func__, command);
		break;
    }
//    printk("%s--value=%d\n", __func__, value);
    gpio_set_value(GPIO_AON_18, value);

}

static const struct arizona_micd_config micd_modes[] = {
        { 0, 1 << ARIZONA_MICD_BIAS_SRC_SHIFT, 0 },
};

static const struct arizona_pdata wm5102_pdata = {
	.irq_base = 512,
	.micd_configs = micd_modes,
	.num_micd_configs = ARRAY_SIZE(micd_modes),
	.ldoena = 41,
	.out_mono[1] = true, //HPOUT2 -> OUT2
	.inmode = {
		[0] = ARIZONA_INMODE_DIFF, /* IN1 is differential for headset */
		[1] = ARIZONA_INMODE_SE,   /* IN2 is single ended for FM */
		[2] = ARIZONA_INMODE_DIFF, /* IN3 is differential for onboard mics */
	},
};


void __init *wm5102_spi_platform_data(void *info)
{
    struct spi_board_info *chip = info;

    memset(&spi_device_wm5102, 0, sizeof(spi_device_wm5102));

    spi_device_wm5102.poll_mode  = 0;
    spi_device_wm5102.tmode  = SPI_TMOD_TR;
    spi_device_wm5102.enable_dma = 0;
    spi_device_wm5102.cs_control = spi_1_ss2_toggle;
   
    chip->controller_data = (void *)&spi_device_wm5102;   
    chip->platform_data = &wm5102_pdata;
    chip->max_speed_hz = 25000000;   

    printk("%s, spi_1 work at 25MHz\n",__func__);
    return (void *)&wm5102_pdata;
}
