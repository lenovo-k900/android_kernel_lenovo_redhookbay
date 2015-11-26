/*
 * platform_btwilink.c: btwilink platform data initilization file
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
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/lnw_gpio.h>
#include <linux/ti_wilink_st.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>
#include <linux/wlan_plat.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mmc/sdhci.h>
#include "platform_brcm_bt_wifi.h"
#include "pci/platform_sdhci_pci.h"

int bt_enable_gpio,bt_reset_gpio;
#if 0
/* Shared transport callbacks */
static int st_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
static int st_resume(struct platform_device *pdev)
{
	return 0;
}

static int st_chip_enable(struct st_data_s *s)
{
	return 0;
}

static int st_chip_disable(struct st_data_s *s)
{
	return 0;
}

static int st_chip_awake(struct st_data_s *s)
{
	/* Tell PM runtime to power on the tty device and block S3 */
	if (!s->is_awake) {
		pm_runtime_get(s->tty_dev);
		wake_lock(&s->wake_lock);
		s->is_awake = 1;
	}

	return 0;
}

static int st_chip_asleep(struct st_data_s *s)
{
	/* Tell PM runtime to release tty device and allow S3 */
	if (s->is_awake) {
		pm_runtime_put(s->tty_dev);
		wake_unlock(&s->wake_lock);
		s->is_awake = 0;
	}

	return 0;
}

static struct ti_st_plat_data kim_pdata = {
	.nshutdown_gpio	= -1,/* BT, FM, GPS gpios */
	.flow_cntrl	= 1,		/* flow control flag */
	.suspend	= st_suspend,
	.resume		= st_resume,
	.chip_enable	= st_chip_enable,
	.chip_disable	= st_chip_disable,
	.chip_asleep	= st_chip_asleep,
	.chip_awake	= st_chip_awake,
};

static struct platform_device linux_kim_device = {
	.name           = "kim", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &kim_pdata,
};
#endif
/* BT WILINK related */
static int bt_enable(struct st_data_s *s)
{
	return 0;
}
static int bt_disable(struct st_data_s *s)
{
	return 0;
}

static struct ti_st_plat_data bt_pdata = {
	.chip_enable    = bt_enable,
	.chip_disable   = bt_disable,
};

static struct platform_device linux_bt_device = {
	.name           = "btwilink", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &bt_pdata,
};

#define BT_GPIO_RESET_NAME "BT_RST_N"
#define BT_GPIO_ENABLE_NAME "BT_EN" 

int bt_enable_gpio,bt_reset_gpio;

static int brcm_bluetooth_power(int on)
{
      printk("brcm_bluetooth_power on=%d  \n ",on);
      if(on){
//              msleep(500);
              gpio_set_value(bt_enable_gpio, 1); 
              gpio_set_value(bt_reset_gpio, 1); 
      } else {
             gpio_set_value(bt_enable_gpio, 0); 
             gpio_set_value(bt_reset_gpio, 0); 
           }
  
            return 0;
}

static struct platform_device bcm4330_rfkill = {
                .name = "bcm4330_rfkill", 
                .id = -1,
                .dev.platform_data = &brcm_bluetooth_power
};


static int __init bluetooth_init(void)
{
	unsigned int UART_index;
	long unsigned int UART_baud_rate;
	int error_reg,err;

	/* KIM INIT */
	/* Get the GPIO number from the SFI table
	   if FM gpio is not provided then BT-reset line is
	   also used to enable FM
	*/
    bt_enable_gpio = get_gpio_by_name(BT_GPIO_ENABLE_NAME);
	if (bt_enable_gpio == -1)
		return -ENODEV;
	gpio_set_value(bt_enable_gpio, 0);


    bt_reset_gpio= get_gpio_by_name(BT_GPIO_RESET_NAME);
    if (bt_reset_gpio == -1)
		return -ENODEV;
	gpio_set_value(bt_reset_gpio, 0);
    
    platform_device_register(&bcm4330_rfkill);
#if 0     
	/* Get Share Transport uart settings */
	/* TODO : add SFI table parsing and one SFI entry for this settings */
//	UART_index = UART_PORT_INDEX;
 //   UART_baudcrate = UART_BAUD_RATE;

	/* Share Transport uart settings */
 // 	sprintf((char *)kim_pdata.dev_name, "/dev/ttyMFD%u", UART_index);
 // 	kim_pdata.baud_rate = UART_baud_rate;

 // 	pr_info("%s: Setting platform_data with UART device name:%s and "
 // 			"UART baud rate:%lu.\n",
	 // 		__func__, kim_pdata.dev_name, kim_pdata.baud_rate);


	wl12xx_vwlan_gpio = get_gpio_by_name(WLAN_GPIO_ENABLE_NAME); 
	if (wl12xx_vwlan_gpio == -1) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return 0 ;
	}





/*Get GPIO numbers from the SFI table*/
	wifi_irq_gpio = get_gpio_by_name(WLAN_GPIO_IRQ_NAME);
	if (wifi_irq_gpio == -1) {
		pr_err("%s: Unable to find WLAN-interrupt GPIO in the SFI table\n",
				__func__);
		return 0;
	}
	err = gpio_request(wifi_irq_gpio, "wl12xx");
	if (err < 0) {
		pr_err("%s: Unable to request GPIO\n", __func__);
		return 0 ;
	}
	err = gpio_direction_input(wifi_irq_gpio);
	if (err < 0) {
		pr_err("%s: Unable to set GPIO direction\n", __func__);
		return 0;
	}
	mid_wifi_control_irq = gpio_to_irq(wifi_irq_gpio);
	pr_err("%s:Error gpio_to_irq:%d->%d\n", __func__, wifi_irq_gpio,
		       mid_wifi_control_irq);
		printk("%s:Error gpio_to_irq:%d->%d\n", __func__, wifi_irq_gpio,
		       mid_wifi_control_irq);
	if (mid_wifi_control_irq< 0) {
		pr_err("%s:Error gpio_to_irq:%d->%d\n", __func__, wifi_irq_gpio,
		       mid_wifi_control_irq);
		return 0;
	}
#endif

}

int wifi_on = 0;
EXPORT_SYMBOL(wifi_on);

static void *wifi_status_cb_devid = NULL;
static void (*wifi_status_cb) (void *dev_id, int card_present);

#define WLAN_GPIO_IRQ_NAME "WLAN_INT"
#define WLAN_GPIO_ENABLE_NAME "WLAN_EN"

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS    4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS    160
#define PREALLOC_WLAN_SECTION_HEADER    24

#define WLAN_SECTION_SIZE_0    (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1    (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2    (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3    (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM    16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *bcmdhd_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;

	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}

int __init bcmdhd_init_wifi_mem(void)
{
	int i = -1;

	for(i=0; ( i < WLAN_SKB_BUF_NUM ); i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}

	for(i=0; ( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS ); i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size, GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}

	return 0;
}

void sdhci_wifi_status_register(void* dev_id,void (*virtual_cd)
					(void *dev_id, int card_present))
{
	printk("%s: register status cb\n", __func__);
	wifi_status_cb_devid = dev_id;
	wifi_status_cb = virtual_cd;
	return;
}
EXPORT_SYMBOL(sdhci_wifi_status_register);

int bcmdhd_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);

	wifi_on = val;
	if (wifi_status_cb) {
		wifi_status_cb(wifi_status_cb_devid, val);
	} else {
		printk("%s: Nobody to notify\n", __func__);
	}
	return 0;
}

int bcmdhd_wifi_power(int on)
{
	int err = -1;
	int wlan_gpio = -1;

	printk("%s: %d\n", __func__, on);

	wlan_gpio = get_gpio_by_name(WLAN_GPIO_ENABLE_NAME);
	if (wlan_gpio == -1) {
		printk(KERN_ERR "%s: Unable to find WLAN-enable GPIO in the SFI table\n", __func__);
		return 0 ;
	}

	err = gpio_request(wlan_gpio, WLAN_GPIO_ENABLE_NAME);
	if (err < 0) {
		printk(KERN_ERR "%s: Unable to request GPIO[%d]\n", __func__, wlan_gpio);
	}
	if(on) {
		gpio_direction_output(wlan_gpio, 1);
	} else {
		gpio_direction_output(wlan_gpio, 0);
	}
	gpio_free(wlan_gpio);

	return 0;
}


int bcmdhd_wifi_reset(int on)
{
	printk("%s: do nothing\n", __func__);
	return 0;
}

static struct wifi_platform_data bcmdhd_wifi_control = {
	.set_power             = bcmdhd_wifi_power,
	.set_reset               = bcmdhd_wifi_reset,
	.set_carddetect  = bcmdhd_wifi_set_carddetect,
	.mem_prealloc     = bcmdhd_wifi_mem_prealloc,
};

static struct resource bcmdhd_wifi_resources[] = {
	[0] = {
		.name = "bcmdhd_wlan_irq",
		.flags   = IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING ,
	},
};

static struct platform_device bcmdhd_wifi_device = {
	.name                       = "bcmdhd_wlan",
	.id                               = 1,
	.num_resources  = ARRAY_SIZE(bcmdhd_wifi_resources),
	.resource                = bcmdhd_wifi_resources,
	.dev                           = {
		.platform_data = &bcmdhd_wifi_control,
        },
};

static int __init bcmdhd_wifi_init(void)
{
	int ret = -1;
	int wifi_irq_gpio = -1;
	unsigned int sdhci_quirk = SDHCI_QUIRK2_ADVERTISE_2V0_FORCE_1V8
			| SDHCI_QUIRK2_DISABLE_MMC_CAP_NONREMOVABLE
			| SDHCI_QUIRK2_ENABLE_MMC_PM_IGNORE_PM_NOTIFY;

	printk("%s: start\n", __func__);

	bcmdhd_init_wifi_mem();

	sdhci_pdata_set_quirks(sdhci_quirk);
	sdhci_pdata_set_embedded_control(&sdhci_wifi_status_register);

	/*Get GPIO numbers from the SFI table*/
	wifi_irq_gpio = get_gpio_by_name(WLAN_GPIO_IRQ_NAME);
	if (wifi_irq_gpio == -1) {
		printk(KERN_ERR "%s: Unable to find WLAN-interrupt GPIO in the SFI table\n", __func__);
		return 0;
	}
	bcmdhd_wifi_device.resource->start = gpio_to_irq(wifi_irq_gpio);
	bcmdhd_wifi_device.resource->end = gpio_to_irq(wifi_irq_gpio);

	ret = platform_device_register(&bcmdhd_wifi_device);
	return ret;
}

static int __init brcm_bt_wifi_init(void)
{
	printk("%s: start\n", __func__);
	bluetooth_init();
	bcmdhd_wifi_init();
}

device_initcall(brcm_bt_wifi_init);
