/*
 * Arizona core driver
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/registers.h>

#include "arizona.h"

static const char *wm5102_core_supplies[] = {
	"AVDD",
	"DBVDD1",
	"DCVDD",
};

static int wm5102_id = 0;
static int wm5102_id_parm_set(const char *val, struct kernel_param *kp)
{
    param_set_int(val, kp);
    return 0;
}
module_param_call(wm5102_id, wm5102_id_parm_set, param_get_int, &wm5102_id, S_IRUGO);

int arizona_clk32k_enable(struct arizona *arizona)
{
	int ret = 0;

	mutex_lock(&arizona->clk_lock);

	arizona->clk32k_ref++;

	if (arizona->clk32k_ref == 1)
		ret = regmap_update_bits(arizona->regmap, ARIZONA_CLOCK_32K_1,
					 ARIZONA_CLK_32K_ENA,
					 ARIZONA_CLK_32K_ENA);

	if (ret != 0)
		arizona->clk32k_ref--;

	mutex_unlock(&arizona->clk_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(arizona_clk32k_enable);

int arizona_clk32k_disable(struct arizona *arizona)
{
	int ret = 0;

	mutex_lock(&arizona->clk_lock);

	BUG_ON(arizona->clk32k_ref <= 0);

	arizona->clk32k_ref--;

	if (arizona->clk32k_ref == 0)
		regmap_update_bits(arizona->regmap, ARIZONA_CLOCK_32K_1,
				   ARIZONA_CLK_32K_ENA, 0);

	mutex_unlock(&arizona->clk_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(arizona_clk32k_disable);

static irqreturn_t arizona_clkgen_err(int irq, void *data)
{
	struct arizona *arizona = data;

	dev_err(arizona->dev, "CLKGEN error\n");

	return IRQ_HANDLED;
}

static irqreturn_t arizona_underclocked(int irq, void *data)
{
	struct arizona *arizona = data;
	unsigned int val;
	int ret;

	ret = regmap_read(arizona->regmap, ARIZONA_INTERRUPT_RAW_STATUS_8,
			  &val);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read underclock status: %d\n",
			ret);
		return IRQ_NONE;
	}

	if (val & ARIZONA_AIF3_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "AIF3 underclocked\n");
	if (val & ARIZONA_AIF3_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "AIF3 underclocked\n");
	if (val & ARIZONA_AIF2_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "AIF1 underclocked\n");
	if (val & ARIZONA_ISRC2_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "ISRC2 underclocked\n");
	if (val & ARIZONA_ISRC1_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "ISRC1 underclocked\n");
	if (val & ARIZONA_FX_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "FX underclocked\n");
	if (val & ARIZONA_ASRC_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "ASRC underclocked\n");
	if (val & ARIZONA_DAC_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "DAC underclocked\n");
	if (val & ARIZONA_ADC_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "ADC underclocked\n");
	if (val & ARIZONA_MIXER_UNDERCLOCKED_STS)
		dev_err(arizona->dev, "Mixer underclocked\n");

	return IRQ_HANDLED;
}

static irqreturn_t arizona_overclocked(int irq, void *data)
{
	struct arizona *arizona = data;
	unsigned int val[2];
	int ret;
	
	ret = regmap_bulk_read(arizona->regmap, ARIZONA_INTERRUPT_RAW_STATUS_6,
			       &val[0], 2);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read overclock status: %d\n",
			ret);
		return IRQ_NONE;
	}

	if (val[0] & ARIZONA_PWM_OVERCLOCKED_STS)
		dev_err(arizona->dev, "PWM overclocked\n");
	if (val[0] & ARIZONA_FX_CORE_OVERCLOCKED_STS)
		dev_err(arizona->dev, "FX core overclocked\n");
	if (val[0] & ARIZONA_DAC_SYS_OVERCLOCKED_STS)
		dev_err(arizona->dev, "DAC SYS overclocked\n");
	if (val[0] & ARIZONA_DAC_WARP_OVERCLOCKED_STS)
		dev_err(arizona->dev, "DAC WARP overclocked\n");
	if (val[0] & ARIZONA_ADC_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ADC overclocked\n");
	if (val[0] & ARIZONA_MIXER_OVERCLOCKED_STS)
		dev_err(arizona->dev, "Mixer overclocked\n");
	if (val[0] & ARIZONA_AIF3_SYNC_OVERCLOCKED_STS)
		dev_err(arizona->dev, "AIF3 overclocked\n");
	if (val[0] & ARIZONA_AIF2_SYNC_OVERCLOCKED_STS)
		dev_err(arizona->dev, "AIF2 overclocked\n");
	if (val[0] & ARIZONA_AIF1_SYNC_OVERCLOCKED_STS)
		dev_err(arizona->dev, "AIF1 overclocked\n");
	if (val[0] & ARIZONA_PAD_CTRL_OVERCLOCKED_STS)
		dev_err(arizona->dev, "Pad control overclocked\n");

	if (val[1] & ARIZONA_SLIMBUS_SUBSYS_OVERCLOCKED_STS)
		dev_err(arizona->dev, "Slimbus subsystem overclocked\n");
	if (val[1] & ARIZONA_SLIMBUS_ASYNC_OVERCLOCKED_STS)
		dev_err(arizona->dev, "Slimbus async overclocked\n");
	if (val[1] & ARIZONA_SLIMBUS_SYNC_OVERCLOCKED_STS)
		dev_err(arizona->dev, "Slimbus sync overclocked\n");
	if (val[1] & ARIZONA_ASRC_ASYNC_SYS_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ASRC async system overclocked\n");
	if (val[1] & ARIZONA_ASRC_ASYNC_WARP_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ASRC async WARP overclocked\n");
	if (val[1] & ARIZONA_ASRC_SYNC_SYS_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ASRC sync system overclocked\n");
	if (val[1] & ARIZONA_ASRC_SYNC_WARP_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ASRC sync WARP overclocked\n");
	if (val[1] & ARIZONA_ADSP2_1_OVERCLOCKED_STS)
		dev_err(arizona->dev, "DSP1 overclocked\n");
	if (val[1] & ARIZONA_ISRC2_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ISRC2 overclocked\n");
	if (val[1] & ARIZONA_ISRC1_OVERCLOCKED_STS)
		dev_err(arizona->dev, "ISRC1 overclocked\n");

	return IRQ_HANDLED;
}

static int arizona_wait_for_boot(struct arizona *arizona)
{
	unsigned int reg;
	int ret, i;

	/*
	 * We can't use an interrupt as we need to runtime resume to do so,
	 * we won't race with the interrupt handler as it'll be blocked on
	 * runtime resume.
	 */
	for (i = 0; i < 5; i++) {
		msleep(1);

		ret = regmap_read(arizona->regmap,
				  ARIZONA_INTERRUPT_RAW_STATUS_5, &reg);
		if (ret != 0) {
			dev_err(arizona->dev, "Failed to read boot state: %d\n",
				ret);
			return ret;
		}

		if (reg & ARIZONA_BOOT_DONE_STS)
			break;
	}

	if (reg & ARIZONA_BOOT_DONE_STS) {
		regmap_write(arizona->regmap, ARIZONA_INTERRUPT_STATUS_5,
			     ARIZONA_BOOT_DONE_STS);
	} else {
		dev_err(arizona->dev, "Device boot timed out: %x\n", reg);
		return -ETIMEDOUT;
	}

	pm_runtime_mark_last_busy(arizona->dev);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int arizona_runtime_resume(struct device *dev)
{
	struct arizona *arizona = dev_get_drvdata(dev);
	int ret;

	if (arizona->pdata.ldoena)
		gpio_set_value_cansleep(arizona->pdata.ldoena, 1);

       msleep(1);

	regcache_cache_only(arizona->regmap, false);

	ret = arizona_wait_for_boot(arizona);
	if (ret != 0)
		return ret;

	regcache_sync(arizona->regmap);

	return 0;
}

static int arizona_runtime_suspend(struct device *dev)
{
	struct arizona *arizona = dev_get_drvdata(dev);

	if (arizona->pdata.ldoena) {
		gpio_set_value_cansleep(arizona->pdata.ldoena, 0);
		regcache_cache_only(arizona->regmap, true);
		regcache_mark_dirty(arizona->regmap);
	}

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int arizona_suspend(struct device *dev)
{
    struct arizona *arizona = dev_get_drvdata(dev);

    printk(KERN_DEBUG "enter %s disable_depth = %d \n", __func__, dev->power.disable_depth);
//    dev_dbg(arizona->dev, "Suspend, disabling IRQ\n");
    disable_irq(arizona->irq);

    return 0;
}

static int arizona_resume(struct device *dev)
{
    struct arizona *arizona = dev_get_drvdata(dev);

    printk(KERN_DEBUG "enter %s disable_depth = %d \n", __func__, dev->power.disable_depth);
//    dev_dbg(arizona->dev, "Resume, reenabling IRQ\n");
    enable_irq(arizona->irq);

    return 0;
}
#endif

const struct dev_pm_ops arizona_pm_ops = {
	SET_RUNTIME_PM_OPS(arizona_runtime_suspend,
			   arizona_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(arizona_suspend, arizona_resume)
};
EXPORT_SYMBOL_GPL(arizona_pm_ops);

static struct mfd_cell early_devs[] = {
	{ .name = "arizona-ldo1" },
};

static struct mfd_cell wm5102_devs[] = {
	{ .name = "arizona-extcon" },
	{ .name = "arizona-gpio" },
	{ .name = "arizona-micsupp" },
	{ .name = "arizona-pwm" },
	{ .name = "wm5102-codec" },
};

static struct mfd_cell wm5110_devs[] = {
	{ .name = "arizona-extcon" },
	{ .name = "arizona-gpio" },
	{ .name = "arizona-micsupp" },
	{ .name = "arizona-pwm" },
	{ .name = "wm5110-codec" },
};

void arizona_core_interrupt_regist(void)
{
    int ret;

	/* Set up for interrupts */
	ret = arizona_irq_init(arizona_global);
	if (ret != 0)
		printk("enter: %s, line %d arizona_irq_init error\n", __func__, __LINE__);

	arizona_request_irq(arizona_global, ARIZONA_IRQ_CLKGEN_ERR, "CLKGEN error",
			    arizona_clkgen_err, arizona_global);
	arizona_request_irq(arizona_global, ARIZONA_IRQ_OVERCLOCKED, "Overclocked",
			    arizona_overclocked, arizona_global);
	arizona_request_irq(arizona_global, ARIZONA_IRQ_UNDERCLOCKED, "Underclocked",
			    arizona_underclocked, arizona_global);

    return;
}
EXPORT_SYMBOL_GPL(arizona_core_interrupt_regist);

int __devinit arizona_dev_init(struct arizona *arizona)
{
	struct device *dev = arizona->dev;
	const char *type_name;
	unsigned int reg, val;
	int ret, i;

	dev_set_drvdata(arizona->dev, arizona);
	mutex_init(&arizona->clk_lock);

	if (dev_get_platdata(arizona->dev))
		memcpy(&arizona->pdata, dev_get_platdata(arizona->dev),
		       sizeof(arizona->pdata));

	regcache_cache_only(arizona->regmap, true);

	switch (arizona->type) {
	case WM5102:
	case WM5110:
		for (i = 0; i < ARRAY_SIZE(wm5102_core_supplies); i++)
			arizona->core_supplies[i].supply
				= wm5102_core_supplies[i];
		arizona->num_core_supplies = ARRAY_SIZE(wm5102_core_supplies);
		break;
	default:
		dev_err(arizona->dev, "Unknown device type %d\n",
			arizona->type);
		return -EINVAL;
	}

	ret = mfd_add_devices(arizona->dev, -1, early_devs,
			      ARRAY_SIZE(early_devs), NULL, 0);
	if (ret != 0) {
		dev_err(dev, "Failed to add early children: %d\n", ret);
		return ret;
	}

#ifdef ARIZONA_REGULATOR
	ret = regulator_bulk_get(dev, arizona->num_core_supplies,
				      arizona->core_supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to request core supplies: %d\n",
			ret);
		goto err_early;
	}

	ret = regulator_bulk_enable(arizona->num_core_supplies,
				    arizona->core_supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to enable core supplies: %d\n",
			ret);
		goto err_early;
	}
#endif

	if (arizona->pdata.reset) {
		/* Start out with /RESET low to put the chip into reset */
		ret = gpio_request_one(arizona->pdata.reset,
				       GPIOF_DIR_OUT | GPIOF_INIT_LOW,
				       "arizona /RESET");
		if (ret != 0) {
			dev_err(dev, "Failed to request /RESET: %d\n", ret);
			goto err_enable;
		}

		gpio_set_value_cansleep(arizona->pdata.reset, 1);
	}

	if (arizona->pdata.ldoena) {
		ret = gpio_request_one(arizona->pdata.ldoena,
				       GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
				       "arizona LDOENA");
		if (ret != 0) {
			dev_err(dev, "Failed to request LDOENA: %d\n", ret);
			goto err_reset;
		}
		gpio_set_value_cansleep(arizona->pdata.ldoena, 1);

		msleep(1);
	}

	regcache_cache_only(arizona->regmap, false);

	ret = regmap_read(arizona->regmap, ARIZONA_SOFTWARE_RESET, &reg);
	if (ret != 0) {
		dev_err(dev, "Failed to read ID register: %d\n", ret);
		goto err_ldoena;
	}

	ret = regmap_read(arizona->regmap, ARIZONA_DEVICE_REVISION,
			  &arizona->rev);
	if (ret != 0) {
		dev_err(dev, "Failed to read revision register: %d\n", ret);
		goto err_ldoena;
	}
	arizona->rev &= ARIZONA_DEVICE_REVISION_MASK;

	switch (reg) {
#ifdef CONFIG_MFD_WM5102
	case 0x5102:
        wm5102_id = 5102;
        printk("enter %s, wm5102_id = 0x%x\n", __func__, reg);
		type_name = "WM5102";
		if (arizona->type != WM5102) {
			dev_err(arizona->dev, "WM5102 registered as %d\n",
				arizona->type);
			arizona->type = WM5102;
		}
		ret = wm5102_patch(arizona);
		break;
#endif
#ifdef CONFIG_MFD_WM5110
	case 0x5110:
		type_name = "WM5110";
		if (arizona->type != WM5110) {
			dev_err(arizona->dev, "WM5110 registered as %d\n",
				arizona->type);
			arizona->type = WM5110;
		}
		ret = wm5110_patch(arizona);
		break;
#endif
	default:
		dev_err(arizona->dev, "Unknown device ID %x\n", reg);
		goto err_ldoena;
	}

	dev_info(dev, "%s revision %c\n", type_name, arizona->rev + 'A');

	if (ret != 0)
		dev_err(arizona->dev, "Failed to apply patch: %d\n", ret);

	/* If we have a /RESET GPIO we'll already be reset */
	if (!arizona->pdata.reset) {
		ret = regmap_write(arizona->regmap, ARIZONA_SOFTWARE_RESET, 0);
		if (ret != 0) {
			dev_err(dev, "Failed to reset device: %d\n", ret);
			goto err_ldoena;
		}
	}

	ret = arizona_wait_for_boot(arizona);
	if (ret != 0) {
		dev_err(arizona->dev, "Device failed initial boot: %d\n", ret);
		goto err_reset;
	}

	for (i = 0; i < ARRAY_SIZE(arizona->pdata.gpio_defaults); i++) {
		if (!arizona->pdata.gpio_defaults[i])
			continue;

		regmap_write(arizona->regmap, ARIZONA_GPIO1_CTRL + i,
			     arizona->pdata.gpio_defaults[i]);
	}

	pm_runtime_set_autosuspend_delay(arizona->dev, 100);
	pm_runtime_use_autosuspend(arizona->dev);
	pm_runtime_enable(arizona->dev);

	/* Chip default */
	if (!arizona->pdata.clk32k_src)
		arizona->pdata.clk32k_src = ARIZONA_32KZ_MCLK2;

	switch (arizona->pdata.clk32k_src) {
	case ARIZONA_32KZ_MCLK1:
	case ARIZONA_32KZ_MCLK2:
		regmap_update_bits(arizona->regmap, ARIZONA_CLOCK_32K_1,
				   ARIZONA_CLK_32K_SRC_MASK,
				   arizona->pdata.clk32k_src - 1);
		break;
	case ARIZONA_32KZ_NONE:
		regmap_update_bits(arizona->regmap, ARIZONA_CLOCK_32K_1,
				   ARIZONA_CLK_32K_SRC_MASK, 2);
		break;
	default:
		dev_err(arizona->dev, "Invalid 32kHz clock source: %d\n",
			arizona->pdata.clk32k_src);
		ret = -EINVAL;
		goto err_ldoena;
	}

	for (i = 0; i < ARIZONA_MAX_INPUT; i++) {
		/* Default for both is 0 so noop with defaults */
		val = arizona->pdata.dmic_ref[i]
			<< ARIZONA_IN1_DMIC_SUP_SHIFT;
		val |= arizona->pdata.inmode[i] << ARIZONA_IN1_MODE_SHIFT;

		regmap_update_bits(arizona->regmap,
				   ARIZONA_IN1L_CONTROL + (i * 8),
				   ARIZONA_IN1_DMIC_SUP_MASK |
				   ARIZONA_IN1_MODE_MASK, val);
	}

	for (i = 0; i < ARIZONA_MAX_OUTPUT; i++) {
		/* Default is 0 so noop with defaults */
		if (arizona->pdata.out_mono[i])
			val = ARIZONA_OUT1_MONO;
		else
			val = 0;

		regmap_update_bits(arizona->regmap,
				   ARIZONA_OUTPUT_PATH_CONFIG_1L + (i * 8),
				   ARIZONA_OUT1_MONO, val);
	}

	for (i = 0; i < ARIZONA_MAX_PDM_SPK; i++) {
		if (arizona->pdata.spk_mute[i])
			regmap_update_bits(arizona->regmap,
					   ARIZONA_PDM_SPK1_CTRL_1 + (i * 2),
					   ARIZONA_SPK1_MUTE_ENDIAN_MASK |
					   ARIZONA_SPK1_MUTE_SEQ1_MASK,
					   arizona->pdata.spk_mute[i]);

		if (arizona->pdata.spk_fmt[i])
			regmap_update_bits(arizona->regmap,
					   ARIZONA_PDM_SPK1_CTRL_2 + (i * 2),
					   ARIZONA_SPK1_FMT_MASK,
					   arizona->pdata.spk_fmt[i]);
	}

/* the register of headset/headphone detection irq moved to k5_machine, 
 * because the k5_machine loaded later than this module, so there is a 
 * bug when the system powered up with headset/headphone plugin.*/
#if 0
	/* Set up for interrupts */
	ret = arizona_irq_init(arizona);
	if (ret != 0)
		goto err_ldoena;

	arizona_request_irq(arizona, ARIZONA_IRQ_CLKGEN_ERR, "CLKGEN error",
			    arizona_clkgen_err, arizona);
	arizona_request_irq(arizona, ARIZONA_IRQ_OVERCLOCKED, "Overclocked",
			    arizona_overclocked, arizona);
	arizona_request_irq(arizona, ARIZONA_IRQ_UNDERCLOCKED, "Underclocked",
			    arizona_underclocked, arizona);
#endif

	switch (arizona->type) {
	case WM5102:
		ret = mfd_add_devices(arizona->dev, -1, wm5102_devs,
				      ARRAY_SIZE(wm5102_devs), NULL, 0);
		break;
	case WM5110:
		ret = mfd_add_devices(arizona->dev, -1, wm5110_devs,
				      ARRAY_SIZE(wm5102_devs), NULL, 0);
		break;
	}

	if (ret != 0) {
		dev_err(arizona->dev, "Failed to add subdevices: %d\n", ret);
		goto err_irq;
	}
	printk(KERN_DEBUG "###enter %s, in line %d\n", __func__, __LINE__);

	return 0;

err_irq:
#if 0
	arizona_irq_exit(arizona);
#endif
err_ldoena:
	if (arizona->pdata.ldoena) {
		gpio_set_value_cansleep(arizona->pdata.ldoena, 0);
		gpio_free(arizona->pdata.ldoena);
	}
err_reset:
	if (arizona->pdata.reset) {
		gpio_set_value_cansleep(arizona->pdata.reset, 1);
		gpio_free(arizona->pdata.reset);
	}
err_enable:
#ifdef ARIZONA_REGULATOR
	regulator_bulk_disable(ARRAY_SIZE(arizona->core_supplies),
			       arizona->core_supplies);
#endif
err_early:
#ifdef ARIZONA_REGULATOR
	mfd_remove_devices(dev);
#endif
	return ret;
}
EXPORT_SYMBOL_GPL(arizona_dev_init);

int __devexit arizona_dev_exit(struct arizona *arizona)
{
	mfd_remove_devices(arizona->dev);
	arizona_free_irq(arizona, ARIZONA_IRQ_UNDERCLOCKED, arizona);
	arizona_free_irq(arizona, ARIZONA_IRQ_OVERCLOCKED, arizona);
	arizona_free_irq(arizona, ARIZONA_IRQ_CLKGEN_ERR, arizona);
	pm_runtime_disable(arizona->dev);
	arizona_irq_exit(arizona);
	return 0;
}
EXPORT_SYMBOL_GPL(arizona_dev_exit);
