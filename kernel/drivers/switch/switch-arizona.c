/*
 * switch-arizona.c - Switch driver Wolfson Arizona devices
 *
 *  Copyright (C) 2012 Wolfson Microelectronics plc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>

#define INTEL_VIBRA_MAX_TIMEDIVISOR  0xFF
#define INTEL_VIBRA_MAX_BASEUNIT 0x80

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

struct arizona_switch_info {
	struct device *dev;
	struct arizona *arizona;
	struct mutex lock;
	struct regulator *micvdd;

	int enabled;
	int micd_mode;
	const struct arizona_micd_config *micd_modes;
	int micd_num_modes;

	bool micd_reva;
	const char	*name;

	bool mic;
	bool detecting;
	bool headset_flag;
	bool headphone_flag;
	bool hook_flag;
	union sst_pwmctrl_reg   pwm;
	int jack_flips;

	struct switch_dev sdev;
    struct work_struct headset_work;
    struct wake_lock headset_wakelock;
    struct wake_lock hook_wakelock;
};

#define UART_AUDIOJACK_SWITCH_PIN_STAT	(64+96)

/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static const struct arizona_micd_config micd_default_modes[] = {
	{ ARIZONA_ACCDET_SRC, 1 << ARIZONA_MICD_BIAS_SRC_SHIFT, 0 },
	{ 0,                  2 << ARIZONA_MICD_BIAS_SRC_SHIFT, 1 },
};

extern struct input_dev *inp_dev;
extern struct snd_soc_jack hs_jack;
extern struct arizona *arizona_global;

extern void resume_set_sysclk(void);
extern void suspend_set_sysclk(void);

bool micvdd_enable_flag;
bool mic_ev_enabled;
struct regulator *micvdd_swit;

static bool headset_plugin_flag;
static struct arizona_switch_info *info_swit;

static ssize_t headset_print_name(struct switch_dev *sdev, char *buf)
{
    switch (switch_get_state(sdev)) {
        case 0:
            return sprintf(buf, "No Device\n");
        case 1:
            return sprintf(buf, "Headset\n");
        case 2:
            return sprintf(buf, "Headphone\n");
    }
    return -EINVAL;
}

/* add for testmode to enable and disable wm5102 module */
#define GPIO_CODEC_LDO_EN (41)
static int wm5102_enable = 0;
static void audio_codec_ldo_en(int enable)
{
    int ret;
    unsigned int val;

    gpio_free(GPIO_CODEC_LDO_EN);
    ret = gpio_request(GPIO_CODEC_LDO_EN, "audio_codec_ldo_en");
    if (ret) {
        printk("enter %s, in line %d\n", __func__, __LINE__);
    }

    if(!!enable) {
        gpio_direction_output(GPIO_CODEC_LDO_EN, 1);

        regmap_read(arizona_global->regmap, ARIZONA_LDO1_CONTROL_1, &val);
        regmap_write(arizona_global->regmap, ARIZONA_LDO1_CONTROL_1, val | 0x0001);

        regmap_read(arizona_global->regmap, ARIZONA_FLL1_CONTROL_1, &val);
        regmap_write(arizona_global->regmap, ARIZONA_FLL1_CONTROL_1, val | 0x0001);
        printk("enter %s, in line %d, enable OCS_CLK\n", __func__, __LINE__);
    } else {
        gpio_direction_output(GPIO_CODEC_LDO_EN, 0);

        regmap_read(arizona_global->regmap, ARIZONA_LDO1_CONTROL_1, &val);
        regmap_write(arizona_global->regmap, ARIZONA_LDO1_CONTROL_1, val & 0xfffe);

        regmap_read(arizona_global->regmap, ARIZONA_FLL1_CONTROL_1, &val);
        regmap_write(arizona_global->regmap, ARIZONA_FLL1_CONTROL_1, val & 0xfffe);
        printk("enter %s, in line %d, disable OCS_CLK\n", __func__, __LINE__);
    }

    return;
}

static int wm5102_enable_parm_set(const char *val, struct kernel_param *kp)
{
    param_set_int(val, kp);

    if (wm5102_enable == 1) {
        printk("enter: %s, enable wm5102 module\n", __func__);
        audio_codec_ldo_en(1);
    } else {
        printk("enter: %s, disable wm5102 module\n", __func__);
        audio_codec_ldo_en(0);
    }
    return 0;
}
module_param_call(wm5102_enable, wm5102_enable_parm_set, param_get_int,
        &wm5102_enable, S_IWUSR | S_IRUGO);
/*--------------------------------end--------------------------------*/

static void arizona_switch_set_mode(struct arizona_switch_info *info, int mode)
{
	struct arizona *arizona = info->arizona;

	gpio_set_value_cansleep(arizona->pdata.micd_pol_gpio,
				info->micd_modes[mode].gpio);
	regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_1,
			   ARIZONA_MICD_BIAS_SRC_MASK,
			   info->micd_modes[mode].bias);
	regmap_update_bits(arizona->regmap, ARIZONA_ACCESSORY_DETECT_MODE_1,
			   ARIZONA_ACCDET_SRC, info->micd_modes[mode].src);

	info->micd_mode = mode;

	dev_dbg(arizona->dev, "Set jack polarity to %d\n", mode);
}

static void arizona_start_mic(struct arizona_switch_info *info)
{
	struct arizona *arizona = info->arizona;
	bool change;
	int ret;

    if (!micvdd_enable_flag) {
        /* Turn on MICVDD for microphone detecting */
        ret = regulator_enable(info->micvdd);
        if (ret != 0) {
            dev_err(arizona->dev, "Failed to enable MICVDD: %d\n", ret);
        }
        micvdd_enable_flag = true;
    }
	regmap_update_bits(arizona->regmap, ARIZONA_MIC_BIAS_CTRL_1, ARIZONA_MICB1_ENA_MASK, ARIZONA_MICB1_ENA);

	info->detecting = true;
	info->mic = false;
	info->jack_flips = 0;

	/* Microphone detection can't use idle mode */
	pm_runtime_get(info->dev);

	if (info->micd_reva) {
		regmap_write(arizona->regmap, 0x80, 0x3);
		regmap_write(arizona->regmap, 0x294, 0);
		regmap_write(arizona->regmap, 0x80, 0x0);
	}

	regmap_update_bits_check(arizona->regmap, ARIZONA_MIC_DETECT_1,
				 ARIZONA_MICD_ENA, ARIZONA_MICD_ENA,
				 &change);
	if (!change) {
        if (micvdd_enable_flag && (!mic_ev_enabled)) {
            regulator_disable(info->micvdd);
            micvdd_enable_flag = false;
        }
		pm_runtime_put_autosuspend(info->dev);
	}
}

static void arizona_stop_mic(struct arizona_switch_info *info)
{
	struct arizona *arizona = info->arizona;
	bool change;

	regmap_update_bits_check(arizona->regmap, ARIZONA_MIC_DETECT_1,
				 ARIZONA_MICD_ENA, 0,
				 &change);

	regmap_update_bits(arizona->regmap, ARIZONA_MIC_BIAS_CTRL_1, ARIZONA_MICB1_ENA_MASK, 0);
	if (info->micd_reva) {
		regmap_write(arizona->regmap, 0x80, 0x3);
		regmap_write(arizona->regmap, 0x294, 2);
		regmap_write(arizona->regmap, 0x80, 0x0);
	}

	if (change) {
        if (micvdd_enable_flag && (!mic_ev_enabled)) {
            regulator_disable(info->micvdd);
            micvdd_enable_flag = false;
        }
		pm_runtime_put_autosuspend(info->dev);
	}
}

static irqreturn_t arizona_micdet(int irq, void *data)
{
	struct arizona_switch_info *info = data;
	struct arizona *arizona = info->arizona;
	unsigned int val;
	int ret;

	mutex_lock(&info->lock);

//	regmap_update_bits(arizona->regmap, ARIZONA_MIC_BIAS_CTRL_1, ARIZONA_MICB1_EXT_CAP_MASK, ARIZONA_MICB1_EXT_CAP);

	ret = regmap_read(arizona->regmap, ARIZONA_MIC_DETECT_3, &val);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read MICDET: %d\n", ret);
		return IRQ_NONE;
	}

    if (info->detecting && (val & 0x04) && (val & 0x01)) {
        ret = regmap_read(arizona->regmap, ARIZONA_MIC_DETECT_3, &val);
//        printk("enter: %s, val = 0x%x----------------------------------\n", __func__, val);
        if (ret != 0) {
            dev_err(arizona->dev, "Failed to read MICDET: %d\n", ret);
            return IRQ_NONE;
        }
    }

    printk(KERN_DEBUG "enter: %s=== status_val = 0x%x\n", __func__, val);

	if (!(val & ARIZONA_MICD_VALID)) {
		dev_warn(arizona->dev, "Microphone detection state invalid\n");
		mutex_unlock(&info->lock);
		return IRQ_NONE;
	}

	/* Due to jack detect this should never happen */
	if (!(val & ARIZONA_MICD_STS)) {
        printk(KERN_DEBUG "enter: %s, Detected open circuit, error==\n", __func__);
//		info->detecting = false;
		goto handled;
	}

	/* If we got a high impedence we should have a headset, report it. */
	if (info->detecting && (val & 0x400)) {

       // printk("enter: %s, detected headset and switch state==\n", __func__);
        printk(KERN_DEBUG "enter: %s-----headset in----------\n", __func__);
        snd_soc_jack_report(&hs_jack, SND_JACK_HEADSET, SND_JACK_HEADSET);
//		switch_set_state(&info->sdev, BIT_HEADSET);
//        input_report_switch(inp_dev, SW_MICROPHONE_INSERT, 1);

		info->mic = true;
		info->detecting = false;
	    info->headset_flag = true;
	    info->headphone_flag = false;
        info->hook_flag = false;
		goto handled;
	}

	/* If we detected a lower impedence during initial startup
	 * then we probably have the wrong polarity, flip it.  Don't
	 * do this for the lowest impedences to speed up detection of
	 * plain headphones.  If both polarities report a low
	 * impedence then give up and report headphones.
	 */
	if (info->detecting && (val & 0x3f8)) {
		info->jack_flips++;

		if (info->jack_flips >= info->micd_num_modes) {
            //printk(KERN_DEBUG "enter: %s, Detected headphone==\n", __func__);
            printk(KERN_DEBUG "enter: %s-----in headphone----------\n", __func__);
			info->detecting = false;

            snd_soc_jack_report(&hs_jack, SND_JACK_HEADPHONE, SND_JACK_HEADPHONE);
//            switch_set_state(&info->sdev, BIT_HEADSET_NO_MIC);
//            input_report_switch(inp_dev, SW_HEADPHONE_INSERT, 1);
	        info->headphone_flag = true;
	        info->headset_flag = false;

		} else {
            printk(KERN_DEBUG "enter: %s, line %d\n", __func__, __LINE__);
			info->micd_mode++;
			if (info->micd_mode == info->micd_num_modes)
				info->micd_mode = 0;
			arizona_switch_set_mode(info, info->micd_mode);

			info->jack_flips++;
		}

		goto handled;
	}

	/*
	 * If we're still detecting and we detect a short then we've
	 * got a headphone.  Otherwise it's a button press, the
	 * button reporting is stubbed out for now.
	 */
	if (val & 0x3fc) {
		if (info->mic) {

            if (wake_lock_active(&info->hook_wakelock))
                wake_unlock(&info->hook_wakelock);
            wake_lock_timeout(&info->hook_wakelock, 2 * HZ);

            if (info->hook_flag) {
                printk(KERN_DEBUG "enter: %s, Mic button has been reported\n", __func__);
            } else {
                printk(KERN_DEBUG "enter: %s, Mic button pressed==\n", __func__);
                input_report_key(inp_dev, KEY_MEDIA, 1);
                input_sync(inp_dev);
                info->hook_flag = true;
            }

		} else if (info->detecting) {
            //printk("enter: %s, Headphone detected==\n", __func__);
            printk(KERN_DEBUG "enter: %s-----headphone in----------\n", __func__);
			info->detecting = false;
			arizona_stop_mic(info);

            snd_soc_jack_report(&hs_jack, SND_JACK_HEADPHONE, SND_JACK_HEADPHONE);
//			switch_set_state(&info->sdev, BIT_HEADSET_NO_MIC);
//            input_report_switch(inp_dev, SW_HEADPHONE_INSERT, 1);
	        info->headphone_flag = true;
	        info->headset_flag = false;

		} else {
            printk(KERN_DEBUG "enter: %s, Button with no mic==\n", __func__);
		}
	} else {
        if (info->hook_flag) {
            printk(KERN_DEBUG "enter: %s,  Mic button released==\n", __func__);
            input_report_key(inp_dev, KEY_MEDIA, 0);
            input_sync(inp_dev);
        }
        info->hook_flag = false;
	}

handled:
	pm_runtime_mark_last_busy(info->dev);
	mutex_unlock(&info->lock);

	return IRQ_HANDLED;
}

static void headset_detection_work(struct work_struct *work)
{
    struct arizona *arizona = info_swit->arizona;
    unsigned int val;
    int ret;

    pm_runtime_get_sync(info_swit->dev);

    mutex_lock(&info_swit->lock);

    ret = regmap_read(arizona->regmap, ARIZONA_AOD_IRQ_RAW_STATUS, &val);
    if (ret != 0) {
        dev_err(arizona->dev, "Failed to read jackdet status: %d\n", ret);
        mutex_unlock(&info_swit->lock);
        pm_runtime_put_autosuspend(info_swit->dev);
        //		return IRQ_NONE;
        return;
    }

    if (!gpio_get_value(UART_AUDIOJACK_SWITCH_PIN_STAT)) {
        if (val & ARIZONA_JD1_STS) {
            printk(KERN_DEBUG "enter: %s, plugin\n", __func__);
            msleep(300);
            arizona_start_mic(info_swit);
        } else {
//            printk(KERN_DEBUG "enter: %s, pull out\n", __func__);

            if (info_swit->headset_flag) {
                printk(KERN_DEBUG "enter: %s-----headset out-----\n", __func__);
//                switch_set_state(&info_swit->sdev, BIT_NO_HEADSET);
                arizona_stop_mic(info_swit);
                snd_soc_jack_report(&hs_jack, 0, SND_JACK_HEADSET);
//                input_report_switch(inp_dev, SW_MICROPHONE_INSERT, 0);
                info_swit->headset_flag = false;
            } else if (info_swit->headphone_flag) {
                printk(KERN_DEBUG "enter: %s-----headphone out-----\n", __func__);
//                switch_set_state(&info_swit->sdev, BIT_NO_HEADSET);
                snd_soc_jack_report(&hs_jack, 0, SND_JACK_HEADPHONE);
//                input_report_switch(inp_dev, SW_HEADPHONE_INSERT, 0);
                info_swit->headphone_flag = false;
            } else {
                // do nothing 
                printk(KERN_DEBUG "enter: %s, --out headset/headphone error--\n", __func__);
//                arizona_stop_mic(info_swit);
            }
        }
    }

    mutex_unlock(&info_swit->lock);

    pm_runtime_mark_last_busy(info_swit->dev);
    pm_runtime_put_autosuspend(info_swit->dev);

    return;
}

static irqreturn_t arizona_jackdet(int irq, void *data)
{
	struct arizona_switch_info *info = data;
/*	struct arizona *arizona = info->arizona;
	unsigned int val;
	int ret;

    if (!headset_plugin_flag) {
        printk(KERN_DEBUG "enter: %s, the invalid interrupt when powerup!!!!!\n", __func__);
        headset_plugin_flag = true;
        return IRQ_HANDLED;
    }

	pm_runtime_get_sync(info->dev);

	mutex_lock(&info->lock);

	ret = regmap_read(arizona->regmap, ARIZONA_AOD_IRQ_RAW_STATUS, &val);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read jackdet status: %d\n", ret);
		mutex_unlock(&info->lock);
		pm_runtime_put_autosuspend(info->dev);
		return IRQ_NONE;
	}

    if (!gpio_get_value(UART_AUDIOJACK_SWITCH_PIN_STAT)) {
        if (val & ARIZONA_JD1_STS) {
            printk(KERN_DEBUG "enter: %s, plugin\n", __func__);
            arizona_start_mic(info);
        } else {
            switch_set_state(&info->sdev, BIT_NO_HEADSET);

            if (info->headset_flag) {
                printk(KERN_DEBUG "enter: %s-----headset out-----\n", __func__);
                arizona_stop_mic(info);
                snd_soc_jack_report(&hs_jack, 0, SND_JACK_HEADSET);
                input_report_switch(inp_dev, SW_MICROPHONE_INSERT, 0);
                info->headset_flag = false;
            } else if (info->headphone_flag) {
                printk(KERN_DEBUG "enter: %s-----headphone out-----\n", __func__);
                snd_soc_jack_report(&hs_jack, 0, SND_JACK_HEADPHONE);
                input_report_switch(inp_dev, SW_HEADPHONE_INSERT, 0);
                info->headphone_flag = false;
            } else
                printk(KERN_DEBUG "enter: %s, --out headset/headphone error--\n", __func__);
        }
    }

	mutex_unlock(&info->lock);

	pm_runtime_mark_last_busy(info->dev);
	pm_runtime_put_autosuspend(info->dev);
*/
    printk(KERN_DEBUG "enter: %s\n", __func__);
    if (!headset_plugin_flag) {
        printk(KERN_DEBUG "enter: %s, the invalid interrupt when powerup!!!!!\n", __func__);
        headset_plugin_flag = true;
        return IRQ_HANDLED;
    }

    if (wake_lock_active(&info->headset_wakelock))
        wake_unlock(&info->headset_wakelock);
    wake_lock_timeout(&info->headset_wakelock, 4 * HZ);

    schedule_work(&info->headset_work);

	return IRQ_HANDLED;
}
 
/* Enable's vibra driver */
#define HAP_CTRL_CONTINUOUS 0x01
#define HAP_CTRL_ONESHOT 0x02
#define HAP_ACT_ERM 0x0000
#define HAP_ACT_LRA 0x0001
#define HAP_ONESHOT_TRIG (1 << 4)

static void vibra_enable(struct arizona_switch_info *info)
{
    unsigned int val;

    // /sys/devices/pci0000:00/0000:00:00.2/spi_master/spi0/spi0.2/arizona-extcon/vibrator
    // /sys/kernel/debug/regmap/spi0.2
    printk("enter:[%s][%d]\n", __func__, __LINE__);
    pm_runtime_get(info->dev);
    resume_set_sysclk();
    mutex_lock(&info->lock);
    info->enabled = true;

    // Enable VIB, check if clk enable
    regmap_read(info->arizona->regmap, ARIZONA_CLOCK_32K_1, &val);
    if ( !(val & ARIZONA_CLK_32K_ENA))
    {
    	regmap_update_bits(info->arizona->regmap, ARIZONA_CLOCK_32K_1, 
			ARIZONA_CLK_32K_ENA_MASK | ARIZONA_CLK_32K_SRC_MASK, 
			ARIZONA_CLK_32K_ENA | 0x01);
    }
    regmap_read(info->arizona->regmap, ARIZONA_SYSTEM_CLOCK_1, &val);
    if ( !(val & ARIZONA_SYSCLK_ENA))
    {
    	regmap_update_bits(info->arizona->regmap, ARIZONA_SYSTEM_CLOCK_1, 
			ARIZONA_SYSCLK_FREQ_MASK | ARIZONA_SYSCLK_ENA_MASK | ARIZONA_SYSCLK_SRC_MASK, 
			0x0300 | ARIZONA_SYSCLK_ENA | 0x0004);
    }
    regmap_write(info->arizona->regmap, ARIZONA_HAPTICS_CONTROL_1, 0x4);
    regmap_write(info->arizona->regmap, ARIZONA_HAPTICS_PHASE_2_DURATION, 0xC5);
    //Enable SPKOUTR
    regmap_write(info->arizona->regmap, ARIZONA_OUT4RMIX_INPUT_1_SOURCE, 0x06);
    //SPKR on
    regmap_update_bits(info->arizona->regmap, ARIZONA_OUTPUT_ENABLES_1, ARIZONA_OUT4R_ENA_MASK, ARIZONA_OUT4R_ENA);
    regmap_write(info->arizona->regmap, ARIZONA_DAC_DIGITAL_VOLUME_4R, 0x280); // toggle VU bit to latch mute
    regmap_write(info->arizona->regmap, ARIZONA_HAPTICS_PHASE_2_INTENSITY, 0x50);

    mutex_unlock(&info->lock);

}

static void vibra_disable(struct arizona_switch_info *info)
{
    printk("enter:[%s][%d]\n", __func__, __LINE__);
    mutex_lock(&info->lock);
    info->enabled = false;

    // Disable VIB
    regmap_write(info->arizona->regmap, ARIZONA_HAPTICS_CONTROL_1, 0);
    regmap_write(info->arizona->regmap, ARIZONA_HAPTICS_PHASE_2_INTENSITY, 0x19);
    regmap_write(info->arizona->regmap, ARIZONA_HAPTICS_PHASE_2_DURATION, 0xC5);
    regmap_write(info->arizona->regmap, ARIZONA_OUT4RMIX_INPUT_1_SOURCE, 0x0);
    regmap_update_bits(info->arizona->regmap, ARIZONA_OUTPUT_ENABLES_1, ARIZONA_OUT4R_ENA_MASK, !ARIZONA_OUT4R_ENA);

    mutex_unlock(&info->lock);

    suspend_set_sysclk();
//    pm_runtime_put(info->dev);
    pm_runtime_put_autosuspend(info->dev);
}

/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/

static ssize_t vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct arizona_switch_info *info = dev_get_drvdata(dev);
	printk("enter:[%s][%d]\n", __func__, __LINE__);

	return sprintf(buf, "%d\n", info->enabled);

}

static ssize_t vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;
	struct arizona_switch_info *info = dev_get_drvdata(dev);

	if (kstrtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	printk("enter:[%s][%d]vibrator_enable = %d\n", __func__, __LINE__, vibrator_enable);
//	if (vibrator_enable == info->enabled)
//		return len;
	if (vibrator_enable == 0)
		vibra_disable(info);
	else if (vibrator_enable == 1)
		vibra_enable(info);
	else
		return -EINVAL;
	return len;
}

static ssize_t vibra_set_pwm_baseunit(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long  pwm_base;
	struct arizona_switch_info *info = dev_get_drvdata(dev);
	printk("enter:[%s][%d]\n", __func__, __LINE__);
//TODO change Freq of VIB
	if (kstrtoul(buf, 0, &pwm_base))
		return -EINVAL;

	pr_debug("PWM value 0x%lx\n", pwm_base);
	pwm_base = abs(pwm_base);
	printk("enter:[%s][%d], pwm_base = 0x%lx\n", __func__, __LINE__, pwm_base);

	if (pwm_base < 0 || pwm_base > INTEL_VIBRA_MAX_BASEUNIT) {
		pr_err("Supported value is out of Range\n");
		return -EINVAL;
	}
	pr_debug("PWM value 0x%lx\n", pwm_base);
	info->pwm.part.pwmbu = pwm_base;
	return len;
}

static ssize_t vibra_show_pwm_baseunit(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct arizona_switch_info *info = dev_get_drvdata(dev);
	printk("enter:[%s][%d]\n", __func__, __LINE__);

	return sprintf(buf, "0x%X\n", info->pwm.part.pwmbu);
}

static ssize_t vibra_set_pwm_ontime_div(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long  pwm_td;
	struct arizona_switch_info *info = dev_get_drvdata(dev);
	printk("enter:[%s][%d]\n", __func__, __LINE__);
//TODO change intensity of VIB
	if (kstrtoul(buf, 0, &pwm_td))
		return -EINVAL;
	printk("PWM value 0x%lx\n", pwm_td);
	pwm_td = abs(pwm_td);
	if (pwm_td > INTEL_VIBRA_MAX_TIMEDIVISOR || pwm_td < 0) {
		pr_err("Supported value is out of Range\n");
		return -EINVAL;
	}
	info->pwm.part.pwmtd = pwm_td;

	return len;
}

static ssize_t vibra_show_pwm_ontime_div(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct arizona_switch_info *info = dev_get_drvdata(dev);
	printk("enter:[%s][%d]\n", __func__, __LINE__);

	return sprintf(buf, "0x%X\n", info->pwm.part.pwmtd);
}

static struct device_attribute vibra_attrs[] = {
	__ATTR(vibrator, S_IRUGO | S_IWUSR,
	       vibra_show_vibrator, vibra_set_vibrator),
	__ATTR(pwm_baseunit, S_IRUGO | S_IWUSR,
	       vibra_show_pwm_baseunit, vibra_set_pwm_baseunit),
	__ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR,
	       vibra_show_pwm_ontime_div, vibra_set_pwm_ontime_div),
};

static int vibra_register_sysfs(struct arizona_switch_info *info)
{
	int r, i;

	for (i = 0; i < ARRAY_SIZE(vibra_attrs); i++) {
		r = device_create_file(info->dev, &vibra_attrs[i]);
		if (r)
			goto fail;
	}
	return 0;
fail:
	while (i--)
		device_remove_file(info->dev, &vibra_attrs[i]);

	return r;
}

static void vibra_unregister_sysfs(struct arizona_switch_info *info)
{
	int i;

	for (i = ARRAY_SIZE(vibra_attrs) - 1; i >= 0; i--)
		device_remove_file(info->dev, &vibra_attrs[i]);
}

void switch_arizona_interrupt_regist(void)
{
    int ret;

	ret = arizona_request_irq(arizona_global, ARIZONA_IRQ_JD_RISE,
				  "JACKDET rise", arizona_jackdet, info_swit);
	if (ret != 0) {
        printk("enter: %s, Failed to get JACKDET rise IRQ: %d==\n", __func__, ret);
	}

	ret = regmap_update_bits(arizona_global->regmap,ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_RISE, ARIZONA_WKUP_JD1_RISE);
	if (ret != 0) {
        printk("enter: %s, Failed to set JD rise IRQ wake: %d==\n", __func__, ret);
	}

	ret = arizona_request_irq(arizona_global, ARIZONA_IRQ_JD_FALL,
				  "JACKDET fall", arizona_jackdet, info_swit);
	if (ret != 0) {
        printk("enter: %s, Failed to get JD fall IRQ: %d==\n", __func__, ret);
	}

	ret = regmap_update_bits(arizona_global->regmap, ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_FALL, ARIZONA_WKUP_JD1_FALL);
	if (ret != 0) {
        printk("enter: %s, Failed to set JD fall IRQ wake: %d==\n", __func__, ret);
	}

	ret = arizona_request_irq(arizona_global, ARIZONA_IRQ_MICDET,
				  "MICDET", arizona_micdet, info_swit);
	if (ret != 0) {
        printk("enter: %s, Failed to get MICDET IRQ: %d==\n", __func__, ret);
	}

	regmap_update_bits(arizona_global->regmap, ARIZONA_MIC_DETECT_1,
			   ARIZONA_MICD_BIAS_STARTTIME_MASK |
			   ARIZONA_MICD_RATE_MASK,
			   6 << ARIZONA_MICD_BIAS_STARTTIME_SHIFT |
			   7 << ARIZONA_MICD_RATE_SHIFT);

	arizona_clk32k_enable(arizona_global);
	regmap_update_bits(arizona_global->regmap, ARIZONA_JACK_DETECT_DEBOUNCE,
			   ARIZONA_JD1_DB, ARIZONA_JD1_DB);
	regmap_update_bits(arizona_global->regmap, ARIZONA_JACK_DETECT_ANALOGUE,
			   ARIZONA_JD1_ENA, ARIZONA_JD1_ENA);

    return;
}
EXPORT_SYMBOL_GPL(switch_arizona_interrupt_regist);

static int __devinit arizona_switch_probe(struct platform_device *pdev)
{
	struct arizona *arizona = dev_get_drvdata(pdev->dev.parent);
	struct arizona_pdata *pdata;
	struct arizona_switch_info *info;
	int ret, mode;

	pdata = dev_get_platdata(arizona->dev);

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err;
	}

	info->micvdd = regulator_get(arizona->dev,"MICVDD");
	if (IS_ERR(info->micvdd)) {
		ret = PTR_ERR(info->micvdd);
		dev_err(arizona->dev, "Fail to get MICVDD: %d\n", ret);
		goto err;
	}

	mutex_init(&info->lock);
	info->arizona = arizona;
	info->dev = &pdev->dev;
	info->detecting = true;
	info->headset_flag = false;
	info->headphone_flag = false;
	platform_set_drvdata(pdev, info);

	switch (arizona->type) {
	case WM5102:
		switch (arizona->rev) {
		case 0:
			info->micd_reva = true;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	info->sdev.name = "h2w";
    info->sdev.print_name = headset_print_name;

	info->name = "intel_mid:vibrator";
	ret = switch_dev_register(&info->sdev);
	if (ret < 0) {
		dev_err(arizona->dev, "switch_dev_register() failed: %d\n",
			ret);
		goto err_micvdd;
	}

	if (pdata->num_micd_configs) {
		info->micd_modes = pdata->micd_configs;
		info->micd_num_modes = pdata->num_micd_configs;
	} else {
		info->micd_modes = micd_default_modes;
		info->micd_num_modes = ARRAY_SIZE(micd_default_modes);
	}

	if (arizona->pdata.micd_pol_gpio > 0) {
		if (info->micd_modes[0].gpio)
			mode = GPIOF_OUT_INIT_HIGH;
		else
			mode = GPIOF_OUT_INIT_LOW;

		ret = gpio_request_one(arizona->pdata.micd_pol_gpio,
				       mode,
				       "MICD polarity");
		if (ret != 0) {
			dev_err(arizona->dev, "Failed to request GPIO%d: %d\n",
				arizona->pdata.micd_pol_gpio, ret);
			goto err_register;
		}
	}

	arizona_switch_set_mode(info, 0);
    info_swit = info;
    micvdd_swit = info->micvdd;

    INIT_WORK(&info->headset_work, headset_detection_work);
    wake_lock_init(&info->headset_wakelock, WAKE_LOCK_SUSPEND, "arizona_headset_wakelock");
    wake_lock_init(&info->hook_wakelock, WAKE_LOCK_SUSPEND, "arizona_hook_wakelock");

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

/* the register of headset/headphone detection irq moved to k5_machine, 
 * because the k5_machine loaded later than this module, so there is a 
 * bug when the system powered up with headset/headphone plugin.*/
#if 0
	ret = arizona_request_irq(arizona, ARIZONA_IRQ_JD_RISE,
				  "JACKDET rise", arizona_jackdet, info);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to get JACKDET rise IRQ: %d\n",
			ret);
		goto err_register;
	}

	ret = regmap_update_bits(arizona->regmap,ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_RISE, ARIZONA_WKUP_JD1_RISE);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to set JD rise IRQ wake: %d\n",
			ret);
		goto err_rise;
	}

	ret = arizona_request_irq(arizona, ARIZONA_IRQ_JD_FALL,
				  "JACKDET fall", arizona_jackdet, info);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to get JD fall IRQ: %d\n", ret);
		goto err_rise_wake;
	}

	ret = regmap_update_bits(arizona->regmap, ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_FALL, ARIZONA_WKUP_JD1_FALL);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to set JD fall IRQ wake: %d\n",
			ret);
		goto err_fall;
	}

	ret = arizona_request_irq(arizona, ARIZONA_IRQ_MICDET,
				  "MICDET", arizona_micdet, info);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to get MICDET IRQ: %d\n", ret);
		goto err_micdet;
	}

	regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_1,
			   ARIZONA_MICD_BIAS_STARTTIME_MASK |
			   ARIZONA_MICD_RATE_MASK,
			   10 << ARIZONA_MICD_BIAS_STARTTIME_SHIFT |
			   11 << ARIZONA_MICD_RATE_SHIFT);

	arizona_clk32k_enable(arizona);
	regmap_update_bits(arizona->regmap, ARIZONA_JACK_DETECT_DEBOUNCE,
			   ARIZONA_JD1_DB, ARIZONA_JD1_DB);
	regmap_update_bits(arizona->regmap, ARIZONA_JACK_DETECT_ANALOGUE,
			   ARIZONA_JD1_ENA, ARIZONA_JD1_ENA);
#endif

	if (vibra_register_sysfs(info) < 0) {
		pr_err("could not register sysfs files\n");
		goto err_micdet;
	}
	pm_runtime_put(&pdev->dev);

	return 0;

err_micdet:
#if 0
	arizona_free_irq(arizona, ARIZONA_IRQ_MICDET, info);
	regmap_update_bits(arizona->regmap, ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_FALL,0);
err_fall:
	arizona_free_irq(arizona, ARIZONA_IRQ_JD_FALL, info);
err_rise_wake:
	regmap_update_bits(arizona->regmap, ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_RISE, 0);
err_rise:
	arizona_free_irq(arizona, ARIZONA_IRQ_JD_RISE, info);
#endif
err_register:
	pm_runtime_disable(&pdev->dev);
	switch_dev_unregister(&info->sdev);
err_micvdd:
	regulator_put(info->micvdd);
err:
	return ret;
}

static int __devexit arizona_switch_remove(struct platform_device *pdev)
{
	struct arizona_switch_info *info = platform_get_drvdata(pdev);
	struct arizona *arizona = info->arizona;

	pm_runtime_disable(&pdev->dev);

	regmap_update_bits(arizona->regmap, ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD1_RISE, 0);
	regmap_update_bits(arizona->regmap, ARIZONA_WAKE_CONTROL, ARIZONA_WKUP_JD2_FALL, 0);

	arizona_free_irq(arizona, ARIZONA_IRQ_MICDET, info);
	arizona_free_irq(arizona, ARIZONA_IRQ_JD_RISE, info);
	arizona_free_irq(arizona, ARIZONA_IRQ_JD_FALL, info);
	regmap_update_bits(arizona->regmap, ARIZONA_JACK_DETECT_ANALOGUE,
			   ARIZONA_JD1_ENA, 0);
	arizona_clk32k_disable(arizona);
	vibra_unregister_sysfs(info);

    if (wake_lock_active(&info->headset_wakelock))
        wake_unlock(&info->headset_wakelock);
    wake_lock_destroy(&info->headset_wakelock);
    if (wake_lock_active(&info->hook_wakelock))
        wake_unlock(&info->hook_wakelock);
    wake_lock_destroy(&info->hook_wakelock);

	switch_dev_unregister(&info->sdev);

	return 0;
}

static struct platform_driver arizona_switch_driver = {
	.driver		= {
		.name	= "arizona-extcon",
		.owner	= THIS_MODULE,
	},
	.probe		= arizona_switch_probe,
	.remove		= __devexit_p(arizona_switch_remove),
};

static int __init arizona_switch_gpio_init(void)
{
	return platform_driver_register(&arizona_switch_driver);
}
module_init(arizona_switch_gpio_init);

static void __exit arizona_switch_gpio_exit(void)
{
	platform_driver_unregister(&arizona_switch_driver);
}
module_exit(arizona_switch_gpio_exit);

MODULE_DESCRIPTION("Arizona Switch driver");
MODULE_AUTHOR("Chris Rattray <crattray@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:switch-arizona");
