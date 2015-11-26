/*
 *  clv_machine.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2011-12 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Vaibhav Agarwal <vaibhav.agarwal@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <asm/intel_sst_ctp.h>
#include <asm/intel_mid_rpmsg.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/switch.h>
#include <linux/input.h>
#include "../ssp/mid_ssp.h"
#include "../../codecs/wm5102.h"
#include "../../codecs/arizona.h"
#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/registers.h>

#define K5_SYSCLK_RATE (2*24576000)

#define DAI_DSP_CODEC 1
#define SSP_IFX_SLOT_NB_SLOT	1
#define SSP_IFX_SLOT_WIDTH		32
#define SSP_IFX_SLOT_RX_MASK	0x1
#define SSP_IFX_SLOT_TX_MASK	0x1

struct snd_soc_jack	hs_jack;
struct input_dev *inp_dev;
static int aif_incall; /* storage to enusure we only sync on AIF2 when in a call  */
static bool card_sysclk = false;
static bool vibrator_sysclk = false;

/* Data path functionalities */
struct snd_pcm_hardware IFX_modem_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

void resume_set_sysclk(void)
{
    if (!aif_incall && !card_sysclk) {
        printk("enter %s, arizona: set sync to AIF1BCLK\n", __func__);
        arizona_set_fll(&wm5102_p->fll[0], ARIZONA_FLL_SRC_AIF1BCLK, 25 * 2 * 48000, K5_SYSCLK_RATE);
        vibrator_sysclk = true;
//    } else {
//        printk("%s, arizona: set sync to AIF2BCLK\n", __func__);
//        arizona_set_fll(&wm5102_p->fll[0], ARIZONA_FLL_SRC_AIF2BCLK, 16 * 2 * 48000, K5_SYSCLK_RATE);
    } else
        printk("enter %s, others have enabled sysclk!!\n", __func__);

    return;
}
EXPORT_SYMBOL_GPL(resume_set_sysclk);

void suspend_set_sysclk(void)
{
    if (card_sysclk)
        printk("enter %s, sysclk used by others!!\n", __func__);
    else {
        printk("enter %s, disable clk!!\n", __func__);
        arizona_set_fll(&wm5102_p->fll[0], 0, 0, 0);
    }
    vibrator_sysclk = false;

    return;
}
EXPORT_SYMBOL_GPL(suspend_set_sysclk);

static int clv_asp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	/* ARIZONA  Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	
	return 0;
}

static int clv_vsp_startup(struct snd_pcm_substream *substream)
{
	aif_incall++;
	return 0;
}

static void clv_vsp_shutdown(struct snd_pcm_substream *substream)
{
	if (aif_incall >= 1)
		aif_incall--;
	else 
		pr_err("Non symmetric settings of VSP DAI power up and down");
}



static int clv_vsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret , clk_source;

	pr_debug("Slave Mode selected\n");
	/* ARIZONA  Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	clk_source = SND_SOC_CLOCK_IN;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	return 0;
}

static int clv_bsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	/* ARIZONA  Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF
					| SND_SOC_DAIFMT_CBM_CFM;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	return 0;
}

static int clv_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "CTP Comms Machine: ERROR NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case CTP_RHB_COMMS_IFX_MODEM_DEV:
		str_runtime->hw = IFX_modem_alsa_hw_param;
		break;
	default:
		pr_err("CTP Comms Machine: bad PCM Device = %d\n",
						substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
						SNDRV_PCM_HW_PARAM_PERIODS);
}

static int clv_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
//	struct snd_soc_card *soc_card = rtd->card;
//	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
//	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
//	unsigned int device = substream->pcm->device;


//	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
//	pr_debug("ssp_voip_master_mode %d\n", ctl->ssp_voip_master_mode);
//	pr_debug("ssp_modem_master_mode %d\n", ctl->ssp_modem_master_mode);

		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
	ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SSP_DAI_SCMODE_0 |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		pr_err("MFLD Comms Machine:  Set FMT Fails %d\n", ret);
		return -EINVAL;
	}

		/*
		 * IFX Modem Mixing SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * Master:
		 *	tristate = 3
		 *	ssp_frmsync_timing_bit = 1, for MASTER
		 *	(NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * Slave:
		 *	tristate = 1
		 *	ssp_frmsync_timing_bit = 0, for SLAVE
		 *	(NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 *
		 */
	nb_slot = SSP_IFX_SLOT_NB_SLOT;
	slot_width = SSP_IFX_SLOT_WIDTH;
	tx_mask = SSP_IFX_SLOT_TX_MASK;
	rx_mask = SSP_IFX_SLOT_RX_MASK;

	tristate_offset = BIT(TRISTATE_BIT) |\
				BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
			rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("CTP Comms Machine:  Set TDM Slot Fails %d\n",
				ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("CTP Comms Machine: Set Tristate Fails %d\n",
				ret);
		return -EINVAL;
	}

	pr_debug("CTP Comms Machine: slot_width = %d\n",
			slot_width);
	pr_debug("CTP Comms Machine: tx_mask = %d\n",
			tx_mask);
	pr_debug("CTP Comms Machine: rx_mask = %d\n",
			rx_mask);
	pr_debug("CTP Comms Machine: tristate_offset = %d\n",
			tristate_offset);

	return 0;

} /* clv_comms_dai_link_hw_params*/

static int clv_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
//	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
//	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
			__func__,
			substream->runtime->rate);

	/* select clock source (if master) */
	if (device == CTP_RHB_COMMS_IFX_MODEM_DEV) {
		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);
	}

	return 0;
} /* ctp_comms_dai_link_prepare */


struct clv_mc_private {
	struct ipc_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
	/* Jack related */
	struct delayed_work jack_work;
	struct snd_soc_jack clv_jack;
	atomic_t bpirq_flag;
};


/* Board specific codec bias level control */
static int clv_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	int ret;

	struct snd_soc_dai *codec_dai = card->rtd[DAI_DSP_CODEC].codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;

	if (dapm->dev != codec->dev)
        return 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
        if (card->dapm.bias_level == SND_SOC_BIAS_OFF) {
            card_sysclk = true;
            if(vibrator_sysclk)
                snd_soc_codec_set_pll(codec, WM5102_FLL1,0,0,0);
            if (!aif_incall) {
                printk("%s, arizona: set sync to AIF1BCLK\n", __func__);
                ret = snd_soc_codec_set_pll(codec, WM5102_FLL1,
                        ARIZONA_FLL_SRC_AIF1BCLK,
                        25 * 2 * 48000,
                        K5_SYSCLK_RATE);
            } else {
                printk("%s, arizona: set sync to AIF2BCLK\n", __func__);
                ret = snd_soc_codec_set_pll(codec, WM5102_FLL1,
                        ARIZONA_FLL_SRC_AIF2BCLK,
                        16 * 2 * 48000,
                        K5_SYSCLK_RATE);
            }
            if (ret < 0) {
                pr_err("can't set FLL %d\n", ret);
                return ret;
            }

			card->dapm.bias_level = level;
		}
		break;
	case SND_SOC_BIAS_OFF:
		/* OSC clk will be turned OFF after processing
		 * codec->dapm.bias_level = SND_SOC_BIAS_OFF.
		 */
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);

	return 0;
}

static int clv_set_bias_level_post(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	int ret;

	struct snd_soc_dai *codec_dai = card->rtd[DAI_DSP_CODEC].codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;

	if (dapm->dev != codec->dev)
        return 0;

 	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/* Processing already done during set_bias_level()
		 * callback. No action required here.
		 */
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF) {
            printk("enter %s, not SND_SOC_BIAS_OFF\n", __func__);
			break;
        }

        ret = snd_soc_codec_set_pll(codec, WM5102_FLL1,0,0,0); 
        if (ret < 0) {
            pr_err("can't turn off FLL %d\n", ret);
            return ret;
        }
        card_sysclk = false;
        printk("enter %s, disable clk!!\n", __func__);

		card->dapm.bias_level = level;
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);

	return 0;
}

static int clv_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
//	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	//struct snd_soc_card *card = runtime->card;
        //struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);

    /* Headset and button jack detection */
    snd_soc_jack_new(codec, "Headset", SND_JACK_HEADSET, &hs_jack);

    inp_dev	= input_allocate_device();
    input_set_capability(inp_dev, EV_SW, SW_HEADPHONE_INSERT);
    input_set_capability(inp_dev, EV_SW, SW_MICROPHONE_INSERT);
    input_set_capability(inp_dev, EV_KEY, KEY_MEDIA);
    inp_dev->name = "Headset Hook Key";

    ret	= input_register_device(inp_dev);
    if (ret != 0) {
        printk("enter: %s, ret = %d: Error in input_register_device\n", __func__, ret);
    }

    arizona_core_interrupt_regist();
    switch_arizona_interrupt_regist();
    wm5102_fll_interrupt_regist();
    
    return 0;
}

static unsigned int rates_48000[] = {
	8000,
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count	= ARRAY_SIZE(rates_48000),
	.list	= rates_48000,
};

static int clv_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_48000);
	return 0;
}

static struct snd_soc_dapm_widget medfield_wm5102_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", NULL ), // HLL medfield_wm5102_event_int_spk),
	SND_SOC_DAPM_HP("Earphone", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Main MIC", NULL),
	SND_SOC_DAPM_MIC("Secondary MIC", NULL),
	SND_SOC_DAPM_MIC("HSMIC", NULL),
	SND_SOC_DAPM_LINE("FM_OUT1", NULL),
	SND_SOC_DAPM_LINE("FM_OUT2", NULL),
  
};

static struct snd_soc_dapm_route medfield_wm5102_audio_paths[] = {
    {"Sub LINN", NULL, "EPOUTN"},
    {"Sub LINP", NULL, "EPOUTP"}, 

    {"Earphone", NULL, "Sub SPKN"},
    {"Earphone", NULL, "Sub SPKP"}, 

	{"Headphone Jack", NULL, "HPOUT1R"},
	{"Headphone Jack", NULL, "HPOUT1L"},

#if 0
	{"Int Spk", NULL, "SPKOUTLP"},
	{"Int Spk", NULL, "SPKOUTLN"},
#else
	{"Int Spk", NULL, "HPOUT2R"},
	{"Int Spk", NULL, "HPOUT2L"},
#endif

	{"HSMIC", NULL, "MICBIAS1"},
	{"IN1L", NULL, "HSMIC"},

	{"Main MIC", NULL, "MICBIAS2"},
	{"IN3L", NULL, "Main MIC"},

	{"Secondary MIC", NULL, "MICBIAS2"},
	{"IN3R", NULL, "Secondary MIC"},

	{"IN2L", NULL, "FM_OUT1"},
	{"IN2R", NULL, "FM_OUT2"},

};

static const struct snd_kcontrol_new medfield_wm5102_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Earphone"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("LineOut"),
	SOC_DAPM_PIN_SWITCH("HSMIC"),
	SOC_DAPM_PIN_SWITCH("Main MIC"),
	SOC_DAPM_PIN_SWITCH("Secondary MIC"),
	SOC_DAPM_PIN_SWITCH("LineIn"),
};

static struct snd_soc_ops clv_asp_ops = {
	.startup = clv_startup,
	.hw_params = clv_asp_hw_params,
};

static struct snd_soc_ops clv_vsp_ops = {
	.hw_params = clv_vsp_hw_params,
	.startup = clv_vsp_startup,
	.shutdown = clv_vsp_shutdown,
};

static struct snd_soc_ops clv_bsp_ops = {
	//.startup = clv_bsp_startup,
	.hw_params = clv_bsp_hw_params,
};

static struct snd_soc_ops clv_comms_dai_link_ops = {
	.startup = clv_comms_dai_link_startup,
	.hw_params = clv_comms_dai_link_hw_params,
	.prepare = clv_comms_dai_link_prepare,
};

/*  static struct snd_soc_ops clv_probe_ops = {
	.startup = clv_startup,
};*/

struct snd_soc_dai_link clv_msic_dailink[] = {
	{
		.name = "Cloverview ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-platform",
		.init = clv_init,
		.ignore_suspend = 1,
		.ops = &clv_asp_ops,
	},

	{
		.name = "Cloverview VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "wm5102-aif2",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &clv_vsp_ops,
	},

	{
		.name = "SoC BT",
		.stream_name = "BT_Voice",
		.cpu_dai_name = "BT_virtual-cpu-dai",
		.codec_dai_name = "wm5102-aif3",
		.codec_name = "wm5102-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &clv_bsp_ops,
	},

    {
        .name = "Cloverview Comp ASP",
        .stream_name = "Compress-Audio",
        .cpu_dai_name = "Compress-cpu-dai",
        .codec_dai_name = "wm5102-aif1",
        .codec_name = "wm5102-codec",
        .platform_name = "sst-platform",
        .init = NULL,
        .ignore_suspend = 1,
        .ops = &clv_asp_ops,
    },

    {
		.name = "Cloverview Comms IFX MODEM",
		.stream_name = "IFX_MODEM_MIXING",
		.cpu_dai_name = SSP_MODEM_DAI_NAME,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "mid-ssp-dai",
		.init = NULL,
		.ops = &clv_comms_dai_link_ops,
    },

    {
        .name = "Cloverview virtual-ASP",
        .stream_name = "virtual-stream",
        .cpu_dai_name = "Virtual-cpu-dai",
        .codec_dai_name = "wm5102-aif1",
        .codec_name = "wm5102-codec",
        .platform_name = "sst-platform",
        .init = NULL,
        .ignore_suspend = 1,
        .ops = &clv_asp_ops,
    },

/*    {
	.name = "Cloverview Probe",
        .stream_name = "CTP Probe",
        .cpu_dai_name = "Probe-cpu-dai",
        .codec_dai_name = "snd-soc-dummy-dai",
        .codec_name = "snd-soc-dummy",
        .platform_name = "sst-platform",
        .init = NULL,
        .ops = &clv_probe_ops,
    },*/
};

#ifdef CONFIG_PM

static int snd_clv_prepare(struct device *dev)
{
	pr_debug("K5 In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static void snd_clv_complete(struct device *dev)
{
	pr_debug("K5 In %s\n", __func__);
	snd_soc_resume(dev);
	return;
}

static int snd_clv_poweroff(struct device *dev)
{
	pr_debug("K5 In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_clv_prepare NULL
#define snd_clv_complete NULL
#define snd_clv_poweroff NULL
#endif

static int medfield_wm5102_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec = card->rtd[0].codec;
//	struct snd_soc_dai *codec_dai = card->rtd[DAI_DSP_CODEC].codec_dai;
	int ret;

	/* We should start and stop the FLL in set_bias_level, can start now
	 * as we don't need the sync clock to start when we have 32kHz ref clock.
	 */
    aif_incall = 0;
    printk("%s, arizona: set sync to AIF1BCLK\n", __func__);
    card_sysclk = true;
    if(vibrator_sysclk)
        snd_soc_codec_set_pll(codec, WM5102_FLL1,0,0,0);
    ret = snd_soc_codec_set_pll(codec, WM5102_FLL1,
            ARIZONA_FLL_SRC_AIF1BCLK,
            25 * 2 * 48000,
            K5_SYSCLK_RATE);
    if (ret < 0) {
        pr_err("can't set FLL in late probe%d\n", ret);
        return ret;
    }

	/* Always derive SYSCLK from the FLL, we will ensure FLL is enabled in
	 * set_bias_level()
	 */
	ret = snd_soc_codec_set_sysclk(codec, ARIZONA_CLK_SYSCLK, ARIZONA_CLK_SRC_FLL1,
				     K5_SYSCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	
	return 0;
}

static struct snd_soc_codec_conf clv_codec_conf[] = {
       {
       .dev_name = "wm2000.1-003a",
       .name_prefix = "Sub",
       },
};

static struct snd_soc_aux_dev clv_aux_devs[] = {
       {
       .name = "wm2000",
       .codec_name = "wm2000.1-003a",
       },
};

/* SoC card */
static struct snd_soc_card snd_soc_card_clv = {
	.name = "K5",
	.dai_link = clv_msic_dailink,
	.num_links = ARRAY_SIZE(clv_msic_dailink),
	.set_bias_level = clv_set_bias_level,
	.set_bias_level_post = clv_set_bias_level_post,
	.aux_dev = clv_aux_devs,
	.num_aux_devs = ARRAY_SIZE(clv_aux_devs),
	.codec_conf = clv_codec_conf,
	.num_configs = ARRAY_SIZE(clv_codec_conf),
	.controls = medfield_wm5102_controls,
	.num_controls = ARRAY_SIZE(medfield_wm5102_controls),
	.dapm_widgets = medfield_wm5102_widgets,
	.num_dapm_widgets = ARRAY_SIZE(medfield_wm5102_widgets),
	.dapm_routes = medfield_wm5102_audio_paths,
	.num_dapm_routes = ARRAY_SIZE(medfield_wm5102_audio_paths),

	.late_probe = medfield_wm5102_late_probe,
};

static int snd_ctp_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct clv_mc_private *ctx;

	pr_debug("In %s\n", __func__);
	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/* register the soc card */
	snd_soc_card_clv.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_clv, ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_clv);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_clv);

	pr_debug("successfully exited probe\n");
	return ret_val;

unalloc:
	kfree(ctx);
	return ret_val;
}

static int __devexit snd_ctp_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	cancel_delayed_work_sync(&ctx->jack_work);
	kfree(ctx);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}
static struct dev_pm_ops snd_ctp_mc_pm_ops = {
	.prepare = snd_clv_prepare,
	.complete = snd_clv_complete,
	.poweroff = snd_clv_poweroff,
};

static struct platform_driver snd_ctp_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ctp_audio",
		.pm   = &snd_ctp_mc_pm_ops,
	},
	.probe = snd_ctp_mc_probe,
	.remove = __devexit_p(snd_ctp_mc_remove),
};

static int __init snd_ctp_driver_init(void)
{
	pr_info("In %s\n", __func__);
	return platform_driver_register(&snd_ctp_mc_driver);
}

static void snd_ctp_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_ctp_mc_driver);
}

static int snd_clv_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_clv rpmsg device\n");

	ret = snd_ctp_driver_init();

out:
	return ret;
}

static void snd_clv_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_ctp_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_clv rpmsg device\n");
}

static void snd_clv_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_clv_rpmsg_id_table[] = {
	{ .name = "rpmsg_msic_clv_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_clv_rpmsg_id_table);

static struct rpmsg_driver snd_clv_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_clv_rpmsg_id_table,
	.probe		= snd_clv_rpmsg_probe,
	.callback	= snd_clv_rpmsg_cb,
	.remove		= snd_clv_rpmsg_remove,
};

static int __init snd_clv_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_clv_rpmsg);
}

late_initcall(snd_clv_rpmsg_init);

static void __exit snd_clv_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_clv_rpmsg);
}
module_exit(snd_clv_rpmsg_exit);


MODULE_DESCRIPTION("ASoC K5 machine driver");
MODULE_LICENSE("GPL v2");
