/*
 * wm5102.h  --  WM5102 ALSA SoC Audio driver
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _WM5102_H
#define _WM5102_H

#include "arizona.h"

#define WM5102_FLL1        1
#define WM5102_FLL2        2
#define WM5102_FLL1_REFCLK 3
#define WM5102_FLL2_REFCLK 4

struct wm5102_priv {
	struct arizona_priv core;
	struct arizona_fll fll[2];

	unsigned int spk_ena:2;
	unsigned int spk_ena_pending:1;
	unsigned int mbc_ena:1;
	unsigned int vss_ena:1;
	unsigned int aec_ena:1;
	unsigned int txnr_ena:1;
};

extern bool micvdd_enable_flag;
extern bool mic_ev_enabled;

extern struct regulator *micvdd_swit;
extern struct wm5102_priv *wm5102_p;
extern struct arizona *arizona_global;

extern void arizona_core_interrupt_regist(void);
extern void switch_arizona_interrupt_regist(void);
extern void wm5102_fll_interrupt_regist(void);

#endif
