/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */
/*
   BQ27531+BQ2419x for K5

 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/reboot.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/usb/penwell_otg.h>
#include <linux/wakelock.h>
#include <linux/string.h>
#include <linux/power/bq27x00_battery.h>
#include <asm/intel_mid_gpadc.h>
#include "bq275x_firmware.h"

#define DRIVER_VERSION			"1.2.0"

#define BQ27530_REG_CNTL		0x00 /*control*/
#define BQ27x00_REG_TEMP		0x06 /*Temperature*/
#define BQ27x00_REG_VOLT		0x08 /*Voltage*/
#define BQ27x00_REG_AI			0x14 /*AverageCurrent*/
#define BQ27x00_REG_FLAGS		0x0A /*FLAGS*/
#define BQ27500_FLAG_DSC		BIT(0)/*Discharging detected*/
#define BQ27500_FLAG_SOC1		BIT(2)/*Soc threshold 1*/
#define BQ27500_FLAG_CHG_EN		BIT(8)/*enable charge*/
#define BQ27500_FLAG_FC			BIT(9)/*Full-charged condition reached*/
#define BQ27500_FLAG_CNTL_EN		BIT(10)/*Full-charged condition reached*/
#define BQ27500_FLAG_UT			BIT(14)/*Undertemperature*/
#define BQ27500_FLAG_OT			BIT(15)/*Overtemperature*/
#define BQ27530_REG_RM			0x10 /*Remaining Capacity*/
#define BQ27530_REG_FCC			0x12 /*FullChargeCapacity*/
#define BQ27500_REG_PCAI		0x1E /*get Charging Current*/
#define BQ27500_REG_PCV			0x20 /*get Charging Voltage*/
#define BQ27500_REG_SOC			0x2C /*SOC*/
#define BQ27x00_REG_TTE			0x16 /*TimeToEmpty*/
#define BQ27x00_REG_NAC			0x0C /* Nominal available capaciy */
#define BQ27x00_REG_CYCT		0x2A /* Cycle count total */
#define BQ27530_REG_CCAI		0x70 /*CalcChargingCurrent*/
#define BQ27530_REG_CCV			0x72 /*CalcChargingVoltage*/
#define BQ27500_REG_DCAP		0x3C /* Design capacity*/

/*Charger Command*/
#define BQ24192_REG_STAT		0x74 /*ChargerStatus*/
#define BQ24192_STAT_WE			BIT(7) /*Waiting on GG_CHGCNTRL_ENB command*/
#define BQ24192_STAT_ERR		BIT(6) /*I2C comm error 27530-24912*/
#define BQ24192_STAT_WFAIL		BIT(4) /*Write Charger error*/
#define BQ24192_STAT_INIT		BIT(2) /*init error*/
#define BQ24192_STAT_WRITE		BIT(1) /*write*/
#define BQ24192_STAT_READ		BIT(0) /*read*/

#if 0
#define BQ24192_REG_CTL_0		0x75 /*Charger Control REG 0*/
#define BQ24192_0_STAT_2		BIT(6) /*Charger Source*/
#define BQ24192_0_STAT_1		BIT(5)
#define BQ24192_0_STAT_0		BIT(4)
#define BQ24192_0_FAL_2			BIT(2)/*Charger fault status bit*/
#define BQ24192_0_FAL_1			BIT(1)
#define BQ24192_0_FAL_0			BIT(0)
#endif
#define BQ24192_REG_POR_1		0x76 /*Charger Control REG 1*/

#define BQ24192_REG_CURRENT		0x77 /*Charge current */
#define BQ24192_CC_5			BIT(7) /*2048mA */
#define BQ24192_CC_4			BIT(6) /*1024mA */
#define BQ24192_CC_3			BIT(5) /*512mA */
#define BQ24192_CC_2			BIT(4) /*256mA */
#define BQ24192_CC_1			BIT(3) /*128mA */
#define BQ24192_CC_0			BIT(2) /*64mA */

#define BQ24192_REG_PRE_TERM_CURRENT	0x78 /*PreTerm_Charge current */
#define BQ24192_PRE_CC_3		BIT(7) /*1024mA */
#define BQ24192_PRE_CC_2		BIT(6) /*512mA */
#define BQ24192_PRE_CC_1		BIT(5) /*256mA */
#define BQ24192_PRE_CC_0		BIT(4) /*128mA */
#define BQ24192_TERM_CC_3		BIT(3) /*1024mA */
#define BQ24192_TERM_CC_2		BIT(2) /*512mA */
#define BQ24192_TERM_CC_1		BIT(1) /*256mA */
#define BQ24192_TERM_CC_0		BIT(0) /*128mA */


#define BQ24192_REG_VOLT_4		0x79 /*Charger Voltage REG 3*/
#define BQ24192_4_VREG_5		BIT(7) /*512mV*/
#define BQ24192_4_VREG_4		BIT(6) /*256mV*/
#define BQ24192_4_VREG_3		BIT(5) /*128mV*/
#define BQ24192_4_VREG_2		BIT(4) /*64mV*/
#define BQ24192_4_VREG_1		BIT(3) /*32mV*/
#define BQ24192_4_VREG_0		BIT(2) /*16mV*/
#define BQ24192_4_BATLOWV		BIT(1) /*Pre-charge to fast charge 0=2.8V 1=3.0V*/
#define BQ24192_4_VRECHG		BIT(0) /*Recharge Threshold 0=100mV 1=300mV*/

#define BQ24192_REG_TERM_TIMER		0x7a
#define BQ24192_5_EN_TERM		BIT(7) /*term detect by coulomb counter*/
#define BQ24192_5_WT_TIMER		0x03 /*watchdog timer set*/
#define BQ24192_5_EN_TIMER		BIT(3) /*0=disable charging timer,1=enable*/
#define BQ24192_5_CHG_TIMER		0x03 /*Fast charging timer*/

#define BQ24192_REG8_STATUS		0x7D
#define BQ24192_8_VBUS_STAT		0x03
#define BQ24192_8_CHRG_STAT		0x03
#define BQ24192_8_DPM_STAT		BIT(3)
#define BQ24192_8_PG_STAT		BIT(2)

#define BQ24192_REG9_FAULT		0x7E
#define BQ24192_9_WT			BIT(7)
#define BQ24192_9_OTG			BIT(6)
#define BQ24192_9_CHRG			0x03
#define BQ24192_9_BAT			BIT(3)

#define BQ24192_REG_DEV_VER		0x7F
#define BQ24192_A_PN			0x07


#define BQ27x00_REG_TTF			0x18 /*TimeToFull--------del or rename*/
#define BQ27x00_REG_TTECP		0x26 /*del or rename FCCF*/
#define BQ27x00_REG_LMD			0x12 /* Last measured discharge ---del or rename*/
#define BQ27x00_REG_AE			0x22 /* Available enery----del or rename*/
#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_REG_ILMD		0x76 /* Initial last measured discharge */
#define BQ27000_FLAG_CHGS		BIT(7)
#define BQ27000_FLAG_FC			BIT(5)
#define BQ27000_RS			20 /* Resistor sense */

/* ADC Channel Numbers */
#define CLT_BATT_NUM_GPADC_SENSORS	1
#define CLT_GPADC_USB_VOLTAGE		0x8
#define CLT_GPADC_VOLTAGE_SAMPLE_COUNT	10


#define CHRG_INT_N			93
#define FG_INT_N			94
#define BQ24190_IC_VERSION			0x4
#define BQ24192_IC_VERSION			0x5
#define BQ24192I_IC_VERSION			0x3

//#define STATUS_UPDATE_INTERVAL		(HZ * 20) /* 60sec */
#define BQ24192_CHRG_OTG_GPIO		36

#define BATT_TEMP_MAX			600
#define BATT_VOLT_MAX			4400
#define BATT_VOLT_MIN			3000

int ovp=0,ovp_flag=0,work_num=0;
int soc_prev = 0;
volatile int work_count=0;
volatile int rd_count=0;
struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
};

static int ctp_get_usb_voltage(struct bq27x00_device_info *di,int *tmp);
static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single);
static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, int num, unsigned char *buf,int rom_mode);
static void fg_reg_show(struct bq27x00_device_info *di);

enum bq27x00_chip { BQ27000, BQ27500 };

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int flags;
	int voltage;
	int current_now;
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;
	struct delayed_work chrg_isr_wrkr;
	struct delayed_work chrg_ovp_mon;
	void *gpadc_handle;
	struct power_supply	bat;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply_charger_cap cap;
	bool mains_online;
	bool usb_online;
	char chrg_vendor[8];
	struct bq27x00_access_methods bus;
	int chrg_irq_n;
	int fg_irq_n;

	struct mutex lock;
	struct wake_lock wakelock;
	struct wake_lock ovp_wakelock;
};

struct bq27x00_device_info *bqdi = NULL;

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	//POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
/*
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
*/
	POWER_SUPPLY_PROP_TECHNOLOGY,
/*
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
*/
	POWER_SUPPLY_PROP_CYCLE_COUNT,
//	POWER_SUPPLY_PROP_ENERGY_NOW,
};
static enum power_supply_property bq24912_mains_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MANUFACTURER
};
static enum power_supply_property bq24912_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MANUFACTURER
};

/*
static unsigned int poll_interval = 360;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
		"0 disables polling");
*/
int firmware_write(struct bq27x00_device_info *di);


static unsigned int bc_fw = 0;
static unsigned int fw_stat = 0;
module_param(fw_stat, uint, 0444);
MODULE_PARM_DESC(fw_stat, "get fw download state");

static unsigned int fw_delay = 0;
module_param(fw_delay, uint, 0644);
MODULE_PARM_DESC(fw_delay, "Get fw download delay");

static unsigned int fw_long_delay = 70;
module_param(fw_long_delay, uint, 0644);
MODULE_PARM_DESC(fw_long_delay, "Get fw download delay");

static unsigned int fw_more_delay = 70;
module_param(fw_more_delay, uint, 0644);
MODULE_PARM_DESC(fw_long_delay, "Get fw download delay");

static unsigned int tim = 60;
module_param(tim, uint, 0644);
MODULE_PARM_DESC(tim, "The value is charge state update time");

static int bat_status = 0;
//static unsigned int otg_switch = 1;

static unsigned int delay_t = 3000;
module_param(delay_t, uint, 0644);
MODULE_PARM_DESC(delay_t, "The value is charge state update time");

/*
 * Common code for BQ27x00 devices
 */
static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
		bool single)
{
	return di->bus.read(di, reg, single);
}
static void bq24192_vbus_stat(struct bq27x00_device_info *di)
{
	int ret;
	ret = bq27x00_read(di, BQ24192_REG8_STATUS, true);
	if((ret >> 6 & 0x3) == 0x3){
		dev_info(di->dev, "vbus status is OTG\n");
		bat_status = 0;
		di->usb_online = 0;
		di->mains_online = 0;
	}else if((ret >> 6 & 0x1) == 0x1){
		dev_info(di->dev, "vbus status is USB\n");
		di->usb_online = 1;
		bat_status = 1;
	}else if((ret >> 6 & 0x2) == 0x2){
		dev_info(di->dev, "vbus status is AC\n");
		di->mains_online = 1;
		bat_status = 1;
	}else if(!(ret >>6)){
		dev_info(di->dev, "No input\n");
		di->usb_online = 0;
		di->mains_online = 0;
		bat_status = 0;
	}else
		printk("%s:Chrgr_status_reg ret is 0x%x",__func__,ret);
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc = -1;
	int volt = -1;
	int chrg_status,chrg_term_tim,chrg_term_curr;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	rsoc = bq27x00_read(di, BQ27500_REG_SOC, false);
	if (rsoc < 0 || volt < 0 )
			dev_err(di->dev, "error reading soc %d or volt %d\n",rsoc,volt);

	/* Only for DVT1 tested until the new battery*/
	/*if(rsoc == 0 && volt >= 3700){*/
		/*rsoc = 22;*/
		/*return rsoc;*/
	/*}*/
	chrg_status = bq27x00_read(di, BQ24192_REG8_STATUS,true);
	if(di->mains_online == 1 || di->usb_online == 1){
		if((chrg_status >> 4 & 0x3) == 0x3){
			if(rsoc != 100){
				rsoc = 100;
				chrg_term_curr = bq27x00_read(di,0x78,true);
				chrg_term_tim = bq27x00_read(di,0x7A,true);
				dev_err(di->dev,"0x7D is 0x%x,0x78 is 0x%x,0x7A is 0x%x!\n",
						chrg_status,chrg_term_curr,chrg_term_tim);
			}
		}
	}
	if(rsoc > 0){
		soc_prev = rsoc;
		rd_count = 0;
	}else{
		if(rd_count >= 9){
			rd_count = 0;
			dev_info(di->dev,"Low soc %d detect!\n",rsoc);
		}else{
			rd_count++;
			dev_info(di->dev,"Low soc %d prev %d loop %d!\n",rsoc,soc_prev,rd_count);
			rsoc = soc_prev;
		}
	}
	return rsoc;
}
/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27x00_read(di, reg, false);
	if (charge < 0) {
		dev_err(di->dev, "error reading nominal available capacity\n");
		return charge;
	}

	if (di->chip == BQ27500)
		charge *= 1000;
	else
		charge = charge * 3570 / BQ27000_RS;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_lmd(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_ilmd(struct bq27x00_device_info *di)
{
	int ilmd;

	if (di->chip == BQ27500)
		ilmd = bq27x00_read(di, BQ27500_REG_DCAP, false);
	else
		ilmd = bq27x00_read(di, BQ27000_REG_ILMD, true);

	if (ilmd < 0) {
		dev_err(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}

	if (di->chip == BQ27500)
		ilmd *= 1000;
	else
		ilmd = ilmd * 256 * 3570 / BQ27000_RS;

	return ilmd;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading register %02x: %d\n", reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };
	//bool is_bq27500 = di->chip == BQ27500;

	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (cache.flags >= 0) {
		cache.capacity = bq27x00_battery_read_rsoc(di);
		cache.temperature = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	//	cache.time_to_empty = bq27x00_battery_read_time(di, BQ27x00_REG_TTE);
	//	cache.time_to_empty_avg = bq27x00_battery_read_time(di, BQ27x00_REG_TTECP);
	//	cache.time_to_full = bq27x00_battery_read_time(di, BQ27x00_REG_TTF);
	//	cache.charge_full = bq27x00_battery_read_lmd(di);
		cache.cycle_count = bq27x00_battery_read_cyct(di);

		cache.current_now = bq27x00_read(di, BQ27x00_REG_AI, false);
		cache.voltage = bq27x00_read(di, BQ27x00_REG_VOLT, false);

		/* We only have to read charge design full once */
/*
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27x00_battery_read_ilmd(di);
*/
	}
	if(ovp == 0)
		bq24192_vbus_stat(di);

	/* Ignore current_now which is a snapshot of the current battery state
	 * and is likely to be different even between two consecutive reads */
	if (memcmp(&di->cache, &cache, sizeof(cache) - sizeof(int)) != 0) {
		di->cache = cache;
		power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
	dev_info(di->dev, "bat_cap=%d,bat_temp=%d,bat_curr=%d,bat_vol=%d\n",
			cache.capacity,(cache.temperature - 2731),
			(int)((s16)cache.current_now),cache.voltage);
	dev_info(di->dev,"bat_flag=0x%x,bat_status=%d\n",cache.flags,bat_status);
    fg_reg_show(di);
}
static void fg_reg_show(struct bq27x00_device_info *di)
{

	unsigned char data[2]={0};
	int ret,fg_cont,fg_rm_cap,fg_fc_cap,fg_pc_volt,fg_cc_volt;

	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: status cmd write error\n",__func__);
	}
	mdelay(50);
	fg_cont = bq27x00_read(di,BQ27530_REG_CNTL, false);

	fg_rm_cap = bq27x00_read(di, 0x10,false);
	fg_fc_cap = bq27x00_read(di, 0x12,false);
	fg_pc_volt = bq27x00_read(di, 0x20,false);
	fg_cc_volt = bq27x00_read(di, 0x72,false);
	dev_info(di->dev,"Control()=0x%x,RemiainingCap=%d,FullChargeCap=%d,0x20=%d,0x72=%d\n",
					fg_cont,fg_rm_cap,fg_fc_cap,fg_pc_volt,fg_cc_volt);
}
static void charge_reg_show(struct bq27x00_device_info *di)
{
	//int ret,cv = 0,cc = 0;
	//int recharge_volt,bat_low_v;
	unsigned int chrg_curr;
	int calc_curr;
	int chrg_status,chrg_fault,chrg_por;

	chrg_curr = bq27x00_read(di, BQ27500_REG_PCAI,false);
	calc_curr = bq27x00_read(di, BQ27530_REG_CCAI,false);
/*
	ret = bq27x00_read(di, BQ24192_REG_CURRENT,true);
	if(ret & BQ24192_CC_5)
		cc +=2048;
	else if (ret & BQ24192_CC_4)
		cc +=1024;
	else if (ret & BQ24192_CC_3)
		cc +=512;
	else if (ret & BQ24192_CC_2)
		cc +=256;
	else if (ret & BQ24192_CC_1)
		cc +=128;
	else if (ret & BQ24192_CC_0)
		cc +=64;
	cc+=500;	//offset 500mA

	ret = bq27x00_read(di, BQ24192_REG_VOLT_4,true);
	if(ret & BQ24192_4_VREG_5)
		cv +=512;
	else if(ret & BQ24192_4_VREG_4)
		cv +=256;
	else if(ret & BQ24192_4_VREG_3)
		cv +=128;
	else if(ret & BQ24192_4_VREG_2)
		cv +=64;
	else if(ret & BQ24192_4_VREG_1)
		cv +=32;
	else if(ret & BQ24192_4_VREG_0)
		cv +=16;
	cv+=3504;	//offset 3.504v

	if(ret & BQ24192_4_VRECHG) //recharge threshold
		recharge_volt = 300;
	else
		recharge_volt = 100;

	if(ret & BQ24192_4_BATLOWV) //from pre-charge to fast charge
		bat_low_v = 3000;
	else
		bat_low_v = 2800;

	dev_info(di->dev, "charge volt =%d, charge curr =%d,recharge volt =%d Pre-charge volt =%d\n",cv,cc,recharge_volt,bat_low_v);
*/
	chrg_por = bq27x00_read(di, BQ24192_REG_POR_1,true);
	chrg_status = bq27x00_read(di, BQ24192_REG8_STATUS,true);
	chrg_fault = bq27x00_read(di, BQ24192_REG9_FAULT,true);
	dev_info(di->dev, "CHRG & FG reg: 0x1E=%d,0x70=%d,0x76=0x%x,0x7D=0x%x,0x7E=0x%x\n",
					chrg_curr,calc_curr,chrg_por,chrg_status,chrg_fault);
}

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);
	//int ret,usb_voltage;

	dev_info(di->dev, "fg event worker\n");
/*
	ret=ctp_get_usb_voltage(bqdi,&usb_voltage);
	if (ret < 0) {
		dev_err(bqdi->dev, "failed to acquire usb voltage\n");
		if (wake_lock_active(&di->wakelock))
			wake_unlock(&di->wakelock);
		return;
	}else if(ret){
		dev_info(di->dev, "USB OVP charging stop\n");
		di->usb_online = 0;
		di->mains_online = 0;
		return;
	}
*/
	bq27x00_update(di);
	if (wake_lock_active(&di->wakelock))
		wake_unlock(&di->wakelock);
	if(work_count > 0){
		work_count-=1;
		if(work_count > 0)
			schedule_delayed_work(&di->work, msecs_to_jiffies(delay_t));
	}
#if 0
	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
#endif
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	if (di->cache.temperature < 0)
		return di->cache.temperature;

	if(di->cache.temperature <= 2331)
		val->intval = 255;
	else
		val->intval = di->cache.temperature - 2731;

	return 0;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int curr;
	curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	val->intval = (int)((s16)curr);

#if 0
	if (di->chip == BQ27500)
		curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	else
		curr = di->cache.current_now;

	if (curr < 0)
		return curr;

	if (di->chip == BQ27500) {
		/* bq27500 returns signed value unit mA*/
		val->intval = (int)((s16)curr);
	} else {
		if (di->cache.flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27000_RS;
	}
#endif
	return 0;
}
static int bq27x00_battery_health(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int ret;
	if (di->cache.temperature < 0){
		dev_err(di->dev, "battery pack temp read fail\n");
		return val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}
	if((di->cache.temperature - 2731) > BATT_TEMP_MAX){
		dev_err(di->dev, "battery pack temp read fail:%d\n",
					di->cache.temperature - 2731);
		return val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	ret = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if(ret < 0){
		dev_err(di->dev, "battery pack voltage read fail\n");
		return val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}
	if(ret > BATT_VOLT_MAX){
		dev_err(di->dev, "Battery Over Voltage condition Detected:%d\n",ret);
		return val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}
	if(ret < BATT_VOLT_MIN){
		dev_err(di->dev, "Low Battery condition Detected:%d\n",ret);
		return val->intval = POWER_SUPPLY_HEALTH_DEAD;
	}
	return val->intval = POWER_SUPPLY_HEALTH_GOOD;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int status;
	int ret,soc,chrg_stat;

	if(ovp == 1){
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		val->intval = status;
		return 0;
	}
	soc = bq27x00_battery_read_rsoc(di);
	chrg_stat = bq27x00_read(di,BQ24192_REG8_STATUS, false);
/*
	ret = bq27x00_read(di, BQ24192_REG8_STATUS, true);

	if((ret >>4 & 0x2) == 0x2){
		if(soc == 100)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if((ret >>4 & 0x3) == 0x3)
		status = POWER_SUPPLY_STATUS_FULL;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;
*/

	ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (bat_status == 1 && soc == 100)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (!bat_status){
		if((chrg_stat >> 6 & 0x3) == 0x3)
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if((chrg_stat >> 6 & 0x1) || (chrg_stat >> 6 & 0x2)){
			status = POWER_SUPPLY_STATUS_CHARGING;
			printk("%s:status charging.\n",__func__);
		}
		else
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

/*
	if (di->chip == BQ27500) {
		if (di->cache.flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27500_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(&di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
*/

	printk("wyh %s:status %d.\n",__func__,status);
	val->intval = status;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int volt;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if (volt < 0)
		return volt;

	val->intval = volt * 1000;

	return 0;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_energy(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int ae;

	ae = bq27x00_read(di, BQ27x00_REG_AE, false);
	if (ae < 0) {
		dev_err(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27500)
		ae *= 1000;
	else
		ae = ae * 29200 / BQ27000_RS;

	val->intval = ae;

	return 0;
}

static void bq24192_charging_usb_changed(struct power_supply *psy,
				struct power_supply_charger_cap *cap)
{
	struct bq27x00_device_info *di = container_of(psy,
				struct bq27x00_device_info, usb);

	mutex_lock(&di->lock);
	di->cap.chrg_evt = cap->chrg_evt;
	di->cap.chrg_type = cap->chrg_type;
	di->cap.mA = cap->mA;
	mutex_unlock(&di->lock);

	if(di->cap.chrg_evt == POWER_SUPPLY_CHARGER_EVENT_CONNECT){
		switch(di->cap.chrg_type){
			case	POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			case	POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			case	POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
			{
				di->usb_online = 1;
				bat_status = 1 ;
				break;
			}
			default:
				break;
		}
	}else if(di->cap.chrg_evt == POWER_SUPPLY_CHARGER_EVENT_DISCONNECT){
		di->mains_online = 0;
		bat_status = 0 ;
	}

	printk("%s:[chrg] evt:%d type:%d cur:%d mains_online:%d usb_online:%d bat_status:%d\n",__func__,
				cap->chrg_evt, cap->chrg_type, cap->mA, di->mains_online,di->usb_online,bat_status);
	power_supply_changed(&di->usb);

	//schedule_delayed_work(&di->chrg_isr_wrkr, 0);
}
static void bq24192_charging_mains_changed(struct power_supply *psy,
				struct power_supply_charger_cap *cap)
{
	struct bq27x00_device_info *di = container_of(psy,
				struct bq27x00_device_info, mains);

	mutex_lock(&di->lock);
	di->cap.chrg_evt = cap->chrg_evt;
	di->cap.chrg_type = cap->chrg_type;
	di->cap.mA = cap->mA;
	mutex_unlock(&di->lock);

	/*if(di->cap.chrg_type == POWER_SUPPLY_TYPE_USB_DCP)*/
	if(di->cap.chrg_evt == POWER_SUPPLY_CHARGER_EVENT_CONNECT){
		switch(di->cap.chrg_type){
			case	POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
			{
				di->mains_online = 1;
				bat_status = 1 ;
				break;
			}
			default:
				break;
		}
	}else if(di->cap.chrg_evt == POWER_SUPPLY_CHARGER_EVENT_DISCONNECT){
		di->mains_online = 0;
		bat_status = 0 ;
	}

	printk("%s:[chrg] evt:%d type:%d cur:%d mains_online:%d usb_online:%d bat_status:%d\n",__func__,
				cap->chrg_evt, cap->chrg_type, cap->mA, di->mains_online,di->usb_online,bat_status);
	power_supply_changed(&di->mains);
	//schedule_delayed_work(&di->chrg_isr_wrkr, 0);
}
static void bq24192_ovp_mon(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, chrg_ovp_mon.work);
	int ret,usb_voltage;

	work_num +=1;
	if (!wake_lock_active(&di->ovp_wakelock))
		wake_lock(&di->ovp_wakelock);

	ret=ctp_get_usb_voltage(di,&usb_voltage);
	if(ret < 0)
		dev_err(di->dev, "failed to acquire usb voltage\n");
	if(ovp == 1 && ovp_flag == 0){
		power_supply_changed(&di->bat);
		ovp_flag = 1;
	}else if(ovp == 0 && ovp_flag == 1){
		power_supply_changed(&di->bat);
		ovp_flag = 0;
	}
/*
	if(di->cap.chrg_type && work_num <= 600){	//10 minutes
		schedule_delayed_work(&di->chrg_ovp_mon,(HZ * 1));
*/
	/*if(work_num <= 600 && (di->cap.chrg_type != POWER_SUPPLY_TYPE_BATTERY*/
						/*&& di->cap.chrg_type != POWER_SUPPLY_TYPE_USB_HOST)){	//10 minutes*/
	if(work_num <= 600 && (di->mains_online || di->usb_online)){	//10 minutes
		schedule_delayed_work(&di->chrg_ovp_mon,(HZ * 1));
	}else{
		if (wake_lock_active(&di->ovp_wakelock))
			wake_unlock(&di->ovp_wakelock);
		work_num = 0;
	}
}
static void bq24192_interrupt_worker(struct work_struct *work)
{

	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, chrg_isr_wrkr.work);

	dev_info(di->dev, "chrg event worker\n");
	bq27x00_update(di);
	charge_reg_show(di);

	/*if(di->cap.chrg_type == POWER_SUPPLY_TYPE_USB_DCP)*/
	/*{*/
		/*di->mains_online = 1;*/
	/*}*/
	/*else if (di->cap.chrg_type == POWER_SUPPLY_TYPE_USB){*/

		/*di->usb_online = 1;*/
	/*}else if(di->cap.chrg_type == POWER_SUPPLY_TYPE_BATTERY){*/
		/*di->mains_online = 0;*/
		/*di->usb_online = 0;*/
	/*}*/


/*
	int chrg_stat_reg;
	int chrg_data,chrg_data_1,chrg_data_2;
	printk("----------%s-----------\n",__func__);
	chrg_stat_reg = bq27x00_read_i2c(di, BQ24192_REG_CTL_0, 1);
	if(chrg_stat_reg < 0){
		printk("%s:charging type reg get error \n",__func__);
		return;
	}
	printk("%s:charging type reg is 0x%x\n",__func__,chrg_stat_reg);
	chrg_data = (chrg_stat_reg & BQ24192_0_STAT_0);
	chrg_data_1 = (chrg_stat_reg & BQ24192_0_STAT_1) << 1;
	chrg_data_2 = (chrg_stat_reg & BQ24192_0_STAT_2) << 2;
	switch(chrg_data_2+chrg_data_1+chrg_data)
	{
		case 0:
			printk("%s:No valid source detected\n",__func__);
			di->mains_online = 0;
			power_supply_changed(&di->mains);
			di->usb_online = 0;
			power_supply_changed(&di->usb);
			break;
		case 3:
			printk("%s:charging type is AC\n",__func__);
			di->mains_online = 1;
			power_supply_changed(&di->mains);
			break;
		case 4:
			printk("%s:charging type is USB\n",__func__);
			di->usb_online = 1;
			power_supply_changed(&di->usb);
			break;
		case 5:
			printk("%s:charging DONE\n",__func__);
			break;
		default:
			printk("%s: error chrg_ctl_data is %d \n",__func__,chrg_data_2+chrg_data_1+chrg_data);
	}
*/
	schedule_delayed_work(&di->chrg_isr_wrkr,(HZ * tim));
}


static irqreturn_t charge_irq(int irq, void *devid)
{
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)devid;
	dev_info(di->dev, "Charge ISR\n");
	schedule_delayed_work(&di->chrg_ovp_mon, msecs_to_jiffies(1000));
	schedule_delayed_work(&di->work, msecs_to_jiffies(delay_t));
	if (!wake_lock_active(&di->wakelock))
	wake_lock(&di->wakelock);
	work_count+=1;
	return IRQ_HANDLED;
}
static irqreturn_t fg_irq(int irq, void *devid)
{
	return IRQ_WAKE_THREAD;
}
static irqreturn_t fg_thread_handler(int irq, void *devid)
{
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)devid;
	int ret,chrg_fault;
	dev_info(di->dev, "FG_ISR\n");
	ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	dev_info(di->dev, "Flag REG ret = 0x%x\n",ret);
	chrg_fault = bq27x00_read(di, BQ24192_REG9_FAULT,true);
	if(chrg_fault)
		dev_info(di->dev,"Chrg reg09 ret = 0x%x\n",chrg_fault);
	return IRQ_HANDLED;
}
static int bq27x00_simple_value(int value,
		union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
		struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);
/*
	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);
*/
	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {

		case POWER_SUPPLY_PROP_STATUS:
			ret = bq27x00_battery_status(di, val);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = bq27x00_battery_voltage(di, val);
			break;

/*
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = di->cache.flags < 0 ? 0 : 1;
			break;
*/
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			ret = bq27x00_battery_current(di, val);
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if(di->cache.temperature <= 2331)
				val->intval = 49;
			else
				ret = bq27x00_simple_value(di->cache.capacity, val);
			break;

		case POWER_SUPPLY_PROP_TEMP:
			ret = bq27x00_battery_temperature(di, val);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			ret = bq27x00_battery_health(di,val);
			break;
/*
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
			ret = bq27x00_simple_value(di->cache.time_to_empty, val);
			break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
			break;
		case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
			ret = bq27x00_simple_value(di->cache.time_to_full, val);
			break;
*/
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
/*
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			ret = bq27x00_simple_value(di->cache.charge_full, val);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			ret = bq27x00_simple_value(di->charge_design_full, val);
			break;
*/
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			ret = bq27x00_simple_value(di->cache.cycle_count, val);
			break;

/*
		case POWER_SUPPLY_PROP_ENERGY_NOW:
			ret = bq27x00_battery_energy(di, val);
			break;
*/
		default:
			return -EINVAL;
	}
	return ret;
}
int fw_ver_get(struct bq27x00_device_info *di)
{

	int ret;
	unsigned char data[2]={0};

	data[0]=0x02;
	data[1]=0x00;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: status cmd write error\n",__func__);
	}
	mdelay(50);
	ret = bq27x00_read(di,BQ27530_REG_CNTL, false);
	printk("%s: FW ver ret is 0x%x\n",__func__,ret);

	return ret;
}
int df_ver_get(struct bq27x00_device_info *di)
{

	int ret;
	unsigned char data[2]={0};

	data[0]=0x1F;
	data[1]=0x00;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: status cmd write error\n",__func__);
	}
	mdelay(50);
	ret = bq27x00_read(di,BQ27530_REG_CNTL, false);
	printk("%s: DF ver ret is 0x%x\n",__func__,ret);
	return ret;
}
static char* bq24192_get_vendor(struct bq27x00_device_info *di)
{
	int ret;
	ret = bq27x00_read(di, BQ24192_REG_DEV_VER,1);
	if(ret <0){
		printk("%s:REG 0x%x read error code = 0x%x\n",__func__,BQ24192_REG_DEV_VER,ret);
		memcpy(di->chrg_vendor, "NULL", sizeof("NULL"));
		fw_stat = 0;
		return di->chrg_vendor;
	}
	ret = (ret >> 3) & 0x07;
	if(ret == BQ24190_IC_VERSION)
	{
		memcpy(di->chrg_vendor, "bq24190", sizeof("bq24190"));
		fw_stat = 1;
		return di->chrg_vendor;
	}
	else if (ret == BQ24192_IC_VERSION)
	{
		memcpy(di->chrg_vendor, "bq24192", sizeof("bq24192"));
		fw_stat = 1;
		return di->chrg_vendor;
	}
	else if (ret == BQ24192I_IC_VERSION)
	{
		memcpy(di->chrg_vendor, "bq24192i", sizeof("bq24192i"));
		fw_stat = 1;
		return di->chrg_vendor;
	}
	else
	{
		memcpy(di->chrg_vendor, "Unknown", sizeof("Unknown"));
		fw_stat = 0;
		return di->chrg_vendor;
	}
}

static int bq24912_mains_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq27x00_device_info *di =
		container_of(psy, struct bq27x00_device_info, mains);
	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = di->mains_online;
		return 0;
	}else if (psp == POWER_SUPPLY_PROP_MANUFACTURER){
		val->strval = bq24192_get_vendor(di);
		return 0;
	}
	return EINVAL;
}

static int bq24912_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq27x00_device_info *di =
		container_of(psy, struct bq27x00_device_info, usb);
	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = di->usb_online;
		return 0;
	}
	else if (psp == POWER_SUPPLY_PROP_MANUFACTURER){
		val->strval = bq24192_get_vendor(di);
		return 0;
	}
	return EINVAL;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

	int (*set_property)(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val);
int bq27x00_battery_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
    return 0;
}
int bq24912_mains_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
    return 0;
}
int bq24912_usb_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
    return 0;
}
static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.set_property = bq27x00_battery_set_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;

	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	mutex_init(&di->lock);
	wake_lock_init(&di->wakelock, WAKE_LOCK_SUSPEND,
						"ctp_charger_wakelock");

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	di->mains.name = "mains";
	di->mains.type = POWER_SUPPLY_TYPE_MAINS;
	di->mains.properties = bq24912_mains_props;
	di->mains.num_properties = ARRAY_SIZE(bq24912_mains_props);
	di->mains.get_property = bq24912_mains_get_property;
	di->mains.set_property = bq24912_mains_set_property;
	di->mains.charging_port_changed = bq24192_charging_mains_changed;
	ret = power_supply_register(di->dev, &di->mains);
	if (ret) {
		dev_err(di->dev, "failed to register mains: %d\n", ret);
		return ret;
	}
	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = bq24912_usb_props;
	di->usb.num_properties = ARRAY_SIZE(bq24912_usb_props);
	di->usb.get_property = bq24912_usb_get_property;
	di->usb.set_property = bq24912_usb_set_property;
	di->usb.charging_port_changed = bq24192_charging_usb_changed;
	ret = power_supply_register(di->dev, &di->usb);
	if (ret) {
		dev_err(di->dev, "failed to register usb: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	cancel_delayed_work_sync(&di->work);
	cancel_delayed_work_sync(&di->chrg_isr_wrkr);

	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->mains);
	power_supply_unregister(&di->usb);

	mutex_destroy(&di->lock);
}


/* i2c specific code */
//#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret,i;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0){
		for(i=0;i<3;i++){
			ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
			if(ret > 0)
				goto ret_true;
		}
		return ret;
	}
ret_true:
	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];
	return ret;
}

static u32 bq27530_read_i2c_4byte(struct bq27x00_device_info *di, u8 reg,int num)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[4]={0};
	u32 ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = 0x0B;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	//msg[1].addr = client->addr;
	msg[1].addr = 0x0B;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = num;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;
	ret = get_unaligned_le32(data);

	return ret;
}
static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, int num, unsigned char *buf,int rom_mode)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	int ret = 0;

	if (!client->adapter)
		return -ENODEV;
	if(rom_mode)
		msg[0].addr = 0x0B;
	else
		msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	if(rom_mode)
		msg[1].addr = 0x0B;
	else
		msg[1].addr = client->addr;
	//msg[1].addr = client->addr;
	msg[1].flags = 0;
	msg[1].buf = buf;
	msg[1].len = num;

	//printk("%s: i2c addr0 is 0x%x, addr1=0x%x \n",__func__,msg[0].addr,msg[1].addr);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;
	return ret;
}
int firmware_write(struct bq27x00_device_info *di)
{
	int i,j=0,t,offset,size;
	unsigned char data[32]={0};
	struct bqfs *bq;
	u32 ret;
	u32 fw_data;
	unsigned char reg;
	int num=0;

	data[0]=0x00;
	data[1]=0x0f;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);//Enter ROM Mode
	if(ret <0){
		printk("%s: reg 0x00 write error\n",__func__);
		return -1;
	}
	mdelay(1000);

	/*
	   volt = bq27x00_read_i2c_rom(di, 0x40, false);
	   printk("%s: reg 0x40 data= 0x%x\n",__func__,volt);

	   data[0]=0x0f;
	   ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,1,data,1);//Enter ROM Mode
	   if(ret <0)
	   printk("%s: reg 0x0 write error\n",__func__);
	   mdelay(10);
*/
/*
	   data[0]=0x0f;
	   ret=bq27x00_write_i2c(di,0x00,1,data,1);//Exit
	   if(ret <0)
	   printk("%s: 0x00 write error\n",__func__);
*/
/*
	   data[0]=0x0f;
	   ret=bq27x00_write_i2c(di,0x64,1,data,1);//Exit
	   if(ret <0)
	   printk("%s: 0x64 write error\n",__func__);
	   data[1]=0x00;
	   ret=bq27x00_write_i2c(di,0x65,1,data,1);//Exit ROM Mode
	   if(ret <0)
	   printk("%s: 0x65 write error\n",__func__);
	   mdelay(4000);
*/
/*
	   printk("%s: Exit rom mode\n",__func__);
		return 0;
*/
/*
	data[0] = 0;
	ret = bq27530_read_i2c_4byte(di,0x04,1);
	printk("%s: first read reg=0x04 data1=0x%x\n",__func__,ret);

	data[0] = ~ret ;
	ret=bq27x00_write_i2c(di,0x04,1,data,1);
	printk("%s: write data=0x%x\n",__func__,data[0]);

	ret = bq27530_read_i2c_4byte(di,0x04,1);
	printk("%s: second read reg=0x04 data1=0x%x\n",__func__,ret);
*/
	/*
	data[0] = ~ret ;
	printk("%s: write data=0x%x\n",__func__,data[0]);
	ret=bq27x00_write_i2c(di,0x04,1,data);
	printk("%s: write return =%d\n",__func__,ret);
	mdelay(10);
	ret = bq27x00_read(di, 0x04, 1);
	printk("%s: read data2=0x%x\n",__func__,ret);
	 
	volt = bq27x00_read(di, 0x08, false);
	printk("%s: reg=0x08 voltage = %d\n",__func__,volt*1000);
*/
write:	for(i=0;i<(sizeof(bqfs_index)/sizeof(*bq));i++)
	{
		for(j=0;j<32;j++)
			data[j]=0;
		j=0;
		t=0;
		offset = bqfs_index[i].data_offset;
		size = bqfs_index[i].data_size;

		if(bqfs_index[i].i2c_cmd == 'W')//write reg
		{
			reg = firmware_data[offset];
			j = offset;
			while(j<(offset+size-1)){
				//printk("%s:The reg= 0x%x data= 0x%x.\n",__func__,reg,firmware_data[j+1]);
				bq27x00_write_i2c(di,reg,1,&firmware_data[j+1],1);
				reg+=1;
				j++;
			}
		}else if(bqfs_index[i].i2c_cmd == 'X')//delay
		{
			//	mdelay(bqfs_index[i].i2c_addr);

			if(bqfs_index[i].i2c_addr < 10)
			//	mdelay(bqfs_index[i].i2c_addr);
				mdelay(bqfs_index[i].i2c_addr+fw_delay);
			else if(bqfs_index[i].i2c_addr == 170)
				mdelay(bqfs_index[i].i2c_addr+fw_long_delay);
			else
				mdelay(bqfs_index[i].i2c_addr+fw_more_delay);

/*
			if(bqfs_index[i].i2c_addr == 4000)
				printk("%s:delay = %d.\n",__func__,bqfs_index[i].i2c_addr);
*/
		}
		else if(bqfs_index[i].i2c_cmd == 'C')//compare data 
		{
			reg = firmware_data[offset];
			j = offset;
			ret = bq27530_read_i2c_4byte(di,reg,size-1);
			//printk("%s: Reg= 0x%x,mult_data= 0x%x\n",__func__,reg,ret);
/*
			if(size > 3){
				ret = bq27530_read_i2c_4byte(di,reg);
				printk("%s: Reg= 0x%x,mult_data= 0x%x\n",__func__,reg,ret);
			}
			else{
				ret = bq27x00_read_i2c_rom(di, reg,1);
				printk("%s: Reg= 0x%x,single_data= 0x%x\n",__func__,reg,ret);
			}
*/
			while(j<(offset+size-1)){
				data[t] = firmware_data[j+1];
				j++;
				t++;
			}
			fw_data = get_unaligned_le32(data);
			//printk("%s: Reg= 0x%x,fw_data= 0x%x\n",__func__,reg,fw_data);
			if(ret != fw_data){
				printk("%s:Error The reg= 0x%x data=0x%x fw_data=0x%x compare fault.\n",__func__,reg,ret,fw_data);
				if(num == 0){
				printk("%s: Exit rom mode\n",__func__);

				}else{
					num++;
				printk("%s: num = %d\n",__func__,num);
					goto write;
				}

				data[0]=0x0f;
				ret=bq27x00_write_i2c(di,0x64,1,data,1);//Exit ROM Mode
				if(ret <0)
					printk("%s: Reg 0x64 write error\n",__func__);
				data[1]=0x00;
				ret=bq27x00_write_i2c(di,0x65,1,data,1);//Exit ROM Mode
				if(ret <0)
					printk("%s: Reg 0x65 write error\n",__func__);
				return -2;
			}
		}
	}

	printk("%s: reset cmd write \n",__func__);
	data[0]=0x41;
	data[1]=0x00;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);/*Reset*/
	if(ret <0){
		printk("%s: reset cmd write error\n",__func__);
		return -1;
	}
	mdelay(1000);
#if 0
	printk("%s: itenable cmd write \n",__func__);
	data[0]=0x21;
	data[1]=0x00;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: itenable cmd write error\n",__func__);
		return -1;
	}
	printk("%s: reset cmd write \n",__func__);
	data[0]=0x41;
	data[1]=0x00;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: reset cmd write error\n",__func__);
		return -1;
	}
	mdelay(1000);

/*
	printk("%s: sealed cmd write \n",__func__);
	data[0]=0x20;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,1,data,0);
	if(ret <0){
		printk("%s: sealed cmd write error\n",__func__);
		return -1;
	}
*/
#endif
	return 0;

}

int otg_func_set(bool votg_on)
{

	int ret;
	unsigned char data[32]={0};

	if(bqdi == NULL)
	{
		pr_err("Failed enable BQ27xxx OTG func\n");
        //msleep(5);
        return -ENODEV;
	}
    pr_err("%s: run parameters %d\n", __func__, votg_on);
	if(votg_on){
		data[0]=0x15;
		data[1]=0x00;
		ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
            return -EIO;
		}
		gpio_direction_output(BQ24192_CHRG_OTG_GPIO, 1);
		mdelay(220);
	}else{
		data[0]=0x16;
		data[1]=0x00;
		ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
            return -EIO;
		}
		gpio_direction_output(BQ24192_CHRG_OTG_GPIO, 0);
		gpio_direction_input(BQ24192_CHRG_OTG_GPIO);
		mdelay(220);
	}
	return 1;
}

static int fg_reboot_notifier_call(struct notifier_block *notifier,
				     unsigned long what, void *data)
{
	if(bqdi < 0)
	{
		dev_err(bqdi->dev, "%s:FG driver was removed!\n",__func__);
		return NOTIFY_DONE;
	}
	disable_irq(bqdi->chrg_irq_n);
	disable_irq(bqdi->fg_irq_n);
	cancel_delayed_work_sync(&bqdi->work);
	cancel_delayed_work_sync(&bqdi->chrg_isr_wrkr);
	if(bqdi->chrg_irq_n)
		free_irq(bqdi->chrg_irq_n, bqdi);
	if(bqdi->fg_irq_n)
		free_irq(bqdi->fg_irq_n, bqdi);

	dev_info(bqdi->dev, "%s: The delay work and ISR was released for FG\n",__func__);

	return NOTIFY_DONE;

}
/*
static void otg_set(const char *val, struct kernel_param *kp)
{
	int ret;
	unsigned char data[32]={0};
	param_set_int(val,kp);

	if(bqdi < 0)
	{
		dev_err(bqdi->dev, "failed to init bq27530\n");
		return;
	}
	if(otg_switch == 1){
		otg_func_set(true);
		dev_info(bqdi->dev, "OTG Enable\n");
	}else{
		otg_func_set(false);
		dev_info(bqdi->dev, "OTG Disable\n");
	}
}
module_param_call(otg_switch,otg_set,NULL,&otg_switch,0644);
MODULE_PARM_DESC(otg_switch, "OTG Switch ");
*/
static int fw_write(const char *val, struct kernel_param *kp)
{
	int ret;
	param_set_int(val, kp);

	if(bc_fw == 1){
		ret = firmware_write(bqdi);
		if (!ret){
			dev_info(bqdi->dev, "FG FW download complete\n");
			fw_stat = 1;
		}
		else{
			dev_err(bqdi->dev, "failed to download FW: %d\n", ret);
			fw_stat = 0;
		}
	}
	else{
		dev_err(bqdi->dev, "Error!Only write 1 to download: %d\n", bc_fw);
	}
	return 0;
}

module_param_call(bc_fw,fw_write,NULL,&bc_fw,0644);
MODULE_PARM_DESC(bc_fw, "Download FW in FG and Charger");

static unsigned int ic_reg = 0;
static int bc_reg_show(const char *val, struct kernel_param *kp)
{
	int ret;
	param_set_int(val, kp);

	if(ic_reg < 0)
		return -1;

	if(ic_reg <= 0x73)
	{
		ret = bq27x00_read(bqdi,ic_reg,false);
		dev_info(bqdi->dev, "FG reg: 0x%x = 0x%x\n",ic_reg,ret);
		return 0;
	}
	else
	{
		ret = bq27x00_read(bqdi,ic_reg,true);
		dev_info(bqdi->dev, "CHRG reg: 0x%x = 0x%x\n",ic_reg,ret);
		return 0;
	}
}
module_param_call(ic_reg,bc_reg_show,NULL,&ic_reg,0644);
MODULE_PARM_DESC(ic_reg, "Download FW in FG and Charger");

static int ctp_get_usb_voltage(struct bq27x00_device_info *di,int *tmp)
{
	int gpadc_sensor_val;
	int ret,reg0;
	unsigned char data[32]={0};
/*
	di->gpadc_handle =
		intel_mid_gpadc_alloc(CLT_BATT_NUM_GPADC_SENSORS,
				adc_ch | CH_NEED_VCALIB
				);
	if (di->gpadc_handle == NULL) {
		dev_err(di->dev,
		 "ADC allocation failed: Check if ADC driver came up\n");
		return -1;
	}
*/
	if (!di->gpadc_handle) {
		ret = -ENODEV;
		return ret;
	}

	ret = intel_mid_gpadc_sample(di->gpadc_handle,
				CLT_GPADC_VOLTAGE_SAMPLE_COUNT,
				&gpadc_sensor_val);
	if (ret) {
		dev_err(di->dev,
			"adc driver api returned error(%d)\n", ret);
		ret = -1;
		return ret;
	}
	if(gpadc_sensor_val > 720)
	{
/*
		data[0]=0x1B;
		data[1]=0x00;
		ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);
*/
		reg0 = bq27x00_read(di, 0x75,true);
		data[0]=reg0 | 0x80;
		ret=bq27x00_write_i2c(di,0x75,1,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
			return -EIO;
		}
		reg0 = bq27x00_read(di, 0x75,true);
		dev_info(di->dev, "USB OVP is %d charging stop\n",gpadc_sensor_val);
		ovp=1;
	}
	else{
		if(ovp == 0)
			return ovp;
/*
		dev_info(di->dev, "VBUS State:%d\n", gpadc_sensor_val);
		data[0]=0x1A;
		data[1]=0x00;
		ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
			return -EIO;
		}
*/
		reg0 = bq27x00_read(di, 0x75,true);
		printk("%s: reg 0x75=0x%x\n",__func__,reg0);
		data[0]=reg0 & 0x7F;
		ret=bq27x00_write_i2c(di,0x75,1,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
			return -EIO;
		}
		reg0 = bq27x00_read(di, 0x75,true);
		printk("%s: reg 0x75= 0x%x\n",__func__,reg0);
		ovp=0;
	}
	return ovp;
}
extern char *saved_command_line;
static int bq27x00_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num;
	int retval = 0;
	int ret;
	int fw_ver;
	int bat_flag;
	int df_ver;
	char *pp = NULL;
	char *boot_reason= "charge";

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->id = num;
	di->dev = &client->dev;
	//di->chip = id->driver_data;
	di->chip = BQ27500;
	//di->bat.name = name;
	di->bat.name = "battery";
	di->bus.read = &bq27x00_read_i2c;

	i2c_set_clientdata(client, di);
	bqdi = di;
	//firmware_write(di);
	if (bq27x00_powersupply_init(di))
		goto batt_failed_3;

	ret = gpio_request(CHRG_INT_N, "FG_INT");
	if (ret) {
		dev_err(di->dev,
			"Failed to request gpio %d with error %d\n",
			FG_INT_N, ret);
	}else{
		di->chrg_irq_n = gpio_to_irq(CHRG_INT_N);
		ret = request_irq(di->chrg_irq_n,
			charge_irq,IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND, "bq24192",di);
		if (ret) {
			dev_err(di->dev, "cannot get IRQ:%d\n", di->chrg_irq_n);
		} else {
			dev_info(di->dev, "Chrg IRQ No:%d\n", di->chrg_irq_n);
			enable_irq_wake(di->chrg_irq_n);
		}
	}
	INIT_DELAYED_WORK(&di->chrg_isr_wrkr, bq24192_interrupt_worker);
	INIT_DELAYED_WORK(&di->chrg_ovp_mon, bq24192_ovp_mon);
	wake_lock_init(&di->ovp_wakelock, WAKE_LOCK_SUSPEND,
						"ovp_charger_wakelock");
	ret = penwell_otg_query_power_supply_cap(&di->cap);
	if (ret < 0) {
		dev_err(di->dev,
					"OTG Query failed. OTGD not loaded\n");
	} else {
		dev_info(di->dev, "Schedule the event worker\n");
		schedule_delayed_work(&di->chrg_isr_wrkr, 0);
	}
	ret = gpio_request(BQ24192_CHRG_OTG_GPIO, "CHRG_OTG");
	if (ret) {
		dev_err(di->dev,
			"Failed to request gpio %d with error %d\n",
			BQ24192_CHRG_OTG_GPIO, ret);
	}
	dev_info(di->dev, "request gpio %d for CHRG_OTG pin\n",
			BQ24192_CHRG_OTG_GPIO);

	ret = gpio_request(FG_INT_N, "FG_INT");
	if (ret) {
		dev_err(di->dev,
			"Failed to request gpio %d with error %d\n",
			FG_INT_N, ret);
	}else{
		di->fg_irq_n = gpio_to_irq(FG_INT_N);
		ret = request_threaded_irq(di->fg_irq_n,
				fg_irq,fg_thread_handler,
				IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND , "bq27531",di);
		if (ret) {
			dev_err(di->dev, "cannot get IRQ:%d\n", di->fg_irq_n);
		} else {
			dev_info(di->dev, "FG IRQ No:%d\n", di->fg_irq_n);
			enable_irq_wake(di->fg_irq_n);
		}
	}

	mdelay(100);
	bat_flag = bq27x00_read(di, BQ27x00_REG_FLAGS, false);

	fw_ver = fw_ver_get(di);
	mdelay(100);
	df_ver = df_ver_get(di);
	dev_info(di->dev, "0x0A REG ret = 0x%x\n",bat_flag);
	//if(fw_ver != 0x0004 ||((bat_flag >>4 & 0x1) == 0)){
	if(fw_ver != 0x0005 || df_ver != 0x1003){
		ret = firmware_write(di);
		if (!ret){
			dev_info(di->dev, "FG FW update complete\n");
			fw_stat = 1;
		}
		else{
			dev_err(di->dev, "failed to update FW: %d\n", ret);
			fw_stat = 0;
		}
	}
	pp=strstr(saved_command_line,boot_reason);
	if(pp)
		schedule_delayed_work(&di->chrg_ovp_mon, msecs_to_jiffies(1000));
	else
		dev_info(di->dev, "Normal Boot\n");
	pp = NULL;

	di->gpadc_handle =
		intel_mid_gpadc_alloc(CLT_BATT_NUM_GPADC_SENSORS,
				CLT_GPADC_USB_VOLTAGE | CH_NEED_VCALIB
				);
	if (di->gpadc_handle == NULL) {
		dev_err(di->dev,
		 "ADC allocation failed: Check if ADC driver came up\n");
		retval = -ENODEV;
	}

	kfree(name);

	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	intel_mid_gpadc_free(di->gpadc_handle);
	wake_lock_destroy(&di->wakelock);
	wake_lock_destroy(&di->ovp_wakelock);
	if(di->chrg_irq_n)
		free_irq(di->chrg_irq_n, di);
	if(di->fg_irq_n)
		free_irq(di->fg_irq_n, di);
	kfree(di);
	kfree(bqdi);

	return 0;
}

static const struct i2c_device_id bq27x00_id[] = {
	//{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27530", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

#ifdef CONFIG_PM
static int bq27x00_battery_suspend(struct device *dev)
{

	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	disable_irq(di->chrg_irq_n);
	cancel_delayed_work_sync(&di->chrg_isr_wrkr);

	dev_info(di->dev, "Bq27531 suspend\n");

	return 0;

}
static int bq27x00_battery_resume(struct device *dev)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	enable_irq(di->chrg_irq_n);
	schedule_delayed_work(&di->chrg_isr_wrkr, 0);

	dev_info(di->dev, "Bq27531 resume\n");
	return 0;
}
#else
#define bq27x00_battery_suspend NULL
#define bq27x00_battery_resume NULL
#endif
static const struct dev_pm_ops bq27x00_pm_ops = {
	.suspend		= bq27x00_battery_suspend,
	.resume			= bq27x00_battery_resume,
};

static struct notifier_block fg_reboot_notifier = {
	.notifier_call = fg_reboot_notifier_call,
};

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27530-battery",
		.owner	= THIS_MODULE,
		.pm = &bq27x00_pm_ops,
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	printk("%s:BQ27530 register_i2c driver\n",__func__);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

//#else

//static inline int bq27x00_battery_i2c_init(void) { return 0; }
//static inline void bq27x00_battery_i2c_exit(void) {};

//#endif

/* platform specific code */
#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM

static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
		bool single)
{
	struct device *dev = di->dev;
	struct bq27000_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int __devinit bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27000_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27500;

	di->bat.name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	return 0;

err_free:
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

static int __devexit bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static struct platform_driver bq27000_battery_driver = {
	.probe	= bq27000_battery_probe,
	.remove = __devexit_p(bq27000_battery_remove),
	.driver = {
		.name = "bq27000-battery",
		.owner = THIS_MODULE,
	},
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27000 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

#else

static inline int bq27x00_battery_platform_init(void) { return 0; }
static inline void bq27x00_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;
	struct bqfs *bq;
	ret = bq27x00_battery_i2c_init();


	printk("%s: bqfs_index is %d\n", __func__,(sizeof(bqfs_index)/sizeof(*bq)));

	if (register_reboot_notifier(&fg_reboot_notifier))
		pr_warning("fg: unable to register reboot notifier");
	/*
	   if (ret)
	   return ret;

	   ret = bq27x00_battery_platform_init();
	   if (ret)
	   bq27x00_battery_i2c_exit();
	 */
	return ret;
}
subsys_initcall(bq27x00_battery_init);
//module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	//	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
	unregister_reboot_notifier(&fg_reboot_notifier);
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
