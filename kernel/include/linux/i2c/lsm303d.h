/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name          : lsm303d.h
* Authors            : MSH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
* Version            : V.2.0.0
* Date               : 2012/May/09
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.

Revision 2-0-0 2012/05/09
 first revision
*******************************************************************************/

#ifndef	__LSM303D_H__
#define	__LSM303D_H__

#define	LSM303D_DEV_NAME	"lsm303d"

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	LSM303D_ACC_FS_MASK	(0x18)
#define LSM303D_ACC_FS_2G 	(0x00)	/* Full scale 2g */
#define LSM303D_ACC_FS_4G 	(0x08)	/* Full scale 4g */
#define LSM303D_ACC_FS_8G 	(0x10)	/* Full scale 8g */
#define LSM303D_ACC_FS_16G	(0x18)	/* Full scale 16g */

/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

/* Magnetometer Sensor Full Scale */
#define LSM303D_MAG_FS_MASK	(0x60)
#define LSM303D_MAG_FS_2G	(0x00)	/* Full scale 2 gauss */
#define LSM303D_MAG_FS_4G	(0x20)	/* Full scale 4 gauss */
#define LSM303D_MAG_FS_8G	(0x40)	/* Full scale 8 gauss */
#define LSM303D_MAG_FS_12G	(0x60)	/* Full scale 12 gauss */


#ifdef	__KERNEL__

#define	LSM303D_ACC_DEV_NAME	"lsm303d_acc"	/* Input file name */
#define	LSM303D_MAG_DEV_NAME	"lsm303d_mag"	/* Input file name */
/*Begin:liuyx4 Add lv sensor for double click--20130106*/
#define	LSM303D_LV_DEV_NAME		"lsm303d_lv"	/* Input file name */
/*End*/

#define	LSM303D_ACC_MIN_POLL_PERIOD_US	10000
#define LSM303D_MAG_MIN_POLL_PERIOD_US	10000

struct lsm303d_acc_platform_data {
	
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct lsm303d_mag_platform_data {

	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct lsm303d_main_platform_data {
	
	struct lsm303d_acc_platform_data *pdata_acc;
	struct lsm303d_acc_platform_data *pdata_mag;
};

#endif	/* __KERNEL__ */
#endif	/* __LSM303DC_H__ */
