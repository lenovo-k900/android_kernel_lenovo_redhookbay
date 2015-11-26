/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Jim Liu <jim.liu@intel.com>
 * Jackie Li<yaodong.li@intel.com>
 * Chao Jiang <chao.jiang@intel.com>
 */

#include "mdfld_output.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/r63311_vid.h"
#include <linux/gpio.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include "psb_drv.h"

extern int PanelID;
static u32 gpio_requested = 0;
static int CESwitch = 1;
static int CABCSwitch = 1;
static int VCOMSwitch = 0;

/*Define a atomic variable for CE/CABC switch controll*/
static atomic_t ce_cabc_counter = ATOMIC_INIT(0);

/*Define a panel kobject for sys interface*/
static struct kobject *panel_object;
//static int LCD_TYPE = 1;
//module_param (LCD_TYPE, byte, 0644);
//MODULE_PARM_DESC(LCD_TYPE, "LCD Panel type for querying");
//module_param_named(LCD_TYPE, LCD_TYPE, int, 0644);

/*LGD panel*/
static u8 r63311_mcs_protect_off[]      = {0xb0, 0x04};
static u8 r63311_nop_command[]          = {0x00};
static u8 r63311_nop_command1[]         = {0x00};
static u8 r63311_interface_setting[]    = {0xb3, 0x14, 0x00, 0x00,0x00,0x00,0x00};
static u8 r63311_dsi_control[]          = {0xb6, 0x3a, 0xd3};
static u8 r63311_cabc_parameter[]       = {0xb8, 0x18, 0x80, 0x18, 0x18, 0xcf, 0x1f, 0x00, 0x0c, 0x12, 0x6c, 0x11, 0x6c, 0x12, 0x0c, 0x12, 0xda, 0x6d, 0xff, 0xff, 0x10, 0x67, 0xa3, 0xdb, 0xfb, 0xff};
static u8 r63311_cabc_parameter1[]      = {0xb9, 0x00, 0x30, 0x18, 0x18, 0x9f, 0x1f, 0x80};
static u8 r63311_cabc_parameter2[]      = {0xba, 0x00, 0x30, 0x04, 0x40, 0x9f, 0x1f, 0xb7};  
static u8 r63311_color_enhance[]        = {0xca, 0x01, 0x80, 0xff, 0xff, 0xff, 0xff, 0xdc, 0xdc, 0x13, 0x20, 0x80, 0x80, 0x0a, 0x4a, 0x37, 0xa0, 0x55, 0xf8, 0x0c, 0x0c, 0x20, 0x10, 0x3f, 0x3f, 0x00, 0x00, 0x10, 0x10, 0x3f, 0x3f, 0x3f, 0x3f};
static u8 r63311_color_enhance_off[]    = {0xca, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x20, 0x80, 0x80, 0x0a, 0x4a, 0x37, 0xa0, 0x55, 0xf8, 0x0c, 0x0c, 0x20, 0x10, 0x3f, 0x3f, 0x00, 0x00, 0x10, 0x10, 0x3f, 0x3f, 0x3f, 0x3f};
static u8 r63311_display_setting1[]     = {0xc1, 0x84, 0x60, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x0c, 0x01, 0x58, 0x73, 0xae, 0x31, 0x20, 0x06,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x22, 0x02, 0x02, 0x00};
static u8 r63311_display_setting2[]    = {0xc2, 0x30, 0xf7, 0x80, 0x0c, 0x08, 0x00, 0x00};
static u8 r63311_source_timing_setting[] = {0xc4, 0x70, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x11, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x11, 0x06};
static u8 r63311_ltps_timing_setting[] = {0xc6, 0x06, 0x6d, 0x06, 0x6d, 0x06, 0x6d ,0x00, 0x00, 0x00, 0x00, 0x06, 0x6d, 0x06, 0x6d, 0x06, 0x6d, 0x15, 0x19, 0x07, 0x00, 0x01, 0x06, 0x6d, 0x06, 0x6d, 0x06, 0x6d, 0x00, 0x00, 0x00, 0x00, 0x06, 0x6d, 0x06, 0x6d, 0x06, 0x6d, 0x15, 0x19, 0x07};
static u8 r63311_gamma_setting_a[]      = {0xc7, 0x00, 0x15, 0x1c, 0x24, 0x31, 0x48, 0x3d, 0x51, 0x5a, 0x62, 0x66, 0x70, 0x00, 0x15, 0x1c, 0x24, 0x31, 0x48, 0x3d, 0x51, 0x5a, 0x62, 0x66, 0x70};
static u8 r63311_gamma_setting_b[]      = {0xc8, 0x00, 0x15, 0x1c, 0x24, 0x31, 0x49, 0x3d, 0x51, 0x5a, 0x62, 0x66, 0x70, 0x00, 0x15, 0x1c, 0x24, 0x31, 0x49, 0x3d, 0x51, 0x5a, 0x62, 0x66, 0x70};
static u8 r63311_gamma_setting_c[]      = {0xc9, 0x00, 0x15, 0x1c, 0x24, 0x31, 0x48, 0x3c, 0x50, 0x5a, 0x62, 0x66, 0x70, 0x00, 0x15, 0x1c, 0x24, 0x31, 0x48, 0x3c, 0x50, 0x5a, 0x62, 0x66, 0x70};
static u8 r63311_display_setting1_dual[]     = {0xc1, 0x84, 0x60, 0x40, 0xeb, 0xff, 0x6f, 0xce, 0xff,0xff, 0x0f, 0x01, 0x58, 0x73, 0xae, 0x31, 0x20, 0xc6,0xff, 0xff, 0x1f, 0xf3, 0xff, 0x5f, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x22, 0x02, 0x02, 0x00};
static u8 r63311_display_setting2_dual[]    = {0xc2, 0x31, 0xf7, 0x80, 0x0c, 0x08, 0x00, 0x00};
static u8 r63311_source_timing_setting_dual[] = {0xc4, 0x70, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x03, 0x00, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x03, 0x00, 0x0c, 0x06};
static u8 r63311_ltps_timing_setting_dual[] = {0xc6, 0x00, 0x79, 0x00, 0x79, 0x00, 0x79 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0x00, 0x79, 0x00, 0x79, 0x10, 0x19, 0x07, 0x00, 0x01, 0x00, 0x79, 0x00, 0x79, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0x00, 0x79, 0x00, 0x79, 0x10, 0x19, 0x07};
static u8 r63311_gamma_setting_a_dual[]      = {0xc7, 0x00, 0x0f, 0x1a, 0x23, 0x30, 0x48, 0x3f, 0x50, 0x5c, 0x69, 0x6f, 0x72, 0x00, 0x0f, 0x1a, 0x23, 0x30, 0x48, 0x3f, 0x50, 0x5c, 0x69, 0x6f, 0x72};
static u8 r63311_gamma_setting_b_dual[]      = {0xc8, 0x00, 0x0f, 0x19, 0x23, 0x2f, 0x48, 0x3f, 0x50, 0x5c, 0x68, 0x6b, 0x70, 0x00, 0x0f, 0x19, 0x23, 0x2f, 0x49, 0x3f, 0x50, 0x5c, 0x68, 0x6b, 0x70};
static u8 r63311_gamma_setting_c_dual[]      = {0xc9, 0x00, 0x0f, 0x1a, 0x22, 0x2f, 0x48, 0x3f, 0x51, 0x5f, 0x67, 0x6b, 0x70, 0x00, 0x0f, 0x1a, 0x22, 0x2f, 0x48, 0x3f, 0x51, 0x5f, 0x67, 0x6b, 0x70};
static u8 r63311_panel_pin_control_dual[]    = {0xcb, 0x31, 0xfc, 0x3f, 0x8c, 0x00, 0x00, 0x00, 0x00, 0xc0};
static u8 r63311_panel_interface_control[]      = {0xcc, 0x09};
static u8 r63311_power_setting[]       = {0xd0, 0x00, 0x00, 0x19, 0x18, 0x99, 0x99, 0x19, 0x01, 0x89, 0x00, 0x55, 0x19, 0x99, 0x01};
static u8 r63311_power_setting_for_internal[]     = {0xd3, 0x1b, 0x33, 0xbb, 0xcc, 0xc4, 0x33, 0x33, 0x33, 0x00, 0x01, 0x00, 0xa0, 0xd8, 0xa0, 0x0d, 0x41, 0x33, 0x44, 0x22, 0x70, 0x02, 0x41, 0x03, 0x3d, 0xbf, 0x00};
static u8 r63311_vcom_setting[]     = {0xd5, 0x06, 0x00, 0x00, 0x01, 0x33, 0x01, 0x33};
static u8 r63311_vcom_setting1[]     = {0xd5, 0x06, 0x00, 0x00, 0x01, 0x33, 0x01, 0x33};
static u8 r63311_vcom_setting_dual[]     = {0xd5, 0x06, 0x00, 0x00, 0x01, 0x29, 0x01, 0x29};
static u8 r63311_vcom_setting1_dual[]     = {0xd5, 0x06, 0x00, 0x00, 0x01, 0x29, 0x01, 0x29};
static u8 r63311_vcom_setting2[]     = {0xd5, 0x06, 0x00, 0x00, 0x01, 0x33, 0x01, 0x33};
static u8 r63311_vcom_setting3[]     = {0xd5, 0x06, 0x00, 0x00, 0x01, 0x33, 0x01, 0x33};
static u8 r63311_vcom_setting4[]     = {0xd5, 0x06, 0x00, 0x00, 0x00, 0x48, 0x00, 0x48};
static u8 r63311_vcom_setting5[]     = {0xd5, 0x06, 0x00, 0x00, 0x00, 0x48, 0x00, 0x48};
static u8 r63311_dimming_function[]    = {0xce, 0x00, 0x06, 0x00, 0xc1, 0x00, 0x0e, 0x14};
static u8 r63311_enable_pwm[]         = {0x53, 0x24};
static u8 r63311_cabc_function[]     = {0x55, 0x03};
static u8 r63311_cabc_off[]         = {0x55, 0x00};
static u8 r63311_exit_sleep_mode[]   = {0x11, 0x00, 0x00, 0x00};
static u8 r63311_set_display_off[]   = {0x28, 0x00, 0x00, 0x00};
static u8 r63311_enter_sleep_mode[]   = {0x10, 0x00, 0x00, 0x00};
static u8 r63311_display_on[]        = {0x29, 0x00, 0x00, 0x00};
static u8 r63311_deep_standby_in[]      = {0xb1, 0x01};

/*BOE panel*/

static u8 nt35596_code[] = {0xFF, 0x01, 0x00, 0x01,0x01, 0x55, 0x02, 0x40, 0x05, 0x60, 0x06, 0x1E, 0x07, 0x28, 0x08, 0x0C,0x0B, 0xA5, 0x0C, 0xA5, 0x0E, 0xA1, 0x0F, 0x4C, 0x10, 0x32, 0x11, 0x2C, 0x12, 0x2C, 0x13, 0x03, 0x14, 0x4A, 0x15, 0x12, 0x16, 0x12, 0x58, 0x84, 0x59, 0x04, 0x5A, 0x04, 0x5B, 0x04, 0x5C, 0x84, 0x5D, 0x84, 0x5E, 0x04, 0x5F, 0x04, 0x18, 0x00, 0x19, 0x77, 0x1A, 0x55, 0x1B, 0x13, 0x1C, 0x01, 0x1D, 0x00, 0x1E, 0x13, 0x1F, 0x00, 0x23, 0x00, 0x24, 0x00, 0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x28, 0x00, 0x60, 0x22, 0x6D, 0x33, 0x6B, 0x40, 0xFB, 0X01, 0xFB, 0x01, 0xFF, 0x05, 0x00, 0x00, 0x01, 0x04,0x02, 0x03, 0x03, 0x00, 0x04, 0x00, 0x05, 0x0B, 0x06, 0x0C, 0x07, 0x01, 0x08, 0x00, 0x09, 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x0C, 0x00, 0x0D, 0x17, 0x0E, 0x15, 0x0F, 0x13, 0x10, 0x00, 0x11, 0x04, 0x12, 0x03, 0x13, 0x00, 0x14, 0x00, 0x15, 0x0B, 0x16, 0x0C, 0x17, 0x01, 0x18, 0x00, 0x19, 0x00, 0x1A, 0x00, 0x1B, 0x00, 0x1C, 0x00, 0x1D, 0x17, 0x1E, 0x15, 0x1F, 0x13, 0x20, 0x00, 0x21, 0x01, 0x22, 0x09, 0x23, 0x73, 0x24, 0x73, 0x25, 0xED, 0x29, 0x38, 0x2A, 0x2B, 0x2B, 0x0A, 0x2F, 0x00, 0x30, 0x00, 0x31, 0x00, 0x32, 0x32, 0x33, 0x00, 0x34, 0x01, 0x35, 0x75, 0x36, 0x00, 0x37, 0x2D, 0x38, 0x18, 0x7A, 0x00, 0x7B, 0x11, 0x7C, 0x58, 0x7D, 0xE0, 0x7E, 0x05, 0x7F, 0x1A, 0x81, 0x08, 0x84, 0x05, 0x85, 0x08, 0x80, 0x02, 0x83, 0x05, 0x93, 0x04, 0x94, 0x05, 0x8A, 0x33, 0x9B, 0x0F, 0x9D, 0xB4, 0xC5, 0x32, 0xFB, 0x01, 0xFF, 0xEE,/* 0x24, 0x4F, 0x38, 0xC8,*/ 0x39, 0x20, 0x1E, 0x00, /*0x1D, 0x0F,*/ 0x7E, 0x01, 0x17, 0x5F, 0x46, 0x63, 0xFB, 0x01, 0xFF, 0x00, 0x36, 0x00, 0xD3, 0x05, 0xD4, 0x04, 0x35, 0x00};


//=yassy panel============
typedef struct 
{
	unsigned char add;
	unsigned char data;
}para_1_data;
para_1_data yt_code[] = {
         {0xFF, 0xEE},
         {0xFB, 0x01},
         {0x24, 0x4F},
         {0x38, 0xC8},
         {0x39, 0x27},
         {0x1E, 0x77},
         {0x1D, 0x0F},
         {0x7E, 0x71},
         {0x7C, 0x03},
         
         {0xFF, 0x01},
         {0xFB, 0x01},
         {0x00, 0x01},
         {0x01, 0x55},
         {0x02, 0x40},
         {0x05, 0x40},
         {0x06, 0x4A},
         {0x07, 0x24},
         {0x08, 0x0C},
         {0x0B, 0x7D},
         {0x0C, 0x7D},
         {0x0E, 0xB0},
         {0x0F, 0xAE},
         {0x11, 0x10},
         {0x12, 0x10},
         {0x13, 0x03},
         {0x14, 0x4A},
         {0x15, 0x12},
         {0x16, 0x12},
         {0x18, 0x00},
         {0x19, 0x77},
         {0x1A, 0x55},
         {0x1B, 0x13},
         {0x1C, 0x00},
         {0x1D, 0x00},
         {0x1E, 0x13},
         {0x1F, 0x00},
         {0x23, 0x00},
         {0x24, 0x00},
         {0x25, 0x00},
         {0x26, 0x00},
         {0x27, 0x00},
         {0x28, 0x00},
         {0x35, 0x00},
         {0x66, 0x00},
         {0x58, 0x82},
         {0x59, 0x02},
         {0x5A, 0x02},
         {0x5B, 0x02},
         {0x5C, 0x82},
         {0x5D, 0x82},
         {0x5E, 0x02},
         {0x5F, 0x02},
         {0x72, 0x31},
         
         {0xFF, 0x05},
         {0xFB, 0x01},
         {0x00, 0x01},
         {0x01, 0x0B},
         {0x02, 0x0C},
         {0x03, 0x09},
         {0x04, 0x0A},
         {0x05, 0x00},
         {0x06, 0x0F},
         {0x07, 0x10},
         {0x08, 0x00},
         {0x09, 0x00},
         {0x0A, 0x00},
         {0x0B, 0x00},
         {0x0C, 0x00},
         {0x0D, 0x13},
         {0x0E, 0x15},
         {0x0F, 0x17},
         {0x10, 0x01},
         {0x11, 0x0B},
         {0x12, 0x0C},
         {0x13, 0x09},
         {0x14, 0x0A},
         {0x15, 0x00},
         {0x16, 0x0F},
         {0x17, 0x10},
         {0x18, 0x00},
         {0x19, 0x00},
         {0x1A, 0x00},
         {0x1B, 0x00},
         {0x1C, 0x00},
         {0x1D, 0x13},
         {0x1E, 0x15},
         {0x1F, 0x17},
         {0x20, 0x00},
         {0x21, 0x03},
         {0x22, 0x01},
         {0x23, 0x40},
         {0x24, 0x40},
         {0x25, 0xED},
         {0x29, 0x58},
         {0x2A, 0x12},
         {0x2B, 0x01},
         {0x4B, 0x06},
         {0x4C, 0x11},
         {0x4D, 0x20},
         {0x4E, 0x02},
         {0x4F, 0x02},
         {0x50, 0x20},
         {0x51, 0x61},
         {0x52, 0x01},
         {0x53, 0x63},
         {0x54, 0x77},
         {0x55, 0xED},
         {0x5B, 0x00},
         {0x5C, 0x00},
         {0x5D, 0x00},
         {0x5E, 0x00},
         {0x5F, 0x15},
         {0x60, 0x75},
         {0x61, 0x00},
         {0x62, 0x00},
         {0x63, 0x00},
         {0x64, 0x00},
         {0x65, 0x00},
         {0x66, 0x00},
         {0x67, 0x00},
         {0x68, 0x04},
         {0x69, 0x00},
         {0x6A, 0x00},
         {0x6C, 0x40},
         {0x75, 0x01},
         {0x76, 0x01},
         {0x7A, 0x80},
         {0x7B, 0xA3},
         {0x7C, 0xD8},
         {0x7D, 0x60},
         {0x7F, 0x15},
         {0x80, 0x81},
         {0x83, 0x05},
         {0x93, 0x08},
         {0x94, 0x10},
         {0x8A, 0x00},
         {0x9B, 0x0F},
	 {0x9D, 0xB6},
         
         {0xFF, 0x01},
         {0xFB, 0x01},
         {0x75, 0x00},
         {0x76, 0xDF},
         {0x77, 0x00},
         {0x78, 0xE4},
         {0x79, 0x00},
         {0x7A, 0xED},
         {0x7B, 0x00},
         {0x7C, 0xF6},
         {0x7D, 0x00},
         {0x7E, 0xFF},
         {0x7F, 0x01},
         {0x80, 0x07},
         {0x81, 0x01},
         {0x82, 0x10},
         {0x83, 0x01},
         {0x84, 0x18},
         {0x85, 0x01},
         {0x86, 0x20},
         {0x87, 0x01},
         {0x88, 0x3D},
         {0x89, 0x01},
         {0x8A, 0x56},
         {0x8B, 0x01},
         {0x8C, 0x84},
         {0x8D, 0x01},
         {0x8E, 0xAB},
         {0x8F, 0x01},
         {0x90, 0xEC},
         {0x91, 0x02},
         {0x92, 0x22},
         {0x93, 0x02},
         {0x94, 0x23},
         {0x95, 0x02},
         {0x96, 0x55},
         {0x97, 0x02},
         {0x98, 0x8B},
         {0x99, 0x02},
         {0x9A, 0xAF},
         {0x9B, 0x02},
         {0x9C, 0xDF},
         {0x9D, 0x03},
         {0x9E, 0x01},
         {0x9F, 0x03},
         {0xA0, 0x2C},
         {0xA2, 0x03},
         {0xA3, 0x39},
         {0xA4, 0x03},
         {0xA5, 0x47},
         {0xA6, 0x03},
         {0xA7, 0x56},
         {0xA9, 0x03},
         {0xAA, 0x66},
         {0xAB, 0x03},
         {0xAC, 0x76},
         {0xAD, 0x03},
         {0xAE, 0x85},
         {0xAF, 0x03},
         {0xB0, 0x90},
         {0xB1, 0x03},
         {0xB2, 0xCB},
         {0xB3, 0x00},
         {0xB4, 0xDF},
         {0xB5, 0x00},
         {0xB6, 0xE4},
         {0xB7, 0x00},
         {0xB8, 0xED},
         {0xB9, 0x00},
         {0xBA, 0xF6},
         {0xBB, 0x00},
         {0xBC, 0xFF},
         {0xBD, 0x01},
         {0xBE, 0x07},
         {0xBF, 0x01},
         {0xC0, 0x10},
         {0xC1, 0x01},
         {0xC2, 0x18},
         {0xC3, 0x01},
         {0xC4, 0x20},
         {0xC5, 0x01},
         {0xC6, 0x3D},
         {0xC7, 0x01},
         {0xC8, 0x56},
         {0xC9, 0x01},
         {0xCA, 0x84},
         {0xCB, 0x01},
         {0xCC, 0xAB},
         {0xCD, 0x01},
         {0xCE, 0xEC},
         {0xCF, 0x02},
         {0xD0, 0x22},
         {0xD1, 0x02},
         {0xD2, 0x23},
         {0xD3, 0x02},
         {0xD4, 0x55},
         {0xD5, 0x02},
         {0xD6, 0x8B},
         {0xD7, 0x02},
         {0xD8, 0xAF},
         {0xD9, 0x02},
         {0xDA, 0xDF},
         {0xDB, 0x03},
         {0xDC, 0x01},
         {0xDD, 0x03},
         {0xDE, 0x2C},
         {0xDF, 0x03},
         {0xE0, 0x39},
         {0xE1, 0x03},
         {0xE2, 0x47},
         {0xE3, 0x03},
         {0xE4, 0x56},
         {0xE5, 0x03},
         {0xE6, 0x66},
         {0xE7, 0x03},
         {0xE8, 0x76},
         {0xE9, 0x03},
         {0xEA, 0x85},
         {0xEB, 0x03},
         {0xEC, 0x90},
         {0xED, 0x03},
         {0xEE, 0xCB},
         {0xEF, 0x00},
         {0xF0, 0xBB},
         {0xF1, 0x00},
         {0xF2, 0xC0},
         {0xF3, 0x00},
         {0xF4, 0xCC},
         {0xF5, 0x00},
         {0xF6, 0xD6},
         {0xF7, 0x00},
         {0xF8, 0xE1},
         {0xF9, 0x00},
         {0xFA, 0xEA},
         {0xFF, 0x02},
         {0xFB, 0x01},
         {0x00, 0x00},
         {0x01, 0xF4},
         {0x02, 0x00},
         {0x03, 0xFE},
         {0x04, 0x01},
         {0x05, 0x07},
         {0x06, 0x01},
         {0x07, 0x28},
         {0x08, 0x01},
         {0x09, 0x44},
         {0x0A, 0x01},
         {0x0B, 0x76},
         {0x0C, 0x01},
         {0x0D, 0xA0},
         {0x0E, 0x01},
         {0x0F, 0xE7},
         {0x10, 0x02},
         {0x11, 0x1F},
         {0x12, 0x02},
         {0x13, 0x22},
         {0x14, 0x02},
         {0x15, 0x54},
         {0x16, 0x02},
         {0x17, 0x8B},
         {0x18, 0x02},
         {0x19, 0xAF},
         {0x1A, 0x02},
         {0x1B, 0xE0},
         {0x1C, 0x03},
         {0x1D, 0x01},
         {0x1E, 0x03},
         {0x1F, 0x2D},
         {0x20, 0x03},
         {0x21, 0x39},
         {0x22, 0x03},
         {0x23, 0x47},
         {0x24, 0x03},
         {0x25, 0x57},
         {0x26, 0x03},
         {0x27, 0x65},
         {0x28, 0x03},
         {0x29, 0x77},
         {0x2A, 0x03},
         {0x2B, 0x85},
         {0x2D, 0x03},
         {0x2F, 0x8F},
         {0x30, 0x03},
         {0x31, 0xCB},
         {0x32, 0x00},
         {0x33, 0xBB},
         {0x34, 0x00},
         {0x35, 0xC0},
         {0x36, 0x00},
         {0x37, 0xCC},
         {0x38, 0x00},
         {0x39, 0xD6},
         {0x3A, 0x00},
         {0x3B, 0xE1},
         {0x3D, 0x00},
         {0x3F, 0xEA},
         {0x40, 0x00},
         {0x41, 0xF4},
         {0x42, 0x00},
         {0x43, 0xFE},
         {0x44, 0x01},
         {0x45, 0x07},
         {0x46, 0x01},
         {0x47, 0x28},
         {0x48, 0x01},
         {0x49, 0x44},
         {0x4A, 0x01},
         {0x4B, 0x76},
         {0x4C, 0x01},
         {0x4D, 0xA0},
         {0x4E, 0x01},
         {0x4F, 0xE7},
         {0x50, 0x02},
         {0x51, 0x1F},
         {0x52, 0x02},
         {0x53, 0x22},
         {0x54, 0x02},
         {0x55, 0x54},
         {0x56, 0x02},
         {0x58, 0x8B},
         {0x59, 0x02},
         {0x5A, 0xAF},
         {0x5B, 0x02},
         {0x5C, 0xE0},
         {0x5D, 0x03},
         {0x5E, 0x01},
         {0x5F, 0x03},
         {0x60, 0x2D},
         {0x61, 0x03},
         {0x62, 0x39},
         {0x63, 0x03},
         {0x64, 0x47},
         {0x65, 0x03},
         {0x66, 0x57},
         {0x67, 0x03},
         {0x68, 0x65},
         {0x69, 0x03},
         {0x6A, 0x77},
         {0x6B, 0x03},
         {0x6C, 0x85},
         {0x6D, 0x03},
         {0x6E, 0x8F},
         {0x6F, 0x03},
         {0x70, 0xCB},
         {0x71, 0x00},
         {0x72, 0x00},
         {0x73, 0x00},
         {0x74, 0x21},
         {0x75, 0x00},
         {0x76, 0x4C},
         {0x77, 0x00},
         {0x78, 0x6B},
         {0x79, 0x00},
         {0x7A, 0x85},
         {0x7B, 0x00},
         {0x7C, 0x9A},
         {0x7D, 0x00},
         {0x7E, 0xAD},
         {0x7F, 0x00},
         {0x80, 0xBE},
         {0x81, 0x00},
         {0x82, 0xCD},
         {0x83, 0x01},
         {0x84, 0x01},
         {0x85, 0x01},
         {0x86, 0x29},
         {0x87, 0x01},
         {0x88, 0x68},
         {0x89, 0x01},
         {0x8A, 0x98},
         {0x8B, 0x01},
         {0x8C, 0xE5},
         {0x8D, 0x02},
         {0x8E, 0x1E},
         {0x8F, 0x02},
         {0x90, 0x20},
         {0x91, 0x02},
         {0x92, 0x52},
         {0x93, 0x02},
         {0x94, 0x88},
         {0x95, 0x02},
         {0x96, 0xAA},
         {0x97, 0x02},
         {0x98, 0xD7},
         {0x99, 0x02},
         {0x9A, 0xF7},
         {0x9B, 0x03},
         {0x9C, 0x21},
         {0x9D, 0x03},
         {0x9E, 0x2E},
         {0x9F, 0x03},
         {0xA0, 0x3D},
         {0xA2, 0x03},
         {0xA3, 0x4C},
         {0xA4, 0x03},
         {0xA5, 0x5E},
         {0xA6, 0x03},
         {0xA7, 0x71},
         {0xA9, 0x03},
         {0xAA, 0x86},
         {0xAB, 0x03},
         {0xAC, 0x94},
         {0xAD, 0x03},
         {0xAE, 0xFA},
         {0xAF, 0x00},
         {0xB0, 0x00},
         {0xB1, 0x00},
         {0xB2, 0x21},
         {0xB3, 0x00},
         {0xB4, 0x4C},
         {0xB5, 0x00},
         {0xB6, 0x6B},
         {0xB7, 0x00},
         {0xB8, 0x85},
         {0xB9, 0x00},
         {0xBA, 0x9A},
         {0xBB, 0x00},
         {0xBC, 0xAD},
         {0xBD, 0x00},
         {0xBE, 0xBE},
         {0xBF, 0x00},
         {0xC0, 0xCD},
         {0xC1, 0x01},
         {0xC2, 0x01},
         {0xC3, 0x01},
         {0xC4, 0x29},
         {0xC5, 0x01},
         {0xC6, 0x68},
         {0xC7, 0x01},
         {0xC8, 0x98},
         {0xC9, 0x01},
         {0xCA, 0xE5},
         {0xCB, 0x02},
         {0xCC, 0x1E},
         {0xCD, 0x02},
         {0xCE, 0x20},
         {0xCF, 0x02},
         {0xD0, 0x52},
         {0xD1, 0x02},
         {0xD2, 0x88},
         {0xD3, 0x02},
         {0xD4, 0xAA},
         {0xD5, 0x02},
         {0xD6, 0xD7},
         {0xD7, 0x02},
         {0xD8, 0xF7},
         {0xD9, 0x03},
         {0xDA, 0x21},
         {0xDB, 0x03},
         {0xDC, 0x2E},
         {0xDD, 0x03},
         {0xDE, 0x3D},
         {0xDF, 0x03},
         {0xE0, 0x4C},
         {0xE1, 0x03},
         {0xE2, 0x5E},
         {0xE3, 0x03},
         {0xE4, 0x71},
         {0xE5, 0x03},
         {0xE6, 0x86},
         {0xE7, 0x03},
         {0xE8, 0x94},
         {0xE9, 0x03},
         {0xEA, 0xFA},
         
         
         {0xFF, 0x01},
         {0xFB, 0x01},
         {0xFF, 0x02},
         {0xFB, 0x01},
         {0xFF, 0x04},
         {0xFB, 0x01},
         {0xFF, 0x00},
         
         {0xD3, 0x0C},
         {0xD4, 0x09},
         {0xFF, 0xFF},
};

struct drm_display_mode *r63311_vid_get_config_mode(void)
{

	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 1080;
	mode->vdisplay = 1920;
	mode->hsync_start = 1080+120;
	mode->hsync_end = 1080+120+8;
	mode->htotal = 1080+120+8+60;
	mode->vsync_start = 1920+12;
	mode->vsync_end = 1920+12+4;
	mode->vtotal = 1920+12+4+12;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void r63311_vid_get_panel_info(int pipe,struct panel_info *pi)
{
	if(pipe == 0){
		pi->width_mm = TMD_PANEL_WIDTH;
		pi->height_mm = TMD_PANEL_HEIGHT;
	}

	PSB_DEBUG_ENTRY("lenovo\n");
}

/* ************************************************************************* *\
 * FUNCTION: mdfld_init_r63311_MIPI
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
static int mdfld_dsi_r63311_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u32 cmd = 0;
	u8 lcd_id = 0;
	u8 *pointer_lcd_id = &lcd_id;
	int i = 0, count;
	int new_pin_value, gpio69_value = -1, gpio176_value = -1;

	DRM_INFO("%s\n", __func__);
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");
	PSB_DEBUG_ENTRY("LCD Driver have been switched to R63311\n");

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	if(PanelID == R63311_VID)//check lcd type only one time
	{
		new_pin_value = gpio_get_value(79);	//panel_gpio_ID
		gpio69_value = gpio_get_value(69);
		gpio176_value = gpio_get_value(176);
		DRM_INFO("%s,the value of gpio 69 is %d\n", __func__, gpio69_value);
		DRM_INFO("%s,the value of gpio 176 is %d\n", __func__, gpio176_value);
		mdfld_dsi_read_gen_lp(sender,0xf4,0,1,pointer_lcd_id,1);
		if ( lcd_id == 0x96)
		{
			if(new_pin_value)
				PanelID = BOE_NT35596_1080P_VID;
			else
				PanelID = YAS_NT35596_1080P_VID;
		}
		else{
			if ( gpio69_value == 32 || gpio176_value == 0)
				PanelID = LG_R63311_1080P_VID_Dual;
			else
				PanelID = LG_R63311_1080P_VID;
				printk("[DRM]: chip ID from 0xf4 is 0x%x, PanelID=%d gpio79 is %x\n",lcd_id, PanelID, new_pin_value);
		}
	}

	if ( BOE_NT35596_1080P_VID == PanelID){
		for(i = 0 ;i < 257; i = i + 2)
		{
		mdfld_dsi_send_gen_long_lp(sender,&nt35596_code[i], 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
		udelay(5);
		}
		mdelay(10);
		mdfld_dsi_send_mcs_short_lp(sender,0x51,0xFF,1,0);
		mdfld_dsi_send_mcs_short_lp(sender,0x53,0x24,1,0);
		mdelay(10);
	}else if ( YAS_NT35596_1080P_VID == PanelID){
		while(1)
		{
		    if(yt_code[i].add == 0xff &&  yt_code[i].data == 0xff)
		            break;
		    mdfld_dsi_send_mcs_short_lp(sender,yt_code[i].add,yt_code[i].data,1,0);
		    udelay(10);
		    //printk("write %x  %x\n", yt_code[i].add, yt_code[i].data);
		    i++;
		}
		mdelay(10);
		mdfld_dsi_send_mcs_short_lp(sender,0x51,0xF0,1,0);
		mdfld_dsi_send_mcs_short_lp(sender,0x53,0x2c,1,0);
		mdelay(10);
	}else if ( PanelID == LG_R63311_1080P_VID ){
		mdfld_dsi_send_gen_long_hs(sender, r63311_mcs_protect_off, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_nop_command, 1, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_nop_command1, 1, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_interface_setting, 7, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_dsi_control, 3, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter, 26, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter1, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter2, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_display_setting1, 35, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_display_setting2, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_source_timing_setting, 23, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_ltps_timing_setting, 41, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_gamma_setting_a, 25, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_gamma_setting_b, 25, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_gamma_setting_c, 25, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_panel_interface_control, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_power_setting, 15, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_power_setting_for_internal, 27, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting1, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_dimming_function, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_enable_pwm, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
	}else {
		mdfld_dsi_send_gen_long_hs(sender, r63311_mcs_protect_off, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_nop_command, 1, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_nop_command1, 1, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_interface_setting, 7, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_dsi_control, 3, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter, 26, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter1, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter2, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_display_setting1_dual, 35, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_display_setting2_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_source_timing_setting_dual, 23, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_ltps_timing_setting_dual, 41, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_gamma_setting_a_dual, 25, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_gamma_setting_b_dual, 25, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_gamma_setting_c_dual, 25, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_panel_pin_control_dual, 10, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_short_hs(sender, 0x36, 0xc0, 1, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_panel_interface_control, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_power_setting, 15, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_power_setting_for_internal, 27, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting1_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_dimming_function, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_enable_pwm, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
	}

	gpio_direction_output(43, 1);
	mdelay(6);
	for (count = 0; count < 5; count++){
		gpio_direction_output(43, 0);
		ndelay(300);
		gpio_direction_output(43, 1);
		ndelay(300);
	}

	return 0;
}

static void
mdfld_dsi_r63311_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x1f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x35;//0x30;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x2;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->clk_lane_switch_time_cnt = 0x2B0014;

	/*420Mbps DSI data rate*/
	hw_ctx->dphy_param = 0x2A18681F;

	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}


static int mdfld_dsi_r63311_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	u32 cmd;
	PSB_DEBUG_ENTRY("lenovo\n");
	DRM_INFO("%s PanelID=%d\n", __func__, PanelID);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	atomic_sub(1, &ce_cabc_counter);

	/*send TURN_ON packet*/
	//err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,

	if ( PanelID == LG_R63311_1080P_VID || PanelID == LG_R63311_1080P_VID_Dual ){
		cmd = 0x29;
		err = mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
    		if(err)
    		{
		DRM_ERROR("Failed to send cmd 0x11\n");
        	return err;
   		}	
		mdelay(10);

		cmd = 0x11;
		err = mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
    		if(err)
    		{	
		DRM_ERROR("Failed to send cmd 0x29\n");
        	return err;
    		}
		
		err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,MDFLD_DSI_DPI_SPK_TURN_ON);
		if (err) {
			DRM_ERROR("Failed to send turn on packet\n");
			return err;
		}

		/*udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting2, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
		
		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting3, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;*/

	}else if ( PanelID == BOE_NT35596_1080P_VID ){
		err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,MDFLD_DSI_DPI_SPK_TURN_ON);
		if (err) {
			DRM_ERROR("Failed to send turn on packet\n");
			return err;
		}
		cmd = 0x29;
		err = mdfld_dsi_send_gen_long_hs(sender, &cmd, 1, 0);
    		if(err)
    		{
		DRM_ERROR("Failed to send cmd 0x11\n");
        	return err;
   		}	
		mdelay(10);

		cmd = 0x11;
		err = mdfld_dsi_send_gen_long_hs(sender, &cmd, 1, 0);
    		if(err)
    		{	
		DRM_ERROR("Failed to send cmd 0x29\n");
        	return err;
    		}
	}else	//yassy
	{
		err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,MDFLD_DSI_DPI_SPK_TURN_ON);
		if (err) {
			DRM_ERROR("Failed to send turn on packet\n");
			return err;
		}
		cmd = 0x11;
		err = mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
    		if(err)
    		{	
		DRM_ERROR("Failed to send cmd 0x29\n");
        	return err;
    		}
		mdelay(150);

		mdfld_dsi_send_mcs_short_lp(sender,0xff ,0x00 ,1,0);	//
		mdfld_dsi_send_mcs_short_lp(sender,0x34 ,0x00 ,1,0);	//Tearing Effect Line OFF
		mdfld_dsi_send_mcs_short_lp(sender,0x35 ,0x00 ,1,0);	//Tearing Effect Line ON
		
		cmd = 0x29;
		err = mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
    		if(err)
    		{
		DRM_ERROR("Failed to send cmd 0x11\n");
        	return err;
   		}	
		mdelay(50);

		mdfld_dsi_send_mcs_short_lp(sender,0x51 ,0xf0 ,1,0);	//
		mdfld_dsi_send_mcs_short_lp(sender,0x5e ,0x20 ,1,0);	//
		mdfld_dsi_send_mcs_short_lp(sender,0x53 ,0x2c ,1,0);	//
		mdfld_dsi_send_mcs_short_lp(sender,0x55 ,0x00 ,1,0);	//

	}
	mdelay(120);
	DRM_INFO("%s  end\n", __func__);
	return 0;
}

static int mdfld_dsi_r63311_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	struct drm_device *dev = dsi_config->dev;
        extern struct drm_device *gpDrmDevice;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) gpDrmDevice->dev_private;
	int err, ret;
	u32 cmd;
	PSB_DEBUG_ENTRY("lenovo\n");

	atomic_add(1, &ce_cabc_counter);

	PSB_DEBUG_ENTRY("Turn off video mode r63311 panel...\n");
	DRM_INFO("%s PanelID=%d\n", __func__, PanelID);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (!gpio_requested) {
		ret = gpio_request(57, "gfx");
		if (ret) {
			DRM_ERROR("Failed to request gpio lenovo %d\n", 57);
			return ret;
		}
		ret = gpio_request(55,"gfx2");
		if(ret){
			DRM_ERROR("Failed to request gpio lenovo %d\n", 55);
			return ret;
		}	
		ret = gpio_request(43, "gfx1");
		printk("lenovo:success\n");
		gpio_requested = 1;
	}

	gpio_direction_output(43,0);

	mdelay(5);
	if ( PanelID == LG_R63311_1080P_VID || PanelID == LG_R63311_1080P_VID_Dual)
	{
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting4, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting5, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
	}
	
	mdelay(50);
	cmd = 0x28;
	mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
	msleep(20);
	
	//mdelay(100);
	cmd = 0x10;
	//mdfld_dsi_send_gen_long_lp(sender, &cmd, 1, 0);
	//mdfld_dsi_send_mcs_long_lp(sender, &cmd, 1, 0);
	mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0); //ok, 5-6mA
	mdelay(100);

	if ( PanelID == LG_R63311_1080P_VID || PanelID == LG_R63311_1080P_VID_Dual)
	{
		//cmd = 0x01b1;
		mdfld_dsi_send_gen_long_hs(sender, r63311_deep_standby_in, 2, 0); 
		mdelay(40);
	}
	/*send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}
	mdelay(100);

	gpio_direction_output(57,0);
	mdelay(3);
	gpio_direction_output(55,0);
	mdelay(10);
	DRM_INFO("%s end\n", __func__);

	return 0;
}

static int mdfld_dsi_r63311_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");
	DRM_INFO("%s\n", __func__);

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return 0;
}

static int mdfld_dsi_r63311_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	u32 tmp, pipestat, dspcntr, dspstride, device_ready;
	int retry, err, count;
	uint32_t val, pipe0_enabled, pipe2_enabled;
	int ret = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct drm_device *dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	regs = &dsi_config->regs;
	dev = dsi_config->dev;
	PSB_DEBUG_ENTRY("lenovo\n");
	DRM_INFO("%s\n", __func__);


	if (!gpio_requested) {
		ret = gpio_request(57, "gfx");
		if (ret) {
			DRM_ERROR("Failed to request gpio lenovo %d\n", 57);
			return ret;
		}
		ret = gpio_request(55,"gfx2");
		if(ret){
			DRM_ERROR("Failed to request gpio lenovo %d\n", 55);
			return ret;
		}	
		ret = gpio_request(43, "gfx1");
		printk("lenovo:success\n");
		gpio_requested = 1;
	}

	gpio_direction_output(43, 0);
	mdelay(6);
	gpio_direction_output(57, 1);
	mdelay(5);
	gpio_direction_output(57, 0);
	gpio_direction_output(55, 1);
	mdelay(15);
	gpio_direction_output(57, 1);
	mdelay(5);

	return 0;
}

static int mdfld_dsi_r63311_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u16 backlight_value;
	u8 data[3]={0};
	u32 cmd = 0;
	//printk("Set brightness level %d...\n", level);
	DRM_INFO("%s  level=%d\n", __func__, level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if ( level < 20 && level != 0)
		level = 20;

	backlight_value = level & 0xff;
	data[0]=0x51;
	data[1]=0;
	data[2]=backlight_value;

	if(BOE_NT35596_1080P_VID == PanelID || YAS_NT35596_1080P_VID == PanelID)
		mdfld_dsi_send_mcs_short_lp(sender,0x51,backlight_value,1,0);
	else
		mdfld_dsi_send_mcs_long_hs(sender,data,3,0);
	
	mdelay(5);
	//mdfld_dsi_send_mcs_short_hs(sender,0x53,0x24,1,0);
	//mdelay(1);

	return 0;
}


void r63311_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	DRM_INFO("%s\n", __func__);
	PSB_DEBUG_ENTRY("\n");
	PSB_DEBUG_ENTRY("lenovo\n");
	p_funcs->get_config_mode = &r63311_vid_get_config_mode;
	p_funcs->get_panel_info = r63311_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_r63311_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_r63311_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_r63311_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_r63311_detect;
	p_funcs->power_on = mdfld_dsi_r63311_power_on;
	p_funcs->power_off = mdfld_dsi_r63311_power_off;
	p_funcs->set_brightness = mdfld_dsi_r63311_set_brightness;
}

static int do_dump_dc_register(const char *val, struct kernel_param *kp)
{
    int value;
    int ret = param_set_int(val, kp);

    if(ret < 0)
    {
        printk(KERN_ERR"Errored set cabc");
        return -EINVAL;
    }
    value = *((int*)kp->arg);

    if(value)
    {
        extern struct drm_device *gpDrmDevice;
        int reg = value;
        struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	    struct drm_device *dev = ((struct mdfld_dsi_config *)dev_priv->dsi_configs[0])->dev;

        for(; reg < value + 0x100; reg += 4)
            pr_err("0x%x  = 0x%x\n", reg, REG_READ(reg));    
    }

    return 0;
}
static int r63311_ce_switch(char *val, struct kernel_param *kp){
	int switcher;
	int ret = param_set_int(val, kp);

	if (ret < 0)
	{
		printk(KERN_ERR "Errored set CESwitch\n");
		return -EINVAL;
	}

	if (atomic_read(&ce_cabc_counter)) {
		DRM_INFO("Display already power off, ignore CE operation!\n");
		return -EPERM;
	}

	switcher = *((int*)kp->arg);

	if ( switcher < 0 || switcher > 1){
		CESwitch = -1;
		return 0;
	}

	DRM_INFO("Sending CE switch package\n");

	extern struct drm_device *gpDrmDevice;
	struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	struct mdfld_dsi_config *dsi_config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	struct mdfld_dsi_pkg_sender *sender =mdfld_dsi_get_pkg_sender(dsi_config);
	switch ( switcher) {
	case 0:
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance_off, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CESwitch = -1;
		else
			CESwitch = 0;
		break;
	case 1:
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CESwitch = -1;
		else
			CESwitch = 1;
		break;
	default:
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CESwitch = -1;
		else
			CESwitch = 1;
		break;
	}
	return 0;
}

static int r63311_vcom_set(char *val, struct kernel_param *kp){
	int switcher,cmd,err;
	int ret = param_set_int(val, kp);
	printk("*****%s\n",__func__);
	if(ret < 0)
	{
		printk(KERN_ERR "Errored: vcom setting\n");
		return -EINVAL;
	}

	switcher = *((int*)kp->arg);

	if(switcher < 0 || switcher >1){
		VCOMSwitch = -1;
		return 0;
	}

	extern struct drm_device *gpDrmDevice;
	struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	struct mdfld_dsi_config *dsi_config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	struct mdfld_dsi_pkg_sender *sender =mdfld_dsi_get_pkg_sender(dsi_config);

	switch (switcher){
	case 1:
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting4, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting5, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		mdelay(100);
		printk("[drm] %s vcom setting sent\n",__func__);
		break;
	case 0:
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting1_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
		break;
	default:
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting1_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
	}

	return 0;
}

static int r63311_cabc_switch(char *val, struct kernel_param *kp){
	int switcher;
	int ret = param_set_int(val, kp);

	if (ret < 0)
	{
		printk(KERN_ERR "Errored set CABCSwitch\n");
		return -EINVAL;
	}

	if (atomic_read(&ce_cabc_counter)) {
		DRM_INFO("Display already power off, ignore CABC operation!\n");
		return -EPERM;
	}

	switcher = *((int*)kp->arg);

	if ( switcher < 0 || switcher > 1){
		CABCSwitch = -1;
		return 0;
	}

	DRM_INFO("Sending CABC switch package\n");

	extern struct drm_device *gpDrmDevice;
	struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	struct mdfld_dsi_config *dsi_config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	struct mdfld_dsi_pkg_sender *sender =mdfld_dsi_get_pkg_sender(dsi_config);
	switch ( switcher ) {
	case 0:
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_off, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CABCSwitch = -1;
		else
			CABCSwitch = 0;
		break;
	case 1:
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CABCSwitch = -1;
		else
			CABCSwitch = 1;
		break;
	default:
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CABCSwitch = -1;
		else
			CABCSwitch = 1;
		break;
	}
	return 0;
	//do_nothing
}
static int dump_dc_register;
module_param_call(dump_dc_register, do_dump_dc_register, param_get_int, &dump_dc_register, S_IRUSR | S_IWUSR);
module_param_call(CESwitch, r63311_ce_switch, param_get_int, &CESwitch, S_IRUSR | S_IWUSR);
module_param_call(CABCSwitch, r63311_cabc_switch, param_get_int, &CABCSwitch, S_IRUSR | S_IWUSR);
module_param_call(VCOMSwitch, r63311_vcom_set, param_get_int, &VCOMSwitch, S_IRUSR | S_IWUSR);

static ssize_t r63311_vcom_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk("LCD: r63311_vcom_show\n");
}

static ssize_t r63311_vcom_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,size_t count)
{
	int para;

	para = *buf;
	//sscanf(buf,"%d",&para);
	printk("LCD: vcom received form user space:%d\n",para);

	if (atomic_read(&ce_cabc_counter)) {
		DRM_ERROR("Display already power off, ignore VCOM operation!\n");
		return -EPERM;
	}

	extern struct drm_device *gpDrmDevice;
	struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	struct mdfld_dsi_config *dsi_config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	struct mdfld_dsi_pkg_sender *sender =mdfld_dsi_get_pkg_sender(dsi_config);

	switch (para){
	case 1:
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting4, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting5, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		mdelay(100);
		printk("[drm] %s vcom setting sent\n",__func__);
		break;
	case 0:
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting1_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
		break;
	default:
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, r63311_vcom_setting1_dual, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
		}
	return count;	
}
static ssize_t r63311_cabc_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk("LCD: r63311_cabc_show\n");
}
static ssize_t r63311_cabc_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,size_t count)
{
	int para;

	//sscanf(buf,"%d",&para);
	para = *buf;
	printk("LCD: cabc received form user space:%d\n",para);

	if (atomic_read(&ce_cabc_counter)) {
	    DRM_ERROR("Display already power off, ignore CABC operation!\n");
		return -EPERM;
	}

	extern struct drm_device *gpDrmDevice;
	struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	struct mdfld_dsi_config *dsi_config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	struct mdfld_dsi_pkg_sender *sender =mdfld_dsi_get_pkg_sender(dsi_config);

	switch ( para ) {
	case 0:
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_off, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CABCSwitch = -1;
		else
			CABCSwitch = 0;
		break;
	case 1:
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CABCSwitch = -1;
		else
			CABCSwitch = 1;
		break;
	default:
		mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CABCSwitch = -1;
		else
			CABCSwitch = 1;
		break;
		}
	return count;
}
static ssize_t r63311_ce_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk("LCD: r63311_ce_show\n");
}

static ssize_t r63311_ce_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,size_t count)
{
	int para;

	//sscanf(buf,"%d",&para);
	para = *buf;
	printk("LCD: ce received form user space:%d\n",para);

	if (atomic_read(&ce_cabc_counter)) {
	    DRM_ERROR("Display already power off, ignore CE operation!\n");
		return -EPERM;
	}

	extern struct drm_device *gpDrmDevice;
	struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	struct mdfld_dsi_config *dsi_config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	struct mdfld_dsi_pkg_sender *sender =mdfld_dsi_get_pkg_sender(dsi_config);

	switch (para) {
	case 0:
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance_off, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CESwitch = -1;
		else
			CESwitch = 0;
		break;
	case 1:
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CESwitch = -1;
		else
			CESwitch = 1;
		break;
	default:
		mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			CESwitch = -1;
		else
			CESwitch = 1;
		break;
	}
	return count;
}
static struct kobj_attribute vcom_attribute= __ATTR(vcom, 0660, r63311_vcom_show, r63311_vcom_store);
static struct kobj_attribute cabc_attribute= __ATTR(cabc, 0660, r63311_cabc_show, r63311_cabc_store);
static struct kobj_attribute ce_attribute= __ATTR(ce, 0660, r63311_ce_show, r63311_ce_store);
static struct attribute *attrs[] = {
	&vcom_attribute.attr,
	&cabc_attribute.attr,
	&ce_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};
static int r63311_vid_lcd_probe(struct platform_device *pdev)
{
	int ret;
	DRM_INFO("%s: LGD_DUAL_SCAN panel detected\n", __func__);
	intel_mid_panel_register(r63311_vid_init);

	/*established the sys interface*/
	panel_object = kobject_create_and_add("panel", NULL);

	ret = sysfs_create_group(panel_object, &attr_group);
	if(ret)
		kobject_put(panel_object);

	return 0;
}

struct platform_driver r63311_lcd_driver = {
	.probe = r63311_vid_lcd_probe,
	.driver = {
		.name = "R63311_VID",
		.owner = THIS_MODULE,
	},
};

/*static int __init r63311_lcd_init(void)
{
	DRM_INFO("%s\n", __func__);
	return platform_driver_register(&r63311_lcd_driver);
}

module_init(r63311_lcd_init);*/
#if 0
static int do_read_ic_register(const char *val, struct kernel_param *kp)
{
    int value;
    int ret = param_set_int(val, kp);

    if(ret < 0)
    {
        printk(KERN_ERR"Errored set cabc");
        return -EINVAL;
    }
    value = *((int*)kp->arg);

    if(value)
    {
        extern struct drm_device *gpDrmDevice;
        int data = 0;
        int reg = value;
        struct drm_psb_private* dev_priv = gpDrmDevice->dev_private;
	    struct mdfld_dsi_pkg_sender *sender =
		    mdfld_dsi_get_pkg_sender(dev_priv->dsi_configs[0]);
        struct mdfld_dsi_config *config = (struct mdfld_dsi_config *)dev_priv->dsi_configs[0];
	    struct drm_device *dev = ((struct mdfld_dsi_config *)dev_priv->dsi_configs[0])->dev;
        int ret = mdfld_dsi_read_gen_lp(sender, reg & 0xff, 0, 1, &data, 1); 
        if(ret)
            DRM_ERROR("Failed to read ic register status: 0x%x\n", data); 
        pr_err("read ic register , 0x%x\n", data);
    }

    return 0;
}
static int read_ic_register;
module_param_call(read_ic_register, do_read_ic_register, param_get_int, &read_ic_register, S_IRUSR | S_IWUSR);
#endif
