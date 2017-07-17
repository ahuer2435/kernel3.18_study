/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <asm-generic/gpio.h>
#include <mt-plat/upmu_common.h>
	
#include "lcm_drv.h"
#include "ddp_irq.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1200)
#define FRAME_HEIGHT (1920)



//#define GPIO_LCD_PWR_EN      GPIO3
static unsigned int GPIO_LCD_3V3_EN = 83;
static unsigned int GPIO_LCD_CABC_EN0 = 146;
static unsigned int GPIO_LCD_CABC_EN1 = 147;
//static unsigned int GPIO_LCD_GPID_EN = 125; 


#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static void init_lcm_registers(void)
{
	unsigned int data_array[1];


 			  
	
		   MDELAY(50); 
	
		   data_array[0]=0x00011500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   
	
		   MDELAY(50); 
	
	
	
		   data_array[0]=0XAA831500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0X11841500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x4BA91500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x00831500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x00841500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x00971500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x04851500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x08861500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x109C1500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	
		   data_array[0]=0x00111500; 
	
		   dsi_set_cmdq(data_array, 1, 1); 
	

		




}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static void lcd_power_en(unsigned char enabled)
{
#ifndef BUILD_LK
	printk("[IND][K] %s : %s\n", __func__, enabled ? "on" : "off");
#else
	printf("[IND][LK] %s : %s\n", __func__, enabled ? "on" : "off");
#endif

	if (enabled)
	{
		lcm_set_gpio_output(GPIO_LCD_3V3_EN, 1);
		
		lcm_set_gpio_output(GPIO_LCD_CABC_EN0, 1);

		lcm_set_gpio_output(GPIO_LCD_CABC_EN1, 1);
		
	}
	else
	{
		lcm_set_gpio_output(GPIO_LCD_3V3_EN, 0);
		
		lcm_set_gpio_output(GPIO_LCD_CABC_EN0, 0);

		lcm_set_gpio_output(GPIO_LCD_CABC_EN1, 0);
	}
	MDELAY(120);
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);
#else
	printf("[IND][LK] %s\n", __func__);
#endif

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.word_count=1200*3;
	params->dsi.mode   = BURST_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;//LCM_DSI_FORMAT_RGB666;
	//  params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;//LCM_PACKED_PS_18BIT_RGB666;

	params->dsi.vertical_sync_active				= 2;//50;
	params->dsi.vertical_backporch					= 25;//30;
	params->dsi.vertical_frontporch 				= 35;//36;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 32;//64;
	params->dsi.horizontal_backporch				= 60;//56;
	params->dsi.horizontal_frontporch				= 80;//60;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 490;
	// Bit rate calculation
	//params->dsi.pll_div1=0;		
	//params->dsi.pll_div2=1; 		
	//params->dsi.fbk_div =19;
	
	params->dsi.cont_clock = 1;
	params->dsi.ssc_disable = 1;
	//params->dsi.noncont_clock = 0;
	//params->dsi.noncont_clock_period = 2;
}

static void lcm_init(void)
{
#ifdef BUILD_LK
	printf("[IND][K] %s\n", __func__);
	lcd_power_en(1);
#else
	printk("[IND][LK] %s\n", __func__);
	lcd_power_en(1);
	
#endif
	printk("[+++auo+++][LK  --------@lgx----111-----boe-k1------------\n");
	//gpio_get_value(125);

	init_lcm_registers();
}

static void lcm_suspend(void)
{  
	unsigned int data_array[64];
	
	data_array[0] = 0x00023902;
	data_array[1] = 0x00000010;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(20);

#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);

	lcd_power_en(0);
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	printk("[IND][K] %s\n", __func__);

	lcm_init();
#endif
}
LCM_DRIVER nt35595_fhd_dsi_cmd_truly_nt50358_lcm_drv = {
	.name = "nt35595_fhd_dsi_cmd_truly_nt50358_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
};

