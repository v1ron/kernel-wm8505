/*++
	linux/include/asm-arm/arch-wmt/wmt_gpio.h

	Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.

	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.

	WonderMedia Technologies, Inc.
	10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/

/* Be sure that virtual mapping is defined right */

#ifndef __ASM_ARCH_HARDWARE_H
#error "You must include hardware.h, not wmt_gpio.h"
#endif

#ifndef __WMT_GPIO_H
/* To assert that only one occurrence is included */
#define __WMT_GPIO_H

/*=== wmt_gpio.h ================================================================
*   Copyright (C) 2008  WonderMedia Technologies, Inc.
*
* MODULE       : wmt_gpio.h --
* AUTHOR       : Kenny Chou
* DATE         : 2009/01/07
* DESCRIPTION  : General Purpose Input/Output definition
*------------------------------------------------------------------------------*/

/*--- History -------------------------------------------------------------------
*Version 0.01 , Kenny Chou, 2009/01/07
*    First version
*
*Version 0.02 , Tommy Huang, 2009/01/19
*    Second version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY --------------------------------------*/
#ifndef APPLICATION
#else
#endif

#ifndef __ASM_ARCH_HARDWARE_H
#include <mach/hardware.h>
#endif
 #ifndef GPIO_CFG_IF_H
 #include <mach/gpio_if.h>
 #endif

/*-------------------- EXPORTED PRIVATE CONSTANTS -----------------------------*/

/*-------------------- EXPORTED PRIVATE TYPES----------------------------------*/

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef XXX_C /* allocate memory for variables only in xxx.c */
#       define EXTERN
#else
#       define EXTERN   extern
#endif /* ifdef XXX_C */


#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/
#ifdef GPIO_BASE_ADDR               /* From wmt_map.h*/
#define __GPIO_BASE      GPIO_BASE_ADDR
#else
#define __GPIO_BASE      0xD8110000
#endif

/*=============================================================================
//
// VT8610 GPIO registers.
//
// Registers Abbreviations:
//
// GPIO_ENABLE_CTRL_1_REG     		GPIO Enable Register for SD/MMC Signals
//
// GPIO_ENABLE_CTRL_2_REG     		GPIO Enable Register for VDIN/VDOUT Signals
//
// GPIO_ENABLE_CTRL_3_REG     		GPIO Enable Register for SYNC Signals
//
// GPIO_ENABLE_CTRL_4_REG     		GPIO Enable Register for NORD Signals
//
// GPIO_ENABLE_CTRL_5_REG     		GPIO Enable Register for NORA Signals
//
// GPIO_ENABLE_CTRL_6_REG     		GPIO Enable Register for AC97 Signals
//
// GPIO_ENABLE_CTRL_7_REG     		GPIO Enable Register for SPI Flash Signals
//
// GPIO_ENABLE_CTRL_8_REG     		GPIO Enable Register for SPI Signals
//
// GPIO_ENABLE_CTRL_9_REG     		GPIO Enable Register for UART Signals
//
// GPIO_ENABLE_CTRL_10_REG     		GPIO Enable Register for dedicated GPIO Signals
//
//-----------------------------------------------------------------------------------------------------
//
// GPIO_OUTPUT_ENABLE_1_REG     	GPIO Output Enable Register for SD/MMC Signals
//
// GPIO_OUTPUT_ENABLE_2_REG     	GPIO Output Enable Register for VDIN/VDOUT Signals
//
// GPIO_OUTPUT_ENABLE_3_REG     	GPIO Output Enable Register for SYNC Signals
//
// GPIO_OUTPUT_ENABLE_4_REG     	GPIO Output Enable Register for NORD Signals
//
// GPIO_OUTPUT_ENABLE_5_REG     	GPIO Output Enable Register for NORA Signals
//
// GPIO_OUTPUT_ENABLE_6_REG     	GPIO Output Enable Register for AC97 Signals
//
// GPIO_OUTPUT_ENABLE_7_REG			GPIO Output Enable Register for SPI Flash Signals
//
// GPIO_OUTPUT_ENABLE_8_REG     	GPIO Output Enable Register for SPI Signals
//
// GPIO_OUTPUT_ENABLE_9_REG     	GPIO Output Enable Register for UART Signals
//
// GPIO_OUTPUT_ENABLE_10_REG		GPIO Output Enable Register for dedicated GPIO Signals
//
//-----------------------------------------------------------------------------------------------------
//
// GPIO_OUTPUT_DATA_1_REG     		GPIO Output Data Register for SD/MMC Signals
//
// GPIO_OUTPUT_DATA_2_REG     		GPIO Output Data Register for VDIN/VDOUT Signals
//
// GPIO_OUTPUT_DATA_3_REG     		GPIO Output Data Register for SYNC Signals
//
// GPIO_OUTPUT_DATA_4_REG     		GPIO Output Data Register for NORD Signals
//
// GPIO_OUTPUT_DATA_5_REG     		GPIO Output Data Register for NORA Signals
//
// GPIO_OUTPUT_DATA_6_REG     		GPIO Output Data Register for AC97 Signals
//
// GPIO_OUTPUT_DATA_7_REG     		GPIO Output Data Register for SPI Flash Signals
//
// GPIO_OUTPUT_DATA_8_REG     		GPIO Output Data Register for SPI Signals
//
// GPIO_OUTPUT_DATA_9_REG     		GPIO Output Data Register for UART Signals
//
// GPIO_OUTPUT_DATA_10_REG     		GPIO Output Data Register for dedicated GPIO Signals
//
//-----------------------------------------------------------------------------------------------------
//
// GPIO_INPUT_DATA_1_REG     		GPIO Input Data Register for SD/MMC Signals
//
// GPIO_INPUT_DATA_2_REG     		GPIO Input Data Register for VDIN/VDOUT Signals
//
// GPIO_INPUT_DATA_3_REG     		GPIO Input Data Register for SYNC Signals
//
// GPIO_INPUT_DATA_4_REG     		GPIO Input Data Register for NORD Signals
//
// GPIO_INPUT_DATA_5_REG     		GPIO Input Data Register for NORA Signals
//
// GPIO_INPUT_DATA_6_REG    		GPIO Input Data Register for AC97 Signals
//
// GPIO_INPUT_DATA_7_REG    		GPIO Input Data Register for SPI Flash Signals
//
// GPIO_INPUT_DATA_8_REG     		GPIO Input Data Register for SPI Signals
//
// GPIO_INPUT_DATA_9_REG    		GPIO Input Data Register for UART Signals
//
// GPIO_INPUT_DATA_10_REG    		GPIO Input Data Register for dedicated GPIO Signals
//
//-----------------------------------------------------------------------------------------------------
//
// GPIO_STRAP_STATUS_REG    		Strapping Option Status Register
//
// GPIO_PMC_RTC_STATUS_REG    		PMC Suspend RTC Clock Exist Status Register
//
// GPIO_AHB_CTRL_REG   				AHB/Debug Control Register
//
// GPIO_SF_EDGE_CTRL_REG			SF Negative Edge Sampling Control Register
//
// GPIO_BINDING_STATUS_REG			Binding Option Status Register
//
// GPIO_PIN_SELECT_REG	  			Pin-Sharing Selection Register
//
// GPIO_INT_REQ_TYPE_REG	 		GPIO Interrupt Request Type Register
//
// GPIO_INT_REQ_STATUS_REG    		GPIO Interrupt Request Status Register
//
//-----------------------------------------------------------------------------------------------------
//
// GPIO_SD_STRENGTH_REG 			Secure Digital I/O Drive Strength and Slew Rate Register
//
// GPIO_VID_STRENGTH_REG   			VID I/O Drive Strength and Slew Rate Register
//
// GPIO_SPI_STRENGTH_REG   			SPI I/O Drive Strength and Slew Rate Register
//
// GPIO_NOR_STRENGTH_REG   			NOR I/O Drive Strength and Slew Rate Register
//
// GPIO_MMC_STRENGTH_REG   			MMC I/O Drive Strength and Slew Rate Register
//
// GPIO_CLKTST_STRENGTH_REG   		CLKTST I/O Drive Strength and Slew Rate Register
//
// GPIO_AC97_I2S_STRENGTH_REG   	AC97/I2S I/O Drive Strength and Slew Rate Register
//
//-----------------------------------------------------------------------------------------------------
//
// GPIO_ENABLE_CTRL_11_REG     		GPIO Enable Register for I2C/PWM Signals
//
// GPIO_OUTPUT_ENABLE_11_REG		GPIO Output Enable Register for I2C/PWM Signals
//
// GPIO_OUTPUT_DATA_11_REG     		GPIO Output Data Register for I2C/PWM Signals
//
// GPIO_INPUT_DATA_11_REG    		GPIO Input Data Register for I2C/PWM Signals
//
//=============================================================================
//=============================================================================
//
// Address constant for each register.
//
//=============================================================================*/
/* 16'h0000-16'h003F Reserved (Read-only, all zeros)*/
#define GPIO_ENABLE_CTRL_1_ADDR				(__GPIO_BASE + 0x0040)
#define GPIO_ENABLE_CTRL_2_ADDR				(__GPIO_BASE + 0x0044)
#define GPIO_ENABLE_CTRL_3_ADDR				(__GPIO_BASE + 0x0048)
#define GPIO_ENABLE_CTRL_4_ADDR				(__GPIO_BASE + 0x004C)
#define GPIO_ENABLE_CTRL_5_ADDR				(__GPIO_BASE + 0x0050)
#define GPIO_ENABLE_CTRL_6_ADDR				(__GPIO_BASE + 0x0054)
#define GPIO_ENABLE_CTRL_7_ADDR				(__GPIO_BASE + 0x0058)
#define GPIO_ENABLE_CTRL_8_ADDR				(__GPIO_BASE + 0x005C)
#define GPIO_ENABLE_CTRL_9_ADDR				(__GPIO_BASE + 0x0060)
#define GPIO_ENABLE_CTRL_10_ADDR			(__GPIO_BASE + 0x0064)
/*=============================================================================*/
#define GPIO_OUTPUT_ENABLE_1_ADDR			(__GPIO_BASE + 0x0068)
#define GPIO_OUTPUT_ENABLE_2_ADDR			(__GPIO_BASE + 0x006C)
#define GPIO_OUTPUT_ENABLE_3_ADDR			(__GPIO_BASE + 0x0070)
#define GPIO_OUTPUT_ENABLE_4_ADDR			(__GPIO_BASE + 0x0074)
#define GPIO_OUTPUT_ENABLE_5_ADDR			(__GPIO_BASE + 0x0078)
#define GPIO_OUTPUT_ENABLE_6_ADDR			(__GPIO_BASE + 0x007C)
#define GPIO_OUTPUT_ENABLE_7_ADDR			(__GPIO_BASE + 0x0080)
#define GPIO_OUTPUT_ENABLE_8_ADDR			(__GPIO_BASE + 0x0084)
#define GPIO_OUTPUT_ENABLE_9_ADDR			(__GPIO_BASE + 0x0088)
#define GPIO_OUTPUT_ENABLE_10_ADDR			(__GPIO_BASE + 0x008C)
/*=============================================================================*/
#define GPIO_OUTPUT_DATA_1_ADDR				(__GPIO_BASE + 0x0090)
#define GPIO_OUTPUT_DATA_2_ADDR				(__GPIO_BASE + 0x0094)
#define GPIO_OUTPUT_DATA_3_ADDR				(__GPIO_BASE + 0x0098)
#define GPIO_OUTPUT_DATA_4_ADDR				(__GPIO_BASE + 0x009C)
#define GPIO_OUTPUT_DATA_5_ADDR				(__GPIO_BASE + 0x00A0)
#define GPIO_OUTPUT_DATA_6_ADDR				(__GPIO_BASE + 0x00A4)
#define GPIO_OUTPUT_DATA_7_ADDR				(__GPIO_BASE + 0x00A8)
#define GPIO_OUTPUT_DATA_8_ADDR				(__GPIO_BASE + 0x00AC)
#define GPIO_OUTPUT_DATA_9_ADDR				(__GPIO_BASE + 0x00B0)
#define GPIO_OUTPUT_DATA_10_ADDR				(__GPIO_BASE + 0x00B4)
/*=============================================================================*/
#define GPIO_INPUT_DATA_1_ADDR				(__GPIO_BASE + 0x00B8)
#define GPIO_INPUT_DATA_2_ADDR				(__GPIO_BASE + 0x00BC)
#define GPIO_INPUT_DATA_3_ADDR				(__GPIO_BASE + 0x00C0)
#define GPIO_INPUT_DATA_4_ADDR				(__GPIO_BASE + 0x00C4)
#define GPIO_INPUT_DATA_5_ADDR				(__GPIO_BASE + 0x00C8)
#define GPIO_INPUT_DATA_6_ADDR				(__GPIO_BASE + 0x00CC)
#define GPIO_INPUT_DATA_7_ADDR				(__GPIO_BASE + 0x00D0)
#define GPIO_INPUT_DATA_8_ADDR				(__GPIO_BASE + 0x00D4)
#define GPIO_INPUT_DATA_9_ADDR				(__GPIO_BASE + 0x00D8)
#define GPIO_INPUT_DATA_10_ADDR				(__GPIO_BASE + 0x00DC)
/*=============================================================================*/
/* 16'h00E0-16'h00FF Reserved (Read-only, all zeros)*/
#define GPIO_STRAP_STATUS_ADDR				(__GPIO_BASE + 0x0100)
#define GPIO_PMC_RTC_STATUS_ADDR			(__GPIO_BASE + 0x0104)
#define GPIO_AHB_CTRL_ADDR					(__GPIO_BASE + 0x0108)
#define GPIO_SF_EDGE_CTRL_ADDR				(__GPIO_BASE + 0x010C)
#define GPIO_BINDING_STATUS_ADDR			(__GPIO_BASE + 0x0110)
/* 16'h0114-16'h01FF Reserved (Read-only, all zeros)*/
#define GPIO_PIN_SELECT_ADDR				(__GPIO_BASE + 0x0200)
/* 16'h0204-16'h02FF Reserved (Read-only, all zeros)*/
#define GPIO_INT_REQ_TYPE_ADDR				(__GPIO_BASE + 0x0300)
#define GPIO_INT_REQ_STATUS_ADDR			(__GPIO_BASE + 0x0304)
/* 16'h0308-16'h03FF Reserved (Read-only, all zeros)*/
/*=============================================================================*/
#define GPIO_SD_STRENGTH_ADDR				(__GPIO_BASE + 0x0400)
#define GPIO_VID_STRENGTH_ADDR				(__GPIO_BASE + 0x0404)
#define GPIO_SPI_STRENGTH_ADDR				(__GPIO_BASE + 0x0408)
#define GPIO_NOR_STRENGTH_ADDR				(__GPIO_BASE + 0x040C)
#define GPIO_MMC_STRENGTH_ADDR				(__GPIO_BASE + 0x0410)
#define GPIO_CLKTST_STRENGTH_ADDR			(__GPIO_BASE + 0x0414)
#define GPIO_AC97_I2S_STRENGTH_ADDR			(__GPIO_BASE + 0x0418)
/*=============================================================================*/
/* 16'h041C-16'h04FF Reserved (Read-only, all zeros)*/
#define GPIO_ENABLE_CTRL_11_ADDR			(__GPIO_BASE + 0x0500)
#define GPIO_OUTPUT_ENABLE_11_ADDR			(__GPIO_BASE + 0x0504)
#define GPIO_OUTPUT_DATA_11_ADDR			(__GPIO_BASE + 0x0508)
#define GPIO_INPUT_DATA_11_ADDR				(__GPIO_BASE + 0x050C)

/*=============================================================================
//
// Register pointer.
//
//=============================================================================*/
/* 16'h0000-16'h003F Reserved (Read-only, all zeros)*/
#define GPIO_ENABLE_CTRL_1_REG				(REG32_PTR(GPIO_ENABLE_CTRL_1_ADDR))
#define GPIO_ENABLE_CTRL_2_REG				(REG32_PTR(GPIO_ENABLE_CTRL_2_ADDR))
#define GPIO_ENABLE_CTRL_3_REG				(REG32_PTR(GPIO_ENABLE_CTRL_3_ADDR))
#define GPIO_ENABLE_CTRL_4_REG				(REG32_PTR(GPIO_ENABLE_CTRL_4_ADDR))
#define GPIO_ENABLE_CTRL_5_REG				(REG32_PTR(GPIO_ENABLE_CTRL_5_ADDR))
#define GPIO_ENABLE_CTRL_6_REG				(REG32_PTR(GPIO_ENABLE_CTRL_6_ADDR))
#define GPIO_ENABLE_CTRL_7_REG				(REG32_PTR(GPIO_ENABLE_CTRL_7_ADDR))
#define GPIO_ENABLE_CTRL_8_REG				(REG32_PTR(GPIO_ENABLE_CTRL_8_ADDR))
#define GPIO_ENABLE_CTRL_9_REG				(REG32_PTR(GPIO_ENABLE_CTRL_9_ADDR))
#define GPIO_ENABLE_CTRL_10_REG				(REG32_PTR(GPIO_ENABLE_CTRL_10_ADDR))
/*=============================================================================*/
#define GPIO_OUTPUT_ENABLE_1_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_1_ADDR))
#define GPIO_OUTPUT_ENABLE_2_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_2_ADDR))
#define GPIO_OUTPUT_ENABLE_3_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_3_ADDR))
#define GPIO_OUTPUT_ENABLE_4_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_4_ADDR))
#define GPIO_OUTPUT_ENABLE_5_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_5_ADDR))
#define GPIO_OUTPUT_ENABLE_6_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_6_ADDR))
#define GPIO_OUTPUT_ENABLE_7_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_7_ADDR))
#define GPIO_OUTPUT_ENABLE_8_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_8_ADDR))
#define GPIO_OUTPUT_ENABLE_9_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_9_ADDR))
#define GPIO_OUTPUT_ENABLE_10_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_10_ADDR))
/*=============================================================================*/
#define GPIO_OUTPUT_DATA_1_REG				(REG32_PTR(GPIO_OUTPUT_DATA_1_ADDR))
#define GPIO_OUTPUT_DATA_2_REG				(REG32_PTR(GPIO_OUTPUT_DATA_2_ADDR))
#define GPIO_OUTPUT_DATA_3_REG				(REG32_PTR(GPIO_OUTPUT_DATA_3_ADDR))
#define GPIO_OUTPUT_DATA_4_REG				(REG32_PTR(GPIO_OUTPUT_DATA_4_ADDR))
#define GPIO_OUTPUT_DATA_5_REG				(REG32_PTR(GPIO_OUTPUT_DATA_5_ADDR))
#define GPIO_OUTPUT_DATA_6_REG				(REG32_PTR(GPIO_OUTPUT_DATA_6_ADDR))
#define GPIO_OUTPUT_DATA_7_REG				(REG32_PTR(GPIO_OUTPUT_DATA_7_ADDR))
#define GPIO_OUTPUT_DATA_8_REG				(REG32_PTR(GPIO_OUTPUT_DATA_8_ADDR))
#define GPIO_OUTPUT_DATA_9_REG				(REG32_PTR(GPIO_OUTPUT_DATA_9_ADDR))
#define GPIO_OUTPUT_DATA_10_REG				(REG32_PTR(GPIO_OUTPUT_DATA_10_ADDR))
/*=============================================================================*/
#define GPIO_INPUT_DATA_1_REG				(REG32_PTR(GPIO_INPUT_DATA_1_ADDR))
#define GPIO_INPUT_DATA_2_REG				(REG32_PTR(GPIO_INPUT_DATA_2_ADDR))
#define GPIO_INPUT_DATA_3_REG				(REG32_PTR(GPIO_INPUT_DATA_3_ADDR))
#define GPIO_INPUT_DATA_4_REG				(REG32_PTR(GPIO_INPUT_DATA_4_ADDR))
#define GPIO_INPUT_DATA_5_REG				(REG32_PTR(GPIO_INPUT_DATA_5_ADDR))
#define GPIO_INPUT_DATA_6_REG				(REG32_PTR(GPIO_INPUT_DATA_6_ADDR))
#define GPIO_INPUT_DATA_7_REG				(REG32_PTR(GPIO_INPUT_DATA_7_ADDR))
#define GPIO_INPUT_DATA_8_REG				(REG32_PTR(GPIO_INPUT_DATA_8_ADDR))
#define GPIO_INPUT_DATA_9_REG				(REG32_PTR(GPIO_INPUT_DATA_9_ADDR))
#define GPIO_INPUT_DATA_10_REG				(REG32_PTR(GPIO_INPUT_DATA_10_ADDR))
/*=============================================================================*/
/* 16'h00E0-16'h00FF Reserved (Read-only, all zeros)*/
#define GPIO_STRAP_STATUS_REG				(REG32_PTR(GPIO_STRAP_STATUS_ADDR))
#define GPIO_PMC_RTC_STATUS_REG				(REG32_PTR(GPIO_PMC_RTC_STATUS_ADDR))
#define GPIO_AHB_CTRL_REG					(REG32_PTR(GPIO_AHB_CTRL_ADDR))
#define GPIO_SF_EDGE_CTRL_REG				(REG32_PTR(GPIO_SF_EDGE_CTRL_ADDR))
#define GPIO_BINDING_STATUS_REG				(REG32_PTR(GPIO_BINDING_STATUS_ADDR))
/* 16'h0114-16'h01FF Reserved (Read-only, all zeros)*/
#define GPIO_PIN_SELECT_REG					(REG32_PTR(GPIO_PIN_SELECT_ADDR))
/* 16'h0204-16'h02FF Reserved (Read-only, all zeros)*/
#define GPIO_INT_REQ_TYPE_REG				(REG32_PTR(GPIO_INT_REQ_TYPE_ADDR))
#define GPIO_INT_REQ_STATUS_REG				(REG32_PTR(GPIO_INT_REQ_STATUS_ADDR))
/* 16'h0308-16'h03FF Reserved (Read-only, all zeros)*/
/*=============================================================================*/
#define GPIO_SD_STRENGTH_REG				(REG32_PTR(GPIO_SD_STRENGTH_ADDR))
#define GPIO_VID_STRENGTH_REG				(REG32_PTR(GPIO_VID_STRENGTH_ADDR))
#define GPIO_SPI_STRENGTH_REG				(REG32_PTR(GPIO_SPI_STRENGTH_ADDR))
#define GPIO_NOR_STRENGTH_REG				(REG32_PTR(GPIO_NOR_STRENGTH_ADDR))
#define GPIO_MMC_STRENGTH_REG				(REG32_PTR(GPIO_MMC_STRENGTH_ADDR))
#define GPIO_CLKTST_STRENGTH_REG			(REG32_PTR(GPIO_CLKTST_STRENGTH_ADDR))
#define GPIO_AC97_I2S_STRENGTH_REG			(REG32_PTR(GPIO_AC97_I2S_STRENGTH_ADDR))
/*=============================================================================*/
/* 16'h041C-16'h04FF Reserved (Read-only, all zeros)*/
#define GPIO_ENABLE_CTRL_11_REG				(REG32_PTR(GPIO_ENABLE_CTRL_11_ADDR))
#define GPIO_OUTPUT_ENABLE_11_REG			(REG32_PTR(GPIO_OUTPUT_ENABLE_11_ADDR))
#define GPIO_OUTPUT_DATA_11_REG				(REG32_PTR(GPIO_OUTPUT_DATA_11_ADDR))
#define GPIO_INPUT_DATA_11_REG				(REG32_PTR(GPIO_INPUT_DATA_11_ADDR))

/*=============================================================================
//
// Register value.
//
//=============================================================================*/
/* 16'h0000-16'h003F Reserved (Read-only, all zeros)*/
#define GPIO_ENABLE_CTRL_1_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_1_ADDR))
#define GPIO_ENABLE_CTRL_2_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_2_ADDR))
#define GPIO_ENABLE_CTRL_3_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_3_ADDR))
#define GPIO_ENABLE_CTRL_4_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_4_ADDR))
#define GPIO_ENABLE_CTRL_5_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_5_ADDR))
#define GPIO_ENABLE_CTRL_6_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_6_ADDR))
#define GPIO_ENABLE_CTRL_7_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_7_ADDR))
#define GPIO_ENABLE_CTRL_8_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_8_ADDR))
#define GPIO_ENABLE_CTRL_9_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_9_ADDR))
#define GPIO_ENABLE_CTRL_10_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_10_ADDR))
/*=============================================================================*/
#define GPIO_OUTPUT_ENABLE_1_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_1_ADDR))
#define GPIO_OUTPUT_ENABLE_2_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_2_ADDR))
#define GPIO_OUTPUT_ENABLE_3_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_3_ADDR))
#define GPIO_OUTPUT_ENABLE_4_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_4_ADDR))
#define GPIO_OUTPUT_ENABLE_5_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_5_ADDR))
#define GPIO_OUTPUT_ENABLE_6_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_6_ADDR))
#define GPIO_OUTPUT_ENABLE_7_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_7_ADDR))
#define GPIO_OUTPUT_ENABLE_8_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_8_ADDR))
#define GPIO_OUTPUT_ENABLE_9_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_9_ADDR))
#define GPIO_OUTPUT_ENABLE_10_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_10_ADDR))
/*=============================================================================*/
#define GPIO_OUTPUT_DATA_1_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_1_ADDR))
#define GPIO_OUTPUT_DATA_2_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_2_ADDR))
#define GPIO_OUTPUT_DATA_3_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_3_ADDR))
#define GPIO_OUTPUT_DATA_4_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_4_ADDR))
#define GPIO_OUTPUT_DATA_5_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_5_ADDR))
#define GPIO_OUTPUT_DATA_6_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_6_ADDR))
#define GPIO_OUTPUT_DATA_7_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_7_ADDR))
#define GPIO_OUTPUT_DATA_8_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_8_ADDR))
#define GPIO_OUTPUT_DATA_9_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_9_ADDR))
#define GPIO_OUTPUT_DATA_10_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_10_ADDR))
/*=============================================================================*/
#define GPIO_INPUT_DATA_1_VAL				(REG32_VAL(GPIO_INPUT_DATA_1_ADDR))
#define GPIO_INPUT_DATA_2_VAL				(REG32_VAL(GPIO_INPUT_DATA_2_ADDR))
#define GPIO_INPUT_DATA_3_VAL				(REG32_VAL(GPIO_INPUT_DATA_3_ADDR))
#define GPIO_INPUT_DATA_4_VAL				(REG32_VAL(GPIO_INPUT_DATA_4_ADDR))
#define GPIO_INPUT_DATA_5_VAL				(REG32_VAL(GPIO_INPUT_DATA_5_ADDR))
#define GPIO_INPUT_DATA_6_VAL				(REG32_VAL(GPIO_INPUT_DATA_6_ADDR))
#define GPIO_INPUT_DATA_7_VAL				(REG32_VAL(GPIO_INPUT_DATA_7_ADDR))
#define GPIO_INPUT_DATA_8_VAL				(REG32_VAL(GPIO_INPUT_DATA_8_ADDR))
#define GPIO_INPUT_DATA_9_VAL				(REG32_VAL(GPIO_INPUT_DATA_9_ADDR))
#define GPIO_INPUT_DATA_10_VAL				(REG32_VAL(GPIO_INPUT_DATA_10_ADDR))
/*=============================================================================*/
/* 16'h00E0-16'h00FF Reserved (Read-only, all zeros)*/
#define GPIO_STRAP_STATUS_VAL				(REG32_VAL(GPIO_STRAP_STATUS_ADDR))
#define GPIO_PMC_RTC_STATUS_VAL				(REG32_VAL(GPIO_PMC_RTC_STATUS_ADDR))
#define GPIO_AHB_CTRL_VAL					(REG32_VAL(GPIO_AHB_CTRL_ADDR))
#define GPIO_SF_EDGE_CTRL_VAL				(REG32_VAL(GPIO_SF_EDGE_CTRL_ADDR))
#define GPIO_BINDING_STATUS_VAL				(REG32_VAL(GPIO_BINDING_STATUS_ADDR))
/* 16'h0114-16'h01FF Reserved (Read-only, all zeros)*/
#define GPIO_PIN_SELECT_VAL					(REG32_VAL(GPIO_PIN_SELECT_ADDR))
/* 16'h0204-16'h02FF Reserved (Read-only, all zeros)*/
#define GPIO_INT_REQ_TYPE_VAL				(REG32_VAL(GPIO_INT_REQ_TYPE_ADDR))
#define GPIO_INT_REQ_STATUS_VAL				(REG32_VAL(GPIO_INT_REQ_STATUS_ADDR))
/* 16'h0308-16'h03FF Reserved (Read-only, all zeros)*/
/*=============================================================================*/
#define GPIO_SD_STRENGTH_VAL				(REG32_VAL(GPIO_SD_STRENGTH_ADDR))
#define GPIO_VID_STRENGTH_VAL				(REG32_VAL(GPIO_VID_STRENGTH_ADDR))
#define GPIO_SPI_STRENGTH_VAL				(REG32_VAL(GPIO_SPI_STRENGTH_ADDR))
#define GPIO_NOR_STRENGTH_VAL				(REG32_VAL(GPIO_NOR_STRENGTH_ADDR))
#define GPIO_MMC_STRENGTH_VAL				(REG32_VAL(GPIO_MMC_STRENGTH_ADDR))
#define GPIO_CLKTST_STRENGTH_VAL			(REG32_VAL(GPIO_CLKTST_STRENGTH_ADDR))
#define GPIO_AC97_I2S_STRENGTH_VAL			(REG32_VAL(GPIO_AC97_I2S_STRENGTH_ADDR))
/*=============================================================================*/
/* 16'h041C-16'h04FF Reserved (Read-only, all zeros)*/
#define GPIO_ENABLE_CTRL_11_VAL				(REG32_VAL(GPIO_ENABLE_CTRL_11_ADDR))
#define GPIO_OUTPUT_ENABLE_11_VAL			(REG32_VAL(GPIO_OUTPUT_ENABLE_11_ADDR))
#define GPIO_OUTPUT_DATA_11_VAL				(REG32_VAL(GPIO_OUTPUT_DATA_11_ADDR))
#define GPIO_INPUT_DATA_11_VAL				(REG32_VAL(GPIO_INPUT_DATA_11_ADDR))
/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/

/*
 *  GPIO_INT_REQ_TYPE_REG
 */

#define GIRQ_LOW                0x00            /* Input zero generate GPIO_IRQ signal */
#define GIRQ_HIGH               0x01            /* Input one generate GPIO_IRQ signal */
#define GIRQ_FALLING            0x02            /* Falling edge generate GPIO_IRQ signal */
#define GIRQ_RISING             0x03            /* Rising edge generate GPIO_IRQ signal */
#define GIRQ_TYPEMASK           0x03
#define GIRQ_TYPE(idx, type)    ((type & GIRQ_TYPEMASK) << (idx * 2)) /* idx must be 0-9 */

#endif

/*=== END wmt_gpio.h ==========================================================*/

