/*++
	linux/arch/arm/mach-wmt/gpio_cfg.h

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

#ifndef GPIO_CFG_H
/* To assert that only one occurrence is included */
#define GPIO_CFG_H

/*=== gpio_cfg.h ================================================================
*   Copyright (C) 2008  WonderMedia Technologies, Inc.
*
* MODULE       : gpio_cfg.h --
* AUTHOR       : Kenny Chou
* DATE         : 2009/01/07
* DESCRIPTION  :
*------------------------------------------------------------------------------*/

/*--- History -------------------------------------------------------------------
*Version 0.01 , Kenny Chou, 2009/01/07
*    First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY --------------------------------------*/
#ifndef APPLICATION
#else
#endif

#ifndef __ASM_ARCH_HARDWARE_H
#include "hardware.h"
#endif

#ifndef __WMT_GPIO_H
#include "wmt_gpio.h"
#endif
/*-------------------- EXPORTED PRIVATE CONSTANTS -----------------------------*/


/*-------------------- EXPORTED PRIVATE TYPES----------------------------------*/
struct gpio_register_t {
	int enable_ctrl_reg_addr;
	int output_ctrl_reg_addr;
	int output_data_reg_addr;
	int input_data_reg_addr;
	int bit;
};


/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef GPIO_CFG_C /* allocate memory for variables only in GPIO_CFG_C */
#       define EXTERN
#else
#       define EXTERN   extern
#endif /* ifdef GPIO_CFG_C */


#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/


/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/


#endif

/*=== END gpio_cfg.h ==========================================================*/
