/*++
	linux/include/asm-arm/arch-wmt/gpio_cfg_if.h

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

#ifndef GPIO_CFG_IF_H
/* To assert that only one occurrence is included */
#define GPIO_CFG_IF_H

/*=== gpio_cfg_if.h ================================================================
*   Copyright (C) 2008  WonderMedia Technologies, Inc.
*
* MODULE       : gpio_cfg_if.h --
* AUTHOR       : Kenny Chou
* DATE         : 2009/01/07
* DESCRIPTION  : provide API function to someone who use GPIO function
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


/*-------------------- EXPORTED PRIVATE CONSTANTS -----------------------------*/


/*-------------------- EXPORTED PRIVATE TYPES----------------------------------*/
/* GPIO function ID */
enum GPIO_FUNC_TYPE{
	GPIO_UART0_RTS,
	GPIO_UART0_TXD,
	GPIO_UART0_CTS,
	GPIO_UART0_RXD,
	GPIO_UART1_RTS,
	GPIO_UART1_TXD,
	GPIO_UART1_CTS,
	GPIO_UART1_RXD,
	GPIO_UART2_RTS,
	GPIO_UART2_TXD,
	GPIO_UART2_CTS,
	GPIO_UART2_RXD,
	GPIO_UART3_RTS,
	GPIO_UART3_TXD,
	GPIO_UART3_CTS,
	GPIO_UART3_RXD,
	GPIO_LED_BLUE,
	GPIO_SD_PW,
	GPIO_TS_INT,
	GPIO_TS_SPI_INT,
	GPIO_AUDIO_MUTE,
	GPIO_FUNC_MAX
};


/* wakeup parameter for PMC */
struct pmc_wakeup_para_s {
    int wakeup_src;
	int wakeup_type;
};


/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef XXX_C /* allocate memory for variables only in xxx.c */
#       define EXTERN
#else
#       define EXTERN   extern
#endif /* ifdef XXX_C */


#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/
#define GPIO_INPUT_MODE		1
#define GPIO_OUTPUT_MODE		0

#define GPIO_OUTPUT_HIGH		1
#define GPIO_OUTPUT_LOW		0

#define GPIO_INPUT_HIGH		1
#define GPIO_INPUT_LOW			0

#define GPIO_READ_ERR			0xffffffff

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
void gpio_enable(enum GPIO_FUNC_TYPE func, unsigned char mode);
void gpio_disable(enum GPIO_FUNC_TYPE func);
void gpio_set_value(enum GPIO_FUNC_TYPE func, int value);
int gpio_get_value(enum GPIO_FUNC_TYPE func);
void excute_gpio_op(char * p);

void gpio_get_wakeup_para(struct pmc_wakeup_para_s *pmc_wakeup_para);



#endif

/*=== END gpio_cfg_if.h ==========================================================*/
