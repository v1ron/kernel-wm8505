/*++
	linux/arch/arm/mach-wmt/gpio_cfg.c

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
#define GPIO_CFG_C

/*=== gpio_cfg.c ================================================================
*   Copyright (C) 2008  WonderMedia Technologies, Inc.
*
* MODULE       : gpio_cfg.c --
* AUTHOR       : Kenny Chou
* DATE         : 2009/01/07
* DESCRIPTION  : provide GPIO configure function for different EVB
*------------------------------------------------------------------------------*/

/*--- History -------------------------------------------------------------------
*Version 0.01 , Kenny Chou, 2009/01/07
*    First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY --------------------------------------*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <mach/gpio-cs.h>
#include <mach/gpio_if.h>


/*----------------------- INTERNAL PRIVATE MARCOS -----------------------------*/
/*#define DEBUG*/
#ifdef DEBUG
#define DBG(x...)	printk(KERN_ALERT x)
#else
#define DBG(x...)
#endif


/*----------------------- INTERNAL PRIVATE VARIABLES --------------------------*/
struct gpio_register_t gpio_func_ary[] = {
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT0}, /* GPIO_UART0_RTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT1}, /* GPIO_UART0_TXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT2}, /* GPIO_UART0_CTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT3}, /* GPIO_UART0_RXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT4}, /* GPIO_UART1_RTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT5}, /* GPIO_UART1_TXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT6}, /* GPIO_UART1_CTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT7}, /* GPIO_UART1_RXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT8}, /* GPIO_UART2_RTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT9}, /* GPIO_UART2_TXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT10}, /* GPIO_UART2_CTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT11}, /* GPIO_UART2_RXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT12}, /* GPIO_UART3_RTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT13}, /* GPIO_UART3_TXD*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT14}, /* GPIO_UART3_CTS*/
	{GPIO_ENABLE_CTRL_9_ADDR, GPIO_OUTPUT_ENABLE_9_ADDR,
	 GPIO_OUTPUT_DATA_9_ADDR, GPIO_INPUT_DATA_9_ADDR, BIT15}, /* GPIO_UART3_RXD*/
	{GPIO_ENABLE_CTRL_10_ADDR, GPIO_OUTPUT_ENABLE_10_ADDR,
	 GPIO_OUTPUT_DATA_10_ADDR, GPIO_INPUT_DATA_10_ADDR, BIT0}, /* GPIO_LED_BLUE*/
	{GPIO_ENABLE_CTRL_10_ADDR, GPIO_OUTPUT_ENABLE_10_ADDR,
	 GPIO_OUTPUT_DATA_10_ADDR, GPIO_INPUT_DATA_10_ADDR, BIT7}, /* GPIO_SD_PW*/
	{GPIO_ENABLE_CTRL_10_ADDR, GPIO_OUTPUT_ENABLE_10_ADDR,
	 GPIO_OUTPUT_DATA_10_ADDR, GPIO_INPUT_DATA_10_ADDR, BIT3}, /* GPIO_TS_INT*/
	{GPIO_ENABLE_CTRL_10_ADDR, GPIO_OUTPUT_ENABLE_10_ADDR,
	 GPIO_OUTPUT_DATA_10_ADDR, GPIO_INPUT_DATA_10_ADDR, BIT3}, /* GPIO_TS_SPI_INT*/
	{GPIO_ENABLE_CTRL_10_ADDR, GPIO_OUTPUT_ENABLE_10_ADDR,
	 GPIO_OUTPUT_DATA_10_ADDR, GPIO_INPUT_DATA_10_ADDR, BIT6}, /* GPIO_AUDIO_MUTE*/
};

/*********************UNABLE CUSTOMIZE GPIO**********************************************************
*
*1.  WMT KEYPAD drive use Dedicated GPIO0 GPIO1 GPIO6 to generate interrupt
*2.  IDE2CF driver Dedicated GPIO1 GPIO2 to generate interrupt
*
******************************************************************************************************/


/*----------------------- INTERNAL PRIVATE FUNCTIONS --------------------------*/
/*!*************************************************************************
* gpio_enable()
*
* Private Function by Kenny Chou, 2009/01/07
*/
/*!
* \brief	setting GPIO to input mode or output mode
*
* \retval
*/
/*!<; // gpio function name, set to input or output mode: 1 = input, 0 = output */
void gpio_enable(enum GPIO_FUNC_TYPE func, unsigned char mode)
{
	if (func >= GPIO_FUNC_MAX)
		return;

	/* set GPIO Enable Control Register */
	if (gpio_func_ary[func].enable_ctrl_reg_addr != 0)
		REG32_VAL(gpio_func_ary[func].enable_ctrl_reg_addr) |= gpio_func_ary[func].bit;

	/* set GPIO Output Control Register */
	if (gpio_func_ary[func].output_ctrl_reg_addr != 0) {
		if (mode == GPIO_OUTPUT_MODE)
			REG32_VAL(gpio_func_ary[func].output_ctrl_reg_addr) |= gpio_func_ary[func].bit;
		else if (mode == GPIO_INPUT_MODE)
			REG32_VAL(gpio_func_ary[func].output_ctrl_reg_addr) &= ~(gpio_func_ary[func].bit);
	}
}
EXPORT_SYMBOL(gpio_enable);


/*!*************************************************************************
* gpio_disable()
*
* Private Function by Kenny Chou, 2009/01/07
*/
/*!
* \brief	disable GPIO function
*
* \retval
*/
void gpio_disable(enum GPIO_FUNC_TYPE func)		/*!<; // gpio function name */
{
	if ((func >= GPIO_FUNC_MAX) || (gpio_func_ary[func].enable_ctrl_reg_addr == 0))
		return;

	/* set GPIO Enable Control Register */
	REG32_VAL(gpio_func_ary[func].enable_ctrl_reg_addr) &= ~(gpio_func_ary[func].bit);
}
EXPORT_SYMBOL(gpio_disable);


/*!*************************************************************************
* gpio_set_value()
*
* Private Function by Kenny Chou, 2009/01/07
*/
/*!
* \brief	setting GPIO at output high level or output low level
*
* \retval
*/
/*!<; // gpio function name, set to output high or output low: 1 = output high, 0 = output low  */
void gpio_set_value(enum GPIO_FUNC_TYPE func, int value)
{
	if ((func >= GPIO_FUNC_MAX) || (gpio_func_ary[func].output_data_reg_addr == 0))
		return;

	/* set GPIO Output Data Register */
	if (value == GPIO_OUTPUT_HIGH)
		REG32_VAL(gpio_func_ary[func].output_data_reg_addr) |= gpio_func_ary[func].bit;
	else if (value == GPIO_OUTPUT_LOW)
		REG32_VAL(gpio_func_ary[func].output_data_reg_addr) &= ~(gpio_func_ary[func].bit);

}
EXPORT_SYMBOL(gpio_set_value);


/*!*************************************************************************
* gpio_get_value()
*
* Private Function by Kenny Chou, 2009/01/07
*/
/*!
* \brief	read GPIO value when input mode
*
* \retval
*/
int gpio_get_value(enum GPIO_FUNC_TYPE func)		/*!<; // gpio function name */
{
	if ((func >= GPIO_FUNC_MAX) || (gpio_func_ary[func].input_data_reg_addr == 0))
		return GPIO_READ_ERR;

	/* set GPIO Input Data Register */
	if (REG32_VAL(gpio_func_ary[func].input_data_reg_addr) & gpio_func_ary[func].bit)
		return GPIO_INPUT_HIGH;
	else
		return GPIO_INPUT_LOW;
}
EXPORT_SYMBOL(gpio_get_value);

/*!*************************************************************************
* gpio_get_wakeup_para()
*
* Private Function by Kenny Chou, 2009/01/07
*/
/*!
* \brief
*
* \retval
*/
/*!<; // wakeup parameter for PMC module */
void gpio_get_wakeup_para(struct pmc_wakeup_para_s *pmc_wakeup_para)
{

	/*
	 *  Enable RTC & WAKEUP[0] for resume button & WAKEUP[1] for USB
	 */
	pmc_wakeup_para->wakeup_src = (PMWE_RTC | PMWE_WAKEUP(0) | PMWE_WAKEUP(1));
	pmc_wakeup_para->wakeup_type = (PMWT_WAKEUP0(PMWT_RISING) | PMWT_WAKEUP1(PMWT_FALLING));

}

/*!*************************************************************************
* excute_gpio_op()
*
* Private Function by HowayHuo, 2010/04/12
*/
/*!
* \brief	excute gpio operation
* \parameter:
* p: a gpio operation string. for example:
*     D8110064|C,D811008C|C,D81100B4&~0x4,D81100B4|0x8,D8130054|0x1
*
* \retval
*/
void excute_gpio_op(char * p)
{
    ulong   addr;
    ulong   val;
    char    op;
    char * endp;

    while(1)
    {
        addr = simple_strtoul(p, &endp, 16);
        if( *endp == '\0')
            break;

        op = *endp;
        if( endp[1] == '~')
        {
            val = simple_strtoul(endp+2, &endp, 16);
            val = ~val;
        }
        else
        {
            val = simple_strtoul(endp+1, &endp, 16);
        }

        printk("gpio op: 0x%lX %c 0x%lX\n", addr, op, val);
        switch(op)
        {
            case '|': REG32_VAL(addr) |= val; break;
            case '=': REG32_VAL(addr) = val; break;
            case '&': REG32_VAL(addr) &= val; break;
            default:
                printk("Error, Unknown operator %c\n", op);
        }

        if(*endp == '\0')
            break;
        p = endp + 1;
    }
    return;
}
EXPORT_SYMBOL(excute_gpio_op);

/*--------------------End of Function Body ------------------------------------*/

#undef GPIO_CFG_C
