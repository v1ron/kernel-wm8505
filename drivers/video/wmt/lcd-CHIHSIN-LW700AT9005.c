/**************************************************************		
Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.	
This program is free software: you can redistribute it and/or modify it under the terms 	
of the GNU General Public License as published by the Free Software Foundation, either
 	version 2 of the License, or (at your option) any later version.
	
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the GNU General Public License for more details. You should have received
 a copy of the GNU General Public License along with this program.  If not, see
 <http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.

--*/
#define LCD_CHIHSIN_LW700AT9005_C

/*=== lcd-oem.c =============================================================
*
* MODULE       : lcd-CHIHSIN-LW700AT9005.c
* AUTHOR       : howayhuo
* DATE         : 2010/03/27
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------*
Version 0.01 , howayhuo, 2010/03/27
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include "lcd.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define LCD_LW700AT9005_XXXX    1     *//*Example*/
#define DEBUG
#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBGMSG(fmt, args...)   DPRINT("[CHIHSIN LW700AT9005] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...)	do {} while(0)
#endif
/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LCD_LW700AT9005_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/
static void lcd_lw700at9005_power_on(void);
static void lcd_lw700at9005_power_off(void);

/*----------------------- INTERNAL PRIVATE VARIABLES ------------------------*/
/* int  lcd_xxx;        *//*Example*/
lcd_parm_t lcd_lw700at9005_parm = {
	.name = "CHIHSIN LW700AT9005",
	.fps = 48,						/* frame per second */
	.bits_per_pixel = 18,
	.capability = LCD_CAP_CLK_HI,
	.timing = {
		.pixel_clock = 40000000,	/* pixel clock */
		.option = 0,				/* option flags */

		.hsync = 127,				/* horizontal sync pulse */
		.hbp = 127,					/* horizontal back porch */
		.hpixel = 800,				/* horizontal pixel */
		.hfp = 2,					/* horizontal front porch */

		.vsync = 22,					/* vertical sync pulse */
		.vbp = 22,					/* vertical back porch */
		.vpixel = 480,				/* vertical pixel */
		.vfp = 1,					/* vertical front porch */
	},

	.initial = lcd_lw700at9005_power_on,
	.uninitial = lcd_lw700at9005_power_off,
};

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
static void lcd_lw700at9005_power_on(void)
{
	DPRINT("lcd_lw700at9005_power_on\n");
	/* TODO */
}

static void lcd_lw700at9005_power_off(void)
{
	DPRINT("lcd_lw700at9005_power_off\n");

	/* TODO */
}

lcd_parm_t *lcd_lw700at9005_get_parm(int arg)
{
	lcd_lw700at9005_parm.bits_per_pixel = arg;
	return &lcd_lw700at9005_parm;
}

static int lcd_lw700at9005_init(void){
	int ret;

	ret = lcd_panel_register(LCD_CHIHSIN_LW700AT9005,(void *) lcd_lw700at9005_get_parm);
	return ret;
} /* End of lcd_oem_init */
module_init(lcd_lw700at9005_init);

/*--------------------End of Function Body -----------------------------------*/
#undef LCD_CHIHSIN_LW700AT9005_C

