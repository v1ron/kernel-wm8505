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
#define LCD_INNOLUX_AT070TN83_C

/*=== lcd-INNOLUX-AT070TN83.c =============================================================
*
* MODULE       : lcd-INNOLUX-AT070TN83.c
* AUTHOR       : Sam Shen
* DATE         : 2009/4/28
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Sam Shen, 2009/4/28
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include "lcd.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  LCD_AT070TN83_XXXX  xxxx    *//*Example*/

// #define DEBUG
#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBGMSG(fmt, args...)   DPRINT("[INNOLUX] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...)	do {} while(0)
#endif
/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LCD_AT070TN83_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/
static void lcd_at070tn83_initial(void);

/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lcd_xxx;        *//*Example*/
lcd_parm_t lcd_at070tn83_parm = {
	.name = "INNOLUX AT707TN83",
	.fps = 60,						/* frame per second */
	.bits_per_pixel = 18,
	.capability = 0,
	.timing = {
		.pixel_clock = 40000000,	/* pixel clock */
		.option = 0,				/* option flags */

		.hsync = 1,					/* horizontal sync pulse */
		.hbp = 45,					/* horizontal back porch */
		.hpixel = 800,				/* horizontal pixel */
		.hfp = 210,					/* horizontal front porch */

		.vsync = 1,					/* vertical sync pulse */
		.vbp = 22,					/* vertical back porch */
		.vpixel = 480,				/* vertical pixel */
		.vfp = 132,					/* vertical front porch */
	},
	
	.initial = lcd_at070tn83_initial,		
};

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
static void lcd_at070tn83_initial(void)
{	
	DPRINT("lcd_at070tn83_initial\n");
		
	/* TODO */
}

lcd_parm_t *lcd_at070tn83_get_parm(int arg) 
{	
	return &lcd_at070tn83_parm;
}

static int lcd_at070tn83_init(void){	
	int ret;	

	ret = lcd_panel_register(LCD_INNOLUX_AT070TN83,(void *) lcd_at070tn83_get_parm);	
	return ret;
} /* End of lcd_oem_init */
module_init(lcd_at070tn83_init);

/*--------------------End of Function Body -----------------------------------*/
#undef LCD_INNOLUX_AT070TN83_C
