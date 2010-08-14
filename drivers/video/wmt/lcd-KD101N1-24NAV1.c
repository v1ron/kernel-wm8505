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
#define LCD_KD101N1_24NAV1_C

/*=== lcd-KD101N1-24NAV1.c =============================================================
*
* MODULE       : lcd-KD101N1-24NAV1.c
* AUTHOR       : HowayHuo
* DATE         : 2010/3/18
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Howayhuo, 2010/3/18
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include "lcd.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
#define DEBUG
#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBGMSG(fmt, args...)   DPRINT("[KD101N1] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...)	do {} while(0)
#endif
/*----------------------- PRIVATE CONSTANTS ----------------------------------*/


/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/
static void lcd_kd101n1_initial(void);

/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lcd_xxx;        *//*Example*/
lcd_parm_t lcd_kd101n1_parm = {
	.name = "KD101N1 24NA V1",
	.fps = 60,						/* frame per second */
	.bits_per_pixel = 18,
	.capability = 0,
	.timing = {
		.pixel_clock = 33000000,	/* pixel clock */
		.option = 0,				/* option flags */

		.hsync = 23,					/* horizontal sync pulse */
		.hbp = 43,					/* horizontal back porch */
		.hpixel = 1024,				/* horizontal pixel */
		.hfp = 2,					/* horizontal front porch */

		.vsync = 8,					/* vertical sync pulse */
		.vbp = 10,					/* vertical back porch */
		.vpixel = 600,				/* vertical pixel */
		.vfp = 2,					/* vertical front porch */
	},
	
	.initial = lcd_kd101n1_initial,		
};

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
static void lcd_kd101n1_initial(void)
{	
	DPRINT("lcd_kd101n1_initial\n");
		
	/* TODO */
}

lcd_parm_t *lcd_kd101n1_get_parm(int arg) 
{	
	return &lcd_kd101n1_parm;
}

static int lcd_kd101n1_init(void){	
	int ret;	

	ret = lcd_panel_register(LCD_KD101N1_24NAV1,(void *) lcd_kd101n1_get_parm);	
	return ret;
} /* End of lcd_oem_init */
module_init(lcd_kd101n1_init);

/*--------------------End of Function Body -----------------------------------*/
#undef LCD_KD101N1_24NAV1_C
