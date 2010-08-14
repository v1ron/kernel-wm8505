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

#define LCD_LW070AT111_C
/*--- lw070at111.c ---------------------------------------------------------------
*-----------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Sakia Liann, 2007/11/30
*	First version
*
*------------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/slab.h>
#include "lcd.h"



static void lcd_a070vw04_initial(void);

/*
 * CHILIN 7" LCD panel information. 800X480
 */	

lcd_parm_t lcd_a070vw04_parm = 
{
 	.name = "AUO A070VW04 16BPP",
 	.fps = 60,
 	.bits_per_pixel = 16,
	.capability = 0,
	.timing = {
		.pixel_clock = 34000000,	/* pixel clock */
		.option = 0,				/* option flags */

		.hsync = 128,					/* horizontal sync pulse */
		.hbp = 216,					/* horizontal back porch */
		.hpixel = 800,				/* horizontal pixel */
		.hfp = 40,					/* horizontal front porch */

		.vsync = 3,					/* vertical sync pulse */
		.vbp = 24,					/* vertical back porch */
		.vpixel = 480,				/* vertical pixel */
		.vfp = 1,					/* vertical front porch */
	},
	.initial = lcd_a070vw04_initial,	

};

static void lcd_a070vw04_initial(void)
{
	printk("%s***************lcd_a070vw04_initial\n", __func__);
}

lcd_parm_t * lcd_a070vw04_get_parm(int arg) 
{
	return &lcd_a070vw04_parm;
}

static int  lcd_a070vw04_register(void)
{
	int ret;

	ret = lcd_panel_register(LCD_A070VW04,(void *) lcd_a070vw04_get_parm);
	return ret;
} /* End of lw070at111_init */

module_init(lcd_a070vw04_register);

#undef LCD_LW070AT111_C
