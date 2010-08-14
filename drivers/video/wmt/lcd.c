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
#define LCD_C

/*=== lcd.c =============================================================
*
* MODULE       : lcd.c
* AUTHOR       : Sam Shen
* DATE         : 2009/4/24
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------*
Version 0.01 , Sam Shen, 2009/4/24
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include "lcd.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  LCD_XXXX  xxxx    *//*Example*/

#define DEBUG
#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBGMSG(fmt, args...)   DPRINT("[LCD] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...)	do {} while(0)
#endif
/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define LCD_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx lcd_xxx_t; *//*Example*/
typedef struct {
	lcd_parm_t* (*get_parm)(int arg);
} lcd_device_t;

/*----------EXPORTED PRIVATE VARIABLES are defined in lcd.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  lcd_xxx;        *//*Example*/
lcd_device_t lcd_device_array[LCD_PANEL_MAX];
lcd_panel_t lcd_panel_id = LCD_PANEL_MAX;
int lcd_panel_on = 1;

#ifdef CONFIG_PWM_WMT
void pwm_set_enable(int no,int enable);
void pwm_set_freq(int no,unsigned int freq);
void pwm_set_level(int no,unsigned int level);
#endif
/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void lcd_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*----------------------- Backlight --------------------------------------*/
void lcd_blt_enable(int no,int enable)
{
#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_PWM_WMT)
	pwm_set_enable(no,enable);
#endif
	return;
} /* End of vt8430_blt_enable */

void lcd_blt_set_level(int no,int level)
{
#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_PWM_WMT)
	pwm_set_level(no,level);
#endif
	return;
}

void lcd_blt_set_freq(int no,unsigned int freq)
{
#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_PWM_WMT)
	pwm_set_freq(no,freq);
#endif
	return;
}

/*----------------------- LCD --------------------------------------*/
static int __init lcd_arg_panel_id
(
	char *str			/*!<; // argument string */
)
{
	sscanf(str,"%d",(int *) &lcd_panel_id);
	if( lcd_panel_id >= LCD_PANEL_MAX ){
		lcd_panel_id = LCD_PANEL_MAX;
	}
	DBGMSG(KERN_INFO "set lcd panel id = %d\n",lcd_panel_id);

  	return 1;
} /* End of lcd_arg_panel_id */

__setup("lcdid=", lcd_arg_panel_id);

int lcd_panel_register
(
	int no,						/*!<; //[IN] device number */
	void (*get_parm)(int mode)	/*!<; //[IN] get info function pointer */
)
{
	lcd_device_t *p;

	if( no >= LCD_PANEL_MAX ){
		DBGMSG(KERN_ERR "*E* lcd device no max is %d !\n",LCD_PANEL_MAX);
		return -1;
	}

	p = &lcd_device_array[no];
	if( p->get_parm ){
		DBGMSG(KERN_ERR "*E* lcd device %d exist !\n",no);
		return -1;
	}
	p->get_parm = (void *) get_parm;
//	printk("lcd_device_register %d 0x%x\n",no,p->get_parm);
	return 0;
} /* End of lcd_device_register */

lcd_parm_t *lcd_get_parm(lcd_panel_t id,unsigned int arg)
{
	lcd_device_t *p;

	p = &lcd_device_array[id];
	if( p && p->get_parm ){
		return p->get_parm(arg);
	}
	return 0;
}

extern unsigned int pwm_pclk(void);
extern unsigned int pwm_level[2];
extern unsigned int pwm_freq[2];
extern int PWM_PERIOD_VAL;
extern int wmt_getsyspara(char *varname, unsigned char *varval, int varlen);

static void wmt_set_backlight(void)
{
	int retval;
        unsigned char buf[80]={0};
        enum
            {
                idx_pwm_no,
                idx_scalar,
                idx_period,
                idx_duty,
                idx_max
            };
        char * p, *endp;
	int i = 0;
	long ps[idx_max];
        unsigned int clock, pwm_no;

//-- read PWM param from U-BOOT param
        lcd_blt_level = 100;
        lcd_blt_freq = 150;
        retval = wmt_getsyspara("pwmparam", buf, 80);
	if(!retval)
	{
	    p = buf;
	    printk("--- pwmparam = %s\n", buf);

                while(i < idx_max)
                {
                    ps[i++] = simple_strtoul(p, &endp, 10);
                    if( *endp == '\0')
                        break;
                    p = endp + 1;

                    if(*p == '\0')
                        break;
              }

              if( i != idx_max)
                  printk("warning: u-boot pwmparam is wrong: need %d arg count, but get %d\n", idx_max, i);
              else if((ps[idx_pwm_no] >= 0 && ps[idx_pwm_no] <= 1 && ps[idx_pwm_no] == VPP_BLT_PWM_NUM)  // check PWM no, only support PWM0 and PWM1
                  && (ps[idx_scalar] > 0 && ps[idx_scalar] <= 1024)
                  && (ps[idx_period] > 0 && ps[idx_period] < 4096)
                  && (ps[idx_duty] > 0 && ps[idx_duty] < 4096))
              {
                  printk("pwm_no = %lu scalar = %lu, period = %lu, duty = %lu\n",
                  ps[idx_pwm_no], ps[idx_scalar], ps[idx_period], ps[idx_duty]);

                  pwm_no = ps[idx_pwm_no];
                  clock = pwm_pclk();
                  PWM_PERIOD_VAL = ps[idx_period];
    	          lcd_blt_level = (ps[idx_duty] *  100)/PWM_PERIOD_VAL;
                  lcd_blt_freq = clock/PWM_PERIOD_VAL/ps[idx_scalar];
    	          pwm_level[VPP_BLT_PWM_NUM] = lcd_blt_level;
    	          pwm_freq[VPP_BLT_PWM_NUM] = lcd_blt_freq;
    	          printk("pwm%d: clock = %u, freq = %u, level = %u\n", VPP_BLT_PWM_NUM, clock, lcd_blt_freq, lcd_blt_level);
              }
    	      else
    	   	  printk("u-boot pwmparam error!\n");
	}
//----
}
int lcd_init(void)
{
    printk("-------------lcd_init\n");
 //   lcd_blt_level = 100;
  //  lcd_blt_freq = 150;
    wmt_set_backlight();
	// lcd_panel_id = LCD_WMT_OEM;
	if( lcd_panel_id >= LCD_PANEL_MAX )
		return -1;
	return 0;
}
/*--------------------End of Function Body -----------------------------------*/
#undef LCD_C
