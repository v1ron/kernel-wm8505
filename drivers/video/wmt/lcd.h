#ifndef LCD_H
/* To assert that only one occurrence is included */
#define LCD_H

/*=== lcd.h =============================================================
*   Copyright (C) 2009 WonderMedia Tech Corp.
*
* MODULE       : lcd.h -- 
* AUTHOR       : Sam Shen
* DATE         : 2009/4/24
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Sam Shen, 2009/4/24
	First version

------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/
#include "vpp.h"

/*	following is the C++ header	*/
#ifdef	__cplusplus
extern	"C" {
#endif

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/
/* #define  LCD_XXXX  1    *//*Example*/

/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  lcd_xxx_t;  *//*Example*/
typedef enum {
	LCD_WMT_OEM,
	LCD_CHILIN_LW0700AT9003,
	LCD_INNOLUX_AT070TN83,
	LCD_AUO_A080SN01,
	LCD_M101NWT1,
	LCD_A070VW04,
	LCD_AT080TN52,
	LCD_KD101N1_24NAV1,	
	LCD_CHIHSIN_LW700AT9005,
	LCD_PANEL_MAX
} lcd_panel_t;

#define LCD_CAP_CLK_HI		BIT(0)
#define LCD_CAP_HSYNC_HI	BIT(1)
#define LCD_CAP_VSYNC_HI	BIT(2)
#define LCD_CAP_DE_LO		BIT(3)
typedef struct {
	char *name;
	int fps;
	int bits_per_pixel;
	unsigned int capability;

	vpp_timing_t timing;

	void (*initial)(void);
	void (*uninitial)(void);
} lcd_parm_t;

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef LCD_C /* allocate memory for variables only in vout.c */
#define EXTERN
#else
#define EXTERN   extern
#endif /* ifdef LCD_C */

/* EXTERN int      lcd_xxx; *//*Example*/

EXTERN lcd_parm_t *p_lcd;
EXTERN lcd_panel_t lcd_panel_id;
EXTERN unsigned int lcd_blt_level;
EXTERN unsigned int lcd_blt_freq;

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/
/* #define LCD_XXX_YYY   xxxx *//*Example*/
/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  lcd_xxx(void); *//*Example*/

int lcd_panel_register(int no,void (*get_parm)(int mode));
lcd_parm_t *lcd_get_parm(lcd_panel_t id,unsigned int arg);
int lcd_init(void);

/* LCD back light */
void lcd_blt_enable(int no,int enable);
void lcd_blt_set_level(int no,int level);
void lcd_blt_set_freq(int no, unsigned int freq);

#ifdef	__cplusplus
}
#endif	
#endif /* ifndef LCD_H */
