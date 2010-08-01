#ifndef VOUT_H
/* To assert that only one occurrence is included */
#define VOUT_H

/*=== vout.c =============================================================
*   Copyright (C) 2009 WonderMedia Tech Corp.
*
* MODULE       : vout.h -- 
* AUTHOR       : Sam Shen
* DATE         : 2009/2/6
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Sam Shen, 2009/2/6
	First version

------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/
#include "vpp.h"

/*	following is the C++ header	*/
#ifdef	__cplusplus
extern	"C" {
#endif

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/
/* #define  VO_XXXX  1    *//*Example*/

/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  vo_xxx_t;  *//*Example*/
typedef enum {
	VOUT_SD_ANALOG,
	VOUT_SD_DIGITAL,
	VOUT_LCD,
	VOUT_DVI,
	VOUT_HDMI,
	VOUT_DVO2HDMI,
	VOUT_DVO,
	VOUT_VGA,
	VOUT_BOOT,
	VOUT_MODE_MAX,
	VOUT_MODE_ALL = VOUT_MODE_MAX
} vout_mode_t;

typedef enum {
	VOCTL_INIT,
	VOCTL_UNINIT,
	VOCTL_COMPATIBLE,
	VOCTL_VISIBLE,
	VOCTL_CONFIG,
	VOCTL_SUSPEND,
	VOCTL_RESUME,
	VOCTL_BRIGHTNESS,
	VOCTL_CONTRAST,
	VOCTL_CHKPLUG,
} vout_ctrl_t;	

typedef struct {
	int (*init)(int arg);
	int (*uninit)(int arg);
	int (*compatible)(int arg);
	int (*visible)(int arg);
	int (*config)(int arg);
	int (*suspend)(int arg);
	int (*resume)(int arg);
	int (*brightness)(int arg);
	int (*contrast)(int arg);
	int (*chkplug)(int arg);
	int (*get_edid)(int arg);
} vout_ops_t;

typedef struct {
	int active;
	int enable;
	vout_ops_t *ops;
	char name[10];
	unsigned int option[3];
	int resx;
	int resy;
	int pixclk;
} vout_t;

typedef struct {
	int type;
	int resx;
	int resy;
	int bpp;
	int fps;

	unsigned int pixclock;		/* pixel clock in ps (pico seconds) */
	unsigned int left_margin;	/* time from sync to picture	*/
	unsigned int right_margin;	/* time from picture to sync	*/
	unsigned int upper_margin;	/* time from sync to picture	*/
	unsigned int lower_margin;
	unsigned int hsync_len;		/* length of horizontal sync	*/
	unsigned int vsync_len;		/* length of vertical sync	*/
} vout_info_t;

typedef struct {
	int (*init)(void *ops);
	void (*set_power_down)(int enable);
	int (*set_mode)(unsigned int *option);
	int (*config)(vout_info_t *info);
	int (*check_plugin)(void);
} vout_dev_ops_t;

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef VOUT_C /* allocate memory for variables only in vout.c */
#define EXTERN
#else
#define EXTERN   extern
#endif /* ifdef VOUT_C */

/* EXTERN int      vo_xxx; *//*Example*/
EXTERN int (*vout_board_info)(int arg);

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/
/* #define VO_XXX_YYY   xxxx *//*Example*/
/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  vo_xxx(void); *//*Example*/

int vout_register(vout_mode_t mode,vout_t *vo);
int vout_unregister(vout_mode_t mode);
int vout_config(vout_mode_t mode,vout_info_t *info);
int vout_set_mode(vout_mode_t mode,int on);
int vout_enable(vout_mode_t mode,int on);
int vout_control(vout_mode_t mode,int cmd,int arg);
int vout_suspend(vout_mode_t mode,int level);
int vout_resume(vout_mode_t mode,int level);
int vout_chkplug(vout_mode_t mode);
vout_t *vout_get_info(vout_mode_t mode);

/* AD9389 */
int ad9389_init(void *ptr);

/* VT1632 */
int vt1632_init(void);
int vt1632_set_mode(vpp_datawidht_t dwidth);
int vt1632_set_power_down(int enable);
int vt1632_check_plugin(void);

/* AD7393 */
int ad7393_check_plugin(void);
void ad7393_set_power_down(int enable);
int ad7393_set_mode(vpp_tvsys_t tvsys,vpp_tvconn_t tvconn);
int ad7393_init(void);

/* CAT6612 */
int cat6612_init(void *ptr);

#ifdef	__cplusplus
}
#endif	

#endif /* ifndef VOUT_H */

/*=== END vout.h ==========================================================*/
