
#ifndef VID_H
/* To assert that only one occurrence is included */
#define VID_H

/*--- vid.h---------------------------------------------------------------
*   Copyright (C) 2009 WondeMedia Tech. Inc.
*
* MODULE       : vid.h -- 
* AUTHOR       : Max Chen
* DATE         : 2009/2/24
* DESCRIPTION  : 
*------------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Max Chen, 2009/2/24
*	First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/
#ifdef __KERNEL__
#else
#endif

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/


/* TV decoder AD7180 address*/
#define VID_ADC_7180		0x20  //0100 0000 ==> 010 0000

#define REG_BASE_VID    0xD8050A00
#define REG_VID_CTRL 	REG_BASE_VID+0x00 
#define REG_INT_CTRL 	REG_BASE_VID+0x08
#define REG_Y0_SA 		REG_BASE_VID+0x54
#define REG_C0_SA	 	REG_BASE_VID+0x58 
#define REG_Y1_SA	 	REG_BASE_VID+0x5C
#define REG_C1_SA 		REG_BASE_VID+0x60 
#define REG_VBUF_SEL	REG_BASE_VID+0x68 
#define REG_VID_STS       REG_BASE_VID+0x6c

#define VID_FRAME_SIZE 1024*768
#define VID_FRAME_NUM   3



typedef enum {
	VID656_N,
	VID656_P,
	VID601_S1_N,
	VID601_S1_P,	     
	VID601_S2_N,
	VID601_S2_P,
	VID_AUTO_SWITCH_ENABLE,
	VID_AUTO_SWITCH_DISABLE,
}vid_mode_e;



typedef enum {
	VIDDEV_STS_LOCK,
	VIDDEV_STS_TVSYS,
}viddev_sts_e;


typedef enum {
	VID_NTSC,
	VID_PAL
}vid_tvsys_e;

typedef enum{
	VIDEO_MOD_VID,
	VIDEO_MOD_CMOS,
}vid_mod_e;


/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef VID_C /* allocate memory for variables only in vt8430-vid.c */
#define EXTERN
#else
#define EXTERN   extern
#endif
EXTERN unsigned int vid_fb_y_addr[3];
EXTERN unsigned int vid_fb_c_addr[3];
EXTERN unsigned int vid_alloc_buffer;
/* Byte read / write*/

typedef struct vid_gov_resol_s{
	unsigned int x;
	unsigned int y;
}vid_gov_resol_t;

EXTERN int vid_i2c_write(int chipId ,unsigned int index,char data);
EXTERN int vid_i2c_read(int chipId ,unsigned int index);
EXTERN int vid_i2c_write_page(int chipId ,unsigned int index,char *pdata,int len);
EXTERN int vid_i2c_read_page(int chipId ,unsigned int index,char *pdata,int len);

EXTERN void vid_video_update(vid_gov_resol_t input,int fbindex);




/* EXTERN int      vid_xxxx; *//*Example*/
#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/


/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  vid_xxxx(vdp_Void); *//*Example*/

#endif /* ifndef VID_H */

/*=== END vid.h ==========================================================*/

