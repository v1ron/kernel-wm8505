
#ifndef CMOS_H
/* To assert that only one occurrence is included */
#define CMOS_H

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

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef CMOS_C /* allocate memory for variables only in vt8430-vid.c */
#define EXTERN
#else
#define EXTERN   extern
#endif

/* Byte read / write*/


typedef enum {
	CMOS_MD_BEGIN =0,
	CMOS_MD_GIF , 	   // 384x288
	CMOS_MD_QVGA,   // 320x240 
	CMOS_MD_VGA,     // 640x480 
	CMOS_MD_SVGA,   // 	800x600 
	CMOS_MD_SXGA,   // 	1280x1024 
	CMOS_MD_UXGA,   // 1600x1200 
	CMOS_MD_END,
}cmos_mode_e;


#define CMOS_IIC_ADDR		0x30



/* EXTERN int      vid_xxxx; *//*Example*/
#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/


/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  vid_xxxx(vdp_Void); *//*Example*/

#endif /* ifndef CMOS_H */

/*=== END vid.h ==========================================================*/

