/* PVCS version log
** $Log:  $
 * 
 */
#ifndef WMT_VID_H
/* To assert that only one occurrence is included */
#define WMT_VID_H

/*--- wmt-vid.h ----------------------------------------------------------------
*   Copyright (C) 2010 WonderMedia Tech. Inc.
*
* MODULE       : wmt-vid.h
* AUTHOR       : Willy Chuang
* DATE         : 2010/05/28
* DESCRIPTION  : 
*------------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Willy Chuang, 2010/05/28
*	First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/

//#include <asm/hardware.h>
#include <mach/hardware.h>

#include "wmt-vidreg.h"

#ifdef __KERNEL__
//    #include <linux/delay.h>     // for mdelay() only
    #define PRINT           printk
#else
//    #define mdelay() 
    #define PRINT           printf
    #define KERN_ERR     
    #define KERN_WARNING
    #define KERN_INFO
    #define KERN_DEBUG
#endif

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  viaapi_xxx_t;  *//*Example*/

typedef enum{
	VID_MODE_CMOS,   /* CMOS sensor mode */
	VID_MODE_TVDEC   /* TV Decoder mode */
} vid_mode;

/*------------------------------------------------------------------------------
    Definitions of structures
------------------------------------------------------------------------------*/
#define VID_FB_M \
    unsigned int  y_addr;   \
    unsigned int  c_addr;   \
    unsigned int  id;       \
    unsigned int  done;     \
    unsigned int  is_busy
/* End of VD_IOCTL_CMD_M */

/* Following structure is used for all HW decoder as input arguments */
typedef struct {
    VID_FB_M;
} vid_fb_t;

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef WMT_VID_C 
    #define EXTERN
#else
    #define EXTERN   extern
#endif 

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  viaapi_xxxx(vdp_Void); *//*Example*/

int wmt_vid_open(vid_mode mode);
int wmt_vid_close(vid_mode mode);
int wmt_vid_set_mode(int width, int height);
int wmt_vid_set_addr(unsigned int y_addr, unsigned int c_addr);
  
int wmt_vid_set_cur_fb(vid_fb_t *fb);
vid_fb_t * wmt_vid_get_cur_fb(void);
  
int wmt_vid_i2c_write(int chipId ,unsigned int index,char data);
int wmt_vid_i2c_read(int chipId ,unsigned int index) ;
int wmt_vid_i2c_read_page(int chipId ,unsigned int index,char *pdata,int len);
int wmt_vid_i2c_write_page(int chipId ,unsigned int index,char *pdata,int len);
      
#endif /* ifndef WMT_VID_H */

/*=== END wmt-vid.h ==========================================================*/
