/* PVCS version log
** $Log:  $
 * 
 */
#ifndef WMT_VIDREG_H
/* To assert that only one occurrence is included */
#define WMT_VIDREG_H

/*--- wmt-vidreg.h ----------------------------------------------------------------
*   Copyright (C) 2010 WonderMedia Tech. Inc.
*
* MODULE       : wmt-vidreg.h
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

#define WMT_VID_IRQ          IRQ_NA12_2   // this may be changed by chip

#define REG_BASE_VID         0xD8050A00

/*------------------------------------------------------------------------------
    Definitions of VID Registers
    About following definitions, please refer "WM3426 VID/CMOS Register List"
    
    Prefix meanings:
    REG_VID_xxx: used for both TV deccoder and CMOS
    REG_VID_TVDEC_xxx: used for TV encoder only
    REG_VID_CMOS_XXX:  used for CMOS only
------------------------------------------------------------------------------*/
#define REG_VID_TVDEC_CTRL          (REG_BASE_VID + 0x00) 
#define REG_VID_TVDEC_CONFIG        (REG_BASE_VID + 0x04) 
#define REG_VID_INT_CTRL            (REG_BASE_VID + 0x08)
#define REG_VID_Y0_SA               (REG_BASE_VID + 0x54)
#define REG_VID_C0_SA               (REG_BASE_VID + 0x58) 
#define REG_VID_Y1_SA               (REG_BASE_VID + 0x5C)
#define REG_VID_C1_SA               (REG_BASE_VID + 0x60) 
#define REG_VID_WIDTH               (REG_BASE_VID + 0x64) 
#define REG_VID_VBUF_SEL            (REG_BASE_VID + 0x68) 
#define REG_VID_STS                 (REG_BASE_VID + 0x6C)
#define REG_VID_HEIGHT              (REG_BASE_VID + 0x7C) 
#define REG_VID_CMOS_EN             (REG_BASE_VID + 0xB0)
#define REG_VID_CMOS_PIXEL_SWAP     (REG_BASE_VID + 0xB4) 



/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  viaapi_xxx_t;  *//*Example*/

/*------------------------------------------------------------------------------
    Definitions of structures
------------------------------------------------------------------------------*/


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

    
#endif /* ifndef WMT_VIDREG_H */

/*=== END wmt-vidreg.h ==========================================================*/
