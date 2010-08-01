/* PVCS version log
** $Log:  $
 * 
 */
#ifndef WM8510_JDEC_H
/* To assert that only one occurrence is included */
#define WM8510_JDEC_H

/*--- vt8430_viaapi.h---------------------------------------------------------------
*   Copyright (C) 2008 WonderMedia Tech. Inc.
*
* MODULE       : wm8510-jdec.h
* AUTHOR       : Willy Chuang
* DATE         : 2008/11/20
* DESCRIPTION  : 
*------------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Willy Chuang, 2008/11/20
*	First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/
/* #define  VIAAPI_XXXX  1    *//*Example*/

#define WMT_JDEC_BASE        0xD80FE000
#define WMT_JDEC_END         0xD80FEFFF

/*------------------------------------------------------------------------------
    Definitions of JDEC Registers
    About following definitions, please refer "WM3426 JPEG Decoder Register List"
------------------------------------------------------------------------------*/

#define REG_JDEC_CLOCK_ENABLE           (WMT_JDEC_BASE + 0x000)
#define REG_JDEC_SW_RESET               (WMT_JDEC_BASE + 0x010)
#define REG_JDEC_INT_ENALBE             (WMT_JDEC_BASE + 0x020)
#define REG_JDEC_INT_STATUS             (WMT_JDEC_BASE + 0x024)
/* JPEG Bitstream Input DMA Register */
#define REG_JDEC_BSDMA_PRD_ADDR         (WMT_JDEC_BASE + 0x100)
#define REG_JDEC_BSDMA_FLUSH_ENABLE     (WMT_JDEC_BASE + 0x104)
#define REG_JDEC_BSDMA_START_TX         (WMT_JDEC_BASE + 0x108)
#define REG_JDEC_BSDMA_READ_WCNT        (WMT_JDEC_BASE + 0x10C)
/* JPEG Decode Control Register */
#define REG_JDEC_INIT                   (WMT_JDEC_BASE + 0x200)
#define REG_JDEC_SRC_WIDTH_MCU          (WMT_JDEC_BASE + 0x210)
#define REG_JDEC_SRC_HEIGHT_MCU         (WMT_JDEC_BASE + 0x214)
#define REG_JDEC_SRC_COLOR_FORMAT       (WMT_JDEC_BASE + 0x218)
#define REG_JDEC_PARTIAL_ENABLE         (WMT_JDEC_BASE + 0x220)
#define REG_JDEC_PARTIAL_H_START        (WMT_JDEC_BASE + 0x230)
#define REG_JDEC_PARTIAL_V_START        (WMT_JDEC_BASE + 0x234)
#define REG_JDEC_PARTIAL_WIDTH          (WMT_JDEC_BASE + 0x238)
#define REG_JDEC_PARTIAL_HEIGHT         (WMT_JDEC_BASE + 0x23C)
#define REG_JDEC_DST_MCU_WIDTH          (WMT_JDEC_BASE + 0x240)
#define REG_JDEC_DST_MCU_HEIGHT         (WMT_JDEC_BASE + 0x244)
#define REG_JDEC_YBASE                  (WMT_JDEC_BASE + 0x250)
#define REG_JDEC_Y_LINE_WIDTH           (WMT_JDEC_BASE + 0x254) /* 64 bytes alignment */
#define REG_JDEC_CBASE                  (WMT_JDEC_BASE + 0x258)
#define REG_JDEC_C_LINE_WIDTH           (WMT_JDEC_BASE + 0x25C)
/* Picture Scaling Control Register */
#define REG_JDEC_Y_SCALE_RATIO          (WMT_JDEC_BASE + 0x300)
#define REG_JDEC_C_SCALE_RATIO_H        (WMT_JDEC_BASE + 0x304)
#define REG_JDEC_C_SCALE_RATIO_V        (WMT_JDEC_BASE + 0x308)
#define REG_JDEC_CHROMA_ENABLE          (WMT_JDEC_BASE + 0x30C)
/* YCbCr to RGB Conversion Control Register */
#define REG_JDEC_YUV2RGB_ENABLE         (WMT_JDEC_BASE + 0x400)
#define REG_JDEC_RGB_ALPHA              (WMT_JDEC_BASE + 0x404)
#define REG_JDEC_YUV2RGB_COEF_F0        (WMT_JDEC_BASE + 0x410)
#define REG_JDEC_YUV2RGB_COEF_F1        (WMT_JDEC_BASE + 0x414)
#define REG_JDEC_YUV2RGB_COEF_F2        (WMT_JDEC_BASE + 0x418)
#define REG_JDEC_YUV2RGB_COEF_F3        (WMT_JDEC_BASE + 0x41C)
#define REG_JDEC_YUV2RGB_COEF_F4        (WMT_JDEC_BASE + 0x420)
#define REG_JDEC_YUV2RGB_COEF_F5        (WMT_JDEC_BASE + 0x424)
#define REG_JDEC_YUV2RGB_COEF_F6        (WMT_JDEC_BASE + 0x428)
#define REG_JDEC_YUV2RGB_Y_SUB_16_EN    (WMT_JDEC_BASE + 0x42C)
/* JPEG Decoder Status Register */
#define REG_JDEC_SOF_Y                  (WMT_JDEC_BASE + 0x500)
#define REG_JDEC_SOF_X                  (WMT_JDEC_BASE + 0x504)
#define REG_JDEC_SOF                    (WMT_JDEC_BASE + 0x508)
#define REG_JDEC_FIFO_STATUS            (WMT_JDEC_BASE + 0x510)


/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  viaapi_xxx_t;  *//*Example*/

/*------------------------------------------------------------------------------
    Definitions of enum
------------------------------------------------------------------------------*/

/* Following enum is used for REG_JDEC_SRC_COLOR_FORMAT only */
typedef enum { 
    ColorFormat_420,  /* 0 */
    ColorFormat_422H, /* 1 */
    ColorFormat_422V, /* 2 */
    ColorFormat_444,  /* 3 */
    ColorFormat_411,  /* 4 */
    GrayLevel         /* 5 *//* only contain Y component */
} jdec_color_format;


/*------------------------------------------------------------------------------
    Definitions of structures
------------------------------------------------------------------------------*/

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef WM8510_JDEC_C /* allocate memory for variables only in wm8510-jdec.c */
    #define EXTERN
#else
    #define EXTERN   extern
#endif /* ifdef WM8510_JDEC_C */

/* EXTERN int      viaapi_xxxx; *//*Example*/

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  viaapi_xxxx(vdp_Void); *//*Example*/

    
#endif /* ifndef WM8510_JDEC_H */

/*=== END wm8510-jdec.h ==========================================================*/
