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

#define WM8510_JDEC_C

/*--- vt8500-jpeg.c ---------------------------------------------------------------
*
* MODULE       : wm8510-jdec.c
* AUTHOR       : Willy Chuang
* DATE         : 2008/11/24
* DESCRIPTION  : 
*-----------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Willy Chuang, 2008/11/24
*	First version
*
*------------------------------------------------------------------------------*/

#include "hw-jdec.h"
#include "wm8510-jdec.h"

//#define JDEC_REG_TRACE
#ifdef JDEC_REG_TRACE
#define JDEC_REG_SET32(addr, val)  \
        PRINT("REG_SET:0x%x -> 0x%0x\n", addr, val);\
        REG32_VAL(addr) = (val)
#else
#define JDEC_REG_SET32(addr, val)      REG32_VAL(addr) = (val)
#endif

//#define JDEC_PRD_DEBUG
#ifdef JDEC_PRD_DEBUG
   #define DBG_PRD(fmt, args...)     PRINT("{%s} " fmt, __FUNCTION__ , ## args)
#else
   #define DBG_PRD(fmt, args...)
#endif

#define WMT_JDEC_IRQ         IRQ_NA0_1     // this may be chagned by chip

#define SW_PATCH_FOR_ZERO_BITS_AHEAD_EOI
#define SW_PATCH_FOR_EOI  

#define ALIGN64(a)          ((((a)+63)>>6)<<6)
#define MAX(a, b)           ((a)>=(b))?(a):(b)

#ifdef SW_PATCH_FOR_ZERO_BITS_AHEAD_EOI
#define PRD_PATCH_BUF_SIZE    1024
unsigned char *tmp_prd_buf;
dma_addr_t     last_prd_phy_addr = 0;
#endif

static jdec_drvinfo_t *current_drv;

typedef enum { 
    SDTV_16_235, /* 0 */
    SDTV_0_255,  /* 1 */
    HDTV_16_235, /* 2 */
    HDTV_0_255,  /* 3 */
    JFIF_0_255   /* 4 */
} yuv2rgb_formula;

static jdec_capability_t capab_table = {
    /* identity */
    MAKE_IDENTITY(VD_JPEG, JDEC_VERSION),
    sizeof(jdec_capability_t), /* size */
    
    8510, /* chip id */
    
    1, /* baseline:1 */
    0, /* progressive:1 */
    1, /* int graylevel:1 */   
    1, /* int partial_decode:1 */
    1, /* decoded_to_YC420:1; */
    1, /* decoded_to_YC422H:1 */
    1, /* decoded_to_YC444:1 */
    1, /* decoded_to_ARGB:1 */
    0, /* reseverd:24 */
    
    4, /* scale_ratio_num */
    {1,2,4,8,0,0,0,0} /* scale_fator[8] */
};

static int s_msd_backup_len = 0;  // for Multi-Segment Decoding only
static char msd_buf[64], msd_prd_buf[128];
static char *msd_prd_virt;

#ifdef __KERNEL__
DECLARE_WAIT_QUEUE_HEAD(jdec_wait);
#endif

/*!*************************************************************************
* dump_prd_table
* 
* Private Function by Willy Chuang, 2009/09/17
*/
/*!
* \brief
*   Dump values in PRD table for debugging
*
* \parameter
*
* \retval  0 if success
*/ 
int dump_prd_table(jdec_drvinfo_t *drv)
{
//#define DUMP_DATA
    unsigned int  prd_addr_in, prd_data_size = 0;
    unsigned int   addr, len, i;
#ifdef DUMP_DATA
    unsigned char *ptr;
    unsigned int   addr, len, i;
#endif

    printk("=== dump_prd_table ===\n");
    prd_addr_in = drv->prd_virt;
    for(i=0; ; i+=2) {
        addr = *(unsigned int *)(prd_addr_in + i * 4);
        len = *(unsigned int *)(prd_addr_in + (i + 1) * 4);
        prd_data_size += (len & 0xFFFF);
        printk("[%02d]Addr: 0x%08x\n", i, addr);
        printk("[%02d]Len:  0x%08x (%d)\n", i+1, len, (len & 0xFFFF));        

#ifdef DUMP_DATA
{
    if((i==0) && (addr != 0)){
        ptr = (unsigned char *)phys_to_virt(addr);
        printk("=== First block (%p) ===\n", (void *)ptr);
        for(j=0; j<32; j++ ){
            if((j%16) == 15) 
                printk("\n");
            else 
                printk("0x%02x ", *ptr);
            ptr++;
        }
    }
}
#endif        
        if(len & 0x80000000) 
            break;
    }
#ifdef DUMP_DATA
{
    ptr = (unsigned char *)phys_to_virt(addr);
    printk("=== Last block (%p) ===\n", (void *)ptr);
    for(j=0; j<32; j++ ){
        if((j%16) == 15) 
            printk("\n");
        else 
            printk("0x%02x ", *ptr);
        ptr++;
    }
}
#endif        
    printk("prd_data_size: %d\n", prd_data_size);
    
    return 0;
} /* End of dump_prd_table() */

/*!*************************************************************************
* dbg_dump_registers
* 
* Private Function by Willy Chuang, 2009/01/08
*/
/*!
* \brief
*   Dump important register values for debugging
*
* \parameter
*
* \retval  0 if success
*/ 
static void dbg_dump_registers(jdec_drvinfo_t *drv)
{
  #define REG_BASE_JDEC              WMT_JDEC_BASE
  #define Dump_RegisterValue(addr)   PRINT("REG(0x%x): 0x%x\n", addr, REG32_VAL(addr));
 
#if 0 // debug only
    dump_prd_table(drv);
#endif 

  Dump_RegisterValue(REG_BASE_JDEC+0x000);
  Dump_RegisterValue(REG_BASE_JDEC+0x010);
  Dump_RegisterValue(REG_BASE_JDEC+0x020);
  Dump_RegisterValue(REG_BASE_JDEC+0x024);
  Dump_RegisterValue(REG_BASE_JDEC+0x030);

  Dump_RegisterValue(REG_BASE_JDEC+0x100);
  Dump_RegisterValue(REG_BASE_JDEC+0x104);
  Dump_RegisterValue(REG_BASE_JDEC+0x108);
  Dump_RegisterValue(REG_BASE_JDEC+0x10C);

  Dump_RegisterValue(REG_BASE_JDEC+0x200);
  Dump_RegisterValue(REG_BASE_JDEC+0x210);
  Dump_RegisterValue(REG_BASE_JDEC+0x214);
  Dump_RegisterValue(REG_BASE_JDEC+0x218);
  Dump_RegisterValue(REG_BASE_JDEC+0x220);
  Dump_RegisterValue(REG_BASE_JDEC+0x230);
  Dump_RegisterValue(REG_BASE_JDEC+0x234);
  Dump_RegisterValue(REG_BASE_JDEC+0x238);
  Dump_RegisterValue(REG_BASE_JDEC+0x23C);
  Dump_RegisterValue(REG_BASE_JDEC+0x240);
  Dump_RegisterValue(REG_BASE_JDEC+0x244);
  Dump_RegisterValue(REG_BASE_JDEC+0x250);
  Dump_RegisterValue(REG_BASE_JDEC+0x254);
  Dump_RegisterValue(REG_BASE_JDEC+0x258);
  Dump_RegisterValue(REG_BASE_JDEC+0x25C);
  Dump_RegisterValue(REG_BASE_JDEC+0x300);
  Dump_RegisterValue(REG_BASE_JDEC+0x304);
  Dump_RegisterValue(REG_BASE_JDEC+0x308);
  Dump_RegisterValue(REG_BASE_JDEC+0x30C);

  Dump_RegisterValue(REG_BASE_JDEC+0x400);
  Dump_RegisterValue(REG_BASE_JDEC+0x404);
  Dump_RegisterValue(REG_BASE_JDEC+0x410);
  Dump_RegisterValue(REG_BASE_JDEC+0x414);
  Dump_RegisterValue(REG_BASE_JDEC+0x418);
  Dump_RegisterValue(REG_BASE_JDEC+0x41C);
  Dump_RegisterValue(REG_BASE_JDEC+0x420);
  Dump_RegisterValue(REG_BASE_JDEC+0x424);
  Dump_RegisterValue(REG_BASE_JDEC+0x428);
  Dump_RegisterValue(REG_BASE_JDEC+0x42C);

  Dump_RegisterValue(REG_BASE_JDEC+0x500);
  Dump_RegisterValue(REG_BASE_JDEC+0x504);
  Dump_RegisterValue(REG_BASE_JDEC+0x508);
  Dump_RegisterValue(REG_BASE_JDEC+0x50C);
  Dump_RegisterValue(REG_BASE_JDEC+0x510);

  Dump_RegisterValue(REG_BASE_JDEC+0x800);
  Dump_RegisterValue(REG_BASE_JDEC+0x804);
  Dump_RegisterValue(REG_BASE_JDEC+0x808);
  Dump_RegisterValue(REG_BASE_JDEC+0x820);
  Dump_RegisterValue(REG_BASE_JDEC+0x824);
  Dump_RegisterValue(REG_BASE_JDEC+0x828);
  Dump_RegisterValue(REG_BASE_JDEC+0x82C);
  Dump_RegisterValue(REG_BASE_JDEC+0x830);
  Dump_RegisterValue(REG_BASE_JDEC+0x834);
  Dump_RegisterValue(REG_BASE_JDEC+0x838);
  Dump_RegisterValue(REG_BASE_JDEC+0x83C);
  Dump_RegisterValue(REG_BASE_JDEC+0x840);
  Dump_RegisterValue(REG_BASE_JDEC+0x844);
  Dump_RegisterValue(REG_BASE_JDEC+0x848);
  Dump_RegisterValue(REG_BASE_JDEC+0x84C);
  Dump_RegisterValue(REG_BASE_JDEC+0x850);
  Dump_RegisterValue(REG_BASE_JDEC+0x854);
  Dump_RegisterValue(REG_BASE_JDEC+0x858);
  Dump_RegisterValue(REG_BASE_JDEC+0x85C);
  Dump_RegisterValue(REG_BASE_JDEC+0x860);
  Dump_RegisterValue(REG_BASE_JDEC+0x864);

} /* End of dbg_dump_registers() */

static void stop_bsdam(void)
{
    JDEC_REG_SET32(REG_JDEC_BSDMA_START_TX, 0);
} /* End of stop_bsdam() */

/*!*************************************************************************
* jdec_clock_en
* 
* API Function by Willy Chuang, 2009/07/24
*/
/*!
* \brief
*	JPEG decoder clock enable/disable
*
* \retval  0 if success
*/ 
static int jdec_clock_en(int enable) 
{    
    if(enable) {
        /*----------------------------------------------------------------------
             Enable clock
        ----------------------------------------------------------------------*/
        JDEC_REG_SET32(REG_JDEC_CLOCK_ENABLE, 1);
    }
    else {
        /*--------------------------------------------------------------------------
            Disable JPEG clock
        --------------------------------------------------------------------------*/
        JDEC_REG_SET32(REG_JDEC_CLOCK_ENABLE, 0);
    }
    return 0;
} /* End of jdec_clock_en() */

/*!*************************************************************************
* get_mcu_unit
* 
* Private Function by Willy Chuang, 2008/12/06
*/
/*!
* \brief
*   Get MCU size
*
* \parameter
*   sof_w         [IN]  Picture width in SOF
*   sof_h         [IN]  Picture height in SOF
*   color_format  [IN]  Picture color format
*   mcu_width     [OUT] Pixel per MCU in width
*   mcu_height    [OUT] Pixel per MCU in height
* \retval  0 if success
*/ 
static int get_mcu_unit(
    unsigned int    sof_w,         /* Picture width in SOF */  
    unsigned int    sof_h,         /* Picture height in SOF */ 
    vdo_color_fmt   color_format,  /* color format */
    unsigned int   *mcu_width,
    unsigned int   *mcu_height)
{
    switch(color_format){
        case VDO_COL_FMT_YUV420:
            *mcu_width  = (sof_w + 15)>>4;
            *mcu_height = (sof_h + 15)>>4;
            break;
        case VDO_COL_FMT_YUV422H:
            *mcu_width  = (sof_w + 15)>>4;
            *mcu_height = (sof_h + 7)>>3;
            break;
        case VDO_COL_FMT_YUV422V:
            *mcu_width  = (sof_w + 7)>>3;
            *mcu_height = (sof_h + 15)>>4;
            break;
        case VDO_COL_FMT_YUV444:
        case VDO_COL_FMT_GRAY:
            *mcu_width  = (sof_w + 7)>>3;
            *mcu_height = (sof_h + 7)>>3;
            break;
        case VDO_COL_FMT_YUV411:
            *mcu_width  = (sof_w + 31)>>5;
            *mcu_height = (sof_h + 7)>>3;
            break;
        default:
            *mcu_width  = 0;
            *mcu_height = 0;
            DBG_ERR("Unknown input color format(%d)\n", color_format);
            return -1;
    }
    DBG_MSG("Color format(%d), MCU w: %d, h: %d\n", color_format, *mcu_width, *mcu_height);
    
    return 0;
}  /* End of get_mcu_unit() */

/*!*************************************************************************
* wait_decode_finish
* 
* Private Function by Willy Chuang, 2008/12/3
*/
/*!
* \brief
*	Wait HW decoder finish job
*
* \retval  none
*/ 
static int wait_decode_finish(jdec_drvinfo_t *drv)
{
    int ret = 0;
    int err_happen = 0;
    
#ifdef __KERNEL__
    /*--------------------------------------------------------------------------
        LOOK OUT: STA_DMA_MOVE_DONE != STA_DECODE_DONE
                  Sometimes, a large pictuer will be individed into many samll 
                  segment to decode.
    --------------------------------------------------------------------------*/
    if((drv->_status & STA_ERR_DMA_TX) || (drv->_status & STA_ERR_UNKNOWN)){
        DBG_ERR("Unexcepted status: 0x%x\n", drv->_status);
        err_happen = 1;
    }
  #ifdef JDEC_IRQ_MODE
    /* Wait decoding finish */
    if( (err_happen) || (wait_event_interruptible_timeout( jdec_wait, 
                    (drv->_status & STA_DECODE_DONE), drv->_timeout) == 0) ){
        unsigned int  read_byte;

        read_byte = REG32_VAL(REG_JDEC_BSDMA_READ_WCNT) * 2;
        DBG_ERR("read_byte(%d), input size(%d)\n", read_byte, drv->arg_in.src_size);
        DBG_ERR("Decode JEPG NOT complete\n");
        DBG_ERR("Time out: %d ms\n", drv->_timeout);

        dbg_dump_registers(drv);
        drv->_status = STA_ERR_DECODING;
        
        stop_bsdam();
        
//        ret = -1;

        /* Disable clock to save power consumption */
        jdec_clock_en(0);
    }
  #else /* if JPEC_POLL_MODE */
  {
    unsigned int  reg_val, i=100000;

    while(i--) {
        reg_val = REG32_VAL(REG_JDEC_INT_STATUS);
        if(reg_val & 0x01) {
            drv->_status |= STA_DECODE_DONE;
            break;
        }
        else if(reg_val & 0x10) {
            DBG_ERR("reg_val: 0x%x at 0x%x\n", reg_val, REG_JDEC_INT_STATUS);
            drv->_status = STA_ERR_DECODING;
            break;
        }
    }
    if(i==0) {
        DBG_ERR("Timeout\n");
    }
  }
  #endif /* #ifdef JDEC_IRQ_MODE */
#else
    /* POST */
#endif /* #ifdef __KERNEL__ */

    return ret;
} /* End of wait_decode_finish() */

/*!*************************************************************************
* do_yuv2rgb
* 
* Private Function by Willy Chuang, 2008/11/24
*/
/*!
* \brief
*	YUV to RGB formula (provided by IC1 TK)
*
* \retval  none
*/ 
static void do_yuv2rgb(void)
{
    yuv2rgb_formula  mode = JFIF_0_255;
    
    /*--------------------------------------------------------------------------
      The formulat is 
      R = F0 * (Y - 16 * A)                   + F1 * (Cr - 128)
      G = F2 * (Y - 16 * A) - F3 * (Cb - 128) - F4 * (Cr - 128)
      B = F5 * (Y - 16 * A) + F6 * (Cb - 128)
    --------------------------------------------------------------------------*/

   if (mode == SDTV_16_235) {
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F0, 0x400); // REG_F0_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F1, 0x57C); // REG_F1_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F2, 0x400); // REG_F2_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F3, 0x2CB); // REG_F3_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F4, 0x158); // REG_F4_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F5, 0x400); // REG_F5_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F6, 0x6ED); // REG_F6_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_Y_SUB_16_EN, 0x000); // REG_Y_SUB_16_EN
     DBG_MSG("YUV2RGB Formula is Set to SDTV (16 ~ 235)\n");
   }
   else if (mode == SDTV_0_255) {
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F0, 0x4A8); // REG_F0_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F1, 0x662); // REG_F1_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F2, 0x4A8); // REG_F2_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F3, 0x341); // REG_F3_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F4, 0x190); // REG_F4_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F5, 0x4A8); // REG_F5_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F6, 0x812); // REG_F6_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_Y_SUB_16_EN, 0x001); // REG_Y_SUB_16_EN
     DBG_MSG("YUV2RGB Formula is Set to SDTV (0 ~ 255)\n");
   }
   else if (mode == HDTV_16_235) {
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F0, 0x400); // REG_F0_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F1, 0x629); // REG_F1_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F2, 0x400); // REG_F2_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F3, 0x1D6); // REG_F3_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F4, 0x0BB); // REG_F4_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F5, 0x400); // REG_F5_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F6, 0x744); // REG_F6_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_Y_SUB_16_EN, 0x000); // REG_Y_SUB_16_EN
     DBG_MSG("YUV2RGB Formula is Set to HDTV (16 ~ 235)\n");
   }
   else if (mode == HDTV_0_255) {
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F0, 0x4A8); // REG_F0_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F1, 0x72C); // REG_F1_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F2, 0x4A8); // REG_F2_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F3, 0x223); // REG_F3_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F4, 0x0DA); // REG_F4_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F5, 0x4A8); // REG_F5_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F6, 0x876); // REG_F6_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_Y_SUB_16_EN, 0x001); // REG_Y_SUB_16_EN
     DBG_MSG("YUV2RGB Formula is Set to HDTV (0 ~ 255)\n");
   }
   else if (mode == JFIF_0_255) {
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F0, 0x400); // REG_F0_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F1, 0x59C); // REG_F1_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F2, 0x400); // REG_F2_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F3, 0x2DB); // REG_F3_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F4, 0x160); // REG_F4_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F5, 0x400); // REG_F5_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_COEF_F6, 0x717); // REG_F6_COEF
     JDEC_REG_SET32 (REG_JDEC_YUV2RGB_Y_SUB_16_EN, 0x000); // REG_Y_SUB_16_EN
     DBG_MSG("YUV2RGB Formula is Set to JFIF (0 ~ 255)\n");
   }

   JDEC_REG_SET32 (REG_JDEC_RGB_ALPHA, 0); // always set 0
   JDEC_REG_SET32 (REG_JDEC_YUV2RGB_ENABLE, 1);  // YUV2RGB Enable Setting
   
   return;
} /* End of YUV2RGB_Formula_Setting() */

/*!*************************************************************************
* do_chroma_scale_up
* 
* Private Function by Willy Chuang, 2008/11/24
*/
/*!
* \brief
*	YUV to RGB formula (provided by IC1 TK)
*
* \retval  none
*/ 
int do_chroma_scale_up(jdec_drvinfo_t *drv)
{
    jdec_hdr_info_t    *hdr;
    jdec_decode_info_t *di;
    int chroma_scale = 0x00;
    int CH_H_scale = 0, CH_V_scale = 0;

    hdr = &drv->attr.hdr;
    di  = &drv->attr.di;

    CH_H_scale = CH_V_scale = drv->scale_ratio;
    /*--------------------------------------------------------------------------
        Do chroma Scale-up setting
    --------------------------------------------------------------------------*/
    if(di->scale_factor == SCALE_ORIGINAL) {
        /*----------------------------------------------------------------------
            Decode to 1:1
        ----------------------------------------------------------------------*/
        if(di->decoded_color == VDO_COL_FMT_YUV444) {
            if(hdr->src_color == VDO_COL_FMT_YUV420) 
                chroma_scale = 0x0101;
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                chroma_scale = 0x0100;
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                chroma_scale = 0x0001;
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                chroma_scale = 0x0100;
        } 
        else if(di->decoded_color == VDO_COL_FMT_YUV422H) {
            if(hdr->src_color == VDO_COL_FMT_YUV420) {
                chroma_scale = 0x0100;
            }
        } 
        else if(di->decoded_color == VDO_COL_FMT_YUV420) {
            if(hdr->src_color == VDO_COL_FMT_YUV420) {
                // Do nothing
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV422H) {
                CH_V_scale = 0x01;
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV422V) {
                CH_H_scale = 0x01;
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV444) {
                CH_H_scale = 0x01;
                CH_V_scale = 0x01;
            }
            else {
                DBG_ERR("Not supported scale up (color: %d -> %d) now\n", 
                         hdr->src_color, di->decoded_color);
            }
        } 
        else if(di->decoded_color == VDO_COL_FMT_ARGB) {
            // Do nothing here
        }
    }
    else if(di->scale_factor == SCALE_HALF) {
        /*----------------------------------------------------------------------
            Decode to 1/2
        ----------------------------------------------------------------------*/
        if(di->decoded_color == VDO_COL_FMT_YUV444) {
            if(hdr->src_color == VDO_COL_FMT_YUV420) {
                CH_H_scale = 0; // 1/2 --> 1/1
                CH_V_scale = 0; // 1/2 --> 1/1
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 0; // 1/2 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                CH_H_scale = 0; // 1/2 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_V_scale = 0; // 1/2 --> 1/1
        } 
        else if(di->decoded_color == VDO_COL_FMT_YUV422H) {
            if(hdr->src_color == VDO_COL_FMT_YUV420)
                CH_V_scale = 0; // 1/2 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 0; // 1/2 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV422V){
                CH_H_scale = 2; // 1/2 --> 1/4
                CH_V_scale = 0; // 1/2 --> 1/1
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV444)
                CH_H_scale = 2; // 1/2 --> 1/4
        }
        else if(di->decoded_color == VDO_COL_FMT_YUV420) {
            if(hdr->src_color == VDO_COL_FMT_YUV411) {
                CH_H_scale = 0; // 1/2 --> 1/1
                CH_V_scale = 2; // 1/2 --> 1/4
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                CH_V_scale = 2; // 1/2 --> 1/4
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_H_scale = 2; // 1/2 --> 1/4
        }
        else if(di->decoded_color == VDO_COL_FMT_ARGB) {
            if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 0; // 1/2 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_V_scale = 0; // 1/2 --> 1/1
        }
    }
    else if(di->scale_factor == SCALE_QUARTER) {
        /*----------------------------------------------------------------------
            Decode to 1/4
        ----------------------------------------------------------------------*/
        if(di->decoded_color == VDO_COL_FMT_YUV444) {
            if(hdr->src_color == VDO_COL_FMT_YUV420) {
                CH_H_scale = 1; // 1/4 --> 1/2
                CH_V_scale = 1; // 1/4 --> 1/2
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 0; // 1/4 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                CH_H_scale = 1; // 1/4 --> 1/2
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_V_scale = 1; // 1/4 --> 1/2
        } 
        else if(di->decoded_color == VDO_COL_FMT_YUV422H) {
            if(hdr->src_color == VDO_COL_FMT_YUV420)
                CH_V_scale = 1; // 1/4 --> 1/2
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 1; // 1/4 --> 1/2
            else if(hdr->src_color == VDO_COL_FMT_YUV422V){
                CH_H_scale = 3; // 1/4 --> 1/8
                CH_V_scale = 1; // 1/4 --> 1/2
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV444)
                CH_H_scale = 3; // 1/4 --> 1/8
        }
        else if(di->decoded_color == VDO_COL_FMT_YUV420) {
            if(hdr->src_color == VDO_COL_FMT_YUV411){
                CH_H_scale = 1; // 1/4 --> 1/2
                CH_V_scale = 3; // 1/4 --> 1/8
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                CH_V_scale = 3; // 1/4 --> 1/8
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_H_scale = 3; // 1/4 --> 1/8
            else if(hdr->src_color == VDO_COL_FMT_YUV444){
                CH_H_scale = 3; // 1/4 --> 1/8
                CH_V_scale = 3; // 1/4 --> 1/8
            }           
        }
        else if(di->decoded_color == VDO_COL_FMT_ARGB) {
            if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 0; // 1/4 --> 1/1
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_V_scale = 1; // 1/4 --> 1/2
        }
    }
    else if(di->scale_factor == SCALE_EIGHTH) {
        /*----------------------------------------------------------------------
            Decode to 1/8
        ----------------------------------------------------------------------*/
        if(di->decoded_color == VDO_COL_FMT_YUV444) {
            if(hdr->src_color == VDO_COL_FMT_YUV420) {
                CH_H_scale = 2; // 1/8 --> 1/4
                CH_V_scale = 2; // 1/8 --> 1/4
            }
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 1; // 1/8 --> 1/2
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                CH_H_scale = 2; // 1/8 --> 1/4
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_V_scale = 2; // 1/8 --> 1/4
        } 
        else if(di->decoded_color == VDO_COL_FMT_YUV422H) {
            if(hdr->src_color == VDO_COL_FMT_YUV420)
                CH_V_scale = 2; // 1/8 --> 1/4
            else if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 2; // 1/8 --> 1/4
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                DBG_ERR("Not support 422 case 1 for 1/8\n");
            else if(hdr->src_color == VDO_COL_FMT_YUV444)
                DBG_ERR("Not support 422 case 2 for 1/8\n");
        }
        else if(di->decoded_color == VDO_COL_FMT_YUV420) {
            if(hdr->src_color == VDO_COL_FMT_YUV411)
                DBG_ERR("Not support 420 case 1 for 1/8\n");
            else if(hdr->src_color == VDO_COL_FMT_YUV422H)
                DBG_ERR("Not support 420 case 2 for 1/8\n");
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                DBG_ERR("Not support 420 case 3 for 1/8\n");
            else if(hdr->src_color == VDO_COL_FMT_YUV444)
                DBG_ERR("Not support 420 case 4 for 1/8\n");
        }
        else if(di->decoded_color == VDO_COL_FMT_ARGB) {
            if(hdr->src_color == VDO_COL_FMT_YUV411)
                CH_H_scale = 2; // 1/8 --> 1/4
            else if(hdr->src_color == VDO_COL_FMT_YUV422V)
                CH_V_scale = 2; // 1/8 --> 1/4
        }
    }
    else {
        DBG_ERR("color: %d -> %d) now\n", hdr->src_color, di->decoded_color);
        DBG_ERR("Not supported chroma scale up (%d) now\n", di->scale_factor);
    }
    JDEC_REG_SET32(REG_JDEC_CHROMA_ENABLE, chroma_scale);

    JDEC_REG_SET32(REG_JDEC_Y_SCALE_RATIO,   drv->scale_ratio);
    JDEC_REG_SET32(REG_JDEC_C_SCALE_RATIO_H, CH_H_scale);
    JDEC_REG_SET32(REG_JDEC_C_SCALE_RATIO_V, CH_V_scale);

    return 0;
} /* End of do_chroma_scale_up() */

/*!*************************************************************************
* jdec_reset
* 
* Private Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	JPEG hardware reset
*
* \retval  0 if success
*/ 
static int jdec_reset(void)
{
    /* Enable clock */
    jdec_clock_en(1);
    
    /*--------------------------------------------------------------------------
        Step 1: Do SW reset
    --------------------------------------------------------------------------*/
    JDEC_REG_SET32(REG_JDEC_SW_RESET, 1); /* Enable SW reset */
    JDEC_REG_SET32(REG_JDEC_SW_RESET, 0); /* Disable SW reset */

    JDEC_REG_SET32(REG_JDEC_INT_STATUS, 0x1F);  /* Clear INT staus (write 1 clear) */

    JDEC_REG_SET32(REG_JDEC_PARTIAL_ENABLE, 0);  /* Partial decode Enable */
    JDEC_REG_SET32(REG_JDEC_YUV2RGB_ENABLE, 0);  /* YUV2RGB Enable */
	    
    s_msd_backup_len = 0;

    return 0;
} /* End of jdec_reset() */

/*!*************************************************************************
* wmt_jdec_set_drv
* 
* API Function by Willy Chuang, 2009/9/29
*/
/*!
* \brief
*	Set current JPEG driver objet
*
* \retval  0 if success
*/ 
int wmt_jdec_set_drv(jdec_drvinfo_t *drv)
{
    current_drv = drv;
    
    return 0;
} /* End of wmt_jdec_set_drv() */

/*!*************************************************************************
* wmt_jdec_get_drv
* 
* API Function by Willy Chuang, 2009/9/29
*/
/*!
* \brief
*	get current JPEG driver objet
*
* \retval  0 if success
*/ 
jdec_drvinfo_t * wmt_jdec_get_drv(void)
{
    return current_drv;
} /* End of wmt_jdec_get_drv() */

/*!*************************************************************************
* wmt_jdec_get_capability
* 
* API Function by Willy Chuang, 2008/12/5
*/
/*!
* \brief
*	Get JPEG hardware capability
*
* \retval  0 if success
*/ 
int wmt_jdec_get_capability(jdec_drvinfo_t *drv)
{
    drv->capab_tbl = &capab_table;

    return 0;
} /* End of wmt_jdec_get_capability() */

/*!*************************************************************************
* wmt_jdec_set_attr
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	JPEG hardware initialization
*
* \retval  0 if success
*/ 
int wmt_jdec_set_attr(jdec_drvinfo_t *drv)
{
    jdec_hdr_info_t    *hdr;
    jdec_decode_info_t *di;
    jdec_color_format   src_color;
    jdec_color_format   dst_color;
    unsigned int        scanline;
    int factor_c;
    
    /*==========================================================================
        When this function is called, it means a new JPEG picture will come.
        In this case, we must stop old jobs and do reset.
    ==========================================================================*/
    hdr = &drv->attr.hdr;
    di  = &drv->attr.di;
    
    drv->_status = STA_ATTR_SET;

    jdec_reset();

    /*--------------------------------------------------------------------------
        Step 1: Set core settings
    --------------------------------------------------------------------------*/
    get_mcu_unit(hdr->sof_w, hdr->sof_h, hdr->src_color, 
                 &drv->src_mcu_width, &drv->src_mcu_height);

    JDEC_REG_SET32(REG_JDEC_SRC_WIDTH_MCU,  drv->src_mcu_width);
    JDEC_REG_SET32(REG_JDEC_SRC_HEIGHT_MCU, drv->src_mcu_height);

    src_color = hdr->src_color;
    dst_color = di->decoded_color;
    
    /* Calculate time out value */
    /* According to experience, it should be save to set 5x10e-5 ms per pixel */
    drv->_timeout = ((hdr->sof_w * hdr->sof_h)*6/100000); // ms
    if( drv->_timeout < 100 ){
        /* Set the smallest time out interval as 100 ms */
        drv->_timeout = 100;
    }
    DBG_MSG("Timeout threshold: %d ms\n", drv->_timeout);
    
    /* LOOK OUT: The order of vd_color_space is based on WM3426 JDEC spce. If it 
       is differernt with the order of your HW decoder, driver should tx it. */
    JDEC_REG_SET32(REG_JDEC_SRC_COLOR_FORMAT, src_color);
        
    /*--------------------------------------------------------------------------
        Step 2: Partial decode settings 
    --------------------------------------------------------------------------*/
    if(di->pd.enable) {
        JDEC_REG_SET32(REG_JDEC_PARTIAL_ENABLE,  1);  /* Partial decode Enable */

        get_mcu_unit(di->pd.x, di->pd.y, src_color, &drv->pd_mcu.x, &drv->pd_mcu.y);
        get_mcu_unit(di->pd.w, di->pd.h, src_color, &drv->pd_mcu.w, &drv->pd_mcu.h);

        JDEC_REG_SET32(REG_JDEC_DST_MCU_WIDTH,   drv->pd_mcu.w);
        JDEC_REG_SET32(REG_JDEC_DST_MCU_HEIGHT,  drv->pd_mcu.h);

        JDEC_REG_SET32(REG_JDEC_PARTIAL_H_START, drv->pd_mcu.x);
        JDEC_REG_SET32(REG_JDEC_PARTIAL_V_START, drv->pd_mcu.y);       
        JDEC_REG_SET32(REG_JDEC_PARTIAL_WIDTH,   drv->pd_mcu.w);
        JDEC_REG_SET32(REG_JDEC_PARTIAL_HEIGHT,  drv->pd_mcu.h);
    }
    else {
        JDEC_REG_SET32(REG_JDEC_PARTIAL_ENABLE,  0);

        JDEC_REG_SET32(REG_JDEC_DST_MCU_WIDTH,  drv->src_mcu_width);
        JDEC_REG_SET32(REG_JDEC_DST_MCU_HEIGHT, drv->src_mcu_height);

        JDEC_REG_SET32(REG_JDEC_PARTIAL_H_START, 0);
        JDEC_REG_SET32(REG_JDEC_PARTIAL_V_START, 0);
        JDEC_REG_SET32(REG_JDEC_PARTIAL_WIDTH,   0);
        JDEC_REG_SET32(REG_JDEC_PARTIAL_HEIGHT,  0);
    }
    /*--------------------------------------------------------------------------
        Step 3: Picture scaling control 
    --------------------------------------------------------------------------*/
    switch(di->scale_factor) {
        case SCALE_BEST_FIT:
        case SCALE_ORIGINAL:
            drv->scale_ratio = 0;
            break;
        case SCALE_HALF:
            drv->scale_ratio = 1;
            break;
        case SCALE_QUARTER:
            drv->scale_ratio = 2;
            break;
        case SCALE_EIGHTH:
            drv->scale_ratio = 3;
            break;
        default:
            DBG_ERR("Illegal scale ratio(%d)\n", di->scale_factor);
            drv->scale_ratio = 0;
            break;
    }
    /*--------------------------------------------------------------------------
        Step 4: Handle scanline offset issue
    --------------------------------------------------------------------------*/
    if(di->pd.enable) {
        drv->decoded_w = di->pd.w;
        drv->decoded_h = di->pd.h;
    }
    else {
        drv->decoded_w = hdr->sof_w;
        drv->decoded_h = hdr->sof_h;
    }
    scanline = drv->decoded_w;
    
    
    /* Because of HW limitation, Y line width should be multiple of 64. */
    switch(dst_color) {
        case VDO_COL_FMT_YUV420:
            drv->line_width_y = ALIGN64( ALIGN64(scanline) >> drv->scale_ratio );
            drv->line_width_y = ALIGN64( MAX(drv->line_width_y, di->scanline));
            drv->line_width_c = drv->line_width_y;
            drv->bpp = 8;
            factor_c = 2;
            break;
        case VDO_COL_FMT_YUV422H:
            drv->line_width_y = ALIGN64( ALIGN64(scanline) >> drv->scale_ratio );
            drv->line_width_y = ALIGN64( MAX(drv->line_width_y, di->scanline));
            drv->line_width_c = drv->line_width_y;
            drv->bpp = 8;
            factor_c = 1;
            break;
        case VDO_COL_FMT_YUV422V:
            drv->line_width_y = ALIGN64( ALIGN64(scanline) >> drv->scale_ratio );
            drv->line_width_y = ALIGN64( MAX(drv->line_width_y, di->scanline));
            drv->line_width_c = drv->line_width_y << 1;
            drv->bpp = 8;
            factor_c = 2;
            break;
        case VDO_COL_FMT_YUV444:
            drv->line_width_y = ALIGN64( ALIGN64(scanline) >> drv->scale_ratio );
            drv->line_width_y = ALIGN64( MAX(drv->line_width_y, di->scanline));
            drv->line_width_c = drv->line_width_y << 1;
            drv->bpp = 8;
            factor_c = 1;
            break;
        case VDO_COL_FMT_YUV411:
            drv->line_width_c = ALIGN64( ALIGN64(scanline) >> drv->scale_ratio );
            drv->line_width_c = ALIGN64( MAX(drv->line_width_c, di->scanline));
            drv->line_width_y = drv->line_width_c << 1;
            drv->bpp = 8;
            factor_c = 2;
            break;
        case VDO_COL_FMT_ARGB:
            drv->line_width_y = ALIGN64( ALIGN64(scanline << 2) >> drv->scale_ratio );
            drv->line_width_y = ALIGN64( MAX(drv->line_width_y, di->scanline));
            drv->line_width_c = 0;
            do_yuv2rgb();
            drv->bpp = 32;
            factor_c = 1;
            break;
        case VDO_COL_FMT_GRAY:
            drv->line_width_y = ALIGN64( ALIGN64(scanline) >> drv->scale_ratio );
            drv->line_width_y = ALIGN64( MAX(drv->line_width_y, di->scanline));
            drv->line_width_c = 0;
            drv->bpp = 8;
            factor_c = 1;
            break;
        default:
            DBG_ERR("Unknown color format(%d)\n", dst_color);
            factor_c = 1; // to avoid divid zero
            break;
    }
    DBG_MSG("sof_w: %d, sof_h: %d\n", hdr->sof_w, hdr->sof_h);
    DBG_MSG("line_width_y: %d, line_width_c: %d\n", drv->line_width_y, drv->line_width_c);

    drv->line_height = (((drv->decoded_h + 15)>>4)<<4) >> drv->scale_ratio;
    drv->req_y_size  = (drv->line_width_y * drv->line_height) >> drv->scale_ratio;
    drv->req_c_size  = ((drv->line_width_c * drv->line_height) >> drv->scale_ratio)/ factor_c;

    DBG_MSG("Required memory size(Y, C): (%d, %d)\n", drv->req_y_size, drv->req_c_size);

    do_chroma_scale_up(drv);

    JDEC_REG_SET32(REG_JDEC_Y_LINE_WIDTH, drv->line_width_y);
    JDEC_REG_SET32(REG_JDEC_C_LINE_WIDTH, drv->line_width_c);

    return 0;
} /* End of wmt_jdec_set_attr() */

/*!*************************************************************************
* set_prd_table
* 
* API Function by Willy Chuang, 2009/12/16
*/
/*!
* \brief
*	Set source PRD table
*
* \retval  0 if success
*/ 
static int set_prd_table(jdec_drvinfo_t *drv)
{
    unsigned int  prd_addr_in;
    unsigned int  flush_en = 0, flush_len = 0;
    unsigned int  ptr_vir;
    int i;
    int extra_prd_item_en = 0;

    DBG_PRD("Dma prd virt addr: 0x%08x\n", drv->prd_virt);
    DBG_PRD("Dma prd phy  addr: 0x%08x\n", drv->prd_addr);

    if( drv->_multi_seg_en ) {
        prd_addr_in = drv->prd_virt + drv->_prd_offset;
        DBG_PRD("[MSD enabled] prd_addr_in: %p\n", (void *)prd_addr_in);
    }
    else {
        prd_addr_in = drv->prd_virt;
    }
    
    for(i=0; ; i+=2) {
        unsigned int  addr, len, addr2, len2;

        addr = *(unsigned int *)(prd_addr_in + i * 4);
        len  = *(unsigned int *)(prd_addr_in + (i + 1) * 4);

        DBG_PRD("[%02d]Addr: 0x%08x\n", i, addr);
        DBG_PRD("    Len:  0x%08x (%d)\n", len, (len & 0xFFFF));
  
        if(i == 0) {
            if((addr & 0x1F) || (len & 0x1F)) {
                /*--------------------------------------------------------------
                    If the first block is not aligment of 32
                --------------------------------------------------------------*/
                addr2 = (addr & 0xFFFFFFE0);
                len2  = ((len + 31)>>5)<<5;

                flush_en = 1;
                flush_len = addr - addr2;

                if(s_msd_backup_len == 0) {
                    *(unsigned int *)(prd_addr_in + i * 4) = addr2;
                    *(unsigned int *)(prd_addr_in + (i + 1) * 4) = len2;

                    DBG_PRD("*[%02d]Addr: 0x%08x\n", i, addr2);
                    DBG_PRD("*    Len:  0x%08x (%d)\n", len2, (len2 & 0xFFFF));
                }
            }
            /*------------------------------------------------------------------
                 Handle Multi-Segment Decoding
            ------------------------------------------------------------------*/
            if( drv->_multi_seg_en && (s_msd_backup_len != 0) ){
                unsigned char *ptr;

                extra_prd_item_en = 1;
                /* pad extra data in previous segment */
                DBG_PRD("flush_len: %d, s_msd_backup_len: %d\n", flush_len, s_msd_backup_len);

                addr2 = ((addr + 31)>>5)<<5;

                if( flush_len >= s_msd_backup_len ) {
                    flush_len -= s_msd_backup_len;                   
                    flush_en = (flush_len == 0)? 0: 1;

                    *(unsigned int *)(prd_addr_in + i * 4) = addr2;
                    *(unsigned int *)(prd_addr_in + (i + 1) * 4) = (len -(addr2-addr));

                    DBG_PRD("*[%02d]Addr: 0x%08x\n", i, addr2);
                    DBG_PRD("*    Len:  0x%08x (%d)\n", (len -(addr2-addr)), ((len -(addr2-addr)) & 0xFFFF));

                    *(unsigned int *)(drv->prd_virt)     = virt_to_phys(msd_prd_virt);
                    *(unsigned int *)(drv->prd_virt + 4) = s_msd_backup_len + (addr2 - addr);

                    DBG_PRD("*[-]Addr: %p\n", (void *)virt_to_phys(msd_prd_virt));
                    DBG_PRD("*   Len:  %d\n", s_msd_backup_len + (addr2 - addr));

                    /* copy data */
                    ptr_vir = (unsigned int)phys_to_virt(addr);

                    ptr = (char *)(msd_prd_virt + flush_en);
                    memcpy(ptr, msd_buf, s_msd_backup_len);
                    ptr += s_msd_backup_len;
                    memcpy(ptr, (char *)ptr_vir, addr2 - addr);
                }
                else {
                    /* flush_len < s_msd_backup_len */
                    flush_len = 32 - (s_msd_backup_len - flush_len);                   
                    flush_en = (flush_len == 0)? 0: 1;

                    /* copy data */
                    ptr_vir = (unsigned int)phys_to_virt(addr);

                    ptr = (char *)(msd_prd_virt + flush_en);
                    memcpy(ptr, msd_buf, s_msd_backup_len);
                    ptr += s_msd_backup_len;
                    memcpy(ptr, (char *)ptr_vir, addr2 - addr);
            
                    *(unsigned int *)(prd_addr_in + i * 4) = addr2;
                    *(unsigned int *)(prd_addr_in + (i + 1) * 4) = (len -(addr2-addr));

                    DBG_PRD("*[%02d]Addr: 0x%08x\n", i, addr2);
                    DBG_PRD("*    Len:  0x%08x (%d)\n", (len -(addr2-addr)), ((len -(addr2-addr)) & 0xFFFF));

                    *(unsigned int *)(drv->prd_virt)     = virt_to_phys(msd_prd_virt);
                    *(unsigned int *)(drv->prd_virt + 4) = s_msd_backup_len + (addr2 - addr);

                    DBG_PRD("*[-]Addr: %p\n", (void *)virt_to_phys(msd_prd_virt));
                    DBG_PRD("*   Len:  %d\n", s_msd_backup_len + (addr2 - addr));

                }
            }
            if(flush_en) {
                DBG_MSG("Flush enable (len: %d)\n", flush_len);
                JDEC_REG_SET32(REG_JDEC_BSDMA_FLUSH_ENABLE, 0x100 | flush_len);
            }
            else {
                JDEC_REG_SET32(REG_JDEC_BSDMA_FLUSH_ENABLE, 0x0);
            }
        }
        /*----------------------------------------------------------------------
             Handle last block
        ----------------------------------------------------------------------*/
        if(len & 0x80000000) {
            len2  = ((len + 31)>>5)<<5;
            if( ((len & 0xFFFF) & 0x1F ) || (flush_len)) {
                /*--------------------------------------------------------------
                    If the lenght of last block is not aligment of 32
                --------------------------------------------------------------*/
                if(i == 0) {
                    len2  = ((len + flush_len + 31)>>5)<<5;
                }

                if(drv->_multi_seg_en){
                    if(!(drv->_segment & JDEC_SEG_LAST)) {
                        len2  = ((len + flush_len)>>5)<<5;
                        s_msd_backup_len = len - len2;

                        *(unsigned int *)(prd_addr_in + (i + 1) * 4) = len2;

                        ptr_vir = *(unsigned int *)(prd_addr_in + i * 4); /* phy. addr */
                        ptr_vir = (unsigned int)phys_to_virt(ptr_vir);

                        if( (len2 & 0xFFFF) == 0 ) {
                            unsigned int prev_len;
                            /* in this case, the prvisou block is last block.
                               We must set end flag bit for it */
                            DBG_PRD("Lenth of last block is 0\n");
                            prev_len = *(unsigned int *)(prd_addr_in + (i - 1) * 4);
                            prev_len |= 0x80000000;
                            *(unsigned int *)(prd_addr_in + (i - 1) * 4) = prev_len;

                            DBG_PRD("*[%02d]Len:  0x%08x (%d)\n", i-1, prev_len, (prev_len & 0xFFFF));
                        }            
                        /* copy extra data to msb_buf. They will be used in next segment */
                        memcpy(msd_buf, (unsigned char *)(ptr_vir + (len2 & 0xFFFF)), s_msd_backup_len);
                    } /* if(!(drv->_segment & JDEC_SEG_LAST)) */
                }
                else {
                    *(unsigned int *)(prd_addr_in + (i + 1) * 4) = len2;
                }

                DBG_PRD("*    Len:  0x%08x (%d)\n", len2, (len2 & 0xFFFF));
            }
            if( (drv->_multi_seg_en) && (!(drv->_segment & JDEC_SEG_LAST)) ){
                /* Not last segment, do nothing */
                break;
            }
#ifdef SW_PATCH_FOR_ZERO_BITS_AHEAD_EOI
{
            #define PATCH_BYTES    32
            unsigned int copy_len;
            unsigned char *ptr;
            int found_EOI = 0;
            int j;

            ptr_vir = *(unsigned int *)(prd_addr_in + i * 4); /* phy. addr */
            ptr_vir = (unsigned int)phys_to_virt(ptr_vir);

            len2 = ((len + flush_len) & 0xFFE0);
            len2 = (len2 > 32)? (len2 - 32): len2;
            copy_len = ((len + flush_len) & 0xFFFF) - len2;
            
            *(unsigned int *)(prd_addr_in + (i + 1) * 4) = len2; 
            DBG_PRD("*    Len:  0x%08x (%d)\n", len2, len2);

            /*------------------------------------------------------------------
                Copy last segment to temp buffer and refine PRD table
            ------------------------------------------------------------------*/
#ifdef JDEC_PRD_DEBUG
            memset(tmp_prd_buf, 0xAA, 128);
#endif
            memcpy(tmp_prd_buf, (unsigned char *)(ptr_vir + len2), copy_len);
            i = i + 2;
            *(unsigned int *)(prd_addr_in + i * 4) = last_prd_phy_addr;
            *(unsigned int *)(prd_addr_in + (i + 1) * 4) = (128 | 0x80000000);

            DBG_PRD("*[%02d]Addr: 0x%08x\n", i, last_prd_phy_addr);
            DBG_PRD("*    Len:  0x%08x (%d)\n", (128 | 0x80000000), 128);

            /*------------------------------------------------------------------
                Check whether EOI in this last segment
            ------------------------------------------------------------------*/
            if(copy_len < 2 ) {
                DBG_PRD("Exception case in last RPD(Len: %d)!\n", copy_len);
                ptr = (unsigned char *)tmp_prd_buf;
                *ptr++ = 0xFF; 
                found_EOI = 1;
            }
            else {
                ptr = (unsigned char *)(tmp_prd_buf + copy_len - 2);
                if( (*(ptr) != 0xFF) || (*(ptr+1) != 0xD9) ) {
                    DBG_PRD("Last two bytes is not EOI\n");
                    ptr--;
                    while(ptr > tmp_prd_buf) {
                        if( (*(ptr) == 0xFF) && (*(ptr+1) == 0xD9) ) {
                            DBG_PRD("EOI was found\n");
                            found_EOI = 1;
                            break;
                        }
                        ptr--;
                    }
                }
                else {
                    found_EOI = 1;
                }
            }
            /*------------------------------------------------------------------
                Start to add patch bytes if the last segment
            ------------------------------------------------------------------*/
            if( found_EOI ) {
                for(j=0; j<PATCH_BYTES; j++) {
                    /* Add N-bytes zero before EOI to patch HW bug */
                    *(ptr+j) = 0;
                }
                ptr += PATCH_BYTES;
                *ptr++ = 0xFF;
                *ptr++ = 0xD9;
#ifdef SW_PATCH_FOR_EOI
                *ptr++ = 0xFF;
                *ptr++ = 0xFF;
#endif
            }
}
#endif //#ifdef SW_PATCH_FOR_ZERO_BITS_AHEAD_EOI
            break;
        } /* if(len & 0x80000000) */
        flush_en = 0;
        flush_len = 0;
    } /* for(i=0; ; i+=2) */

    if( (drv->_multi_seg_en) && (extra_prd_item_en == 0) ) {
        JDEC_REG_SET32(REG_JDEC_BSDMA_PRD_ADDR, drv->prd_addr + drv->_prd_offset);
    }
    else {
        JDEC_REG_SET32(REG_JDEC_BSDMA_PRD_ADDR, drv->prd_addr);
    }
    
    DBG_MSG("PRD_ADDR: 0x%x \n", REG32_VAL(REG_JDEC_BSDMA_PRD_ADDR));
    
    return 0;
} /* End of set_prd_table() */

/*!*************************************************************************
* wmt_jdec_decode_proc
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	Open JPEG hardware
*
* \retval  0 if success
*/ 
int wmt_jdec_decode_proc(jdec_drvinfo_t *drv)
{
    unsigned int  y_base, c_base;
//    unsigned int  flush_en = 0, flush_len = 0;
    jdec_input_t *arg;
    jdec_attr_t  *attr;
    int ret = 0;
   
    /*--------------------------------------------------------------------------
        Step 1: Check input arguments
    --------------------------------------------------------------------------*/
    arg = &drv->arg_in;
    attr = &drv->attr;

    /*--------------------------------------------------------------------------
        Step 2: Set PRD table
    --------------------------------------------------------------------------*/
    set_prd_table(drv);

    /*--------------------------------------------------------------------------
        Step 3: Check whether space of decoded buffer is enough or not
    --------------------------------------------------------------------------*/
    if(attr->di.decoded_color != VDO_COL_FMT_ARGB) {
        /* YUV color domain */
        if(arg->dst_y_size < drv->req_y_size) {
            DBG_ERR("Decoded buffer size (Y) is not enough (%d < %d)\n", 
                                        arg->dst_y_size, drv->req_y_size);
            goto EXIT_wmt_jdec_decode_proc;
        }
        if(arg->dst_c_size < drv->req_c_size) {
            DBG_ERR("Decoded buffer size (C) is not enough (%d < %d)\n", 
                                        arg->dst_c_size, drv->req_c_size);
            goto EXIT_wmt_jdec_decode_proc;
        }
    }
    else {
        /* RGB color domain */
        if(arg->dst_y_size < drv->req_y_size) {
            DBG_ERR("Decoded buffer size (ARGB) is not enough (%d < %d)\n", 
                                        arg->dst_y_size, drv->req_y_size);
            DBG_ERR("line_width_y: %d, line_height: %d\n", drv->line_width_y, drv->line_height);
                        
            goto EXIT_wmt_jdec_decode_proc;
        }
    }    
    if(arg->dst_y_addr == 0) {
        DBG_ERR("NULL Y Addr! Stop decoding!\n");
        goto EXIT_wmt_jdec_decode_proc;
    }
/*    if(arg->dst_c_addr == 0) {
        DBG_MSG("Warning: C Addr is NULL!\n");
    }    */
    /*--------------------------------------------------------------------------
        Step 4: 
    --------------------------------------------------------------------------*/
    if((arg->linesize !=0) && (arg->linesize > drv->line_width_y)) {
        PRINT("linesize: %d\n", arg->linesize);
        /* In this case, we only support 4:2:0 mode */
        drv->line_width_y = arg->linesize;
        drv->line_width_c = drv->line_width_y >> 1; // temp
        
        JDEC_REG_SET32(REG_JDEC_Y_LINE_WIDTH, drv->line_width_y);
        JDEC_REG_SET32(REG_JDEC_C_LINE_WIDTH, drv->line_width_c);
    }
    
    /* Physical address */
    y_base = arg->dst_y_addr;
    c_base = arg->dst_c_addr;

    if((arg->dec_to_x == 0) && ((arg->dec_to_y == 0))) {
        /* Decode to start address of destination buffer */
    }
    else {
        /* Decode to any postion of destination buffer */
        if(drv->attr.di.decoded_color == VDO_COL_FMT_YUV422H) {
            y_base += arg->dec_to_y * drv->line_width_y + arg->dec_to_x;
            c_base += arg->dec_to_y * drv->line_width_c + arg->dec_to_x;
            DBG_MSG("[422H] Phy. Y addr changed from: 0x%x to 0x%x\n", arg->dst_y_addr, y_base);
            DBG_MSG("[422H] Phy. C addr changed from: 0x%x to 0x%x\n", arg->dst_c_addr, c_base);
        }
        else if(drv->attr.di.decoded_color == VDO_COL_FMT_YUV444) {
            y_base += arg->dec_to_y * drv->line_width_y + arg->dec_to_x;
            c_base += arg->dec_to_y * drv->line_width_c + (arg->dec_to_x << 1);
            DBG_MSG("[444] Phy. Y addr changed from: 0x%x to 0x%x\n", arg->dst_y_addr, y_base);
            DBG_MSG("[444 Phy. C addr changed from: 0x%x to 0x%x\n", arg->dst_c_addr, c_base);
        }
        else if(drv->attr.di.decoded_color == VDO_COL_FMT_ARGB) {
            y_base += arg->dec_to_y * drv->line_width_y + (arg->dec_to_x << 2);
            c_base = 0;
            DBG_MSG("[ARGB] Phy. Y addr changed from: 0x%x to 0x%x\n", arg->dst_y_addr, y_base);
            DBG_MSG("[ARGB] Phy. C addr changed from: 0x%x to 0x%x\n", arg->dst_c_addr, c_base);
        }
        if(drv->attr.di.decoded_color == VDO_COL_FMT_YUV420) {
            y_base += arg->dec_to_y * drv->line_width_y + arg->dec_to_x;
            c_base += (arg->dec_to_y >> 1) * drv->line_width_c + arg->dec_to_x;
            DBG_MSG("[420] Phy. Y addr changed from: 0x%x to 0x%x\n", arg->dst_y_addr, y_base);
            DBG_MSG("[420] Phy. C addr changed from: 0x%x to 0x%x\n", arg->dst_c_addr, c_base);
        }
        else {
            DBG_MSG("decoded_color: %d\n", drv->attr.di.decoded_color);
        }
    }
    JDEC_REG_SET32(REG_JDEC_YBASE, y_base);
    JDEC_REG_SET32(REG_JDEC_CBASE, c_base);
    /*--------------------------------------------------------------------------
        Step 4: Interrupt enable
                bit 4: JPEG bitstream ERROR INT enable
                bit 3: JBSDMA INT enable
                bit 2: JPEG HW parsing SOF INT enable
                bit 1: VLD decode finish INT enable
                bit 0: JPEG decoded frame out finish INT enable
    --------------------------------------------------------------------------*/
    drv->_status = STA_DMA_START;

    if( (drv->_multi_seg_en == 0) ||
        ((drv->_multi_seg_en) && (drv->_segment & JDEC_SEG_FIRST)) ){

        JDEC_REG_SET32(REG_JDEC_SW_RESET, 1); /* Enable SW reset */
        JDEC_REG_SET32(REG_JDEC_SW_RESET, 0); /* Disable SW reset */

        /* Befoe we start JBSDMA, we must do pictuer init */
        JDEC_REG_SET32(REG_JDEC_INIT, 1);  // Picture Init
        JDEC_REG_SET32(REG_JDEC_INIT, 0);  // End of Picture Init
    }
    JDEC_REG_SET32(REG_JDEC_INT_ENALBE, 0x19);

    JDEC_REG_SET32(REG_JDEC_BSDMA_START_TX, 1);  // Set JBSDMA Start Enable

    if(drv->_multi_seg_en) {
        if( !(drv->_segment & JDEC_SEG_LAST) ) {
            DBG_MSG("Wait for segment decoding finished\n");

            /* In multi-segment, we must wait DMA done or decoding finished */
            if( wait_event_interruptible_timeout( jdec_wait, 
                (drv->_status & STA_DMA_MOVE_DONE) ||
                (drv->_status & STA_DECODE_DONE), drv->_timeout) == 0){
                    DBG_ERR("[MSD] DMA move Time out: %d ms\n", drv->_timeout);

                    drv->_status &= (~STA_DMA_START);
                    drv->_status |= STA_ERR_DMA_TX;
                    stop_bsdam();
                    ret = -1;
            }
            else {
                DBG_MSG("DMA move done already\n");
            }
        }
    }
EXIT_wmt_jdec_decode_proc:

    return ret;
} /* End of wmt_jdec_decode_proc() */

/*!*************************************************************************
* set_frame_info
* 
* API Function by Willy Chuang, 2009/12/03
*/
/*!
* \brief
*	Set frame buffer information
*
* \retval  0 if success
*/ 
static int set_frame_info(jdec_drvinfo_t *drv)
{
    jdec_frameinfo_t *frame_info;
    jdec_attr_t      *attr;
    unsigned int      y_base, c_base;

    
    frame_info = &drv->arg_out;
    attr = &drv->attr;
    
    /* Physical address */
    y_base = drv->arg_in.dst_y_addr;
    c_base = drv->arg_in.dst_c_addr;
          
    if((drv->arg_in.dec_to_x == 0) && ((drv->arg_in.dec_to_y == 0))) {
        /* Decode to start address of destination buffer */
        if(y_base != REG32_VAL(REG_JDEC_YBASE) ){
            DBG_ERR("Y addr not match(0x%x, 0x%x\n", y_base, REG32_VAL(REG_JDEC_YBASE));
        }
        if(c_base != REG32_VAL(REG_JDEC_CBASE) ){
            DBG_ERR("C addr not match(0x%x, 0x%x)\n", c_base, REG32_VAL(REG_JDEC_CBASE));
        }
    }
    /*--------------------------------------------------------------------------
        Step 2: Set information for application 
    --------------------------------------------------------------------------*/
    frame_info->fb.img_w   = drv->decoded_w / (1 << drv->scale_ratio);
    frame_info->fb.img_h   = drv->decoded_h / (1 << drv->scale_ratio);
    frame_info->fb.fb_w    = drv->line_width_y/(drv->bpp >> 3);
    frame_info->fb.fb_h    = drv->line_height;
    frame_info->fb.col_fmt = attr->di.decoded_color;
    frame_info->fb.bpp     = drv->bpp;
    frame_info->fb.h_crop  = 0;
    frame_info->fb.v_crop  = 0;
    frame_info->scaled     = 0;

    if(frame_info->fb.col_fmt != VDO_COL_FMT_ARGB) {
        /* YUV color domain */
        frame_info->fb.y_addr    = drv->arg_in.dst_y_addr;
        frame_info->fb.y_size    = drv->arg_in.dst_y_size;
        frame_info->fb.c_addr    = drv->arg_in.dst_c_addr;
        frame_info->fb.c_size    = drv->arg_in.dst_c_size;
        frame_info->y_addr_user  = drv->arg_in.dst_y_addr_user;
        frame_info->c_addr_user  = drv->arg_in.dst_c_addr_user;
    }
    else {
        /* RGB color domain */
        frame_info->fb.y_addr    = drv->arg_in.dst_y_addr;
        frame_info->fb.y_size    = drv->arg_in.dst_y_size;
        frame_info->fb.c_addr    = 0;
        frame_info->fb.c_size    = 0;
        frame_info->y_addr_user  = drv->arg_in.dst_y_addr_user;
        frame_info->c_addr_user  = 0;
    }
    if((attr->hdr.src_color == VDO_COL_FMT_GRAY) && 
       (attr->di.decoded_color != VDO_COL_FMT_ARGB)) {
        /* Draw C plane for gray level JPEG under YUV color space */
        void *vir_addr;
	    
    	vir_addr = (void *)phys_to_virt(frame_info->fb.c_addr);
        memset(vir_addr, 0x80, frame_info->fb.c_size);
    }    
    
    if((attr->di.scale_factor == SCALE_HALF) || (attr->di.scale_factor == SCALE_QUARTER)
        || (attr->di.scale_factor == SCALE_EIGHTH)){
        frame_info->scaled = attr->di.scale_factor;
    }
    return 0;
} /* End of set_frame_info()*/

/*!*************************************************************************
* wmt_jdec_decode_finish
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	Wait JPEG hardware finished
*
* \retval  0 if success
*/ 
int wmt_jdec_decode_finish(jdec_drvinfo_t *drv)
{
    int   ret;
    
   /*==========================================================================
        In this function, we just wait for HW decoding finished.
    ==========================================================================*/
    /*--------------------------------------------------------------------------
        Step 1: Wait intrrupt (JDEC_INTRQ)
    --------------------------------------------------------------------------*/
    ret =  wait_decode_finish(drv);

    /*--------------------------------------------------------------------------
        Step 2: Set information for application 
    --------------------------------------------------------------------------*/
    set_frame_info(drv);

    return ret;
} /* End of wmt_jdec_decode_finish() */

/*!*************************************************************************
* wmt_jdec_decode_flush
* 
* API Function by Willy Chuang, 2009/12/03
*/
/*!
* \brief
*	Cancel JPEG decoding process
*
* \retval  0 if success
*/ 
int wmt_jdec_decode_flush(jdec_drvinfo_t *drv)
{
    /*--------------------------------------------------------------------------
        Step 1: Stop BSDMA
    --------------------------------------------------------------------------*/
    stop_bsdam();
        
    /* Disable clock to save power consumption */
    jdec_clock_en(0);

    /*--------------------------------------------------------------------------
        Step 2: Set information for application 
    --------------------------------------------------------------------------*/
    set_frame_info(drv);

    return 0;
} /* End of wmt_jdec_decode_flush() */

/*!*************************************************************************
* wmt_jdec_isr
* 
* API Function by Willy Chuang, 2008/12/3
*/
/*!
* \brief
*	Open JPEG hardware
*
* \retval  0 if not handle INT, others INT has been handled.
*/ 
#ifdef __KERNEL__
int wmt_jdec_isr(int irq, void *dev_id, struct pt_regs *regs)
{
#ifdef JDEC_IRQ_MODE
    jdec_drvinfo_t *drv;
    unsigned int    reg_val, irq_en;
    unsigned int    bsdma_int, error_int, picend_int;
    unsigned int    wake_up;
    unsigned long   flags =0;

    TRACE("Enter (dev_id: %p)\n", dev_id);
    irq_en  = REG32_VAL(REG_JDEC_INT_ENALBE);
    reg_val = REG32_VAL(REG_JDEC_INT_STATUS);
    DBG_MSG("irq_en: 0x%x, reg_val: 0x%0x\n", irq_en, reg_val);

    error_int  = reg_val & 0x10;
    bsdma_int  = reg_val & 0x08;
    picend_int = reg_val & 0x01;

    if (!bsdma_int && !error_int && !picend_int) {  // other interrupt
        return IRQ_NONE;
    }
    drv = wmt_jdec_get_drv();
    
    if( drv->ref_num != jdec_ref_cur ) {
        DBG_MSG("drv->ref_num(%d) != jdec_ref_cur(%d)\n", drv->ref_num, jdec_ref_cur);
        TRACE("Leave (jump)\n");
        return IRQ_NONE;
    }
    spin_lock_irqsave(&drv->_lock, flags);

    wake_up = 0;
    if(error_int) {
        DBG_ERR("Some error happened(reg_val: 0x%x)!\n", reg_val);

        drv->_status = STA_ERR_UNKNOWN;
        
        /* Disable all interrupt */
        irq_en &= 0x0f;
        JDEC_REG_SET32(REG_JDEC_INT_ENALBE, irq_en); 
        JDEC_REG_SET32(REG_JDEC_INT_STATUS, 0x10);  /* Clear INT staus (write 1 clear) */
 
        wake_up = 1;
    }
    else {
        if(picend_int){  /* Full impage are decoded and write to DRAM */
            DBG_MSG("HW Decode Finish\n");
        
            drv->_status |= STA_DECODE_DONE;

            /* Disable all interrupt */
            irq_en &= 0x1e;
            JDEC_REG_SET32(REG_JDEC_INT_ENALBE, irq_en);
            JDEC_REG_SET32(REG_JDEC_INT_STATUS, 0x01);  /* Clear INT staus (write 1 clear) */

            wake_up = 1;
        }
        if(bsdma_int) { /* All data are moved by DMA */
            drv->_status &= (~STA_DMA_START);
            drv->_status |= STA_DMA_MOVE_DONE;
        
            DBG_MSG("JBSDMA Move Done (_status: 0x%x)\n", drv->_status);
            DBG_MSG("read_byte(%d)\n", REG32_VAL(REG_JDEC_BSDMA_READ_WCNT)*2);

            /* Disable all interrupt */
            irq_en &= 0x17;
            JDEC_REG_SET32(REG_JDEC_INT_ENALBE, irq_en); 
            JDEC_REG_SET32(REG_JDEC_INT_STATUS, 0x08);  /* Clear INT staus (write 1 clear) */

            wake_up = 1;
        }
    }
    if(picend_int){  /* Full impage are decoded and write to DRAM */
        /* Clear BSDMA START */
        JDEC_REG_SET32(REG_JDEC_BSDMA_START_TX, 0);

        /* Disable clock to save power consumption */
        jdec_clock_en(0);
    }
    if(wake_up) {
       wake_up_interruptible(&jdec_wait);
    }
    spin_unlock_irqrestore(&drv->_lock, flags);
    
#endif /* #ifdef JDEC_IRQ_MODE */
    TRACE("Leave(dev_id: %p)\n", dev_id);
    
    return IRQ_HANDLED;
} /* End of wmt_jdec_isr() */
#endif /* #ifdef __KERNEL__ */

/*!*************************************************************************
* wmt_jdec_suspend
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	Open JPEG hardware
*
* \retval  0 if success
*/ 
int wmt_jdec_suspend(void)
{
    // TO DO
    return 0;
} /* End of wmt_jdec_suspend() */

/*!*************************************************************************
* wmt_jdec_resume
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	Open JPEG hardware
*
* \retval  0 if success
*/ 
int wmt_jdec_resume(void)
{
    // TO DO
    return 0;
} /* End of wmt_jdec_resume() */

/*!*************************************************************************
* wmt_jdec_get_info
* 
* API Function by Willy Chuang, 2009/11/24
*/
/*!
* \brief
*	Open JPEG hardware
*
* \retval  0 if success
*/ 
int wmt_jdec_get_info(char *buf, char **start, off_t offset, int len)
{
    jdec_drvinfo_t *drv;
    char *p = buf;

    p += sprintf(p, "WMT JDEC Start\n");

    drv = wmt_jdec_get_drv();
    if( drv == 0 ) {
        p += sprintf(p, "** JDEC driver was never opened **\n");
        goto EXIT_wmt_jdec_get_info;
    }
    dbg_dump_registers(drv);

    p += sprintf(p, "**** [IN] jdec_attr_t ****\n");
    p += sprintf(p, "profile:    0x%08x\n", (drv->attr.hdr.profile & 0xFF));
    if( drv->attr.hdr.profile & DECFLAG_MJPEG_BIT) {
        p += sprintf(p, "-- MJPEG is playing\n");
    }
    p += sprintf(p, "sof_w:      %d\n", drv->attr.hdr.sof_w);
    p += sprintf(p, "sof_h:      %d\n", drv->attr.hdr.sof_h);
    p += sprintf(p, "filesize:   %d\n", drv->attr.hdr.filesize);
    p += sprintf(p, "src_color:  %d\n", drv->attr.hdr.src_color);
    p += sprintf(p, "avi1.type:  %d\n", drv->attr.hdr.avi1.type);
    p += sprintf(p, "field_size: %d\n", drv->attr.hdr.avi1.field_size);

    p += sprintf(p, "**** [IN] jdec_input_t ****\n");
    p += sprintf(p, "src_addr:   0x%08x\n", drv->arg_in.src_addr);
    p += sprintf(p, "src_size:   %d\n", drv->arg_in.src_size);
    p += sprintf(p, "flags:      0x%08x\n", drv->arg_in.flags);
    p += sprintf(p, "dst_y_addr: 0x%08x\n", drv->arg_in.dst_y_addr);
    p += sprintf(p, "dst_c_addr: 0x%08x\n", drv->arg_in.dst_c_addr);
    p += sprintf(p, "dst_y_size: %d\n", drv->arg_in.dst_y_size);
    p += sprintf(p, "dst_c_size: %d\n", drv->arg_in.dst_c_size);
    p += sprintf(p, "linesize:   %d\n", drv->arg_in.linesize);

    p += sprintf(p, "dst_y_addr_user: 0x%08x\n", drv->arg_in.dst_y_addr_user);
    p += sprintf(p, "dst_c_addr_user: 0x%08x\n", drv->arg_in.dst_c_addr_user);
    p += sprintf(p, "dec_to_x:   %d\n", drv->arg_in.dec_to_x);
    p += sprintf(p, "dec_to_y:   %d\n", drv->arg_in.dec_to_y);

    p += sprintf(p, "**** [OUT] jdec_frameinfo_t ****\n");
    p += sprintf(p, "y_addr:  0x%08x\n", drv->arg_out.fb.y_addr);
    p += sprintf(p, "c_addr:  0x%08x\n", drv->arg_out.fb.c_addr);
    p += sprintf(p, "y_size:  %d\n", drv->arg_out.fb.y_size);
    p += sprintf(p, "c_size:  %d\n", drv->arg_out.fb.c_size);
    p += sprintf(p, "img_w:   %d\n", drv->arg_out.fb.img_w);
    p += sprintf(p, "img_h:   %d\n", drv->arg_out.fb.img_h);
    p += sprintf(p, "fb_w:    %d\n", drv->arg_out.fb.fb_w);
    p += sprintf(p, "fb_h:    %d\n", drv->arg_out.fb.fb_h);

    p += sprintf(p, "bpp:     %d\n", drv->arg_out.fb.bpp);
    p += sprintf(p, "col_fmt: %d\n", drv->arg_out.fb.col_fmt);
    p += sprintf(p, "h_crop:  %d\n", drv->arg_out.fb.h_crop);
    p += sprintf(p, "v_crop:  %d\n", drv->arg_out.fb.v_crop);
    p += sprintf(p, "flag:    0x%08x\n", drv->arg_out.fb.flag);

    p += sprintf(p, "y_addr_user:  0x%08x\n", drv->arg_out.y_addr_user);
    p += sprintf(p, "c_addr_user:  0x%08x\n", drv->arg_out.c_addr_user);
    p += sprintf(p, "scaled:       %d\n", drv->arg_out.scaled);

    p += sprintf(p, "**** Variables ****\n");
    p += sprintf(p, "decoded_w:    %d\n", drv->decoded_w);
    p += sprintf(p, "decoded_h:    %d\n", drv->decoded_h);
    p += sprintf(p, "mcu_width:    %d\n", drv->src_mcu_width);
    p += sprintf(p, "mcu_height:   %d\n", drv->src_mcu_height);
    p += sprintf(p, "scale_ratio:  %d\n", drv->scale_ratio);

    p += sprintf(p, "prd_virt:     0x%08x\n", drv->prd_virt);
    p += sprintf(p, "prd_addr:     0x%08x\n", drv->prd_addr);
    p += sprintf(p, "line_width_y: %d\n", drv->line_width_y);
    p += sprintf(p, "line_width_c: %d\n", drv->line_width_c);
    p += sprintf(p, "bpp:          %d\n", drv->bpp);

    p += sprintf(p, "req_y_size:   %d\n", drv->req_y_size);
    p += sprintf(p, "req_c_size:   %d\n", drv->req_c_size);

    /* Following member is used for multi-tasks */
    p += sprintf(p, "ref_num:      %d\n", drv->ref_num);

    /* Following members are used for hw-jpeg.c only */
    p += sprintf(p, "timeout(ms):  %d\n", drv->_timeout);
    p += sprintf(p, "status:       0x%08x\n", drv->_status);
    p += sprintf(p, "multi_seg_en: %d\n", drv->_multi_seg_en);
    p += sprintf(p, "segment no:   %d\n", drv->_segment);
    p += sprintf(p, "remain_byte:  %d\n", drv->_remain_byte);
    p += sprintf(p, "_is_mjpeg:    %d\n", drv->_is_mjpeg);
    p += sprintf(p, "_prd_offset:  %d\n", drv->_prd_offset);
EXIT_wmt_jdec_get_info:
    p += sprintf(p, "WMT JDEC End\n\n");

    return (p - buf);
} /* End of wmt_jdec_get_info() */

/*!*************************************************************************
* wmt_jdec_open
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	Open JPEG hardware
*
* \retval  0 if success
*/ 
int wmt_jdec_open(jdec_drvinfo_t *drv)
{
    drv->_prd_offset = 8;

    return 0;
} /* End of wmt_jdec_open() */

/*!*************************************************************************
* wmt_jdec_close
* 
* API Function by Willy Chuang, 2008/11/17
*/
/*!
* \brief
*	Close JPEG hardware
*
* \retval  0 if success
*/ 
int wmt_jdec_close(jdec_drvinfo_t *drv)
{
    return 0;
} /* End of wmt_jdec_close() */

/*!*************************************************************************
* wmt_jdec_probe
* 
* API Function by Willy Chuang, 2009/5/22
*/
/*!
* \brief
*	Probe function
*
* \retval  0 if success
*/ 
int wmt_jdec_probe(void)
{
    int ret = 0;

    TRACE("Enter\n");

#ifdef __KERNEL__
  #ifdef JDEC_IRQ_MODE   
    ret = request_irq(WMT_JDEC_IRQ, wmt_jdec_isr, IRQF_SHARED, "jdec", (void *)WMT_JDEC_BASE);
    if (ret) {
        DBG_ERR("Can't get irq %i, err %d\n", WMT_JDEC_IRQ, ret);
        ret = -EBUSY;
    }
  #else
    jpeg_isr->id      = VD_JPEG;
    jpeg_isr->irq_num = 0;
    jpeg_isr->isr     = 0;
  #endif 
#else    
  #ifdef JDEC_IRQ_MODE
    set_irq_handlers(WMT_JDEC_IRQ, wmt_jdec_isr);
    set_int_route(WMT_JDEC_IRQ, 0);
    unmask_interrupt(WMT_JDEC_IRQ);
  #endif
#endif

#ifdef SW_PATCH_FOR_ZERO_BITS_AHEAD_EOI
  #ifdef __KERNEL__
    tmp_prd_buf = (void *) dma_alloc_coherent(NULL, PRD_PATCH_BUF_SIZE, 
                                                &last_prd_phy_addr, GFP_KERNEL);
    if(!tmp_prd_buf){ 
        PRINT("*E* dma allocate failure!\n");
        return -1;
    }
  #endif
//    dma_free_coherent(NULL, PRD_PATCH_BUF_SIZE, tmp_prd_buf, last_prd_phy_addr);
#endif
    
    msd_prd_virt = (char *)(((((int)&msd_prd_buf) + 31)>>5)<<5); // 32 algiment
    
    TRACE("Leave\n");

    return ret;
} /* End of wmt_jdec_probe() */

/*--------------------End of Function Body -----------------------------------*/

#undef WM8510_JDEC_C
