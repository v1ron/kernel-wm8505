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
#define WMT_VID_C

/*--- vid.c ---------------------------------------------------------------
*
* MODULE       : vid.c
* AUTHOR       : Max Chen
* DATE         : 2009/2/24
* DESCRIPTION  : 
*-----------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Max Chen, 2009/2/24
*	First version
*
*------------------------------------------------------------------------------*/

#include <linux/i2c.h>      /* for I2C_M_RD */

#include "wmt-vid.h"


//#define VID_REG_TRACE
#ifdef VID_REG_TRACE
#define VID_REG_SET32(addr, val)  \
        PRINT("REG_SET:0x%x -> 0x%0x\n", addr, val);\
        REG32_VAL(addr) = (val)
#else
#define VID_REG_SET32(addr, val)      REG32_VAL(addr) = (val)
#endif

//#define VID_DEBUG    /* Flag to enable debug message */
//#define VID_DEBUG_DETAIL
//#define VID_TRACE

#ifdef VID_DEBUG
#define DBG_MSG(fmt, args...)      PRINT("{%s} " fmt, __FUNCTION__ , ## args)
#else
#define DBG_MSG(fmt, args...)
#endif

#ifdef VID_DEBUG_DETAIL
#define DBG_DETAIL(fmt, args...)   PRINT("{%s} " fmt, __FUNCTION__ , ## args)
#else
#define DBG_DETAIL(fmt, args...)
#endif

#ifdef VID_TRACE
  #define TRACE(fmt, args...)      PRINT("{%s}:  " fmt, __FUNCTION__ , ## args)
#else
  #define TRACE(fmt, args...) 
#endif

#define DBG_ERR(fmt, args...)      PRINT("*E* {%s} " fmt, __FUNCTION__ , ## args)


#define VID_INT_MODE        

#define ALIGN64(a)          ((((a)+63)>>6)<<6)

extern void wmt_i2c_xfer_continue_if(struct i2c_msg *msg, unsigned int num);
extern void wmt_i2c_xfer_if(struct i2c_msg *msg);

static vid_fb_t *_cur_fb;
static vid_fb_t *_prev_fb;

static unsigned int  cur_y_addr;   // temp
static unsigned int  cur_c_addr;   // temp

static spinlock_t   vid_lock;


/*-------------------------------Body Functions------------------------------------*/
#if 0
static void vid_dumpreg(char *buf,int len)
{
    int index=0;
    char *vir_buf=buf;

    DBG_MSG("dump addr 0x %08X length %d\n",vir_buf, len);
    for (index=0;index<len;index+=4) {
        DBG_MSG("%08X %08X \n",vir_buf+index,*(int *)(vir_buf+index));
    }
} /* End of vid_dumpreg() */
#endif

/*!*************************************************************************
* vid_gpio_init
* 
* Private Function by Max Chen, 2009/1/25
*/
/*!
* \Init gpio setting
*
* \retval none
*/
static void vid_gpio_init(vid_mode mode)
{
    volatile unsigned int tmpReg =0;
    volatile unsigned int *tmpRegPtr = (volatile unsigned int *)0xD8110044;
    volatile unsigned int *tmpRegGpio100 =(volatile unsigned int *)0xD8110100;

    volatile unsigned int *tmpRegGpio200 =(volatile unsigned int *)0xD8110200;

	volatile unsigned int *tmpRegPmc =(volatile unsigned int *)0xD8130254;

	//*tmpRegPmc = (*tmpRegPmc) | (BIT30|BIT14);
	//sleep(100);
	
    tmpReg = *tmpRegPtr;
    tmpReg = tmpReg & 0xffffff00;//set bit0-7 to 0x0 
    *tmpRegPtr  = tmpReg;	/*not set SDTV output to gpio*/ 
    DBG_MSG("VID   addr 0x%08x data 0x%08x\n",tmpRegPtr ,*tmpRegPtr);

    if( mode == VID_MODE_CMOS) {
        DBG_MSG("GPIO set to cmos mode\n");
        tmpReg=*tmpRegGpio200;
        tmpReg &= ~(BIT23);  //set bit23 to 0
        tmpReg |= (BIT3);  //set bit3 to 1
        *tmpRegGpio200=tmpReg;

        tmpReg=*tmpRegGpio100;
        tmpReg |=   (BIT21|BIT22|BIT23);   //set bit(21-23) = 0x1
        *tmpRegGpio100=tmpReg;
    }
    else {
        DBG_MSG("GPIO set to vid mode\n");
        tmpReg=*tmpRegGpio200;
        tmpReg &= 0xff7fffff;//set bit23 to 0
        *tmpRegGpio200=tmpReg;
    }    
    DBG_MSG("VID  gpio share pin addr 0x%08x data 0x%08x\n",tmpRegGpio200 ,*tmpRegGpio200);
    
    return;
} /* End of vid_gpio_init()*/

/*!*************************************************************************
* wmt_vid_i2c_write_page
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID mode
* \retval  0 if success
*/
int wmt_vid_i2c_write_page(int chipId ,unsigned int index,char *pdata,int len)
{
    struct i2c_msg msg[1];
    unsigned char buf[len+1];
    
    buf[0] = index;
    memcpy(&buf[1],pdata,len);
    msg[0].addr = chipId;
    msg[0].flags = 0 ;
    msg[0].flags &= ~(I2C_M_RD);
    msg[0].len = len;
    msg[0].buf = buf;
    wmt_i2c_xfer_if(msg);
    return 0;
} /* End of wmt_vid_i2c_write_page() */

/*!*************************************************************************
* wmt_vid_i2c_read_page
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID mode
* \retval  0 if success
*/ 
int wmt_vid_i2c_read_page(int chipId ,unsigned int index,char *pdata,int len) 
{
    struct i2c_msg msg[2];
    unsigned char buf[len+1];
    
    memset(buf,0x55,len+1);
    buf[0] = index;

    msg[0].addr = chipId;
    msg[0].flags = 0 ;
    msg[0].flags &= ~(I2C_M_RD);
    msg[0].len = 1;
    msg[0].buf = buf;

    msg[1].addr = chipId;
    msg[1].flags = 0 ;
    msg[1].flags |= (I2C_M_RD);
    msg[1].len = len;
    msg[1].buf = buf;

    wmt_i2c_xfer_continue_if(msg, 2);
    memcpy(pdata,buf,len);

    return 0;
} /* End of wmt_vid_i2c_read_page() */

/*!*************************************************************************
* wmt_vid_i2c_read
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID mode
* \retval  0 if success
*/ 
int wmt_vid_i2c_read(int chipId ,unsigned int index) 
{
    char retval;
    wmt_vid_i2c_read_page( chipId ,index,&retval,1) ;    
    return retval;
} /* End of wmt_vid_i2c_read() */

/*!*************************************************************************
* wmt_vid_i2c_write
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID mode
* \retval  0 if success
*/ 
int wmt_vid_i2c_write(int chipId ,unsigned int index,char data)
{
    wmt_vid_i2c_write_page(chipId ,index,&data,2);
    
    return 0;
} /* End of wmt_vid_i2c_write() */

/*!*************************************************************************
* wmt_vid_set_mode
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID mode
* \retval  0 if success
*/ 
int wmt_vid_set_mode(int width, int height)
{
    int reg_width;
    
    TRACE("Enter\n");
    
    reg_width = ((ALIGN64(width)>>3)<<8) | (width >> 3);
    VID_REG_SET32( REG_VID_WIDTH,  reg_width ); /* VID output width */
    VID_REG_SET32( REG_VID_HEIGHT, height );    /* VID output height */
        
    TRACE("Leave\n");

    return 0;
} /* End of wmt_vid_set_mode() */

/*!*************************************************************************
* wmt_vid_set_cur_fb
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID source address for Y and C
* \retval  0 if success
*/ 
int wmt_vid_set_cur_fb(vid_fb_t *fb)
{
    unsigned long flags =0;

    spin_lock_irqsave(&vid_lock, flags);

    _prev_fb = _cur_fb;
    _cur_fb  = fb;

    fb->is_busy = 1;
    fb->done    = 0;
    wmt_vid_set_addr(fb->y_addr, fb->c_addr);		    

    //DBG_MSG("[%d] y_addr: 0x%x, c_addr: 0x%x\n", fb->id, fb->y_addr, fb->c_addr);
    //DBG_MSG("[%d] done: %d, is_busy: %d\n", fb->id, fb->done, fb->is_busy);

    spin_unlock_irqrestore(&vid_lock, flags);

    return 0;
} /* End of wmt_vid_set_cur_fb() */

/*!*************************************************************************
* wmt_vid_get_cur_fb
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID source address for Y and C
* \retval  0 if success
*/ 
vid_fb_t * wmt_vid_get_cur_fb(void)
{
    unsigned long flags =0;
    vid_fb_t *fb;    
    
    spin_lock_irqsave(&vid_lock, flags);
    fb = _cur_fb;
    //DBG_MSG("[%d] y_addr: 0x%x, c_addr: 0x%x\n", fb->id, fb->y_addr, fb->c_addr);
    //DBG_MSG("[%d] done: %d, is_busy: %d\n", fb->id, fb->done, fb->is_busy);
    spin_unlock_irqrestore(&vid_lock, flags);

    return fb;
} /* End of wmt_vid_get_cur_fb() */

/*!*************************************************************************
* wmt_vid_set_addr
* 
* Public Function by Willy Chuang, 2010/06/01
*/
/*!
* \brief
*       set VID source address for Y and C
* \retval  0 if success
*/ 
int wmt_vid_set_addr(unsigned int y_addr, unsigned int c_addr)
{
    TRACE("Enter\n");

    cur_y_addr = REG32_VAL(REG_VID_Y0_SA);
    cur_c_addr = REG32_VAL(REG_VID_C0_SA);

    if( (y_addr != cur_y_addr) || (c_addr != cur_c_addr)) {
        VID_REG_SET32( REG_VID_Y0_SA, y_addr ); /* VID Y FB address */
        VID_REG_SET32( REG_VID_C0_SA, c_addr ); /* VID C FB address */
    }
    TRACE("Leave\n");

    return 0;
} /* End of wmt_vid_set_addr() */

/*!*************************************************************************
* wmt_vid_open
* 
* Public Function by Willy Chuang, 2010/05/28
*/
/*!
* \brief
*       init CMOS module
* \retval  0 if success
*/ 
int wmt_vid_open(vid_mode mode)
{
    int value, int_ctrl;
    
    TRACE("Enter\n");

    /*--------------------------------------------------------------------------
        Step 1: Init GPIO for CMOS or TVDEC mode
    --------------------------------------------------------------------------*/
    vid_gpio_init(mode);

    /*--------------------------------------------------------------------------
        Step 2: Init CMOS or TVDEC module
    --------------------------------------------------------------------------*/
    value = REG32_VAL(REG_VID_TVDEC_CTRL);

    int_ctrl = 0x00;
    if(mode == VID_MODE_CMOS) {
        VID_REG_SET32( REG_VID_TVDEC_CTRL, (value & 0xFFFFFFE)); /* disable TV decoder */
        VID_REG_SET32( REG_VID_CMOS_PIXEL_SWAP, 0x2);    /* 0x2 for YUYV */
#ifdef VID_INT_MODE
        int_ctrl = 0x0808;
#endif
        VID_REG_SET32( REG_VID_INT_CTRL, int_ctrl );
//        VID_REG_SET32( REG_VID_CMOS_EN, 0x1);	/* enable CMOS */
    }
    else {
        VID_REG_SET32( REG_VID_TVDEC_CTRL, (value | 0x1) );	/* enable TV decoder */
#ifdef VID_INT_MODE
        int_ctrl = 0x0404;
#endif
        VID_REG_SET32( REG_VID_INT_CTRL, int_ctrl );
        VID_REG_SET32( REG_VID_CMOS_EN, 0x0);	/* disable CMOS */
    }
    VID_REG_SET32( REG_VID_VBUF_SEL, 0x0);
    
    cur_y_addr = 0;
    cur_c_addr = 0;
    
    _cur_fb  = 0;
    _prev_fb = 0;

    spin_lock_init(&vid_lock);

    TRACE("Leave\n");

    return 0;
} /* End of wmt_vid_open() */

/*!*************************************************************************
* wmt_vid_close
* 
* Public Function by Willy Chuang, 2010/05/28
*/
/*!
* \brief
*       release CMOS module
* \retval  0 if success
*/ 
int wmt_vid_close(vid_mode mode)
{
    TRACE("Enter\n");

    if(mode == VID_MODE_CMOS) {
        VID_REG_SET32( REG_VID_CMOS_EN, 0x0);	/* disable CMOS */
    }
    else {
        int value = REG32_VAL(REG_VID_TVDEC_CTRL);
        VID_REG_SET32( REG_VID_TVDEC_CTRL, (value & 0xFFFFFFE)); /* disable TV decoder */
    }
    TRACE("Leave\n");

    return 0;
} /* End of wmt_vid_close() */

/*--------------------End of Function Body -----------------------------------*/
#undef DBG_MSG
#undef DBG_DETAIL
#undef TRACE
#undef DBG_ERR

#undef WMT_VID_C
