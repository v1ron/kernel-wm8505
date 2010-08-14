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
#define GOVRH_C

#include "govrh.h"

#ifdef WMT_FTBLK_GOVRH
#ifdef __KERNEL__
//#include <asm/uaccess.h>
//#include <asm/io.h>
#define DPRINT printk
#else
//#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define DPRINT printf
#endif

#ifdef GOVRH_DEBUG
#define GOVRHMSG(fmt, args...) DPRINT("[GOVRH] %s: " fmt, __FUNCTION__ , ## args)
#else
#define GOVRHMSG(fmt, args...) do {} while(0)
#endif

void govrh_reg_dump(void)
{
	int i;
	U32 addr,reg;

	DPRINT("========== GOVRH register dump ==========\n");
	for(i=0;i<0xe8;i+=16){
		DPRINT("0x%08x : 0x%08x 0x%08x 0x%08x 0x%08x\n",GOVRH_BASE2_ADDR+i,vppif_reg32_in(GOVRH_BASE2_ADDR+i),
			vppif_reg32_in(GOVRH_BASE2_ADDR+i+4),vppif_reg32_in(GOVRH_BASE2_ADDR+i+8),vppif_reg32_in(GOVRH_BASE2_ADDR+i+12));
	}

	for(i=0x30;i<0xe8;i+=16){
		DPRINT("0x%08x : 0x%08x 0x%08x 0x%08x 0x%08x\n",GOVRH_BASE1_ADDR+i,vppif_reg32_in(GOVRH_BASE1_ADDR+i),
			vppif_reg32_in(GOVRH_BASE1_ADDR+i+4),vppif_reg32_in(GOVRH_BASE1_ADDR+i+8),vppif_reg32_in(GOVRH_BASE1_ADDR+i+12));
	}

	addr = GOVRH_BASE2_ADDR+0x0;
	reg = vppif_reg32_in(addr);
	DPRINT("---------- GOVRH TG1 ----------\n");
	DPRINT("TG enable %d, Twin mode %d\n",reg & BIT0,reg & BIT8);
	DPRINT("DVO clk %d,Read cyc %d\n",vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO),vppif_reg32_read(GOVRH_READ_CYC));
	DPRINT("H total %d, Sync %d, beg %d, end %d\n",vppif_reg32_read(GOVRH_H_ALLPXL),vppif_reg32_read(GOVRH_HDMI_HSYNW),
		vppif_reg32_read(GOVRH_ACTPX_BG),vppif_reg32_read(GOVRH_ACTPX_END));
	DPRINT("V total %d, Sync %d, beg %d, end %d\n",vppif_reg32_read(GOVRH_V_ALLLN),vppif_reg32_read(GOVRH_HDMI_VBISW),
		vppif_reg32_read(GOVRH_ACTLN_BG),vppif_reg32_read(GOVRH_ACTLN_END));
	DPRINT("VBIE %d,PVBI %d\n",vppif_reg32_read(GOVRH_VBIE_LINE),vppif_reg32_read(GOVRH_PVBI_LINE));
	DPRINT("---------- GOVRH TG2 ----------\n");
	DPRINT("H total %d, Sync %d, beg %d, end %d\n",vppif_reg32_read(GOVRH_H_ALLPXL2),vppif_reg32_read(GOVRH_HDMI_HSYNW2),
		vppif_reg32_read(GOVRH_ACTPX_BG2),vppif_reg32_read(GOVRH_ACTPX_END2));
	DPRINT("V total %d, Sync %d, beg %d, end %d\n",vppif_reg32_read(GOVRH_V_ALLLN2),vppif_reg32_read(GOVRH_HDMI_VBISW2),
		vppif_reg32_read(GOVRH_ACTLN_BG2),vppif_reg32_read(GOVRH_ACTLN_END2));
	DPRINT("VBIE %d,PVBI %d\n",vppif_reg32_read(GOVRH_VBIE_LINE2),vppif_reg32_read(GOVRH_PVBI_LINE2));
	DPRINT("-------------------------------\n");
	DPRINT("DVO enable %d, polarity %s,data width %d bits\n",vppif_reg32_read(GOVRH_DVO_ENABLE),
		(vppif_reg32_read(GOVRH_DVO_SYNC_POLAR))? "Low":"High",(vppif_reg32_read(GOVRH_DVO_OUTWIDTH))? 12:24);
	switch(govrh_get_dvo_color_format()){
		case VDO_COL_FMT_ARGB:
			DPRINT("DVO color format RGB32\n");
			break;
		case VDO_COL_FMT_YUV420:
			DPRINT("DVO color format YUV420\n");
			break;
		case VDO_COL_FMT_YUV422H:
			DPRINT("DVO color format YUV422\n");
			break;
		case VDO_COL_FMT_YUV444:
			DPRINT("DVO color format YUV444\n");
			break;
		default:
			break;
	}
	DPRINT("color bar enable %d,mode %d,inv %d\n",vppif_reg32_read(GOVRH_CB_ENABLE),vppif_reg32_read(GOVRH_CB_MODE),
		vppif_reg32_read(GOVRH_CB_INVERSION));
	DPRINT("VGA HSync %d,VSync %d\n",vppif_reg32_read(GOVRH_VGA_HSYNW),vppif_reg32_read(GOVRH_VGA_VSYNW));
	DPRINT("VGA H polar %s,V polar %s\n",(vppif_reg32_read(GOVRH_VGA_HSYN_POLAR))? "Low":"High",
		(vppif_reg32_read(GOVRH_VGA_VSYN_POLAR))? "Low":"High");

	DPRINT("Contrast 0x%x,Brightness 0x%x\n",vppif_reg32_in(REG_GOVRH_CONTRAST),vppif_reg32_in(REG_GOVRH_BRIGHTNESS));
	DPRINT("CSC mode %s\n",(vppif_reg32_read(GOVRH_CSC_MOD))?"YUV2RGB":"RGB2YUV");
	DPRINT("VGA CSC %d, DVO CSC %d\n",vppif_reg32_read(GOVRH_DVO_YUV2RGB_ENABLE),vppif_reg32_read(GOVRH_VGA_YUV2RGB_ENABLE));
	DPRINT("H264 %d\n",vppif_reg32_read(GOVRH_H264_INPUT_ENABLE));
	DPRINT("source format %s\n",(vppif_reg32_read(GOVRH_INFMT))?"field":"frame");
	DPRINT("Y addr 0x%x, C addr 0x%x\n",vppif_reg32_in(REG_GOVRH_YSA),vppif_reg32_in(REG_GOVRH_CSA));
	DPRINT("Active width %d,frame buf width %d\n",vppif_reg32_read(GOVRH_AWIDTH),vppif_reg32_read(GOVRH_FWIDTH));
	DPRINT("V crop %d, H crop %d\n",vppif_reg32_read(GOVRH_VCROP),vppif_reg32_read(GOVRH_HCROP));
	switch(govrh_get_color_format()){
		case VDO_COL_FMT_ARGB:
			DPRINT("color format RGB32\n");
			break;
		case VDO_COL_FMT_YUV420:
			DPRINT("color format YUV420\n");
			break;
		case VDO_COL_FMT_YUV422H:
			DPRINT("color format YUV422\n");
			break;
		case VDO_COL_FMT_YUV444:
			DPRINT("color format YUV444\n");
			break;
		default:
			break;
	}
	DPRINT("DAC PwrDn %d\n",vppif_reg32_read(GOVRH_DAC_PWRDN));
}


vpp_flag_t govrh_set_tg_enable(vpp_flag_t enable)
{
	switch (enable) {
	case VPP_FLAG_ENABLE:
	case VPP_FLAG_DISABLE:
		vppif_reg32_write(GOVRH_TG_ENABLE, enable);
		break;
	default:
		return VPP_FLAG_ERROR;
	}
	return VPP_FLAG_SUCCESS;
}

void govrh_set_timing1(vpp_clock_t *timing)
{
	vppif_reg32_write(GOVRH_TG_MODE,0);
	vppif_reg32_write(GOVRH_OUTFMT,0);

	vppif_reg32_write(GOVRH_READ_CYC, timing->read_cycle);

	vppif_reg32_write(GOVRH_ACTPX_BG, timing->begin_pixel_of_active);
	vppif_reg32_write(GOVRH_ACTPX_END, timing->end_pixel_of_active);
	vppif_reg32_write(GOVRH_H_ALLPXL, timing->total_pixel_of_line);

	vppif_reg32_write(GOVRH_ACTLN_BG, timing->begin_line_of_active);
	vppif_reg32_write(GOVRH_ACTLN_END, timing->end_line_of_active);
	vppif_reg32_write(GOVRH_V_ALLLN, timing->total_line_of_frame);

	vppif_reg32_write(GOVRH_VBIE_LINE, timing->line_number_between_VBIS_VBIE);
	vppif_reg32_write(GOVRH_PVBI_LINE, timing->line_number_between_PVBI_VBIS);

	vppif_reg32_write(GOVRH_HDMI_HSYNW,timing->hsync);
	vppif_reg32_write(GOVRH_HDMI_VBISW,timing->vsync + 1);
}

void govrh_set_timing2(vpp_clock_t *timing)
{
	vppif_reg32_write(GOVRH_TG_MODE,1);
	vppif_reg32_write(GOVRH_OUTFMT,1);

	vppif_reg32_write(GOVRH_ACTPX_BG2, timing->begin_pixel_of_active);
	vppif_reg32_write(GOVRH_ACTPX_END2, timing->end_pixel_of_active);
	vppif_reg32_write(GOVRH_H_ALLPXL2, timing->total_pixel_of_line);

	vppif_reg32_write(GOVRH_ACTLN_BG2, timing->begin_line_of_active);
	vppif_reg32_write(GOVRH_ACTLN_END2, timing->end_line_of_active);
	vppif_reg32_write(GOVRH_V_ALLLN2, timing->total_line_of_frame);

	vppif_reg32_write(GOVRH_VBIE_LINE2, timing->line_number_between_VBIS_VBIE);
	vppif_reg32_write(GOVRH_PVBI_LINE2, timing->line_number_between_PVBI_VBIS);

	vppif_reg32_write(GOVRH_HDMI_HSYNW2,timing->hsync);
	vppif_reg32_write(GOVRH_HDMI_VBISW2,timing->vsync + 1);
}

void govrh_get_timing(vpp_clock_t * tmr)
{
	tmr->read_cycle = vppif_reg32_read(GOVRH_READ_CYC);
	tmr->total_pixel_of_line = vppif_reg32_read(GOVRH_H_ALLPXL);
	tmr->begin_pixel_of_active = vppif_reg32_read(GOVRH_ACTPX_BG);
	tmr->end_pixel_of_active = vppif_reg32_read(GOVRH_ACTPX_END);

	tmr->total_line_of_frame = vppif_reg32_read(GOVRH_V_ALLLN);
	tmr->begin_line_of_active = vppif_reg32_read(GOVRH_ACTLN_BG);
	tmr->end_line_of_active = vppif_reg32_read(GOVRH_ACTLN_END);

	tmr->line_number_between_VBIS_VBIE = vppif_reg32_read(GOVRH_VBIE_LINE);
	tmr->line_number_between_PVBI_VBIS = vppif_reg32_read(GOVRH_PVBI_LINE);

	tmr->hsync = vppif_reg32_read(GOVRH_HDMI_HSYNW);
	tmr->vsync = vppif_reg32_read(GOVRH_HDMI_VBISW) - 1;

#ifdef PATCH_GOVRH_ASYNC_FIFO
//	printk("PATCH_GOVRH_ASYNC_FIFO@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	switch( g_vpp.govrh_async_fifo_patch ){
		case 2:
			if( g_vpp.govrh_interlace_mode == 0 ){
				return;
			}
			break;
		case 3:
			if( g_vpp.govrh_interlace_mode == 0 ){
				unsigned int vsync;
//				printk("PATCH_GOVRH_ASYNC_FIFO@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@1\n");
				tmr->end_line_of_active = g_vpp.govrh_async_fifo_reg;
				vsync = vppif_reg32_read(GOVRH_HDMI_VBISW2);
				if( vsync ){
					vsync -= 1;
					tmr->vsync = vsync;
					tmr->begin_line_of_active += vsync;
					tmr->end_line_of_active += vsync;
					tmr->total_line_of_frame += vsync;
				}
				return;
			}
			break;
		default:
			break;
	}
#endif
	if( vppif_reg32_read(GOVRH_TG_MODE) ){
		tmr->total_line_of_frame += vppif_reg32_read(GOVRH_V_ALLLN2);
		tmr->begin_line_of_active += vppif_reg32_read(GOVRH_ACTLN_BG2);
		tmr->end_line_of_active += vppif_reg32_read(GOVRH_ACTLN_END2);
	}
}

vpp_int_err_t govrh_get_int_status(void)
{
	vpp_int_err_t int_sts;

	int_sts = 0;
	if (vppif_reg32_read(GOVRH_INT_MEM)) {
		int_sts |= VPP_INT_ERR_GOVRH_MIF;
	}
	return int_sts;
}

void govrh_clean_int_status(vpp_int_err_t int_sts)
{
	if (int_sts & VPP_INT_ERR_GOVRH_MIF) {
		vppif_reg16_out(REG_GOVRH_INT+0x2,0x2);
	}
}

void govrh_set_dvo_enable(vpp_flag_t enable)
{
	vppif_reg32_write(GOVRH_DVO_ENABLE, enable);
#if(WMT_CUR_PID == WMT_PID_8510)
	vppif_reg32_write(0xd8110200,0x80000000,31,enable);
#endif
#if(WMT_CUR_PID == WMT_PID_8435)
	if( enable ){
		vppif_reg32_out(GPIO_BASE_ADDR+0x44,0x0);	// Disable DVO/CCIR656 GPIO
	}
	else {
		vppif_reg32_out(GPIO_BASE_ADDR+0x44,0xFFFFFFFF);	// Enable GPIO
		vppif_reg32_out(GPIO_BASE_ADDR+0x84,0x0);			// GPIO output enable
		vppif_reg32_out(GPIO_BASE_ADDR+0x484,0xFFFFFFFF);	// GPIO pull enable
		vppif_reg32_out(GPIO_BASE_ADDR+0x4C4,0x0);			// 1: pull up, 0: pull down
	}
#endif
}

void govrh_set_dvo_sync_polar(vpp_flag_t hsync,vpp_flag_t vsync)
{
	vppif_reg32_write(GOVRH_DVO_SYNC_POLAR,hsync);
#ifdef WMT_FTBLK_GOVRH_EXT
	vppif_reg32_write(GOVRH_DVO_VSYNC_POLAR,vsync);
#endif
}

vdo_color_fmt govrh_get_dvo_color_format(void)
{
	if(	vppif_reg32_read(GOVRH_DVO_RGB) ){
		return VDO_COL_FMT_ARGB;
	}
	if( vppif_reg32_read(GOVRH_DVO_YUV422) ){
		return VDO_COL_FMT_YUV422H;
	}
	return VDO_COL_FMT_YUV444;
}

void govrh_set_dvo_color_format(vdo_color_fmt fmt)
{
	switch( fmt ){
		case VDO_COL_FMT_ARGB:
			vppif_reg32_write(GOVRH_DVO_RGB,0x1);
			vppif_reg32_write(GOVRH_DVO_YUV422,0x0);
			break;
		case VDO_COL_FMT_YUV422H:
			vppif_reg32_write(GOVRH_DVO_RGB,0x0);
			vppif_reg32_write(GOVRH_DVO_YUV422,0x1);
			break;
		case VDO_COL_FMT_YUV444:
		default:
			vppif_reg32_write(GOVRH_DVO_RGB,0x0);
			vppif_reg32_write(GOVRH_DVO_YUV422,0x0);
			break;
	}
}

vpp_flag_t govrh_set_dvo_outdatw(vpp_datawidht_t width)
{
	switch (width) {
	case VPP_DATAWIDHT_12:
		vppif_reg32_write(GOVRH_DVO_OUTWIDTH, 1);
		break;
	case VPP_DATAWIDHT_24:
		vppif_reg32_write(GOVRH_DVO_OUTWIDTH, 0);
		break;
	default:
		return VPP_FLAG_ERROR;
	}
	return VPP_FLAG_SUCCESS;
}

void govrh_set_dvo_clock_delay(int inverse,int delay)
{
	vppif_reg32_write(GOVRH_DVO_CLK_INV,inverse);
	vppif_reg32_write(GOVRH_DVO_CLK_DLY,delay);
}

void govrh_set_colorbar(vpp_flag_t enable,int mode,int inv)
{
	vppif_reg32_write(GOVRH_CB_ENABLE, enable);
	vppif_reg32_write(GOVRH_CB_MODE, mode);
	vppif_reg32_write(GOVRH_CB_INVERSION, inv);
}

void govrh_set_vga_enable(vpp_flag_t enable)
{
#if(WMT_CUR_PID == WMT_PID_8510)
	vppif_reg32_write(0xd8110200,0x40000000,30,enable);
#endif
#if(WMT_CUR_PID == WMT_PID_8435)
	if( enable ){
		vppif_reg8_out(GPIO_BASE_ADDR+0x49,0x0);		// Disable VGA GPIO
	}
	else {
		vppif_reg8_out(GPIO_BASE_ADDR+0x49,0xFF);		// Enable GPIO
		vppif_reg8_out(GPIO_BASE_ADDR+0x89,0x0);		// GPIO output enable
		vppif_reg8_out(GPIO_BASE_ADDR+0x489,0xFF);		// GPIO pull enable
		vppif_reg8_out(GPIO_BASE_ADDR+0x4C9,0x0);		// 1: pull up, 0: pull down
	}
#endif
}

void govrh_set_VGA_sync(U32 hsync, U32 vsync)
{
	vppif_reg32_write(GOVRH_VGA_HSYNW, hsync);
	vppif_reg32_write(GOVRH_VGA_VSYNW, vsync);
}

void govrh_set_VGA_sync_polar(U32 hsync, U32 vsync)
{
	vppif_reg32_write(GOVRH_VGA_HSYN_POLAR, hsync);
	vppif_reg32_write(GOVRH_VGA_VSYN_POLAR, vsync);
}

#if 0
int govrh_monitor_DISP_DAC_sense(void)
{
	unsigned int pwrdn;

	vppif_reg32_write(GOVRH_DAC_TEST_A,0x55);
	vppif_reg32_write(GOVRH_DAC_TEST_B,0x0);
	vppif_reg32_write(GOVRH_DAC_TEST_C,0x0);
	vppif_reg32_write(GOVRH_DAC_TEST_ENABLE,0x1);

	pwrdn = (vppif_reg32_in(REG_GOVRH_DAC_CON))?0x00:0x01;
	vppif_reg32_write(GOVRH_DAC_TEST_ENABLE,0x0);
}
#endif

int govrh_monitor_DAC_sense(void)
{
	static unsigned int govrh_vbis_cnt = 0;
//	unsigned int i;
	unsigned int pwrdn;
	static unsigned int pre_pwrdn;

	govrh_vbis_cnt++;
	if( (govrh_vbis_cnt % p_govrh->vga_dac_sense_cnt) != 0 ){
		return 0;
	}

#ifdef PATCH_GOVRH_ASYNC_FIFO
	switch( g_vpp.govrh_async_fifo_patch ){
		case 1:
		case 3:
			vppif_reg32_write(GOVRH_ACTLN_END, g_vpp.govrh_async_fifo_reg);
			break;
		default:
			break;
	}
#endif

#if 1
	if( vppif_reg32_read(GOVRH_DAC_PWRDN) == 1 ){
		vppif_reg32_write(GOVRH_DAC_PWRDN, 0);	/* DAC power on */
		govrh_vbis_cnt--;
		return 0;
	}
#else
	vppif_reg32_write(GOVRH_DAC_PWRDN, 0);	/* DAC power on */
	/* wait status stable */
	for (i = 0; i < vpp_dac_sense_nop; i++) {
		nop();
	}
#endif

	vppif_reg32_write(GOVRH_DAC_MANUAL_SENSE, 0x01);
	vppif_reg32_write(GOVRH_DAC_SCD_ENABLE, 0x00);

	pwrdn = (vppif_reg32_in(REG_GOVRH_DAC_CON))?0x00:0x01;
	vppif_reg32_write(GOVRH_DAC_SCD_ENABLE, 0x01);

	vppif_reg32_write(GOVRH_DAC_MANUAL_SENSE, 0x00);
	vppif_reg32_write(GOVRH_DAC_PWRDN, pwrdn);	/* set DAC power */

//	DPRINT("[GOVRH] DAC sense 0x%x\n",pwrdn);

	if( pre_pwrdn != pwrdn ){
		DPRINT(KERN_ALERT "[GOVRH] VGA DAC %s\n",(pwrdn)?"DISCONNECT":"CONNECT");
		pre_pwrdn = pwrdn;
		govrh_set_vga_enable((pwrdn == 0));
		return (pwrdn==0);
	}
	return 0;
}

void govrh_set_DAC_pwrdn(vpp_flag_t enable)
{
	vppif_reg32_write(GOVRH_DAC_PWRDN, enable);
}

void govrh_set_contrast(int level)
{
	vppif_reg32_write(GOVRH_CONTRAST_YAF, level);
	vppif_reg32_write(GOVRH_CONTRAST_PBAF, level);
	vppif_reg32_write(GOVRH_CONTRAST_PRAF, level);
}

int govrh_get_contrast(void)
{
	return vppif_reg32_read(GOVRH_CONTRAST_YAF);
}

void govrh_set_brightness(int level)
{
	vppif_reg32_write(GOVRH_BRIGHTNESS_Y, level);
}

int govrh_get_brightness(void)
{
	return vppif_reg32_read(GOVRH_BRIGHTNESS_Y);
}

void govrh_set_MIF_enable(vpp_flag_t enable)
{
	vppif_reg32_write(GOVRH_MIF_ENABLE, enable);
}

vpp_flag_t govrh_set_data_format(vdo_color_fmt format)
{
	if (format != VDO_COL_FMT_ARGB) {
		vppif_reg32_write(GOVRH_RGB_MODE, 0);
	}

	switch (format) {
	case VDO_COL_FMT_YUV420:
		vppif_reg32_write(GOVRH_COLFMT, 1);
		vppif_reg32_write(GOVRH_COLFMT2, 0);
		break;
	case VDO_COL_FMT_YUV422H:
		vppif_reg32_write(GOVRH_COLFMT, 0);
		vppif_reg32_write(GOVRH_COLFMT2, 0);
		break;
	case VDO_COL_FMT_YUV444:
		vppif_reg32_write(GOVRH_COLFMT, 0);
		vppif_reg32_write(GOVRH_COLFMT2, 1);
		break;
	case VDO_COL_FMT_ARGB:
		vppif_reg32_write(GOVRH_RGB_MODE, 1);
		break;
	default:
		GOVRHMSG("*E* check the parameter\n");
		return VPP_FLAG_ERROR;
	}
	return VPP_FLAG_SUCCESS;
}

vdo_color_fmt govrh_get_color_format(void)
{
	if( vppif_reg32_read(GOVRH_RGB_MODE) ){
		return VDO_COL_FMT_ARGB;
	}
	if( vppif_reg32_read(GOVRH_COLFMT2) ){
		return VDO_COL_FMT_YUV444;
	}
	if( vppif_reg32_read(GOVRH_COLFMT) ){
		return VDO_COL_FMT_YUV420;
	}
	return VDO_COL_FMT_YUV422H;
}

vpp_flag_t govrh_set_source_format(vpp_display_format_t format)
{
	switch (format) {
	case VPP_DISP_FMT_FRAME:
	case VPP_DISP_FMT_FIELD:
		vppif_reg32_write(GOVRH_INFMT, format);
		break;
	default:
		GOVRHMSG("*E* check the parameter\n");
		return VPP_FLAG_ERROR;
	}
	return VPP_FLAG_SUCCESS;
}

void govrh_set_output_format(vpp_display_format_t field)
{
	vppif_reg32_write(GOVRH_OUTFMT,field);
}

void govrh_set_fb_addr(U32 y_addr,U32 c_addr)
{
//	GOVRHMSG("y_addr:0x%08x, c_addr:0x%08x\n", (U32) y_addr,(U32) c_addr);

	vppif_reg32_out(REG_GOVRH_YSA,y_addr);
	vppif_reg32_out(REG_GOVRH_CSA,c_addr);
}

void govrh_get_fb_addr(U32 *y_addr, U32 *c_addr)
{
	*y_addr = vppif_reg32_in(REG_GOVRH_YSA);
	*c_addr = vppif_reg32_in(REG_GOVRH_CSA);
//	GOVRHMSG("y_addr:0x%08x, c_addr:0x%08x\n",(U32) *y_addr,(U32) *c_addr);
}

void govrh_set_fb_info(U32 width, U32 act_width, U32 x_offset, U32 y_offset)
{
//	GOVRHMSG("set fb info fb_w %d,img_w %d,x %d,y %d\n",width,act_width,x_offset,y_offset);

	vppif_reg32_write(GOVRH_AWIDTH, act_width);
	vppif_reg32_write(GOVRH_FWIDTH, width);
	vppif_reg32_write(GOVRH_VCROP, x_offset);
	vppif_reg32_write(GOVRH_HCROP, y_offset);
}

void govrh_get_fb_info(U32 * width, U32 * act_width, U32 * x_offset, U32 * y_offset)
{
	*act_width = vppif_reg32_read(GOVRH_AWIDTH);
	*width = vppif_reg32_read(GOVRH_FWIDTH);
	*x_offset = vppif_reg32_read(GOVRH_VCROP);
	*y_offset = vppif_reg32_read(GOVRH_HCROP);
}

void govrh_set_fifo_index(U32 index)
{
	vppif_reg32_write(GOVRH_FIFO_IND, index);
}

vpp_flag_t govrh_set_reg_level(vpp_reglevel_t level)
{
	switch (level) {
	case VPP_REG_LEVEL_1:
		vppif_reg32_write(GOVRH_REG_LEVEL, 0x0);
		break;
	case VPP_REG_LEVEL_2:
		vppif_reg32_write(GOVRH_REG_LEVEL, 0x1);
		break;
	default:
		GOVRHMSG("*E* check the parameter\n");
		return VPP_FLAG_ERROR;
	}
	return VPP_FLAG_SUCCESS;
}

void govrh_set_reg_update(vpp_flag_t enable)
{
	vppif_reg32_write(GOVRH_REG_UPDATE, enable);
}

void govrh_set_timing(vpp_clock_t *tmr)
{
	vppif_reg32_write(GOVRH_TG_ENABLE, 0);
	govrh_set_timing1(tmr);
	vppif_reg32_write(GOVRH_TG_ENABLE, 1);
}

void govrh_set_video_mode(unsigned int resx,unsigned int resy,unsigned int pixel_clock,vpp_timing_t *tmr)
{
	int cnt;
	vpp_timing_t *ptr;
	unsigned int line_pixel;
	unsigned int clk_delay = 0;
	vpp_clock_t t1,t2;

	if( tmr ){
//		printk("PATCH_GOVRH_ASYNC_FIFO@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2\n");
		ptr = tmr;
	}
	else {
		ptr = vpp_get_video_mode(resx,resy,pixel_clock);
	}
	vpp_trans_timing(VPP_MOD_GOVRH,ptr,&t1,1);
	pixel_clock = ptr->pixel_clock;
	t1.read_cycle = vpp_set_base_clock(VPP_MOD_GOVRH, pixel_clock);
	GOVRHMSG("pix clk %d,rd cyc %d\n",pixel_clock,t1.read_cycle);

	vppif_reg32_write(GOVRH_TG_ENABLE, 0);
#ifdef PATCH_GOVRH_ASYNC_FIFO
	if( vppif_reg32_read(GOVRH_DVO_ENABLE) && (pixel_clock >= 100000000) ){
//	if( ((vppif_reg32_in(0xd8110200) & BIT30)==0) && (pixel_clock >= 100000000) ){
		g_vpp.govrh_async_fifo_patch = 3;
	}
	else {
		g_vpp.govrh_async_fifo_patch = 2;
	}

	GOVRHMSG("[GOVRH] async fifo patch %d\n",g_vpp.govrh_async_fifo_patch);

	switch( g_vpp.govrh_async_fifo_patch ){
		case 1:
		case 3:
			vppm_set_int_enable(VPP_FLAG_DISABLE,VPP_INT_GOVRH_PVBI | VPP_INT_GOVRH_VBIE);
			break;
		default:
			break;
	}
#endif
	g_vpp.govrh_interlace_mode = 0;
	g_vpp.govrh_field = 0;
	govrh_set_timing1(&t1);
	cnt = t1.total_line_of_frame;
	if( ptr->option & VPP_OPT_INTERLACE ){
		vpp_trans_timing(VPP_MOD_GOVRH,ptr+1,&t2,1);
		govrh_set_timing2(&t2);
		cnt += t2.total_line_of_frame;
		g_vpp.govrh_interlace_mode = 1;
	}
	else {
#ifdef PATCH_GOVRH_ASYNC_FIFO
		switch( g_vpp.govrh_async_fifo_patch ){
			case 2:
				vppif_reg32_write(GOVRH_HDMI_VBISW,1);
				vppif_reg32_write(GOVRH_ACTLN_BG, t1.begin_line_of_active - t1.vsync);
				vppif_reg32_write(GOVRH_ACTLN_END, t1.end_line_of_active - t1.vsync);
				vppif_reg32_write(GOVRH_V_ALLLN, t1.total_line_of_frame - t1.vsync);

				govrh_set_timing2(&t1);
				vppif_reg32_write(GOVRH_V_ALLLN2, t1.vsync);
				vppif_reg32_write(GOVRH_OUTFMT,0);
				break;
			case 3:
				g_vpp.govrh_async_fifo_reg = t1.end_line_of_active - t1.vsync + 1;
				vppif_reg32_write(GOVRH_HDMI_VBISW,2);
				vppif_reg32_write(GOVRH_ACTLN_BG, t1.begin_line_of_active - t1.vsync + 1);
				vppif_reg32_write(GOVRH_ACTLN_END, t1.end_line_of_active - t1.vsync + 1);
				vppif_reg32_write(GOVRH_V_ALLLN, t1.total_line_of_frame - t1.vsync + 1);

				vppm_set_int_enable(VPP_FLAG_ENABLE, VPP_INT_GOVRH_PVBI | VPP_INT_GOVRH_VBIE);
				govrh_set_timing2(&t1);

				vppif_reg32_write(GOVRH_ACTLN_END2, 0);
				vppif_reg32_write(GOVRH_HDMI_VBISW2, t1.vsync );
				vppif_reg32_write(GOVRH_V_ALLLN2, t1.vsync - 1);
				vppif_reg32_write(GOVRH_OUTFMT,0);
				break;
			default:
				break;
		}
#endif
	}

#ifdef PATCH_GOVRH_ASYNC_FIFO
	switch( g_vpp.govrh_async_fifo_patch ){
		case 1:
			g_vpp.govrh_async_fifo_reg = t1.end_line_of_active;
			vppm_set_int_enable(VPP_FLAG_ENABLE, VPP_INT_GOVRH_PVBI | VPP_INT_GOVRH_VBIE);
			g_vpp.govrh_async_fifo_cnt = 60;
			break;
		default:
			break;
	}
#endif

	GOVRHMSG("video mode (%d):(%dx%d@%d)\n",index,ptr->hpixel,line_pixel,ptr->pixel_clock/(t1.total_pixel_of_line * cnt));

	line_pixel = t1.total_pixel_of_line;
	govrh_set_VGA_sync(ptr->hsync, ptr->vsync * line_pixel);
	govrh_set_VGA_sync_polar((ptr->option & VPP_VGA_HSYNC_POLAR_HI)?0:1,(ptr->option & VPP_VGA_VSYNC_POLAR_HI)?0:1);
	govrh_set_dvo_sync_polar((ptr->option & VPP_VGA_HSYNC_POLAR_HI)?0:1,(ptr->option & VPP_VGA_VSYNC_POLAR_HI)?0:1);
	vppif_reg32_write(0xd8050904,BIT7,7,(ptr->option & VPP_OPT_INTERLACE)?1:0);

#if(WMT_CUR_PID == WMT_PID_8435)
	if( vppif_reg32_read(GOVRH_DVO_OUTWIDTH) ){
		clk_delay = 0x30;
	}
	else {
		clk_delay = 0x7E0;
	}
#else
	if( vppif_reg32_read(GOVRH_DVO_OUTWIDTH) ){
		if( pixel_clock > 121500000 ){
			clk_delay = 0x1900;
		}
		else if( pixel_clock > 101250000 ){
			clk_delay = 0x1813;
		}
	}
#endif
	govrh_set_dvo_clock_delay(((clk_delay & 0x1000)!=0x0),clk_delay & 0xFFF);
//	vppif_reg32_write(GOVRH_TG_ENABLE, 1);
}

void govrh_set_csc_mode(vpp_csc_t mode)
{
	vdo_color_fmt src_fmt,dst_fmt;
	unsigned int enable;

	const U32 csc_mode_parm[VPP_CSC_MAX][9] = {
		{0x000004a8, 0x04a80662, 0x1cbf1e70, 0x081204a8, 0x00000000, 0x00010001, 0x00000001, 0x00000003, 0x18},	// YUV2RGB_SDTV_0_255
		{0x00000400, 0x0400057c, 0x1d351ea8, 0x06ee0400, 0x00000000, 0x00010001, 0x00000001, 0x00000001, 0x18},	// YUV2RGB_SDTV_16_235
		{0x000004a8, 0x04a8072c, 0x1ddd1f26, 0x087604a8, 0x00000000, 0x00010001, 0x00000001, 0x00000003, 0x18},	// YUV2RGB_HDTV_0_255
		{0x00000400, 0x04000629, 0x1e2a1f45, 0x07440400, 0x00000000, 0x00010001, 0x00000001, 0x00000001, 0x18},	// YUV2RGB_HDTV_16_235
		{0x00000400, 0x0400059c, 0x1d251ea0, 0x07170400, 0x00000000, 0x00010001, 0x00000001, 0x00000001, 0x18},	// YUV2RGB_JFIF_0_255
		{0x00000400, 0x0400057c, 0x1d351ea8, 0x06ee0400, 0x00010000, 0x00010001, 0x00000001, 0x00000001, 0x18},	// YUV2RGB_SMPTE170M
		{0x00000400, 0x0400064d, 0x1e001f19, 0x074f0400, 0x00010000, 0x00010001, 0x00000001, 0x00000003, 0x18},	// YUV2RGB_SMPTE240M
		{0x02040107, 0x1f680064, 0x01c21ed6, 0x1e8701c2, 0x00001fb7, 0x01010021, 0x00000101, 0x00000000, 0x1C},	// RGB2YUV_SDTV_0_255
		{0x02590132, 0x1f500075, 0x020b1ea5, 0x1e4a020b, 0x00001fab, 0x01010001, 0x00000101, 0x00000000, 0x1C},	// RGB2YUV_SDTV_16_235
		{0x027500bb, 0x1f99003f, 0x01c21ea6, 0x1e6701c2, 0x00001fd7, 0x01010021, 0x00000101, 0x00000000, 0x1C},	// RGB2YUV_HDTV_0_255
		{0x02dc00da, 0x1f88004a, 0x020b1e6d, 0x1e25020b, 0x00001fd0, 0x01010001, 0x00000101, 0x00000000, 0x1C},	// RGB2YUV_HDTV_16_235
		{0x02590132, 0x1f530075, 0x02001ead, 0x1e530200, 0x00001fad, 0x01010001, 0x00000101, 0x00000000, 0x1C},	// RGB2YUV_JFIF_0_255
		{0x02590132, 0x1f500075, 0x020b1ea5, 0x1e4a020b, 0x00011fab, 0x01010101, 0x00000000, 0x00000000, 0x1C},	// RGB2YUV_SMPTE170M
		{0x02ce00d9, 0x1f890059, 0x02001e77, 0x1e380200, 0x00011fc8, 0x01010101, 0x00000000, 0x00000000, 0x1C},	// RGB2YUV_SMPTE240M
	};

	enable = 0;
	src_fmt = govrh_get_color_format();
	dst_fmt = govrh_get_dvo_color_format();
	if( src_fmt == VDO_COL_FMT_ARGB ){
		enable |= (dst_fmt != VDO_COL_FMT_ARGB)? 0x01:0x00;	// DVO
	}
	else {
		enable |= (dst_fmt == VDO_COL_FMT_ARGB)? 0x01:0x00;	// DVO
		enable |= 0x2;	// VGA
	}
#ifdef WMT_FTBLK_DISP
	if( src_fmt == VDO_COL_FMT_ARGB ){
		if( disp_get_color_format() != VDO_COL_FMT_ARGB){
			enable |= BIT2;
		}
	}
	else {
		if( disp_get_color_format() == VDO_COL_FMT_ARGB){
			enable |= BIT2;
		}
	}
#endif
#ifdef WMT_FTBLK_LVDS
	if( src_fmt == VDO_COL_FMT_ARGB ){
		if( govrh_LVDS_get_color_format() != VDO_COL_FMT_ARGB){
			enable |= BIT3;
		}
	}
	else {
		if( govrh_LVDS_get_color_format() == VDO_COL_FMT_ARGB){
			enable |= BIT3;
		}
	}
#endif
	mode = vpp_check_csc_mode(mode,src_fmt,dst_fmt,enable);
	if( mode >= VPP_CSC_MAX ){
		vppif_reg32_write(GOVRH_DVO_YUV2RGB_ENABLE,0);
		vppif_reg32_write( GOVRH_VGA_YUV2RGB_ENABLE,0);
#ifdef WMT_FTBLK_DISP
		vppif_reg32_write( GOVRH_DISP_CSC_ENABLE,0);
#endif
#ifdef WMT_FTBLK_LVDS
		vppif_reg32_write( GOVRH_LVDS_CSC_ENABLE,0);
#endif
		return;
	}

	vppif_reg32_out(REG_GOVRH_DMACSC_COEF0, csc_mode_parm[mode][0]);
	vppif_reg32_out(REG_GOVRH_DMACSC_COEF1, csc_mode_parm[mode][1]);
	vppif_reg32_out(REG_GOVRH_DMACSC_COEF2, csc_mode_parm[mode][2]);
	vppif_reg32_out(REG_GOVRH_DMACSC_COEF3, csc_mode_parm[mode][3]);
	vppif_reg32_out(REG_GOVRH_DMACSC_COEF4, csc_mode_parm[mode][4]);
	vppif_reg32_out(REG_GOVRH_DMACSC_COEF5, csc_mode_parm[mode][5]);
	vppif_reg32_out(REG_GOVRH_DMACSC_COEF6, csc_mode_parm[mode][6]);
	vppif_reg32_out(REG_GOVRH_CSC_MODE, csc_mode_parm[mode][7]);

	/* enable bit0:DVO, bit1:VGA, bit2:DISP, bit3:LVDS */
#ifdef WMT_FTBLK_DISP
	vppif_reg32_write( GOVRH_DISP_CSC_ENABLE,(enable & BIT2)?1:0);
#endif
#ifdef WMT_FTBLK_LVDS
	vppif_reg32_write( GOVRH_LVDS_CSC_ENABLE,(enable & BIT3)?1:0);
#endif
	enable = enable & 0x3;
	vppif_reg32_out(REG_GOVRH_YUV2RGB, (csc_mode_parm[mode][8] | enable));
}

void govrh_set_framebuffer(vdo_framebuf_t *fb)
{
	govrh_set_MIF_enable(VPP_FLAG_DISABLE);
	govrh_set_fb_addr(fb->y_addr, fb->c_addr);
	govrh_set_data_format(fb->col_fmt);
	govrh_set_fb_info(fb->fb_w, fb->img_w, fb->h_crop, fb->v_crop);
	govrh_set_source_format(vpp_get_fb_field(fb));
	govrh_set_csc_mode(p_govrh->fb_p->csc_mode);
	// govrh TG
//	govrh_set_video_mode(fb->img_w,fb->img_h,p_govrh->fb_p->framerate,p_govrh->vmode);
	govrh_set_MIF_enable(VPP_FLAG_ENABLE);
}

void govrh_get_framebuffer(vdo_framebuf_t *fb)
{
	govrh_get_fb_addr(&fb->y_addr,&fb->c_addr);
	fb->col_fmt = govrh_get_color_format();
	govrh_get_fb_info(&fb->fb_w, &fb->img_w, &fb->h_crop, &fb->v_crop);
	fb->flag = vppif_reg32_read(GOVRH_INFMT)? VDO_FLAG_INTERLACE:0;
}

#ifdef WMT_FTBLK_GOVRH_EXT
void govrh_set_media_format(vpp_flag_t h264)
{
	vppif_reg32_write(GOVRH_H264_FMT,h264);
}
#endif /* WMT_FTBLK_GOVRH_EXT */

/*----------------------- GOVRH LVDS --------------------------------------*/
#ifdef WMT_FTBLK_LVDS
void govrh_LVDS_set_enable(vpp_flag_t enable)
{
	vppif_reg32_write(GOVRH_LVDS_EN,enable);
}

void govrh_LVDS_set_polar(vpp_flag_t hsync,vpp_flag_t vsync)
{
	/* 0-active High, 1-active Low */
	vppif_reg32_write(GOVRH_LVDS_HSYNC_POLAR,hsync);
	vppif_reg32_write(GOVRH_LVDS_VSYNC_POLAR,vsync);
}

void govrh_LVDS_set_width(unsigned int width)
{
	vppif_reg32_write(GOVRH_LVDS_OUTWIDTH,width);
}

void govrh_LVDS_set_color_format(vdo_color_fmt fmt)
{
	switch(fmt){
		case VDO_COL_FMT_ARGB:
			vppif_reg32_write(GOVRH_LVDS_PIX,0x1);
			break;
		case VDO_COL_FMT_YUV444:
			vppif_reg32_write(GOVRH_LVDS_PIX,0x0);
			break;
		case VDO_COL_FMT_YUV422H:
			vppif_reg32_write(GOVRH_LVDS_PIX,0x2);
			break;
		default:
			GOVRHMSG("*E* not support\n");
			break;
	}
}

vdo_color_fmt govrh_LVDS_get_color_format(void)
{
	vdo_color_fmt fmt;

	switch( vppif_reg32_read(GOVRH_LVDS_PIX) ){
		case 0x0:
			fmt = VDO_COL_FMT_YUV444;
			break;
		case 0x1:
			fmt = VDO_COL_FMT_ARGB;
			break;
		case 0x2:
		default:
			fmt = VDO_COL_FMT_YUV422H;
			break;
	}
	return fmt;
}

void govrh_LVDS_set_clk(int double_channel,vpp_flag_t inv)
{
	vppif_reg32_write(GOVRH_LVDS_CLK_SEL,double_channel);
	vppif_reg32_write(GOVRH_LVDS_CLKINV_EN,inv);
}

#endif /* WMT_FTBLK_LVDS */

/*----------------------- GOVRH DISP --------------------------------------*/
#ifdef WMT_FTBLK_DISP
void govrh_DISP_set_enable(vpp_flag_t enable)
{
	/* 1: enable DISP, and disable DVO/VGA */
	vppif_reg32_write(GOVRH_DISP_SEL,enable);

	/* TODO : set VPP DAC SEL */
//	vppif_reg32_write(WM8510_VPP_BASE+0x14,BIT0,0,enable);
}
#endif /* WMT_FTBLK_DISP */

/*----------------------- GOVRH IGS --------------------------------------*/
#ifdef WMT_FTBLK_IGS
void govrh_IGS_set_mode(int no,int mode_18bit,int msb)
{
	if( no == 0 ){ // DVO
		vppif_reg32_write(GOVRH_IGS_MODE,mode_18bit);	// 0-24bit,10-18bit,01-555,11-565
		vppif_reg32_write(GOVRH_LDI_MODE,msb);			// 0-lsb,1-msb
	}
	else { // LVDS
		vppif_reg32_write(GOVRH_IGS_MODE2,mode_18bit);	// 0-24bit,10-18bit,01-555,11-565
		vppif_reg32_write(GOVRH_LDI_MODE2,msb);			// 0-lsb,1-msb
	}
}
#endif /* WMT_FTBLK_IGS */

#ifdef CONFIG_PM
static unsigned int *govrh_reg_bk2;
static unsigned int govrh_pm_enable,govrh_pm_tg;
static unsigned int govrh_vo_div,govrh_vo_pll;
void govrh_suspend(int sts)
{
	switch( sts ){
		case 0:	// disable module
			govrh_pm_enable = vppif_reg32_read(GOVRH_MIF_ENABLE);
			vppif_reg32_write(GOVRH_MIF_ENABLE,0);
			break;
		case 1: // disable tg
			govrh_pm_tg = vppif_reg32_read(GOVRH_TG_ENABLE);
			vppif_reg32_write(GOVRH_TG_ENABLE,0);
			break;
		case 2:	// backup register
			p_govrh->reg_bk = vpp_backup_reg(GOVRH_BASE1_ADDR+0x30,0xB4);	/* 0x30 - 0xE8 */
			govrh_reg_bk2 = vpp_backup_reg(GOVRH_BASE2_ADDR+0x00,0xEC);		/* 0x00 - 0xE8 */
			govrh_vo_div = vppif_reg32_in(0xd8130374);
			govrh_vo_pll = vppif_reg16_in(0xd8130208);
			break;
		default:
			break;
	}
}

void govrh_resume(int sts)
{
	switch( sts ){
		case 0:	// restore register
			vppif_reg32_out(0xd8130374,govrh_vo_div);
			vppif_reg16_out(0xd8130208,govrh_vo_pll);
			vpp_restore_reg(GOVRH_BASE2_ADDR+0x00,0xE8,govrh_reg_bk2);		/* 0x00 - 0xE8 */
			vpp_restore_reg(GOVRH_BASE1_ADDR+0x30,0xB4,p_govrh->reg_bk);	/* 0x30 - 0xE8 */
			govrh_reg_bk2 = 0;
			p_govrh->reg_bk = 0;
			break;
		case 1:	// enable module
			vppif_reg32_write(GOVRH_MIF_ENABLE,govrh_pm_enable);
			break;
		case 2: // enable tg
			vppif_reg32_write(GOVRH_TG_ENABLE,govrh_pm_tg);
			break;
		default:
			break;
	}
}
#else
#define govrh_suspend NULL
#define govrh_resume NULL
#endif

void govrh_init(void *base)
{
	govrh_mod_t *mod_p;

	mod_p = (govrh_mod_t *) base;

	govrh_set_reg_level(VPP_REG_LEVEL_1);
	govrh_set_colorbar(VPP_FLAG_DISABLE, 1, 0);
	govrh_set_tg_enable(VPP_FLAG_ENABLE);
	govrh_set_dvo_color_format(VDO_COL_FMT_ARGB);
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	vppif_reg32_out(REG_GOVRH_DAC_VAL,GOVRH_DAC_SENSE_VALUE << 16 | GOVRH_DAC_SENSE_VALUE);
	govrh_set_DAC_pwrdn(VPP_FLAG_ENABLE);
	govrh_set_framebuffer(&mod_p->fb_p->fb);
	govrh_set_fifo_index(0xf);
	govrh_set_contrast(0x80);
	govrh_set_brightness(0x0);
	govrh_set_csc_mode(mod_p->fb_p->csc_mode);
  	vppif_reg32_out(REG_GOVRH_DAC_TEST, 0x7000000);
	vppif_reg32_out(REG_GOVRH_DAC_MOD, 0x00770000);
	govrh_set_reg_update(VPP_FLAG_ENABLE);
	govrh_set_tg_enable(VPP_FLAG_ENABLE);
	govrh_set_MIF_enable(VPP_FLAG_ENABLE);
}

int govrh_mod_init(void)
{
	govrh_mod_t *mod_p;
	vpp_fb_base_t *mod_fb_p;
	vdo_framebuf_t *fb_p;

	mod_p = (govrh_mod_t *) vpp_mod_register(VPP_MOD_GOVRH,sizeof(govrh_mod_t),VPP_MOD_FLAG_FRAMEBUF);
	if( !mod_p ){
		DPRINT("*E* GOVRH module register fail\n");
		return -1;
	}

	/* module member variable */
	mod_p->int_catch = VPP_INT_NULL;
	mod_p->vga_dac_sense_cnt = VPP_DAC_SENSE_SECOND * 60;

	/* module member function */
	mod_p->init = govrh_init;
	mod_p->dump_reg = govrh_reg_dump;
	mod_p->set_enable = govrh_set_MIF_enable;
	mod_p->set_colorbar = govrh_set_colorbar;
	mod_p->set_tg = govrh_set_timing;
	mod_p->get_tg = govrh_get_timing;
	mod_p->get_sts = govrh_get_int_status;
	mod_p->clr_sts = govrh_clean_int_status;
	mod_p->suspend = govrh_suspend;
	mod_p->resume = govrh_resume;

	/* module frame buffer */
	mod_fb_p = mod_p->fb_p;
	mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_JFIF_0_255;
	mod_fb_p->framerate = VPP_HD_DISP_FPS;
	mod_fb_p->media_fmt = VPP_MEDIA_FMT_MPEG;
	mod_fb_p->wait_ready = 0;
	mod_fb_p->capability = BIT(VDO_COL_FMT_YUV420) | BIT(VDO_COL_FMT_YUV422H) | BIT(VDO_COL_FMT_YUV444) | BIT(VDO_COL_FMT_ARGB)
							| VPP_FB_FLAG_CSC | VPP_FB_FLAG_MEDIA | VPP_FB_FLAG_FIELD;

	/* module frame buffer member function */
	mod_fb_p->set_framebuf = govrh_set_framebuffer;
	mod_fb_p->set_addr = govrh_set_fb_addr;
	mod_fb_p->get_addr = govrh_get_fb_addr;
	mod_fb_p->set_csc = govrh_set_csc_mode;
	mod_fb_p->get_color_fmt = govrh_get_color_format;
	mod_fb_p->set_color_fmt = (void *) govrh_set_data_format;

	fb_p = &mod_p->fb_p->fb;
	fb_p->y_addr = 0;
	fb_p->c_addr = 0;
	fb_p->col_fmt = VDO_COL_FMT_YUV444;
	fb_p->img_w = VPP_HD_DISP_RESX;
	fb_p->img_h = VPP_HD_DISP_RESY;
	fb_p->fb_w = VPP_HD_MAX_RESX;
	fb_p->fb_h = VPP_HD_MAX_RESY;
	fb_p->h_crop = 0;
	fb_p->v_crop = 0;
	fb_p->flag = 0;

	p_govrh = mod_p;
	return 0;
}
#ifdef __KERNEL__
module_init(govrh_mod_init);
#endif
#endif				//WMT_FTBLK_GOVRH
