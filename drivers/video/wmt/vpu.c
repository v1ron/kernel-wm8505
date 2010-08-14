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


/*--- History ------------------------------------------------------------------- 
* Version 0.01 , Sam Shen, 2009/03/12
*           First version
*
*------------------------------------------------------------------------------*/

#define VPU_C
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "vpu.h"

#ifdef WMT_FTBLK_VPU
/*----------------------- PRIVATE MACRO --------------------------------------*/
// #define DEBUG
#ifdef DEBUG
#define DBGMSG(fmt, args...)  DPRINT("[VPU] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...) do {} while(0)
#endif

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define VPU_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx vpu_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in vpu.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  vpu_xxx;        *//*Example*/

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void vpu_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
void vpu_set_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_SCALAR_ENABLE,enable);
}

void vpu_set_reg_update(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_REG_UPDATE,enable);
}

void vpu_set_reg_read_select(int level)
{
	vppif_reg32_write(VPU_REG_READ_SEL,level);
}

/*----------------------- VPU SCALE --------------------------------------*/
void vpu_set_H_scale_ptr(unsigned int init_val,unsigned int ret_val)
{
	unsigned int reg;

	reg = ((init_val) << 16) + (ret_val);
	vppif_reg32_out(REG_VPU_HPTR,reg);	

	/* init V scale registers */
	vppif_reg32_out(REG_VPU_HTB0,0x1);
	vppif_reg32_out(REG_VPU_HTB1,0x0);
	vppif_reg32_out(REG_VPU_HRES_TB,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB0,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB1,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB2,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB3,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB4,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB5,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB6,0x0);
	vppif_reg32_out(REG_VPU_HDIV_TB7,0x0);
}

void vpu_set_H_scale_table(int no,int value)
{
	int reg_offset = no / 16;
	int tab_offset = no % 16;

	/* value 
 	3: The interpolation line data ((current line data + previous line data) /DIV) will be latched to next stage 
 	1: The current line data will be latched to next stage
 	0: The current line will be by held
 	*/
	vppif_reg32_write(REG_VPU_HTB0+ (4*reg_offset), (0x3 << (2*tab_offset)), (2*tab_offset), value);
}

void vpu_set_H_rec_table(int no,int value)
{
	/* value 
	0: The current line data will be written back into line buffer
	1: The interpolation line data ((current line data + previous line data) /DIV) will be written back into line buffer
	*/
	vppif_reg32_write(REG_VPU_HRES_TB, (0x1 << no), no, value);
}

void vpu_set_H_div_table(int no,int value)
{
	int reg_offset = no / 4;
	int tab_offset = no % 4;

	vppif_reg32_write(REG_VPU_HDIV_TB0 + (4*reg_offset),(0xFF << (8*tab_offset)),(8*tab_offset),value);
}

void vpu_set_V_scale_ptr(unsigned int init_val,unsigned int ret_val)
{
	unsigned int reg;

	reg = ((init_val) << 16) + (ret_val);
	vppif_reg32_out(REG_VPU_VPTR,reg);	

	/* init V scale registers */
	vppif_reg32_out(REG_VPU_VTB0,0x1);
	vppif_reg32_out(REG_VPU_VTB1,0x0);
	vppif_reg32_out(REG_VPU_VRES_TB,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB0,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB1,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB2,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB3,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB4,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB5,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB6,0x0);
	vppif_reg32_out(REG_VPU_VDIV_TB7,0x0);
}

void vpu_set_V_scale_table(int no,int value)
{
	/* value 
 	3: The interpolation line data ((current line data + previous line data) /DIV) will be latched to next stage 
 	1: The current line data will be latched to next stage
 	0: The current line will be by held
 	*/
	int reg_offset = no / 16;
	int tab_offset = no % 16;

	vppif_reg32_write(REG_VPU_VTB0+ (4*reg_offset), (0x3 << (2*tab_offset)), (2*tab_offset), value);
}

void vpu_set_V_rec_table(int no,int value)
{
	/* value 
	0: The current line data will be written back into line buffer
	1: The interpolation line data ((current line data + previous line data) /DIV) will be written back into line buffer
	*/
	vppif_reg32_write(REG_VPU_VRES_TB, (0x1 << no), no, value);
}

void vpu_set_V_div_table(int no,int value)
{
	int reg_offset = no / 4;
	int tab_offset = no % 4;

	vppif_reg32_write(REG_VPU_VDIV_TB0 + (4*reg_offset),(0xFF << (8*tab_offset)),(8*tab_offset),value);
}

static void vpu_set_RT_bilinear(int src,int dst,int htb)
{
	int i,index;
	int integer_pos = 0;
	int decimal_pos = 0;
	int threshold;
	int scl_tb = 0,div_tb = 0;
	int pos;

	pos = src * 100 / dst;
	threshold = ( (dst * 2) > src )? 25:50;
	for (i=0,index=0; i<src; i++){
		if( i == integer_pos ){
			if (decimal_pos <= threshold){
				scl_tb = 0x1;
				div_tb = 0x0;
			}
			else if (decimal_pos > threshold){
				scl_tb = 0x0;
				div_tb = 0x0;
			}
		}
		else if( i > integer_pos ){
			if(decimal_pos > threshold){
				scl_tb = 0x3;
				div_tb = 0x80;
			}
			else{
				scl_tb = 0x0;
				div_tb = 0x0;
			}
		}
		else if( i < integer_pos ){
			scl_tb = 0x0;
			div_tb = 0x0;
		}

		if( scl_tb != 0x0 ){
			index++;
			integer_pos = (index * pos) / 100;
			decimal_pos = (index * pos) % 100;
		}

		if( htb ){
			vpu_set_H_rec_table(i,0);
			vpu_set_H_scale_table(i,scl_tb);
			vpu_set_H_div_table(i,div_tb);
		}
		else {
			vpu_set_V_rec_table(i,0);
			vpu_set_V_scale_table(i,scl_tb);
			vpu_set_V_div_table(i,div_tb);
		}
	}
}

static void vpu_set_RT_htb(int Src_Width, int Width)
{
	int A,B;
	int gcd;
	int i, j;
	int pre,cur;
	int index;
	int diff;

	if(Src_Width < Width){
		vppif_reg32_out(REG_VPU_HTB0,3);
		vppif_reg32_out(REG_VPU_HPTR,0);
		return;
	}

	gcd = vpp_get_gcd(Src_Width,Width);
	A = Width / gcd;
	B = Src_Width / gcd;
//	printk("H Src %d,Dst %d, (%d,%d)\n",Src_Width,Width,A,B);
	if ((Src_Width == Width) || (B>32)){
		vpu_set_H_scale_ptr(0,0);
		return;
	}

	vpu_set_H_scale_ptr(0,B-1);

#ifdef PATCH_VPP_SCALE_DOWN_QUALITY
	if( (B != 1) && ((4*A)>B) ){
		vpu_set_RT_bilinear(B, A, 1);
		return;
	}
#endif
	
	for(i=0,pre=0,index=0;i<A;i++){
		cur = (B*(i+1))/A;
		diff = cur - pre;
		pre = cur;
		if( diff == 1 ){
			vpu_set_H_rec_table(index,0);	// first 0, other 1
			vpu_set_H_scale_table(index,1);	// diff(1) 1, diff(>1) last 3,other 0
			vpu_set_H_div_table(index,0);	// last 256/diff, other 0			
			index++;
		}
		else {
			/* 1st */
			vpu_set_H_rec_table(index,0);	// first 0, other 1
			vpu_set_H_scale_table(index,0);	// diff(1) 1, diff(>1) last 3,other 0
			vpu_set_H_div_table(index,0);	// last 256/diff, other 0
			index++;

			/* middle */
			for(j=0;j<(diff-2);j++){
				vpu_set_H_rec_table(index,1);	// first 0, other 1
				vpu_set_H_scale_table(index,0);	// diff(1) 1, diff(>1) last 3,other 0
				vpu_set_H_div_table(index,0);	// last 256/diff, other 0
				index++;
			}

			/* last */
			vpu_set_H_rec_table(index,1);	// first 0, other 1
			vpu_set_H_scale_table(index,3);	// diff(1) 1, diff(>1) last 3,other 0
			vpu_set_H_div_table(index,(256/diff));	// last 256/diff, other 0		
			index++;
		}
	}
}

static void vpu_set_RT_vtb(int Src_Height, int Height)
{
	int A,B;
	int gcd;
	int i, j;
	int pre,cur;
	int index;
	int diff;


	if( Src_Height < Height ){
		vppif_reg32_out(REG_VPU_VTB0,3);
		vppif_reg32_out(REG_VPU_VPTR,0);
		return;
	}

	gcd = vpp_get_gcd(Src_Height,Height);
	A = Height / gcd;
	B = Src_Height / gcd;
	if ((Src_Height == Height) || (B>32)){
		vpu_set_V_scale_ptr(0,0);
		return;
	}

	vpu_set_V_scale_ptr(0,B-1);

#ifdef PATCH_VPP_SCALE_DOWN_QUALITY
	if( (B != 1) && ((4*A)>B) ){
		vpu_set_RT_bilinear(B, A, 0);
		return;
	}
#endif
	
	for(i=0,pre=0,index=0;i<A;i++){
		cur = (B*(i+1))/A;
		diff = cur - pre;
		pre = cur;
		if( diff == 1 ){
			vpu_set_V_rec_table(index,0);	// first 0, other 1
			vpu_set_V_scale_table(index,1);	// diff(1) 1, diff(>1) last 3,other 0
			vpu_set_V_div_table(index,0);	// last 256/diff, other 0			
			index++;
		}
		else {
			/* 1st */
			vpu_set_V_rec_table(index,0);	// first 0, other 1
			vpu_set_V_scale_table(index,0);	// diff(1) 1, diff(>1) last 3,other 0
			vpu_set_V_div_table(index,0);	// last 256/diff, other 0
			index++;

			/* middle */
			for(j=0;j<(diff-2);j++){
				vpu_set_V_rec_table(index,1);	// first 0, other 1
				vpu_set_V_scale_table(index,0);	// diff(1) 1, diff(>1) last 3,other 0
				vpu_set_V_div_table(index,0);	// last 256/diff, other 0
				index++;
			}

			/* last */
			vpu_set_V_rec_table(index,1);	// first 0, other 1
			vpu_set_V_scale_table(index,3);	// diff(1) 1, diff(>1) last 3,other 0
			vpu_set_V_div_table(index,(256/diff));	// last 256/diff, other 0
			index++;
		}
	}
}

void vpu_set_scale_enable(vpp_flag_t vscl_enable, vpp_flag_t hscl_enable)
{
	DBGMSG("V %d,H %d\n",vscl_enable,hscl_enable);
	vppif_reg32_write(VPU_V_SCALEUP,vscl_enable);
	vppif_reg32_write(VPU_H_SCALEUP,hscl_enable);
}

void vpu_set_V_scale(unsigned int A,unsigned int B)
{
	unsigned int V_STEP;
	unsigned int V_SUB_STEP;
	unsigned int V_THR_DIV2;

//	DBGMSG("vpu_set_V_scale(%d,%d)\r\n", A, B);
	if( A > B ){
		V_STEP = (B -1) * 16 / A;
	   	V_SUB_STEP = (B -1) * 16  % A;
	}
	else {
		V_STEP = (16 * B / A);
		V_SUB_STEP = ((16 * B) % A);
	}
	V_THR_DIV2 = A;
//	DBGMSG("V step %d,sub step %d, div2 %d\r\n", V_STEP, V_SUB_STEP, V_THR_DIV2);

	vppif_reg32_write(VPU_VXWIDTH,B);
	vppif_reg32_write(VPU_V_STEP, V_STEP);
	vppif_reg32_write(VPU_V_SUBSTEP, V_SUB_STEP);
	vppif_reg32_write(VPU_V_THR, V_THR_DIV2);
	vppif_reg32_write(VPU_V_I_SUBSTEPCNT, 0);
}

void vpu_set_H_scale(unsigned int A,unsigned int B)
{
	unsigned int H_STEP;
	unsigned int H_SUB_STEP;
	unsigned int H_THR_DIV2;

//	DBGMSG("vpu_set_H_scale(%d,%d)\r\n", A, B);
	if( A > B ){
		H_STEP = (B -1) * 16 / A;
	   	H_SUB_STEP = (B -1) * 16  % A;
	}
	else {
		H_STEP = (16 * B / A);
		H_SUB_STEP = ((16 * B) % A);
	}
	H_THR_DIV2 = A;
//	DBGMSG("H step %d,sub step %d, div2 %d\r\n", H_STEP, H_SUB_STEP, H_THR_DIV2);

	vppif_reg32_write(VPU_HXWIDTH,((A>B)?A:B));
	vppif_reg32_write(VPU_H_STEP, H_STEP);
	vppif_reg32_write(VPU_H_SUBSTEP, H_SUB_STEP);
	vppif_reg32_write(VPU_H_THR, H_THR_DIV2);
	vppif_reg32_write(VPU_H_I_SUBSTEPCNT, 0);
}

void vpu_set_scale(unsigned int SRC_W,unsigned int SRC_H,unsigned int DST_W,unsigned int DST_H)
{
	int h_scale_up;
	int v_scale_up;

	DBGMSG("src(%dx%d),dst(%dx%d)\n",SRC_W,SRC_H,DST_W,DST_H);

	h_scale_up = ( DST_W > SRC_W )? 1:0;
	v_scale_up = ( DST_H > SRC_H )? 1:0;

	if( ((DST_W / SRC_W) >= 32) || ((DST_W / SRC_W) < 1/32)  ){
		DBGMSG("*W* VPU H scale rate invalid\n");
	}

	if( ((DST_H / SRC_H) >= 32) || ((DST_H / SRC_H) < 1/32)  ){
		DBGMSG("*W* VPU V scale rate invalid\n");
	}

	vppif_reg32_out(REG_VPU_V_BYPASS,(DST_W > 1440)? 1:0);

//	DBGMSG("scale H %d,V %d\n",h_scale_up,v_scale_up);
	vpu_set_H_scale(DST_W,SRC_W);
	vpu_set_RT_htb(SRC_W,DST_W);
#ifdef PATCH_SCL_SCALEUP
	if( h_scale_up ){
		vppif_reg32_write(VPU_R_YPXLWID,vppif_reg32_read(VPU_R_YPXLWID)+1);
	}
#endif

	vpu_set_V_scale(DST_H,SRC_H);
	vpu_set_RT_vtb(SRC_H,DST_H);
	vpu_set_scale_enable(v_scale_up, h_scale_up);
}

void vpu_set_crop(int offset_x, int offset_y)
{
	offset_x &= ~0xf;

	vppif_reg32_write(VPU_H_I_STEPCNT, offset_x * 16);
	vppif_reg32_write(VPU_V_I_STEPCNT, offset_y * 16);
	DBGMSG("crop - x : 0x%x, y : 0x%x \r\n", offset_x * 16, offset_y * 16);
}

void vpu_set_drop_line(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_DROP_LINE,enable);
}

/*----------------------- VPU CSC --------------------------------------*/
void vpu_set_csc_mode(vpp_csc_t mode)
{
	vdo_color_fmt src_fmt,dst_fmt;

	src_fmt = vpu_r_get_color_format();
#if (defined(WMT_FTBLK_GOVW_CSC) && defined(WMT_FTBLK_PIP))
	dst_fmt = ( vpu_get_timing_master() == VPP_MOD_VPU )? vpu_w_get_color_format():govm_get_pip_color_format();
#else
	dst_fmt = ( vpu_get_timing_master() == VPP_MOD_VPU )? vpu_w_get_color_format():VDO_COL_FMT_YUV444;
#endif
	mode = vpp_check_csc_mode(mode,src_fmt,dst_fmt,0);

	if (mode >= VPP_CSC_MAX) {
		vppif_reg32_write(VPU_CSC_EN, VPP_FLAG_DISABLE);
	} else {
		vppif_reg32_out(REG_VPU_CSC1, vpp_csc_parm[mode][0]);	//CSC1
		vppif_reg32_out(REG_VPU_CSC2, vpp_csc_parm[mode][1]);	//CSC2
		vppif_reg32_out(REG_VPU_CSC3, vpp_csc_parm[mode][2]);	//CSC3
		vppif_reg32_out(REG_VPU_CSC4, vpp_csc_parm[mode][3]);	//CSC4
		vppif_reg32_out(REG_VPU_CSC5, vpp_csc_parm[mode][4]);	//CSC5
		vppif_reg32_out(REG_VPU_CSC6, vpp_csc_parm[mode][5]);	//CSC6
		vppif_reg32_out(REG_VPU_CSC, vpp_csc_parm[mode][6]);	//CSC_CTL
		vppif_reg32_write(VPU_CSC_EN, VPP_FLAG_ENABLE);
	}
}

/*----------------------- VPU TG --------------------------------------*/
void vpu_set_tg_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_TG_ENABLE, enable);
}

void vpu_set_timing(vpp_clock_t *timing)
{
	timing->read_cycle = ( timing->read_cycle < 3 )? 3:timing->read_cycle;
	timing->read_cycle = ( timing->read_cycle > 255 )? 0xFF:timing->read_cycle;

	vppif_reg32_write(VPU_TG_RDCYC, timing->read_cycle);
	vppif_reg32_write(VPU_TG_H_ALLPIXEL, timing->total_pixel_of_line);
	vppif_reg32_write(VPU_TG_H_ACTBG, timing->begin_pixel_of_active);
	vppif_reg32_write(VPU_TG_H_ACTEND, timing->end_pixel_of_active);
	vppif_reg32_write(VPU_TG_V_ALLLINE, timing->total_line_of_frame);
	vppif_reg32_write(VPU_TG_V_ACTBG, timing->begin_line_of_active);
	vppif_reg32_write(VPU_TG_V_ACTEND, timing->end_line_of_active);
	vppif_reg32_write(VPU_TG_VBIE, timing->line_number_between_VBIS_VBIE);
	vppif_reg32_write(VPU_TG_PVBI, timing->line_number_between_PVBI_VBIS);
}

void vpu_get_timing(vpp_clock_t * p_timing)
{
	p_timing->read_cycle = vppif_reg32_read(VPU_TG_RDCYC);
	p_timing->total_pixel_of_line = vppif_reg32_read(VPU_TG_H_ALLPIXEL);
	p_timing->begin_pixel_of_active = vppif_reg32_read(VPU_TG_H_ACTBG);
	p_timing->end_pixel_of_active = vppif_reg32_read(VPU_TG_H_ACTEND);
	p_timing->total_line_of_frame = vppif_reg32_read(VPU_TG_V_ALLLINE);
	p_timing->begin_line_of_active = vppif_reg32_read(VPU_TG_V_ACTBG);
	p_timing->end_line_of_active = vppif_reg32_read(VPU_TG_V_ACTEND);
	p_timing->line_number_between_VBIS_VBIE = vppif_reg32_read(VPU_TG_VBIE);
	p_timing->line_number_between_PVBI_VBIS = vppif_reg32_read(VPU_TG_PVBI);
}

void vpu_set_wait_ready(unsigned int count)
{
	if (0 != count) {
		vppif_reg32_write(VPU_TG_WAITRDY_CNT, count);
		vppif_reg32_write(VPU_TG_WAITRDY_ENABLE, VPP_FLAG_TRUE);
	} else {
		vppif_reg32_write(VPU_TG_WAITRDY_ENABLE, VPP_FLAG_FALSE);
	}
}

void vpu_set_timing_master(vpp_mod_t mod_bit)
{
	if (VPP_MOD_VPU == mod_bit) {
		vppif_reg32_write(VPU_TG_GOVWTG_ENABLE, VPP_FLAG_DISABLE);
	} 
	else if (VPP_MOD_GOVW == mod_bit) {
		vppif_reg32_write(VPU_TG_GOVWTG_ENABLE, VPP_FLAG_ENABLE);
	} 
	else {
		DBGMSG("*E* check the parameter.\n");
		return;
	}
}

vpp_mod_t vpu_get_timing_master(void)
{
	if( vppif_reg32_read(VPU_TG_GOVWTG_ENABLE) ){
		return VPP_MOD_GOVW;
	}
	return VPP_MOD_VPU;
}

/*----------------------- VPU R MIF --------------------------------------*/
void vpu_r_set_mif_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_R_MIF_ENABLE, enable);
}

void vpu_r_set_mif2_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_R_MIF2_ENABLE, enable);
}

void vpu_r_set_colorbar(vpp_flag_t enable,int width, int inverse)
{
	vppif_reg32_write(VPU_R_COLBAR_MODE, width);
	vppif_reg32_write(VPU_R_COLBAR_INVERSION, inverse);
	vppif_reg32_write(VPU_R_COLBAR_ENABLE, enable);
}

void vpu_r_set_field_mode(vpp_display_format_t fmt)
{
	vppif_reg32_write(VPU_R_FIELD_MODE,fmt);	/*0:Frame, 1:Field */
}

void vpu_r_set_color_format(vdo_color_fmt format)
{
	switch (format) {
	case VDO_COL_FMT_ARGB:
		vppif_reg32_write(VPU_R_COLFMT_RGB, 0x1);
		break;
	case VDO_COL_FMT_YUV444:
		vppif_reg32_write(VPU_R_COLFMT_RGB, 0x0);
		vppif_reg32_write(VPU_R_COLFMT_YUV, 0x2);
		break;
	case VDO_COL_FMT_YUV422H:
		vppif_reg32_write(VPU_R_COLFMT_RGB, 0x0);
		vppif_reg32_write(VPU_R_COLFMT_YUV, 0x0);
		break;
	case VDO_COL_FMT_YUV420:
		vppif_reg32_write(VPU_R_COLFMT_RGB, 0x0);
		vppif_reg32_write(VPU_R_COLFMT_YUV, 0x1);
		break;
	default:
		DBGMSG("*E* check the parameter.\n");
		return;
	}
}

vdo_color_fmt vpu_r_get_color_format(void)
{
	if( vppif_reg32_read(VPU_R_COLFMT_RGB) ){
		return VDO_COL_FMT_ARGB;
	}
	switch( vppif_reg32_read(VPU_R_COLFMT_YUV) ){
		case 0: return VDO_COL_FMT_YUV422H;
		case 1: return VDO_COL_FMT_YUV420;
		case 2: return VDO_COL_FMT_YUV444;
		default: break;		
	}
	return VDO_COL_FMT_YUV444;
}

void vpu_r_set_media_format(vpp_media_format_t format)
{
	switch (format) {
	case VPP_MEDIA_FMT_MPEG:
		vppif_reg32_write(VPU_R_MEDIAFMT_H264, 0x0);
		break;
	case VPP_MEDIA_FMT_H264:
		vppif_reg32_write(VPU_R_MEDIAFMT_H264, 0x1);
		break;
	default:
		DBGMSG("*E* check the parameter.\n");
		return;
	}
}

void vpu_r_set_420C_field(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_R_420C_FMT,enable);	/*0:Frame, 1:Field */
}

void vpu_r_set_fb_addr(unsigned int y_addr,unsigned int c_addr)
{
	unsigned int line_offset;

	vppif_reg32_out(REG_VPU_R_Y1SA,y_addr);
	vppif_reg32_out(REG_VPU_R_C1SA,c_addr);
//	line_offset = ( vppif_reg32_read(VPU_DEI_ENABLE) )? 0:vppif_reg32_read(VPU_R_YBUFWID);
	line_offset = 0;

	vppif_reg32_out(REG_VPU_R_Y2SA,y_addr+line_offset);
	switch(vpu_r_get_color_format()){
		case VDO_COL_FMT_YUV420:
#if(WMT_CUR_PID == WMT_PID_8425)	// sw patch for Y/C crop bug
			{
				int v_crop;
				unsigned int offset;
				if( (v_crop = vppif_reg32_read(VPU_R_VCROP)) ){
					offset = vppif_reg32_read(VPU_R_YBUFWID) * v_crop;
					vppif_reg32_write(VPU_R_VCROP, 0);
					vppif_reg32_out(REG_VPU_R_Y1SA,y_addr+offset);
					vppif_reg32_out(REG_VPU_R_C1SA,c_addr+offset/2);
				}
			}
#endif
			line_offset /= 2;
			break;
		case VDO_COL_FMT_YUV422H:
			break;
		case VDO_COL_FMT_ARGB:
			line_offset *= 4;
			vppif_reg32_out(REG_VPU_R_Y2SA,y_addr+line_offset);
			break;
		default:
			line_offset *= 2;
			break;
	}
	vppif_reg32_out(REG_VPU_R_C2SA,c_addr+line_offset);
}

void vpu_r_get_fb_addr(unsigned int *y_addr,unsigned int *c_addr)
{
	*y_addr = vppif_reg32_in(REG_VPU_R_Y1SA);
	*c_addr = vppif_reg32_in(REG_VPU_R_C1SA);
//	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n", *y_addr, *c_addr);
}

void vpu_r_set_width(unsigned int y_pixel,unsigned int y_buffer)
{
	vppif_reg32_write(VPU_R_YPXLWID, y_pixel);
	vppif_reg32_write(VPU_R_YBUFWID, y_buffer);
}

void vpu_r_get_width(unsigned int * p_y_pixel,unsigned int * p_y_buffer)
{
	*p_y_pixel = vppif_reg32_read(VPU_R_YPXLWID);
	*p_y_buffer = vppif_reg32_read(VPU_R_YBUFWID);
}

void vpu_r_set_crop(unsigned int h_crop,unsigned int v_crop)
{
	vppif_reg32_write(VPU_R_HCROP, h_crop);
	vppif_reg32_write(VPU_R_VCROP, v_crop);
	vppif_reg32_write(VPU_R_HCROP2, h_crop);
	if( vppif_reg32_read(VPU_DEI_ENABLE) && (vppif_reg32_read(VPU_DEI_MODE)==VPP_DEI_FIELD) ){
		vppif_reg32_write(VPU_R_VCROP2, v_crop+2);
	}
	else {
		vppif_reg32_write(VPU_R_VCROP2, v_crop+1);
	}
}

void vpu_r_set_threshold(unsigned int value)
{
	vppif_reg32_write(VPU_R_FIFO_THR, value);
}

void vpu_r_get_fb_info(unsigned int *p_y_buffer,unsigned int *p_y_pixel,unsigned int *x_offset,unsigned int *y_offset)
{
	*p_y_buffer = vppif_reg32_read(VPU_R_YBUFWID);
	*p_y_pixel = vppif_reg32_read(VPU_R_YPXLWID);
	*x_offset = vppif_reg32_read(VPU_R_HCROP);
	*y_offset = vppif_reg32_read(VPU_R_VCROP);
}

void vpu_r_set_framebuffer(vdo_framebuf_t *fb)
{
	static unsigned int img_w,img_h,vis_w,vis_h;
	unsigned int SRC_W,SRC_H,DST_W,DST_H;

	if( (img_w != fb->img_w) || (img_h != fb->img_h) 
		|| (vis_w != p_vpu->resx_visual) || (vis_h != p_vpu->resy_visual) ){
		SRC_W = img_w = fb->img_w;
		SRC_H = img_h = fb->img_h;
		DST_W = vis_w = p_vpu->resx_visual;
		DST_H = vis_h = p_vpu->resy_visual;
		vpp_check_scale_ratio(&SRC_W,&DST_W,VPP_SCALE_UP_RATIO_H,VPP_SCALE_DN_RATIO_H);
		vpp_check_scale_ratio(&SRC_H,&DST_H,VPP_SCALE_UP_RATIO_V,VPP_SCALE_DN_RATIO_V);
		p_vpu->resx_virtual_scale = SRC_W;
		p_vpu->resy_virtual_scale = SRC_H;
		p_vpu->resx_visual_scale = DST_W;
		p_vpu->resy_visual_scale = DST_H;
	}
	else {
		SRC_W = p_vpu->resx_virtual_scale;
		SRC_H = p_vpu->resy_virtual_scale;
		DST_W = p_vpu->resx_visual_scale;
		DST_H = p_vpu->resy_visual_scale;
	}

	vpu_r_set_color_format(fb->col_fmt);
	vpu_r_set_crop(fb->h_crop, fb->v_crop);
	vpu_r_set_width(SRC_W, fb->fb_w);
	vpu_r_set_fb_addr(fb->y_addr, fb->c_addr);
	if( (p_vpu->dei_mode == VPP_DEI_DYNAMIC) && (fb->flag & VDO_FLAG_INTERLACE) ){
		vpu_dei_set_enable(fb->flag & VDO_FLAG_INTERLACE);
	}
	else {
		vpu_dei_set_mode((fb->flag & VDO_FLAG_INTERLACE)? p_vpu->dei_mode:VPP_DEI_MAX);
	}
	vpu_r_set_420C_field(fb->flag & VDO_FLAG_INTERLACE);
//	vpu_r_set_field_mode(vpp_get_fb_field(fb));
	vpu_w_set_fb_width(DST_W,fb->fb_w);
	vpu_set_scale(SRC_W, SRC_H, DST_W, DST_H);
	vpu_set_csc_mode(p_vpu->fb_p->csc_mode);

	if( (vppif_reg32_in(REG_DEI_EN) & 0x107) == 0x105 ){	// motion vector deinterlace
		vpu_mvr_set_addr(fb->y_addr+(fb->fb_w*fb->fb_h)+1088);
		vpu_mvr_set_width(SRC_W,2048);
		vpu_mvr_set_field_mode(1);
		vpu_mvr_set_enable(VPP_FLAG_ENABLE);
	}
	else {
		vpu_mvr_set_enable(VPP_FLAG_DISABLE);
	}
}
/*----------------------- VPU W --------------------------------------*/
void vpu_w_set_mif_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_W_MIF_EN, enable);
}

void vpu_w_set_color_format(vdo_color_fmt format)
{
	switch (format) {
	case VDO_COL_FMT_ARGB:
		vppif_reg32_write(VPU_W_COLFMT_RGB, 1);
		break;
	case VDO_COL_FMT_YUV444:
		vppif_reg32_write(VPU_W_COLFMT_RGB, 0);
		vppif_reg32_write(VPU_W_COLFMT_YUV, 0);
		break;
	case VDO_COL_FMT_YUV422H:
		vppif_reg32_write(VPU_W_COLFMT_RGB, 0);
		vppif_reg32_write(VPU_W_COLFMT_YUV, 1);
		break;
	default:
		DBGMSG("*E* check the parameter.\n");
		return;
	}
}

vdo_color_fmt vpu_w_get_color_format(void)
{
	if( vppif_reg32_read(VPU_W_COLFMT_RGB) )
		return VDO_COL_FMT_ARGB;

	if( vppif_reg32_read(VPU_W_COLFMT_YUV) ){
		return VDO_COL_FMT_YUV422H;
	}
	return VDO_COL_FMT_YUV444;
}

void vpu_w_set_fb_addr(unsigned int y_addr,unsigned int c_addr)
{
	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n",y_addr,c_addr);
	vppif_reg32_out(REG_VPU_W_Y1SA,y_addr);
	vppif_reg32_out(REG_VPU_W_C1SA,c_addr);
}

void vpu_w_get_fb_addr(unsigned int *y_addr,unsigned int *c_addr)
{
	*y_addr = vppif_reg32_in(REG_VPU_W_Y1SA);
	*c_addr = vppif_reg32_in(REG_VPU_W_C1SA);
	DBGMSG("y_addr:0x%08x, c_addr:0x%08x\n", *y_addr, *c_addr);
}

void vpu_w_set_fb_width(unsigned int width,unsigned int buf_width)
{
	vppif_reg32_write(VPU_W_YPXLWID, width);
	vppif_reg32_write(VPU_W_YBUFWID, buf_width);
	if( vpu_w_get_color_format() == VDO_COL_FMT_YUV444 ){
		vppif_reg32_write(VPU_W_CPXLWID, width);
		vppif_reg32_write(VPU_W_CBUFWID, buf_width * 2);
	}
	else {
		vppif_reg32_write(VPU_W_CPXLWID, width / 2);
		vppif_reg32_write(VPU_W_CBUFWID, buf_width);
	}
}

void vpu_w_get_fb_width(unsigned int *width,unsigned int *buf_width)
{
	*width = vppif_reg32_read(VPU_W_YPXLWID);
	*buf_width = vppif_reg32_read(VPU_W_YBUFWID);
}

void vpu_w_set_framebuffer(vdo_framebuf_t *fb)
{
	vdo_framebuf_t *scl_fb;
	vpp_clock_t timing;
	unsigned int pixel_clock;

	vpu_w_set_fb_addr(fb->y_addr, fb->c_addr);
	vpu_w_set_color_format(fb->col_fmt);
	vpu_w_set_fb_width(fb->img_w, fb->fb_w);

	scl_fb = &p_scl->fb_p->fb;
	vpu_set_scale(scl_fb->img_w, scl_fb->img_h, fb->img_w, fb->img_h);
	
	p_scl->fb_p->set_csc(p_scl->fb_p->csc_mode);
			
	// scl TG
	timing.total_pixel_of_line = (fb->img_w > scl_fb->img_w)?fb->img_w:scl_fb->img_w;
	timing.total_line_of_frame = (fb->img_h> scl_fb->img_h)?fb->img_h:scl_fb->img_h;

	timing.begin_pixel_of_active = 100;
	timing.end_pixel_of_active = timing.total_pixel_of_line + 100;
	timing.total_pixel_of_line = timing.total_pixel_of_line + 200;
	timing.begin_line_of_active = 8;
	timing.end_line_of_active = timing.total_line_of_frame + 8;
	timing.total_line_of_frame = timing.total_line_of_frame + 16;
	timing.line_number_between_VBIS_VBIE = 4;
	timing.line_number_between_PVBI_VBIS = 10;
	pixel_clock = timing.total_pixel_of_line * timing.total_line_of_frame * p_scl->fb_p->framerate;
	timing.read_cycle = vpp_set_base_clock(VPP_MOD_VPU,pixel_clock);
	vpu_set_timing(&timing);

}
/*----------------------- VPU deinterlace --------------------------------------*/
void vpu_dei_set_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_DEI_ENABLE,enable);
	vppif_reg32_write(VPU_DEI_SUM_UPDATE_EN,enable);
}

void vpu_dei_set_detect_mode(int detect_yc)
{
	/* 	0: Y only Diff = Diff(Y)
		1: Y & C, Diff = min(Diff(Y),Diff(C))
	*/

	vppif_reg32_write(VPU_DEI_DETECT_MODE,detect_yc);
}

void vpu_dei_set_mode(vpp_deinterlace_t mode)
{
	int enable;
	unsigned int yaddr,caddr;

	/* 	0:Weave(ByPass) : Top & Bottom field bypass
		1:Bob : Top field duplicate
		2:Adaptive one : Top & Bottom diff one direction (Diff = Diff(D1))
		3:Adaptive three : Top & Bottom diff three direction (Diff = min(Diff(D1),Diff(D2),Diff(D3)))
		4:Field deinterlace : Top field diff five direction 
		5:Motion vector deinterlace : Motion vector table create by VDU
	*/
	enable = (mode >= VPP_DEI_MAX)? VPP_FLAG_DISABLE:VPP_FLAG_ENABLE;
	if( (enable==VPP_FLAG_DISABLE) && (vppif_reg32_read(VPU_DROP_LINE)) ){
		vpu_r_set_mif2_enable(VPP_FLAG_DISABLE);
	}
	else {
		vpu_r_set_mif2_enable(VPP_FLAG_ENABLE);
	}
	vpu_dei_set_enable(enable);
//	vppif_reg32_write(VPU_R_MIF2_ENABLE,enable);
	vppif_reg32_write(VPU_R_FIELD_MODE,(mode == VPP_DEI_FIELD)? 1:0);
	vppif_reg32_out(REG_VPU_R_CROP2,(vppif_reg32_read(VPU_R_HCROP2) + ((mode == VPP_DEI_FIELD)? 0x20000:0x10000)));
	vppif_reg32_out(REG_SW_FIELD,(!enable || mode == VPP_DEI_WEAVE)? 0x0:0x1);	// sw field, top field
	vppif_reg32_write(VPU_DEI_MODE,mode);
	vpu_r_get_fb_addr(&yaddr,&caddr);
	vpu_r_set_fb_addr(yaddr,caddr);
}

void vpu_dei_set_range(int level,int value)
{
	int reg_offset = level / 4;
	int range_offset = level % 4;

	/* 
		Alpha_A = level, Alpha_B = 0x10 - level 
		if( diff < value ){
			pixel = (pre_pixel * Alpha_A) + (pixel * Alpha_B);
		}
	*/

	vppif_reg32_write(REG_DEI_RANGE_A + (4*reg_offset),(0xFF << (8*range_offset)),(8*range_offset),value);
}

void vpu_dei_set_sum_update(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_DEI_SUM_UPDATE_EN,enable);
}

void vpu_dei_get_sum(unsigned int *ysum,unsigned int *usum,unsigned int *vsum)
{
	*ysum = vppif_reg32_read(VPU_DEI_Y_SUM);
	*usum = vppif_reg32_read(VPU_DEI_U_SUM);
	*vsum = vppif_reg32_read(VPU_DEI_V_SUM);
}

/*----------------------- VPU MV --------------------------------------*/
void vpu_mvr_set_enable(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_MVR_MIF_ENABLE,enable);
}

void vpu_mvr_set_field_mode(vpp_flag_t enable)
{
	vppif_reg32_write(VPU_MVR_SRC_FMT,enable);	// 1-16 pixel,0-8 pixel
}

void vpu_mvr_set_media_format(vpp_media_format_t format)
{
	switch (format) {
		case VPP_MEDIA_FMT_MPEG:
			vppif_reg32_write(VPU_MVR_H264_FMT, 0x0);
			break;
		case VPP_MEDIA_FMT_H264:
			vppif_reg32_write(VPU_MVR_H264_FMT, 0x1);
			break;
		default:
			DBGMSG("*E* check the parameter.\n");
			return;
	}
}

void vpu_mvr_set_addr(unsigned int addr)
{
	vppif_reg32_out(REG_MVR_YSA,addr);
}

void vpu_mvr_set_width(unsigned int width,unsigned int fb_w)
{
	vppif_reg32_write(VPU_MVR_LNSIZE,width);
	vppif_reg32_write(VPU_MVR_FBW,fb_w/128);
}

/*----------------------- VPU INTERRUPT --------------------------------------*/
void vpu_int_set_enable(vpp_flag_t enable, vpp_int_t int_bit)
{
	//clean status first before enable/disable interrupt
	vpu_int_clean_status(int_bit);

	if (int_bit & VPP_INT_ERR_VPU_TG) {
		vppif_reg32_write(VPU_TG_ERR_EN, enable);
	}
	if (int_bit & VPP_INT_ERR_VPUR1_MIF) {
		vppif_reg32_write(VPU_R_MIF1_ERR_EN, enable);
	}
	if (int_bit & VPP_INT_ERR_VPUR2_MIF) {
		vppif_reg32_write(VPU_R_MIF2_ERR_EN, enable);
	}
	if (int_bit & VPP_INT_ERR_VPUW_MIFRGB) {
		vppif_reg32_write(VPU_W_MIF_RGB_ERR_EN, enable);
	}
	if (int_bit & VPP_INT_ERR_VPUW_MIFY) {
		vppif_reg32_write(VPU_W_MIF_Y_ERR_EN, enable);
	}
	if (int_bit & VPP_INT_ERR_VPUW_MIFC) {
		vppif_reg32_write(VPU_W_MIF_C_ERR_EN, enable);
	}
}

vpp_int_err_t vpu_int_get_status(void)
{
	vpp_int_err_t int_sts;

	int_sts = 0;
	if (vppif_reg32_read(VPU_INTSTS_TGERR))
		int_sts |= VPP_INT_ERR_VPU_TG;
	if (vppif_reg32_read(VPU_R_INTSTS_R1MIFERR))
		int_sts |= VPP_INT_ERR_VPUR1_MIF;
//	if (vppif_reg32_read(VPU_R_INTSTS_R2MIFERR))
//		int_sts |= VPP_INT_ERR_VPUR2_MIF;
	if (vppif_reg32_read(VPU_W_MIF_YERR))
		int_sts |= VPP_INT_ERR_VPUW_MIFY;
	if (vppif_reg32_read(VPU_W_MIF_CERR))
		int_sts |= VPP_INT_ERR_VPUW_MIFC;
	
	return int_sts;
}

void vpu_int_clean_status(vpp_int_err_t int_sts)
{
	if (int_sts & VPP_INT_ERR_VPU_TG) {
		vppif_reg32_out(REG_VPU_TG_STATUS+0x0,BIT0);
	}
	if (int_sts & VPP_INT_ERR_VPUR1_MIF) {
		vppif_reg8_out(REG_VPU_R_FF_CTL+0x1,BIT0);
	}
//	if (int_sts & VPP_INT_ERR_VPUR2_MIF) {
//		vppif_reg8_out(REG_SCLR_FIFO_CTL+0x1,BIT1);
//	}
	if (int_sts & VPP_INT_ERR_VPUW_MIFY) {
		vppif_reg32_out(REG_VPU_W_FF_CTL+0x0,BIT8);
	}
	if (int_sts & VPP_INT_ERR_VPUW_MIFC) {
		vppif_reg32_out(REG_VPU_W_FF_CTL+0x0,BIT0);
	}
}

void vpu_reg_dump(void)
{
	int i;

	DPRINT("========== VPU register dump ==========\n");
	for(i=0;i<0xf4;i+=16){
		DPRINT("0x%8x : 0x%08x 0x%08x 0x%08x 0x%08x\n",VPU_BASE_ADDR+i,vppif_reg32_in(VPU_BASE_ADDR+i),
			vppif_reg32_in(VPU_BASE_ADDR+i+4),vppif_reg32_in(VPU_BASE_ADDR+i+8),vppif_reg32_in(VPU_BASE_ADDR+i+12));
	}

	for(i=0;i<0x50;i+=16){
		DPRINT("0x%8x : 0x%08x 0x%08x 0x%08x 0x%08x\n",VPU_BASE2_ADDR+i,vppif_reg32_in(VPU_BASE2_ADDR+i),
			vppif_reg32_in(VPU_BASE2_ADDR+i+4),vppif_reg32_in(VPU_BASE2_ADDR+i+8),vppif_reg32_in(VPU_BASE2_ADDR+i+12));
	}

	DPRINT("---------- VPU common ----------\n");
	DPRINT("vpu enable %d,update %d,level %d\n",vppif_reg32_read(VPU_SCALAR_ENABLE),
						vppif_reg32_read(VPU_REG_UPDATE),vppif_reg32_read(VPU_REG_READ_SEL));

	DPRINT("TG err %d,INT en %d\n",vppif_reg32_read(VPU_INTSTS_TGERR),vppif_reg32_read(VPU_TG_ERR_EN));
	DPRINT("R1 MIF err %d,INT en %d\n",vppif_reg32_read(VPU_R_INTSTS_R1MIFERR),vppif_reg32_read(VPU_R_MIF1_ERR_EN));
	DPRINT("R2 MIF err %d,INT en %d\n",vppif_reg32_read(VPU_R_INTSTS_R1MIFERR),vppif_reg32_read(VPU_R_MIF2_ERR_EN));
	DPRINT("W MIF Y err %d,INT en %d\n",vppif_reg32_read(VPU_W_MIF_YERR),vppif_reg32_read(VPU_W_MIF_Y_ERR_EN));
	DPRINT("W MIF C err %d,INT en %d\n",vppif_reg32_read(VPU_W_MIF_CERR),vppif_reg32_read(VPU_W_MIF_C_ERR_EN));
	DPRINT("W MIF RGB err %d,INT en %d\n",vppif_reg32_read(VPU_W_MIF_CERR),vppif_reg32_read(VPU_W_MIF_RGB_ERR_EN));
	
	DPRINT("---------- VPU scale ----------\n");
	DPRINT("H scale %s,V scale %s\n",(vppif_reg32_read(VPU_H_SCALEUP))?"UP":"DOWN",
									(vppif_reg32_read(VPU_V_SCALEUP))?"UP":"DOWN");
	DPRINT("init %d,return %d\n",vppif_reg32_read(VPU_HPTR_INIT),vppif_reg32_read(VPU_HPTR_RETURN));
	DPRINT("width H %d, V %d\n",vppif_reg32_read(VPU_HXWIDTH),vppif_reg32_read(VPU_VXWIDTH));
	DPRINT("drop line %d\n",vppif_reg32_read(VPU_DROP_LINE));

	DPRINT("---------- VPU TG ----------\n");
	DPRINT("TG source : %s,error status %d\n",(vppif_reg32_read(VPU_TG_GOVWTG_ENABLE))?"GOVW":"VPU",
					vppif_reg32_read(VPU_INTSTS_TGERR));
	DPRINT("TG enable %d,wait ready %d,wait ready cnt %d\n",vppif_reg32_read(VPU_TG_ENABLE),
					vppif_reg32_read(VPU_TG_WAITRDY_ENABLE),vppif_reg32_read(VPU_TG_WAITRDY_CNT));
	DPRINT("rd cyc %d\n",vppif_reg32_read(VPU_TG_RDCYC));
	DPRINT("H pixel %d,V line %d\n",vppif_reg32_read(VPU_TG_H_ALLPIXEL),vppif_reg32_read(VPU_TG_V_ALLLINE));
	DPRINT("H beg %d,end %d\n",vppif_reg32_read(VPU_TG_H_ACTBG),vppif_reg32_read(VPU_TG_H_ACTEND));
	DPRINT("V beg %d,end %d\n",vppif_reg32_read(VPU_TG_V_ACTBG),vppif_reg32_read(VPU_TG_V_ACTEND));
	DPRINT("pvbi %d,vbie %d\n",vppif_reg32_read(VPU_TG_PVBI),vppif_reg32_read(VPU_TG_VBIE));

	DPRINT("---------- VPU R MIF ----------\n");
	DPRINT("VPU MIF enable %d,MIF2 enable %d\n",vppif_reg32_read(VPU_R_MIF_ENABLE),vppif_reg32_read(VPU_R_MIF2_ENABLE));
	switch(vpu_r_get_color_format()){
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
	DPRINT("field %d,h264 %d,420C field %d\n",vppif_reg32_read(VPU_R_FIELD_MODE),
					vppif_reg32_read(VPU_R_MEDIAFMT_H264),vppif_reg32_read(VPU_R_420C_FMT));
	DPRINT("color bar %d,mode %d,inverse %d\n",vppif_reg32_read(VPU_R_COLBAR_ENABLE),
					vppif_reg32_read(VPU_R_COLBAR_MODE),vppif_reg32_read(VPU_R_COLBAR_INVERSION));
	DPRINT("Y1 0x%x,C1 0x%x\n",vppif_reg32_in(REG_VPU_R_Y1SA),vppif_reg32_in(REG_VPU_R_C1SA));
	DPRINT("Y2 0x%x,C2 0x%x\n",vppif_reg32_in(REG_VPU_R_Y2SA),vppif_reg32_in(REG_VPU_R_C2SA));
	DPRINT("pixel %d,fb width %d\n",vppif_reg32_read(VPU_R_YPXLWID),vppif_reg32_read(VPU_R_YBUFWID));
	DPRINT("crop(%d,%d),fifo thr %d\n",vppif_reg32_read(VPU_R_HCROP),vppif_reg32_read(VPU_R_VCROP),
					vppif_reg32_read(VPU_R_FIFO_THR));

	DPRINT("---------- VPU W MIF ----------\n");
	DPRINT("VPU MIF enable %d\n",vppif_reg32_read(VPU_W_MIF_EN));
	switch(vpu_w_get_color_format()){
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
	DPRINT("Y1 0x%x,C1 0x%x\n",vppif_reg32_in(REG_VPU_W_Y1SA),vppif_reg32_in(REG_VPU_W_C1SA));
	DPRINT("Y pixel %d,fb width %d\n",vppif_reg32_read(VPU_W_YPXLWID),vppif_reg32_read(VPU_W_YBUFWID));
	DPRINT("C pixel %d,fb width %d\n",vppif_reg32_read(VPU_W_CPXLWID),vppif_reg32_read(VPU_W_CBUFWID));

	DPRINT("---------- VPU CSC ----------\n");
	DPRINT("enable %d,Y sub en %d\n",vppif_reg32_read(VPU_CSC_EN),vppif_reg32_read(VPU_Y_SUB_EN));
	DPRINT("mode : %s\n",(vppif_reg32_read(VPU_CSC_MODE))?"YUV to RGB":"RGB to YUV");

	DPRINT("---------- VPU deinterlace ----------\n");
	DPRINT("enable %d,detect YC %d\n",vppif_reg32_read(VPU_DEI_ENABLE),vppif_reg32_read(VPU_DEI_DETECT_MODE));
	switch(vppif_reg32_read(VPU_DEI_MODE)){
		case 0: DPRINT("Weave(ByPass) : Top & Bottom field bypass\n"); break;
		case 1: DPRINT("Bob : Top field duplicate\n"); break;
		case 2: DPRINT("Adaptive one : Top & Bottom diff one direction (Diff = Diff(D1))\n"); break;
		case 3: DPRINT("Adaptive three : Top & Bottom diff three direction (Diff = min(Diff(D1),Diff(D2),Diff(D3)))\n"); break;
		case 4: DPRINT("Field deinterlace : Top field diff five direction\n"); break;
		case 5: DPRINT("Motion vector deinterlace : Motion vector table create by VDU\n"); break;
		default: break;
	}
	
	DPRINT("sum update enable %d\n",vppif_reg32_read(VPU_DEI_SUM_UPDATE_EN));
	DPRINT("sum Y %d,U %d,V %d\n",vppif_reg32_read(VPU_DEI_Y_SUM),vppif_reg32_read(VPU_DEI_U_SUM),vppif_reg32_read(VPU_DEI_V_SUM));

	DPRINT("---------- VPU MV ----------\n");
	DPRINT("MVR MIF enable %d\n",vppif_reg32_read(VPU_MVR_MIF_ENABLE));
	DPRINT("src fmt %d,H264 %d\n",vppif_reg32_read(VPU_MVR_SRC_FMT),vppif_reg32_read(VPU_MVR_H264_FMT));
	DPRINT("line size %d,fb width %d\n",vppif_reg32_read(VPU_MVR_LNSIZE),vppif_reg32_read(VPU_MVR_FBW));
}

void vpu_proc_view(int read,vdo_view_t *view)
{
	vdo_framebuf_t *fb;

	fb = &p_vpu->fb_p->fb;
	if( read ){
		view->resx_src = fb->fb_w;
		view->resy_src = fb->fb_h;
		view->resx_virtual = fb->img_w;
		view->resy_virtual = fb->img_h;
		view->resx_visual = p_vpu->resx_visual;
		view->resy_visual = p_vpu->resy_visual;
		view->posx = p_vpu->posx;
		view->posy = p_vpu->posy;
		view->offsetx = fb->h_crop;
		view->offsety = fb->v_crop;
	}
	else {
		vpp_set_video_scale(view);
	}
}

int vpu_proc_scale(vdo_framebuf_t *src_fb,vdo_framebuf_t *dst_fb)
{
	int i;
	int ret = 0;
	int rd_cyc;

	// scl mode
	govw_set_tg_enable(VPP_FLAG_DISABLE);
	vpu_set_timing_master(VPP_MOD_VPU);

	p_scl->fb_p->fb = *src_fb;
	p_scl->fb_p->set_framebuf(src_fb);

	p_sclw->fb_p->fb = *dst_fb;		
	p_sclw->fb_p->set_framebuf(dst_fb);

	// scale process
	vpu_w_set_mif_enable(VPP_FLAG_ENABLE);
	vpu_set_tg_enable(VPP_FLAG_ENABLE);
	vppif_reg32_out(REG_VPU_TG_STATUS+0x0,BIT0);
	do {
		// wait ready
		vppm_set_int_enable(VPP_FLAG_ENABLE,VPP_INT_VPU_VBIE);	
		for(i=0;i<3;i++){
			vpp_proc_func(0,0,VPP_INT_VPU_VBIE,1);
		}
		vppm_set_int_enable(VPP_FLAG_DISABLE,VPP_INT_VPU_VBIE);

		// check scale status
		if( vppif_reg32_read(VPU_INTSTS_TGERR) == 0 ){
			break;
		}

		// add rd_cyc to retry
		rd_cyc = vppif_reg32_read(VPU_TG_RDCYC);
		if( rd_cyc > 0xFD ){
			printk("[VPU] *E* scale error\n");
			ret = -1;
			break;
		}
		rd_cyc = rd_cyc + (0xFF - rd_cyc) / 2;
		if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
			printk("[VPU] scale retry, rcyc 0x%x\n",rd_cyc);
		}
		vppif_reg32_write(VPU_TG_RDCYC,rd_cyc);
		vppif_reg32_out(REG_VPU_TG_STATUS+0x0,BIT0);
	} while(1);

	// recover to vpu mode
	vpu_w_set_mif_enable(VPP_FLAG_DISABLE);
	vpu_set_timing_master(VPP_MOD_GOVW);
	vpu_set_tg_enable(VPP_FLAG_DISABLE);
	p_vpu->fb_p->set_framebuf(&p_vpu->fb_p->fb);
	govw_set_tg_enable(VPP_FLAG_ENABLE);
	return ret;
}

#ifdef CONFIG_PM
static unsigned int *vpu_pm_bk2;
static unsigned int vpu_pm_enable,vpu_pm_tg;
static unsigned int vpu_pm_r_mif1,vpu_pm_r_mif2,vpu_pm_w_mif;
void vpu_suspend(int sts)
{
	switch( sts ){
		case 0:	// disable module
			vpu_pm_enable = vppif_reg32_read(VPU_SCALAR_ENABLE);
			vppif_reg32_write(VPU_SCALAR_ENABLE,0);
			vpu_pm_r_mif1 = vppif_reg32_read(VPU_R_MIF_ENABLE);
			vpu_pm_r_mif2 = vppif_reg32_read(VPU_R_MIF2_ENABLE);
			vpu_pm_w_mif = vppif_reg32_read(VPU_W_MIF_EN);
			vppif_reg32_write(VPU_W_MIF_EN, 0);
			vppif_reg32_write(VPU_R_MIF_ENABLE, 0);
			vppif_reg32_write(VPU_R_MIF2_ENABLE, 0);
			break;
		case 1: // disable tg
			vpu_pm_tg = vppif_reg32_read(VPU_TG_ENABLE);
			vppif_reg32_write(VPU_TG_ENABLE,0);
			break;
		case 2:	// backup register
			p_vpu->reg_bk = vpp_backup_reg(VPU_BASE_ADDR+0x00,0xF8); /* 0x00 - 0xF8 */
			vpu_pm_bk2 = vpp_backup_reg(VPU_BASE2_ADDR+0x00,0x40); /* 0x00 - 0x40 */
			break;
		default:
			break;
	}
}

void vpu_resume(int sts)
{
	switch( sts ){
		case 0:	// restore register
			vpp_restore_reg(VPU_BASE_ADDR+0x00,0xF8,p_vpu->reg_bk); /* 0x00 - 0xF8 */
			vpp_restore_reg(VPU_BASE2_ADDR+0x00,0x40,vpu_pm_bk2); /* 0x00 - 0x40 */
			p_vpu->reg_bk = 0;
			vpu_pm_bk2 = 0;
			break;
		case 1:	// enable module
			vppif_reg32_write(VPU_W_MIF_EN,vpu_pm_w_mif);
			vppif_reg32_write(VPU_R_MIF_ENABLE, vpu_pm_r_mif1);
			vppif_reg32_write(VPU_R_MIF2_ENABLE, vpu_pm_r_mif2);
			vppif_reg32_write(VPU_SCALAR_ENABLE,vpu_pm_enable);
			break;
		case 2: // enable tg
			vppif_reg32_write(VPU_TG_ENABLE,vpu_pm_tg);
			break;
		default:
			break;
	}
}
#else
#define vpu_suspend NULL
#define vpu_resume NULL
#endif

/*----------------------- VPU INIT --------------------------------------*/
void vpu_init(void *base)
{
	vpu_mod_t *mod_p;
	vpp_fb_base_t *fb_p;	

	mod_p = (vpu_mod_t *) base;
	fb_p = mod_p->fb_p;

	vpu_set_reg_read_select(VPP_REG_LEVEL_1);
	vpu_set_tg_enable(VPP_FLAG_DISABLE);
	vpu_set_enable(VPP_FLAG_DISABLE);
	vpu_int_set_enable(VPP_FLAG_DISABLE, VPP_INT_ALL);
	vpu_r_set_mif_enable(VPP_FLAG_DISABLE);
	vpu_r_set_mif2_enable(VPP_FLAG_DISABLE);
	vpu_r_set_colorbar(VPP_FLAG_DISABLE,0,0);

	vpu_int_set_enable(VPP_FLAG_ENABLE, mod_p->int_catch);
	vpu_set_wait_ready(fb_p->wait_ready);
	vpu_r_set_media_format(fb_p->media_fmt);
	vpu_r_set_threshold(0xf);
	vpu_set_drop_line(VPP_FLAG_DISABLE);

	vpu_r_set_framebuffer(&mod_p->fb_p->fb);
	vpu_set_timing_master(VPP_MOD_GOVW);
	// dei range value
#if 1	// motion vector mode
	vppif_reg32_out(REG_DEI_RANGE_A,0x0);
	vppif_reg32_out(REG_DEI_RANGE_B,0x01);
	vppif_reg32_out(REG_DEI_RANGE_C,0x20000);
	vppif_reg32_out(REG_DEI_RANGE_D,0x0);
#endif
#if 0	// dynamic deinterlace mode
	vppif_reg32_out(REG_DEI_RANGE_A,0x0);
	vppif_reg32_out(REG_DEI_RANGE_B,0x0);
	vppif_reg32_out(REG_DEI_RANGE_C,0x40);
	vppif_reg32_out(REG_DEI_RANGE_D,0xb0a09080);
#endif	
	vpu_dei_set_mode(mod_p->dei_mode);

	vpu_r_set_mif_enable(VPP_FLAG_ENABLE);
	vpu_set_enable(VPP_FLAG_ENABLE);
	vpu_set_reg_update(VPP_FLAG_ENABLE);
	vpu_set_tg_enable(VPP_FLAG_DISABLE);
}

void vpu_scl_init(void *base)
{

}

void vpu_sclw_init(void *base)
{
	vpu_w_set_mif_enable(VPP_FLAG_DISABLE);
	vpu_w_set_fb_width(VPP_HD_DISP_RESX, VPP_HD_MAX_RESX);
}

int vpu_mod_init(void)
{
	vpp_fb_base_t *mod_fb_p;
	vdo_framebuf_t *fb_p;

	/* -------------------- VPU module -------------------- */
	{
		vpu_mod_t *vpu_mod_p;

		vpu_mod_p = (vpu_mod_t *) vpp_mod_register(VPP_MOD_VPU,sizeof(vpu_mod_t),VPP_MOD_FLAG_FRAMEBUF);
		if( !vpu_mod_p ){
			DPRINT("*E* VPU module register fail\n");
			return -1;
		}

		/* module member variable */
		vpu_mod_p->int_catch = VPP_INT_NULL;	
		vpu_mod_p->resx_visual = VPP_HD_DISP_RESX;
		vpu_mod_p->resy_visual = VPP_HD_DISP_RESY;
		vpu_mod_p->posx = 0;
		vpu_mod_p->posy = 0;
		vpu_mod_p->dei_mode = VPP_DEI_MOTION_VECTOR; // VPP_DEI_DYNAMIC;

		/* module member function */
		vpu_mod_p->init = vpu_init;
		vpu_mod_p->dump_reg = vpu_reg_dump;
		vpu_mod_p->set_enable = vpu_set_enable;
		vpu_mod_p->set_colorbar = vpu_r_set_colorbar;
		vpu_mod_p->get_sts = vpu_int_get_status;
		vpu_mod_p->clr_sts = vpu_int_clean_status;
		vpu_mod_p->suspend = vpu_suspend;
		vpu_mod_p->resume = vpu_resume;

		/* module frame buffer */
		mod_fb_p = vpu_mod_p->fb_p;
		mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_JFIF_0_255;	
		mod_fb_p->framerate = 30;
		mod_fb_p->media_fmt = VPP_MEDIA_FMT_MPEG;
		mod_fb_p->wait_ready = 0x1000;
		mod_fb_p->capability = BIT(VDO_COL_FMT_YUV420) | BIT(VDO_COL_FMT_YUV422H) | BIT(VDO_COL_FMT_YUV444) | BIT(VDO_COL_FMT_ARGB)
								| VPP_FB_FLAG_CSC | VPP_FB_FLAG_MEDIA | VPP_FB_FLAG_FIELD;
		
		fb_p = &vpu_mod_p->fb_p->fb;
		fb_p->y_addr = 0;
		fb_p->c_addr = 0;
		fb_p->col_fmt = VDO_COL_FMT_YUV422H;
		fb_p->img_w = VPP_HD_DISP_RESX;
		fb_p->img_h = VPP_HD_DISP_RESY;
		fb_p->fb_w = VPP_HD_MAX_RESX;
		fb_p->fb_h = VPP_HD_MAX_RESY;	
		fb_p->h_crop = 0;
		fb_p->v_crop = 0;
		fb_p->flag = 0;

		/* module frame buffer member function */
		mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_JFIF_0_255;
		mod_fb_p->set_framebuf = vpu_r_set_framebuffer;
		mod_fb_p->set_addr = vpu_r_set_fb_addr;
		mod_fb_p->set_csc = vpu_set_csc_mode;
		mod_fb_p->fn_view = vpu_proc_view;

		p_vpu = vpu_mod_p;
	}

#ifndef WMT_FTBLK_SCL
	/* -------------------- SCL module -------------------- */
	{
		scl_mod_t *scl_mod_p;
		
		scl_mod_p = (scl_mod_t *) vpp_mod_register(VPP_MOD_SCL,sizeof(scl_mod_t),VPP_MOD_FLAG_FRAMEBUF);
		if( !scl_mod_p ){
			DPRINT("*E* SCL module register fail\n");
			return -1;
		}

		/* module member variable */
		scl_mod_p->int_catch = VPP_INT_NULL;

		/* module member function */
		scl_mod_p->init = vpu_scl_init;
		scl_mod_p->set_enable = vpu_set_enable;
		scl_mod_p->set_colorbar = vpu_r_set_colorbar;
		scl_mod_p->scale = vpu_proc_scale;

		/* module frame buffer variable */
		mod_fb_p = scl_mod_p->fb_p;
		fb_p = &mod_fb_p->fb;

		fb_p->y_addr = 0;
		fb_p->c_addr = 0;
		fb_p->col_fmt = VDO_COL_FMT_YUV422H;
		fb_p->img_w = VPP_HD_DISP_RESX;
		fb_p->img_h = VPP_HD_DISP_RESY;
		fb_p->fb_w = VPP_HD_MAX_RESX;
		fb_p->fb_h = VPP_HD_MAX_RESY;
		fb_p->h_crop = 0;
		fb_p->v_crop = 0;

		/* module frame buffer member function */
		mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_JFIF_0_255;
		mod_fb_p->set_framebuf = vpu_r_set_framebuffer;
		mod_fb_p->set_addr = vpu_r_set_fb_addr;
		mod_fb_p->get_addr = vpu_r_get_fb_addr;
		mod_fb_p->set_csc = vpu_set_csc_mode;
		mod_fb_p->framerate = 10;
		mod_fb_p->wait_ready = 0x1fff;
		mod_fb_p->capability = BIT(VDO_COL_FMT_YUV420) | BIT(VDO_COL_FMT_YUV422H) | BIT(VDO_COL_FMT_YUV444) 
			| BIT(VDO_COL_FMT_ARGB) | VPP_FB_FLAG_CSC | VPP_FB_FLAG_FIELD;

		p_scl = scl_mod_p;
	}
	
	/* -------------------- SCLW module -------------------- */
	{
		sclw_mod_t *sclw_mod_p;
		
		sclw_mod_p = (sclw_mod_t *) vpp_mod_register(VPP_MOD_SCLW,sizeof(sclw_mod_t),VPP_MOD_FLAG_FRAMEBUF);
		if( !sclw_mod_p ){
			DPRINT("*E* SCLW module register fail\n");
			return -1;
		}

		/* module member variable */
		sclw_mod_p->int_catch = VPP_INT_NULL;

		/* module member function */
		sclw_mod_p->init = vpu_sclw_init;
		sclw_mod_p->set_enable = vpu_w_set_mif_enable;

		/* module frame buffer */
		mod_fb_p = sclw_mod_p->fb_p;
		fb_p = &mod_fb_p->fb;

		fb_p->y_addr = 0;
		fb_p->c_addr = 0;
		fb_p->col_fmt = VDO_COL_FMT_YUV422H;
		fb_p->img_w = VPP_HD_DISP_RESX;
		fb_p->img_h = VPP_HD_DISP_RESY;
		fb_p->fb_w = VPP_HD_MAX_RESX;
		fb_p->fb_h = VPP_HD_MAX_RESY;
		fb_p->h_crop = 0;
		fb_p->v_crop = 0;

		/* module frame buffer member function */
		mod_fb_p->csc_mode = VPP_CSC_RGB2YUV_JFIF_0_255;
		mod_fb_p->set_framebuf = vpu_w_set_framebuffer;
		mod_fb_p->set_addr = vpu_w_set_fb_addr;
		mod_fb_p->get_addr = vpu_w_get_fb_addr;
		mod_fb_p->set_csc = vpu_set_csc_mode;
		mod_fb_p->wait_ready = 0x1fff;
		mod_fb_p->capability = BIT(VDO_COL_FMT_YUV422H) | BIT(VDO_COL_FMT_YUV444) | BIT(VDO_COL_FMT_ARGB)
							| VPP_FB_FLAG_CSC;

		p_sclw = sclw_mod_p;
	}
#endif
	return 0;
}
#ifdef __KERNEL__
module_init(vpu_mod_init);
#endif
#endif                                //WMT_FTBLK_VPU
