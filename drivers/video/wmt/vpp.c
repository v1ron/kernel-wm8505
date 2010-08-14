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


/* PVCS version log
 * $Log: POST/vpp/vppif.c $
 *
 * 01	08/05/15, Andy Chiu, Initial version for vt8440 (pending)
 * 02	08/10/01, Andy Chiu, Refine for vt8510 
 * 03	08/10/15, Andy Chiu, Merge Sam Shen's source of GOVRH 
 *
 */
#define VPPIF_C

#include "vpp.h"

#ifdef __KERNEL__
//#include <asm/uaccess.h>
//#include <asm/io.h>
#define DPRINT printk

#else
//#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define DPRINT printf

static __inline__ U32 inl(U32 offset)
{
	return REG32_VAL(offset);
}

static __inline__ void outl(U32 val,U32 offset)
{
	REG32_VAL(offset) = val;
}

static __inline__ U16 inw(U32 offset)
{
	return REG16_VAL(offset);
}

static __inline__ void outw(U16 val,U32 offset)
{
	REG16_VAL(offset) = val;
}

static __inline__ U8 inb(U32 offset)
{
	return REG8_VAL(offset);
}

static __inline__ void outb(U8 val,U32 offset)
{
	REG8_VAL(offset) = val;
}

#define kmalloc(a,b) 	malloc(a)
#define kfree(a)		free(a)
#define GFP_KERNEL		0

int get_key(void) 
{
	int key;

	extern int get_num(unsigned int min,unsigned int max,char *message,unsigned int retry);
	key = get_num(0, 256, "Input:", 5);
	DPRINT("\n");
	return key;
}
#endif

#ifdef VPPIF_DEBUG
#define VPPIFMSG(fmt, args...) DPRINT("[VPPIF] %s: " fmt, __FUNCTION__ , ## args)
#else
#define VPPIFMSG(fmt, args...) do {} while(0)
#endif

typedef struct {
	unsigned int pixel_clock;
	unsigned int PLL;
	unsigned int divisor;
	unsigned int rd_cyc;
} vpp_base_clock_t;

vpp_mod_base_t *vpp_mod_base_list[VPP_MOD_MAX];

//Internal functions
U8 vppif_reg8_in(U32 offset)
{
	return (inb(offset));
}

U8 vppif_reg8_out(U32 offset, U8 val)
{
	outb(val, offset);
	return (val);
}

U16 vppif_reg16_in(U32 offset)
{
	return (inw(offset));
}

U16 vppif_reg16_out(U32 offset, U16 val)
{
	outw(val, offset);
	return (val);
}

U32 vppif_reg32_in(U32 offset)
{
	return (inl(offset));
}

U32 vppif_reg32_out(U32 offset, U32 val)
{
	outl(val, offset);
	return (val);
}

U32 vppif_reg32_write(U32 offset, U32 mask, U32 shift, U32 val)
{
	U32 new_val;

#ifdef VPPIF_DEBUG
	if( val > (mask >> shift) ){
		VPPIFMSG("*E* check the parameter 0x%x 0x%x 0x%x 0x%x\n",offset,mask,shift,val);
	}
#endif	

	new_val = (inl(offset) & ~(mask)) | (((val) << (shift)) & mask);
	outl(new_val, offset);
	return (new_val);
}

U32 vppif_reg32_read(U32 offset, U32 mask, U32 shift)
{
	return ((inl(offset) & mask) >> shift);
}

U32 vppif_reg32_mask(U32 offset, U32 mask, U32 shift)
{
	return (mask);
}

void vpp_set_dbg_gpio(int no,int value)
{
	unsigned int mask;

	return;
	mask = 0x1 << no;
	REG32_VAL(0xd8110040) |= mask;
	REG32_VAL(0xd8110080) |= mask;
	if( value == 0xFF ){
		if( REG32_VAL(0xd81100C0) & mask ){
			REG32_VAL(0xd81100C0) &= ~mask;			
		}
		else {
			REG32_VAL(0xd81100C0) |= mask;
		}
	}
	else {
		if( value ){
			REG32_VAL(0xd81100C0) |= mask;
		}
		else {
			REG32_VAL(0xd81100C0) &= ~mask;
		}
	}
}

int vpp_check_dbg_level(vpp_dbg_level_t level)
{
	switch( g_vpp.dbg_msg_level ){
		case VPP_DBGLVL_DISABLE:
			break;
		case VPP_DBGLVL_ALL:
			return 1;
		default:
			if( g_vpp.dbg_msg_level == level ){
				return 1;
			}
			break;
	}
	return 0;
}

void vpp_mod_unregister(vpp_mod_t mod)
{
	vpp_mod_base_t *mod_p;

	if( mod >= VPP_MOD_MAX ){
		return;
	}

	if( !(mod_p = vpp_mod_base_list[mod]) )
		return;
	
	if( mod_p->fb_p ) kfree(mod_p->fb_p);
	kfree(mod_p);
	vpp_mod_base_list[mod] = 0;
}

vpp_mod_base_t *vpp_mod_register(vpp_mod_t mod,int size,unsigned int flags)
{
	vpp_mod_base_t *mod_p;

	if( mod >= VPP_MOD_MAX ){
		return 0;
	}

	if( vpp_mod_base_list[mod] ){
		vpp_mod_unregister(mod);
	}

	mod_p = (void *) kmalloc(size,GFP_KERNEL);
	if( !mod_p ) return 0;

	vpp_mod_base_list[mod] = mod_p;
	memset(mod_p,0,size);
	mod_p->mod = mod;

	if( flags & VPP_MOD_FLAG_FRAMEBUF ){
		if( !(mod_p->fb_p = (void *) kmalloc(sizeof(vpp_fb_base_t),GFP_KERNEL)) ){
			goto error;
		}
		memset(mod_p->fb_p,0,sizeof(vpp_fb_base_t));
	}
//	DPRINT("vpp mod register %d,0x%x,0x%x\n",mod,(int)mod_p,(int)mod_p->fb_p);
	return mod_p;
	
error:
	vpp_mod_unregister(mod);
	DPRINT("vpp mod register NG %d\n",mod);	
	return 0;
}

vpp_mod_base_t *vpp_mod_get_base(vpp_mod_t mod)
{
	if( mod >= VPP_MOD_MAX )
		return 0;

	return vpp_mod_base_list[mod];
}

vpp_fb_base_t *vpp_mod_get_fb_base(vpp_mod_t mod)
{
	vpp_mod_base_t *mod_p;
	mod_p = vpp_mod_get_base(mod);
	if( mod_p )
		return mod_p->fb_p;

	return 0;
}

vdo_framebuf_t *vpp_mod_get_framebuf(vpp_mod_t mod)
{
	vpp_mod_base_t *mod_p;

	mod_p = vpp_mod_get_base(mod);
	if( mod_p && mod_p->fb_p )
		return &mod_p->fb_p->fb;

	return 0;
}

void vpp_mod_set_timing(vpp_mod_t mod,vpp_timing_t *tmr)
{
	vdo_framebuf_t *fb;
	vpp_clock_t clk;

	vpp_trans_timing(mod,tmr,&clk,1);
#ifdef WMT_FTBLK_GOVRH	
	clk.read_cycle = vpp_set_base_clock(VPP_MOD_GOVRH, tmr->pixel_clock);
	p_govrh->set_tg(&clk);

	p_govrh->fb_p->fb.img_w = tmr->hpixel;
	p_govrh->fb_p->fb.img_h = tmr->vpixel;
	p_govrh->fb_p->framerate = tmr->pixel_clock / (tmr->hpixel * tmr->vpixel);
	p_govrh->fb_p->set_framebuf(&p_govrh->fb_p->fb);
	p_govrh->vga_dac_sense_cnt = p_govrh->fb_p->framerate * VPP_DAC_SENSE_SECOND;
#endif

#ifdef WMT_FTBLK_LCDC
	clk.read_cycle = vpp_set_base_clock(VPP_MOD_LCDC, tmr->pixel_clock);
	p_lcdc->set_tg(&clk);
	p_lcdc->fb_p->fb.img_w = tmr->hpixel;
	p_lcdc->fb_p->fb.img_h = tmr->vpixel;
	p_lcdc->fb_p->framerate = tmr->pixel_clock / (tmr->hpixel * tmr->vpixel);
	p_lcdc->fb_p->set_framebuf(&p_lcdc->fb_p->fb);
#endif
	
	p_govw->fb_p->fb.img_w = tmr->hpixel;
	p_govw->fb_p->fb.img_h = tmr->vpixel;
	p_govw->fb_p->set_framebuf(&p_govw->fb_p->fb);
	
	if( g_vpp.direct_path ){
		govw_set_tg_enable(VPP_FLAG_DISABLE);
	}

	fb = &p_vpu->fb_p->fb;
	if( fb->y_addr ){ 
		vdo_view_t view;
		
		p_vpu->resx_visual = (fb->fb_w > tmr->hpixel)? tmr->hpixel:fb->fb_w;
		p_vpu->resy_visual = (fb->fb_h > tmr->vpixel)? tmr->vpixel:fb->fb_h;

		view.resx_src = fb->fb_w;
		view.resy_src = fb->fb_h;
		view.resx_virtual = fb->img_w;
		view.resy_virtual = fb->img_h;
		view.resx_visual = p_vpu->resx_visual;
		view.resy_visual = p_vpu->resy_visual;
		view.posx = p_vpu->posx;
		view.posy = p_vpu->posy;
		view.offsetx = fb->h_crop;
		view.offsety = fb->v_crop;
		vpp_set_video_scale(&view);
	}
}

vpp_display_format_t vpp_get_fb_field(vdo_framebuf_t *fb)
{
	if( fb->flag & VDO_FLAG_INTERLACE )
		return VPP_DISP_FMT_FIELD;
	
	return VPP_DISP_FMT_FRAME;
}

void vpp_fill_framebuffer(vdo_framebuf_t *fb,unsigned int x,unsigned int y,unsigned int w,unsigned int h,unsigned int color)
{
	unsigned int line_pxl, fb_pxl, fb_xoft, fb_yoft;
	unsigned int  y_addr, c_addr;
	unsigned char *y_ptr, y_color;
	unsigned short *c_ptr, c_color;
	unsigned int *rgb_ptr;
	unsigned int c_pixel, c_line;
	unsigned int i, j;
	vdo_color_fmt fmt;

//	VPPIFMSG("w %d,h %d,x %d,y %d,color 0x%x\n",w,h,x,y,color);
	y_color = (color & 0xFF0000) >> 16;
	c_color = (color & 0xFFFF);

	fmt = fb->col_fmt;
	fb_pxl = fb->img_w;
	line_pxl = fb->fb_w;
	fb_xoft = fb->h_crop;
	fb_yoft = fb->v_crop;
	y_addr = fb->y_addr;
	c_addr = fb->c_addr;

	switch (fmt) {
	case VDO_COL_FMT_YUV444:
		c_pixel = 1;
		c_line = 1;
		break;
	case VDO_COL_FMT_YUV422H:
		c_pixel = 2;
		c_line = 1;
		break;
	case VDO_COL_FMT_YUV420:
		c_pixel = 2;
		c_line = 2;
		break;
	case VDO_COL_FMT_ARGB:
		rgb_ptr =(unsigned int *) (y_addr + (fb_xoft+x) * 4 + (fb_yoft+y) * line_pxl * 4);
		for (i = 0; i < h; i++) {
			for (j = 0; j < w; j++) {
				*(rgb_ptr + line_pxl * i + j) = color;
			}
		}
		return;
	default:
		return;
	}
	y_ptr =(unsigned char *) (y_addr + (fb_xoft+x) + (fb_yoft+y) * line_pxl);
	c_ptr =(unsigned short *) (c_addr + ((fb_xoft+x) * 2 / c_pixel) + (line_pxl * 2 / c_pixel) * (fb_yoft+y));

#if 0
	VPPIFMSG("w %d,h %d,x %d,y %d,color 0x%x\n",w,h,x,y,color);
	VPPIFMSG("line %d,buf %d,x %d,y %d\n",line_pxl,fb_pxl,fb_xoft,fb_yoft);
	VPPIFMSG("Y ptr 0x%x,addr 0x%x %d\n",y_ptr,y_addr);
	VPPIFMSG("C ptr 0x%x,addr 0x%x %d\n",c_ptr,c_addr);
	VPPIFMSG("fb pixel %d, fmt %d, c pixel %d\n",fb_pxl,fmt,c_pixel);
#endif	
	/* Y */
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			*(y_ptr + (i * line_pxl) + j) = y_color;
		}
	}

	/* CbCr */
	for (i = 0; i < (h / c_line); i++) {
		for (j = 0; j < (w / c_pixel); j++) {
			*(c_ptr + (i * (line_pxl / (c_pixel))) + j) = c_color;
		}
	}
}

void vpp_fill_pattern(vpp_mod_t mod,int no,int arg)
{
	vpp_mod_base_t *mod_p;
	vdo_framebuf_t *fb;
	int i,level;
	unsigned int color,resx,resy;

	mod_p = vpp_mod_get_base(mod);	
	fb = vpp_mod_get_framebuf(mod);
	if( no != VPP_PATTERN_HW_COLORBAR ){
		if( mod_p->set_colorbar ){
			mod_p->set_colorbar(VPP_FLAG_DISABLE,0,0);
		}
	}
	
	switch(no){
		case VPP_PATTERN_HW_COLORBAR:	// hw color bar
			if( mod_p->set_colorbar ){
				mod_p->set_colorbar(VPP_FLAG_ENABLE,arg,0);
			}
			break;
		case VPP_PATTERN_BLACK:	/* black */
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0x0:VPP_COL_BLACK;
			vpp_fill_framebuffer(fb,0,0,fb->fb_w,fb->fb_h,color);
			break;
		case VPP_PATTERN_WHITE:	/* white */
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFFFFFFFF:VPP_COL_WHITE;
			vpp_fill_framebuffer(fb,0,0,fb->fb_w,fb->fb_h,color);
			break;
		case VPP_PATTERN_WHITE_BLACK: /* white & black */
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFFFFFFFF:VPP_COL_WHITE;			
			vpp_fill_framebuffer(fb, 0, 0, fb->img_w, fb->img_h, color);
			for(i=0;i<(fb->img_w-100);i+=100){
				if( i % 200 ){
					color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0x0:VPP_COL_BLACK;
					vpp_fill_framebuffer(fb, i, 0, 100, fb->img_h,color);
				}
				else {
					color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFFFFFFFF:VPP_COL_WHITE;
					vpp_fill_framebuffer(fb, i, 0, 100, fb->img_h,color);
				}
			}
			break;
		case VPP_PATTERN_FILL: /* fill color */
			vpp_fill_framebuffer(fb,0,0,fb->fb_w,fb->fb_h,arg);
			break;
		case VPP_PATTERN_GRADIENT: /* Gradient */
			resx = fb->img_w;
			resy = fb->img_h;
			level = arg;
			for(i=0;i<level;i++){
				color = 256 * i / level;
				color = (color << 16) | (color << 8) | color;
				vpp_fill_framebuffer(fb, resx * i / level, 0
					, resx/level+1, resy/4, color);
			}
			for(i=0;i<level;i++){
				color = 256 * i / level;
				color = (color << 16);
				vpp_fill_framebuffer(fb, resx * i / level, resy/4
					, resx/level+1, resy/4, color);
			}
			for(i=0;i<level;i++){
				color = 256 * i / level;
				color = (color << 8);
				vpp_fill_framebuffer(fb, resx * i / level, resy/2
					, resx/level+1, resy/4, color);
			}
			for(i=0;i<level;i++){
				color = 256 * i / level;
				vpp_fill_framebuffer(fb, resx * i / level, resy*3/4
					, resx/level+1, resy/4, color);
			}
			break;
		case VPP_PATTERN_BLOCK: /* scale pattern */
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFFFFFFFF:VPP_COL_WHITE;
			vpp_fill_framebuffer(fb,0,0,fb->img_w,fb->img_h,color);
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFF:VPP_COL_BLUE;
			vpp_fill_framebuffer(fb,0,0,fb->img_w/4,fb->img_h/4,color);
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFF00:VPP_COL_GREEN;
			vpp_fill_framebuffer(fb,0,0,fb->img_w/16,fb->img_h/16,color);
			color = (fb->col_fmt == VDO_COL_FMT_ARGB)? 0xFF0000:VPP_COL_RED;
			vpp_fill_framebuffer(fb,0,0,fb->img_w/32,fb->img_h/32,color);	
			break;
		default:
			break;
	}
}

unsigned int vpp_get_base_clock(vpp_clk_pll_t pll,vpp_clk_div_t div)
{
	unsigned int clock;
	unsigned int reg;
	unsigned int value;

	reg = vppif_reg32_in(pll);
	switch(pll){
		case VPP_PLL_A:	// PLL_A
		case VPP_PLL_B:	// PLL_B
#if(WMT_CUR_PID == WMT_PID_8425)
			clock = 27000000;
#else
			clock = 25000000;	/* system freq 25M */
#endif
			break;
		case VPP_PLL_C:	// PLL_C
			clock = ( reg & BIT5 )? 25000000:27000000;
			break;
		case VPP_PLL_D:	// PLL_D
			clock = 27000000;	/* system freq 25M */
			break;
		default:
			return 0;
	}

	if( (reg & BIT8) == 0 ){	// Pre-Divisor Bypass
		clock /= 2;
	}
	
	value = reg & 0x1F;
	if( value >= 4 ){
		value = value * 2;
	}
	else {
		value = 1;
	}
	clock *= value;

	if( div ){
		reg = vppif_reg8_in(div);
		if( div == VPP_DIV_LCD ){
			reg = reg & 0xFF;
			value = (reg==0)? 256:reg;
		}
		else {
			reg = reg & 0x1F;
			value = (reg==0)? 32:reg;
		}
		clock /= value;
	}
	return clock;
}

unsigned int vpp_calculate_diff(unsigned int val1,unsigned int val2)
{
	return (val1 >= val2)?(val1-val2):(val2-val1);
}

void vpp_calculate_clock(vpp_base_clock_t *clk)
{
	unsigned int pixclk;
	unsigned int sum1,sum2;
	int diff,diff_min;
	int base,mul,div;
	int base_bk,mul_bk,div_bk;

	diff_min = pixclk = clk->pixel_clock;
	base_bk = mul_bk = div_bk = 1;
	for(div=1;div<=32;div++){
		base = 6250000;
		mul = pixclk * div / base;
		sum1 = base * mul / div;
		sum2 = base * (mul + 1) / div;
		sum1 = vpp_calculate_diff(sum1,pixclk);
		sum2 = vpp_calculate_diff(sum2,pixclk);
		mul = (sum1 < sum2)? mul:(mul+1);
		sum1 = base * mul / div;
		mul /= 2;
		base *= 2;
		if( mul > 62 ){
			base *= 2;
			mul /= 2;
		}
		sum1 = base * mul;
		if( (sum1 < 300000000) || (sum1 > 750000000) ) continue;
		if( (mul % 2) || (mul < 8) || (mul > 62) ) continue;
		sum1 = sum1 / div;
		diff = vpp_calculate_diff(sum1,pixclk);
		if( diff < diff_min ){
			diff_min = diff;
			base_bk = base;
			mul_bk = mul;
			div_bk = div;
		}
	}

	for(div=1;div<=32;div++){
		base = 6750000;
		mul = pixclk * div / base;
		sum1 = base * mul / div;
		sum2 = base * (mul + 1) / div;
		sum1 = vpp_calculate_diff(sum1,pixclk);
		sum2 = vpp_calculate_diff(sum2,pixclk);
		mul = (sum1 < sum2)? mul:(mul+1);

		sum1 = base * mul / div;
		mul /= 2;
		base *= 2;
		if( mul > 62 ){
			base *= 2;
			mul /= 2;
		}
		sum1 = base * mul;
		if( (sum1 < 300000000) || (sum1 > 750000000) ) continue;
		if( (mul % 2) || (mul < 8) || (mul > 62) ) continue;
		sum1 = sum1 / div;
		diff = vpp_calculate_diff(sum1,pixclk);
		if( diff < diff_min ){
			diff_min = diff;
			base_bk = base;
			mul_bk = mul;
			div_bk = div;
		}
	}
	
//	VPPIFMSG("pixclk %d, base %d, mul %d, div %d,diff %d\n",pixclk,base_bk,mul_bk,div_bk,diff_min);
	clk->pixel_clock = base_bk * mul_bk / div_bk;
	clk->divisor = div_bk;
	clk->rd_cyc = 0;

	switch( base_bk ){
		case 12500000: 
			clk->PLL = 0x20; 
			break;
		default:
		case 25000000: 
			clk->PLL = 0x120; 
			break;
		case 13500000:
			clk->PLL = 0x00;
			break;
		case 27000000:
			clk->PLL = 0x100;
			break;
	}
	clk->PLL = clk->PLL | (mul_bk / 2);
	
}

unsigned int vpp_set_base_clock(vpp_mod_t mod,unsigned int pixel_clock)
{
	/* notice : 750M > clk * PLL mul > 300M */
	vpp_base_clock_t vpp_base_clock_ary[] = {
		{ 13514000, 0x108, 32, 0 },		/* PreDiv 1, 27M base, PLL multi 16, Div 32, rd_cyc 1 */
		{ 23750000, 0x33, 20, 0 },		/* PreDiv 2, 25M base, PLL multi 38, Div 20, rd_cyc 1 */
		{ 25180000, 0x107, 15, 0 },		/* PreDiv 1, 27M base, PLL multi 14, Div 15, rd_cyc 1 */
		{ 27000000, 0x107, 14, 0 },		/* PreDiv 1, 27M base, PLL multi 14, Div 14, rd_cyc 1 */
		{ 29500000, 0x2d, 11, 0 },		/* PreDiv 2, 25M base, PLL multi 26, Div 11, rd_cyc 1 */				
		{ 30750000, 0x128, 13, 0 },		/* PreDiv 1, 25M base, PLL multi 16, Div 13, rd_cyc 1 */		
		{ 31500000, 0x107, 12, 0 },		/* PreDiv 1, 27M base, PLL multi 14, Div 12, rd_cyc 1 */
		{ 38250000, 0x11, 12, 0 },		/* PreDiv 2, 27M base, PLL multi 34, Div 12, rd_cyc 1 */
		{ 38500000, 0x12a, 13, 0 },		/* PreDiv 1, 25M base, PLL multi 20, Div 13, rd_cyc 1 */		
		{ 40000000, 0x128, 10, 0 },		/* PreDiv 1, 25M base, PLL multi 16, Div 10, rd_cyc 1 */
		{ 49000000, 0x10a, 11, 0 },		/* PreDiv 1, 27M base, PLL multi 20, Div 11, rd_cyc 1 */		
		{ 49500000, 0x10b, 12, 0 },		/* PreDiv 1, 27M base, PLL multi 22, Div 12, rd_cyc 1 */
		{ 54054000, 0x104, 4, 0 },		/* PreDiv 1, 27M base, PLL multi 8,  Div 4, rd_cyc 1 */
		{ 63500000, 0x37, 9, 0 },		/* PreDiv 2, 25M base, PLL multi 46, Div 9, rd_cyc 1 */		
		{ 65000000, 0x2d, 5, 0 },		/* PreDiv 2, 25M base, PLL multi 26, Div 5, rd_cyc 1 */
		{ 74500000, 0x10b, 8, 0 },		/* PreDiv 1, 27M base, PLL multi 22, Div 8, rd_cyc 1 */		
		{ 78750000, 0x12b, 7, 0 },		/* PreDiv 1, 25M base, PLL multi 22, Div 7, rd_cyc 1 */		
		{ 79500000, 0x33, 6, 0 },		/* PreDiv 2, 25M base, PLL multi 38, Div 6, rd_cyc 1 */
		{ 82000000, 0x37, 7, 0 },		/* PreDiv 2, 25M base, PLL multi 46, Div 7, rd_cyc 1 */				
		{ 83500000, 0x12a, 6, 0 },		/* PreDiv 1, 25M base, PLL multi 20, Div 6, rd_cyc 1 */
		{ 95750000, 0x37, 6, 0 },		/* PreDiv 2, 25M base, PLL multi 46, Div 6, rd_cyc 1 */
		{ 101250000, 0x0f, 4, 0 },		/* PreDiv 2, 27M base, PLL multi 30, Div 4, rd_cyc 1 */
		{ 102250000, 0x13, 5, 0 },		/* PreDiv 2, 27M base, PLL multi 38, Div 5, rd_cyc 1 */		
		{ 104000000, 0x39, 6, 0 },		/* PreDiv 2, 25M base, PLL multi 50, Div 6, rd_cyc 1 */
		{ 106500000, 0x31, 4, 0 },		/* PreDiv 2, 25M base, PLL multi 34, Div 4, rd_cyc 1 */		
		{ 108000000, 0x108, 4, 0 },		/* PreDiv 1, 27M base, PLL multi 16, Div 4, rd_cyc 1 */
		{ 109000000, 0x2d, 3, 0 },		/* PreDiv 2, 25M base, PLL multi 26, Div 3, rd_cyc 1 */		
		{ 121750000, 0x109, 4, 0 },		/* PreDiv 1, 27M base, PLL multi 18, Div 4, rd_cyc 1 */
		{ 130000000, 0x12d, 5, 0 },		/* PreDiv 1, 25M base, PLL multi 26, Div 5, rd_cyc 1 */
		{ 135000000, 0x10a, 4, 0 },		/* PreDiv 1, 27M base, PLL multi 20, Div 4, rd_cyc 1 */		
		{ 136750000, 0x12b, 4, 0 },		/* PreDiv 1, 25M base, PLL multi 22, Div 4, rd_cyc 1 */
		{ 154000000, 0x114, 7, 0 },		/* PreDiv 1, 27M base, PLL multi 40, Div 7, rd_cyc 1 */	
		{ 162000000, 0x12, 3, 0 },		/* PreDiv 2, 27M base, PLL multi 12, Div 1, rd_cyc 1 */
	};

	vpp_base_clock_t *ptr;
	int i,ary_cnt;
	unsigned int temp;
	unsigned rd_cyc;
	vpp_base_clock_t cal_clock;

	switch( mod ){
		case VPP_MOD_GOVRH:
			ary_cnt = sizeof(vpp_base_clock_ary)/sizeof(vpp_base_clock_t);
			for(i=0;i<ary_cnt;i++){
				ptr = &vpp_base_clock_ary[i];
				if( ptr->pixel_clock >= pixel_clock ){
					break;
				}
			}

			if( i == ary_cnt ){
				ptr = &vpp_base_clock_ary[ary_cnt-1];
			}
			else {
				if( pixel_clock != ptr->pixel_clock ){
					cal_clock.pixel_clock = pixel_clock;
					vpp_calculate_clock(&cal_clock);
					ptr = &cal_clock;
				}
			}
		
	//		VPPIFMSG("pixclk %d,clock %d,PLL 0x%x,Div %d,RCYC %d\n",pixel_clock,ptr->pixel_clock,ptr->PLL,ptr->divisor,ptr->rd_cyc);
{
			extern int dvo_pll_set (unsigned int multi, unsigned char divisor, unsigned short resx, unsigned short resy);
			if( dvo_pll_set(ptr->PLL, ptr->divisor, g_vpp.resx, g_vpp.resy) == 0 ){
				printk("not support %dx%d\n",g_vpp.resx,g_vpp.resy);
			}
}
			rd_cyc = ptr->rd_cyc;
			break;
		case VPP_MOD_LCDC:
			{
			unsigned int diff1,diff2;
				
			temp = vpp_get_base_clock(VPP_PLL_B,0);
			rd_cyc = temp / pixel_clock;
			diff1 = vpp_calculate_diff(pixel_clock,temp/rd_cyc);
			diff2 = vpp_calculate_diff(pixel_clock,temp/(rd_cyc+1));
			rd_cyc = (diff1 < diff2)? rd_cyc:(rd_cyc+1);
			if( rd_cyc >= 256 ) rd_cyc = 0;
			vppif_reg8_out(VPP_DIV_LCD,rd_cyc);
			printk("[VPP] set lcdc pixclk %d,pll %d,div %d\n",pixel_clock,temp,rd_cyc);
			}
			break;
		default:
			rd_cyc = vpp_get_base_clock(VPP_PLL_B,VPP_DIV_VPP) / pixel_clock;
			break;
	}
	return rd_cyc;
}

unsigned int vpp_check_value(unsigned int val,unsigned int min,unsigned int max)
{
	if( min >= max ) 
		return min;
	val = (val < min)? min:val;
	val = (val > max)? max:val;
	return val;
}

void vpp_show_timing(vpp_timing_t *tmr,vpp_clock_t *clk)
{
	if( tmr ){
		DPRINT("pixel clock %d,option 0x%x\n",tmr->pixel_clock,tmr->option);
		DPRINT("H sync %d,bp %d,pixel %d,fp %d\n",tmr->hsync,tmr->hbp,tmr->hpixel,tmr->hfp);
		DPRINT("V sync %d,bp %d,pixel %d,fp %d\n",tmr->vsync,tmr->vbp,tmr->vpixel,tmr->vfp);
	}
	
	if( clk ){
		DPRINT("H beg %d,end %d,total %d\n",clk->begin_pixel_of_active,clk->end_pixel_of_active,clk->total_pixel_of_line);
		DPRINT("V beg %d,end %d,total %d\n",clk->begin_line_of_active,clk->end_line_of_active,clk->total_line_of_frame);
		DPRINT("Hsync %d, Vsync %d\n",clk->hsync,clk->vsync);
		DPRINT("VBIE %d,PVBI %d\n",clk->line_number_between_VBIS_VBIE,clk->line_number_between_PVBI_VBIS);
	}
}

void vpp_trans_timing(vpp_mod_t mod,vpp_timing_t *tmr,vpp_clock_t *hw_tmr,int to_hw)
{
	vpp_fb_base_t *mod_fb_p;
	unsigned int pixel_clock;
	int temp;
	
	mod_fb_p = vpp_mod_get_fb_base(mod);

	if( to_hw ){
		hw_tmr->begin_pixel_of_active = tmr->hsync + tmr->hbp;
		hw_tmr->end_pixel_of_active = hw_tmr->begin_pixel_of_active + tmr->hpixel;
		hw_tmr->total_pixel_of_line = hw_tmr->end_pixel_of_active + tmr->hfp;	
		hw_tmr->begin_line_of_active = tmr->vsync + tmr->vbp + 1;
		hw_tmr->end_line_of_active = hw_tmr->begin_line_of_active + tmr->vpixel;
		hw_tmr->total_line_of_frame = hw_tmr->end_line_of_active + tmr->vfp -1;	
		hw_tmr->line_number_between_VBIS_VBIE = tmr->vsync + 1; // hw_tmr->begin_line_of_active - 3;
		temp = hw_tmr->total_line_of_frame - hw_tmr->end_line_of_active;
		hw_tmr->line_number_between_PVBI_VBIS = (temp>2)? (temp-1):1;
		hw_tmr->hsync = tmr->hsync;
		hw_tmr->vsync = tmr->vsync;
		
		// pixel_clock = hw_tmr->total_pixel_of_line * hw_tmr->total_line_of_frame * mod_fb_p->framerate;
		pixel_clock = tmr->pixel_clock;
		if( mod == VPP_MOD_GOVRH ){
			hw_tmr->read_cycle = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO) / pixel_clock;		
		}
		else {
			hw_tmr->read_cycle = vpp_get_base_clock(VPP_PLL_B,VPP_DIV_VPP) / pixel_clock;		
		}
	}
	else {
		pixel_clock = hw_tmr->total_pixel_of_line * hw_tmr->total_line_of_frame * mod_fb_p->framerate;
		tmr->pixel_clock = pixel_clock;
		tmr->option = 0;
		
		tmr->hsync = hw_tmr->hsync;
		tmr->hbp = hw_tmr->begin_pixel_of_active - hw_tmr->hsync;
		tmr->hpixel = hw_tmr->end_pixel_of_active - hw_tmr->begin_pixel_of_active;
		tmr->hfp = hw_tmr->total_pixel_of_line - hw_tmr->end_pixel_of_active;

		tmr->vsync = hw_tmr->vsync;
		tmr->vbp = hw_tmr->begin_line_of_active - hw_tmr->vsync -1;
		tmr->vpixel = hw_tmr->end_line_of_active - hw_tmr->begin_line_of_active;
		tmr->vfp = hw_tmr->total_line_of_frame - hw_tmr->end_line_of_active +1;
	}
}

void vpp_calculate_timing(vpp_mod_t mod,unsigned int fps,vpp_clock_t *tmr)
{
	unsigned int base_clock;
	unsigned int rcyc_max,rcyc_min;
	unsigned int h_min,h_max;
	unsigned int v_min,v_max;
	unsigned int diff_min,diff_h,diff_v,diff_rcyc;	
	unsigned int hbp_min,hfp_min,hporch_min;
	unsigned int vbp_min,vfp_min,vporch_min;	
	int i,j,k,temp;
	unsigned int hpixel,vpixel;

	hpixel = tmr->end_pixel_of_active - tmr->begin_pixel_of_active;
	vpixel = tmr->end_line_of_active - tmr->begin_line_of_active;

	if( mod == VPP_MOD_GOVRH ){
		base_clock = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO) / fps;
	}
	else {
		base_clock = vpp_get_base_clock(VPP_PLL_B,VPP_DIV_VPP) / fps;
	}
	hbp_min = tmr->begin_pixel_of_active;
	hfp_min = tmr->total_pixel_of_line - tmr->end_pixel_of_active;
	hporch_min = hbp_min + hfp_min;
	vbp_min = tmr->begin_line_of_active;
	vfp_min = tmr->total_line_of_frame - tmr->end_line_of_active;
	vporch_min = vbp_min + vfp_min;

	rcyc_min = vpp_check_value((base_clock / (4096 * 4096)),4,256);
	rcyc_max = vpp_check_value((base_clock / (hpixel * vpixel)) + 1,4,256);

	h_min = vpp_check_value((base_clock / (rcyc_max * 4096)),hpixel+hporch_min,4096);
	h_max = vpp_check_value((base_clock / (rcyc_min * vpixel)) + 1,hpixel+hporch_min,4096);

	v_min = vpp_check_value((base_clock / (rcyc_max * 4096)),vpixel+vporch_min,4096);
	v_max = vpp_check_value((base_clock / (rcyc_min * hpixel)) + 1,vpixel+vporch_min,4096);

	diff_min=0xFFFFFFFF;
	diff_rcyc = diff_h = diff_v = 0;
	for(i=rcyc_max;i>=rcyc_min;i--){
		for(j=v_min;j<=v_min;j++){
			for(k=h_min;k<=h_max;k++){
				temp = i * j * k;
				temp = vpp_calculate_diff(base_clock,temp);
				if(diff_min > temp){
					if( (diff_rcyc != i) && ( (diff_min - temp) < 5000 )){
						continue;
					}
					diff_min = temp;
					diff_h = k;
					diff_v = j;
					diff_rcyc = i;
				}
			}
		}
	}

	tmr->read_cycle = diff_rcyc - 1;
	tmr->total_pixel_of_line = diff_h;
	temp = diff_h - hpixel - hbp_min;
	tmr->begin_pixel_of_active = ( hbp_min + temp/2 );
	tmr->begin_pixel_of_active = vpp_check_value(tmr->begin_pixel_of_active,hbp_min,504);
	tmr->end_pixel_of_active = tmr->begin_pixel_of_active + hpixel;

	tmr->total_line_of_frame = diff_v;
	temp = diff_v - vpixel - vbp_min;
	tmr->begin_line_of_active = ( vbp_min + temp/2 );
	tmr->begin_line_of_active = vpp_check_value(tmr->begin_line_of_active,vbp_min,504);
	tmr->end_line_of_active = tmr->begin_line_of_active + vpixel;

	tmr->line_number_between_VBIS_VBIE = tmr->begin_line_of_active - 3;

#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
	g_vpp.govw_tg_rcyc = tmr->read_cycle;
	g_vpp.govw_tg_rtn_cnt = 0;
	g_vpp.govw_tg_rtn_max = fps;
#endif	
}

vpp_csc_t vpp_check_csc_mode(vpp_csc_t mode,vdo_color_fmt src_fmt,vdo_color_fmt dst_fmt,unsigned int flags)
{
	if( mode >= VPP_CSC_MAX ) 
		return VPP_CSC_BYPASS;

	mode = (mode > VPP_CSC_RGB2YUV_MIN)? (mode - VPP_CSC_RGB2YUV_MIN):mode;
	if( src_fmt == VDO_COL_FMT_ARGB ){
		mode = VPP_CSC_RGB2YUV_MIN + mode;
	}
	else {
		src_fmt = VDO_COL_FMT_YUV444;
	}
	dst_fmt = (dst_fmt == VDO_COL_FMT_ARGB)? VDO_COL_FMT_ARGB:VDO_COL_FMT_YUV444;
	if( flags == 0 ){
		mode = ( src_fmt != dst_fmt )? mode:VPP_CSC_BYPASS;
	}
	return mode;
}

void vpp_set_vout_resolution(int resx,int resy,int fps)
{
	p_govw->fb_p->fb.img_w = resx;
	p_govw->fb_p->fb.img_h = resy;
	p_govw->fb_p->set_framebuf(&p_govw->fb_p->fb);

#ifdef WMT_FTBLK_GOVRH
	p_govrh->fb_p->fb.img_w = resx;
	p_govrh->fb_p->fb.img_h = resy;
	p_govrh->fb_p->framerate = fps;
	p_govrh->fb_p->set_framebuf(&p_govrh->fb_p->fb);
#endif	
}

int vpp_get_gcd(int A, int B)
{
	while(A != B){
		if( A > B ){
			A = A - B;
		}
		else {
			B = B - A;
		}
	}
	return A;
}

#ifdef PATCH_SCL_R_WIDTH
void vpp_check_scale_ratio_align(int *src,int *dst,int max,int min)
{
	int val1,val2;
	int gcd;
	int div;
	if( *dst >= *src ){	// scale up
		if( (*dst) > ((*src)*max) ){
			DPRINT("*W* scale up over spec (max %d)\n",max);
			*dst = (*src) * max;
		}
	}
	else {	// scale down
		gcd = 1;
		div = 1;	
		val1 = *src;
		val2 = *dst;
		while(val1 > val2){
			gcd = vpp_get_gcd(val1,val2);
			if( (val1/gcd) <= min ){
				break;
			}
			val1 = val1/gcd;
			val2 = val2/gcd;
			div *= gcd;			

			val1 = (val1/2)*2;
			val2 = (val2/2)*2;
			if( val2 < 2 )
				val2 = 2;
		}
		
		*src = (val1*div);
		*dst = (val2*div);
	}
}	
#endif

void vpp_check_scale_ratio(int *src,int *dst,int max,int min)
{
	int val1,val2;

	if( *dst >= *src ){	// scale up
		if( (*dst) > ((*src)*max) ){
			DPRINT("*W* scale up over spec (max %d)\n",max);
			*dst = (*src) * max;
		}
	}
	else {	// scale down
		int p,q,diff;
		int diff_min,p_min,q_min;

		val1 = val2 = (*dst) * 1000000 / (*src);
		diff_min = val1;
		p_min = 1;
		q_min = 32;
		for(q=2;q<=min;q++){
			for(p=1;p<q;p++){
				val2 = p * 1000000 / q;
				if( val1 < val2 ){
					break;
				}
				diff = vpp_calculate_diff(val1,val2);
				if( diff < diff_min ){
					diff_min = diff;
					p_min = p;
					q_min = q;
				}
			}
			if( val1 == val2 )
				break;
		}
		val1 = (*src) / q_min;
		val2 = (*dst) / p_min;
		val1 = (val1 < val2)? val1:val2;
		*src = val1 * q_min;
		*dst = val1 * p_min;
	}
}	

void vpp_calculate_scale_ratio(int *src,int *dst,int max,int min,int align_s,int align_d)
{
	int i,diff,min_diff,min_s,min_d;
	int val1,val2;
	int diff2;
	int temp_s,temp_d;
	int match = 0;

	if( *dst >= *src ){
		if( (*dst) > ((*src)*max) ){
			DPRINT("*W* scale up over spec (max %d)\n",max);
			*dst = (*src) * max;
		}

		*src -= (*src % align_s);
		*dst -= (*dst % align_d);
		return;
	}
	
	diff = diff2 = min_diff = (*src + *dst) * 100;
	min_s = *src;
	min_d = *dst;
	for(i=-((*src / *dst)+1);;i++){
		if( (i>0) && (diff == 0) && match )
			break;
		if( i > min ){
			(*dst)++;
			i=-((*src / *dst)+1);
			continue;
		}
		
		val1 = *src + i;
		val2 = *dst;
		vpp_check_scale_ratio(&val1,&val2,max,min);
		
		if( val1 % align_s ) continue;	// src img_w align
		if( val2 % align_d ) continue;	// dst img_w align

		if( val1 >= *src ){
			temp_s = *src * 100;
			temp_d = (temp_s) * val2 / val1;
			diff = 0;
			diff2 = vpp_calculate_diff(*dst*100,temp_d);
		}
		else {
			temp_s = val1 * 100;
			diff = vpp_calculate_diff(*src*100,temp_s);

			temp_d = temp_s * val2 / val1;
			diff2 = vpp_calculate_diff(*dst*100,temp_d);
		}

//		if( temp_s < (*src*100) ) continue;
//		if( temp_d > (*dst*100) ) continue;

		match = 1;
//		if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
//			printk("%d:%d->%d,%d->%d,%d->%d,s diff %d,d diff %d\n",i,*src,*dst,val1,val2,temp_s,temp_d,diff,diff2);
//		}

//		if( diff2 < min_diff ){
//			min_diff = diff2;
		if( (diff+diff2) < min_diff ){
			min_diff = diff + diff2;
			min_s = val1;
			min_d = val2;
		}
			
		if( (diff + diff2) == 0 )
			break;
	}
	*src = min_s;
	*dst = min_d;
}

int vpp_set_recursive_scale(vdo_framebuf_t *src_fb,vdo_framebuf_t *dst_fb)
{
	int ret;
	unsigned int s_w,s_h;
	unsigned int d_w,d_h;
	int dst_w_flag = 0;
	int align_s,align_d;
	int keep_ratio;
	int width_min;

	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		DPRINT("[S1] src(%dx%d),Y 0x%x,C 0x%x,fb(%dx%d),colfmt %d\n",src_fb->img_w,src_fb->img_h,
								src_fb->y_addr,src_fb->c_addr,src_fb->fb_w,src_fb->fb_h,src_fb->col_fmt);
		DPRINT("[S1] dst(%dx%d),Y 0x%x,C 0x%x,fb(%dx%d),colfmt %d\n",dst_fb->img_w,dst_fb->img_h,
								dst_fb->y_addr,dst_fb->c_addr,dst_fb->fb_w,dst_fb->fb_h,dst_fb->col_fmt);
	}

	if( (dst_fb->img_w < src_fb->img_w) && (dst_fb->img_h > dst_fb->img_h) ){
		DPRINT("*W* WM3426 scale quality issue: H scale dn & V scale up\n");
		DPRINT("** Please scale by 2 step\n");
	}

	s_w = src_fb->img_w;
	s_h = src_fb->img_h;
	d_w = dst_fb->img_w;
	d_h = dst_fb->img_h;
	keep_ratio = 0;
	if( g_vpp.scale_keep_ratio ){
		if( d_w < s_w  ){	// keep ratio feature just in scale down, bcz scale up don't have the scale entry limitation
		keep_ratio = ( vpp_calculate_diff((s_w*100/d_w),(s_h*100/d_h)) < 15 )? 1:0;		// keep ratio if s to d ratio diff less than 15%
		}
	}
	
	// H scale
	if( src_fb->col_fmt == VDO_COL_FMT_ARGB ){
		align_s = ( src_fb->h_crop )? 2:1;
	}
	else {
		align_s = ( src_fb->h_crop )? 8:1;
	}
#ifdef PATCH_SCL_R_WIDTH
	align_d = ( dst_fb->col_fmt == VDO_COL_FMT_ARGB )? 2:8;	// dst YC 8 align in not match spec
#else
	align_d = ( dst_fb->col_fmt == VDO_COL_FMT_ARGB )? 2:1;
#endif

//	printk("[VPP] align s %d,d %d\n",align_s,align_d);
	vpp_calculate_scale_ratio(&src_fb->img_w,&dst_fb->img_w, VPP_SCALE_UP_RATIO_H,
								(keep_ratio)?VPP_SCALE_DN_RATIO_V:VPP_SCALE_DN_RATIO_H,align_s,align_d);
	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		printk("[H] cal: src %d,dst %d\n",src_fb->img_w,dst_fb->img_w);
	}

	// dst width should more than 64 bytes
	width_min = (dst_fb->col_fmt == VDO_COL_FMT_ARGB)? 16:64;
	if( dst_fb->img_w < width_min ){
		int ratio = 1;
		do {
			if( (dst_fb->img_w * ratio) >= width_min)
				break;

			ratio++;
		} while(1);
			
		src_fb->img_w *= ratio;
		dst_fb->img_w *= ratio;
		dst_w_flag = 1;
		if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
			printk("[H] width>=64: src %d,dst %d\n",src_fb->img_w,dst_fb->img_w);
		}
	}

#ifdef PATCH_SCLW_RGB_SRAM_SWITCH
	// patch for sclw ARGB dst width issue 
	if( dst_fb->col_fmt == VDO_COL_FMT_ARGB ){
		int temp;

		temp = dst_fb->img_w % 96;
		if( (temp > 64) && (temp <= 80) ){
#if 1	// patch 2 : scale more than 80 and cut but dst fb should alloc more
			unsigned int new_w,gcd;

			gcd = vpp_get_gcd(src_fb->img_w,dst_fb->img_w);
			temp -= 64;
			new_w = dst_fb->img_w - temp + 18;
			temp = new_w % (dst_fb->img_w / gcd);
			if( temp ){
				new_w = new_w - temp + (dst_fb->img_w / gcd);
			}
			
			src_fb->img_w = src_fb->img_w * new_w / dst_fb->img_w;
			dst_fb->img_w = new_w;
			dst_w_flag = 1;
#else	// patch 1 : dst img_w should be 64 if 65 - 80
			temp -= 64;
			dst_fb->img_w -= temp;
#endif
			if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
				printk("[H] RGB mod 96: src %d,dst %d\n",src_fb->img_w,dst_fb->img_w);
			}
		}
	}
#endif

	// V scale
	if( keep_ratio ){
		int p,q,gcd;
		gcd = vpp_get_gcd(src_fb->img_w,dst_fb->img_w);
		p = dst_fb->img_w / gcd;
		q = src_fb->img_w / gcd;
		dst_fb->img_h = (dst_fb->img_h / p) * p;
		src_fb->img_h = dst_fb->img_h * q / p;
		if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		printk("[V] keep ratio %d/%d,src %d,dst %d\n",p,q,src_fb->img_h,dst_fb->img_h);
		}
	}
	else {
		vpp_calculate_scale_ratio(&src_fb->img_h,&dst_fb->img_h, VPP_SCALE_UP_RATIO_V, VPP_SCALE_DN_RATIO_V,1,1);
	}
	
	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		printk("[V] cal: src %d,dst %d\n",src_fb->img_h,dst_fb->img_h);
	}

	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		int h_gcd,v_gcd;
		DPRINT("[S2] src(%dx%d)\n",src_fb->img_w,src_fb->img_h);
		DPRINT("[S2] dst(%dx%d)\n",dst_fb->img_w,dst_fb->img_h);
		h_gcd = vpp_get_gcd(src_fb->img_w,dst_fb->img_w);
		v_gcd = vpp_get_gcd(src_fb->img_h,dst_fb->img_h);
		DPRINT("[S2] H %d/%d,%d, V %d/%d,%d \n",dst_fb->img_w/h_gcd,src_fb->img_w/h_gcd,h_gcd,
			dst_fb->img_h/v_gcd,src_fb->img_h/v_gcd,v_gcd);
	}

	ret = p_scl->scale(src_fb,dst_fb);

	// cut dummy byte
	if( dst_fb->img_w < src_fb->img_w  ){
		if( src_fb->img_w > s_w ){
			unsigned int d_w;
			d_w = dst_fb->img_w;
			dst_fb->img_w = s_w * dst_fb->img_w / src_fb->img_w;
			src_fb->img_w = src_fb->img_w * dst_fb->img_w / d_w;
		}
	}
	else {
		if( dst_w_flag ){
			if( src_fb->img_w > s_w ){
				dst_fb->img_w = s_w * dst_fb->img_w / src_fb->img_w;
				src_fb->img_w = s_w;
			}
		}
	}

	if( dst_fb->img_h < src_fb->img_h ){
		if( src_fb->img_h > s_h ){
			unsigned int d_h;
			d_h = dst_fb->img_h;
			dst_fb->img_h = s_h * dst_fb->img_h / src_fb->img_h;
			src_fb->img_h = src_fb->img_h * dst_fb->img_h / d_h;
		}
	}

	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		DPRINT("[S3] src(%dx%d)\n",src_fb->img_w,src_fb->img_h);
		DPRINT("[S3] dst(%dx%d)\n",dst_fb->img_w,dst_fb->img_h);
	}
	return ret;
}

static int vpp_check_view(vdo_view_t *vw)
{
	vdo_framebuf_t *fb;
	
	fb = &p_vpu->fb_p->fb;
	if( fb->img_w != vw->resx_virtual ) return 0;
	if( fb->img_h != vw->resy_virtual ) return 0;
	if( fb->fb_w != vw->resx_src ) return 0;
	if( fb->fb_h != vw->resy_src ) return 0;
	if( fb->h_crop != vw->offsetx ) return 0;
	if( fb->v_crop != vw->offsety ) return 0;
	if( p_vpu->resx_visual != vw->resx_visual ) return 0;
	if( p_vpu->resy_visual != vw->resy_visual ) return 0;
	if( p_vpu->posx != vw->posx ) return 0;
	if( p_vpu->posy != vw->posy ) return 0;
	return 1;
}

void vpp_set_video_scale(vdo_view_t *vw)
{
	vdo_framebuf_t *fb;
	unsigned int vis_w,vis_h;

	if( vpp_check_view(vw) ){
		if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
			DPRINT("[VPP] view not change\n");
		}		
		goto set_video_scale_end;
	}

	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		DPRINT("[V1] X src %d,vit %d,vis %d,pos %d,oft %d\n",vw->resx_src,vw->resx_virtual,vw->resx_visual,vw->posx,vw->offsetx);
		DPRINT("[V1] Y src %d,vit %d,vis %d,pos %d,oft %d\n",vw->resy_src,vw->resy_virtual,vw->resy_visual,vw->posy,vw->offsety);
	}
	
	fb = &p_vpu->fb_p->fb;
	fb->img_w = vw->resx_virtual;
	fb->img_h = vw->resy_virtual;
	fb->fb_w = vw->resx_src;
	fb->fb_h = vw->resy_src;
	fb->h_crop = vw->offsetx;
	fb->v_crop = vw->offsety;
	
	p_vpu->resx_visual = vw->resx_visual;
	p_vpu->resy_visual = vw->resy_visual;
	p_vpu->posx = vw->posx;
	p_vpu->posy = vw->posy;

	p_vpu->fb_p->set_framebuf(fb);
	g_vpp.govw_skip_frame = 1;	

set_video_scale_end:
	vis_w = p_vpu->resx_visual_scale;
	vis_h = p_vpu->resy_visual_scale;
#if 0
	if( g_vpp.video_fit_edge ){
		if( p_vpu->resx_virtual_scale > vw->resx_virtual ){
			vis_w = vw->resx_virtual * p_vpu->resx_visual_scale / p_vpu->resx_virtual_scale;
		}
		if( p_vpu->resy_virtual_scale > vw->resy_virtual ){
			vis_h = vw->resy_virtual * p_vpu->resy_visual_scale / p_vpu->resy_virtual_scale;
		}
	}
#endif	
	govm_set_vpu_coordinate(vw->posx,vw->posy,vw->posx+vis_w-1,vw->posy+vis_h-1);
	if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
		int h_gcd,v_gcd;
		DPRINT("[V2] X src %d,vit %d,vis %d,pos %d,oft %d\n",vw->resx_src,p_vpu->resx_virtual_scale,p_vpu->resx_visual_scale,vw->posx,vw->offsetx);
		DPRINT("[V2] Y src %d,vit %d,vis %d,pos %d,oft %d\n",vw->resy_src,p_vpu->resy_virtual_scale,p_vpu->resy_visual_scale,vw->posy,vw->offsety);
		h_gcd = vpp_get_gcd(p_vpu->resx_virtual_scale,p_vpu->resx_visual_scale);
		v_gcd = vpp_get_gcd(p_vpu->resy_virtual_scale,p_vpu->resy_visual_scale);
		DPRINT("[V2] H %d/%d,%d, V %d/%d,%d \n",p_vpu->resx_visual_scale/h_gcd,p_vpu->resx_virtual_scale/h_gcd,
												h_gcd,p_vpu->resy_visual_scale/v_gcd,p_vpu->resy_virtual_scale/v_gcd,v_gcd);
	}
}

unsigned int vpp_get_video_mode_fps(vpp_timing_t *timing)
{
	unsigned int temp;

	temp = timing->pixel_clock - (timing->pixel_clock % 1000);
	temp = temp / (timing->hsync + timing->hbp + timing->hpixel + timing->hfp);
	temp = temp / (timing->vsync + timing->vbp + timing->vpixel + timing->vfp);
	return temp;
}

vpp_timing_t *vpp_get_video_mode(unsigned int resx,unsigned int resy,unsigned int fps_pixclk)
{
	int is_fps;
	int i,j;
	vpp_timing_t *ptr;
	unsigned int line_pixel;
	int index;

	is_fps = ( fps_pixclk >= 1000000 )? 0:1;
	for(i=0,index=0;;i++){
		ptr = (vpp_timing_t *) &vpp_video_mode_table[i];
		if( ptr->pixel_clock == 0 ){
			break;
		}
		line_pixel = (ptr->option & VPP_OPT_INTERLACE)? (ptr->vpixel*2):ptr->vpixel;
		if ((ptr->hpixel == resx) && (line_pixel == resy)) {
			for(j=i,index=i;;j++){
				ptr = (vpp_timing_t *) &vpp_video_mode_table[j];
				if( ptr->pixel_clock == 0 ){
					break;
				}
				if( ptr->hpixel != resx ){
					break;
				}
				if( is_fps ){
					if( fps_pixclk == vpp_get_video_mode_fps(ptr) ){
						index = j;
						break;
					}
				}
				else {
					if( fps_pixclk == ptr->pixel_clock ){
						index = j;
						break;
					}
				}
			}
			break;
		}
		if( ptr->hpixel > resx ){
			break;
		}
		index = i;
		if( ptr->option & VPP_OPT_INTERLACE ){
			i++;
		}
	}
	ptr = (vpp_timing_t *) &vpp_video_mode_table[index];
	return ptr;	
}

void vpp_set_video_quality(int mode)
{
#ifdef WMT_FTBLK_VPU
	vpu_set_drop_line((mode)?0:1);
	if((vppif_reg32_read(VPU_DEI_ENABLE)==0) && (vppif_reg32_read(VPU_DROP_LINE)) ){
		vpu_r_set_mif2_enable(VPP_FLAG_DISABLE);
	}
	else {
		vpu_r_set_mif2_enable(VPP_FLAG_ENABLE);
	}
#endif

#ifdef WMT_FTBLK_SCL
	scl_set_drop_line((mode)?0:1);
	if( vppif_reg32_read(SCL_TG_GOVWTG_ENABLE) && (vppif_reg32_read(SCL_SCLDW_METHOD)==0)){
		sclr_set_mif2_enable(VPP_FLAG_ENABLE);
	}
	else {
		sclr_set_mif2_enable(VPP_FLAG_DISABLE);
	}
#endif
}

void vpp_mod_init(void)
{
	DPRINT("[VPP] vpp_mod_init\n");
	vppm_mod_init();
#ifdef WMT_FTBLK_GOVRH	
	govrh_mod_init();
#endif
#ifdef WMT_FTBLK_LCDC
	lcdc_mod_init();
#endif
#ifdef WMT_FTBLK_GOVW
	govw_mod_init();
#endif
#ifdef WMT_FTBLK_SCL
	scl_mod_init();
#endif
#ifdef WMT_FTBLK_GOVM
	govm_mod_init();
#endif
#ifdef WMT_FTBLK_VPU
	vpu_mod_init();
#endif
#ifdef WMT_FTBLK_DISP	
	disp_mod_init();
#endif	
}
