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


/*
 * ChangeLog:
 *
 * 2009-11-18  Vincent Chen  <vincentchen@via.com.tw>
 *         * Fix ge_get_color for RGB32.
 *         * Fix bug on register XOR.
 *         * Replace 'regs->g1_fb_sel ^= 1' by 'regs->g1_fb_sel = BOOL'.
 *
 * 2009-11-11  Vincent Chen  <vincentchen@via.com.tw>
 *         * Fix ge_get_color for RGB32.
 *
 * 2009-10-13  Vincent Chen  <vincentchen@via.com.tw>
 *         * Fix amx_hide_surface.
 *
 * 2009-09-02  Vincent Chen  <vincentchen@via.com.tw>
 *         * Update ge_trylock.
 *
 * 2009-08-27  Vincent Chen  <vincentchen@via.com.tw>
 *         * Set WM8435 as default for amx_set_alpha.
 *
 * 2009-07-24  Vincent Chen  <vincentchen@via.com.tw>
 *         * Add SoC chip detection code.
 *         * Add struct ge_regs_8430, ge_regs_8510, and ge_regs_8435.
 *         * New architecture with global geinfo removed.
 *
 * 2009-07-13  Vincent Chen  <vincentchen@via.com.tw>
 *         * use ARCH_WM8510_ECO2, ARCH_WM8435.
 *
 * 2009-07-09  Vincent Chen  <vincentchen@via.com.tw>
 *         * Adjust for VT3426 eco2.
 *
 * 2009-06-02  Vincent Chen  <vincentchen@via.com.tw>
 *         * Update for VT3426 eco2 and WM3437.
 *
 * 2008-01-09  Vincent Chen  <vincentchen@via.com.tw>
 *         * Fix some code for GE_MODE_POST.
 *
 * 2008-12-31  Vincent Chen  <vincentchen@via.com.tw>
 *         * Use static semaphore.
 *
 * 2008-12-22  Vincent Chen  <vincentchen@via.com.tw>
 *         * Rename as ge_regs.[c, h] for WM8510.
 *
 * 2008-09-18  Vincent Chen  <vincentchen@via.com.tw>
 *         * Code clean.
 *
 * 2007-10-17  Vincent Chen  <vincentchen@via.com.tw>
 *         * Intial released by Vincent Chen.
 *
 * @author: Vincent Chen < vincentchen@via.com.tw >
 */

#include <linux/delay.h>
#include "ge_regs.h"

/* GE core functions */
#ifndef INLINE
#define INLINE
#endif

#if (GE_MODE == GE_MODE_KERN)
DECLARE_MUTEX(ge_sem);
struct task_struct *ge_sem_owner;

INLINE int ge_sys_init(ge_info_t **geinfo, void *priv)
{
	ge_sem_owner = NULL;

	*geinfo = kmalloc(sizeof(ge_info_t), GFP_KERNEL);

	if (*geinfo)
		(*geinfo)->mmio = (void *)(GE_MMIO_START + GE_MMIO_OFFSET);
	else
		return -1;

	return 0;
}

INLINE int ge_sys_exit(ge_info_t *geinfo, void *priv)
{
	kfree(geinfo);

	return 0;
}

INLINE int ge_get_chip_id(ge_info_t *geinfo, unsigned int *chip_id)
{
	*chip_id = SCC_CHIP_ID;

	return 0;
}

INLINE void ge_wait_sync(ge_info_t *geinfo)
{
	volatile struct ge_regs_8430 *regs;

	if (!geinfo) {
		printk(KERN_ERR "%s: geinfo is null.\n", __func__);
		return;
	}

	regs = geinfo->mmio;

	if (regs->ge_int_en & BIT8) {
		if (regs->ge_status & BIT2) {
			wait_event_interruptible(ge_wq,
				!(regs->ge_status & BIT2));
		}
	} else {
		while (regs->ge_status & BIT2)
			msleep_interruptible(1); /* 1 ms */
		regs->ge_int_flag |= ~0;
	}
}

INLINE int ge_lock(ge_info_t *geinfo)
{
	int ret;

	if (ge_sem_owner == current)
		return -EBUSY;

	ret = down_interruptible(&ge_sem);

	if (ret == 0)
		ge_sem_owner = current;

	return ret;
}

INLINE int ge_unlock(ge_info_t *geinfo)
{
	if (ge_sem_owner == current) {
		ge_sem_owner = NULL;
		up(&ge_sem);
	} else {
		return -EACCES;
	}

	return 0;
}

INLINE int ge_trylock(ge_info_t *geinfo)
{
	int ret;

	ret = down_trylock(&ge_sem);

	if (ret == 0)
		ge_sem_owner = current;

	return ret;
}
#elif (GE_MODE == GE_MODE_USER)
INLINE int ge_sys_init(ge_info_t **geinfo, void *priv)
{
	struct fb_fix_screeninfo fb_fix;
	unsigned int off;

	*geinfo = malloc(sizeof(ge_info_t));

	(*geinfo)->fd = open("/dev/fb0", O_RDWR);
	ioctl((*geinfo)->fd, FBIOGET_FSCREENINFO, &fb_fix);

	/*
	 * linux framebuffer mmap rules:
	 * if offset < smem_len, smem_start was mapped.
	 * if offset => smem_len, mmio_start was mapped.
	 */
	off = (fb_fix.smem_len + 0x0fff) & ~0x0fff;
	(*geinfo)->mmio = mmap(0, fb_fix.mmio_len, PROT_READ|PROT_WRITE,
		MAP_SHARED, (*geinfo)->fd, off);
	(*geinfo)->mmio += GE_MMIO_OFFSET;

	return 0;
}

INLINE int ge_sys_exit(ge_info_t *geinfo, void *priv)
{
	struct fb_fix_screeninfo fb_fix;

	ioctl(geinfo->fd, FBIOGET_FSCREENINFO, &fb_fix);

	geinfo->mmio -= GE_MMIO_OFFSET;

	munmap(geinfo->mmio, fb_fix.mmio_len);
	close(geinfo->fd);

	free(geinfo);

	return 0;
}

INLINE int ge_get_chip_id(ge_info_t *geinfo, unsigned int *chip_id)
{
	return ioctl(geinfo->fd, GEIOGET_CHIP_ID, chip_id);
}

INLINE void ge_wait_sync(ge_info_t *geinfo)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;

	if (regs->ge_int_en & BIT8) {
		ioctl(geinfo->fd, GEIO_WAIT_SYNC, 1);
	} else {
		while (regs->ge_status & BIT2)
			msleep_interruptible(1); /* 1 ms */
		regs->ge_int_flag |= ~0;
	}
}

INLINE int ge_lock(ge_info_t *geinfo)
{
	int fd;
	int ret;

	fd = geinfo->fd;
	ret = ioctl(fd, GEIO_LOCK, 1);

	return ret;
}

INLINE int ge_unlock(ge_info_t *geinfo)
{
	int fd;
	int ret;

	fd = geinfo->fd;
	ret = ioctl(fd, GEIO_LOCK, 0);

	return ret;
}

INLINE int ge_trylock(ge_info_t *geinfo)
{
	int fd;
	int ret;

	fd = geinfo->fd;
	ret = ioctl(fd, GEIO_LOCK, 2);

	return ret;
}
#elif (GE_MODE == GE_MODE_POST)
static ge_info_t geinfo_static;

INLINE int ge_sys_init(ge_info_t **geinfo, void *priv)
{
	*geinfo = &geinfo_static;

	(*geinfo)->fd = 0;
	(*geinfo)->mmio = GE_MMIO_START + GE_MMIO_OFFSET;

	return 0;
}

INLINE int ge_sys_exit(ge_info_t *geinfo, void *priv)
{
	memset(geinfo, 0, sizeof(ge_info_t));

	return 0;
}

INLINE int ge_get_chip_id(ge_info_t *geinfo, unsigned int *chip_id)
{
	*chip_id = SCC_CHIP_ID;

	return 0;
}

INLINE void ge_wait_sync(ge_info_t *geinfo)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;
	regs->ge_int_en = 0; /* disable interrupt */

	while (regs->ge_status & BIT2)
			msleep_interruptible(1); /* 1 ms */
	regs->ge_int_flag |= ~0;
}

INLINE int ge_lock(ge_info_t *geinfo)
{
	return ge_wait_sync(geinfo);
}

INLINE int ge_unlock(ge_info_t *geinfo)
{
	return ge_wait_sync(geinfo);
}

INLINE int ge_trylock(ge_info_t *geinfo)
{
	return ge_wait_sync(geinfo);
}
#endif

/* PUT SYSTEM INDEPENDANT FUNCTIONS HERE */

INLINE int ge_set_pixelformat(ge_info_t *geinfo, unsigned int pixelformat)
{
	volatile struct ge_regs_8430 *regs;
	int ret = 0;

	regs = geinfo->mmio;

	switch (pixelformat) {
	case GEPF_LUT8:
		regs->color_depth = 0;
		break;
	case GEPF_RGB16:
		regs->color_depth = 1;
		regs->hm_sel = 0;
		break;
	case GEPF_RGB555:
		regs->color_depth = 1;
		regs->hm_sel = 1;
		break;
	case GEPF_RGB454:
		regs->color_depth = 1;
		regs->hm_sel = 2;
		break;
	case GEPF_RGB32:
		regs->color_depth = 3;
		break;
	default:
		ret = -1;
	}

	return ret;
}

INLINE int ge_set_destination(ge_info_t *geinfo, ge_surface_t *dst)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;

	regs->des_baddr = dst->addr;
	regs->des_disp_w = dst->xres_virtual - 1;
	regs->des_disp_h = dst->yres_virtual - 1;

	regs->des_x_start = dst->x;
	regs->des_y_start = dst->y;
	regs->des_width = dst->xres - 1;
	regs->des_height = dst->yres - 1;

	return ge_set_pixelformat(geinfo, dst->pixelformat);
}

INLINE int ge_set_source(ge_info_t *geinfo, ge_surface_t *src)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;

	regs->src_baddr = src->addr;
	regs->src_disp_w = src->xres_virtual - 1;
	regs->src_disp_h = src->yres_virtual - 1;

	regs->src_x_start = src->x;
	regs->src_y_start = src->y;
	regs->src_width = src->xres - 1;
	regs->src_height = src->yres - 1;

	return 0;
}

INLINE int ge_set_mask(ge_info_t *geinfo, ge_surface_t *src)
{
	volatile struct ge_regs_8435 *regs;

	regs = geinfo->mmio;

	regs->mask_baddr = src->addr;
	regs->mask_disp_w = src->xres_virtual - 1;
	regs->mask_disp_h = src->yres_virtual - 1;

	regs->mask_x_start = src->x;
	regs->mask_y_start = src->y;
	regs->mask_width = src->xres - 1;
	regs->mask_height = src->yres - 1;

	return 0;
}

INLINE int ge_set_command(ge_info_t *geinfo, unsigned int cmd, unsigned int rop)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;

	regs->ge_command = cmd;
	regs->rop_code = rop;	/* cc = s->d, 0f = p->d */
	regs->ge_eng_en = 1;
	regs->ge_fire = 1;

	return 0;
}

INLINE unsigned int ge_get_color(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	unsigned int color;

	switch (pixfmt) {
	case GEPF_LUT8:
		color = a;
		break;
	case GEPF_RGB16:
		color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
		break;
	case GEPF_RGB555:
		color = ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
		break;
	case GEPF_RGB454:
		color = ((r >> 4) << 9) | ((g >> 3) << 4) | (b >> 4);
		break;
	case GEPF_RGB32:
		color = (a << 24) | (r << 16) | (g << 8) | (b) ;
		break;
	default:
		color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
		break;
	}

	return color;
}

INLINE void ge_set_color(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int color;

	color = ge_get_color(geinfo, r, g, b, a, pixfmt);

	regs = geinfo->mmio;
	regs->pat0_color = color;
}

INLINE void ge_set_sck(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int color;

	color = ge_get_color(geinfo, r, g, b, a, pixfmt);

	regs = geinfo->mmio;
	regs->src_ck = color;
	regs->ck_sel |= BIT2;
}

INLINE void ge_set_dck(ge_info_t *geinfo,
	unsigned int r, unsigned int g, unsigned int b, unsigned int a,
	unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int color;

	color = ge_get_color(geinfo, r, g, b, a, pixfmt);

	regs = geinfo->mmio;
	regs->des_ck = color;
	regs->ck_sel |= BIT3 | BIT1;
}

INLINE int ge_blit(ge_info_t *geinfo)
{
	return ge_set_command(geinfo, GECMD_BLIT, 0xcc);
}

INLINE int ge_fillrect(ge_info_t *geinfo)
{
	return ge_set_command(geinfo, GECMD_BLIT, 0xf0);
}

INLINE int ge_rotate(ge_info_t *geinfo, unsigned int arc)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int chip_id;

	regs = geinfo->mmio;

	/*
	 * Rotation function requires LUT disabled
	 *
	 * VT8430: G2_LUT and G3_LUT have to be disabled.
	 * WM8510: G2_LUT has to be disabled.
	 * VT8435: nothing to do.
	 */

	ge_get_chip_id(geinfo, &chip_id);
	chip_id >>= 16;

	switch (chip_id) {
	case 0x3357:	/* VT8430 */
		regs->g2_lut_en = 0;
		regs->g3_lut_en = 0;
		break;
	case 0x3426:	/* WM8510 */
		regs->g2_lut_en = 0;
		break;
	case 0x3437:	/* WM8435 */
	default:
		/* Nothing to do here... */
		break;
	}

	switch (arc % 360) {
	case 0:
		regs->rotate_mode = 0;
		break;
	case 90:
		regs->rotate_mode = 1;
		break;
	case 180:
		regs->rotate_mode = 2;
		break;
	case 270:
		regs->rotate_mode = 3;
		break;
	default:
		break;
	}
	ge_set_command(geinfo, GECMD_ROTATE, 0);

	ge_wait_sync(geinfo); /* Required. */

	return 0;
}

INLINE int ge_mirror(ge_info_t *geinfo, int mode)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;
	regs->mirror_mode = mode & BIT0;

	ge_set_command(geinfo, GECMD_MIRROR, 0);

	return 0;
}

/* AMX barebone functions */

INLINE int amx_show_surface(ge_info_t *geinfo,
	int id, ge_surface_t *s, int x, int y)
{
	volatile struct ge_regs_8430 *regs;
	volatile unsigned int *gov_regs;
	unsigned int cd; /* color depth */
	unsigned int hm; /* hi color mode */

	cd = (s->pixelformat) >> 4;
	hm = (s->pixelformat) & 0xf;

	regs = geinfo->mmio;

	gov_regs = geinfo->mmio;
	gov_regs -= 64;

	if (gov_regs[7]) {
		/* if gov was initialized ... */
		regs->disp_x_end = gov_regs[7] & 0x7ff;
		regs->disp_y_end = gov_regs[7] >> 16;
	} else {
		/* if gov is not initialized yet ... */
		regs->disp_x_end = s->xres_virtual - 1;
		regs->disp_y_end = s->yres_virtual - 1;
	}

	if (id == 0) {
		/* G1 */
		regs->g1_cd = cd;
		regs->g1_amx_hm = hm;

		/* If FG being used, use BG. */
		if (regs->g1_fb_sel == 0) {
			regs->g1_bg_addr = s->addr;
			regs->g1_fb_sel = 1;
		} else {
			regs->g1_fg_addr = s->addr;
			regs->g1_fb_sel = 0;
		}

		regs->g1_fbw = s->xres_virtual;
		regs->g1_hcrop = s->x;
		regs->g1_vcrop = s->y;
		regs->g1_x_start =  x;
		regs->g1_x_end = x + s->xres - 1;
		regs->g1_y_start = y;
		regs->g1_y_end = y + s->yres - 1;
		regs->g1_amx_en = 1;
	} else {
		/* G2 */
		regs->g2_cd = cd;
		regs->g2_amx_hm = hm;

		/* If FG being used, use BG. */
		if (regs->g2_fb_sel == 0) {
			regs->g2_bg_addr = s->addr;
			regs->g2_fb_sel = 1;
		} else {
			regs->g2_fg_addr = s->addr;
			regs->g2_fb_sel = 0;
		}

		regs->g2_fbw = s->xres_virtual;
		regs->g2_hcrop = s->x;
		regs->g2_vcrop = s->y;
		regs->g2_x_start =  x;
		regs->g2_x_end = x + s->xres - 1;
		regs->g2_y_start = y;
		regs->g2_y_end = y + s->yres - 1;
		regs->g2_amx_en = 1;
	}

	return 0;
}

INLINE int amx_get_surface(ge_info_t *geinfo, int id, ge_surface_t *s)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int cd; /* color depth */
	unsigned int hm; /* hi color mode */

	regs = geinfo->mmio;

	if (id == 0) {
		/* G1 */
		cd = regs->g1_cd;
		hm = regs->g1_amx_hm;

		s->addr = regs->g1_fb_sel ?
			regs->g1_bg_addr : regs->g1_fg_addr;
		s->x = regs->g1_hcrop;
		s->y = regs->g1_vcrop;
		s->xres = regs->g1_x_end - regs->g1_x_start + 1;
		s->yres = regs->g1_y_end - regs->g1_y_start + 1;
		s->xres_virtual = regs->g1_fbw;
		s->yres_virtual = s->yres;
	} else {
		/* G2 */
		cd = regs->g2_cd;
		hm = regs->g2_amx_hm;

		s->addr = regs->g2_fb_sel ?
			regs->g2_bg_addr : regs->g2_fg_addr;
		s->x = regs->g2_hcrop;
		s->y = regs->g2_vcrop;
		s->xres = regs->g2_x_end - regs->g2_x_start + 1;
		s->yres = regs->g2_y_end - regs->g2_y_start + 1;
		s->xres_virtual = regs->g2_fbw;
		s->yres_virtual = s->yres;
	}

	s->pixelformat = GEPF_FMT(cd, hm);

	return 0;
}

INLINE int amx_hide_surface(ge_info_t *geinfo, int id)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;

	switch (id) {
	case 0:
		regs->g1_amx_en = 0;
		break;
	case 1:
		regs->g2_amx_en = 0;
		break;
	default:
		break;
	}
	return 0;
}

INLINE int amx_set_colorkey(ge_info_t *geinfo, int id, int enable,
	unsigned int r, unsigned int g, unsigned int b, unsigned int pixfmt)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;

	switch (pixfmt) {
	case GEPF_RGB16:
		r &= ~0x7;
		g &= ~0x3;
		b &= ~0x7;
		break;
	case GEPF_RGB555:
		r &= ~0x7;
		g &= ~0x7;
		b &= ~0x7;
		break;
	case GEPF_RGB454:
		r &= ~0xf;
		g &= ~0x7;
		b &= ~0xf;
		break;
	}

	switch (id) {
	case 0:
		regs->g1_ck_en = enable;
		regs->g1_c_key = enable ?
			((r << 16) | (g << 8) | (b)) : 0;
		break;
	case 1:
		regs->g2_ck_en = enable;
		regs->g2_c_key = enable ?
			((r << 16) | (g << 8) | (b)) : 0;
		break;
	default:
		break;
	}

	return 0;
}

INLINE int amx_set_alpha_vt8430(ge_info_t *geinfo, int id, unsigned int alpha)
{
	volatile struct ge_regs_8430 *regs;

	/*
	 * GE = G1 * ALPHA_1 + G2 * (1 - ALPHA_1)
	 * VOUT = GE * (1 - ALPHA_2) + VPU * ALPHA2
	 */

	/*
	 *	G2_HIT	G1_HIT	G2_CK	G1_CK	ALPHA_1	ALPHA_2	VT8430
	 *	0	0	0	0	X (FF)	FF	Yes
	 *	0	1	0	0	FF	00	Yes
	 *	1	0	0	0	00	00	Yes
	 *	1	1	0	0	00	00	Yes
	 *
	 *	0	1	0	1	X (00)	FF	Yes
	 *	1	0	1	0	00	FF	No
	 *	1	1	0	1	00	00	Yes
	 *	1	1	1	0	FF	00	Yes
	 *	1	1	1	1	X	FF	No
	 */

	/*
	 *	GE AMX setting for VT8430
	 *
	 *	{MSB,...,LSB}
	 *
	 *	GE_FIX_APA = {00,00,ff,ff}
	 *	GE_FIX2_APA = {00,00,00,ff}
	 *	GE_AMX_CTL = {N,N,N,N,0,0,0,0}
	 *	GE_AMX2_CTL = {N,N,X,1,1,0,0,0}
	 *	GE_CK_APA = {N,N,N,N,N,0,1,0}
	 */

	alpha &= 0xff;

	regs = geinfo->mmio;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_fix2_apa = 0xff;
	regs->ge_amx_ctl = 0;
	regs->ge_amx2_ctl = BIT4 | BIT3;
	regs->ge_ck_apa = BIT1;

	return 0;
}

INLINE int amx_set_alpha_wm8510_eco2(ge_info_t *geinfo, unsigned int alpha)
{
	volatile struct ge_regs_8510 *regs;

	alpha &= 0xff;

	regs = geinfo->mmio;

	regs->ge_amx_ctl = 0;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_ck_apa = BIT1;

	regs->ge_amx2_ctl = 0xf;
	regs->ge_fix2_apa = 0xff;
	regs->ge_ck_apa |= BIT8 | BIT7 | BIT5 | BIT3;

	return 0;
}

INLINE int amx_set_alpha_wm8435(ge_info_t *geinfo, unsigned int alpha)
{
	volatile struct ge_regs_8435 *regs;

	alpha &= 0xff;

	regs = geinfo->mmio;

	regs->ge_amx_ctl = 0;
	regs->ge_fix_apa = 0xffff | alpha << 24;
	regs->ge_ck_apa = BIT9;

	regs->ge_amx2_ctl = 0x76;
	regs->ge_fix2_apa = 0xff;
	regs->ge_ck2_apa = BIT11 | BIT7 | BIT3;

	return 0;
}

INLINE int amx_set_alpha(ge_info_t *geinfo, int id, unsigned int alpha)
{
	unsigned int chip_id;
	unsigned int chip_version;

	ge_get_chip_id(geinfo, &chip_id);
	chip_version = chip_id & 0xffff;

	chip_id >>= 16;

	switch (chip_id) {
	case 0x3357:	/* VT8430 */
		amx_set_alpha_vt8430(geinfo, id, alpha);
		break;
	case 0x3426:	/* WM8510 */
		if (chip_version > 0x0102) /* WM8510 A1 */
			amx_set_alpha_wm8510_eco2(geinfo, alpha);
		else
			amx_set_alpha_vt8430(geinfo, id, alpha);
		break;
	case 0x3437:	/* WM8435 */
	default:
		amx_set_alpha_wm8435(geinfo, alpha);
		break;
	}

	return 0;
}

INLINE int amx_sync(ge_info_t *geinfo)
{
	volatile struct ge_regs_8430 *regs;

	regs = geinfo->mmio;
	regs->ge_reg_sel = 0; /* select level1 registers */
	regs->ge_reg_upd = 1; /* update level2 registers */

	return 0;
}

INLINE int amx_set_csc(ge_info_t *geinfo, int table_id)
{
	volatile struct ge_regs_8510 *regs;

	regs = geinfo->mmio;

	switch (table_id) {
	case AMX_CSC_DEFAULT:
		break;
	case AMX_CSC_SDTV_16_235:
		regs->c1_coef = 0x132;
		regs->c2_coef = 0x259;
		regs->c3_coef = 0x75;
		regs->c4_coef = 0x1f50;
		regs->c5_coef = 0x1ea5;
		regs->c6_coef = 0x20b;
		regs->c7_coef = 0x20b;
		regs->c8_coef = 0x1e4a;
		regs->c9_coef = 0x1fab;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_SDTV_0_255:
		regs->c1_coef = 0x107;
		regs->c2_coef = 0x204;
		regs->c3_coef = 0x64;
		regs->c4_coef = 0x1f68;
		regs->c5_coef = 0x1ed6;
		regs->c6_coef = 0x1c2;
		regs->c7_coef = 0x1c2;
		regs->c8_coef = 0x1e78;
		regs->c9_coef = 0x1fb7;
		regs->coef_i = 0x20;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_HDTV_16_235:
		regs->c1_coef = 0xda;
		regs->c2_coef = 0x2dc;
		regs->c3_coef = 0x4a;
		regs->c4_coef = 0x1f88;
		regs->c5_coef = 0x1e6d;
		regs->c6_coef = 0x20b;
		regs->c7_coef = 0x20b;
		regs->c8_coef = 0x1e25;
		regs->c9_coef = 0x1fd0;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_HDTV_0_255:
		regs->c1_coef = 0xbb;
		regs->c2_coef = 0x275;
		regs->c3_coef = 0x3f;
		regs->c4_coef = 0x1f99;
		regs->c5_coef = 0x1ea6;
		regs->c6_coef = 0x1c2;
		regs->c7_coef = 0x1c2;
		regs->c8_coef = 0x1e67;
		regs->c9_coef = 0x1fd7;
		regs->coef_i = 0x20;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_JFIF_0_255:
		regs->c1_coef = 0x132;
		regs->c2_coef = 0x259;
		regs->c3_coef = 0x75;
		regs->c4_coef = 0x1f53;
		regs->c5_coef = 0x1ead;
		regs->c6_coef = 0x200;
		regs->c7_coef = 0x200;
		regs->c8_coef = 0x1e53;
		regs->c9_coef = 0x1fad;
		regs->coef_i = 0x0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_SMPTE_170M:
		regs->c1_coef = 0x132;
		regs->c2_coef = 0x259;
		regs->c3_coef = 0x75;
		regs->c4_coef = 0x1f50;
		regs->c5_coef = 0x1ea5;
		regs->c6_coef = 0x20b;
		regs->c7_coef = 0x20b;
		regs->c8_coef = 0x1e4a;
		regs->c9_coef = 0x1fab;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	case AMX_CSC_SMPTE_240M:
		regs->c1_coef = 0xd9;
		regs->c2_coef = 0x2ce;
		regs->c3_coef = 0x59;
		regs->c4_coef = 0x2f89;
		regs->c5_coef = 0x1e77;
		regs->c6_coef = 0x200;
		regs->c7_coef = 0x200;
		regs->c8_coef = 0x1e38;
		regs->c9_coef = 0x1fc8;
		regs->coef_i = 0;
		regs->coef_j = 0x100;
		regs->coef_k = 0x100;
		break;
	}

	return 0;
}

