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

/* *shudder* */
#include "../../char/wmt-pwm.h"

// #including "govrh.h" makes a huge mess
extern void govrh_suspend(int);
extern void govrh_resume(int);

/**
 * viaGL API reference
 *
 * extern int ge_sys_init(ge_info_t **geinfo, void *priv);
 * extern int ge_sys_exit(ge_info_t *geinfo, void *priv);
 * extern int ge_lock(ge_info_t *geinfo);
 * extern int ge_unlock(ge_info_t *geinfo);
 * extern int ge_trylock(ge_info_t *geinfo);
 * extern void ge_wait_sync(ge_info_t *geinfo);
 * extern void WAIT_PXD_INT(void);
 * extern int ge_set_pixelformat(ge_info_t *geinfo, u32 pixelformat);
 * extern int ge_set_destination(ge_info_t *geinfo, ge_surface_t *dst);
 * extern int ge_set_source(ge_info_t *geinfo, ge_surface_t *src);
 * extern int ge_set_command(ge_info_t *geinfo, u32 cmd, u32 rop);
 * extern void ge_set_color(ge_info_t *geinfo,
 *         u32 r, u32 g, u32 b, u32 a, u32 pixfmt);
 * extern void ge_set_sck(ge_info_t *geinfo,
 *         u32 r, u32 g, u32 b, u32 a, u32 pixfmt);
 * extern void ge_set_dck(ge_info_t *geinfo,
 *         u32 r, u32 g, u32 b, u32 a, u32 pixfmt);
 * extern int ge_blit(ge_info_t *geinfo);
 * extern int ge_fillrect(ge_info_t *geinfo);
 * extern int ge_rotate(ge_info_t *geinfo, u32 arc);
 * extern int ge_mirror(ge_info_t *geinfo, int mode);
 * extern int amx_show_surface(ge_info_t *geinfo, int id,
 *         ge_surface_t *s, int x, int y)
 * extern int amx_get_surface(ge_info_t *geinfo, int id, ge_surface_t *s)
 * extern int amx_hide_surface(ge_info_t *geinfo, int id)
 * extern int amx_set_colorkey(ge_info_t *geinfo, int id,
 *         int enable, u32 r, u32 g, u32 b, u32 pixfmt);
 * extern int amx_set_alpha(ge_info_t *geinfo, int id, u32 alpha)
 * extern int amx_sync(ge_info_t *geinfo)
 */

#include <linux/sched.h>
#include "ge_accel.h"
#include "vpp.h"


#ifdef DEBUG
#define DPRINTK(fmt, args...) \
	printk(KERN_WARN "%s: " fmt, __func__ , ## args)
#define ENTER() DPRINTK("Enter %s, file:%s line:%d\n", \
	__func__, __FILE__, __LINE__)
#define LEAVE() DPRINTK("Exit %s, file:%s line:%d\n",\
	__func__, __FILE__, __LINE__)
#else
#define DPRINTK(fmt, args...)
#define ENTER()
#define LEAVE()
#endif

DECLARE_WAIT_QUEUE_HEAD(ge_wq);
static ge_info_t *geinfo;


/**************************
 *    Export functions    *
 **************************/

/**
 * ge_vram_addr - Detect a valid address for VRAM.
 *
 */
#define M(x) ((x)<<20)
unsigned int ge_vram_addr(u32 size)
{
	unsigned int memsize = (num_physpages << PAGE_SHIFT);

	if (memsize > M(256)) {         /* 512M */
		memsize = M(512);
	} else if (memsize > M(128)) {  /* 256M */
		memsize = M(256);
	} else if (memsize > M(64)) {   /* 128M */
		memsize = M(128);
	} else if (memsize > M(32)) {   /* 64M */
		memsize = M(64);
	} else if (memsize > M(16)) {   /* 32M */
		memsize = M(32);
	} else {
		memsize = M(0);
	}

	DPRINTK("GE has detected that RAM size is %d MB \n", memsize>>20);
	return (memsize - M(size));
}
EXPORT_SYMBOL(ge_vram_addr);

#include <linux/semaphore.h>
#include <linux/interrupt.h>

static irqreturn_t ge_interrupt(int irq, void *dev_id,
	struct pt_regs *regs)
{
	volatile struct ge_regs_8430 *ge_regs;

	ge_regs = geinfo->mmio;

	/* Reset if GE timeout. */
	if ((ge_regs->ge_int_en & BIT9) && (ge_regs->ge_int_flag & BIT9)) {
		printk("%s: GE Engine Time-Out Status! \n", __func__);
		ge_regs->ge_eng_en = 0;
		ge_regs->ge_eng_en = 1;
		while (ge_regs->ge_status & (BIT5 | BIT4 | BIT3));
	}

	/* Clear GE interrupt flags. */
	ge_regs->ge_int_flag |= ~0;

	if (ge_regs->ge_status == 0)
		wake_up_interruptible(&ge_wq);
	else
		printk(KERN_ERR "%s: Incorrect GE status (0x%x)! \n",
			__func__, ge_regs->ge_status);

	return IRQ_HANDLED;
}

unsigned char* g_current_phy = 0;

struct fb_stauts g_fb_stauts;

/**
 * ge_init - Initial and display framebuffer.
 *
 * Fill the framebuffer with a default color, back.
 * Display the framebuffer using GE AMX.
 *
 * Although VQ is supported in design, I just can't find any benefit
 * from VQ. It wastes extra continuous physical memory, and runs much
 * slower than direct register access. Moreover, the source code
 * becomes more complex and is hard to maintain. Accessing VQ from
 * the user space is also a nightmare. In brief, the overhead of VQ makes
 * it useless. In order to gain the maximum performance
 * from GE and to keep the driver simple, I'm going to stop using VQ.
 * I will use VQ only when it is necessary.
 *
 * @info is the fb_info provided by framebuffer driver.
 * @return zero on success.
 */
int ge_init(struct fb_info *info)
{
	static int boot_init; /* boot_init = 0 */
	volatile struct ge_regs_8430 *regs;
	struct fb_var_screeninfo *var;
	unsigned int offset;
	unsigned int chip_id;
	unsigned int ge_irq;
	ge_surface_t s;
	ge_surface_t cs;

	/*
	 * Booting time initialization
	 */
	if (boot_init != 0x1) {
		ge_sys_init(&geinfo, info);

		regs = geinfo->mmio;

		DPRINTK(KERN_INFO "ge: iomem region at 0x%lx, mapped to 0x%x, "
			"using %d, total %d\n",
			info->fix.mmio_start, (u32) geinfo->mmio,
			info->fix.mmio_len, info->fix.mmio_len);

		 /* 0x00000 (fastest) - 0x30003 (slowest) */
		regs->ge_delay = 0x10001;

		ge_get_chip_id(geinfo, &chip_id);
		chip_id >>= 16;

		switch (chip_id) {
		case 0x3357:		/* VT8430 */
			ge_irq = 66;
			break;
		case 0x3426:		/* WM8510 */
			ge_irq = 85;	/* IRQ_NA12_6 */
			break;
		case 0x3437:		/* WM8435 */
			ge_irq = 83;	/* IRQ_VPP_IRQ7 */
			break;
		case 0x3429:		/* WM8425 */
			ge_irq = 71;	/* IRQ_VPP_IRQ7 */
			break;
		default:
			ge_irq = 0;
			break;
		}

		/*
		 * GE interrupt is enabled by default.
		 */
		if (ge_irq) {
			request_irq(ge_irq, ge_interrupt, IRQF_DISABLED,
				"ge", NULL);
			regs->ge_int_en = BIT8 | BIT9;
		}

		REG_SET32(0xd8050308, 0x00000001); /* Turn on GE by GOV */

		amx_set_csc(geinfo, AMX_CSC_JFIF_0_255);
		amx_set_alpha(geinfo, 0, 0xff);
		boot_init = 1;
	}

	var = &info->var;

	offset = (var->yoffset * var->xres_virtual + var->xoffset);
	offset *= var->bits_per_pixel >> 3;

	s.addr = info->fix.smem_start + offset;
	s.xres = info->var.xres;
	s.yres = info->var.yres;
	s.xres_virtual = info->var.xres_virtual;
	s.yres_virtual = info->var.yres;
	s.x = 0;
	s.y = 0;

	switch (info->var.bits_per_pixel) {
	case 8:
		s.pixelformat = GEPF_LUT8;
		break;
	case 16:
		if ((info->var.red.length == 5) &&
			(info->var.green.length == 6) &&
			(info->var.blue.length == 5)) {
			s.pixelformat = GEPF_RGB16;
		} else if ((info->var.red.length == 5) &&
			(info->var.green.length == 5) &&
			(info->var.blue.length == 5)) {
			s.pixelformat = GEPF_RGB555;
		} else {
			s.pixelformat = GEPF_RGB454;
		}
		break;
	case 32:
		s.pixelformat = GEPF_RGB32;
		break;
	default:
		s.pixelformat = GEPF_RGB16;
		break;
	}

	amx_get_surface(geinfo, 0, &cs);

	if (!g_vpp.govrh_preinit && memcmp(&cs, &s, sizeof(ge_surface_t))) {
		ge_lock(geinfo);
		ge_set_color(geinfo, 0, 0, 0, 0, s.pixelformat);
		ge_set_destination(geinfo, &s);
		ge_set_pixelformat(geinfo, s.pixelformat);
		ge_set_command(geinfo, GECMD_BLIT, 0xf0);
		ge_unlock(geinfo);
	}

	amx_show_surface(geinfo, 0, &s, 0, 0); /* id:0, x:0, y:0 */
	amx_sync(geinfo);

	return 0;
}

/**
 * ge_exit - Disable GE.
 *
 * No memory needs to be released here.
 * Turn off the AMX to stop displaying framebuffer.
 * Update the index of MMU.
 *
 * @info is fb_info from fbdev.
 * @return zero on success.
 */
int ge_exit(struct fb_info *info)
{
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
	ge_sys_exit(geinfo, info);

	return 0;
}

static int get_args(unsigned int *to, void *from, int num)
{
	unsigned int count;

	count = sizeof(unsigned int);
	count *= num;

	if (copy_from_user(to, from, count)) {
		printk(KERN_ERR "%s: copy_from_user failure\n", __func__);
		return  -EFAULT;
	}

	return 0;
}


/**
 * ge_ioctl - Extension for fbdev ioctl.
 *
 * Not quite usable now.
 *
 * @return zero on success.
 */
int ge_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	volatile struct ge_regs_8430 *regs;
	int ret = 0;
	unsigned int chip_id;
	unsigned int args[8];

	void __user *argp = (void __user *)arg;

	struct mouse_pos {
		int x;
		int y;
	};
	struct mouse_pos mp;

	regs = geinfo->mmio;

	switch (cmd) {
	case GEIOSET_AMX_EN:
		if (arg) {
			/* Only enable G1 */
			regs->g1_amx_en = 1;
			regs->g2_amx_en = 0;
			regs->ge_reg_upd = 1;
		} else {
			/* Disable G1 and G2 */
			regs->g1_amx_en = 0;
			regs->g2_amx_en = 0;
			regs->ge_reg_upd = 1;
		}
		break;
	case GEIO_ALPHA_BLEND:
	case GEIOSET_OSD:
		ge_alpha_blend((u32)arg);
		break;
	case GEIO_WAIT_SYNC:
		ge_wait_sync(geinfo);
		break;
	case GEIO_LOCK:
		switch (arg) {
		case 0:
			ret = ge_unlock(geinfo);
			break;
		case 1:
			ret = ge_lock(geinfo);
			break;
		default:
			ret = ge_trylock(geinfo);
			break;
		}
		break;
	case GEIOGET_CHIP_ID:
		ge_get_chip_id(geinfo, &chip_id);
		copy_to_user((void *)arg, (void *) &chip_id,
			sizeof(unsigned int));
		break;
	case GEIO_ROTATE:
		ret = get_args(args, (void *) arg, 6);
		if (ret == 0) {
			ge_simple_rotate(args[0], args[1], args[2], args[3],
					 args[4], args[5]);
		}
		break;
	default:
		ret = -1;
	}

	return ret;
}

int ge_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
	unsigned transp, struct fb_info *info)
{
	volatile struct ge_regs_8430 *regs;
	unsigned int chip_id;
	unsigned int color;

	ge_get_chip_id(geinfo, &chip_id);
	chip_id >>= 16;

	if (chip_id == 0x3357) { /* VT8430 */
		red   >>= 8;
		green >>= 8;
		blue  >>= 8;

		color = red;
		color <<= 8;
		color |= green;
		color <<= 8;
		color |= blue;

		regs = geinfo->mmio;
		regs->g1_lut_en = 1;
		regs->g1_lut_adr = regno;
		regs->g1_lut_dat = color;
	}

	return 0;
}

int wait_vsync(void)
{
	const int has_vbie = 1;

	if (has_vbie) {
		REG_VAL32(0xd8050f04) |= 0x4; /* VBIE */

		while (!(REG_VAL32(0xd8050f04) & 0x4))
			msleep_interruptible(10); /* 10 ms */
	}

	return 0;
}

int ge_sync(struct fb_info *info)
{
	ge_wait_sync(geinfo);

	return 0;
}

int ge_release(struct fb_info *info)
{
	if (ge_sem_owner == current)
		return ge_unlock(geinfo);

	return 0;
}

#define CONFIG_AMX_CROP_EN 0

/**
 * ge_pan_display - Pans the display.
 *
 * Pan (or wrap, depending on the `vmode' field) the display using the
 * `xoffset' and `yoffset' fields of the `var' structure.
 * If the values don't fit, return -EINVAL.
 *
 * @var: frame buffer variable screen structure
 * @info: frame buffer structure that represents a single frame buffer
 *
 * Returns negative errno on error, or zero on success.
 */
int ge_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	ge_surface_t s;
	unsigned int offset;

	DPRINTK("xoff = %d, yoff = %d, xres = %d, yres = %d \n",
	       __func__, var->xoffset, var->yoffset,
	       info->var.xres, info->var.yres);

	if ((var->xoffset + info->var.xres > info->var.xres_virtual) ||
	    (var->yoffset + info->var.yres > info->var.yres_virtual)) {
		/* Y-pan is used in most case.
		 * So please make sure that yres_virtual is
		 * greater than (yres + yoffset).
		 */
		printk(KERN_ERR "%s: out of range \n", __func__);
		return -EINVAL;
	}
#if CONFIG_AMX_CROP_EN
	s.addr = info->fix.smem_start;
	s.xres = var->xres;
	s.yres = var->yres;
	s.xres_virtual = var->xres_virtual;
	s.yres_virtual = var->yres_virtual;
	s.x = var->xoffset;
	s.y = var->yoffset;
#else
	s.addr = info->fix.smem_start;
	s.xres = var->xres;
	s.yres = var->yres;
	s.xres_virtual = var->xres_virtual;
	s.yres_virtual = var->yres_virtual;
	s.x = 0;
	s.y = 0;
	offset = var->yoffset * var->xres_virtual + var->xoffset;
	offset *= var->bits_per_pixel >> 3;
	s.addr += offset;
#endif
#ifdef SW_CURSOR_FOR_ANDROID
	// back up the cursor info, we restore the bitmap of frone buffer, 
	// which become back buffer after "PAN", after the "PAN" actually finished.
	backup_cursor_info();
		
	g_current_phy = (unsigned char *) (info->screen_base + offset);	

	// this if statement is important.
	if (g_cursor_info.b_saved && is_mouse_enable()) {
		// save the current bitmap in the new position
		// (-1, -1), means don't update the position.
		backup_cursor_area(info, -1,-1);

		// modify the current bitmap to cursor
		draw_cursor(info);
	}
#endif

	switch (var->bits_per_pixel) {
	case 8:
		s.pixelformat = GEPF_LUT8;
		break;
	case 16:
		if ((info->var.red.length == 5) &&
			(info->var.green.length == 6) &&
			(info->var.blue.length == 5)) {
			s.pixelformat = GEPF_RGB16;
		} else if ((info->var.red.length == 5) &&
			(info->var.green.length == 5) &&
			(info->var.blue.length == 5)) {
			s.pixelformat = GEPF_RGB555;
		} else {
			s.pixelformat = GEPF_RGB454;
		}
		break;
	case 32:
		s.pixelformat = GEPF_RGB32;
		break;
	default:
		s.pixelformat = GEPF_RGB16;
		break;
	}

	/*
	if (var->activate != FB_ACTIVATE_NOW) {
		vpp_wait_vsync();
	}
	*/

	amx_show_surface(geinfo, 0, &s, 0, 0); /* id:0, x:0, y:0 */
	amx_sync(geinfo);

	return 0;
}

/**
 *  ge_blank - for APM
 *
 */
int ge_blank(int mode, struct fb_info *info)
{
	volatile struct ge_regs_8430 *regs;
    int i;

	/* Disable FB_BLANK due to the buggy VT8430 APM. */
//	return 0;

	ge_wait_sync(geinfo);

	regs = geinfo->mmio;

	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		REG_SET32(0xd8050308, 0x00000000); /* Turn off GE by GOV */

		regs->g1_amx_en = 0;	/* G1 AMX disable */
		regs->g2_amx_en = 0;	/* G2 AMX disable */
		regs->ge_reg_upd = 1;	/* register update */
        pwm_set_enable(0,0);
        for (i=0; i<3; i++)
            govrh_suspend(i);
		break;
	case FB_BLANK_UNBLANK:
        for (i=0; i<3; i++)
            govrh_resume(i);
		regs->g1_amx_en = 1;	/* G1 AMX enable */
		regs->ge_reg_upd = 1;	/* register update */

		REG_SET32(0xd8050308, 0x00000001); /* Turn on GE by GOV */
        pwm_set_enable(0,1);
		break;
	default:
		break;
	}

	return 0;
}

/**
 * ge_alpha_blend - Set alpha and fill transparent color to the RGB screen.
 *
 * The color consists of A, R, G, B. The screen is opaque while alpha{A} equals
 * to zero. The alpha blending formula is as follow:
 * DISPLAY = A*G1 + (0xFF - A)*VPU.
 * Please note that all colors on the G1 is effected by alpha
 * except the transparent color{R,G,B}.
 *
 * @param color is the transparency color of {A,R,G,B}
 */

void ge_alpha_blend(unsigned int color)
{
	ge_surface_t s;
	u8 a, r, g, b;

	a = (u8)((color >> 24) & 0xff);
	r = (u8)((color >> 16) & 0xff);
	g = (u8)((color >> 8) & 0xff);
	b = (u8)(color & 0xff);

	/* Set transparency */
	amx_get_surface(geinfo, 0, &s);

	switch (s.pixelformat) {
	case GEPF_RGB16:
		/* 5:6:5 => 8:8:8 */
		r = (r >> 3) << 3;
		g = (g >> 2) << 2;
		b = (b >> 3) << 3;
		break;
	case GEPF_RGB555:
		/* 5:5:5 => 8:8:8 */
		r = (r >> 3) << 3;
		g = (g >> 3) << 3;
		b = (b >> 3) << 3;
		break;
	case GEPF_RGB454:
		/* 4:5:4 => 8:8:8 */
		r = (r >> 4) << 4;
		g = (g >> 3) << 3;
		b = (b >> 4) << 4;
		break;
	default:
		break;
	}

	ge_wait_sync(geinfo);

    //Fix Android SurfaceFlinger layer lock timeout issue
#define VICYUAN_FIX_LAYER_LOCK_TIMEOUT
    
	if (a) {
        #ifndef VICYUAN_FIX_LAYER_LOCK_TIMEOUT
            amx_show_surface(geinfo, 0, &s, 0, 0);
        #endif
        
		amx_set_colorkey(geinfo, 0, 1, r, g, b, s.pixelformat);
	} else {
        #ifndef VICYUAN_FIX_LAYER_LOCK_TIMEOUT
            amx_hide_surface(geinfo, 0);
        #endif
		amx_set_colorkey(geinfo, 0, 0, r, g, b, s.pixelformat);
	}
	amx_sync(geinfo);

#ifndef VICYUAN_FIX_LAYER_LOCK_TIMEOUT
	ge_lock(geinfo);

	ge_set_color(geinfo, r, g, b, 0, s.pixelformat);
	ge_set_destination(geinfo, &s);
	ge_set_pixelformat(geinfo, s.pixelformat);
	ge_set_command(geinfo, GECMD_BLIT, 0xf0);

	ge_unlock(geinfo);
#endif
}

void ge_simple_rotate(unsigned int phy_src, unsigned int phy_dst,
	int width, int height, int bpp, int arc)
{
	volatile struct ge_regs_8430 *regs;
	ge_surface_t s, d;
	unsigned int chip_id;
	unsigned int g1_lut_en = 0;
	unsigned int g2_lut_en = 0;

	regs = geinfo->mmio;

	if (arc == 0) {
		ge_simple_blit(phy_src, phy_dst, width, height, bpp);
		return;
	}

	switch (bpp) {
	case 8:
		s.pixelformat = GEPF_LUT8;
		d.pixelformat = GEPF_LUT8;
		break;
	case 16:
		s.pixelformat = GEPF_RGB16;
		d.pixelformat = GEPF_RGB16;
		break;
	case 32:
		s.pixelformat = GEPF_RGB32;
		d.pixelformat = GEPF_RGB32;
		break;
	default:
		/* Not supported */
		return;
	}

	s.addr = phy_src;
	s.x = 0;
	s.y = 0;
	s.xres = width;
	s.yres = height;
	s.xres_virtual = width;
	s.yres_virtual = height;

	d.addr = phy_dst;
	d.x = 0;
	d.y = 0;

	switch (arc) {
	case 90:
	case 270:
		d.xres = height;
		d.yres = width;
		d.xres_virtual = height;
		d.yres_virtual = width;
		break;
	default:
		d.xres = width;
		d.yres = height;
		d.xres_virtual = width;
		d.yres_virtual = height;
		break;
	}

	ge_get_chip_id(geinfo, &chip_id);
	chip_id >>= 16;


	/* Rotate */
	ge_lock(geinfo);

	switch (chip_id) {
	case 0x3357:		/* VT8430 */
	case 0x3400:		/* VT8500 */
	case 0x3426:		/* WM8510 */
		g1_lut_en = regs->g1_lut_en;
		g2_lut_en = regs->g2_lut_en;
		regs->g1_lut_en = 0;
		regs->g2_lut_en = 0;
		break;
	default:
		break;
	}

	ge_set_source(geinfo, &s);
	ge_set_destination(geinfo, &d);
	ge_set_pixelformat(geinfo, s.pixelformat);
	ge_rotate(geinfo, arc);
	ge_wait_sync(geinfo);

	switch (chip_id) {
	case 0x3357:		/* VT8430 */
	case 0x3400:		/* VT8500 */
	case 0x3426:		/* WM8510 */
		regs->g1_lut_en = g1_lut_en;
		regs->g2_lut_en = g2_lut_en;
		break;
	default:
		break;
	}

	ge_unlock(geinfo);
}

void ge_simple_blit(unsigned int phy_src, unsigned int phy_dst,
	int width, int height, int bpp)
{
	ge_surface_t s, d;

	switch (bpp) {
	case 8:
		s.pixelformat = GEPF_LUT8;
		d.pixelformat = GEPF_LUT8;
		break;
	case 16:
		s.pixelformat = GEPF_RGB16;
		d.pixelformat = GEPF_RGB16;
		break;
	case 32:
		s.pixelformat = GEPF_RGB32;
		d.pixelformat = GEPF_RGB32;
		break;
	default:
		/* Not supported */
		return;
	}

	s.addr = phy_src;
	s.x = 0;
	s.y = 0;
	s.xres = width;
	s.yres = height;
	s.xres_virtual = width;
	s.yres_virtual = height;

	d.addr = phy_dst;
	d.x = 0;
	d.y = 0;
	d.xres = width;
	d.yres = height;
	d.xres_virtual = width;
	d.yres_virtual = height;

	/* Blit */
	ge_lock(geinfo);

	ge_set_source(geinfo, &s);
	ge_set_destination(geinfo, &d);
	ge_set_pixelformat(geinfo, s.pixelformat);
	ge_blit(geinfo);
	ge_wait_sync(geinfo);

	ge_unlock(geinfo);
}


