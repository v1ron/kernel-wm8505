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
 * ChangeLog
 *
 * 2009-11-10  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Add video=gefb:vtotal:N.
 *
 * 2009-09-04  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Update gefb_release.
 *     * Default color depth is RGB32.
 *
 * 2009-08-27  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Set WM3429 with the same configuration as WM3426.
 *
 * 2009-08-17  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Auto select video memory size by chip id.
 *
 * 2009-08-12  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Use vpp_get_info for best display resolution.
 *
 * 2009-07-24  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Update for VT8430/WM8510/WM8435.
 *
 * 2008-12-31  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * Add support for fb_sync.
 *
 * 2008-12-30  Vincent Chen  <vincentchen@wondermedia.com.tw>
 *     * First release.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/sched.h>

#include "ge_accel.h"

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC _IOW('F', 0x20, u_int32_t)
#endif

static int vram_total __initdata;

static struct fb_fix_screeninfo __initdata gefb_fix = {
	.id             = "WMT GE",
	.smem_start     = 0,
	.smem_len       = 0,
	.type           = FB_TYPE_PACKED_PIXELS,
	.type_aux       = 0,
	.visual         = FB_VISUAL_TRUECOLOR,
	.xpanstep       = 1,
	.ypanstep       = 1,
	.ywrapstep      = 1,
	.line_length    = 0,
	.mmio_start     = 0xD8050000,
	.mmio_len       = 0x0700,
	.accel          = FB_ACCEL_WM8510
};

static struct fb_var_screeninfo __initdata gefb_var = {
	.xres           = CONFIG_DEFAULT_RESX,
	.yres           = CONFIG_DEFAULT_RESY,
	.xres_virtual   = CONFIG_DEFAULT_RESX,
	.yres_virtual   = CONFIG_DEFAULT_RESY,
	.bits_per_pixel = 32,
	.red            = {16, 8, 0},
	.green          = {8, 8, 0},
	.blue           = {0, 8, 0},
	.transp         = {0, 0, 0},
	/*
	.bits_per_pixel = 16,
	.red            = {11, 5, 0},
	.green          = {5, 6, 0},
	.blue           = {0, 5, 0},
	.transp         = {0, 0, 0},
	*/
	.activate       = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE,
	.height         = -1,
	.width          = -1,
	.pixclock       = 39721,
	.left_margin    = 40,
	.right_margin   = 24,
	.upper_margin   = 32,
	.lower_margin   = 11,
	.hsync_len      = 96,
	.vsync_len      = 2,
	.vmode          = FB_VMODE_NONINTERLACED
};

static int gefb_open(struct fb_info *info, int user)
{
	return 0;
}

static int gefb_release(struct fb_info *info, int user)
{
	return ge_release(info);
}

static int gefb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		if (var->red.offset > 8) {
			/* LUT8 */
			var->red.offset = 0;
			var->red.length = 8;
			var->green.offset = 0;
			var->green.length = 8;
			var->blue.offset = 0;
			var->blue.length = 8;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 16:
		if (var->transp.length) {
			/* ARGB 1555 */
			var->red.offset = 10;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {
			/* RGB 565 */
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 24:
		/* RGB 888 */
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:
		/* ARGB 8888 */
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	}
	return 0;
}

static int gefb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	/* init your hardware here */
	if (var->bits_per_pixel == 8)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;

	if (ge_init(info))
		return -ENOMEM;

	return 0;
}

static int gefb_setcolreg(unsigned regno, unsigned red,
			      unsigned green, unsigned blue,
			      unsigned transp, struct fb_info *info)
{
	if (regno >= 256)  /* no. of hw registers */
		return -EINVAL;

	/* grayscale */

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	ge_setcolreg(regno, red, green, blue, transp, info);

	/*  The following is for fbcon. */

	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
		info->fix.visual == FB_VISUAL_DIRECTCOLOR) {

		if (regno >= 16)
			return -EINVAL;

		switch (info->var.bits_per_pixel) {
		case 16:
			((unsigned int *)(info->pseudo_palette))[regno] =
				(red & 0xf800) |
				((green & 0xfc00) >> 5) |
				((blue & 0xf800) >> 11);
				break;
		case 24:
		case 32:
			red   >>= 8;
			green >>= 8;
			blue  >>= 8;
			((unsigned int *)(info->pseudo_palette))[regno] =
				(red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue  << info->var.blue.offset);
			break;
		}
	}
	return 0;
}

static int gefb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	ge_pan_display(var, info);
	return 0;
}

static int gefb_ioctl(struct fb_info *info, unsigned int cmd,
			  unsigned long arg)
{
	int retval = 0;

	if (_IOC_TYPE(cmd) == GEIO_MAGIC)
		return ge_ioctl(info, cmd, arg);

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		vpp_wait_vsync();
		break;
	default:
		break;
	}

	return retval;
}

int gefb_hw_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return -EINVAL; // force soft_cursor()
}

int gefb_sync(struct fb_info *info)
{
	return ge_sync(info);
}

static struct fb_ops gefb_ops = {
	.owner          = THIS_MODULE,
	.fb_open        = gefb_open,
	.fb_release     = gefb_release,
	.fb_check_var   = gefb_check_var,
	.fb_set_par     = gefb_set_par,
	.fb_setcolreg   = gefb_setcolreg,
	.fb_pan_display = gefb_pan_display,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_blank       = ge_blank,
	.fb_cursor      = gefb_hw_cursor,
	.fb_ioctl       = gefb_ioctl,
	.fb_sync	= gefb_sync,
};

static int __init gefb_setup(char *options)
{
	char *this_opt;
	unsigned long val;

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		if (!strncmp(this_opt, "vtotal:", 7)) {
			if (!strict_strtoul(this_opt + 7, 10, &val))
				vram_total = val;
		}
	}

	return 0;
}

//--> Added by howayhuo on 20100305
//extern char __iomem *plcd_viraddr;	/* Virtual address */
//<-- end added

static int __init gefb_probe(struct platform_device *pdev)
{
//	struct platform_device *dev = to_platform_device(device);
	struct fb_info *info;
	int cmap_len, retval;
	unsigned int map_size;
	unsigned int chip_id;

	char mode_option[] = "1024x768@60";

	/*  Dynamically allocate memory for fb_info and par.*/
	info = framebuffer_alloc(sizeof(unsigned int) * 16, &pdev->dev);
	if (!info) {
		release_mem_region(info->fix.smem_start, info->fix.smem_len);
		return -ENOMEM;
	}

	/* Set default fb_info */
	info->fbops = &gefb_ops;
	info->fix = gefb_fix;

	chip_id = SCC_CHIP_ID >> 16;

	/* Auto detect VRAM */
	if (vram_total) {
		info->fix.smem_len = vram_total * 0x100000;
		info->fix.smem_start = ge_vram_addr(vram_total);
	} else if (CONFIG_GE_BUFFER_SIZE) {
		info->fix.smem_len = CONFIG_GE_BUFFER_SIZE * 0x100000;
		info->fix.smem_start = ge_vram_addr(CONFIG_GE_BUFFER_SIZE);
	} else {
		switch (chip_id) {
		case 0x3357:
			info->fix.smem_len = 0x400000;
			info->fix.smem_start = ge_vram_addr(4);
			break;
		case 0x3400:
		case 0x3429:
			info->fix.smem_len = 0x800000;
			info->fix.smem_start = ge_vram_addr(8);
			break;
		case 0x3426:
		case 0x3437:
		default:
			info->fix.smem_len = 0x300000;
			info->fix.smem_start = ge_vram_addr(3);
		}
	}

	/* Set video memory */
//--> Added by howayhuo on 20100305
	//if(plcd_viraddr)
	//	info->screen_base = plcd_viraddr;
	//else
	{
//<-- end added
	if (!request_mem_region(info->fix.smem_start,
		info->fix.smem_len, "gefb")) {
		printk(KERN_WARNING
			"%s: request memory region failed at 0x%lx\n",
			__func__, info->fix.smem_start);
	}

	info->screen_base = ioremap(info->fix.smem_start, info->fix.smem_len);
	if (!info->screen_base) {
		printk(KERN_ERR
			"%s: ioremap fail 0x%x bytes at 0x%lx\n",
			__func__, info->fix.smem_len, info->fix.smem_start);
		return -EIO;
	}

	printk(KERN_INFO "gefb: framebuffer at 0x%lx, mapped to 0x%p, "
		"using %dk, total %dk\n",
		info->fix.smem_start, info->screen_base,
		info->fix.smem_len / 1024, info->fix.smem_len / 1024);

//--> Added by howayhuo on 20100308
        //plcd_viraddr = info->screen_base;
//<-- End Added

}
	/*
	 *  Do as a normal fbdev does, but allocate a larger memory for GE.
	 */
	map_size = PAGE_ALIGN(info->fix.smem_len);

	/*
	 * The pseudopalette is an 16-member array for fbcon.
	 */
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_DEFAULT;	/* flag for fbcon */

	/*
	 * This should give a reasonable default video mode.
	 */
	retval = fb_find_mode(&info->var, info, mode_option,
			      NULL, 0, NULL, 8);

	if (!retval || retval == 4)
		return -EINVAL;

	/*
	 *  This has to been done !!!
	 */
	cmap_len = 256;	/* Be the same as VESA */
	retval = fb_alloc_cmap(&info->cmap, cmap_len, 0);
	if (retval < 0)
		printk(KERN_ERR "%s: fb_alloc_cmap fail.\n", __func__);

	/*
	 *  The following is done in the case of
	 *  having hardware with a static mode.
	 */
	info->var = gefb_var;

	/*
	 *  Load video output setting from VPP.
	 */
	vpp_get_info(&info->var);

	/*
	 *  For drivers that can...
	 */
	gefb_check_var(&info->var, info);

	/*
	 *  It's safe to allow fbcon to do it for you.
	 *  But in this case, we need it here.
	 */
	gefb_set_par(info);

	if (register_framebuffer(info) < 0) {
		ge_exit(info);
		return -EINVAL;
	}
	printk(KERN_INFO "fb%d: %s frame buffer device\n",
		info->node, info->fix.id);
	dev_set_drvdata(&pdev->dev, info);

	return 0;
}

static int gefb_remove(struct platform_device *pdev)
{
//	struct platform_device *dev = to_platform_device(device);
	struct fb_info *info = dev_get_drvdata(&pdev->dev);

	if (info) {
		ge_exit(info);
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

static int gefb_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int gefb_resume(struct platform_device *pdev)
{
	return 0;
}

/*
static struct device_driver gefb_driver = {
	.name           = "gefb",
	.bus            = &platform_bus_type,
	.probe          = gefb_probe,
	.remove         = gefb_remove,
	.suspend        = gefb_suspend,
	.resume         = gefb_resume,
};
*/

struct platform_driver gefb_driver = {
	.probe = gefb_probe,
	.remove = gefb_remove,
	.suspend        = gefb_suspend,
	.resume         = gefb_resume,
	.driver = { .name = "gefb" }
};

static u64 vt8430_fb_dma_mask = 0xffffffffUL;
static struct platform_device gefb_device = {
	.name   = "gefb",
	.dev    = {
		.dma_mask = &vt8430_fb_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

static int __init gefb_init(void)
{
	int ret;
	char *option = NULL;

	vram_total = 0;

	fb_get_options("gefb", &option);
	gefb_setup(option);

//	ret = driver_register(&gefb_driver);
        ret = platform_driver_register(&gefb_driver);
	if (!ret) {
		ret = platform_device_register(&gefb_device);
		if (ret)
		//	driver_unregister(&gefb_driver);
		    platform_driver_unregister(&gefb_driver);
	}
	return ret;
}
module_init(gefb_init);

static void __exit gefb_exit(void)
{
	//driver_unregister(&gefb_driver);
	 platform_driver_unregister(&gefb_driver);
	platform_device_unregister(&gefb_device);
	return;
}

module_exit(gefb_exit);

MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("WM GE framebuffer driver");
MODULE_LICENSE("GPL");

