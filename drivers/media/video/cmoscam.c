/*
 * WonderMedia CMOS Camera V4L2 driver
 *
 * (C) Copyright 2010 WonderMedia Technologies, Inc.
 *
 * Written by Vincent Chen <vincentchen@wondermedia.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the BSD Licence, GNU General Public License
 * as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */


#include "cmoscam.h"
#include <media/v4l2-ioctl.h>


#ifndef REG_SET32
#define REG_SET32(addr, val)    (*(volatile unsigned int *)(addr)) = val
#define REG_GET32(addr)         (*(volatile unsigned int *)(addr))
#define REG_VAL32(addr)         (*(volatile unsigned int *)(addr))
#define REG_SET16(addr, val)    (*(volatile unsigned short *)(addr)) = val
#define REG_GET16(addr)         (*(volatile unsigned short *)(addr))
#define REG_VAL16(addr)         (*(volatile unsigned short *)(addr))
#define REG_SET8(addr, val)     (*(volatile unsigned char *)(addr)) = val
#define REG_GET8(addr)          (*(volatile unsigned char *)(addr))
#define REG_VAL8(addr)          (*(volatile unsigned char *)(addr))
#endif


static struct video_device cmos_cam;
static int video_nr = -1; /* /dev/videoN, -1 for autodetect */

extern long video_ioctl2(struct file *file,
	       unsigned int cmd, unsigned long arg);


//static struct file_operations cmos_cam_fops = {

static const struct v4l2_file_operations cmos_cam_fops = {
	.owner          = THIS_MODULE,
	.open           = cmos_cam_open, //video_exclusive_open,
	.release        = cmos_cam_release, //video_exclusive_release,
	.ioctl          = video_ioctl2, /* V4L2 ioctl handler */
	/*
	.read           = cmos_cam_read,
	*/
	.mmap           = cmos_cam_mmap,
	//.llseek         = no_llseek,   
};

struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap	  = cmos_cam_querycap,
	//.vidioc_enum_fmt_cap  = cmos_cam_enum_fmt_cap,	
	.vidioc_enum_fmt_vid_cap = cmos_cam_enum_fmt_cap,
	//.vidioc_g_fmt_cap	  = cmos_cam_g_fmt_cap,		
	.vidioc_g_fmt_vid_cap =	 cmos_cam_g_fmt_cap, 
	//.vidioc_try_fmt_cap   = cmos_cam_try_fmt_cap,	
	.vidioc_try_fmt_vid_cap = cmos_cam_try_fmt_cap,
	//.vidioc_s_fmt_cap	  = cmos_cam_s_fmt_cap,		
	.vidioc_s_fmt_vid_cap = cmos_cam_s_fmt_cap, 
	.vidioc_reqbufs 	  = cmos_cam_reqbufs,
	.vidioc_querybuf	  = cmos_cam_querybuf,
	.vidioc_qbuf		  = cmos_cam_qbuf,
	.vidioc_dqbuf		  = cmos_cam_dqbuf,
	.vidioc_s_std		  = cmos_cam_s_std,
	.vidioc_enum_input	  = cmos_cam_enum_input,
	.vidioc_g_input 	  = cmos_cam_g_input,
	.vidioc_s_input 	  = cmos_cam_s_input,
	.vidioc_queryctrl	  = cmos_cam_queryctrl,
	.vidioc_g_ctrl		  = cmos_cam_g_ctrl,
	.vidioc_s_ctrl		  = cmos_cam_s_ctrl,
	.vidioc_streamon	  = cmos_cam_streamon,
	.vidioc_streamoff	  = cmos_cam_streamoff,
};


static struct video_device cmos_cam = {
	.name           = "cmos_cam",
	//.type           = VID_TYPE_CAPTURE,
	//.hardware       = 0,
	.fops           = &cmos_cam_fops,
	.minor          = -1,
	.release        = video_device_release,
	.debug          = 0,

	.ioctl_ops 		= &ioctl_ops,     
};

extern int ov7670_exist();

]
	
static int __init cmos_cam_init(void)
{
	int ret;

	// Noodles: If we do not check if the OV7670 exist, 
	// the /dev/video0 will be always registered.
	if (ov7670_exist()){
		ret = video_register_device(&cmos_cam, VFL_TYPE_GRABBER, video_nr);
		if (ret < 0)
			printk(KERN_ERR "WonderMedia CMOS camera register failed\n");
		else{
		}
	}
	return ret;
}

static void __exit cmos_cam_exit(void)
{
	video_unregister_device(&cmos_cam);
}

module_init(cmos_cam_init);
module_exit(cmos_cam_exit);

MODULE_DESCRIPTION("WonderMedia CMOS Camera for ARM SoC");
MODULE_AUTHOR("WonderMedia Technologies, Inc.");
MODULE_LICENSE("Dual BSD/GPL");

