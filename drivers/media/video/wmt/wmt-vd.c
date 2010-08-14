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
*Version 0.01 , Jason Lin, 2008/12/08
* First version
*
*------------------------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/mman.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/major.h>

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#endif

#include "wmt-vd.h"

#define DRIVER_NAME "wmt-vd"
//#define DEBUG
// #define WORDY

#define INFO(fmt,args...) printk(KERN_INFO "[" DRIVER_NAME "] " fmt , ## args)
#define WARN(fmt,args...) printk(KERN_WARNING "[" DRIVER_NAME "] " fmt , ## args)
#define ERROR(fmt,args...) printk(KERN_ERR "[" DRIVER_NAME "] " fmt , ## args)

#ifdef DEBUG
#define DBG(fmt, args...) printk(KERN_DEBUG "[" DRIVER_NAME "] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBG(fmt, args...)
#undef WORDY
#endif

#ifdef WORDY
#define WDBG(fmt, args...)	DBG(fmt, args...)
#else
#define WDBG(fmt, args...)
#endif

#define VD_MAJOR            236   /* For all Video Decoders */

static int videodecoder_minor = 0;
static int videodecoder_dev_nr = 1;
static struct cdev *videodecoder_cdev = NULL;

static struct videodecoder *decoders[VD_MAX] =
	{NULL,NULL,NULL,NULL,NULL,NULL};

static struct videodecoder_info vdinfo = {NULL,0};

static DECLARE_MUTEX(vd_sem);

static int videodecoder_open(
	struct inode *inode,
	struct file *filp
)
{
	unsigned int ret = -EINVAL;
	unsigned idx = VD_MAX;

	idx = iminor(inode);

	if (idx < VD_MAX && decoders[idx]){
		struct videodecoder *vd = decoders[idx];
		if(vd && vd->fops.open){
			down(&vd_sem);
			filp->private_data = (void *)&vdinfo;
			ret = vd->fops.open(inode,filp);
			up(&vd_sem);
		}
		//INFO("decoder %s opened.\n",vd->name);
	}

	return ret;
}

static int videodecoder_release(
	struct inode *inode,
	struct file *filp
)
{
	unsigned int ret = -EINVAL;
	unsigned idx = VD_MAX;

	idx = iminor(inode);

	if (idx < VD_MAX && decoders[idx]){
		struct videodecoder *vd = decoders[idx];
		if(vd && vd->fops.release){
			down(&vd_sem);
			ret = vd->fops.release(inode,filp);
			up(&vd_sem);
		}
		//INFO("decoder %s closed.\n",vd->name);
	}

	return ret;
}

static int videodecoder_ioctl(
	struct inode *inode,
	struct file *filp,
	unsigned int cmd,
	unsigned long arg
)
{
	unsigned int ret = -EINVAL;
	unsigned idx = VD_MAX;

	/* check type and number, if fail return ENOTTY */
	if( _IOC_TYPE(cmd) != VD_IOC_MAGIC )   return -ENOTTY;
	if( _IOC_NR(cmd) > VD_IOC_MAXNR )   return -ENOTTY;

	/* check argument area */
	if( _IOC_DIR(cmd) & _IOC_READ )
		ret = !access_ok( VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd));
	else if ( _IOC_DIR(cmd) & _IOC_WRITE )
		ret = !access_ok( VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd));

	if( ret ) return -EFAULT;

	idx = iminor(inode);
	if (idx < VD_MAX && decoders[idx]){
		struct videodecoder *vd = decoders[idx];
		if(vd && vd->fops.ioctl){
			down(&vd_sem);
			ret = vd->fops.ioctl(inode,filp,cmd,arg);
			up(&vd_sem);
		}
	}

	return ret;
}

static int videodecoder_mmap(
	struct file *filp,
	struct vm_area_struct *vma
)
{
	unsigned int ret = -EINVAL;
	unsigned idx = VD_MAX;

	if(filp && filp->f_dentry && filp->f_dentry->d_inode)
		idx = iminor(filp->f_dentry->d_inode);

	if (idx < VD_MAX && decoders[idx]){
		struct videodecoder *vd = decoders[idx];
		if(vd && vd->fops.mmap){
			down(&vd_sem);
			ret = vd->fops.mmap(filp,vma);
			up(&vd_sem);
		}
	}

	return ret;
}

static ssize_t videodecoder_read(
	struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *f_pos
)
{
	ssize_t ret = 0;
	unsigned idx = VD_MAX;

	if(filp && filp->f_dentry && filp->f_dentry->d_inode)
		idx = iminor(filp->f_dentry->d_inode);

	if (idx < VD_MAX && decoders[idx]){
		struct videodecoder *vd = decoders[idx];
		if(vd && vd->fops.read){
			down(&vd_sem);
			ret = vd->fops.read(filp,buf,count,f_pos);
			up(&vd_sem);
		}
	}

	return ret;
} /* videodecoder_read */

static ssize_t videodecoder_write(
	struct file *filp,
	const char __user *buf,
	size_t count,
	loff_t *f_pos
)
{
	ssize_t ret = 0;
	unsigned idx = VD_MAX;

	if(filp && filp->f_dentry && filp->f_dentry->d_inode)
		idx = iminor(filp->f_dentry->d_inode);

	if (idx < VD_MAX && decoders[idx]){
		struct videodecoder *vd = decoders[idx];
		if(vd && vd->fops.write){
			down(&vd_sem);
			ret = vd->fops.write(filp,buf,count,f_pos);
			up(&vd_sem);
		}
	}

	return ret;
} /* End of videodecoder_write() */

struct file_operations videodecoder_fops = {
	.owner			= THIS_MODULE,
	.open			= videodecoder_open,
	.release		= videodecoder_release,
	.read			= videodecoder_read,
	.write			= videodecoder_write,
	.ioctl			= videodecoder_ioctl,
	.mmap			= videodecoder_mmap,
};

static int videodecoder_probe(struct platform_device *pdev)
{
	int ret = 0;
	dev_t dev_no;

	dev_no = MKDEV(VD_MAJOR, videodecoder_minor);

	/* register char device */
	videodecoder_cdev = cdev_alloc();
	if(!videodecoder_cdev){
		ERROR("alloc dev error.\n");
		return -ENOMEM;
	}

	cdev_init(videodecoder_cdev,&videodecoder_fops);
	ret = cdev_add(videodecoder_cdev,dev_no,1);

	if(ret){
		ERROR("reg char dev error(%d).\n",ret);
		cdev_del(videodecoder_cdev);
		return ret;
	}

    vdinfo.prdt_virt = dma_alloc_coherent(NULL, MAX_INPUT_BUF_SIZE,
										&vdinfo.prdt_phys, GFP_KERNEL);

	if(!vdinfo.prdt_virt){
		ERROR("allocate video input buffer error.\n");
		cdev_del(videodecoder_cdev);
		vdinfo.prdt_virt = NULL;
		vdinfo.prdt_phys = 0;
		return -ENOMEM;
	}

	INFO("prob /dev/%s major %d, minor %d, prdt %p/%x, size %d KB\n",
		DRIVER_NAME, VD_MAJOR, videodecoder_minor,
		vdinfo.prdt_virt,vdinfo.prdt_phys,MAX_INPUT_BUF_SIZE/1024);

	return ret;
}

static int videodecoder_remove(struct platform_device *pdev)
{
	unsigned int idx = 0;

	while(idx < VD_MAX){
		if(decoders[idx] && decoders[idx]->remove){
			down(&vd_sem);
			decoders[idx]->remove();
			decoders[idx] = NULL;
			up(&vd_sem);
		}
		idx++;
	}

	if(vdinfo.prdt_virt)
	    dma_free_coherent(NULL, MAX_INPUT_BUF_SIZE, vdinfo.prdt_virt, vdinfo.prdt_phys);

	vdinfo.prdt_virt = NULL;
	vdinfo.prdt_phys = 0;

	return 0;
}

static int videodecoder_suspend(struct platform_device * pdev, pm_message_t state)
{
	int ret;
	unsigned int idx = 0;
        printk("videodecoder_suspend\n");
	while(idx < VD_MAX){
		if(decoders[idx] && decoders[idx]->suspend){
			down(&vd_sem);
			ret = decoders[idx]->suspend(state);
			up(&vd_sem);
			if(ret < 0){
				WARN("video decoder %32s suspend error. ret = %d\n",
					decoders[idx]->name,ret);
			}
		}
		idx++;
	}

	return 0;
}

static int videodecoder_resume(struct platform_device * pdev)
{
	int ret;
	unsigned int idx = 0;
        printk("videodecoder_resume\n");
	while(idx < VD_MAX){
		if(decoders[idx] && decoders[idx]->resume){
			down(&vd_sem);
			ret = decoders[idx]->resume();
			up(&vd_sem);
			if(ret < 0){
				WARN("video decoder %32s resume error. ret = %d\n",
					decoders[idx]->name,ret);
			}
		}
		idx++;
	}

	return 0;
}

static void videodecoder_platform_release(struct device *device)
{
	unsigned int idx = 0;

	while(idx < VD_MAX){
		if(decoders[idx] && decoders[idx]->remove){
			down(&vd_sem);
			decoders[idx]->remove();
			decoders[idx] = NULL;
			up(&vd_sem);
		}
		idx++;
	}

	if(vdinfo.prdt_virt)
	    dma_free_coherent(NULL, MAX_INPUT_BUF_SIZE, vdinfo.prdt_virt, vdinfo.prdt_phys);

	vdinfo.prdt_virt = NULL;
	vdinfo.prdt_phys = 0;

	return;
}

/*
static struct device_driver videodecoder_driver = {
	.name			= "wmt-vd", // This name should equal to platform device name.
	.bus			= &platform_bus_type,
	.probe			= videodecoder_probe,
	.remove			= videodecoder_remove,
	.suspend		= videodecoder_suspend,
	.resume			= videodecoder_resume
};
*/

static struct platform_driver videodecoder_driver = {
	.probe		= videodecoder_probe,
	.remove		= videodecoder_remove,
	.suspend	        = videodecoder_suspend,
	.resume		= videodecoder_resume,
	.driver		= {
		.name		= "wmt-vd",
	},
};

static struct platform_device videodecoder_device = {
	.name           = "wmt-vd",
	.id             = 0,
	.dev            = 	{
						.release =  videodecoder_platform_release,
						},
	.num_resources  = 0,		/* ARRAY_SIZE(spi_resources), */
	.resource       = NULL,		/* spi_resources, */
};

#ifdef CONFIG_PROC_FS
static int videodecoder_read_proc(
	char *buf,
	char **start,
	off_t offset,
	int len
)
{
	int size;
	unsigned int idx = 0;
    char *p = buf;

	p += sprintf(p, "***** video decoder information *****\n");

	while(idx < VD_MAX){
		if(decoders[idx] && decoders[idx]->get_info) {
		    down(&vd_sem);
		    size = decoders[idx]->get_info(p,start,offset,len);
		    up(&vd_sem);
		    len -= size;
		    if(len <= 40)
			    break;
		    p += size;
	    }
		idx++;
	}

	p += sprintf(p, "**************** end ****************\n");
    return (p - buf);
}
#endif

int	videodecoder_register(struct videodecoder *decoder)
{
	int ret = 0;
	dev_t dev_no;

	if(!decoder || decoder->id >= VD_MAX){
		WARN("register invalid video decoder\n");
		return -1;
	}

	if(decoders[decoder->id]){
		WARN("video decoder (ID=%d) exist.(E:%32s, N:%32s)\n",
			decoder->id,decoders[decoder->id]->name,decoder->name);
		return -1;
	}

	dev_no = MKDEV(VD_MAJOR, decoder->id);
	ret = register_chrdev_region(dev_no,videodecoder_dev_nr,decoder->name);
	if( ret < 0 ){
		ERROR("can't get %s device minor %d\n",decoder->name,decoder->id);
		return ret;
	}

	decoder->device = cdev_alloc();
	if(!decoder->device){
		unregister_chrdev_region(dev_no,videodecoder_dev_nr);
		ERROR("alloc dev error.\n");
		return -ENOMEM;
	}

	cdev_init(decoder->device,&videodecoder_fops);
	ret = cdev_add(decoder->device,dev_no,1);
	if(ret){
		ERROR("reg char dev error(%d).\n",ret);
		unregister_chrdev_region(dev_no,videodecoder_dev_nr);
		cdev_del(decoder->device);
		decoder->device = NULL;
		return ret;
	}

	if(decoder->setup){
		ret = decoder->setup();
	}
	else{
		DBG("%s setup function is not exist\n",decoder->name);
	}

	if(ret >= 0){
		down(&vd_sem);
		decoders[decoder->id] = decoder;
		up(&vd_sem);
		DBG("%s registered major %d minor %d\n",
			decoder->name, VD_MAJOR, decoder->id);
	}
	else{
		DBG("%s register major %d minor %d fail\n",
			decoder->name, VD_MAJOR, decoder->id);
		unregister_chrdev_region(dev_no,videodecoder_dev_nr);
		cdev_del(decoder->device);
		decoder->device = NULL;
		return ret;
	}

	return ret;
}

int	videodecoder_unregister(struct videodecoder *decoder)
{
	int ret = 0;
	dev_t dev_no;

	if(!decoder || decoder->id >= VD_MAX || !decoder->device){
		WARN("unregister invalid video decoder\n");
		return -1;
	}

	if(decoders[decoder->id] != decoder){
		WARN("unregiseter wrong video decoder. (E:%32s, R:%32s)\n",
			decoders[decoder->id]->name,decoder->name);
		return -1;
	}

	down(&vd_sem);
	decoders[decoder->id] = NULL;
	up(&vd_sem);

	if(decoder->remove)
		ret = decoder->remove();

	dev_no = MKDEV(VD_MAJOR, decoder->id);
	unregister_chrdev_region(dev_no,videodecoder_dev_nr);
	cdev_del(decoder->device);
	decoder->device = NULL;

	return ret;
}

static int __init videodecoder_init (void)
{
	int ret;
	dev_t dev_no;

	dev_no = MKDEV(VD_MAJOR, videodecoder_minor);
	ret = register_chrdev_region(dev_no,videodecoder_dev_nr,"wmt-vd");
	if( ret < 0 ){
		ERROR("can't get %s device major %d\n",DRIVER_NAME, VD_MAJOR);
		return ret;
	}

#ifdef CONFIG_PROC_FS
//    create_proc_info_entry("wmt-vd", 0, NULL, videodecoder_read_proc);
	DBG("create video decoder proc\n");
#endif

	platform_device_register(&videodecoder_device);
//	ret = driver_register(&videodecoder_driver);
        platform_driver_register(&videodecoder_driver);
	INFO("WonderMedia HW decoder driver inited\n");

	return ret;
}

void __exit videodecoder_exit (void)
{
	dev_t dev_no;

//	driver_unregister(&videodecoder_driver);
        platform_device_unregister(&videodecoder_driver);
	platform_device_unregister(&videodecoder_device);
	dev_no = MKDEV(VD_MAJOR, videodecoder_minor);
	unregister_chrdev_region(dev_no,videodecoder_dev_nr);

	INFO("WonderMedia HW decoder driver exit\n");

	return;
}

module_init(videodecoder_init);
module_exit(videodecoder_exit);
MODULE_DESCRIPTION("Video Codec device driver");
MODULE_LICENSE("GPL");

