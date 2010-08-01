#define WMT_PWM_C

/*--- wmt-pwm.c ---------------------------------------------------------------
*   Copyright (C) 2009 WMT Tech. Inc.
*
* MODULE       : wmt-pwm.c
* AUTHOR       : Sam Shen
* DATE         : 2009/8/12
* DESCRIPTION  :
*-----------------------------------------------------------------------------*/

/*--- History -------------------------------------------------------------------
*Version 0.01 , Sam Shen, 2009/8/12
*	First version
*
*------------------------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/major.h>
#include <linux/delay.h>
#include <linux/sysctl.h>

#include "wmt-pwm.h"
#include "../video/wmt/vpp-project.h"

// #define DEBUG

#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBG(fmt, args...)   DPRINT("[BTM] %s: " fmt, __FUNCTION__ , ## args)
#define DBG_DETAIL(fmt, args...)   DPRINT("[BTM] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBG(fmt, args...)
#define DBG_DETAIL(fmt, args...)
#endif

#define DEVICE_NAME "WMT-PWM"
#define PWM_NUM	2		/* WM3429 has 2 pwm outputs */

struct pwm_dev_s {
	/* module parameters */

	/* char dev struct */
	struct cdev cdev;
};
#define PWM_MAJOR			239

static int pwm_dev_major = PWM_MAJOR;
static int pwm_dev_minor = 0;
static int pwm_dev_nr = 1;
static int pwm_dev_ref = 0; /* is device open */
static struct pwm_dev_s pwm_dev;

unsigned int pwm_level[2] = { 0, 0 };
unsigned int pwm_freq[2] = {1000, 1000 };
unsigned int pwm_enable[2] = {0,0};
int PWM_PERIOD_VAL = 1000;

DECLARE_MUTEX(pwm_lock);

#define pwm_write_reg(addr,val,wait)	\
	REG32_VAL(addr) = val;	\
	while(REG32_VAL(0xd8220040)&=wait);

static void pwm_set_control(int no,unsigned int ctrl)
{
	unsigned int addr;

	addr = PWM_CTRL_REG_ADDR + (0x10 * no);
	//pwm_write_reg(addr,ctrl,PWM_CTRL_UPDATE << (4*no));
	pwm_write_reg(addr,ctrl,PWM_CTRL_UPDATE << (8*no));
} /* End of pwm_proc */

static void pwm_set_scalar(int no,unsigned int scalar)
{
	unsigned int addr;

	addr = PWM_SCALAR_REG_ADDR + (0x10 * no);
	//pwm_write_reg(addr,scalar,PWM_SCALAR_UPDATE << (4*no));
	pwm_write_reg(addr,scalar,PWM_SCALAR_UPDATE << (8*no));
}

static void pwm_set_period(int no,unsigned int period)
{
	unsigned int addr;

	addr = PWM_PERIOD_REG_ADDR + (0x10 * no);
	//pwm_write_reg(addr,period,PWM_PERIOD_UPDATE << (4*no));
	pwm_write_reg(addr,period,PWM_PERIOD_UPDATE << (8*no));
}

void pwm_set_duty(int no,unsigned int duty)  //modified to glabal function from static function by howayhuo
{
	unsigned int addr;

	addr = PWM_DUTY_REG_ADDR + (0x10 * no);
	//pwm_write_reg(addr,duty,PWM_DUTY_UPDATE << (4*no));
	pwm_write_reg(addr,duty,PWM_DUTY_UPDATE << (8*no));
}

unsigned int pwm_get_period(int no)
{
	unsigned int addr;

	addr = PWM_PERIOD_REG_ADDR + (0x10 * no);
	return (REG32_VAL(addr) & 0xFFF);
}

unsigned int pwm_get_duty(int no)
{
	unsigned int addr;

	addr = PWM_DUTY_REG_ADDR + (0x10 * no);
	return (REG32_VAL(addr) & 0xFFF);
}
/*
unsigned int pwm_get_scalar(int no)
{
	unsigned int addr;

	addr = PWM_SCALAR_REG_ADDR + (0x10 * no);
        return (REG32_VAL(addr) & 0x3FF);
}
*/
static void pwm_set_gpio(int no,int enable)
{
	unsigned int pwm_pin;
	unsigned int reg;

 //       printk("chip id: 0x%08X\n", REG32_VAL(0xd8120000) );
	/* WM3437 */
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34370000 ){
		pwm_pin = (no==0)? 0x8000000:0x4000000;
		if( no == 0 ){
			if( enable ){
				REG32_VAL(0xd81100dc) |= pwm_pin;
				REG32_VAL(0xd811005c) &= ~0x08000000;	// PWM0 gpio enable
			}
			else {
				REG32_VAL(0xd811005c) |= pwm_pin;	// PWM0 gpio enable
				REG32_VAL(0xd811009c) |= pwm_pin;	// PWM0 OC
				REG32_VAL(0xd81100dc) &= ~pwm_pin;	// PWM0 OD
			}
		}

		if( no == 1 ){
			reg = REG32_VAL(0xd8110200);
			if( enable ){
				reg |= 0x10;
			}
			else {
				reg &= ~0x10;
			}
			REG32_VAL(0xd8110200) = reg;
		}
	}

	/* WM3426 */
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34260000 ){
		pwm_pin = (no==0)? PWM_GPIO_BIT_0:PWM_GPIO_BIT_1;
		if( enable ){
			REG32_VAL(PWM_GPIO_OD_REG) |= pwm_pin;
			REG32_VAL(PWM_GPIO_CTRL_REG) &= ~pwm_pin;
		}
		else {
			REG32_VAL(PWM_GPIO_CTRL_REG) |= pwm_pin;
			REG32_VAL(PWM_GPIO_OC_REG) |= pwm_pin;
			REG32_VAL(PWM_GPIO_OD_REG) &= ~pwm_pin;
		}
	}

	/* WM3429 */
	if( (REG32_VAL(0xd8120000) & 0xFFFF0000) == 0x34290000 ){
		pwm_pin = (no==0)? 0x1:0x2;
		if( enable ){
			REG32_VAL(0xd8110074) |= pwm_pin;
			REG32_VAL(0xd8110014) &= ~pwm_pin;
		}
		else {
			REG32_VAL(0xd8110014) |= pwm_pin;
			REG32_VAL(0xd8110044) |= pwm_pin;
			REG32_VAL(0xd8110074) &= ~pwm_pin;
		}

		if( no == 1 ){
			reg = REG32_VAL(0xd8110200);
			if( enable ){
				reg |= 0x100;
			}
			else {
				reg &= ~0x100;
			}
			REG32_VAL(0xd8110200) = reg;
			REG32_VAL(0xd8110618) &= ~0x200;
		}
	}
}

void pwm_set_enable(int no,int enable)
{
	unsigned int addr,reg;

	pwm_enable[no] = enable;
	addr = PWM_CTRL_REG_ADDR + (0x10 * no);
	reg = REG32_VAL(addr);
	if( enable ){
		reg |= PWM_ENABLE;
	}
	else {
		reg &= ~PWM_ENABLE;
	}
	//pwm_write_reg(addr,reg,PWM_CTRL_UPDATE << (4*no));
	pwm_write_reg(addr,reg,PWM_CTRL_UPDATE << (8*no));
	pwm_set_gpio(no,enable);
}

unsigned int pwm_get_enable(int no)
{
	unsigned int addr,reg;

	addr = PWM_CTRL_REG_ADDR + (0x10 * no);
	reg = REG32_VAL(addr);
	reg &= PWM_ENABLE;
	return reg;
}

//--> added by howayhuo on 20091111
unsigned int pwm_pclk(void)
{
    unsigned int clock;
    unsigned int reg;
    unsigned int value;

    reg = REG32_VAL(0xd8130204);
    clock = 25000000;	/* system freq 25M */

    if( (reg & BIT8) == 0 )	// Pre-Divisor Bypass
        clock /= 2;

    value = reg & 0x1F;
    if( value >= 4 )
        value = value * 2;
    else
        value = 1;
    clock *= value;
    //     printf("1: reg =%d, value = %d, clock = %d\n", reg, value, clock);
    reg = REG8_VAL(0xd8130348);
    reg = reg & 0x1F;
    value = (reg==0)? 32:reg;
    clock /= value;
    //     printf("2: reg =%d, value = %d, clock = %d\n", reg, value, clock);
    return clock;
}
//<-- end add

void pwm_set_freq(int no,unsigned int freq)
{
	unsigned int clock;
	unsigned int reg;
	unsigned int value;

	if( (REG32_VAL(0xd8130250) & BIT10) == 0 )	// check pwm power on
		return;

	pwm_freq[no] = freq;
	reg = REG32_VAL(0xd8130204);
//#if(WMT_CUR_PID == WMT_PID_8425)
//	clock = 27000000;	/* system freq 27M */
//#else
	clock = 25000000;	/* system freq 25M */
//#endif
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

	reg = REG8_VAL(0xd8130348);
	reg = reg & 0x1F;
	value = (reg==0)? 32:reg;
	clock /= value;

	reg = (clock / freq / PWM_PERIOD_VAL) -1;
	pwm_set_scalar(no,reg);
}

void pwm_set_level(int no,unsigned int level)
{
	unsigned int duty,period;

	if( (REG32_VAL(0xd8130250) & BIT10) == 0 )	// check pwm power on
		return;

	pwm_level[no] = level;
	period = PWM_PERIOD_VAL - 1;
	duty = (level * PWM_PERIOD_VAL / 100);
	duty = (duty)? (duty-1):0;

	pwm_set_period(no,period);
	pwm_set_duty(no,duty);
	pwm_set_control(no,(level)? 0x35:0x8);
	pwm_set_gpio(no,level);
}

void pwm_get_config(int no,unsigned int *freq,unsigned int *level)
{
	*freq = pwm_freq[no];
	*level = pwm_level[no];
}

#ifdef CONFIG_PROC_FS
static int pwm_do_proc(ctl_table * ctl,int write,struct file *file,void *buffer,size_t * len,loff_t *ppos)
{
	int ret;
	int no;

	ret = proc_dointvec(ctl, write, file, buffer, len, ppos);
	if( write ){
		switch( ctl->ctl_name ){
			case 1:
			case 2:
				no = ctl->ctl_name - 1;
				pwm_set_freq(no,pwm_freq[no]);
				break;
			case 3:
			case 4:
				no = ctl->ctl_name - 3;
				pwm_set_level(no,pwm_level[no]);
				break;
			default:
				break;
		}
	}
	return ret;
}

static ctl_table pwm_table[] = {
    {
		.ctl_name	= 1,
		.procname	= "freq0",
		.data		= &pwm_freq[0],
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler = &pwm_do_proc,
	},
    {
		.ctl_name	= 2,
		.procname	= "freq1",
		.data		= &pwm_freq[1],
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler = &pwm_do_proc,
	},
    {
		.ctl_name	= 3,
		.procname	= "level0",
		.data		= &pwm_level[0],
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler = &pwm_do_proc,
	},
    {
		.ctl_name	= 4,
		.procname	= "level1",
		.data		= &pwm_level[1],
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler = &pwm_do_proc,
	},
	{ .ctl_name = 0 }
};

static ctl_table pwm_root_table[] = {
	{
		.ctl_name	= CTL_DEV,
		.procname	= "pwm",	// create path ==> /proc/sys/vpp
		.mode		= 0555,
		.child 		= pwm_table
	},
	{ .ctl_name = 0 }
};
static struct ctl_table_header *pwm_table_header;
#endif

/*!*************************************************************************
* pwm_open()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_open
(
	struct inode *inode,	/*!<; //[IN] a pointer point to struct inode */
	struct file *filp		/*!<; //[IN] a pointer point to struct file  */
)
{
	struct pwm_dev_s *dev;
	int minor_no;

	DBG("Enter pwm_open\n");

	if(pwm_dev_ref)
		return -EBUSY;

	pwm_dev_ref++;

	try_module_get(THIS_MODULE);

	dev = container_of(inode->i_cdev,struct pwm_dev_s,cdev);
	filp->private_data = dev;
	minor_no = iminor(inode);	/* get */

	/* TODO */
	/* Step 1: check  hardware resource */

	/* Step 2: if first then initial hardware */

	/* Step 3: update f_op pointer */

	/* Step 4: allocate and assign provate_data structure */
	/* filp->private_data */

	return 0;
} /* End of pwm_open() */

/*!*************************************************************************
* pwm_release()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_release
(
	struct inode *inode,	/*!<; //[IN] a pointer point to struct inode */
	struct file *filp		/*!<; //[IN] a pointer point to struct file  */
)
{
	struct pwm_dev_s *dev;

	DBG("Enter pwm_release\n");

	dev = container_of(inode->i_cdev,struct pwm_dev_s,cdev);

	pwm_dev_ref--;
	module_put(THIS_MODULE);

	/* TODO */
	/* Step 1: free private data resource */

	/* Step 2: if last then shutdown hardware */
	return 0;
} /* End of pwm_release() */

/*!*************************************************************************
* pwm_ioctl()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_ioctl
(
	struct inode *inode,	/*!<; //[IN] a pointer point to struct inode */
	struct file *filp,		/*!<; //[IN] a pointer point to struct file  */
	unsigned int cmd,		/*!<; // please add parameters description her*/
	unsigned long arg		/*!<; // please add parameters description her*/
)
{
	int err = 0;
	int retval = 0;
	pwm_ctrl_t parm;

	/* check type and number, if fail return ENOTTY */
	if( _IOC_TYPE(cmd) != PWM_IOC_MAGIC )	return -ENOTTY;
	if( _IOC_NR(cmd) >= PWM_IOC_MAXNR )	return -ENOTTY;

	/* check argument area */
	if( _IOC_DIR(cmd) & _IOC_READ )
		err = !access_ok( VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd));
	else if ( _IOC_DIR(cmd) & _IOC_WRITE )
		err = !access_ok( VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd));

	if( err ) return -EFAULT;

	down(&pwm_lock);

	switch(cmd){
		case PWMIOSET_ENABLE:
			copy_from_user( (void *) &parm, (const void *)arg, sizeof(pwm_ctrl_t));
			pwm_set_enable(parm.no,parm.value);
			break;
		case PWMIOSET_FREQ:
			copy_from_user( (void *) &parm, (const void *)arg, sizeof(pwm_ctrl_t));
			pwm_freq[parm.no] = parm.value;
			pwm_set_freq(parm.no,parm.value);
			break;
		case PWMIOGET_FREQ:
			copy_from_user( (void *) &parm, (const void *)arg, sizeof(pwm_ctrl_t));
			parm.value = pwm_freq[parm.no];
			copy_to_user((void *) arg,(void *) &parm, sizeof(pwm_ctrl_t));
			break;
		case PWMIOSET_LEVEL:
			copy_from_user( (void *) &parm, (const void *)arg, sizeof(pwm_ctrl_t));
			pwm_level[parm.no] = parm.value;
			pwm_set_level(parm.no,parm.value);
			break;
		case PWMIOGET_LEVEL:
			copy_from_user( (void *) &parm, (const void *)arg, sizeof(pwm_ctrl_t));
			parm.value = pwm_level[parm.no];
			copy_to_user((void *) arg,(void *) &parm, sizeof(pwm_ctrl_t));
			break;
		default:
			retval = -ENOTTY;
			break;
	}

	up(&pwm_lock);
	return retval;
} /* End of pwm_ioctl() */

/*!*************************************************************************
	driver file operations struct define
****************************************************************************/
struct file_operations pwm_fops = {
	.owner = THIS_MODULE,
	.open = pwm_open,
	.ioctl = pwm_ioctl,
	.release = pwm_release,
};

/*!*************************************************************************
* pwm_probe()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_probe
(
	struct platform_device *dev	/*!<; // please add parameters description her*/
)
{
	int ret = 0;
	dev_t dev_no;
	struct cdev *cdev;
//	int i;

	dev_no = MKDEV(pwm_dev_major,pwm_dev_minor);

	/* register char device */
	// cdev = cdev_alloc();
	cdev = &pwm_dev.cdev;
	cdev_init(cdev,&pwm_fops);
	ret = cdev_add(cdev,dev_no,1);

	if( ret ){
		printk(KERN_ALERT "*E* register char dev \n");
		return ret;
	}

//	printk( KERN_ALERT "/dev/%s major number %d, minor number %d\n", DEVICE_NAME, pwm_dev_major,pwm_dev_minor);

#ifdef CONFIG_PROC_FS
	pwm_table_header = register_sysctl_table(pwm_root_table);
#endif
	return ret;
} /* End of pwm_probe() */

/*!*************************************************************************
* pwm_remove()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_remove
(
	struct platform_device *dev	/*!<; // please add parameters description her*/
)
{
	struct cdev *cdev;

	cdev = &pwm_dev.cdev;
	cdev_del(cdev);
	printk( KERN_ALERT "Enter pwm_remove \n");
	return 0;
} /* End of pwm_remove() */

/*!*************************************************************************
* pwm_suspend()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_suspend
(
	struct platform_device *dev,     /*!<; // a pointer point to struct device */
	pm_message_t state		/*!<; // suspend state */
)
{
        printk("pwm_suspend\n");
//	int i;
/*
	for (i=0;i<PWM_NUM;i++) {
		pwm_enable[i] = pwm_get_enable(i);
	}
*/
	return 0;
} /* End of btm_suspend() */

/*!*************************************************************************
* pwm_resume()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_resume
(
	struct platform_device *dev 	/*!<; // a pointer point to struct device */
)
{
//	int i;
        printk("pwm_resume\n");
/*
	for (i=0; i<PWM_NUM; i++) {
		pwm_set_freq(i, pwm_freq[i]);
		pwm_set_level(i, pwm_level[i]);
		pwm_set_enable(i, pwm_enable[i]);
	}
*/
	return 0;
} /* End of pwm_resume() */

/*!*************************************************************************
	device driver struct define
****************************************************************************/
/*
static struct device_driver pwm_driver = {
	.name           = "wmt-pwm", // This name should equal to platform device name.
	.bus            = &platform_bus_type,
	.probe          = pwm_probe,
	.remove         = pwm_remove,
	.suspend        = pwm_suspend,
	.resume         = pwm_resume
};
*/
struct platform_driver pwm_driver = {
	.probe = pwm_probe,
	.remove = pwm_remove,
	.suspend        = pwm_suspend,
	.resume         = pwm_resume,
	.driver = { .name = "wmt-pwm" }
};
/*!*************************************************************************
* pwm_platform_release()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static void pwm_platform_release
(
	struct device *device	/*!<; // please add parameters description her*/
)
{
} /* End of pwm_platform_release() */

/*!*************************************************************************
	platform device struct define
****************************************************************************/
#if 0
static struct resource pwm_resources[] = {
	[0] = {
		.start  = 0xf8011400,
		.end    = 0xf80117ff,
		.flags  = IORESOURCE_MEM,
	},
};

static u64 pwm_dma_mask = 0xffffffffUL;
#endif

static struct platform_device pwm_device = {
	.name           = "wmt-pwm",
	.id             = 0,
	.dev            = 	{	.release = pwm_platform_release,
#if 0
							.dma_mask = &pwm_dma_mask,
							.coherent_dma_mask = ~0,
#endif
						},
	.num_resources  = 0,		/* ARRAY_SIZE(btm_resources), */
	.resource       = NULL,		/* pwm_resources, */
};


/*!*************************************************************************
* pwm_init()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static int pwm_init(void)
{
	int ret;
	dev_t dev_no;

	printk("------------ pwm_init\n");

	if( pwm_dev_major ){
		dev_no = MKDEV(pwm_dev_major,pwm_dev_minor);
		ret = register_chrdev_region(dev_no,pwm_dev_nr,"wmt-pwm");
	}
	else {
		ret = alloc_chrdev_region(&dev_no,pwm_dev_minor,pwm_dev_nr,"wmt-pwm");
		pwm_dev_major = MAJOR(dev_no);
	}

	if( ret < 0 ){
		printk(KERN_ALERT "*E* can't get major %d\n",pwm_dev_major);
		return ret;
	}
//        wmt_set_backlight();
	platform_device_register(&pwm_device);
//	ret = driver_register(&pwm_driver);
        ret = platform_driver_register(&pwm_driver);

	return ret;
} /* End of pwm_init() */

module_init(pwm_init);

/*!*************************************************************************
* pwm_exit()
*
* Private Function by Sam Shen, 2009/8/12
*/
/*!
* \brief
*
* \retval  0 if success
*/
static void pwm_exit(void)
{
	dev_t dev_no;

	printk(KERN_ALERT "Enter pwm_exit\n");

//	driver_unregister(&pwm_driver);
        platform_driver_unregister(&pwm_driver);
	platform_device_unregister(&pwm_device);
	dev_no = MKDEV(pwm_dev_major,pwm_dev_minor);
	unregister_chrdev_region(dev_no,pwm_dev_nr);
	return;
} /* End of pwm_exit() */

module_exit(pwm_exit);

MODULE_AUTHOR("WonderMedia SW Team Sam Shen");
MODULE_DESCRIPTION("wmt-pwm device driver");
MODULE_LICENSE("GPL");

/*--------------------End of Function Body -----------------------------------*/

#undef WMT_PWM_C

