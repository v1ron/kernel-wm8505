/**************************************************************
Copyright (c) 2008 WonderMedia Technologies, Inc.

Module Name:
    $Workfile: wmt-lcd-backlight.c $
Abstract:
    This program is the WMT LCD backlight driver for Android 1.6 system.
    Andriod1.6 API adjusts the LCD backlight by writing follwing file:
        /sys/class/leds/lcd-backlight/brightness
    Use WMT PWM to control the LCD backlight
Revision History:
    Jan.08.2010 First Created by HowayHuo

**************************************************************/
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/io.h>

#include "../char/wmt-pwm.h"

static int pwm_no;  // 0:PWM0, 1:PWM1
static unsigned int pwm_period;
extern unsigned int lcd_blt_level;
extern int PWM_PERIOD_VAL;

extern struct proc_dir_entry proc_root;

static int __init pwm_no_arg
(
	char *str			/*!<; // argument string */
)
{
	sscanf(str,"%d",&pwm_no);
	printk("set pwm no = %d\n",pwm_no);

  	return 1;
}

__setup("pwmno=", pwm_no_arg); //从UBOOT参数bootargs中读取pwmno


/*
 * For simplicity, we use "brightness" as if it were a linear function
 * of PWM duty cycle.  However, a logarithmic function of duty cycle is
 * probably a better match for perceived brightness: two is half as bright
 * as four, four is half as bright as eight, etc
 */
static void lcd_brightness(struct led_classdev *cdev, enum led_brightness b)
{
    unsigned int duty, pwm_pin;

//     printk("--------lcd_brightness b = %d, pwm_period = %d\n", b, pwm_period);

    duty = (b *  pwm_period) / 255;
    if(duty)
	if(--duty > 0xFFF)
	    duty = 0xFFF;
    pwm_set_duty(pwm_no, duty);
    //printk("b= %d, duty = %d\n", b, duty);

    if(duty)
    {
        pwm_pin = (pwm_no==0)? PWM_GPIO_BIT_0:PWM_GPIO_BIT_1;
        if(REG32_VAL(PWM_GPIO_CTRL_REG)  & pwm_pin)
        {
            //printk("lcd turn on -->\n");
            pwm_set_enable(pwm_no,1);
        }
    }
    else
    {
        //printk("lcd turn off  <--\n");
	pwm_set_enable(pwm_no,0);
    }
    return;
}

/*
 * NOTE:  we reuse the platform_data structure of GPIO leds,
 * but repurpose its "gpio" number as a PWM channel number.
 */
static int __init lcd_backlight_probe(struct platform_device *pdev)
{
	const struct gpio_led_platform_data	*pdata;
	struct led_classdev				*leds;
	int					i;
	int					status;

	pdata = pdev->dev.platform_data;
	if (!pdata || pdata->num_leds < 1)
		return -ENODEV;

	leds = kcalloc(pdata->num_leds, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	for (i = 0; i < pdata->num_leds; i++) {
		struct led_classdev		*led = leds + i;
		const struct gpio_led	*dat = pdata->leds + i;

		led->name = dat->name;
		led->brightness = 128;
		led->brightness_set = lcd_brightness;
		led->default_trigger = dat->default_trigger;

		/* Hand it over to the LED framework */
		status = led_classdev_register(&pdev->dev, led);
		if (status < 0) {
			goto err;
		}
	}

	platform_set_drvdata(pdev, leds);

	REG32_VAL(0xd8130250) |= BIT10; //Enable PWM Clock

	if((pwm_no < 0) || (pwm_no > 1)) //WM8505 only has 2 PWM
	    pwm_no = 0; //default set to 0 when pwm_no is invalid

	pwm_period  = pwm_get_period(pwm_no) + 1;
	//printk("--> pwm no = %d, pwm_period = %d\n", pwm_no, pwm_period);

	return 0;

err:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			led_classdev_unregister(leds + i);
		}
	}
	kfree(leds);

	return status;
}

static int __exit lcd_backlight_remove(struct platform_device *pdev)
{
	const struct gpio_led_platform_data	*pdata;
	struct led_classdev				*leds;
	unsigned				i;

	pdata = pdev->dev.platform_data;
	leds = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		struct led_classdev		*led = leds + i;

		led_classdev_unregister(led);
	}

	kfree(leds);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#define _U	0x01	/* upper */
#define _L	0x02	/* lower */
#define _D	0x04	/* digit */
#define _C	0x08	/* cntrl */
#define _P	0x10	/* punct */
#define _S	0x20	/* white space (space/lf/tab) */
#define _X	0x40	/* hex digit */
#define _SP	0x80	/* hard space (0x20) */

static unsigned char _ctype[] = {
_C,_C,_C,_C,_C,_C,_C,_C,			/* 0-7 */
_C,_C|_S,_C|_S,_C|_S,_C|_S,_C|_S,_C,_C,		/* 8-15 */
_C,_C,_C,_C,_C,_C,_C,_C,			/* 16-23 */
_C,_C,_C,_C,_C,_C,_C,_C,			/* 24-31 */
_S|_SP,_P,_P,_P,_P,_P,_P,_P,			/* 32-39 */
_P,_P,_P,_P,_P,_P,_P,_P,			/* 40-47 */
_D,_D,_D,_D,_D,_D,_D,_D,			/* 48-55 */
_D,_D,_P,_P,_P,_P,_P,_P,			/* 56-63 */
_P,_U|_X,_U|_X,_U|_X,_U|_X,_U|_X,_U|_X,_U,	/* 64-71 */
_U,_U,_U,_U,_U,_U,_U,_U,			/* 72-79 */
_U,_U,_U,_U,_U,_U,_U,_U,			/* 80-87 */
_U,_U,_U,_P,_P,_P,_P,_P,			/* 88-95 */
_P,_L|_X,_L|_X,_L|_X,_L|_X,_L|_X,_L|_X,_L,	/* 96-103 */
_L,_L,_L,_L,_L,_L,_L,_L,			/* 104-111 */
_L,_L,_L,_L,_L,_L,_L,_L,			/* 112-119 */
_L,_L,_L,_P,_P,_P,_P,_C,			/* 120-127 */
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,		/* 128-143 */
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,		/* 144-159 */
_S|_SP,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,   /* 160-175 */
_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,_P,       /* 176-191 */
_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,_U,       /* 192-207 */
_U,_U,_U,_U,_U,_U,_U,_P,_U,_U,_U,_U,_U,_U,_U,_L,       /* 208-223 */
_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,_L,       /* 224-239 */
_L,_L,_L,_L,_L,_L,_L,_P,_L,_L,_L,_L,_L,_L,_L,_L};      /* 240-255 */

#define __ismask(x) (_ctype[(int)(unsigned char)(x)])
#define isspace(c)	((__ismask(c)&(_S)) != 0)
#define isdigit(c)	((__ismask(c)&(_D)) != 0)

static int atoi(char *s) //added by howayhuo
{
    int i,n,sign;

    for(i=0;isspace(s[i]);i++) //跳过空白符
          ;
    sign=(s[i]=='-')?-1:1;
    if(s[i]=='+'||s[i]=='-')   //跳过符号
          i++;
    for(n=0;isdigit(s[i]);i++)
          n=10*n+(s[i]-'0');  //将数字字符转换成整形数字
    return sign *n;
}

static int lcdbltlevel_proc_write( struct file   *file,
                           const char    *buffer,
                           unsigned long count,
                           void          *data )
{
    char value[32];
    int len = count, brightness;

    if( len >= sizeof(value))
        len = sizeof(value) - 1;

    if(copy_from_user(value, buffer, len))
        return -EFAULT;

    value[len] = '\0';

    brightness = atoi(value);

    if(brightness > 255)
        brightness = 255;

    printk("lcdbltlevel_proc_write: value = %s, brightness = %d\n", value, brightness);

    if(brightness >= 0)
        lcd_blt_level = (brightness * 100) / 255;
    else
    {
    printk("duty = %d\n", pwm_get_duty(pwm_no) + 1);
    lcd_blt_level = ((pwm_get_duty(pwm_no) + 1) *  100)/PWM_PERIOD_VAL;
    }

    printk("lcdbltlevel_proc_write: lcd_blt_level = %d\n", lcd_blt_level);
    return count;
}

static int lcdbltlevel_proc_read(char *page, char **start, off_t off, int count,  int *eof, void *data)
{
	int len = 0;
	len = sprintf(page, "%u\n", lcd_blt_level);
	return len;
}

static struct gpio_led lcd_pwm[] = {
	{
		.name			= "lcd-backlight",
	},
};


static struct gpio_led_platform_data lcd_backlight_data = {
	.leds		= lcd_pwm,
	.num_leds	= ARRAY_SIZE(lcd_pwm),
};

static struct platform_device lcd_backlight_device = {
	.name           = "lcd-backlight",
	.id             = 0,
	.dev            = 	{	.platform_data = &lcd_backlight_data,
	},
};

static struct platform_driver lcd_backlight_driver = {
	.driver = {
		.name =		"lcd-backlight",
		.owner =	THIS_MODULE,
	},
	/* REVISIT add suspend() and resume() methods */
	.remove =	__exit_p(lcd_backlight_remove),
};

static int __init lcd_backlight_init(void)
{
     int ret;
     struct proc_dir_entry *lcdbltlevel_proc_file;

    printk("------------ lcd_backlight_init\n");
     ret = platform_device_register(&lcd_backlight_device);
     if(ret)
     {
         printk("[lcd_backlight_init]Error: Can not register LCD backlight device\n");
	 return ret;
     }
     ret = platform_driver_probe(&lcd_backlight_driver, lcd_backlight_probe);
     if(ret)
     {
         printk("[lcd_backlight_init]Error: Can not register LCD backlight driver\n");
	 platform_device_unregister(&lcd_backlight_device);
	 return ret;
     }

      lcdbltlevel_proc_file = create_proc_entry( "lcd-bltlevel", 0644, &proc_root);
      if( lcdbltlevel_proc_file != NULL )
      {
          lcdbltlevel_proc_file->read_proc = lcdbltlevel_proc_read;
          lcdbltlevel_proc_file->write_proc = lcdbltlevel_proc_write;
      }
      else
          printk("[lcd_backlight_init]Error: fail to create /proc/lcd-bltlevel\n");

      return 0;
}
module_init(lcd_backlight_init);

static void __exit lcd_backlight_exit(void)
{
	platform_driver_unregister(&lcd_backlight_driver);
	platform_device_unregister(&lcd_backlight_device);
}
module_exit(lcd_backlight_exit);

MODULE_DESCRIPTION("Driver for LCD with PWM-controlled brightness");
MODULE_LICENSE("GPL");

