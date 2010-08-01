/********************************************************************************************
 * Brief: 
 *	Provide some functions of wm9715 to the driver of touch and battery.
 * History:
 *	Created	2009-4-21
 ********************************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
//#include <asm/semaphore.h>
#include <linux/completion.h>
#include <linux/delay.h>
//#include <asm/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <mach/hardware.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <mach/ac97.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include "wm9715-ts.h"
#include "wm9715-api.h"

/**********************************Const macro*******************************************************/

#define WM97XX_XYDECOUPLED_10NF 1
#define WM97XX_TS_RPU 0x01  // 10nF--> 0x1F (less sensitive)
                          // nornaml --> 1 (most sensitive)

/*
 * Set adc sample delay.
 *
 * For accurate touchpanel measurements, some settling time may be
 * required between the switch matrix applying a voltage across the
 * touchpanel plate and the ADC sampling the signal.
 *
 * This delay can be set by setting delay = n, where n is the array
 * position of the delay in the array delay_table below.
 * Long delays > 1ms are supported for completeness, but are not
 * recommended.
 */
static int delay = 3;
module_param(delay, int, 0);
MODULE_PARM_DESC(delay, "Set adc sample delay.");




//#define DEBUG

#ifdef DEBUG
	#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
	#define dbg(format, arg...)
#endif


/*
 * ADC sample delay times in uS
 */
static const int delay_table[] = {
	21,	   // 1 AC97 Link frames
	42,	   // 2
	84,	   // 4
	167,   // 8
	333,   // 16
	667,   // 32
	1000,  // 48
	1333,  // 64
	2000,  // 96
	2667,  // 128
	3333,  // 160
	4000,  // 192
	4667,  // 224
	5333,  // 256
	6000,  // 288
	0	   // No delay, switch matrix always on
};

struct wm9715_api_context_tag
{
	struct mutex api_mutex;
};

static struct wm9715_api_context_tag* l_wm9715context = NULL;
extern struct codec_s *codec_attach(void);
extern void ac97_init(void);
extern int codec_read(u16 addr, u16 *data);
extern int codec_write(u16 addr, u16 data);
struct codec_s * pcodec;



// function
void wm9715_pendown_detect(void)
{
	u16 val;

	mutex_lock(&l_wm9715context->api_mutex);
	codec_read(AC97_GPIO_CFG, &val);
	val &= ~BIT3;
	codec_write(AC97_GPIO_CFG, val );


	codec_read(AC97_MISC_AFE, &val);
	val &= ~BIT3;
	codec_write(AC97_MISC_AFE, val );

	codec_read(AC97_WM97XX_DIGITISER2, &val);
	val |= (BIT15 | BIT14 ); 
	val &= (~BIT12);
	val |= WM97XX_TS_RPU;
	val |= BIT8;
	codec_write(AC97_WM97XX_DIGITISER2, val  );
	mutex_unlock(&l_wm9715context->api_mutex);
}

inline int wm9715_pen_down(void)
{
	u16 val;

	mutex_lock(&l_wm9715context->api_mutex);
	codec_read(AC97_WM97XX_DIGITISER_RD, &val);
	mutex_unlock(&l_wm9715context->api_mutex);
	return val & BIT15;
}


void wm9715_gpio_config(void)
{
	u16 val;
	mutex_lock(&l_wm9715context->api_mutex);

	codec_read(AC97_GPIO_CFG, &val);
	val &= ~BIT2;
	codec_write(AC97_GPIO_CFG, val );

	codec_read(AC97_MISC_AFE, &val);
	val &= ~BIT2;
	codec_write(AC97_MISC_AFE, val );
	
	codec_read(AC97_GPIO_POLARITY, &val);
	val &= ~BIT14;
	codec_write(AC97_GPIO_POLARITY, val );


	codec_read(AC97_ADF1_CTL, &val);
	//comp2del and wakeup enable and irqinv
	val |= BIT1;
	codec_write(AC97_ADF1_CTL, val );

	codec_read(AC97_ADF2_CTL, &val);
	//comp2 reference vol and signal source
	val &= ~BIT9;
	val |= BIT10;
	val &= ~BIT11;
	codec_write(AC97_ADF2_CTL, val );

	mutex_unlock(&l_wm9715context->api_mutex);
}



/*
 * Delay after issuing a POLL command.
 *
 * The delay is 3 AC97 link frames + the touchpanel settling delay
 */
static inline void poll_delay(int d)
{
	//udelay (3 * AC97_LINK_FRAME * delay_table [d]);
	udelay (3 * AC97_LINK_FRAME  + delay_table [d]);
}


/*
 * Read a sample from the WM9712 adc in polling mode.
 */
int wm9712_poll_sample (int adcsel, int *sample)
{
	int timeout = 5 * delay;
	int ret;
	u16 regval;
	#if WM97XX_XYDECOUPLED_10NF
	int suggest_delay = 4;
	#else
	int suggest_delay = delay;
	#endif

	mutex_lock(&l_wm9715context->api_mutex);
	codec_write(AC97_WM97XX_DIGITISER1, adcsel | WM97XX_POLL | WM97XX_DELAY(suggest_delay));

	/* wait 3 AC97 time slots  delay for conversion */
	poll_delay (delay);

	/* wait for POLL to go low */
	do {
		udelay(AC97_LINK_FRAME);
		if (codec_read(AC97_WM97XX_DIGITISER1, &regval) != 0)
		{
			dbg("Error when reading ac97 codec register.\n");
			ret = RC_CODECREG_ERROR;
			goto end;
		}
		timeout--;
	} while (((regval & WM97XX_POLL) != 0) && (timeout != 0));


	if (timeout <= 0) {
		dbg ("adc sample timeout");
		ret = RC_TIMEOUT;
		goto end;
	}

	if(codec_read(AC97_WM97XX_DIGITISER_RD, &regval) != 0)
	{
		dbg("[%s ]Error when read codecreg AC97_WM97XX_DIGITISER_RD.\n", __func__);
		ret = RC_CODECREG_ERROR;
		goto end;
	}

	/* check we have correct sample */
	*sample = regval;
	if ((regval & WM97XX_ADCSEL_MASK) != adcsel) {
		dbg ("adc wrong sample, read %x got %x", adcsel, *sample & WM97XX_ADCSEL_MASK);
		ret = RC_ERROR;
		goto end;
	}

	if (!(regval & WM97XX_PEN_DOWN)) {
//		the_wm9715->pen_probably_down = 0;
		ret = RC_PENUP;
		goto end;
	}
	ret = RC_VALID;
end:
	mutex_unlock(&l_wm9715context->api_mutex);
	return ret;
}

int wm9712_poll_sample_battery (int *sample)
{
	int timeout = 1000;
	unsigned short regval = 0;
	int ret;
	#if WM97XX_XYDECOUPLED_10NF
	int suggest_delay = 4;
	#else
	int suggest_delay = delay;
	#endif

	mutex_lock(&l_wm9715context->api_mutex);
	if (codec_read(AC97_WM97XX_DIGITISER2, &regval) != 0)
	{
		dbg("Error when read ac97 codec register.\n");
		ret = -1;
		goto sampleend;
	}
	regval |= (BIT14|BIT15);
	if (codec_write(AC97_WM97XX_DIGITISER2, regval) != 0)
	{
		dbg("Error when write ac97 codec register.\n");
		ret = -1;
		goto sampleend;
	}
	if (codec_write(AC97_WM97XX_DIGITISER1, WM97XX_ADCSEL_BMON | WM97XX_POLL | WM97XX_DELAY(suggest_delay)) != 0)
	{
		dbg("Error when writing ac97 codec register.\n");
		ret = -1;
		goto sampleend;
	}

	/* wait 3 AC97 time slots  delay for conversion */
	poll_delay (delay);

	/* wait for POLL to go low */
	do {
		udelay(AC97_LINK_FRAME);
		if (codec_read(AC97_WM97XX_DIGITISER1, &regval) != 0)
		{
			dbg("Error when reading ac97 codec register.\n");
			ret = -1;
			goto sampleend;
		}
		timeout--;
	} while (((regval & WM97XX_POLL) != 0) && (timeout != 0));
	
	
	if (timeout <= 0) {
		/* If PDEN is set, we can get a timeout when pen goes up */
// 		if (wm9715_pen_down())
// 			//the_wm9715->pen_probably_down = 0;
// 			the_wm9715->pen_is_down = 0;
// 		else
// 			dbg ("adc sample timeout");
		dbg("adc sample timeout.....\n");
		ret = -2;
		goto sampleend;
	}

	if (codec_read(AC97_WM97XX_DIGITISER_RD, &regval) != 0)
	{
		dbg("Error when reading ac97 codec register.\n");
		ret = -1;
		goto sampleend;
	}

	*sample = 0x0FFF & regval; // the fact value
	//*sample = 2*(*sample)*3*3300/0xFFF;
	//dbg ("adc  sample, got %d\n", *sample);
	ret = RC_VALID;
sampleend:
	mutex_unlock(&l_wm9715context->api_mutex);
	return ret;
}

static int __init wm9715_api_init(void)
{
	l_wm9715context = kmalloc(sizeof(struct wm9715_api_context_tag), GFP_KERNEL);
	if (NULL == l_wm9715context)
	{
		return -ENOMEM;
	}
	mutex_init(&l_wm9715context->api_mutex);
	printk("%s %d\n",__FILE__,__LINE__);
	pcodec = codec_attach();
	pcodec->ops->init();
	printk("%s %d\n",__FILE__,__LINE__);
	return 0;
}

static void __exit wm9715_api_exit(void)
{
	if (l_wm9715context != NULL)
	{
		kfree(l_wm9715context);
	}

}

module_init(wm9715_api_init);
module_exit(wm9715_api_exit);

EXPORT_SYMBOL(wm9715_pendown_detect);
EXPORT_SYMBOL(wm9715_pen_down);
EXPORT_SYMBOL(wm9712_poll_sample);
EXPORT_SYMBOL(wm9712_poll_sample_battery);
EXPORT_SYMBOL(wm9715_gpio_config);

MODULE_DESCRIPTION("WM9715 api for the driver of touchscreen and battery");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("VIA ShenZhen MCE SW Team");

