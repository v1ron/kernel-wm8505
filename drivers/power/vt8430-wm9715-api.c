/********************************************************************************************
 * Brief: 
 *	Provide some functions of wm9715 to the driver of touch and battery.
 * History:
 *	Created	2009-4-21
 ********************************************************************************************/
#include <linux/device.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <asm/semaphore.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
//#include <asm/hardware.h>
#include <linux/proc_fs.h>
#include <mach/ac97.h>

//#include <sound/driver.h>
#include <sound/core.h>
#include <sound/ac97_codec.h>
#include "wm9715-ts.h"
#include "vt8430-wm9715-api.h"

/*
 * Bits definitions
 */
#define BIT0                0x00000001
#define BIT1                0x00000002
#define BIT2                0x00000004
#define BIT3                0x00000008
#define BIT4                0x00000010
#define BIT5                0x00000020
#define BIT6                0x00000040
#define BIT7                0x00000080
#define BIT8                0x00000100
#define BIT9                0x00000200
#define BIT10               0x00000400
#define BIT11               0x00000800
#define BIT12               0x00001000
#define BIT13               0x00002000
#define BIT14               0x00004000
#define BIT15               0x00008000
#define BIT16               0x00010000
#define BIT17               0x00020000
#define BIT18               0x00040000
#define BIT19               0x00080000
#define BIT20               0x00100000
#define BIT21               0x00200000
#define BIT22               0x00400000
#define BIT23               0x00800000
#define BIT24               0x01000000
#define BIT25               0x02000000
#define BIT26               0x04000000
#define BIT27               0x08000000
#define BIT28               0x10000000
#define BIT29               0x20000000
#define BIT30               0x40000000
#define BIT31               0x80000000





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
static int delay = 1;
module_param(delay, int, 0);
MODULE_PARM_DESC(delay, "Set adc sample delay.");

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

static void my_udelay(int n)
{
	volatile int i;
	for(i=0;i<100*n;i++)
		;
}
#define udelay	my_udelay

static u16 wm9715_reg_read(u16 reg)
{
    u16 val;
    int ret;
    ret = codec_read(reg, &val);
    if( ret )
    {
        printk("wm9715 reg%X read error: %d\n", reg, ret);
    }
    return val;
	//return ucb->ac97->bus->ops->read(ucb->ac97, reg);
}

static int wm9715_reg_write(u16 reg, u16 val)
{
    int ret = codec_write(reg, val);
    if( ret )
    {
        printk("wm9715 reg%X write 0x%X error: %d\n", reg, val, ret);
    }
    return ret;
}


// function
void wm9715_pendown_detect(void)
{
	u16 val;

	mutex_lock(&(l_wm9715context->api_mutex));
	val = wm9715_reg_read(AC97_GPIO_CFG);
	val &= ~BIT3;
	wm9715_reg_write(AC97_GPIO_CFG, val );


	val = wm9715_reg_read(AC97_MISC_AFE);
	val &= ~BIT3;
	wm9715_reg_write(AC97_MISC_AFE, val );

	val = wm9715_reg_read(AC97_WM97XX_DIGITISER2);
	val |= (BIT15 | BIT14);
	wm9715_reg_write(AC97_WM97XX_DIGITISER2, val  );
	mutex_unlock(&l_wm9715context->api_mutex);
}

inline int wm9715_pen_down(void)
{
	u16 val;

	mutex_lock(&l_wm9715context->api_mutex);
	val = wm9715_reg_read(AC97_WM97XX_DIGITISER_RD);
	mutex_unlock(&l_wm9715context->api_mutex);
	return val & BIT15;
}

/*
 * Delay after issuing a POLL command.
 *
 * The delay is 3 AC97 link frames  the touchpanel settling delay
 */
static inline void poll_delay(int d)
{
	udelay (3 * AC97_LINK_FRAME * delay_table [d]);
}


/*
 * Read a sample from the WM9712 adc in polling mode.
 */
int wm9712_poll_sample (int adcsel, int *sample)
{
	int timeout = 5 * delay;
	int ret;

	mutex_lock(&l_wm9715context->api_mutex);
	wm9715_reg_write(AC97_WM97XX_DIGITISER1, adcsel | WM97XX_POLL | WM97XX_DELAY(delay));

	/* wait 3 AC97 time slots  delay for conversion */
	poll_delay (delay);

	/* wait for POLL to go low */
	while ((wm9715_reg_read(AC97_WM97XX_DIGITISER1) & WM97XX_POLL) && timeout) {
		udelay(AC97_LINK_FRAME);
		timeout--;
	}

	if (timeout <= 0) {
		dbg ("adc sample timeout");
		ret = RC_TIMEOUT;
		goto end;
	}

	*sample = wm9715_reg_read(AC97_WM97XX_DIGITISER_RD);

	/* check we have correct sample */
	if ((*sample & WM97XX_ADCSEL_MASK) != adcsel) {
		dbg ("adc wrong sample, read %x got %x", adcsel, *sample & WM97XX_ADCSEL_MASK);
		ret = RC_ERROR;
		goto end;
	}

	if (!(*sample & WM97XX_PEN_DOWN)) {
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
	if (codec_write(AC97_WM97XX_DIGITISER1, WM97XX_ADCSEL_BMON | WM97XX_POLL | WM97XX_DELAY(1)) != 0)
	{
		dbg("Error when writing ac97 codec register.\n");
		ret = -1;
		goto sampleend;
	}

	/* wait 3 AC97 time slots  delay for conversion */
	poll_delay (1);

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

MODULE_DESCRIPTION("WM9715 api for the driver of touchscreen and battery");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("VIA ShenZhen MCE SW Team");

