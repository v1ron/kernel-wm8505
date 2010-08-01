/*******************************************************************************
	Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.

	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.

	WonderMedia Technologies, Inc.
	10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.


*******************************************************************************/
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/timer.h>

#include <mach/hardware.h>
#include <asm/memory.h>
#include <asm/system.h>
#include <asm/leds.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>

//#define CONFIG_KBDC_WAKEUP    //removed by howayhuo. we only need power button wakeup
//#define KB_WAKEUP_SUPPORT
//#define MOUSE_WAKEUP_SUPPORT
#define	SOFT_POWER_SUPPORT
//#define	RTC_WAKEUP_SUPPORT
#define     KEYPAD_POWER_SUPPORT

#define DRIVER_NAME	"PMC"
#if defined(CONFIG_PM_RTC_IS_GMT) && defined(RTC_WAKEUP_SUPPORT)
#include <linux/rtc.h>
#endif

#if defined(KEYPAD_POWER_SUPPORT)
#include <linux/input.h>
#include <linux/suspend.h>
#define KPAD_POWER_FUNCTION_NUM  2
static struct input_dev *kpadPower_dev;
static unsigned int kpadPower_codes[KPAD_POWER_FUNCTION_NUM] = {
        [0] = KEY_END,
        [1] = KEY_ESC,    
};
static unsigned int powerKey_is_pressed;
static unsigned int pressed_jiffies;
static struct timer_list   kpadPower_timer, vibra_timer;
static spinlock_t kpadPower_lock;
static int vibra_enable;

extern int wmt_getsyspara_cache(char *varname, unsigned char *varval, int varlen);
#endif

#define DEBUG
/*
 *  Debug macros
 */
#ifdef DEBUG
#  define DPRINTK(fmt, args...) printk(KERN_ALERT "%s: " fmt, __func__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

/*
 * For saving discontinuous registers during hibernation.
 */
#define SAVE(x)		(saved[SAVED_##x] = (x##_VAL))
#define RESTORE(x)	((x##_VAL) = saved[SAVED_##x])

enum {
	SAVED_SP = 0,
	SAVED_OSTW, SAVED_OSTI,
	SAVED_PMCEL, SAVED_PMCEU,
	/* SAVED_ATAUDMA, */
	SAVED_SIZE
};

struct apm_dev_s {
    char id;
};


extern unsigned int wmt_read_oscr(void);
extern void wmt_read_rtc(unsigned int *date, unsigned int *time);

/* Hibernation entry table physical address */
#define LOADER_ADDR												0xffff0000
#define HIBERNATION_ENTER_EXIT_CODE_BASE_ADDR	0xFFFFFFC0
#define DO_POWER_ON_SLEEP			            (HIBERNATION_ENTER_EXIT_CODE_BASE_ADDR + 0x00)
#define DO_POWER_OFF_SUSPEND			        (HIBERNATION_ENTER_EXIT_CODE_BASE_ADDR + 0x04)

static unsigned int exec_at = (unsigned int)-1;
static void (*theKernel)(void);

#if defined(SOFT_POWER_SUPPORT) && defined(CONFIG_PROC_FS)
static struct proc_dir_entry *proc_softpower;
static unsigned int softpower_data;
#endif

static long rtc2sys;

static struct proc_dir_entry *proc_shutdown;
struct apm_dev_s *pm_dev;
struct tasklet_struct	PMC_tasklet;
struct work_struct	PMC_shutdown;
static struct work_struct l_reportwork;
static int l_specialpwbtn = 0; // 0--no,1--puzhi

unsigned long PMC_status;
#define PMC_status_normal  0  /*normal status*/
#define PMC_status_turn_off_video 1   /*turn off vedio status*/
#define PMC_status_shutdown 2           /*shutdown*/

static unsigned int shutdown_time_slot = 2;
struct timer_list shutdown_timer;
int shutdown_counter;

#define not_shutdown 0
#define prepare_shutdown 1
int shutdown_status = not_shutdown;  /*0 not shutdown   1 prepare shutdown*/

char hotplug_path[256] = "/sbin/hotplug";


#define PM_resume_status_normal 0      /*under normal status*/
#define PM_resume_status_on_going 1   /*still execute resume process and the power button still not release*/
#define PM_resume_status_on_check 2  /*on check if the power button is release*/
unsigned int PM_resume_status = PM_resume_status_normal; /*to record if execute resume process*/
struct timer_list PM_resume_status_timer;

static void run_pmc_softpower(unsigned long arg);
static void pm_check_resume_status(unsigned long arg);

#define REG_VAL(addr) (*((volatile unsigned int *)(addr)))


static void wmt_tasklet_pmc(unsigned long status)
{
	schedule_work(&PMC_shutdown);
}

static void report_end(struct work_struct *work)
{
	msleep(600);
	input_report_key(kpadPower_dev, KEY_END, 0);
	input_sync(kpadPower_dev);
}

static void run_shutdown(struct work_struct *work)
{
	int ret;
    char *argv[] = { hotplug_path, "PMC", NULL };
	char *envp_turn_off_video[] =
		{ "HOME=/", "PATH=/sbin:/bin:/usr/sbin:/usr/bin", "ACTION=turn_off_video", NULL };
	char *envp_shutdown[] =
		{ "HOME=/", "PATH=/sbin:/bin:/usr/sbin:/usr/bin", "ACTION=shutdown", NULL };
    if (PMC_status == PMC_status_turn_off_video)
		ret = call_usermodehelper(argv[0], argv, envp_turn_off_video, 0);
	if (PMC_status == PMC_status_shutdown)
		ret = call_usermodehelper(argv[0], argv, envp_shutdown, 0);
}

/* Blink blue led by GPIO 0 */
/*
static void led_light(unsigned int count)
{
    unsigned int result = 0;
    while (count-- > 0) {
        REG_VAL(0xD8110064) = 0XFF;
        REG_VAL(0xD811008C) = 0XFF;
        REG_VAL(0xD81100B4) = 0X00;
        for (result = 8000000; result > 0; result--)
            REG_VAL(0xD81100B4) = 3;
        for (result = 8000000; result > 0; result--)
            REG_VAL(0xD81100B4) = 0;
    }
}
*/
extern suspend_state_t requested_suspend_state;

static irqreturn_t pmc_wakeup_isr(int this_irq, void *dev_id)
{
	unsigned short status;
//	int i;

#if defined(KEYPAD_POWER_SUPPORT)
    spin_lock_irq(&kpadPower_lock);
    status = PMWS_VAL;          /* Copy the wakeup status */
    if((status & BIT14) && kpadPower_dev && (requested_suspend_state != PM_SUSPEND_MEM))
    {

        if(!powerKey_is_pressed)
        {
            powerKey_is_pressed = 1;
            switch (l_specialpwbtn)
            {
            	case 1: // puzhi do nothing
            		break;
            	default:
            		input_report_key(kpadPower_dev, KEY_END, 1); //power key is pressed
            		input_sync(kpadPower_dev);
            		break;
            };
            
            pressed_jiffies = jiffies;

	    if(vibra_enable)
	        mod_timer(&vibra_timer, jiffies + msecs_to_jiffies(1000));

            disable_irq(IRQ_PMC_WAKEUP);
            switch (l_specialpwbtn)
            {
            	case 1: // puzhi
            		mod_timer(&kpadPower_timer, jiffies + msecs_to_jiffies(200));
            		break;
            	default:
            		mod_timer(&kpadPower_timer, jiffies + msecs_to_jiffies(300));
            		break;
            };
            //
            
            DPRINTK("power key pressed -->\n");
        }
//        else
//        {
 //           if(jiffies - pressed_jiffies >= ms_to_jiffies(400))
//            {
//                input_event(kpadPower_dev, EV_KEY, KEY_END, 2); // power key repeat
//                input_sync(kpadPower_dev);
//                DPRINTK("power key repeat\n");
  //          }
//        }
//	disable_irq(IRQ_PMC_WAKEUP);
//        mod_timer(&kpadPower_timer, jiffies + msecs_to_jiffies(300));
//	PMWS_VAL |= BIT14;
    }
    spin_unlock_irq(&kpadPower_lock);
#else
    status = PMWS_VAL;          /* Copy the wakeup status */
#endif

	udelay(100);
	PMWS_VAL = status;
#if 0
	if (status & BIT14) {       /* Power button wake up */
#if defined(SOFT_POWER_SUPPORT) && defined(CONFIG_PROC_FS)
		softpower_data = 1;
#endif
		/*press the power button, prepare to shutdown */
		if ((shutdown_status == not_shutdown)
			&& (PM_resume_status == PM_resume_status_normal)) {
			shutdown_counter = 0;
			init_timer(&shutdown_timer);
			shutdown_timer.function = run_pmc_softpower;
			shutdown_timer.data = 0;
			shutdown_timer.expires = jiffies + (HZ/100);
			shutdown_status = prepare_shutdown;  /*prepare shutdown*/
			disable_irq(IRQ_PMC_WAKEUP);
			add_timer(&shutdown_timer);
		}
	}

	if (PM_resume_status == PM_resume_status_on_going) {
		init_timer(&PM_resume_status_timer);
		PM_resume_status_timer.function = pm_check_resume_status;
		PM_resume_status_timer.data = 0;
		PM_resume_status_timer.expires = jiffies + (HZ/100);
		PM_resume_status = PM_resume_status_on_check;
		disable_irq(IRQ_PMC_WAKEUP);
		add_timer(&PM_resume_status_timer);
	}
#endif

#ifdef RTC_WAKEUP_SUPPORT
	if((*(volatile unsigned short*)0xd8120002) == 0x3426)
		if((*(volatile unsigned short*)0xd8120000) >= 0x0103)
	if (status & BIT15)        /* Check RTC wakeup status bit */
		PMWS_VAL |= BIT15;
#endif

#ifdef MOUSE_WAKEUP_SUPPORT
	if (status & BIT11)
		PMWS_VAL |= BIT11;
#endif

#ifdef KB_WAKEUP_SUPPORT
	if (status & BIT11)
		PMWS_VAL |= BIT10;
#endif
	if(status & BIT0)
		PMWS_VAL |= BIT0;

//        PMWS_VAL = status;
/*
	for(i = 0; i < 10; i++)
	{
	    PMWS_VAL |= BIT14;
//	    mdelay(1);
	    if(!(PMWS_VAL & BIT14))
		break;
	}
        printk("pmc isr i = %d\n", i);
*/
	return IRQ_HANDLED;
}

extern int PM_device_PostSuspend(void);
extern int PM_device_PreResume(void);

/* wmt_pm_standby()
 *
 * Entry to the power-on sleep hibernation mode.
 */
static void wmt_pm_standby(void)
{
	volatile unsigned int hib_phy_addr = 0,base = 0;
	int result;

    /* Enable Wake on Lan feature [mac0] */
    /*PMWE_VAL = PMWE_WAKEUP(4);*/
    /*PMWT_VAL = PMWT_WAKEUP4(PMWT_ONE);*/

    /* Get standby virtual address entry point */
	base = (unsigned int)ioremap_nocache(LOADER_ADDR, 0x10000);
	exec_at = base + (DO_POWER_ON_SLEEP - LOADER_ADDR);
    hib_phy_addr = *(unsigned int *) exec_at;
	exec_at = base + (hib_phy_addr - LOADER_ADDR);

    /*led_light(2);*/
    theKernel = (void (*)())exec_at;		/* set rom address */
    theKernel();					        /* jump to rom */
 		iounmap((void *)base);
    /*led_light(2);*/
	result = PM_device_PreResume();
	if (result)  //modified by howayhuo
		printk(KERN_ALERT "PM_device_PreResume fail\n");
}

static void pm_check_resume_status(unsigned long arg)
{
	unsigned short status;

	status = PMWS_VAL;
	if (status & BIT14) {     /*the power button still be pressed*/
		PMWS_VAL |= BIT14;    /*clear the power button status*/
		/*wait 100ms, then enter the function again*/
		mod_timer(&PM_resume_status_timer, jiffies + (HZ/10));
	} else {
		PM_resume_status = PM_resume_status_normal;
		del_timer(&PM_resume_status_timer);
		enable_irq(IRQ_PMC_WAKEUP);
	}
}
static void run_pmc_softpower(unsigned long arg)
{
	unsigned short status;

	status = PMWS_VAL;

	if (shutdown_counter < (shutdown_time_slot * 10)) {
		if (status & BIT14) {     /*the power button still be pressed*/
			PMWS_VAL |= BIT14;    /*clear the power button status*/
			shutdown_counter++;
			/*wait 100ms, then enter the function again*/
			mod_timer(&shutdown_timer, jiffies + (HZ/10));
		} else {
			enable_irq(IRQ_PMC_WAKEUP);
			del_timer(&shutdown_timer);
			shutdown_counter = 0;
			shutdown_status = not_shutdown;
		}
	} else {

		if (status & BIT14) {     /*the power button still be pressed*/
			if (PMC_status == PMC_status_normal) {
				PMC_status = PMC_status_turn_off_video;
				tasklet_schedule(&PMC_tasklet);    /*call turn off vedio*/
			}
			PMWS_VAL |= BIT14;    /*clear the power button status*/
			shutdown_counter++;
			/*wait 100ms, then enter the function again*/
			mod_timer(&shutdown_timer, jiffies + (HZ/10));

		} else {
			PMC_status = PMC_status_shutdown;
			tasklet_schedule(&PMC_tasklet);    /*call shutdown*/
			del_timer(&shutdown_timer);
			shutdown_counter = 0;
			shutdown_status = prepare_shutdown;
			enable_irq(IRQ_PMC_WAKEUP);
		}

	}
}

/* wmt_pm_suspend()
 *
 * Entry to the power-off suspend hibernation mode.
 */
static void wmt_pm_suspend(void)
{
	unsigned int saved[SAVED_SIZE];
	int result;

	volatile unsigned int hib_phy_addr = 0,base = 0;

/* FIXME */
#if 0
	result = PM_device_PostSuspend();
	if (result)
		printk(KERN_ALERT "PM_device_PostSuspend fail\n");
#endif

	SAVE(OSTW);	                /* save vital registers */
	SAVE(OSTI);
	SAVE(PMCEL);	            /* save clock gating */
	SAVE(PMCEU);
		base = (unsigned int)ioremap_nocache(LOADER_ADDR, 0x10000);

    /* Get suspend virtual address entry point */
		exec_at = base + (DO_POWER_OFF_SUSPEND - LOADER_ADDR);
    hib_phy_addr = *(unsigned int *) exec_at;
		exec_at = base + (hib_phy_addr - LOADER_ADDR);

    /*led_light(4);*/
    theKernel = (void (*)())exec_at;
    theKernel();

    /*led_light(4);*/
		iounmap((void *)base);
	RESTORE(PMCEU);             /* restore clock gating */
	RESTORE(PMCEL);
	RESTORE(OSTI);              /* restore vital registers */
	RESTORE(OSTW);

//--> modified by howayhuo to clear "suspend to ram" bit
//-- org
//	PMPB_VAL |= 1;				/* cant not clear RGMii connection */
//-- new
    PMPB_VAL = 3;
//<-- end modification

    result = PM_device_PreResume();
	if (result)  //modified by howayhuo
		printk(KERN_ALERT "PM_device_PreResume fail\n");
}

/* wmt_pm_enter()
 *
 * To Finally enter the sleep state.
 *
 * Note: Only support PM_SUSPEND_STANDBY and PM_SUSPEND_MEM
 */
static int wmt_pm_enter(suspend_state_t state)
{
	unsigned short status;
    volatile unsigned int Wake_up_enable = 0;
    volatile unsigned int Wake_up_type = 0;

    printk("wmt_pm_enter\n");
	if (!((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM))) {
		printk(KERN_ALERT "%s, Only support PM_SUSPEND_STANDBY and PM_SUSPEND_MEM\n", DRIVER_NAME);
		return -EINVAL;
	}

#ifdef RTC_WAKEUP_SUPPORT
	if((*(volatile unsigned short*)0xd8120002) == 0x3426)
		if((*(volatile unsigned short*)0xd8120000) >= 0x0103)
	Wake_up_enable |= (PMWE_RTC);
#endif

#ifdef CONFIG_KBDC_WAKEUP
	i8042_write_2e(0xE0);
	i8042_write_2f(0xB);

#ifdef MOUSE_WAKEUP_SUPPORT
	Wake_up_enable |= (BIT11);
	i8042_write_2e(0xE9);
	i8042_write_2f(0x00);
#endif
#ifdef KB_WAKEUP_SUPPORT
	Wake_up_enable |= (BIT10);
	i8042_write_2e(0xE1);
	i8042_write_2f(0x00);
#endif
#endif


	/* only enable fiq in normal operation */
	local_fiq_disable();
	local_irq_disable();

	/* disable system OS timer */
	OSTC_VAL &= ~OSTC_ENABLE;

	PMWS_VAL = PMWS_VAL;

    /*
     * choose wake up event
     */
#if 0   //needn't USB and Eth wakeup. removed by howayhuo
    if (state == PM_SUSPEND_STANDBY) {                   /* PM_SUSPEND_MEM */
		Wake_up_enable |= ((1 << 9) | (1 << 12) | (1 << 13));
    } else {                                              /* PM_SUSPEND_MEM */
		Wake_up_enable |= ((1 << 9) | (1 << 12) | (1 << 13));
    }
#endif

    /* 2008/09/24 Neil
     * Write to Wake-up Event Enable Register ONLY once, it is becausebe this register is
     * synced into RTC clock domain. Which means two consequtive write cannot be too close
     * (wait more than 2~3 RTC clocks). Otherwise the later write data will be missing.
     */
    //PMWE_VAL = Wake_up_enable;
    //PMWT_VAL = Wake_up_type;

    PMWE_VAL = Wake_up_enable |PMWE_WAKEUP(0);
    PMWT_VAL = Wake_up_type | PMWT_WAKEUP0(PMWT_FALLING);


	/*
	 * We use pm_standby as apm_standby for power-on hibernation.
	 * but we still suspend memory in both case.
	 */
	if (state == PM_SUSPEND_STANDBY)
        wmt_pm_standby();    /* Go to standby mode*/
    else
		wmt_pm_suspend();    /* Go to suspend mode*/

	/*
	 * Clean wakeup source
	 */
	if (PMWS_VAL & BIT14)
		PM_resume_status = PM_resume_status_on_going;

	status = PMWS_VAL;
	udelay(100);
	PMWS_VAL = status;

#ifdef RTC_WAKEUP_SUPPORT
	if((*(volatile unsigned short*)0xd8120002) == 0x3426)
		if((*(volatile unsigned short*)0xd8120000) >= 0x0103)
	RTAS_VAL = 0x0;       	/* Disable RTC alarm */
#endif

    /*
	 * Force to do once CPR for system.
	 */
	OSM1_VAL = wmt_read_oscr() + LATCH;
	OSTC_VAL |= OSTC_ENABLE;

    /*udelay(200);*/  /* delay for resume not complete */

	local_irq_enable();
	local_fiq_enable();

	/*
	 * reset wakeup settings
	 */
	PMWE_VAL = 0;

	return 0;
}

static int wmt_pm_valid(suspend_state_t state)
{
    return 1;
}


/* wmt_pm_prepare()
 *
 * Called after processes are frozen, but before we shut down devices.
 */
static int wmt_pm_prepare(void )
{
	unsigned int date, time;
	struct timeval tv;

	/*
	 * Estimate time zone so that wmt_pm_finish can update the GMT time
	 */
         printk("wmt_pm_prepare\n");
	rtc2sys = 0;

	if ((*(volatile unsigned int *)0xd8120000) > 0x34260102)
	wmt_read_rtc(&date, &time);

	do_gettimeofday(&tv);

	rtc2sys = mktime(RTCD_YEAR(date) + ((RTCD_CENT(date) * 100) + 2000),
                     RTCD_MON(date),
                     RTCD_MDAY(date),
                     RTCT_HOUR(time),
                     RTCT_MIN(time),
                     RTCT_SEC(time));
	rtc2sys = rtc2sys-tv.tv_sec;
	if (rtc2sys > 0)
		rtc2sys += 10;
	else
		rtc2sys -= 10;
	rtc2sys = rtc2sys/60/60;
	rtc2sys = rtc2sys*60*60;

	return 0;
}

/* wmt_pm_finish()
 *
 * Called after devices are re-setup, but before processes are thawed.
 */
static void wmt_pm_finish(void)
{
	unsigned int date, time;
	struct timespec tv;

//	struct rtc_time tm;
//	unsigned long tmp = 0;
//	struct timeval tv1;

/*	printk(KERN_ALERT "%s: [wmt_pm_finish] \n", DRIVER_NAME);*/
//	iounmap((void *)exec_at);
       printk("wmt_pm_finish\n");
	/*
	 * Update kernel time spec.
	 */
	if ((*(volatile unsigned int *)0xd8120000) > 0x34260102)
    wmt_read_rtc(&date, &time);

	tv.tv_nsec = 0;
	tv.tv_sec = mktime(RTCD_YEAR(date) + ((RTCD_CENT(date) * 100) + 2000),
                       RTCD_MON(date),
                       RTCD_MDAY(date),
                       RTCT_HOUR(time),
                       RTCT_MIN(time),
                       RTCT_SEC(time));
	/* RTC stores local time, adjust GMT time, tv */
	tv.tv_sec = tv.tv_sec-rtc2sys;
	do_settimeofday(&tv);

	return;
}

static struct platform_suspend_ops wmt_pm_ops = {
	.valid              = wmt_pm_valid,
	.prepare        = wmt_pm_prepare,
	.enter          = wmt_pm_enter,
	.finish         = wmt_pm_finish,
};


#if defined(SOFT_POWER_SUPPORT) && defined(CONFIG_PROC_FS)

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

static int procfile_read(char *page, char **start, off_t off, int count,  int *eof, void *data)
{
	int len = 0;
	len = sprintf(page, "%d\n", softpower_data);
	return len;
}


static int procfile_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char *endp;
	int ret;

	softpower_data = simple_strtoul(buffer, &endp, 0);

	if (softpower_data != 0)
		softpower_data = 1;

/*	printk("%s: %s, softpower_data=[0x%X]\n", DRIVER_NAME, __FUNCTION__, softpower_data );*/
/*	printk("%s: return [%d]\n", DRIVER_NAME, (count + endp - buffer ) );*/
	ret = count + endp - buffer;
	return ret;
}

#endif	/* defined(SOFT_POWER_SUPPORT) && defined(CONFIG_PROC_FS)*/

static int procfile_shutdown_read(char *page, char **start, off_t off, int count,  int *eof, void *data)
{
	int len = 0;
	len = sprintf(page, "%d\n", shutdown_time_slot);
	return len;
}


static int procfile_shutdown_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char *endp;
	int ret;

	shutdown_time_slot = simple_strtoul(buffer, &endp, 0);

	if (shutdown_time_slot >= 4)
		shutdown_time_slot = 4;

	ret = count + endp - buffer;
	return ret;
}

#if defined(KEYPAD_POWER_SUPPORT)
static inline void
kpadPower_timeout(unsigned long fcontext)
{
    //timeout, release all key
    struct input_dev *dev = (struct input_dev *) fcontext;
    //printk("-------------------------> kpadPower time out\n");
//    int i;

    if(!kpadPower_dev)
        return;

    spin_lock_irq(&kpadPower_lock);
/*
	for(i = 0; i < 10; i++)
	{
	    PMWS_VAL |= BIT14;
//	    mdelay(1);
	    if(!(PMWS_VAL & BIT14))
		break;
	}
*/
    PMWS_VAL |= BIT14;
    mdelay(10);
    if(!(PMWS_VAL & BIT14) && powerKey_is_pressed )
//    printk("timeout i = %d\n", i);
//    if((i < 10) && powerKey_is_pressed )
    {
        printk("bit14 is cleaned\n");
        switch (l_specialpwbtn)
        {
        	case 1: // puzhi specail
        		if (jiffies - pressed_jiffies < msecs_to_jiffies(400))
		        {
		        	printk("Kt<400ms!!\n");
		        	input_report_key(dev, KEY_ESC, 1);
		        	input_report_key(dev, KEY_ESC, 0);
		        	input_sync(dev);
		        } else {
		        	printk("kt>400ms!!\n");        	
		    		input_report_key(dev, KEY_END, 1); //power key is released
				    input_sync(dev);
				    schedule_work(&l_reportwork);		    
		        }
        		break;
        	default:
        		input_report_key(dev, KEY_END, 0); //power key is released
				input_sync(dev);
        		break;
        };
        
        powerKey_is_pressed = 0;

//	if(vibra_enable)
//	    del_timer(&vibra_timer);
        enable_irq(IRQ_PMC_WAKEUP);
	DPRINTK("power key released <--\n");
    }
    else
    {
    	switch (l_specialpwbtn)
    	{
    		case 1: // puzhi
    			if (jiffies - pressed_jiffies >= msecs_to_jiffies(400))
		    	{
		    		// long press
		    		printk("kt>400ms,Pressed!!\n");
				    input_report_key(dev, KEY_END, 1); //power key is released
				    input_sync(dev);
		    	}
		    	mod_timer(&kpadPower_timer, jiffies + msecs_to_jiffies(200));
    			break;
    		default:
    			mod_timer(&kpadPower_timer, jiffies + msecs_to_jiffies(300));
    			break;
    	};
    	
    
  //                  input_event(kpadPower_dev, EV_KEY, KEY_END, 2); // power key repeat
  //              input_sync(kpadPower_dev);
        
	printk("bit14 don't clean\n");
    }

//    PMWS_VAL |= BIT14;
//    enable_irq(IRQ_PMC_WAKEUP);

    spin_unlock_irq(&kpadPower_lock);
}

static void vibra_timeout(unsigned long fcontext)
{
    unsigned long vibra_duration = 200; //msec
    int retval;
    char buf[200]={0};
    char * endp;

    //timeout, release all key
//    struct input_dev *dev = (struct input_dev *) fcontext;
    //printk("-------------------------> kpadPower time out\n");

    if(!kpadPower_dev)
        return;

    if(!vibra_enable)
	return;

    spin_lock_irq(&kpadPower_lock);

    if(!powerKey_is_pressed)
    {
        spin_unlock_irq(&kpadPower_lock);
	return;
    }

    printk("vibra_timeout\n");
     retval = wmt_getsyspara_cache("vibra_start", buf, 200);
    if(!retval)
    {
        printk("vibra_start\n");
        excute_gpio_op(buf);
	memset(buf, 0, sizeof(buf));
        retval = wmt_getsyspara_cache("vibra_time", buf, 200);
	if(!retval)
	    vibra_duration = simple_strtoul(buf, &endp, 10);
	else
	    vibra_duration = 200;

        printk("vibra_duration = %lu\n", vibra_duration);
	mdelay(vibra_duration);

        memset(buf, 0, sizeof(buf));
        retval = wmt_getsyspara_cache("vibra_stop", buf, 200);
	if(!retval)
	{
	    printk("vibra_stop\n");
	    excute_gpio_op(buf);
	}
/*
        if(powerKey_is_pressed )
        {
            input_report_key(dev, KEY_END, 0); //power key is released
            input_sync(dev);
            powerKey_is_pressed = 0;

            DPRINTK("power key released <--\n");
        }
*/
    }

    spin_unlock_irq(&kpadPower_lock);

}
#endif

extern struct proc_dir_entry proc_root;

/*
 * Initialize power management interface
 */
static int __init wmt_pm_init(void)
{
 #if defined(KEYPAD_POWER_SUPPORT)
    int i;
    int retval;
    char buf[200]={0};

    kpadPower_dev = input_allocate_device();
    if(kpadPower_dev)
    {
        //Initial the static variable
        spin_lock_init(&kpadPower_lock);
        powerKey_is_pressed = 0;
        pressed_jiffies = 0;
        init_timer(&kpadPower_timer);
        kpadPower_timer.function = kpadPower_timeout;
        kpadPower_timer.data = (unsigned long)kpadPower_dev;

        retval = wmt_getsyspara_cache("vibra_enable", buf, 200);
	if(!retval)
	{
	    vibra_enable = 1;

	    init_timer(&vibra_timer);
	    vibra_timer.function = vibra_timeout;
	    vibra_timer.data = (unsigned long)kpadPower_dev;
	}
		if (!wmt_getsyspara_cache("pwbtn", buf, 200))
		{
			if (!strcmp(buf, "puzhi"))
			{
				l_specialpwbtn = 1;
			}
		}
        /* Register an input event device. */
        set_bit(EV_KEY,kpadPower_dev->evbit);
        for (i = 0; i < KPAD_POWER_FUNCTION_NUM; i++)
            set_bit(kpadPower_codes[i], kpadPower_dev->keybit);

        kpadPower_dev->name = "kpadPower",
        kpadPower_dev->phys = "kpadPower",


        kpadPower_dev->keycode = kpadPower_codes;
        kpadPower_dev->keycodesize = sizeof(unsigned int);
        kpadPower_dev->keycodemax = KPAD_POWER_FUNCTION_NUM;

        /*
        * For better view of /proc/bus/input/devices
        */
        kpadPower_dev->id.bustype = 0;
        kpadPower_dev->id.vendor  = 0;
        kpadPower_dev->id.product = 0;
        kpadPower_dev->id.version = 0;
        input_register_device(kpadPower_dev);
    }
    else
        printk("[wmt_pm_init]Error: No memory for registering Kpad Power\n");
 #endif

	/* Press power button (either hard-power or soft-power) will trigger a power button wakeup interrupt*/
	/* Press reset button will not trigger any PMC wakeup interrupt*/
	/* Hence, force write clear all PMC wakeup interrupts before request PMC wakeup IRQ*/
	PMWS_VAL = PMWS_VAL;
	PMC_status = PMC_status_normal;
	tasklet_init(&PMC_tasklet, wmt_tasklet_pmc, PMC_status);
	INIT_WORK(&PMC_shutdown, run_shutdown);
	INIT_WORK(&l_reportwork, report_end);

#ifdef RTC_WAKEUP_SUPPORT
	if((*(volatile unsigned short*)0xd8120002) == 0x3426)
		if((*(volatile unsigned short*)0xd8120000) >= 0x0103)
	PMWE_VAL &= ~(PMWE_RTC);
#endif
	/*
	 *  set interrupt service routine
	 */
/*	if (request_irq(IRQ_PMC_WAKEUP, &pmc_wakeup_isr, SA_SHIRQ, "pmc", &pm_dev) < 0) {*/
	if (request_irq(IRQ_PMC_WAKEUP, &pmc_wakeup_isr, IRQF_DISABLED, "pmc", &pm_dev) < 0)
		printk(KERN_ALERT "%s: [Wondermedia_pm_init] Failed to register pmc wakeup irq \n"
               , DRIVER_NAME);

#ifndef CONFIG_SKIP_DRIVER_MSG
	/*
     * Plan to remove it to recude core size in the future.
     */
	printk(KERN_INFO "%s: WonderMedia Power Management driver\n", DRIVER_NAME);
#endif

	/*
     * Setup PM core driver into kernel.
     */
	suspend_set_ops(&wmt_pm_ops);

#if defined(SOFT_POWER_SUPPORT) && defined(CONFIG_PROC_FS)

	/* Power button is configured as soft power*/
	printk(KERN_INFO "%s: Power button is configured as soft power\n", DRIVER_NAME);
	PMPB_VAL |= PMPB_SOFTPWR;

	/* Create proc entry*/
	proc_softpower = create_proc_entry("softpower", 0644, &proc_root);
	proc_softpower->data = (void *)softpower_data;
	proc_softpower->read_proc = procfile_read;
	proc_softpower->write_proc =  procfile_write;
	softpower_data = 1;

#else
	/* Power button is configured as hard power*/
	printk(KERN_INFO "%s: Power button is configured as hard power\n", DRIVER_NAME);
	PMPB_VAL = 0;

#endif	/* defined(SOFT_POWER_SUPPORT) && defined(CONFIG_PROC_FS)*/

	/* Create proc shutdown entry*/
	proc_shutdown = create_proc_entry("shutdown", 0666, &proc_root);
	proc_shutdown->data = (void *)shutdown_time_slot;
	proc_shutdown->read_proc = procfile_shutdown_read;
	proc_shutdown->write_proc =  procfile_shutdown_write;
/*
	for(i = 0; i < 10; i++)
	{
	    PMWS_VAL |= BIT14;
	    mdelay(1);
	    if(!(PMWS_VAL & BIT14))
		break;
	}

	printk("------------- pm: i=%d\n", i);
*/
	return 0;
}

void send_ESC_key(void)
{
        if(!kpadPower_dev)
        {
            printk("ESC key isn't registered\n");
	    return;
        }
	printk("--> send ESC key\n");
	input_report_key(kpadPower_dev, KEY_END, 1);
	input_sync(kpadPower_dev);
	input_report_key(kpadPower_dev, KEY_END, 0);
	input_sync(kpadPower_dev);
}

late_initcall(wmt_pm_init);
