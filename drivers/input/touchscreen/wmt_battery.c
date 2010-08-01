/*
 * linux/drivers/power/wmt_for_android_battery.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/wm97xx.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/wm97xx_batt.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <linux/delay.h>
#include "wm9715-api.h"



//#define DEBUG

#ifdef DEBUG
	#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
	#define dbg(format, arg...)
#endif

static int charg_stat = 1;
static int charg_gpio = 0;
static int low_alarm = 3;//SPI0SS--BIT3
static int batt_gpiostate = 0;
static int BAT_MAX = 1716;
static int BAT_MIN = 1392;
static int BAT_GPIO = 4;
static int bat_exist=1;
static int bat_lowalarmcnt = 0;

enum{
	AudioIC_WM9715,
	AudioIC_VT1613
};
static int audioic = 0;


#define BATTERY_DETECT_SEC	240
module_param(charg_stat, int, 0);
module_param(charg_gpio, int, 0);
module_param(BAT_MAX, int, 0);
module_param(BAT_MIN, int, 0);
module_param(BAT_GPIO, int, 0);

MODULE_PARM_DESC(charg_stat, "Charging stat.");



static struct timer_list polling_timer;

static DEFINE_MUTEX(bat_lock);
static struct work_struct bat_work;
struct mutex work_lock;
static int bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
static int bat_health = POWER_SUPPLY_HEALTH_GOOD;
static int bat_online = true;
static int bat_capacity = 50;
static int BASEVOLT = 3300;
static struct wm97xx_batt_info *pdata;

int polling_interval=5000;
extern int wm9712_poll_sample_battery (int *sample);
extern int wmt_getsyspara(char *varname, unsigned char *varval, int varlen);


#define WM8505
#ifdef WM8505
#define GPIO_ENA_REG	0xd8110064
#define GPIO_OUT_ENA_REG	0xd811008c
#define GPIO_OUT_DATA_REG	0xd81100b4
#define GPIO_IN_DATA_REG	0xd81100dc

#define GPIO_ENA_SPI 0xd811005c
#define GPIO_OUT_ENA_SPI 0xd8110084
#define GPIO_SPI_DATA_REG 0xd81100d4


void gpio_outlow(int gpio)
{
	writel(readl(GPIO_ENA_REG)|(1<<gpio),GPIO_ENA_REG);

	writel(readl(GPIO_OUT_ENA_REG)|(1<<gpio),GPIO_OUT_ENA_REG);

	writel(readl(GPIO_OUT_DATA_REG)&(~(1<<gpio)),GPIO_OUT_DATA_REG);
}

int gpio_in_get(int gpio)
{
	writel(readl(GPIO_ENA_REG)|(1<<gpio),GPIO_ENA_REG);
	writel(readl(GPIO_OUT_ENA_REG)&(~(1<<gpio)),GPIO_OUT_ENA_REG);
	return readl(GPIO_IN_DATA_REG)&(1<<gpio);
}

void gpio_in_set(int gpio)
{
	writel(readl(GPIO_ENA_REG)|(1<<gpio),GPIO_ENA_REG);
	writel(readl(GPIO_OUT_ENA_REG)&(~(1<<gpio)),GPIO_OUT_ENA_REG);
}

int gpio_lowalarm_get(int gpio)
{
	writel(readl(GPIO_ENA_SPI)|(1<<gpio),GPIO_ENA_SPI);
	writel(readl(GPIO_OUT_ENA_SPI)&(~(1<<gpio)),GPIO_OUT_ENA_SPI);
	return readl(GPIO_SPI_DATA_REG)&(1<<gpio);
}


#endif

#define BATT_DISCHARGE_SAM 11
static signed long slIndexTable[BATT_DISCHARGE_SAM*2+1];
signed long  * fBatPara;


signed long fBatParaLocal[]={
    100,90,80,70,60,
    50,40,30,20,10,
    0,-1
};

#define  CurSample_Volt(x)     (*(signed long *)(fBatPara + x))
#define  CurSample_Percent(x)  (*(signed long *)(fBatPara + x + 1))
#define  NextSample_Volt(x)    (*(signed long *)(fBatPara + x + 2))
#define  NextSample_Percent(x) (*(signed long *)(fBatPara + x + 3))

signed long wm85xx_getleftcapacity(signed long dwVolt)
{
    char Index = 0;
    signed long dwRet = 0;
    signed long k;
    signed long b;
    char  ElementNumber = 0;

	for(; CurSample_Volt(ElementNumber) != -1; ElementNumber ++);
	    for(;Index < ElementNumber;Index=Index+2)
	    {
	        if (dwVolt < CurSample_Volt(0))
				return dwRet;
		if (dwVolt > CurSample_Volt(ElementNumber - 2))
		{
			dwRet = 100;
			return dwRet;
		}
		if (dwVolt == CurSample_Volt(Index))
			return (100 - CurSample_Percent(Index));
	        else if ((dwVolt < NextSample_Volt(Index)) && (dwVolt > CurSample_Volt(Index)))
			break;
	    }
   	// k = (y2-y1)/(x2-x1)
	k = (NextSample_Volt(Index) - CurSample_Volt(Index))/
		(NextSample_Percent(Index) - CurSample_Percent(Index));

	// b = (y1*x2 - y2*x1)/(x2 - x1)
	b = (CurSample_Volt(Index)  * NextSample_Percent(Index) -
         NextSample_Volt(Index) * CurSample_Percent(Index))/
		(NextSample_Percent(Index) - CurSample_Percent(Index));

	// x = (y-b)/k
    	dwRet = ((signed long)dwVolt - b)/k;

    	dwRet = 100 - dwRet;
	return dwRet;
}


unsigned long wmt_simple_strtoul(const char *cp,char **endp,unsigned int base)
{
	unsigned long result = 0,value;

	if (*cp == '0') {
		cp++;
	}
	if (!base) {
		base = 10;
	}

    while ((*cp)!= ',')
 	{
        if(*cp>='0'&&*cp<='9') value = *cp - '0';
        else if(*cp>='a'&&*cp<='f') value = *cp - 'a' + 10;
        else if(*cp>='A'&&*cp<='F') value = *cp - 'A' + 10;
        else break;
        if (value >= base) break;

 		result = result*base + value;
 		cp++;
 	}

	if (endp)
		*endp = (char *)cp;
	return result;
}


bool wm85xx_getenv2int(char *varname, unsigned char *varval, int varlen, int *pInt)
{
    wmt_getsyspara(varname,varval,varlen);
    if (!varval)
    {
        return false;
    }

    *pInt = wmt_simple_strtoul(varval, NULL, 0);
    return true;
}


bool wm85xx_parse_voltlist(char *param)
{

	char * p = param;
	char * endp;
	int i=0;
	int j,k;
	int dwVolt[BATT_DISCHARGE_SAM];
	if(!param)
		return false;
	while(i<=BATT_DISCHARGE_SAM){
		dwVolt[i]=wmt_simple_strtoul(p, &endp, 10);
        	//printk("dwVolt[%d]  = %d\r\n ",i, dwVolt[i]);
		i++;
		if(*endp == '\0')
			break;
		p=endp+1;
		if(*p == '\0')
			break;
	}
	for(k=0,j=0;k<BATT_DISCHARGE_SAM*2;k=k+2,j++){
    		slIndexTable[k]= dwVolt[j];
	}
	for(k=0,j=0;k<(BATT_DISCHARGE_SAM)*2;k=k+2,j++){
	    	slIndexTable[k+1]= fBatParaLocal[j];
	}
	slIndexTable[BATT_DISCHARGE_SAM*2]=fBatParaLocal[j];
	fBatPara = slIndexTable;
	return true;
}




static unsigned long wm85xx_read_bat(struct power_supply *bat_ps)
{
	return 4000;
	/*
	return wm85xx_read_aux_adc(bat_ps->dev->parent->driver_data,
					pdata->batt_aux) * pdata->batt_mult /
					pdata->batt_div;
	*/
}

static unsigned long wm85xx_read_temp(struct power_supply *bat_ps)
{
	return 50;
	/*
	return wm85xx_read_aux_adc(bat_ps->dev->parent->driver_data,
					pdata->temp_aux) * pdata->temp_mult /
					pdata->temp_div;
	*/
}

static unsigned long wm85xx_read_status(struct power_supply *bat_ps)
{

	int status;
	int charging;
	int chargstatus;
	charging = gpio_in_get(charg_gpio);
	chargstatus = gpio_in_get(charg_stat);
	if(batt_gpiostate & BIT0){
		if(charging & BIT0){
			status = POWER_SUPPLY_STATUS_CHARGING;
			dbg("1 charging...\n");
		}
		else{
			status = POWER_SUPPLY_STATUS_DISCHARGING;
			dbg("1 battery...\n");
		}
	}
	else{
		if(!(charging & BIT0)){
			status = POWER_SUPPLY_STATUS_CHARGING;
			dbg("2 charging...\n ");
		}
		else{
			status = POWER_SUPPLY_STATUS_DISCHARGING;
			dbg("2 battery...");
		}
	}

	if(batt_gpiostate & BIT1){
		if(status == POWER_SUPPLY_STATUS_CHARGING)
			if(chargstatus & BIT1){
			status = POWER_SUPPLY_STATUS_FULL;
			dbg("3 charge full....\n");
			}
	}
	else{
		if(status == POWER_SUPPLY_STATUS_CHARGING)
			if(!(chargstatus & BIT1)){
			status = POWER_SUPPLY_STATUS_FULL;
			dbg("4 charge full...\n");
			}
	}
	return status;
}

static unsigned long wm85xx_read_health(struct power_supply *bat_ps)
{
	/*
	enum {
	        POWER_SUPPLY_HEALTH_UNKNOWN = 0,
	        POWER_SUPPLY_HEALTH_GOOD,
	        POWER_SUPPLY_HEALTH_OVERHEAT,
	        POWER_SUPPLY_HEALTH_DEAD,
	        POWER_SUPPLY_HEALTH_OVERVOLTAGE,
	        POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
	        POWER_SUPPLY_HEALTH_COLD,
	};
	*/
	return POWER_SUPPLY_HEALTH_GOOD;
}

static unsigned long wm85xx_read_online(struct power_supply *bat_ps)
{

	return true;
}


static unsigned long wm85xx_read_capacity(struct power_supply *bat_ps)
{
	int capacity_default=50;
	if(POWER_SUPPLY_STATUS_DISCHARGING == bat_status)
	{
		//kevin modify ,dont delete static , it used by 1613
		static int capacity=50;
		if(audioic == AudioIC_WM9715){
			int ret;
			int sample;
			ret = wm9712_poll_sample_battery(&sample);
			sample = 2*(sample)*3*BASEVOLT/0xFFF;
			dbg ("adc  sample, got %d\n", sample);
			capacity = wm85xx_getleftcapacity(sample);
			//power off system for rtc running etc.
			//<1% poweroff system
			//<15% user hint
			if(capacity < 1)
				capacity = 0;
			//printk("capacity = %d ",capacity);
			//return capacity;
		}
		else if(audioic == AudioIC_VT1613){

			static int num=0;
			int bat_min=1;
			int bat_max=(BAT_MAX-BAT_MIN);
			int lowalarm;		
			int sample;
			int res;
			volatile int i;

			//low alarm to power off sys
			lowalarm = gpio_lowalarm_get(low_alarm);			
			//printk("lowalarm = %d\n", lowalarm);				
			if(!(lowalarm & BIT3)){//SPI0SS
				bat_lowalarmcnt++;
				if(bat_lowalarmcnt>7){//for fake gpio status
					capacity = 0;					
					/**********************************
					capacity = 5;
					signal(SIGALRM, poweroff);//bat_capacity = 0;
					struct itimerval tick;
					memset(&tick, 0, sizeof(tick));
					//timeout to run first
					tick.it_value.tv_sec = 30;//sec
					tick.it_value.tv_usec = 0;//micro sec
					//to run interval
					tick.it_interval.tv_sec = 30;//
					tick.it_interval.tv_usec = 0;//
					
					res = setitimer(ITIMER_REAL, &tick, NULL);
					if(res)
						printk("set timer failed!\n");
					***********************************/
				}
				else if(bat_lowalarmcnt>2){
					capacity = 5;//inaccurate fake adc no alarm.
				}
				return capacity;
			}
			else{
				bat_lowalarmcnt = 0;//fake info,reset
			}
				
			num++;
			if(!bat_exist){
				printk("bat_exist = %d\n",bat_exist);
				return capacity;
			}

			//detect the first 10 time, and detect every BATTERY_DETECT_SEC
			//printk("num %d  %d  %d\n",num,BATTERY_DETECT_SEC*polling_interval/1000,
			//num%(BATTERY_DETECT_SEC*polling_interval/1000)!=1);
			if(charg_stat!=2){
				if(num>10&&(num%(BATTERY_DETECT_SEC*(polling_interval/10)/1000)!=1))
					return capacity;
			}

			gpio_outlow(BAT_GPIO);//gpio out_low for c d-charge
			msleep(1000);
                        
			for(i=0;i<1000;i++){
				if(gpio_in_get(BAT_GPIO))
					break;
				udelay(1000);
			}

			if(i==1000){
				//bat is not exist,set to full
				capacity = 100;
				bat_exist = 0;
				return capacity;
			}
			if(charg_stat==2)
				printk("bat cnt test value :%d\n",i);

			sample = BAT_MAX-i;
			if(sample<bat_min)
				sample = bat_min;
			capacity = (sample-bat_min)*100/(bat_max-bat_min);

			//quantify fake adc info
			if(capacity>69)
				capacity = 70;//high_batt_volt
			else if(capacity>29)
				capacity = 50;//normal_batt_volt
			else if(capacity>14)
				capacity = 30;//low_batt_volt	
			else if(capacity>4)
				capacity = 15;//alarm_batt_volt
			else if(capacity<5)
				capacity = 5;//no need to power off here randolph
			else if(capacity>100)
				capacity=100;
		}
		return capacity;
	}
	else if(POWER_SUPPLY_STATUS_FULL == bat_status){
		bat_lowalarmcnt = 0;
		return 100;
	}
	else if(POWER_SUPPLY_STATUS_CHARGING == bat_status){
		bat_lowalarmcnt = 0;
		return capacity_default;//50%
	}
	else
		return capacity_default;
}


static int wm85xx_bat_get_property(struct power_supply *bat_ps,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bat_health;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bat_online;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->batt_tech;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (pdata->batt_aux >= 0)
			val->intval = wm85xx_read_bat(bat_ps);
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (pdata->temp_aux >= 0)
			val->intval = wm85xx_read_temp(bat_ps);
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (pdata->max_voltage >= 0)
			val->intval = pdata->max_voltage;
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		if (pdata->min_voltage >= 0)
			val->intval = pdata->min_voltage;
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
   		val->intval = 3;
   		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
   		val->intval = 3;
   		break;
	default:
		//printk("%s %d\n",__FUNCTION__,__LINE__);
		return -EINVAL;
	}
	return 0;
}

static void wm85xx_bat_external_power_changed(struct power_supply *bat_ps)
{
	printk("wm85xx_bat_external_power_changed.............\n");
	schedule_work(&bat_work);
}

#define CAPACITY_NUMER	100
int capacity_array[CAPACITY_NUMER];
int capacity_index=0;

static void wm85xx_bat_update(struct power_supply *bat_ps)
{
	int old_status = bat_status;
	int old_capacity = bat_capacity;
	mutex_lock(&work_lock);

	bat_status = wm85xx_read_status(bat_ps);
	bat_health = wm85xx_read_health(bat_ps);
	bat_online = wm85xx_read_online(bat_ps);

	bat_capacity = wm85xx_read_capacity(bat_ps);

	//power_supply_changed(bat_ps);

	if (old_status != bat_status||old_capacity!=bat_capacity) {
		//printk("%s: %i -> %i\n", bat_ps->name, old_status, bat_status);
		//printk("%s: %i -> %i\n", bat_ps->name, old_capacity, bat_capacity);
		power_supply_changed(bat_ps);
	}
	mutex_unlock(&work_lock);
}


static struct power_supply bat_ps = {
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= wm85xx_bat_get_property,
	.external_power_changed = wm85xx_bat_external_power_changed,
	.use_for_apm		= 1,
};

static void wm85xx_bat_work(struct work_struct *work)
{
	wm85xx_bat_update(&bat_ps);
}

#ifdef CONFIG_PM
extern int vt1613_init(void);
extern void vt1613_exit(void);
static int wm85xx_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	flush_scheduled_work();
	vt1613_exit();
	return 0;
}

static int wm85xx_bat_resume(struct platform_device *dev)
{
       vt1613_init();
	schedule_work(&bat_work);
	return 0;
}
#else
#define wm85xx_bat_suspend NULL
#define wm85xx_bat_resume NULL
#endif


static void polling_timer_func(unsigned long unused)
{
//	printk("polling...\n");

	schedule_work(&bat_work);
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(polling_interval));

}

static enum power_supply_property prop[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CAPACITY,

};

static int __devinit wm85xx_bat_probe(struct platform_device *dev)
{
	int ret = 0;
	//int i = 0;


	pdata = dev->dev.platform_data;
	if (dev->id != -1)
		return -EINVAL;

	mutex_init(&work_lock);

	if (!pdata) {
		dev_err(&dev->dev, "Please use wm85xx_bat_set_pdata\n");
		return -EINVAL;
	}

	INIT_WORK(&bat_work, wm85xx_bat_work);

	if (!pdata->batt_name) {
		dev_info(&dev->dev, "Please consider setting proper battery "
				"name in platform definition file, falling "
				"back to name \"wm85xx-batt\"\n");
		bat_ps.name = "wm85xx-batt";
	} else
		bat_ps.name = pdata->batt_name;

	bat_ps.properties = prop;
	bat_ps.num_properties = ARRAY_SIZE(prop);

	ret = power_supply_register(&dev->dev, &bat_ps);
	if (!ret)
		schedule_work(&bat_work);
	else
		goto err;

	setup_timer(&polling_timer, polling_timer_func, 0);
	mod_timer(&polling_timer,
			  jiffies + msecs_to_jiffies(polling_interval));

	return 0;
err:
	return ret;
}

static int __devexit wm85xx_bat_remove(struct platform_device *dev)
{

	flush_scheduled_work();
	del_timer_sync(&polling_timer);

	power_supply_unregister(&bat_ps);
	kfree(prop);
	return 0;
}
static struct platform_driver wm85xx_bat_driver = {
	.driver	= {
		.name	= "wmt-battery",
		.owner	= THIS_MODULE,
	},
	.probe		= wm85xx_bat_probe,
	.remove		= __devexit_p(wm85xx_bat_remove),
	.suspend	= wm85xx_bat_suspend,
	.resume		= wm85xx_bat_resume,
};

static int __init wm85xx_bat_init(void)
{
    unsigned char retval[128];
    wmt_getsyspara("battvoltlist", retval, 128);
    wm85xx_parse_voltlist(retval);
    wm85xx_getenv2int("gpiostate",retval,32,(int*)&batt_gpiostate);
	dbg("batt_gpiostate = %d\n",batt_gpiostate);
    wmt_getsyspara("audioic",retval, 32);
    if(!strcmp(retval,"wm9715"))
	audioic = AudioIC_WM9715;
    else if(!strcmp(retval,"vt1613")){
	audioic = AudioIC_VT1613;
	wm85xx_getenv2int("battmax",retval,32,(int*)&BAT_MAX);
	wm85xx_getenv2int("battmin",retval,32,(int*)&BAT_MIN);
	wm85xx_getenv2int("battgpio",retval,32,(int*)&BAT_GPIO);
	
	printk("BAT_MAX = %d\n",BAT_MAX);
	printk("BAT_MIN = %d\n",BAT_MIN);
	printk("BAT_GPIO = %d\n",BAT_GPIO);
	

	wmt_getsyspara("chargetest", retval, 32);
	if(retval)
		if(!strcmp(retval,"true")){
			printk("begin test vt1613 fakeadc\n!");
			charg_stat = 2;
		}
		else
			printk("vt1613 fakeadc passthrough\n!");
    }
    printk("audioic = %d\n",audioic);
    wm85xx_getenv2int("basevolt",retval,32,(int*)&BASEVOLT);
    printk("BASEVOLT = %d\n",BASEVOLT);
    wm9715_gpio_config();

    return platform_driver_register(&wm85xx_bat_driver);
}

static void __exit wm85xx_bat_exit(void)
{
	platform_driver_unregister(&wm85xx_bat_driver);
}
module_init(wm85xx_bat_init);
module_exit(wm85xx_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Vasut <marek.vasut@gmail.com>");
MODULE_DESCRIPTION("WM97xx battery driver");
