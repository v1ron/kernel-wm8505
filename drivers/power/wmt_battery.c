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

static struct timer_list polling_timer;

static DEFINE_MUTEX(bat_lock);
static struct work_struct bat_work;
struct mutex work_lock;
static int bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
static int bat_health = POWER_SUPPLY_HEALTH_GOOD;
static int bat_online = true;
static int bat_capacity = 30;
static struct wm97xx_batt_info *pdata;

int polling_interval=500;
extern int wm9712_poll_sample_battery (int *sample);
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
/*
enum {
        POWER_SUPPLY_STATUS_UNKNOWN = 0,
        POWER_SUPPLY_STATUS_CHARGING,
        POWER_SUPPLY_STATUS_DISCHARGING,
        POWER_SUPPLY_STATUS_NOT_CHARGING,
        POWER_SUPPLY_STATUS_FULL,
};
*/
	int status;
	int chargering;
	#define GPIO_BASE_ADDR                          0xD8110000  /* 64K  */
	
	chargering = REG32_VAL(GPIO_BASE_ADDR+0xdc)&0x2;
	if(chargering)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	//if(bat_capacity >95)
	//	status = POWER_SUPPLY_STATUS_FULL;
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
#define BAT_MIN		0x600
#define BAT_MAX		0x785
	int capacity=0;
	int ret;
	int sample;	
	ret = wm9712_poll_sample_battery(&sample);
	if(ret == RC_VALID)
		capacity = (sample-BAT_MIN)*100/(BAT_MAX-BAT_MIN);
	//printk("0x%x ",sample);
	return capacity;
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
			//printk("%s %d psp %d\n",__FUNCTION__,val->intval,psp);
	return 0;
}

static void wm85xx_bat_external_power_changed(struct power_supply *bat_ps)
{
	printk("wm85xx_bat_external_power_changed======================\n");

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
//	printk("wm85xx_bat_update======================\n");
	/*
	bat_status = (pdata->charge_gpio >= 0) ?
			(gpio_get_value(pdata->charge_gpio) ?
			POWER_SUPPLY_STATUS_DISCHARGING :
			POWER_SUPPLY_STATUS_CHARGING) :
			POWER_SUPPLY_STATUS_UNKNOWN;
	*/
	bat_status = wm85xx_read_status(bat_ps);
	bat_health = wm85xx_read_health(bat_ps);
	bat_online = wm85xx_read_online(bat_ps);

	capacity_array[capacity_index++]= wm85xx_read_capacity(bat_ps);
	if(capacity_index==CAPACITY_NUMER){
		int i,tmp_capacity=0;
		for(i=0;i<CAPACITY_NUMER;i++)
			tmp_capacity+=capacity_array[i];
		bat_capacity = tmp_capacity/CAPACITY_NUMER;
		capacity_index = 0;
		printk("\nbat_capacity %d/100\n",bat_capacity);
	}
	
//	printk("status %d %d\n",old_status,bat_status);
	if (old_status != bat_status||old_capacity!=bat_capacity) {
		printk("%s: %i -> %i\n", bat_ps->name, old_status,
					bat_status);
		printk("%s: %i -> %i\n", bat_ps->name, old_capacity,
					bat_capacity);		
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
//	printk("wm85xx_bat_work\n");

	wm85xx_bat_update(&bat_ps);
}

#ifdef CONFIG_PM
static int wm85xx_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	flush_scheduled_work();
	return 0;
}

static int wm85xx_bat_resume(struct platform_device *dev)
{
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
	int i = 0;

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
