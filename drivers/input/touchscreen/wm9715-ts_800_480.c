/* PVCS version log
 *
 * 7     07/09/07 5:10p Paulkwong
 *
 * 6     07/09/07 11:21a Paulkwong
 * Full ioctl function
 *
 * 5     07/08/14 3:53p Paulkwong
 * Add polling function
 *
 * 4     07/07/02 6:11p Paulkwong
 * Verify with unit test
 *
 * 3     07/06/25 11:43a Paulkwong
 *
 * 2     07/03/05 10:32a Paulkwong
 *
 * 1     07/02/01 12:09p Paulkwong
 *
 */

#include <linux/unistd.h>
#include <linux/time.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
//#include <asm/semaphore.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <mach/hardware.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include "wm9715-ts.h"
#include "wm9715-api.h"

#define WM_DEBOUNCE_TIME 150
//#define WMT_MULTITOUCH_SUPPORT 1 // Whether support multi-touch screen
#define WMT_FILTER_POINTS_ALGORITHM 0

#define DRIVER_NAME	 "wm9715_touch"
#define F1_REGION_MIN_X (panel.res_x-40)
#define F1_REGION_MAX_Y (40)

//#define DEBUG
#ifdef DEBUG
	#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
	#define dbg(format, arg...)
#endif

#define ABS_XY(a) (((a)>=0)?(a):(-(a)))


// static int abs_x[3] = {350,3900,5};
// module_param_array(abs_x, int, NULL, 0);
// MODULE_PARM_DESC(abs_x, "Touchscreen absolute X min, max, fuzz");

// static int abs_y[3] = {320,3750,40};
// module_param_array(abs_y, int, NULL, 0);
// MODULE_PARM_DESC(abs_y, "Touchscreen absolute Y min, max, fuzz");
//
// static int abs_p[3] = {0,150,4};
// module_param_array(abs_p, int, NULL, 0);
// MODULE_PARM_DESC(abs_p, "Touchscreen absolute Pressure min, max, fuzz");


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
static unsigned long lastpoint_timestamp;
static int delay = 1;
module_param(delay, int, 0);
MODULE_PARM_DESC(delay, "Set adc sample delay.");

//static int ts_delay = 55; /* us */
// module_param(ts_delay, int, 0444);
// MODULE_PARM_DESC(ts_delay, "Delay between panel setup and position read. Default = 55us.");


// static int ts_delay_pressure;	/* us */
// module_param(ts_delay_pressure, int, 0444);
// MODULE_PARM_DESC(ts_delay_pressure,
// 				 "delay between panel setup and pressure read.  Default = 0us.");



/* The structure used to return arch specific sampled data into */
struct wm97xx_data {
    int x;
    int y;
    int p;
    unsigned long timestamp;
};


/*
 * Map the mutex'es from IRIX to Linux semaphores.
 *
 * Destroy just simply initializes to -99 which should block all other
 * callers.
 */


struct input_dev *ts_input_dev;
struct pt_regs *ts_input_regs;

struct wm9715_s {
	struct input_dev* inputdev;
	struct task_struct	*ts_task;
    struct mutex	codec_mutex;
    struct completion ts_exit;

    int			        pen_irq;
    wait_queue_head_t   pen_irq_wait;	/* Pen IRQ wait queue */

    unsigned int		pen_is_down : 1;
    short				valid_point_cnt; // the number of valid points from pendown to up
};

typedef struct wm9715_s wm9715_t;

static  wm9715_t    * the_wm9715;

typedef struct {
	int res_x;
	int res_y;
	int raw_min_x;
	int raw_max_x;
	int x_rev;
	int raw_min_y;
	int raw_max_y;
	int y_rev;
	int xyswap;
	int xy_inv;
} ts_panel_info_t;

static ts_panel_info_t panel =
{
	.res_x = 800,
	.res_y = 480,

    .raw_min_x = 183, //193,
    .raw_max_x  = 3669, //3925, //3940,
    .x_rev = 0, //


    .raw_min_y = 150, //168,
    .raw_max_y = 3925, //3725,

    .y_rev = 1, //

	.xyswap = 1,
	.xy_inv = 0,
};


typedef struct {
	short pressure;
	short x;
	short y;
	short millisecs;
} TS_EVENT;

#define TS_NAME                 "touchscreen"

#define BUFSIZE                 128

static TS_EVENT buf[BUFSIZE];
static int head, tail;


struct touch_device {
	struct mutex mlock;
	struct class* dev_class;
	struct device *device;
};

static struct touch_device l_vibratedev = {
	.dev_class = NULL,
	.device = NULL,
};

#define IRQ_GPIO4	4
#define IRQ_GPIO5	5
#define IRQ_GPIO6  	6
static int touch_phy_irqnum;
static int panelres_x;
static int panelres_y;
static DECLARE_WAIT_QUEUE_HEAD(queue);

static CALIBRATION_PARAMETER g_CalcParam;
volatile bool g_bCalibrating = true;
static TS_EVENT g_evLast;
extern int wmt_getsyspara(char *varname, unsigned char *varval, int varlen);

void TouchPanelCalibrateAPoint(
    int   UncalX,     //@PARM The uncalibrated X coordinate
    int   UncalY,     //@PARM The uncalibrated Y coordinate
    int   *pCalX,     //@PARM The calibrated X coordinate
    int   *pCalY      //@PARM The calibrated Y coordinate
    )
{
    int   x, y;

    x = (g_CalcParam.a1 * UncalX + g_CalcParam.b1 * UncalY +
         g_CalcParam.c1) / g_CalcParam.delta;
    y = (g_CalcParam.a2 * UncalX + g_CalcParam.b2 * UncalY +
         g_CalcParam.c2) / g_CalcParam.delta;
    if ( x < 0 ){
        x = 0;
    }

    if ( y < 0 ){
        y = 0;
    }

    *pCalX = x;
    *pCalY = y;
}




static unsigned long GetMillisecondTimestamp()
{

	unsigned long currentTime;
	struct timeval currenttemp;
	do_gettimeofday(&currenttemp);
	currentTime = currenttemp.tv_sec * 1000 + currenttemp.tv_usec/1000;
	return currentTime;
}
bool IsPenDebouncing()
{

    bool	debouncing = false;
    unsigned int    lastPointTime, delay;

    lastPointTime = lastpoint_timestamp;
    delay = GetMillisecondTimestamp() - lastPointTime;
    if (delay<(WM_DEBOUNCE_TIME + 1)  ){
            /* We're still in debounce. */
            debouncing = true;
            //printk("touch debouncing %d\r\n", delay);
        }
    return debouncing;
}


unsigned long wmt_strtoul(const char *cp,char **endp,unsigned int base)
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


bool wmt_getenv2int(char *varname, unsigned char *varval, int varlen, int *pInt)
{
    wmt_getsyspara(varname,varval,varlen);
    if (!varval)
    {
        return false;
    }

    *pInt = wmt_strtoul(varval, NULL, 0);
    return true;
}

static void ts_clear(void)
{
    mutex_lock(&the_wm9715->codec_mutex);

    head = tail = 0;
	memset(buf, 0, sizeof(buf));

    mutex_unlock(&the_wm9715->codec_mutex);
}


// Note: For some reseason, now adding data from head, and getting data from tail
// But the right way is contrary


static void ts_get_data(TS_EVENT* ts)
{
    int last;

    mutex_lock(&the_wm9715->codec_mutex);

    last = tail;
    *ts = buf[tail];
    tail = (tail+1)%BUFSIZE;

    dbg("ts_get_data:  take buf[%d]: x= %d, y= %d, p= %d, m= %d\n",
           last,buf[last].x,buf[last].y,buf[last].pressure,(unsigned short)buf[last].millisecs);

    mutex_unlock(&the_wm9715->codec_mutex);
}

static void ts_add_data(TS_EVENT* ts)
{
     int i;
     //short tmp;
     TS_EVENT t;// = *ts;
	static int count = 0;
	//int raw_min_x,raw_max_x;
	//int raw_min_y,raw_max_y;
	int cal_x, cal_y;


	t.millisecs = ts->millisecs;
	t.pressure = ts->pressure;
	t.x = ts->x;
	t.y = ts->y;

	if (g_bCalibrating)
  {
  	t.x = 10;
  	t.y = 10;
  }
	else
	{
		TouchPanelCalibrateAPoint(ts->x, ts->y, &cal_x, &cal_y);
		t.x = (short)cal_x;
		t.y = (short)cal_y;
	}

    mutex_lock(&the_wm9715->codec_mutex);

	// whether the queue is full, if full then clear the queue and add the lastest data
	if (((head+1)%BUFSIZE) == tail) // the queue is full
	{
		head=tail=0;
	}

  g_evLast = *ts;
	buf[head] = t;
	i = head;
	head = (head+1)%BUFSIZE; // add one data to queue

	//input_regs(the_wm9715->inputdev, ts_input_regs);
	if(buf[i].pressure==0){
		dbg("pen up........\n");
		input_report_key(the_wm9715->inputdev, BTN_TOUCH, 0);
		input_report_abs(the_wm9715->inputdev, ABS_PRESSURE, 0);
	}
	else{
		dbg("pen down........\n");
		input_report_abs(the_wm9715->inputdev, ABS_X, buf[i].x);
		input_report_abs(the_wm9715->inputdev, ABS_Y, buf[i].y);
		input_report_key(the_wm9715->inputdev, BTN_TOUCH, 1);
		input_report_abs(the_wm9715->inputdev, ABS_PRESSURE, 1);
		dbg("ts_add_data: x=%d, y=%d,p=%d\n", buf[i].x, buf[i].y, buf[i].pressure);
	}

	input_sync(the_wm9715->inputdev);

    dbg("  put buf[%d]: x= %d, y= %d, p= %d, m= %d\n",
           i,buf[i].x,buf[i].y,buf[i].pressure,buf[i].millisecs);

    mutex_unlock(&the_wm9715->codec_mutex);
}

static void ts_report_event(TS_EVENT * ts)
{
	static TS_EVENT last_data = { 0, 0, 0, 0};
	//int diff0, diff1, diff2, diff3;


	ts->millisecs = jiffies;
    //printk("ts->p=%d ts->x=%d ts->y=%d ts->m=%d\n", ts->pressure, ts->x, ts->y, ts->millisecs);
//	if(ts->pressure == 0 || (ts->pressure > 15/*100*/ && ts->pressure < 40/*250*/))
	{
		//last_data = *ts;
		last_data.millisecs = ts->millisecs;
		last_data.pressure = ts->pressure;
		last_data.x = ts->x;
		last_data.y = ts->y;
		ts_add_data(ts);
	}
	wake_up_interruptible(&queue);
}


/*
 * Sample the WM9712 touchscreen in polling mode
 */
static int wm9712_poll_touch(struct wm97xx_data *data)
{
	int rc;
	int repeat = 1; //3; // the count of repeat sample
	int sampoints = 1;
	//int diffxlim = 15; // the up limit of difference between the sample of adjacent x
	//int diffylim = 15; // // the up limit of difference between the sample of adjacent y
	int i,j;
	//int k;
	//int index1,index2;
	struct wm97xx_data smdata[3][3];
	int diff[3][2]; // (diffx,diffy)
	#if WMT_FILTER_POINTS_ALGORITHM
	repeat = 1;
	sampoints = 3;
	#endif

	memset(smdata, 0 , sizeof(smdata));
	memset(diff, 0, sizeof(diff));
	for (i = 0; i < repeat; i++)
	{
		for (j = 0; j < sampoints /*3*/; j++)
		{
			// sample
			if ((rc = wm9712_poll_sample(WM97XX_ADCSEL_X, &smdata[i][j].x)) != RC_VALID)
			{
				dbg("Error(0x%08x) when sample X !!!\n", rc);
				break;
			}

			if ((rc = wm9712_poll_sample(WM97XX_ADCSEL_Y, &smdata[i][j].y)) != RC_VALID)
			{
				dbg("Error when sample y !!!\n");
				break;
			}

			if ((rc = wm9712_poll_sample(WM97XX_ADCSEL_PRES, &smdata[i][j].p)) != RC_VALID)
			{
				dbg("Error when sample pressusre !!!\n");
				break;
			}
		};
		if (rc != RC_VALID)
		{
			dbg("Errors when sample !!!!\n");
			#if WMT_FILTER_POINTS_ALGORITHM
			// more than two points, the lastest point is valid
			if (((i-0)*3+j) > 1)
			{
				if (j != 0)
				{
					index1 = i;
					index2 = j - 1;
				} else
				{
					index1 = i - 1;
					index2 = 2;
				}
				data->x = smdata[index1][index2].x;
				data->y = smdata[index1][index2].y;
				data->p = smdata[index1][index2].p + 5;
			}
			#endif
			return rc; // now is RC_PENUP
		} else
		{
			#if WMT_FILTER_POINTS_ALGORITHM
			// whether is the right coordiante pair
			diff[0][0] = ABS_XY(smdata[i][0].x-smdata[i][1].x);
			diff[0][1] = ABS_XY(smdata[i][0].y-smdata[i][1].y);
			diff[1][0] = ABS_XY(smdata[i][1].x-smdata[i][2].x);
			diff[1][1] = ABS_XY(smdata[i][1].y-smdata[i][2].y);
			diff[2][0] = ABS_XY(smdata[i][2].x-smdata[i][0].x);
			diff[2][1] = ABS_XY(smdata[i][2].y-smdata[i][0].y);
			//dbg("xy (%d,%d),(%d,%d),(%d,%d)\n", smdata[i][0].x, smdata[i][0].y,
			//	smdata[i][1].x, smdata[i][1].y,
			//	smdata[i][2].x, smdata[i][2].y);
			//dbg("diff (%d,%d),(%d,%d),(%d,%d)\n",
			//	diff[0][0],diff[0][1],diff[1][0],diff[1][1],diff[2][0],diff[2][1]);
			 //sure the nearst two point and average
			for (k = 0; k < 3; k++)
			{
				if ((diff[k][0] <= diffxlim)
					&& (diff[k][1] <= diffylim))
				{
					data->x = (smdata[i][k].x + smdata[i][(k+1)%3].x)/2;
					data->y = (smdata[i][k].y + smdata[i][(k+1)%3].y)/2;
					data->p = smdata[i][k].p;
					//dbg("right location k=%d\n", k);
					return RC_VALID;
				}
			};
			#else
			// directly report the sampled points
			data->x = smdata[0][0].x;
			data->y = smdata[0][0].y;
			data->p = smdata[0][0].p;
			lastpoint_timestamp = data->timestamp = GetMillisecondTimestamp();
			return RC_VALID;
			#endif
		}

	};
	// failed to location (x,y), return the latest point
	//dbg("default location (x,y).\n");
	data->x = smdata[i-1][2].x;
	data->y = smdata[i-1][2].y;
	data->p = smdata[i-1][2].p;

	return RC_VALID; //
}

/* Private struct for communication between struct wm97xx_tshread
 * and wm97xx_read_samples */
struct ts_state {
	int sleep_time;
	int min_sleep_time;
};


static int wm97xx_read_samples(struct ts_state *state)
{
	struct wm97xx_data data;
	int rc;
	TS_EVENT ts;
	static TS_EVENT last_data;

	memset(&data, 0, sizeof(struct wm97xx_data));
	rc = wm9712_poll_touch(&data);

	if (rc & RC_PENUP)
		{
		if(!IsPenDebouncing()){
		if (the_wm9715->pen_is_down!=0){
			the_wm9715->pen_is_down = 0;
			dbg("pen up\n");

			if (the_wm9715->valid_point_cnt > 0)
			{
				//ts.x = data.x & 0xfff;
				//ts.y = data.y & 0xfff;
				ts.x=last_data.x;
				ts.y=last_data.y;
				ts.pressure = 0;
				dbg("a) pen up report : x=%d, y=%d, pressure=%d\n", ts.x, ts.y, ts.pressure);
				ts_report_event(&ts);
			} else
			{
				if (data.p != 0)
				{
					ts.x = data.x & 0xfff;
					ts.y = data.y & 0xfff;
					ts.pressure = data.p & 0xfff;
					dbg("b) pen up report : x=%d, y=%d, pressure=%d\n", ts.x, ts.y, ts.pressure);
					ts_report_event(&ts); // down
					ts.pressure = 0;
					ts_report_event(&ts); // up
				}
			}
			state->sleep_time = HZ / 10;
		} else if (!(rc & RC_AGAIN)) {
			/* We need high frequency updates only while pen is down,
			* the user never will be able to touch screen faster than
			* a few times per second... On the other hand, when the
			* user is actively working with the touchscreen we don't
			* want to lose the quick response. So we will slowly
			* increase sleep time after the pen is up and quicky
			* restore it to ~one task switch when pen is down again.
			*/
			dbg("(rc & RC_AGAIN)...in wm97xx_read_samples\n");
			if (state->sleep_time < HZ / 10)
				state->sleep_time = HZ / 10;
			}
		}
		else
			return rc;
	}
	else if (rc & RC_VALID)	{
		the_wm9715->valid_point_cnt++;
		ts.x = data.x & 0xfff;
		ts.y = data.y & 0xfff;
		last_data.x=ts.x;
		last_data.y=ts.y;
		ts.pressure = data.p & 0xfff;

        dbg("c) pen down report : x=%d, y=%d, pressure=%d\n", ts.x, ts.y, ts.pressure);
        ts_report_event(&ts);


		the_wm9715->pen_is_down = 1;
		state->sleep_time = state->min_sleep_time;
	} else if (rc & RC_PENDOWN) {
		dbg("pen down");
		the_wm9715->pen_is_down = 1;
		state->sleep_time = state->min_sleep_time;
	} else // error when sampling
	{
		if (the_wm9715->valid_point_cnt > 0)
		{
			ts.x=last_data.x;
			ts.y=last_data.y;
			ts.pressure = 0;
			dbg("d) pen up report : x=%d, y=%d, pressure=%d\n", ts.x, ts.y, ts.pressure);
			ts_report_event(&ts);
		}
	}

	return rc;
}

int touch_sleep_time = 20;

static int wm9715_ts_thread(void * data)
{
	int rc;
	struct ts_state state;
	wm9715_t *wm = (wm9715_t *) data;

	/* set up thread context */
	wm->ts_task = current;
//	daemonize("kwm97xxts");
	touch_sleep_time = 2;
//	complete(&the_wm9715->ts_init);
	wm->pen_is_down = 0;
	state.min_sleep_time = HZ >= 100 ? HZ / 100 : 1;
	//if (state.min_sleep_time < 1)
	//	state.min_sleep_time = 1;
	state.sleep_time = state.min_sleep_time;


	/* touch reader loop */
	while (wm->ts_task) {
		do {
//			try_to_freeze();
			rc = wm97xx_read_samples(&state);

			if (rc & RC_AGAIN)
			{
				dbg("int ts_thread, try again.\n");
				msleep(touch_sleep_time);
			}
		} while (rc & RC_AGAIN);

		//if (!wm->pen_is_down && wm->pen_irq)
		if ((!wm->pen_is_down && wm->pen_irq) ||
			(rc == RC_PENUP) ||
			(rc == RC_ERROR) ||
			(rc == RC_TIMEOUT))
		{
			if(!IsPenDebouncing())
			{
				wm->pen_is_down = 0;
				dbg("waiting for next interrupt...\r\n \r\n \r\n");
				/* Nice, we don't have to poll for pen down event */
				wait_event_interruptible(wm->pen_irq_wait, wm->pen_is_down);
			}
			else
			{
				dbg("preparing for sampling continuous...\n");
				set_task_state(current, TASK_INTERRUPTIBLE);
				schedule_timeout(state.sleep_time);
			}
		} else {
			dbg("preparing for sampling continuous...\n");
			set_task_state(current, TASK_INTERRUPTIBLE);
			schedule_timeout(state.sleep_time);
		}
	}
	complete_and_exit(&the_wm9715->ts_exit, 0);
	return 0;
}



irqreturn_t wm9715_hard_irq(int irqnr, void *devid/*, struct pt_regs * regs*/)
{
	//int pmw_status;
	wm9715_t *wm = devid;
	if(touch_phy_irqnum == IRQ_GPIO5)
		REG32_VAL(0xd8110000 + 0x304) |= BIT5; //clear interrupt
	else if(touch_phy_irqnum == IRQ_GPIO4)
		REG32_VAL(0xd8110000 + 0x304) |= BIT4; //clear interrupt
	else if(touch_phy_irqnum == IRQ_GPIO6)
		REG32_VAL(0xd8110000 + 0x304) |= BIT6; //clear interrupt
	dbg("touch screen INT(AC97_INT)....\n");
	if (wm->pen_is_down == 0) // sometime one pendown occurs two interrupt
	{
		wm->pen_is_down = 1;
		wm->valid_point_cnt = 0;

	}
	wake_up_interruptible(&wm->pen_irq_wait);
	return IRQ_HANDLED;
}

static int wm9715_ts_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	printk("wm9715 ts driver v1.3 opening...\n");

	ts_clear();
	try_module_get(THIS_MODULE);

    return ret;
}

static int wm9715_ts_close(struct inode *inode, struct file *filp)
{
	ts_clear();
    module_put(THIS_MODULE);

	return 0;
}

static int wm9715_ts_ioctl(struct inode * node, struct device *dev, unsigned int cmd, unsigned long arg)
{
	int nBuff[7];

	//printk("wm9715_ts_ioctl(node=0x%08x, dev=0x%08x, cmd=0x%08x, arg=0x%08x)\n", node, dev, cmd, arg);

	if (_IOC_TYPE(cmd) != TS_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > TS_IOC_MAXNR) return -ENOTTY;

	switch (cmd) {
	case TS_IOCTL_CAL_START:
		printk("wm9715_ts_ioctl: TS_IOCTL_CAL_START\n");
		g_bCalibrating = true;
		return 0;
	case TS_IOCTL_CAL_DONE:
		printk("wm9715_ts_ioctl: TS_IOCTL_CAL_DONE\n");
		copy_from_user(nBuff, (unsigned int*)arg, 7*sizeof(int));
		g_CalcParam.a1 = nBuff[0];
		g_CalcParam.b1 = nBuff[1];
		g_CalcParam.c1 = nBuff[2];
		g_CalcParam.a2 = nBuff[3];
		g_CalcParam.b2 = nBuff[4];
		g_CalcParam.c2 = nBuff[5];
		g_CalcParam.delta = nBuff[6];
		printk("g_CalcParam = %d, %d, %d, %d, %d, %d, %d\n",
			g_CalcParam.a1, g_CalcParam.b1, g_CalcParam.c1,
			g_CalcParam.a2, g_CalcParam.b2, g_CalcParam.c2, g_CalcParam.delta);
		g_bCalibrating = false;
		return 0;
	case TS_IOCTL_GET_RAWDATA:
		printk("wm9715_ts_ioctl: TS_IOCTL_GET_RAWDATA\n");
		if (!g_bCalibrating) return -EINVAL;
		nBuff[0] = g_evLast.x;
		nBuff[1] = g_evLast.y;
		copy_to_user((unsigned int*)arg, nBuff, 2*sizeof(int));
		printk("raw data: x=%d, y=%d\n", nBuff[0], nBuff[1]);
		return 0;
	}
	return -EINVAL;
}

static ssize_t wm9715_ts_read(struct file *filp, char *buf, size_t count, loff_t *l)
{
	int i;
	TS_EVENT t;
	//short tmp;

//re_read_ts:
	if ( head == tail ) {
		if (filp->f_flags & O_NONBLOCK) {
			//printk("-EAGAIN\n");
			return -EAGAIN;
		}
		if(wait_event_interruptible(queue, head != tail))
			return -EINTR;
	}

	for ( i= count; i>=sizeof(TS_EVENT); i-=sizeof(TS_EVENT), buf+=sizeof(TS_EVENT) ) {
		if ( head == tail ) {
			dbg("break\n");
			break;
		}
		ts_get_data(&t);
		dbg("Report to User: pressure=%d x=%d y=%d, ms=%d\n", t.pressure, t.x, t.y, t.millisecs);
		copy_to_user(buf, &t, sizeof(t));
	}

	//printk("Exit ts read \n");
	return count - i;
}


static unsigned int wm9715_ts_poll(struct file *filp, poll_table *wait)
{
    poll_wait(filp, &queue, wait);
    if ( head != tail )
        return (POLLIN | POLLRDNORM);
    return 0;
}


static struct file_operations wm9715_ts_fops = {
	.read           = wm9715_ts_read,
  .poll           = wm9715_ts_poll,
  .ioctl          = wm9715_ts_ioctl,
	.open           = wm9715_ts_open,
	.release        = wm9715_ts_close,
};


static int wm9715_ts_probe(struct platform_device *pdev)
{
	int ret;
	int error = 0;

	dbg("in [%s]\n",__FUNCTION__);
	//ac97_init();
	the_wm9715 = kmalloc(sizeof(wm9715_t), GFP_KERNEL);
	if (!the_wm9715)
		return -ENOMEM;

	memset(the_wm9715 , 0, sizeof(*the_wm9715));


	//mutex_init(&l_vibratedev.mlock);
    	// register char device
	if (register_chrdev (TS_MAJOR, TS_NAME, &wm9715_ts_fops)) {
		printk (KERN_ERR "wmt touch: unable to get major %d\n", TS_MAJOR);
		error = -EIO;
		//goto initend;
		return error;
	}

	l_vibratedev.dev_class = class_create(THIS_MODULE, TS_NAME);
	if (IS_ERR(l_vibratedev.dev_class))
	{
		error = PTR_ERR(l_vibratedev.dev_class);
		printk(KERN_ERR "Can't class_create touch device !!\n");
		return error;
	}

	l_vibratedev.device = device_create(l_vibratedev.dev_class, NULL, MKDEV(TS_MAJOR, 0), NULL, TS_NAME);
    	if (IS_ERR(l_vibratedev.device))
    	{
	    	error = PTR_ERR(l_vibratedev.device);
	    	printk(KERN_ERR "Failed to create device %s !!!",TS_NAME);
	    	//goto initend2;
	    	return error;
    	}
	//register_chrdev(TS_MAJOR, TS_NAME, &wm9715_ts_fops);

	mutex_init(&the_wm9715->codec_mutex);
	init_waitqueue_head(&the_wm9715->pen_irq_wait);

	init_completion(&the_wm9715->ts_exit);

	if(touch_phy_irqnum == IRQ_GPIO5){
		// set GPIO5 signal as extern int 5 and not gpio5
		REG32_VAL(0xd8110000 + 0x64) &= ~BIT5;
	        // Raising edg for pendow , in fact the setting related to exter5
	 	REG32_VAL(0xD8110000 + 0x300) |= (BIT11 | BIT10);
	 	REG32_VAL(0xd8110000 + 0x304) |= BIT5; //clear interrupt

	 	REG8_VAL(0xd8140000 + 0x40 + 52) |= BIT3; // enable int

		the_wm9715->pen_irq = 52; // extern5 interrupt
	}
	else if(touch_phy_irqnum == IRQ_GPIO4){
		// set GPIO4 signal as extern int 4 and not gpio4
		REG32_VAL(0xd8110000 + 0x64) &= ~BIT4;
	        // Raising edg for pendow , in fact the setting related to exter4
	 	REG32_VAL(0xD8110000 + 0x300) |= (BIT9 | BIT8);
	 	REG32_VAL(0xd8110000 + 0x304) |= BIT4; //clear interrupt

	 	REG8_VAL(0xd8140000 + 0x40 + 15) |= BIT3; // enable int

		the_wm9715->pen_irq = 15; // extern4 interrupt
	}
	else if(touch_phy_irqnum == IRQ_GPIO6){
		// set GPIO6 signal as extern int 6 and not gpio6
		REG32_VAL(0xd8110000 + 0x64) &= ~BIT6;
	        // Raising edg for pendow , in fact the setting related to exter4
	 	REG32_VAL(0xD8110000 + 0x300) |= (BIT13 | BIT12);
	 	REG32_VAL(0xd8110000 + 0x304) |= BIT6; //clear interrupt

	 	REG8_VAL(0xd8140000 + 0x40 + 53) |= BIT3; // enable int

		the_wm9715->pen_irq = 53; // extern4 interrupt
	}


	ret = request_irq(the_wm9715->pen_irq, wm9715_hard_irq,IRQF_DISABLED /*| IRQF_TRIGGER_RISING*/, "wm9715", the_wm9715);

	if (ret) {
		dbg(KERN_ERR "wm9715: unable to grab irq%d: %d\n",
			   the_wm9715->pen_irq, ret);

		kfree(the_wm9715);
		the_wm9715 = NULL;
		return -EBUSY;
	}
	//init_input_dev(the_wm9715->inputdev);
	the_wm9715->inputdev = input_allocate_device();
	if (!the_wm9715->inputdev)
	{
		return -ENOMEM;
	}
	the_wm9715->inputdev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	the_wm9715->inputdev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	#ifdef WMT_MULTITOUCH_SUPPORT
	input_set_abs_params(the_wm9715->inputdev, ABS_MT_POSITION_X, 0, 800, 0, 0);
	input_set_abs_params(the_wm9715->inputdev, ABS_MT_POSITION_Y, 0, 480, 0, 0);
	//input_set_abs_params(the_wm9715->inputdev, ABS_MT_TRACKING_ID, 0, dt->num_users, 0, 0);
	#else
	input_set_abs_params(the_wm9715->inputdev, ABS_X, 0, panelres_x, 0, 0);
	input_set_abs_params(the_wm9715->inputdev, ABS_Y, 0, panelres_y, 0, 0);
	input_set_abs_params(the_wm9715->inputdev, ABS_PRESSURE, 0, 1, 0, 0);
	#endif
	set_bit(KEY_F1, the_wm9715->inputdev->keybit);
	the_wm9715->inputdev->name = "touchscreen";
	the_wm9715->inputdev->phys = "touchscreen";
	the_wm9715->inputdev->id.bustype = 0;
    the_wm9715->inputdev->id.vendor  = 0;
    the_wm9715->inputdev->id.product = 0;
    the_wm9715->inputdev->id.version = 0;
	input_register_device(the_wm9715->inputdev);


	ts_clear();
	wm9715_pendown_detect();
	dev_set_drvdata(&pdev->dev, the_wm9715);
	kthread_run(wm9715_ts_thread, the_wm9715, "wm9715_ts");
	dbg("wm9715 touch screen driver register ok\n");
	return 0;
}

static int wm9715_ts_remove(struct platform_device *pdev)
{
	/* kill thread */
	if (the_wm9715->ts_task) {
		the_wm9715->ts_task = NULL;
		the_wm9715->pen_is_down = 1;
		mutex_unlock(&the_wm9715->codec_mutex);
		wake_up_interruptible(&the_wm9715->pen_irq_wait);
		wait_for_completion(&the_wm9715->ts_exit);
		the_wm9715->pen_is_down = 0;
		mutex_lock(&the_wm9715->codec_mutex);
	}

	free_irq(the_wm9715->pen_irq, the_wm9715);

	unregister_chrdev(TS_MAJOR, TS_NAME);
	dev_set_drvdata(&pdev->dev, NULL);
	input_unregister_device(the_wm9715->inputdev);
	input_free_device(the_wm9715->inputdev);
	kfree(the_wm9715);
	the_wm9715 = NULL;
	return 0;
}

#ifdef CONFIG_PM

static int wm9715_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("%s status = %d\n", __FUNCTION__, state);
	#if 0
	    printk("%s status = %d, level = %d\n", __FUNCTION__, state, level);
	    switch ( level ) {
	    case SUSPEND_NOTIFY:
	        printk("SUSPEND_NOTIFY \n");
	       break;

	    case SUSPEND_DISABLE:
	        printk("SUSPEND_DISABLE \n");
	        break;

	    case SUSPEND_SAVE_STATE:
	        printk("SUSPEND_SAVE_STATE \n");
	        if ( state == PM_SUSPEND_MEM ) {
	            printk("\tPM_SUSPEND_MEM \n");
	        }
	        break;

	    case SUSPEND_POWER_DOWN:
	        printk("SUSPEND_POWER_DOWN \n");
	        break;
	    }
	#endif
	    if(touch_phy_irqnum == IRQ_GPIO5){
			printk("%s touchirq = gpio5\n", __FUNCTION__);
		 	REG8_VAL(0xd8140000 + 0x40 + 52) &= ~BIT3; // disable int
		 	free_irq(the_wm9715->pen_irq, the_wm9715);

			//kthread_stop(kenvctrld_task);
		}
		else if(touch_phy_irqnum == IRQ_GPIO4){
			printk("%s touchirq = gpio4\n", __FUNCTION__);
			REG8_VAL(0xd8140000 + 0x40 + 15) &= ~BIT3; // disable int
			free_irq(the_wm9715->pen_irq, the_wm9715);

			//kthread_stop(kenvctrld_task);
		}
		else if(touch_phy_irqnum == IRQ_GPIO6){
			printk("%s touchirq = gpio6\n", __FUNCTION__);
			REG8_VAL(0xd8140000 + 0x40 + 53) &= ~BIT3; // disable int
			free_irq(the_wm9715->pen_irq, the_wm9715);

			//kthread_stop(kenvctrld_task);
		}
	    return 0;
}


static int wm9715_ts_resume(struct platform_device *pdev)
{
	int ret;
	printk("%s \n", __FUNCTION__);


	/*#define SUSPEND_NOTIFY 0
	#define SUSPEND_SAVE_STATE 1
	#define SUSPEND_DISABLE 2
	#define SUSPEND_POWER_DOWN 3
	#define RESUME_POWER_ON 0
	#define RESUME_RESTORE_STATE 1
	#define RESUME_ENABLE 2*/
	#if 0
	    printk("%s level = %d\n", __FUNCTION__, level);
	    switch ( level ) {
	    case RESUME_POWER_ON:
	        printk("RESUME_POWER_ON \n");
	        //ac97_bus_cold_reset();
	        break;
	    case RESUME_RESTORE_STATE:
	        printk("RESUME_RESTORE_STATE \n");
	        break;
	    case RESUME_ENABLE:
	        wm9715_pendown_detect();
	        printk("RESUME_ENABLE \n");
	        break;
	    }
	 #endif
	    if(touch_phy_irqnum == IRQ_GPIO5){
			printk("%s touchirq = gpio5\n", __FUNCTION__);

			// set GPIO5 signal as extern int 5 and not gpio5
			REG32_VAL(0xd8110000 + 0x64) &= ~BIT5;
		        // Raising edg for pendow , in fact the setting related to exter5
		 	REG32_VAL(0xD8110000 + 0x300) |= (BIT11 | BIT10);
		 	REG32_VAL(0xd8110000 + 0x304) |= BIT5; //clear interrupt

		 	REG8_VAL(0xd8140000 + 0x40 + 52) |= BIT3; // enable int

			//the_wm9715->pen_irq = 52; // extern5 interrupt
			printk("the_wm9715->pen_irq = %d\n", the_wm9715->pen_irq);

			ret = request_irq(the_wm9715->pen_irq, wm9715_hard_irq,IRQF_DISABLED /*| IRQF_TRIGGER_RISING*/, "wm9715", the_wm9715);

			if (ret) {
				dbg(KERN_ERR "wm9715: unable to grab irq%d: %d\n",
					   the_wm9715->pen_irq, ret);

				kfree(the_wm9715);
				the_wm9715 = NULL;
				return -EBUSY;
			}

			wm9715_pendown_detect();
			//dev_set_drvdata(&pdev->dev, the_wm9715);
			//kenvctrld_task = kthread_run(wm9715_ts_thread, the_wm9715, "wm9715_ts");

		}
		else if(touch_phy_irqnum == IRQ_GPIO4){
			printk("%s touchirq = gpio4\n", __FUNCTION__);
			// set GPIO4 signal as extern int 4 and not gpio4
			REG32_VAL(0xd8110000 + 0x64) &= ~BIT4;
		        // Raising edg for pendow , in fact the setting related to exter4
		 	REG32_VAL(0xD8110000 + 0x300) |= (BIT9 | BIT8);
		 	REG32_VAL(0xd8110000 + 0x304) |= BIT4; //clear interrupt

		 	REG8_VAL(0xd8140000 + 0x40 + 15) |= BIT3; // enable int

			//the_wm9715->pen_irq = 15; // extern4 interrupt
			printk("the_wm9715->pen_irq = %d\n", the_wm9715->pen_irq);

			ret = request_irq(the_wm9715->pen_irq, wm9715_hard_irq,IRQF_DISABLED /*| IRQF_TRIGGER_RISING*/, "wm9715", the_wm9715);

			if (ret) {
				dbg(KERN_ERR "wm9715: unable to grab irq%d: %d\n",
					   the_wm9715->pen_irq, ret);

				kfree(the_wm9715);
				the_wm9715 = NULL;
				return -EBUSY;
			}

			wm9715_pendown_detect();
			//dev_set_drvdata(&pdev->dev, the_wm9715);
			//kenvctrld_task = kthread_run(wm9715_ts_thread, the_wm9715, "wm9715_ts");
		}
		else if(touch_phy_irqnum == IRQ_GPIO6){
			printk("%s touchirq = gpio6\n", __FUNCTION__);
			// set GPIO4 signal as extern int 6 and not gpio6
			REG32_VAL(0xd8110000 + 0x64) &= ~BIT6;
		        // Raising edg for pendow , in fact the setting related to exter4
		 	REG32_VAL(0xD8110000 + 0x300) |= (BIT13 | BIT12);
		 	REG32_VAL(0xd8110000 + 0x304) |= BIT6; //clear interrupt

		 	REG8_VAL(0xd8140000 + 0x40 + 53) |= BIT3; // enable int

			printk("the_wm9715->pen_irq = %d\n", the_wm9715->pen_irq);

			ret = request_irq(the_wm9715->pen_irq, wm9715_hard_irq,IRQF_DISABLED /*| IRQF_TRIGGER_RISING*/, "wm9715", the_wm9715);

			if (ret) {
				dbg(KERN_ERR "wm9715: unable to grab irq%d: %d\n",
					   the_wm9715->pen_irq, ret);

				kfree(the_wm9715);
				the_wm9715 = NULL;
				return -EBUSY;
			}

			wm9715_pendown_detect();
			//dev_set_drvdata(&pdev->dev, the_wm9715);
			//kenvctrld_task = kthread_run(wm9715_ts_thread, the_wm9715, "wm9715_ts");
		}
	    return 0;
}
#else

#define wm9715_ts_suspend NULL
#define wm9715_ts_resume NULL

#endif  // CONFIG_PM


/************************Battery Proc**********************/

#define BATT_PROC_NAME "wm9715_battery"
static struct proc_dir_entry * wm9715_batt;

static int wm9715_battery_read_info( char  *page,
                           char  **start,
                           off_t off,
                           int   count,
                           int   *eof,
                           void  *data )
{
	int sample = 0, len = 0;
	//printk("readind Battery ADC sample\n");
	if ( wm9712_poll_sample(WM97XX_ADCSEL_BMON, &sample) != RC_VALID)
	{
		sample &= 0xfff;
		len = sprintf( page, "battery=%d\nvoltage=%dmV\n", sample, sample * 3 * 3300 / 4096);
	}
	//printk("battery=%d  voltage=%dmV  ret=%d\n", sample, sample * 3 * 3300 / 4096,len);
    return len;
}

extern struct proc_dir_entry proc_root;
static int wm9715_battery_proc(int add)
{
	if(add)
	{
		wm9715_batt = create_proc_read_entry( BATT_PROC_NAME,
											0444,
											&proc_root,
											wm9715_battery_read_info,
											NULL);
		if( wm9715_batt != NULL )
		{
			wm9715_batt->owner = THIS_MODULE;
			dbg("\"wm9715_battery\" create proc: successful\n");
			return 0;
		}
		else
		{
			dbg("\"wm9715_battery\" create proc: fail\n");
			return -ENOMEM;
		}

	}
	else
		remove_proc_entry(BATT_PROC_NAME, &proc_root);

	return 0;
}

/***************************************************************************
	platform device struct define
****************************************************************************/
static void wm9715_platform_release(struct device *device)
{
    return;
}

/*
static struct resource wm9715_ts_resources[] = {
    [0]             = {
	.start  = IRQ_GPIO5,   //External Interrupt 5
       .end    = IRQ_GPIO5,
       .flags  = IORESOURCE_IRQ,
       },
};
*/

static struct platform_device wm9715_ts_device = {
    .name           = DRIVER_NAME,
    .id             = 0,
    .dev            = {
        .release = wm9715_platform_release,
    },
//    .num_resources  = ARRAY_SIZE(wm9715_ts_resources),
//    .resource       = wm9715_ts_resources,
};


/***********************************************************/

/*
static struct device_driver wm9715_ts_driver = {
	.name           = DRIVER_NAME,
	.bus            = &platform_bus_type,
	.probe          = wm9715_ts_probe,
	.remove         = wm9715_ts_remove,
    .suspend        = wm9715_ts_suspend,
    .resume         = wm9715_ts_resume
};
*/

static struct platform_driver wm9715_ts_driver = {
	.driver = {
	             .name = DRIVER_NAME,
		     .owner	= THIS_MODULE,
	 },
	.probe = wm9715_ts_probe,
	.remove = wm9715_ts_remove,
	.suspend        = wm9715_ts_suspend,
	.resume         = wm9715_ts_resume,
};

static int __init wm9715_ts_init(void)
{
	int ret;
	//int error = 0;
    	unsigned char retval[32];
	wmt_getsyspara("touchic", retval, 32);
	if(!strcmp(retval,"false")){
		printk("touchic no exist....................!");
		return 0;
	}

	wmt_getsyspara("touchcodec", retval, 32);
	if(!strcmp(retval,"cs7146")){
		printk("touchic set to cs7146, exit wm97xx touch! ");
		return 0;
	}

    	wmt_getsyspara("touchirq", retval, 32);
	{
		if(!strcmp(retval,"gpio5"))
			touch_phy_irqnum = IRQ_GPIO5;
		else if(!strcmp(retval,"gpio4"))
			touch_phy_irqnum = IRQ_GPIO4;
		else if(!strcmp(retval,"gpio6"))
			touch_phy_irqnum = IRQ_GPIO6;
	}
	printk("touchirq = %d!", touch_phy_irqnum);
	wmt_getenv2int("panelres.x",retval,32,(int*)&panelres_x);
	wmt_getenv2int("panelres.y",retval,32,(int*)&panelres_y);
	printk("panelres.x = %d, panelres.y = %d!", panelres_x, panelres_y);

	g_CalcParam.a1 = 1;
	g_CalcParam.b1 = 0;
	g_CalcParam.c1 = 0;
	g_CalcParam.a2 = 0;
	g_CalcParam.b2 = 1;
	g_CalcParam.c2 = 0;
	g_CalcParam.delta = 1;

    ret = platform_device_register(&wm9715_ts_device);
    if(ret)
        return ret;

//    ret = driver_register(&wm9715_ts_driver);
    ret = platform_driver_register(&wm9715_ts_driver);
    if(ret)
    {
        printk("can not register wm9715 touchscreen driver\n");
        platform_device_unregister(&wm9715_ts_device);
        return ret;
    }

	printk("wm9715 ts driver init ok!\n");
	//wm9715_battery_proc(1); // adding for battery
	return ret;
}

static void __exit wm9715_ts_exit(void)
{
	dbg("%s\n",__FUNCTION__);
	//wm9715_battery_proc(0); // adding for battery

	//driver_unregister(&wm9715_ts_driver);
	platform_driver_unregister(&wm9715_ts_driver);
	platform_device_unregister(&wm9715_ts_device);
}


module_init(wm9715_ts_init);
module_exit(wm9715_ts_exit);

MODULE_DESCRIPTION("Wolfson WM9715 touchscreen driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("VIA ShenZhen MCE SW Team");


