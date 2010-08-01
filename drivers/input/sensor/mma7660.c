#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/mma7660.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <mach/hardware.h>

//#define DEBUG 1

#define DEBUG_SENSOR
#ifdef DEBUG_SENSOR
	#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
	#define dbg(format, arg...)
#endif


#define MAX_FAILURE_COUNT 10

#define MMA7660_ADDR	0x4C
//#define MMA7660_IRQ	53


#define LAND_PORT_MASK 0x1C
#define LAND_LEFT 0x1
#define LAND_RIGHT 0x2
#define PORT_INVERT 0x5
#define PORT_NORMAL 0x6

#define LANDSCAPE_LOCATION 0
#define PORTRAIT_LOCATION  1

#define SENSOR_UI_MODE 0
#define SENSOR_GRAVITYGAME_MODE 1

#define UI_SAMPLE_RATE 0xFC

#define GSENSOR_PROC_NAME "gsensor_config"

#define sin30_1000  500
#define	cos30_1000  866


/*double xyz_convert_table[64] = {
0,0.047,0.094,0.141,0.188,0.234,0.281,0.328,
0.375,0.422,0.469,0.516,0.563,0.609,0.656,0.703,
0.750,0.797,0.844,0.891,0.938,0.984,1.031,1.078,
1.125,1.172,1.219,1.266,1.313,1.359,1.406,1.453,
-1.500,-1.453,-1.406,-1.359,-1.313,-1.266,-1.219,-1.172,
-1.125,-1.078,-1.031,-0.984,-0.938,-0.891,-0.844,-0.797,
-0.750,-0.703,-0.656,-0.609,-0.563,-0.516,-0.469,-0.422,
-0.375,-0.328,-0.281,-0.234,-0.188,-0.141,-0.094,-0.047
};*/

int xyz_convert_table[64] = {
0,47,94,141,188,234,281,328,
375,422,469,516,563,609,656,703,
750,797,844,891,938,984,1031,1078,
1125,1172,1219,1266,1313,1359,1406,1453,
-1500,-1453,-1406,-1359,-1313,-1266,-1219,-1172,
-1125,-1078,-1031,-984,-938,-891,-844,-797,
-750,-703,-656,-609,-563,-516,-469,-422,
-375,-328,-281,-234,-188,-141,-94,-47
};

int xyzdegree_convert_table[64][2] = {
	{0, 9000},{269,8731},{538,8462},{808,8192},
	{1081,7919},{1355,7645},{1633,7367},{1916,7084},
	{2202,6798},{2495,6505},{2795,6205},{3104,5896},
	{3423,5577},{3754,5246},{4101,4899},{4468,4532},
	{4859,4141},{5283,3717},{5754,3246},{6295,2705},
	{6964,2036},{7986,1014},
	/* These are invalaid from [22] to [42] invalaid */
	[22]={0xFFFF,0xFFFF},
	[23]={0xFFFF,0xFFFF},
	[24]={0xFFFF,0xFFFF},
	[25]={0xFFFF,0xFFFF},
	[26]={0xFFFF,0xFFFF},
	[27]={0xFFFF,0xFFFF},
	[28]={0xFFFF,0xFFFF},
	[29]={0xFFFF,0xFFFF},
	[30]={0xFFFF,0xFFFF},
	[31]={0xFFFF,0xFFFF},
	[32]={0xFFFF,0xFFFF},
	[33]={0xFFFF,0xFFFF},
	[34]={0xFFFF,0xFFFF},
	[35]={0xFFFF,0xFFFF},
	[36]={0xFFFF,0xFFFF},
	[37]={0xFFFF,0xFFFF},
	[38]={0xFFFF,0xFFFF},
	[39]={0xFFFF,0xFFFF},
	[40]={0xFFFF,0xFFFF},
	[41]={0xFFFF,0xFFFF},
	[42]={0xFFFF,0xFFFF},
	[43]={-7986,-1014},
	[44]={-6964,-2036},
	[45]={-6295,-2705},
	[46]={-5754,-3246},
	[47]={-5283, -3717},
	[48]={-4859,-4141},
	[49]={-4468,-4532},
	[50]={-4101,-4899},
	[51]={-3754,-5246},
	[52]={-3423,-5577},
	[53]={-3104,-5896},
	[54]={-2795,-6205},
	[55]={-2495,-6505},
	[56]={-2202,-6798},
	[57]={-1916,-7084},
	[58]={-1633,-7367},
	[59]={-1355,-7645},
	[60]={-1081,-7919},
	[61]={-808,-8192},
	[62]={-538,-8462},
	[63]={-269,-8731},
};


static struct platform_device *this_pdev;

struct mma7660_data {
	struct input_dev *input_dev;
	struct work_struct work;
};

struct mma7660_config
{
	int xyz_axis[3][3]; // (axis,direction)
	int rxyz_axis[3][3];
	int irq;
	int int_gpio; //0-7
	struct proc_dir_entry* sensor_proc;
	int sensorlevel;
	unsigned int sensitive;
	int shake_enable; // 1--enable shake, 0--disable shake
	int manual_rotation; // 0--landance, 90--vertical
};

static struct mma7660_config l_sensorconfig = {
			.xyz_axis = {
				{ABS_X, 1},
				{ABS_Y, 1},
				{ABS_Z, -1},
				},
			/*
			.rxyz_axis = {
				{ABS_RY, -1},
				{ABS_RX, 1},
				{ABS_RZ, -1},
				},
				*/
			.irq = 53,
			.int_gpio = 6,
			.sensor_proc = NULL,
			.sensorlevel = SENSOR_UI_MODE,
			.sensitive = 0x9,
			.shake_enable = 1, // default enable shake
};


//
#define SET_GPIO_GSENSOR_INT(num) {\
	REG32_VAL(0xd8110064) &= ~(1<<(num)); /*0x00000040;*/ \
	REG32_VAL(0xd8110300) |= (1<<((num)*2+1));/*0x00002000;*/ \
	REG32_VAL(0xd8110300) |= (1<<((num)*2));/*0x00001000;*/ \
	REG32_VAL(0xd8110304) |= (1<<(num));/*0x00000040;*/ \
}

// 
#define SET_GSENSOR_INT_OUTGPIO(num) {\
	REG32_VAL(0xd8110064) |= (1<<(num)); \
	REG32_VAL(0xd811008C) |= (1<<(num)); \
}

/* Addresses to scan -- protected by sense_data_mutex */
//static char sense_data[RBUFF_SIZE + 1];
static struct mutex sense_data_mutex;

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static char cspec_num;
static atomic_t cspec_frq;

//static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t t_flag;
static atomic_t mv_flag;

//static int pffd_mode = 0;
//static int failure_count = 0;

static short mmad_delay = 0;

static atomic_t suspend_flag = ATOMIC_INIT(0);

//static struct mma7660_platform_data *pdata;
static int revision = -1;
/* mma HW info */
static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "AK8976A_%#x\n", revision);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static struct kobject *android_gsensor_kobj;

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		printk(KERN_ERR
		       "mma7660 gsensor_sysfs_init:"\
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR
		       "mma7660 gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

extern int i2c_wmt_read_msg(
	unsigned int slave_addr, 	/*!<; //[IN] Salve address */
	char *buf, 				/*!<; //[OUT] Pointer to data */
	unsigned int length,		/*!<; //Data length */
	int restart, 				/*!<; //Need to restart after a complete read */
	int last 					/*!<; //Last read */
);
extern int i2c_wmt_write_msg(
	unsigned int slave_addr, 	/*!<; //[IN] Salve address */
	char *buf, 				/*!<; //[OUT] Pointer to data */
	unsigned int length,		/*!<; //Data length */
	int restart, 				/*!<; //Need to restart after a complete write */
	int last 					/*!<; //Last read */
);

extern void wmt_i2c_xfer_continue_if(struct i2c_msg *msg, unsigned int num);
extern void wmt_i2c_xfer_if(struct i2c_msg *msg);

static int sensor_i2c_do_xfer(
	struct i2c_msg msgs[], 		/*!<; //[IN] transfer data  */
	int num						/*!<; //[IN] transfer data length */
)
{
	int i;
	struct i2c_msg *pmsg = NULL;
	int ret = 0;

	for (i = 0; ret >= 0 && i < num; i++) {
		int last = ((i + 1) == num);
		int restart = (i != 0) ;
		pmsg = &msgs[i];
		if (pmsg->flags & I2C_M_NOSTART)
			restart = 1;
		if (pmsg->flags & I2C_M_RD)/* READ*/
			ret = i2c_wmt_read_msg(pmsg->addr , pmsg->buf, pmsg->len, restart, last) ;
		else/* Write*/
			ret = i2c_wmt_write_msg(pmsg->addr , pmsg->buf, pmsg->len, restart, last) ;
	}

	if (ret < 0)
		return ret;
	else
		return i;

}

int sensor_i2c_write(unsigned int addr,unsigned int index,char *pdata,int len)
{
    struct i2c_msg msg[1];
	unsigned char buf[len+1];

	//addr = (addr >> 1);
    buf[0] = index;
	memcpy(&buf[1],pdata,len);
    msg[0].addr = addr;
    msg[0].flags = 0 ;
    msg[0].flags &= ~(I2C_M_RD);
    msg[0].len = len+1;
    msg[0].buf = buf;
    //sensor_i2c_do_xfer(msg,1);
    wmt_i2c_xfer_if(msg);

#ifdef DEBUG
{
	int i;

	printk("sensor_i2c_write(addr 0x%x,index 0x%x,len %d\n",addr,index,len);
	for(i=0;i<len;i+=8){
		printk("%d : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",i,
			pdata[i],pdata[i+1],pdata[i+2],pdata[i+3],pdata[i+4],pdata[i+5],pdata[i+6],pdata[i+7]);
	}
}
#endif
    return 0;
} /* End of sensor_i2c_write */

int sensor_i2c_read(unsigned int addr,unsigned int index,char *pdata,int len)
{
	struct i2c_msg msg[2];
	unsigned char buf[len+1];

	//addr = (addr >> 1);
	memset(buf,0x55,len+1);
    buf[0] = index;
	buf[1] = 0x0;

    msg[0].addr = addr;
    msg[0].flags = 0 ;
	msg[0].flags &= ~(I2C_M_RD);
	msg[0].len = 1;
    msg[0].buf = buf;

	msg[1].addr = addr;
	msg[1].flags = 0 ;
	msg[1].flags |= (I2C_M_RD);
	msg[1].len = len;
	msg[1].buf = buf;

	//sensor_i2c_do_xfer(msg, 2);
	wmt_i2c_xfer_continue_if(msg,2);
	memcpy(pdata,buf,len);
#ifdef DEBUG
{
	int i;

	printk("sensor_i2c_read(addr 0x%x,index 0x%x,len %d\n",addr,index,len);
	for(i=0;i<len;i+=8){
		printk("%d : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",i,
			pdata[i],pdata[i+1],pdata[i+2],pdata[i+3],pdata[i+4],pdata[i+5],pdata[i+6],pdata[i+7]);
	}
}
#endif
    return 0;
} /* End of sensor_i2c_read */

void mma7660_chip_init(void)
{
	char txData[1];
	
	// the default mode is for UI
	txData[0]=0;
	sensor_i2c_write(MMA7660_ADDR,7,txData,1);
	switch (l_sensorconfig.sensorlevel)
	{
		case SENSOR_UI_MODE:
			txData[0]=60;
			sensor_i2c_write(MMA7660_ADDR,5,txData,1);
			txData[0] = (1 == l_sensorconfig.shake_enable) ? 0xe3:0x03;
			sensor_i2c_write(MMA7660_ADDR,6,txData,1);
			txData[0]=0xFC;  
			sensor_i2c_write(MMA7660_ADDR,8,txData,1);
			break;
		case SENSOR_GRAVITYGAME_MODE:
			txData[0]=0;
			sensor_i2c_write(MMA7660_ADDR,5,txData,1);
			txData[0]=0x10; 
			sensor_i2c_write(MMA7660_ADDR,6,txData,1);
			txData[0]=0xFB; // should right ?????
			sensor_i2c_write(MMA7660_ADDR,8,txData,1);
			break;
	};	
	txData[0]=0xf9;
	sensor_i2c_write(MMA7660_ADDR,7,txData,1);

	// set gpio as external interrupt
	SET_GPIO_GSENSOR_INT(l_sensorconfig.int_gpio);

	/*
	REG32_VAL(0xd8110064) &= ~0x00000040;
	REG32_VAL(0xd8110300) |= 0x00002000;
	REG32_VAL(0xd8110300) |= 0x00001000;
	REG32_VAL(0xd8110304) |= 0x00000040;
	//REG8_VAL(0xd8140000 + 0x40 + 52) |= 0x8;
	*/

	return;
}

static int mma_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	return ret;
}

static int mma_aot_release(struct inode *inode, struct file *file)
{
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static int
mma_aot_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_TFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		atomic_set(&a_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		break;
	case ECS_IOCTL_APP_SET_TFLAG:
		atomic_set(&t_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_TFLAG:
		flag = atomic_read(&t_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		mmad_delay = flag;
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = mmad_delay;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static int mma_pffd_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 2) == 0) {
			atomic_set(&reserve_open_flag, 2);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	return ret;
}

static int mma_pffd_release(struct inode *inode, struct file *file)
{
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static int
mma_pffd_ioctl(struct inode *inode, struct file *file,
	       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;
	int ret = 0;

	switch (cmd) {
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_RESET_PEDOMETER:
		//ret = AKECS_Set_PERST();
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		mmad_delay = flag;
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = mmad_delay;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static int mmad_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int mmad_release(struct inode *inode, struct file *file)
{
	//AKECS_CloseDone();
	return 0;
}

static int
mmad_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{

	void __user *argp = (void __user *)arg;

	char msg[RBUFF_SIZE + 1], rwbuf[5], numfrq[2];
	int ret = -1, status;
	short mode, value[12], step_count, delay;
	char *pbuffer = 0;

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_MODE:
		if (copy_from_user(&mode, argp, sizeof(mode)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_YPR:
		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_STEP_CNT:
		if (copy_from_user(&step_count, argp, sizeof(step_count)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_INIT:
		//ret = AKECS_Init();
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		//AKECS_Reset();
		break;
	case ECS_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		//ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		//ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		//ret = AKECS_SetMode((char)mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GETDATA:
		//ret = AKECS_TransRBuff(msg, RBUFF_SIZE);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_NUMFRQ:
		numfrq[0] = cspec_num;
		numfrq[1] = atomic_read(&cspec_frq);
		break;
	case ECS_IOCTL_SET_PERST:
		//ret = AKECS_Set_PERST();
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_G0RST:
		//ret = AKECS_Set_G0RST();
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		//AKECS_Report_Value(value);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		//status = AKECS_GetOpenStatus();
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		//status = AKECS_GetCloseStatus();
		break;
	case ECS_IOCTL_SET_STEP_CNT:
		//AKECS_Report_StepCount(step_count);
		break;
	case ECS_IOCTL_GET_CALI_DATA:
		//pbuffer = get_mma_cal_ram();
		break;
	case ECS_IOCTL_GET_DELAY:
		delay = mmad_delay;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GETDATA:
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_NUMFRQ:
		if (copy_to_user(argp, &numfrq, sizeof(numfrq)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_CALI_DATA:
		if (copy_to_user(argp, pbuffer, MAX_CALI_SIZE))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static int report_orient_degree(struct input_dev *dev, unsigned int code, int value)
{
	if ((value != 0xFFFF) && (value != -0xFFFF))
	{
		input_report_abs(dev, code, value);
		return 0;
	}
	return -1;
}


#define SHOW_AXIS_INFO(axis, val) 
/*{ \
	switch (axis) \
	{ \
		case ABS_X: \
			printk(KERN_ALERT "X-axis:%d\n", val); \
			break; \
		case ABS_Y: \
			printk(KERN_ALERT "Y-axis:%d\n", val); \
			break; \
		case ABS_Z: \
			printk(KERN_ALERT "Z-axis:%d\n", val); \
			break; \
	}; \
}
*/

static void mma_work_func(struct work_struct *work)
{
	struct mma7660_data *data;
	char rxData[1];
	char txData[1];
	int reportval = 0;
	unsigned char tiltval = 0;	
	int location = -1;

	data = dev_get_drvdata(&this_pdev->dev);

	mutex_lock(&sense_data_mutex);
	switch (l_sensorconfig.sensorlevel)
	{
		case SENSOR_UI_MODE: // for UI
			sensor_i2c_read(MMA7660_ADDR,3,rxData,1);
			//sensor_i2c_read(MMA7660_ADDR,6,txData,1);
			//dbg(KERN_ALERT "tilt-new reg:0x%x******\nintsureg:0x%x\n", rxData[0], txData[0]);
			tiltval = rxData[0];
			if(tiltval&0x80)
			{
				if (1 == l_sensorconfig.shake_enable)
				{
					//txData[0]=0x03;
					//sensor_i2c_write(MMA7660_ADDR,6,txData,1);
					//sensor_i2c_read(MMA7660_ADDR,0,rxData,1);
					//printk("X-aixs %d\n",xyz_convert_table[rxData[0]]);
					//sensor_i2c_read(MMA7660_ADDR,1,rxData,1);
					//printk("Y-aixs %d\n",xyz_convert_table[rxData[0]]);
					//sensor_i2c_read(MMA7660_ADDR,2,rxData,1);
					//printk("Z-aixs %d\n",xyz_convert_table[rxData[0]]);
					printk("shake!\n");
					input_report_key(data->input_dev, KEY_NEXTSONG, 1);
					input_report_key(data->input_dev, KEY_NEXTSONG, 0);
					input_sync(data->input_dev);					
				}
			} else
			{
				/*
				switch (tiltval)
				{
					case 0x2a:
					case 0x26:
						if (!l_sensorconfig.xyz_axis[0][0])
						{
							location = LANDSCAPE_LOCATION;
						} else
						{
							location = PORTRAIT_LOCATION;
						}
						break;
					case 0x3a:
					case 0x36:
						if (!l_sensorconfig.xyz_axis[0][0])
						{
							location = PORTRAIT_LOCATION;
						} else
						{
							location = LANDSCAPE_LOCATION;
						}
						break;
				};
				switch (location)
				{
					case PORTRAIT_LOCATION:
						// portrait
						input_report_abs(data->input_dev, ABS_X, cos30_1000);
						input_report_abs(data->input_dev, ABS_Y, 0);
						input_report_abs(data->input_dev, ABS_Z, sin30_1000);
						input_sync(data->input_dev);
						
						break;
					case LANDSCAPE_LOCATION:
						// landscape
						input_report_abs(data->input_dev, ABS_X, 0);
						input_report_abs(data->input_dev, ABS_Y, cos30_1000);
						input_report_abs(data->input_dev, ABS_Z, sin30_1000);
						input_sync(data->input_dev);
						break;
				};
				*/
				if (l_sensorconfig.xyz_axis[0][0] == ABS_X)
				{
					sensor_i2c_read(MMA7660_ADDR,0,rxData,1);
				} else
				{
					sensor_i2c_read(MMA7660_ADDR,1,rxData,1);
				}
				if(xyz_convert_table[rxData[0]] < 0)
				{
					reportval = -xyz_convert_table[rxData[0]];
				} else
				{
					reportval = xyz_convert_table[rxData[0]];
				}
				if (reportval >= 600)
				{
					// portrait
					input_report_abs(data->input_dev, ABS_X, cos30_1000);
					input_report_abs(data->input_dev, ABS_Y, 0);
					input_report_abs(data->input_dev, ABS_Z, sin30_1000);
					input_sync(data->input_dev);
				} else if (reportval <= 450)
				{
					// landscape
					input_report_abs(data->input_dev, ABS_X, 0);
					input_report_abs(data->input_dev, ABS_Y, cos30_1000);
					input_report_abs(data->input_dev, ABS_Z, sin30_1000);
					input_sync(data->input_dev);
				}
			}
			/*
			// for landscape and portrait rotation
			switch ((tiltval&LAND_PORT_MASK)>>2)
			{
				case LAND_LEFT:
				case LAND_RIGHT:
					input_report_abs(data->input_dev, ABS_X, cos30_1000);
					input_report_abs(data->input_dev, ABS_Y, 0);
					input_report_abs(data->input_dev, ABS_Z, sin30_1000);
					input_sync(data->input_dev);
					break;
				case PORT_INVERT:
				case PORT_NORMAL:
					input_report_abs(data->input_dev, ABS_X, 0);
					input_report_abs(data->input_dev, ABS_Y, cos30_1000);
					input_report_abs(data->input_dev, ABS_Z, sin30_1000);
					input_sync(data->input_dev);
					break;
			}; */
			break;
		case SENSOR_GRAVITYGAME_MODE: // for gravitive game
			sensor_i2c_read(MMA7660_ADDR,0,rxData,1);
			reportval = (xyz_convert_table[rxData[0]])*l_sensorconfig.xyz_axis[0][1];
			//printk("X-aixs %d\n", reportval);
			SHOW_AXIS_INFO(l_sensorconfig.xyz_axis[0][0], reportval);
			//input_report_abs(data->input_dev, ABS_RX, xyz_convert_table[rxData[0]]<0?-xyz_convert_table[rxData[0]]:xyz_convert_table[rxData[0]]);
			// rawx
			input_report_abs(data->input_dev, l_sensorconfig.xyz_axis[0][0], reportval);
			//printk("RX degree %d\n",xyzdegree_convert_table[rxData[0]][0]);
			//report_orient_degree(data->input_dev, l_sensorconfig.rxyz_axis[0][0], xyzdegree_convert_table[rxData[0]][0]*l_sensorconfig.rxyz_axis[0][1]);

			// rawy
			sensor_i2c_read(MMA7660_ADDR,1,rxData,1);
			reportval = (xyz_convert_table[rxData[0]])*l_sensorconfig.xyz_axis[1][1];
			//printk("Y-aixs %d\n", reportval);
			SHOW_AXIS_INFO(l_sensorconfig.xyz_axis[1][0], reportval);
			input_report_abs(data->input_dev, l_sensorconfig.xyz_axis[1][0], reportval);
			//printk("RY degree %d\n",xyzdegree_convert_table[rxData[0]][0]);
			//report_orient_degree(data->input_dev, l_sensorconfig.rxyz_axis[1][0], xyzdegree_convert_table[rxData[0]][0]*l_sensorconfig.rxyz_axis[1][1]);

			//rawz
			sensor_i2c_read(MMA7660_ADDR,2,rxData,1);
			reportval = (xyz_convert_table[rxData[0]])*l_sensorconfig.xyz_axis[2][1];
			//printk("Z-aixs %d\n", reportval);
			SHOW_AXIS_INFO(l_sensorconfig.xyz_axis[2][0], reportval);
			input_report_abs(data->input_dev, l_sensorconfig.xyz_axis[2][0], reportval);
			//printk("RZ degree %d\n",xyzdegree_convert_table[rxData[0]][1]);
			//report_orient_degree(data->input_dev, l_sensorconfig.rxyz_axis[2][0], xyzdegree_convert_table[rxData[0]][1]*l_sensorconfig.rxyz_axis[2][1]);

			input_sync(data->input_dev);

			break;
	};



	//dbg("sensor move*************\n");
	//txData[0]=0xe3;
	//sensor_i2c_write(MMA7660_ADDR,6,txData,1);
	enable_irq(l_sensorconfig.irq);
	mutex_unlock(&sense_data_mutex);
}

static irqreturn_t mma7660_interrupt(int irq, void *dev_id)
{
	//printk("sensor interrupt!\n");
	struct mma7660_data *data = dev_id;
	disable_irq(l_sensorconfig.irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

static int sensor_change_mode(int mode)
{
	//char rxData[1];
	char txData[1];

	txData[0]=0;
	sensor_i2c_write(MMA7660_ADDR,7,txData,1);
	switch (mode)
	{
		case SENSOR_UI_MODE:
			txData[0]=60;
			sensor_i2c_write(MMA7660_ADDR,5,txData,1);
			txData[0] = (1 == l_sensorconfig.shake_enable) ? 0xe3:0x03;
			sensor_i2c_write(MMA7660_ADDR,6,txData,1);
			txData[0]= UI_SAMPLE_RATE;  
			sensor_i2c_write(MMA7660_ADDR,8,txData,1);
			break;
		case SENSOR_GRAVITYGAME_MODE:
			txData[0]=0;
			sensor_i2c_write(MMA7660_ADDR,5,txData,1);
			txData[0]=0x10; 
			sensor_i2c_write(MMA7660_ADDR,6,txData,1);
			txData[0]=0xFB; // should right ?????
			sensor_i2c_write(MMA7660_ADDR,8,txData,1);
			break;
	};	
	txData[0]=0xf9;
	sensor_i2c_write(MMA7660_ADDR,7,txData,1);

	return 0;
}

static int mmad_early_suspend()
{
	char txData[1];

	txData[0]=0;
	sensor_i2c_write(MMA7660_ADDR,7,txData,1);
	txData[0]=60;
	sensor_i2c_write(MMA7660_ADDR,5,txData,1);
	txData[0] = 0;//(1 == l_sensorconfig.shake_enable) ? 0xe3:0x03;
	sensor_i2c_write(MMA7660_ADDR,6,txData,1);
	txData[0]=0xFC;  
	sensor_i2c_write(MMA7660_ADDR,8,txData,1);
	txData[0]=0xf9;
	sensor_i2c_write(MMA7660_ADDR,7,txData,1);
	
	return 0;
}

static int mmad_late_resume(int mode)
{
	sensor_change_mode(mode);
	return 0;
}

static int sensor_writeproc( struct file   *file,
                           const char    *buffer,
                           unsigned long count,
                           void          *data )
{
	char regval[1];
	char oldval;
	unsigned int sensitive_level = 0;
	int command;
	struct mma7660_data * mma7660data = dev_get_drvdata(&this_pdev->dev);

	SET_GSENSOR_INT_OUTGPIO(l_sensorconfig.int_gpio);
	mutex_lock(&sense_data_mutex);
	// disable int
	//disable_irq(l_sensorconfig.irq);
	// get sensor level and set sensor level
	if (sscanf(buffer, "sensitive=%d\n", &sensitive_level))
	{
		if ((sensitive_level >= 0) && (sensitive_level <= 7))
		{
			l_sensorconfig.sensitive = 7 - sensitive_level;
			sensor_i2c_read(MMA7660_ADDR,7,regval,1);
			regval[0] &= 0xFE;
			oldval = regval[0];
			sensor_i2c_write(MMA7660_ADDR,7,regval,1); // standard mode
			sensor_i2c_read(MMA7660_ADDR,8,regval,1);
			regval[0] = (regval[0] & 0xF8) | l_sensorconfig.sensitive;
			sensor_i2c_write(MMA7660_ADDR,8,regval,1);
			oldval |= BIT0; // active mode
			sensor_i2c_write(MMA7660_ADDR,7,&oldval,1);
		}
	} else if (sscanf(buffer, "level=%d\n", &l_sensorconfig.sensorlevel))
	{
		// write sensor reg		
		sensor_i2c_read(MMA7660_ADDR,7,regval,1);
		oldval = regval[0];
		regval[0] &= 0xFE;		
		sensor_i2c_write(MMA7660_ADDR,7,regval,1); // standard mode
		switch (l_sensorconfig.sensorlevel)
		{
			case SENSOR_UI_MODE: // UI mode
				regval[0] = (1 == l_sensorconfig.shake_enable) ? 0xe3:0x03;
				//sensor_i2c_read(MMA7660_ADDR,6,regval,1);
				//regval[0] &= 0xEF;
				sensor_i2c_write(MMA7660_ADDR,6, regval,1);
				regval[0]=UI_SAMPLE_RATE;  			
				sensor_i2c_write(MMA7660_ADDR,8, regval,1);
				break;
			case SENSOR_GRAVITYGAME_MODE: // grative game mode, no shake and other interrupt
				regval[0]=0x10; // report every measurement
				sensor_i2c_write(MMA7660_ADDR,6, regval,1);
				regval[0]=0xFB;  			
				sensor_i2c_write(MMA7660_ADDR,8, regval,1);
				break;
		};
		oldval |= BIT0; // active mode
		sensor_i2c_write(MMA7660_ADDR,7,&oldval,1);
	} else if (sscanf(buffer, "shakenable=%d\n", &l_sensorconfig.shake_enable))
	{
		sensor_i2c_read(MMA7660_ADDR,7,regval,1);
		oldval = regval[0];	
		regval[0] &= 0xFE;			
		sensor_i2c_write(MMA7660_ADDR,7,regval,1); // standard mode		
		sensor_i2c_read(MMA7660_ADDR,6,regval,1);
		switch (l_sensorconfig.shake_enable)
		{
			case 0: // disable shake
				regval[0] &= 0x1F;
				sensor_i2c_write(MMA7660_ADDR,6, regval,1);
				dbg("Shake disable!!\n");
				break;				
			case 1: // enable shake
				regval[0] |= 0xE0;
				sensor_i2c_write(MMA7660_ADDR,6, regval,1);
				dbg("Shake enable!!\n");
				break;
		};		
		sensor_i2c_write(MMA7660_ADDR,7,&oldval,1);
	} else if (sscanf(buffer, "rotation=%d\n", &l_sensorconfig.manual_rotation))
	{
		switch (l_sensorconfig.manual_rotation)
		{
			case 90:
				// portrait
				input_report_abs(mma7660data->input_dev, ABS_X, cos30_1000);
				input_report_abs(mma7660data->input_dev, ABS_Y, 0);
				input_report_abs(mma7660data->input_dev, ABS_Z, sin30_1000);
				input_sync(mma7660data->input_dev);
				break;
			case 0:
				// landscape
				input_report_abs(mma7660data->input_dev, ABS_X, 0);
				input_report_abs(mma7660data->input_dev, ABS_Y, cos30_1000);
				input_report_abs(mma7660data->input_dev, ABS_Z, sin30_1000);
				input_sync(mma7660data->input_dev);
				break;
		};
	} else if (sscanf(buffer, "early_suspend=%d\n", &command))
	{ // for app, do something before suspend
		// switch the ui mode and disable all interrupt
		if (1 == command)
		{
			mmad_early_suspend();
		}
	} else if (sscanf(buffer, "late_resume=%d\n", &command))
	{	// for app, do something after resume
		// swithc the state before early suspend
		if (2 == command)
		{
			mmad_late_resume(l_sensorconfig.sensorlevel);
		}
	}
	//enable int
	//enable_irq(l_sensorconfig.irq);
	SET_GPIO_GSENSOR_INT(l_sensorconfig.int_gpio);
	mutex_unlock(&sense_data_mutex);
	return count;
}

static int sensor_readproc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len = sprintf(page, "level=%d\nshakenable=%d\nrotation=%d\n", 
				l_sensorconfig.sensorlevel,
				l_sensorconfig.shake_enable,
				l_sensorconfig.manual_rotation);
	return len;
}



static int mma7660_init_client(struct platform_device *pdev)
{
	struct mma7660_data *data;
	int ret;

	data = dev_get_drvdata(&pdev->dev);
	mutex_init(&sense_data_mutex);	
	ret = request_irq(l_sensorconfig.irq, mma7660_interrupt, IRQF_DISABLED,
			  "mma7660", data);
	if (ret < 0) {
		printk(KERN_ERR "mma7660_init_client: request irq failed\n");
		goto err;
	}
	l_sensorconfig.sensor_proc = create_proc_entry(GSENSOR_PROC_NAME, 0666, NULL/*&proc_root*/);
	if (l_sensorconfig.sensor_proc != NULL)
	{
		l_sensorconfig.sensor_proc->write_proc = sensor_writeproc;
		l_sensorconfig.sensor_proc->read_proc = sensor_readproc;
	}

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	/* As default, report all information */
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&t_flag, 1);
	atomic_set(&mv_flag, 1);

	return 0;

//err_free_irq:
//	free_irq(l_sensorconfig.irq, 0);
//err_alloc_data_failed:
err:
	return ret;
}

static struct file_operations mmad_fops = {
	.owner = THIS_MODULE,
	.open = mmad_open,
	.release = mmad_release,
	.ioctl = mmad_ioctl,
};

static struct file_operations mma_aot_fops = {
	.owner = THIS_MODULE,
	.open = mma_aot_open,
	.release = mma_aot_release,
	.ioctl = mma_aot_ioctl,
};

static struct file_operations mma_pffd_fops = {
	.owner = THIS_MODULE,
	.open = mma_pffd_open,
	.release = mma_pffd_release,
	.ioctl = mma_pffd_ioctl,
};

static struct miscdevice mma_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma7660_aot",
	.fops = &mma_aot_fops,
};

static struct miscdevice mma_pffd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma7660_pffd",
	.fops = &mma_pffd_fops,
};

static struct miscdevice mmad_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma7660_daemon",
	.fops = &mmad_fops,
};

static int mma7660_probe(
	struct platform_device *pdev)
{
	struct mma7660_data *mma;
	int err;
//	char rxData[8];
//	char rxData1[8];
//	char txData[2] = {0xf,0};
//	char txData1[2] = {0xff,0};
	printk("mma7660_probe&&&&&&&&&&&&&&&&&&&&&&&\n");

	l_sensorconfig.sensorlevel = SENSOR_UI_MODE;
	mma7660_chip_init();
	/*sensor_i2c_write(MMA7660_ADDR,5,txData,1);
	sensor_i2c_write(MMA7660_ADDR,6,txData1,1);

	sensor_i2c_read(MMA7660_ADDR,5,rxData,1);
	sensor_i2c_read(MMA7660_ADDR,6,rxData1,1);*/

#if 1
	mma = kzalloc(sizeof(struct mma7660_data), GFP_KERNEL);
	if (!mma) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	INIT_WORK(&mma->work, mma_work_func);
	dev_set_drvdata(&pdev->dev,mma);
	mma7660_init_client(pdev);
	this_pdev = pdev;

	mma->input_dev = input_allocate_device();

	if (!mma->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "mma7660_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	set_bit(EV_KEY, mma->input_dev->evbit);
	set_bit(KEY_NEXTSONG, mma->input_dev->keybit);

	set_bit(EV_ABS, mma->input_dev->evbit);
	/* yaw */
	input_set_abs_params(mma->input_dev, ABS_RX, 0, 360*100, 0, 0);
	/* pitch */
	input_set_abs_params(mma->input_dev, ABS_RY, -180*100, 180*100, 0, 0);
	/* roll */
	input_set_abs_params(mma->input_dev, ABS_RZ, -90*100, 90*100, 0, 0);
	/* x-axis acceleration */
	input_set_abs_params(mma->input_dev, ABS_X, -1872, 1872, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(mma->input_dev, ABS_Y, -1872, 1872, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(mma->input_dev, ABS_Z, -1872, 1872, 0, 0);
	/* temparature */
	input_set_abs_params(mma->input_dev, ABS_THROTTLE, -30, 85, 0, 0);
	/* status of magnetic sensor */
	input_set_abs_params(mma->input_dev, ABS_RUDDER, -32768, 3, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(mma->input_dev, ABS_WHEEL, -32768, 3, 0, 0);
	/* step count */
	input_set_abs_params(mma->input_dev, ABS_GAS, 0, 65535, 0, 0);
	/* x-axis of raw magnetic vector */
	input_set_abs_params(mma->input_dev, ABS_HAT0X, -2048, 2032, 0, 0);
	/* y-axis of raw magnetic vector */
	input_set_abs_params(mma->input_dev, ABS_HAT0Y, -2048, 2032, 0, 0);
	/* z-axis of raw magnetic vector */
	input_set_abs_params(mma->input_dev, ABS_BRAKE, -2048, 2032, 0, 0);

	mma->input_dev->name = "compass";

	err = input_register_device(mma->input_dev);

	if (err) {
		printk(KERN_ERR
		       "mma7660_probe: Unable to register input device: %s\n",
		       mma->input_dev->name);
		goto exit_input_register_device_failed;
	}

	err = misc_register(&mmad_device);
	if (err) {
		printk(KERN_ERR
		       "mma7660_probe: mmad_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = misc_register(&mma_aot_device);
	if (err) {
		printk(KERN_ERR
		       "mma7660_probe: mma_aot_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = misc_register(&mma_pffd_device);
	if (err) {
		printk(KERN_ERR
		       "mma7660_probe: mma_pffd_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	//err = device_create_file(&pdev->dev, &dev_attr_ms1);
	//err = device_create_file(&pdev->dev, &dev_attr_ms2);
	//err = device_create_file(&pdev->dev, &dev_attr_ms3);

	gsensor_sysfs_init();
#endif
	return 0;

exit_misc_device_register_failed:
exit_input_register_device_failed:
	input_free_device(mma->input_dev);
exit_input_dev_alloc_failed:
	kfree(mma);
exit_alloc_data_failed:
//exit_check_functionality_failed:
	return err;
}

static int mma7660_remove(struct platform_device *pdev)
{
	struct mma7660_data *mma = dev_get_drvdata(&pdev->dev);
	free_irq(l_sensorconfig.irq, mma);
	input_unregister_device(mma->input_dev);
	kfree(mma);
	return 0;
}

static int mma7660_suspend(struct platform_device *pdev, pm_message_t state)
{
    char txData[1];

	/*
	atomic_set(&suspend_flag, 1);
	if (atomic_read(&open_flag) == 2)
	{
	     txData[0] = 0;
	     sensor_i2c_write(MMA7660_ADDR,7,txData,1);
	}

	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
	*/
	disable_irq(l_sensorconfig.irq);	
	// set as gpio output
	SET_GSENSOR_INT_OUTGPIO(l_sensorconfig.int_gpio);
	return 0;
}

static int mma7660_resume(struct platform_device *pdev)
{
        char txData[1];

    mma7660_chip_init();    
	enable_irq(l_sensorconfig.irq);
	/*
	if (atomic_read(&open_flag) == 2)
	{
	    txData[0] = 0xf9;
	    sensor_i2c_write(MMA7660_ADDR,7,txData,1);
	}
	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);
	*/
	return 0;
}

static struct platform_device mma7660_device = {
    .name           = "mma7660",
    .id             = 0,
};

static struct platform_driver mma7660_driver = {
	.probe = mma7660_probe,
	.remove = mma7660_remove,
	.suspend	= mma7660_suspend,
	.resume		= mma7660_resume,
	.driver = {
		   .name = "mma7660",
		   },
};

/*
 * Brief:
 *	Get the configure of sensor from u-boot.
 * Input:
 *	no use.
 * Output:
 *	no use.
 * Return:
 *	0--success, -1--error.
 * History:
 *	Created by HangYan on 2010-4-19
 * Author:
 * 	Hang Yan in ShenZhen.
 */
extern int wmt_getsyspara_cache(char *varname, unsigned char *varval, int varlen);
static int get_axisset(void* param)
{
	char varbuf[64];
	int n;

	memset(varbuf, 0, sizeof(varbuf));
	if (wmt_getsyspara_cache("gsensor_axis", varbuf, sizeof(varbuf)))
	{
		printk(KERN_ERR "Can't get gsensor config in u-boot!!!!\n");
		return -1;
	}
	n = sscanf(varbuf, "%d,%d,%d,%d,%d,%d",
				&(l_sensorconfig.xyz_axis[0][0]),&(l_sensorconfig.xyz_axis[0][1]),
				&(l_sensorconfig.xyz_axis[1][0]),&(l_sensorconfig.xyz_axis[1][1]),
				&(l_sensorconfig.xyz_axis[2][0]),&(l_sensorconfig.xyz_axis[2][1]));
	if (n != 6)
	{
		printk(KERN_ERR "gsensor axis format is error in u-boot!!!\n");
		return -1;
	}
	memset(varbuf, 0, sizeof(varbuf));
	if (wmt_getsyspara_cache("gsensor_int", varbuf, sizeof(varbuf)))
	{
		printk(KERN_ERR "Can't get gsensor interrupt in u-boot!!!\n");
		return -1;
	}
	n = sscanf(varbuf, "gpio%d", &l_sensorconfig.int_gpio);
	if (n != 1)
	{
		printk(KERN_ERR "gsensor interrupt format is error in u-boot!!!\n");
		return -1;
	}
	printk("get the sensor config:%d,%d,%d,%d,%d,%d**********\n",
			l_sensorconfig.xyz_axis[0][0],l_sensorconfig.xyz_axis[0][1],
			l_sensorconfig.xyz_axis[1][0],l_sensorconfig.xyz_axis[1][1],
			l_sensorconfig.xyz_axis[2][0],l_sensorconfig.xyz_axis[2][1]);
	printk("gsensor interrupt:gpio%d ******************\n", l_sensorconfig.int_gpio);
	return 0;
}

static int __init mma7660_init(void)
{
    int ret;

	get_axisset(NULL); // get gsensor config from u-boot
    ret = platform_device_register(&mma7660_device);
    if(ret)
        return ret;

	printk(KERN_INFO "mma7660A compass driver: init\n");
	return platform_driver_register(&mma7660_driver);
}

static void __exit mma7660_exit(void)
{
	if (l_sensorconfig.sensor_proc != NULL)
	{
		remove_proc_entry(GSENSOR_PROC_NAME, NULL);
		l_sensorconfig.sensor_proc = NULL;
	}
    platform_driver_unregister(&mma7660_driver);
    platform_device_unregister(&mma7660_device);
}

module_init(mma7660_init);
module_exit(mma7660_exit);
MODULE_LICENSE("GPL");
