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


/* Header */

#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>

/* Define */
#ifndef BIT0
#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000
#endif

#define VDMA_REG_BASE		((void *) 0xd8001c00)
#define MAX_VDMA_CHANNEL	8
#define VDMA_DEBUG		0
#define VDMA_MAJOR		236
#define MAX_VDMA_MINORS		1

#define VDMA_MAGIC			0x69
#define VDMA_IOCTL_REQ_VDMA		_IO(VDMA_MAGIC, 0)
#define VDMA_IOCTL_SET_VDMA_SOURCE	_IOW(VDMA_MAGIC, 1, void *)
#define VDMA_IOCTL_SET_VDMA_DESTINATION	_IOW(VDMA_MAGIC, 2, void *)
#define VDMA_IOCTL_SET_VDMA_REGION	_IOW(VDMA_MAGIC, 3, void *)
#define VDMA_IOCTL_ENABLE_VDMA		_IO(VDMA_MAGIC, 4)
#define VDMA_IOCTL_FREE_VDMA		_IO(VDMA_MAGIC, 5)
#define VDMA_IOCTL_VDMA_TEST		_IO(VDMA_MAGIC, 9)

/* Macro */

#if (VDMA_DEBUG)
#define T() printk(KERN_DEBUG "Trace %s\n", __func__);
#else
#define T() do { } while (0);
#endif

/* Structure */

struct _vdma_regs {
	unsigned int reserved[16];
	unsigned int dma_gcr;
	unsigned int reserved1;
	unsigned int dma_ier;
	unsigned int dma_isr;
	unsigned int rdp_des_adr;
	unsigned int rdp_init_skip_bcnt;
	unsigned int rdp_line_skip_bcnt;
	unsigned int rdp_pic_line_bcnt;
	unsigned int pic_h;
	unsigned int reserved2[3];
	unsigned int ps_start_addr;
	unsigned int ps_init_skip_bcnt;
	unsigned int ps_line_skip_bcnt;
	unsigned int ps_pic_line_bcnt;
	unsigned int dma_ccr;
	unsigned int des_fix_bcnt;
	unsigned int reserved3[2];
};

struct _vdma_desc {
	unsigned short req_count;
	unsigned short int_en : 1;
	unsigned short reserved : 14;
	unsigned short end : 1;
	unsigned int data_addr;
} __attribute__((packed));

struct _reg_dma_ccr {
	unsigned int event_code	: 4;
	unsigned int p0_cpl : 1;
	unsigned int reserved : 1;
	unsigned int trans_cpl : 1;
	unsigned int run : 1;
	unsigned int reserved1 : 4;
	unsigned int prot : 4;
	unsigned int trans_cpl_int_en : 1;
	unsigned int reserved2 : 6;
	unsigned int ps_ori : 1;
	unsigned int ps_rgb_format : 1;
	unsigned int reserved3 : 3;
	unsigned int rdp_rgb_format : 2;
	unsigned int reserved4 : 2;
} __attribute__((packed));

struct _vdma_channel {
	struct _vdma_desc *desc;
	unsigned int flags;
	unsigned int src_addr;
	unsigned int src_x;
	unsigned int src_y;
	unsigned int src_w;
	unsigned int src_h;
	unsigned int src_bpp;
	unsigned int dst_addr;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int dst_w;
	unsigned int dst_h;
	unsigned int dst_bpp;
	unsigned int pic_w;
	unsigned int pic_h;
	unsigned int busy;
	unsigned int chunksize;
};

/* Declare functions */

void free_vdma(unsigned int channel);
int request_vdma(unsigned int channel, const char *name);
void set_vdma_source(int channel, unsigned int addr,
		     int w, int h, int bpp);
void set_vdma_destination(int channel, unsigned int addr,
			  int w, int h, int bpp);
void set_vdma_region(int channel, int src_x, int src_y,
		     int dst_x, int dst_y, int pic_w, int pic_h);
int enable_vdma(unsigned int channel);
static int wait_vdma(unsigned int channel);
static int reset_vdma(unsigned int channel);

static int vdma_open(struct inode *inode, struct file *filp);
static int vdma_release(struct inode *inode, struct file *filp);
static int vdma_ioctl(struct inode *inode, struct file *filp,
		      unsigned int command, unsigned long arg);

static int vdma_test(int channel);

/* Const */

const unsigned int use_vdma_irq			= 1;
const unsigned int vdma_max_req_count		= 32768;
const unsigned int vdma_max_req_count_mask	= 0xffff;
const unsigned int vdma_irq			= 9; /* VDMA_IRQ */

/* Global variable */

static DEFINE_MUTEX(vdma_mutex);
static DECLARE_WAIT_QUEUE_HEAD(vdma_wq);
static struct _vdma_regs *const vdma_regs = VDMA_REG_BASE;;
static struct _vdma_channel vdma_channel[MAX_VDMA_CHANNEL];
static unsigned int reg_save_dma_gcr;
static unsigned int reg_save_dma_isr;
static unsigned int reg_save_dma_ier;
static unsigned int reg_save_dma_ccr;

static const struct file_operations vdma_fops = {
	.open		= vdma_open,
	.release	= vdma_release,
	.ioctl		= vdma_ioctl,
	.owner		= THIS_MODULE,
};

static struct cdev vdma_cdev = {
	.owner = THIS_MODULE,
};

/* Source */

static inline void vdma_cache_sync(void)
{
	/*
	 * MRC{cond} p<cpnum>, <op1>, Rd, CRn, CRm, <op2>
	 */
	__asm__ __volatile__ (
	"1:	mrc p15, 0, r15, c7, c14, 3 \n\t"
	"	bne 1b"
	);
}

static void set_vdma_desc_method_1(unsigned int channel)
{
	struct _vdma_channel *ch;
	struct _vdma_desc *desc;
	unsigned int totlen;
	unsigned int max_req_count;
	unsigned int req_count;
	unsigned int data_addr;
	unsigned int src_pixel_width;

	ch = &vdma_channel[channel];
	desc = (struct _vdma_desc *) ch->desc;
	src_pixel_width = ch->src_bpp >> 3;

	totlen = ch->src_w * ch->src_h * src_pixel_width;

	max_req_count = vdma_max_req_count;
	data_addr = ch->src_addr;

	while (totlen) {
		req_count = (totlen < max_req_count) ? totlen : max_req_count;
		desc->end = (req_count == totlen) ? 1 : 0;
		desc->int_en = (req_count == totlen) ? 1 : 0;
		desc->req_count = 0;
		desc->data_addr = data_addr;
		totlen -= req_count;
		data_addr += req_count;
		desc++;
	}
}

static void set_vdma_desc_method_2(unsigned int channel)
{
	struct _vdma_channel *ch;
	struct _vdma_desc *desc;
	unsigned int totlen;
	unsigned int max_req_count;
	unsigned int req_count;
	unsigned int data_addr;
	unsigned int src_pixel_width;

	ch = &vdma_channel[channel];
	desc = (struct _vdma_desc *) ch->desc;
	src_pixel_width = ch->src_bpp >> 3;

	totlen = ch->src_w * ch->src_h * src_pixel_width;

	max_req_count = vdma_max_req_count & vdma_max_req_count_mask;
	data_addr = ch->src_addr;

	while (totlen) {
		req_count = (totlen < max_req_count) ? totlen : max_req_count;
		desc->end = (req_count == totlen) ? 1 : 0;
		desc->int_en = (req_count == totlen) ? 1 : 0;
		desc->req_count = req_count;
		desc->data_addr = data_addr;
		totlen -= req_count;
		data_addr += req_count;
		desc++;
	}
}

static inline int is_1k_aligned(int chunksize)
{
	return !(chunksize & 0x3ff);
}

static void set_vdma_desc(unsigned int channel, int chunksize)
{
	struct _vdma_channel *ch;

	ch = &vdma_channel[channel];
	ch->chunksize = chunksize;

	if (is_1k_aligned(chunksize))
		set_vdma_desc_method_1(channel);
	else
		set_vdma_desc_method_2(channel);
}

static void dump_vdma_desc(unsigned int channel)
{
	struct _vdma_channel *ch;
	struct _vdma_desc *desc;
	unsigned int max_desc_num;

	ch = &vdma_channel[channel];
	desc = (struct _vdma_desc *) ch->desc;
	max_desc_num = PAGE_SIZE >>  3;

	while (max_desc_num) {
		printk(KERN_DEBUG "%d, %d, 0x%x, 0x%x\n",
			desc->end, desc->int_en,
			desc->req_count, desc->data_addr);
		if (desc->end)
			break;
		desc++;
		max_desc_num--;
	}
}

static irqreturn_t vdma_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	const unsigned int isr_debug = 0;
	const unsigned int event_code_mask = 0xf;
	unsigned int event_code;

	reg_save_dma_gcr = vdma_regs->dma_gcr;
	reg_save_dma_ier = vdma_regs->dma_ier;
	reg_save_dma_isr = vdma_regs->dma_isr;
	reg_save_dma_ccr = vdma_regs->dma_ccr;

	event_code = vdma_regs->dma_ccr & event_code_mask;

	if (event_code != 0xf)
		vdma_regs->dma_gcr = BIT8;

	T();

	vdma_regs->dma_gcr = BIT0;
	vdma_regs->dma_ier = BIT0;
	vdma_regs->dma_isr = BIT0;

	if (isr_debug) {
		printk(KERN_DEBUG "%s: vdma_dma_isr = 0x%x\n",
			__func__, vdma_regs->dma_isr);
		printk(KERN_DEBUG "%s: vdma_dma_ier = 0x%x\n",
			__func__, vdma_regs->dma_ier);
		printk(KERN_DEBUG "%s: vdma_dma_ccr = 0x%x\n",
			__func__, vdma_regs->dma_ccr);
	}

	wake_up_interruptible(&vdma_wq);

	return IRQ_HANDLED;
}

static int vdma_open(struct inode *inode, struct file *filp)
{
	mutex_lock(&vdma_mutex);

	T();

	mutex_unlock(&vdma_mutex);

	return 0;
}

static int vdma_release(struct inode *inode, struct file *filp)
{

	mutex_lock(&vdma_mutex);

	T();

	mutex_unlock(&vdma_mutex);

	return 0;
}

static int get_args(unsigned int *to, void *from, int num)
{
	unsigned int count;

	count = sizeof(unsigned int);
	count *= num;

	if (copy_from_user(to, from, count)) {
		printk(KERN_ERR "%s: copy_from_user failure\n", __func__);
		return  -EFAULT;
	}

	return 0;
}

static int vdma_ioctl(struct inode *inode, struct file *filp,
	unsigned int command, unsigned long arg)
{
	unsigned int args[8];
	int ret;

	mutex_lock(&vdma_mutex);

	T();

	ret = 0;

	switch (command) {
	case VDMA_IOCTL_REQ_VDMA:
		ret = request_vdma((int) arg, 0);
		break;
	case VDMA_IOCTL_SET_VDMA_SOURCE:
		ret = get_args(args, (void *) arg, 5);
		if (ret == 0) {
			set_vdma_source(args[0], args[1], args[2], args[3],
					args[4]);
		}
		break;
	case VDMA_IOCTL_SET_VDMA_DESTINATION:
		ret = get_args(args, (void *) arg, 5);
		if (ret == 0) {
			set_vdma_destination(args[0], args[1], args[2],
					     args[3], args[4]);
		}
		break;
	case VDMA_IOCTL_SET_VDMA_REGION:
		ret = get_args(args, (void *) arg, 7);
		if (ret == 0) {
			set_vdma_region(args[0], args[1], args[2], args[3],
					args[4], args[5], args[6]);
		}
		break;
	case VDMA_IOCTL_ENABLE_VDMA:
		ret = enable_vdma((int) arg);
		break;
	case VDMA_IOCTL_FREE_VDMA:
		free_vdma((int) arg);
		break;
	case VDMA_IOCTL_VDMA_TEST:
		ret = vdma_test((int) arg);
		break;
	default:
		printk(KERN_ERR "%s: invalid vdma command (0x%08lx)\n",
			__func__, arg);
		return -EINVAL;
	}

	mutex_unlock(&vdma_mutex);

	return 0;
}

static int init_vdma(void)
{
	int i;
	int err;
	dev_t dev;

	memset(vdma_channel, 0, sizeof(vdma_channel));

	for (i = 0; i < MAX_VDMA_CHANNEL; i++) {
		vdma_channel[i].desc =
			(void *) __get_free_page(GFP_KERNEL | GFP_DMA);
	}

	err = request_irq(vdma_irq, vdma_interrupt, IRQF_SHARED,
			  "vdma", (void *) vdma_regs);
	if (err) {
		printk(KERN_ERR "%s: request_irq = %d\n", __func__, err);
		return -EFAULT;
	}

	dev = MKDEV(VDMA_MAJOR, vdma_irq);

	err = register_chrdev_region(dev, MAX_VDMA_MINORS, "vdma");
	
	if (err) {
		printk(KERN_ERR "%s: register_chrdev_region = %d\n",
			__func__, err);
		return -EFAULT;
	}

	cdev_init(&vdma_cdev, &vdma_fops);

	err = cdev_add(&vdma_cdev, dev, 1);
	if (err) {
		printk(KERN_ERR "%s: cdev_add = %d\n", __func__, err);
		unregister_chrdev_region(dev, MAX_VDMA_MINORS);
		return -EFAULT;
	}
	
	printk(KERN_DEBUG "%s: major = %d, minor = %d, irq = %d\n",
		__func__, VDMA_MAJOR, vdma_irq, vdma_irq);

	
	static struct class *fc;	
	fc=class_create(THIS_MODULE, "vdma");
	
	device_create(fc, NULL, dev, NULL, "%s","vdma");
	

#if 0
	err = do_mknod("/dev/vdma", S_IFCHR, VDMA_MAJOR, 0);
	if (err) {
		printk(KERN_ERR "%s: register_node /dev/vdma = %d\n",
			__func__, err);
		return -EFAULT;
	}
	
#endif
	return 0;
}

static int exit_vdma(void)
{
	int i;
	unsigned long p;
	dev_t dev;

	dev = MKDEV(VDMA_MAJOR, vdma_irq);
	unregister_chrdev_region(dev, MAX_VDMA_MINORS);
	cdev_del(&vdma_cdev);

	free_irq(vdma_irq, (void *) vdma_regs);

	for (i = 0; i < MAX_VDMA_CHANNEL; i++) {
		p = (unsigned long) vdma_channel[i].desc;
		if (p)
			free_page(p);
	}

/*
	device_destroy(fc,first);
	class_destroy(fc);
*/	
	return 0;
}

/**
 * request_vdma - request a vdma channel
 * @channel:	video dma channel id from 0 to 7
 * @name:	a name used for debug
 *
 * Request a vdma channel before using it.
 */
int request_vdma(unsigned int channel, const char *name)
{
	if (channel >= MAX_VDMA_CHANNEL)
		return -EINVAL;

	if (vdma_channel[channel].busy)
		return -EBUSY;

	vdma_channel[channel].busy = 1;

	return 0;
}

/**
 * free_vdma - free a vdma channel
 * @channel:	channel id to be freed
 *
 * Free a vdma channel by its id.
 */
void free_vdma(unsigned int channel)
{
	vdma_channel[channel].busy = 0;
}

/**
 * set_vdma_source - assign a source to vdma channel
 * @channel:	channel id to be freed
 * @addr:	physical memory address of source surface
 * @w:		pixel width of source surface
 * @h:		pixel height of source surface
 * @bpp:	bits per pixel of source surface
 *
 * Assign a source to the vdma channel. VDMA descriptors
 * will be updated by set_vdma_desc to a 4k memory page.
 */
void set_vdma_source(int channel, unsigned int addr,
	int w, int h, int bpp)
{
	struct _vdma_channel *ch;

	ch = &vdma_channel[channel];
	ch->src_addr = addr;
	ch->src_w = w;
	ch->src_h = h;
	ch->src_bpp = bpp;

	set_vdma_desc(channel, vdma_max_req_count);
}

/**
 * set_vdma_destination - assign a destination to vdma channel
 * @channel:	channel id to be freed
 * @addr:	physical memory address of destination surface
 * @w:		pixel width of destination surface
 * @h:		pixel height of destination surface
 * @bpp:	bits per pixel of destination surface
 *
 * Assign a destination to the vdma channel.
 */
void set_vdma_destination(int channel, unsigned int addr,
	int w, int h, int bpp)
{
	struct _vdma_channel *ch;

	ch = &vdma_channel[channel];
	ch->dst_addr = addr;
	ch->dst_w = w;
	ch->dst_h = h;
	ch->dst_bpp = bpp;
}

/**
 * set_vdma_region - set the region to be transfered.
 * @channel:	channel id to be freed
 * @src_x:	x-coordinate of source surface
 * @src_y:	y-coordinate of source surface
 * @dst_x:	x-coordinate of destination surface
 * @dst_y:	y-coordinate of destination surface
 * @pic_w:	pixel width
 * @pic_h:	pixel height
 *
 * Set VDMA region by coordinates and size of involved surfaces.
 */
void set_vdma_region(int channel, int src_x, int src_y,
	int dst_x, int dst_y, int pic_w, int pic_h)
{
	struct _vdma_channel *ch;

	T();
	ch = &vdma_channel[channel];
	ch->src_x = src_x;
	ch->src_y = src_y;
	ch->dst_x = dst_x;
	ch->dst_y = dst_y;
	ch->pic_w = pic_w;
	ch->pic_h = pic_h;
}

static int reset_vdma(unsigned int channel)
{
	T();

	vdma_regs->dma_gcr = BIT8;
	wmb();

	return 0;
}

static int simple_log2(unsigned int n)
{
	unsigned int i;
	unsigned int val;

	if (n == 0)
		return -1;

	i = 0;
	val = 1;

	while (val < n) {
		i++;
		val <<= 1;
	}

	if (val > n)
		i--;

	return i;
}

static int do_vdma(unsigned int channel)
{
	struct _vdma_channel *ch;
	unsigned int src_pixel_width;
	unsigned int dst_pixel_width;
	struct _reg_dma_ccr *reg_dma_ccr;

	if (channel > MAX_VDMA_CHANNEL) {
		printk(KERN_ERR "%s: invalid vdma channel %d\n",
			__func__, channel);
		return -EINVAL;
	}

	ch = &vdma_channel[channel];
	src_pixel_width = ch->src_bpp >> 3;
	dst_pixel_width = ch->dst_bpp >> 3;

	T();

	if (reset_vdma(channel) < 0)
		return -1;

	vdma_regs->dma_gcr	= BIT0;
	vdma_regs->dma_ier	= 0;
	vdma_regs->dma_isr	= BIT0;
	wmb();

	vdma_regs->rdp_des_adr	= __pa(ch->desc);

	vdma_regs->rdp_init_skip_bcnt =
		(ch->src_y * ch->src_w + ch->src_x) * src_pixel_width;

	vdma_regs->rdp_line_skip_bcnt =
		(ch->src_w - ch->pic_w) * src_pixel_width;

	vdma_regs->rdp_pic_line_bcnt	= ch->pic_w * src_pixel_width;
	vdma_regs->pic_h		= ch->pic_h;
	vdma_regs->ps_start_addr	= ch->dst_addr;

	vdma_regs->ps_init_skip_bcnt =
		(ch->dst_y * ch->dst_w + ch->dst_x) * dst_pixel_width;

	vdma_regs->ps_line_skip_bcnt =
		(ch->dst_w - ch->pic_w) * dst_pixel_width;

	vdma_regs->ps_pic_line_bcnt	= ch->pic_w * dst_pixel_width;
	if (is_1k_aligned(ch->chunksize))
		vdma_regs->des_fix_bcnt	= simple_log2(ch->chunksize) - 9;
	else
		vdma_regs->des_fix_bcnt	= 0;

	reg_dma_ccr = (struct _reg_dma_ccr *) &vdma_regs->dma_ccr;

	/* 0: 16 bpp, 1: 24 bpp, 2: 32 bpp */
	reg_dma_ccr->rdp_rgb_format = src_pixel_width - 2;

	/* 0: 16 bpp, 1: 32 bpp */
	reg_dma_ccr->ps_rgb_format = dst_pixel_width >> 2;

	reg_dma_ccr->ps_ori = 0;
	reg_dma_ccr->trans_cpl_int_en = 0;
	reg_dma_ccr->prot = 0;
	reg_dma_ccr->trans_cpl = 1;

	if (use_vdma_irq) {
		vdma_regs->dma_ier = 1;
		reg_dma_ccr->trans_cpl_int_en = 1;
	}

	wmb();
	vdma_cache_sync();
	reg_dma_ccr->run = 1;

	return wait_vdma(channel);
}

/**
 * enable_vdma - start vdma transfer.
 * @channel:	channel id to be freed
 *
 * Set VDMA registers then start VDMA transfer from source surface
 * to the destination of an assigned region. If any VDMA error occurs,
 * then it will try again for 4 times at most.
 */
int enable_vdma(unsigned int channel)
{
	int t;

	T();

	t = 4;

	while (t && do_vdma(channel) < 0) {
		msleep_interruptible(10);
		t--;
	}

	if (t == 0)
		printk(KERN_ERR "%s: error, retry counts = %d\n",
			__func__, 4 - t);

	return (t != 0) ? 0 : -EFAULT;
}

static int wait_vdma(unsigned int channel)
{
	const unsigned int event_code_mask = 0xf;
	unsigned int event_code;
	struct _reg_dma_ccr *reg_dma_ccr;
	int ret;

	T();

	reg_dma_ccr = (struct _reg_dma_ccr *) &vdma_regs->dma_ccr;

	rmb();

	wait_event_interruptible(vdma_wq, reg_save_dma_isr);

	event_code = reg_save_dma_ccr & event_code_mask;

	if (event_code != 0xf) {
		printk(KERN_ERR "%s: vdma error. event_code = 0x%x\n",
			__func__, event_code);
		ret = -EFAULT;
	} else {
		ret = 0;
	}

	reg_save_dma_gcr = vdma_regs->dma_gcr;
	reg_save_dma_ier = vdma_regs->dma_ier;
	reg_save_dma_isr = vdma_regs->dma_isr;
	reg_save_dma_ccr = vdma_regs->dma_ccr;

	return ret;
}

static int self_test(int channel, unsigned int addr,
	int w, int h, int bpp, int item)
{
	const int function_debug = 0;
	int i;
	int ret = 0;
	int ch = channel;

	int src_w = w;
	int src_h = h >> 1;
	int dst_w = src_w;
	int dst_h = src_h;
	int pic_w = src_w;
	int pic_h = src_h - 10;
	int src_x = 0;
	int src_y = 0;
	int dst_x = 0;
	int dst_y = 0;
	unsigned int src_addr;
	unsigned int dst_addr;

	T();

	if (item == 0) {
		src_addr = addr;
		dst_addr = addr + ((w * h * bpp) >> 3);
	} else {
		dst_addr = addr;
		src_addr = addr + ((w * h * bpp) >> 3);
	}

	if (request_vdma(ch, 0) == 0) {

		set_vdma_source(ch, src_addr, src_w, src_h, bpp);

		if (function_debug)
			dump_vdma_desc(ch);

		set_vdma_destination(ch, dst_addr, dst_w, dst_h, bpp);

		i = 8;
		while (i) {
			set_vdma_region(ch, src_x, src_y, dst_x, dst_y,
					pic_w - dst_x, pic_h);
			ret = enable_vdma(ch);
			i--;
		}
	}

	free_vdma(ch);

	return ret;
}

static int vdma_test(int channel)
{
	unsigned int addr	= 0x7800000;
	unsigned int w		= 1024;
	unsigned int h		= 768;
	unsigned int bpp	= 32;
	unsigned int item	= 0;

	T();

	return self_test(channel, addr, w, h, bpp, item);
}

static int __init vdma_module_init(void)
{
	init_vdma();

	return 0;
}

static void __exit vdma_module_exit(void)
{
	exit_vdma();
}

#include <linux/module.h>

module_init(vdma_module_init);
module_exit(vdma_module_exit);

#define DRIVER_AUTHOR "Vincent Chen <vincentchen@wondermedia.com.tw>"
#define DRIVER_DESC   "WM8510 video DMA driver"

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

