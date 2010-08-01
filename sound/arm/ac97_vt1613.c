/*++
	sound/arm/ac97_vt1613.c

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

	History:
		The code was inherit from vt8430
		2009/02/24 First Version
--*/
#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#include <mach/hardware.h>

#include <mach/ac97.h>
#include <mach/vt1613.h>


/*
 *  Debug macros
 */
//#define DEBUG
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

/*
 * Remove this to static in the future
 */
#define CONFIG_WMT_AC97_DEBUG        1

#define	DRIVER_VERSION          "0.72"

#define CODEC_FAIL              (-1)
#define CODEC_MASK              (0x0000FFFF)

void ac97_init(void);
void ac97_exit(void);

int codec_read(u16 addr, u16 *data);
int codec_write(u16 addr, u16 data);

static int vt1613_ref;
EXPORT_SYMBOL(vt1613_ref);
static struct codec_s *codec;

/*
 * wait queue head is using for waiting CODEC ready, CODEC read done,
 * and CODEC write done interrups.
 */
static DECLARE_WAIT_QUEUE_HEAD(codec_wq);

/*
 * Change this to static in the future
 */
ac97_t ac97 = {
	(struct ac97_regs_s *)(io_p2v(__AC97_BASE)),
	/* interrupt counters */
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	/* irq number */
	IRQ_AC97,
	/* reference counter */
	0,
	ac97_init,
	ac97_exit,
};

/*
 * Export debug module for AC'97 audio driver.
 */
EXPORT_SYMBOL(ac97);

// mutex for read and write codec
static DECLARE_MUTEX(l_sem_wrrdcodec);
#ifdef CONFIG_WMT_AC97_DEBUG

/* vt8430_ac97_interrupt()
 *
 * It's only interrupt counter now, might be useful to
 * debug or benchmark.
 * The CODEC I/O is asynchronous now, enable interrupt bits
 * in routine and then sleep, interrupts here will detect and
 * turn them off, Apr.18.2005 by Harry.
 */
static irqreturn_t
wmt_ac97_interrupt(int irq, void *dev_id)
{
	unsigned int fifo;
	unsigned int global;

	/*
	 * Read and clear global interrupt sources.
	 */
	global = ac97.regs->ACGS;
	ac97.regs->ACGS = global & ACGS_W1CMASK;

	/*
	 * Carefully detect each toggled global interrupt and then mask it.
	 */
	if (global & ACGS_CRDY) {             /* CODEC Ready */
		ac97.ints.crdy++;
		ac97.regs->ACGC &= ~ACGC_CRIE;
	}
	if (global & ACGS_CWD) {              /* CODEC Write Done */
		ac97.ints.cwd++;
		ac97.regs->ACGC &= ~ACGC_CWDIE;
	}
	if (global & ACGS_CRD) {              /* CODEC Read Done */
		ac97.ints.crd++;
		ac97.regs->ACGC &= ~ACGC_CRDIE;
	}
	if (global & ACGS_CST) {              /* CODEC Status Timeout */
		ac97.ints.cst++;
		ac97.regs->ACGC &= ~ACGC_CRDIE;
	}

	if (global & ACGS_MFIA) {             /* Mic FIFO Interrupt Active */
		/*
		 * Read and clear MIFS interrupt sources
		 */
		fifo = ac97.regs->MIFS;
		ac97.regs->MIFS = fifo & MIFS_W1CMASK;

		/*
		 * Carefully detect each toggled mic fifo
		 * interrupts and then mask it.
		 */
		if (fifo & MIFS_MFF) {        /* Mic FIFO Full */
			ac97.ints.mff++;
			ac97.regs->MIFC &= ~MIFC_MFFIE;
		}
		if (fifo & MIFS_MFA) {        /* Mic FIFO Almost Full,
						 * take care MFADE
						 */
			ac97.ints.mfa++;
			ac97.regs->MIFC &= ~MIFC_MFAIE;
		}
		if (fifo & MIFS_MFUE) {       /* Mic FIFO Underrun Error */
			ac97.ints.mfue++;
			ac97.regs->MIFC &= ~MIFC_MFUIE;
		}
		if (fifo & MIFS_MFOE) {       /* Mic FIFO Overrun Error */
			ac97.ints.mfoe++;
			ac97.regs->MIFC &= ~MIFC_MFOIE;
		}
	}

	if (global & ACGS_PRFIA) {            /* PCM Rx FIFO Interrupt Active */
		/*
		 * Read and clear PRFS interrupt sources.
		 */
		fifo = ac97.regs->PRFS;
		ac97.regs->PRFS = fifo & PRFS_W1CMASK;

		/*
		 * Carefully detect each toggled mic fifo
		 * interrupts and then mask it.
		 */
		if (fifo & PRFS_PRFF) {       /* PCM Rx FIFO Full */
			ac97.ints.prff++;
			ac97.regs->PRFC &= ~PRFC_PRFFIE;
		}
		if (fifo & PRFS_PRFA) {       /* PCM Rx FIFO Almost Full,
						 * take care PRFADE.
						 */
			ac97.ints.prfa++;
			ac97.regs->PRFC &= ~PRFC_PRFAIE;
		}
		if (fifo & PRFS_PRFUE) {      /* PCM Rx FIFO Underrun Error */
			ac97.ints.prfue++;
			ac97.regs->PRFC &= ~PRFC_PRFUIE;
		}
		if (fifo & PRFS_PRFOE) {      /* PCM Rx FIFO Overrun Error */
			ac97.ints.prfoe++;
			ac97.regs->PRFC &= ~PRFC_PRFOIE;
		}
	}

	/*
	 * Carefully detect each toggled tx fifo interrupts and make it.
	 */
	if (global & ACGS_PTFIA) {            /* PCM Tx FIFO Interrupt Active */
		/*
		 * Read and clear PTFS interrupt sources
		 */
		fifo = ac97.regs->PTFS;
		ac97.regs->PTFS = fifo & PTFS_W1CMASK;

		if (fifo & PTFS_PTFE) {       /* PCM Tx FIFO Empty */
			ac97.ints.ptfe++;
			ac97.regs->PTFC &= ~PTFC_PTFEIE;
		}
		if (fifo & PTFS_PTFA) {       /* PCM Tx FIFO Almost Empty,
						 * take care PTFADE.
						 */
			ac97.ints.ptfa++;
			ac97.regs->PTFC &= ~PTFC_PTFAIE;
		}
		if (fifo & PTFS_PTFUE) {      /* PCM Tx FIFO Underrun Error */
			ac97.ints.ptfue++;
			ac97.regs->PTFC &= ~PTFC_PTFUIE;
		}
		if (fifo & PTFS_PTFOE) {      /* PCM Tx FIFO Overrun Error */
			ac97.ints.ptfoe++;
			ac97.regs->PTFC &= ~PTFC_PTFUIE;
		}
	}
	wake_up_interruptible(&codec_wq);
	return IRQ_HANDLED;
}

#ifdef CONFIG_PROC_FS
#if 0
static struct proc_dir_entry *proc_ac97;

/* ac97_irq_read()
 *
 * Entry for reading AC'97 controller status
 *
 * We created a node in /proc/driver/ac97/interrupts
 * Using "cat /proc/driver/ac97/interrupts" for debugging.
 *
 * TODO: migrate to sysfs in the future.
 */
static int ac97_irq_read(char *buf, char **start, off_t offset, int len)
{
	char *p = buf;

	p += sprintf(p, "Global Status\n");
	p += sprintf(p, "CRDY \t: %d\n", ac97.ints.crdy);
	p += sprintf(p, "CWD  \t: %d\n", ac97.ints.cwd);
	p += sprintf(p, "CRD  \t: %d\n", ac97.ints.crd);
	p += sprintf(p, "CST  \t: %d\n", ac97.ints.cst);

	p += sprintf(p, "PCM Tx FIFO Status\n");
	p += sprintf(p, "PTFE \t: %d\n", ac97.ints.ptfe);
	p += sprintf(p, "PTFA \t: %d\n", ac97.ints.ptfa);
	p += sprintf(p, "PTFUE\t: %d\n", ac97.ints.ptfue);
	p += sprintf(p, "PTFOE\t: %d\n", ac97.ints.ptfoe);

	p += sprintf(p, "PCM Rx FIFO Status\n");
	p += sprintf(p, "PRFF \t: %d\n", ac97.ints.prff);
	p += sprintf(p, "PRFA \t: %d\n", ac97.ints.prfa);
	p += sprintf(p, "PRFUE\t: %d\n", ac97.ints.prfue);
	p += sprintf(p, "PRFOE\t: %d\n", ac97.ints.prfoe);

	p += sprintf(p, "Mic FIFO Status\n");
	p += sprintf(p, "MFF  \t: %d\n", ac97.ints.mff);
	p += sprintf(p, "MFA  \t: %d\n", ac97.ints.mfa);
	p += sprintf(p, "MFUE \t: %d\n", ac97.ints.mfue);
	p += sprintf(p, "MFOE \t: %d\n", ac97.ints.mfoe);

	return p - buf;
}

/* ac97_reg_read()
 *
 * Entry for reading AC'97 controller status
 *
 * We created a node in /proc/driver/ac97/register
 * Using "cat /proc/driver/ac97/register" for debugging.
 *
 * TODO: migrate to sysfs in the future.
 */
static int ac97_reg_read(char *buf, char **start, off_t offset, int len)
{
	char *p = buf;

	p += sprintf(p, "reg : value\n");
	p += sprintf(p, "ACGC: 0x%.8x\n", ac97.regs->ACGC);
	p += sprintf(p, "ACGS: 0x%.8x\n", ac97.regs->ACGS);
	p += sprintf(p, "CCSL: 0x%.8x\n", ac97.regs->CCSL);
	p += sprintf(p, "CCMD: 0x%.8x\n", ac97.regs->CCMD);
	p += sprintf(p, "CRSD: 0x%.8x\n", ac97.regs->CRSD);
	p += sprintf(p, "PTFC: 0x%.8x\n", ac97.regs->PTFC);
	p += sprintf(p, "PTFS: 0x%.8x\n", ac97.regs->PTFS);
	p += sprintf(p, "PRFC: 0x%.8x\n", ac97.regs->PRFC);
	p += sprintf(p, "PRFS: 0x%.8x\n", ac97.regs->PRFS);
	p += sprintf(p, "MIFC: 0x%.8x\n", ac97.regs->MIFC);
	p += sprintf(p, "MIFS: 0x%.8x\n", ac97.regs->MIFS);

	return p - buf;
}
#endif
#endif  /* CONFIG_PROC_FS */

void ac97_reg_dump(void)
{
/*
	DPRINTK("ACGC: 0x%.8x\n", ac97.regs->ACGC);
	DPRINTK("ACGS: 0x%.8x\n", ac97.regs->ACGS);
	DPRINTK("CCSL: 0x%.8x\n", ac97.regs->CCSL);
	DPRINTK("CCMD: 0x%.8x\n", ac97.regs->CCMD);
	DPRINTK("CRSD: 0x%.8x\n", ac97.regs->CRSD);
	DPRINTK("PTFC: 0x%.8x\n", ac97.regs->PTFC);
	DPRINTK("PTFS: 0x%.8x\n", ac97.regs->PTFS);
	DPRINTK("PRFC: 0x%.8x\n", ac97.regs->PRFC);
	DPRINTK("PRFS: 0x%.8x\n", ac97.regs->PRFS);
	DPRINTK("MIFC: 0x%.8x\n", ac97.regs->MIFC);
	DPRINTK("MIFS: 0x%.8x\n", ac97.regs->MIFS);
*/
	printk("ACGC: 0x%.8x\n", ac97.regs->ACGC);
	printk("ACGS: 0x%.8x\n", ac97.regs->ACGS);
	printk("CCSL: 0x%.8x\n", ac97.regs->CCSL);
	printk("CCMD: 0x%.8x\n", ac97.regs->CCMD);
	printk("CRSD: 0x%.8x\n", ac97.regs->CRSD);
	printk("PTFC: 0x%.8x\n", ac97.regs->PTFC);
	printk("PTFS: 0x%.8x\n", ac97.regs->PTFS);
	printk("PRFC: 0x%.8x\n", ac97.regs->PRFC);
	printk("PRFS: 0x%.8x\n", ac97.regs->PRFS);
	printk("MIFC: 0x%.8x\n", ac97.regs->MIFC);
	printk("MIFS: 0x%.8x\n", ac97.regs->MIFS);

}
#endif  /* CONFIG_WMT_AC97_DEBUG */

/*zhf-ac97 struct ac97_regs_s *pbackup_ac97_reg; */
static struct ac97_regs_s pbackup_ac97_reg;
void ac97_reg_backup(void)
{
	REG32_VAL(0xD8130250) |= 0x00080000;	/*Bit19  enable ac97 clk*/
	udelay(2);      //zhf-ac97
/*zhf-ac97
        if (pbackup_ac97_reg == NULL)
                pbackup_ac97_reg = kmalloc(sizeof(struct ac97_regs_s), GFP_KERNEL);

        pbackup_ac97_reg->ACGC = ac97.regs->ACGC;
        pbackup_ac97_reg->ACGS = ac97.regs->ACGS;
        pbackup_ac97_reg->CCSL = ac97.regs->CCSL;
        pbackup_ac97_reg->CCMD = ac97.regs->CCMD;
        pbackup_ac97_reg->CRSD = ac97.regs->CRSD;
        pbackup_ac97_reg->PTFC = ac97.regs->PTFC;
        pbackup_ac97_reg->PTFS = ac97.regs->PTFS;
        pbackup_ac97_reg->PRFC = ac97.regs->PRFC;
        pbackup_ac97_reg->PRFS = ac97.regs->PRFS;
        pbackup_ac97_reg->MIFC = ac97.regs->MIFC;
        pbackup_ac97_reg->MIFS = ac97.regs->MIFS;
zhf-ac97*/
        pbackup_ac97_reg.ACGC = ac97.regs->ACGC;
        pbackup_ac97_reg.ACGS = ac97.regs->ACGS;
        pbackup_ac97_reg.CCSL = ac97.regs->CCSL;
        pbackup_ac97_reg.CCMD = ac97.regs->CCMD;
        pbackup_ac97_reg.CRSD = ac97.regs->CRSD;
        pbackup_ac97_reg.PTFC = ac97.regs->PTFC;
        pbackup_ac97_reg.PTFS = ac97.regs->PTFS;
        pbackup_ac97_reg.PRFC = ac97.regs->PRFC;
        pbackup_ac97_reg.PRFS = ac97.regs->PRFS;
        pbackup_ac97_reg.MIFC = ac97.regs->MIFC;
        pbackup_ac97_reg.MIFS = ac97.regs->MIFS;

}
EXPORT_SYMBOL(ac97_reg_backup);

void ac97_reg_restore(void)
{
	REG32_VAL(0xD8130050) |= 0x00080000;	/*Bit19  enable ac97 clk*/
						/*BA_PMC = 0xD8130000
						Power Management Control Base Address*/

	/*
	 * Assert a cold reset.
	 */
	ac97.regs->ACGC = ACGC_ACR;

	/*
	 * Delay at least 1 us
	 */
	udelay(2);      /* temp*/

	/*
	 * Deassert ac97_rst_x signal.
	 */
	ac97.regs->ACGC = 0;

/*zhf-ac97
        if (pbackup_ac97_reg == NULL)
                return;
        ac97.regs->ACGC = pbackup_ac97_reg->ACGC;
        ac97.regs->ACGS = pbackup_ac97_reg->ACGS;
        ac97.regs->CCSL = pbackup_ac97_reg->CCSL;
        ac97.regs->CCMD = pbackup_ac97_reg->CCMD;
        ac97.regs->CRSD = pbackup_ac97_reg->CRSD;
        ac97.regs->PTFC = pbackup_ac97_reg->PTFC;
        ac97.regs->PTFS = pbackup_ac97_reg->PTFS;
        ac97.regs->PRFC = pbackup_ac97_reg->PRFC;
        ac97.regs->PRFS = pbackup_ac97_reg->PRFS;
        ac97.regs->MIFC = pbackup_ac97_reg->MIFC;
        ac97.regs->MIFS = pbackup_ac97_reg->MIFS;
        kfree(pbackup_ac97_reg);
        pbackup_ac97_reg = NULL;
zhf-ac97*/
        ac97.regs->ACGC = pbackup_ac97_reg.ACGC;
        ac97.regs->ACGS = pbackup_ac97_reg.ACGS;
        ac97.regs->CCSL = pbackup_ac97_reg.CCSL;
        ac97.regs->CCMD = pbackup_ac97_reg.CCMD;
        ac97.regs->CRSD = pbackup_ac97_reg.CRSD;
        ac97.regs->PTFC = pbackup_ac97_reg.PTFC;
        ac97.regs->PTFS = pbackup_ac97_reg.PTFS;
        ac97.regs->PRFC = pbackup_ac97_reg.PRFC;
        ac97.regs->PRFS = pbackup_ac97_reg.PRFS;
        ac97.regs->MIFC = pbackup_ac97_reg.MIFC;
        ac97.regs->MIFS = pbackup_ac97_reg.MIFS;
}
EXPORT_SYMBOL(ac97_reg_restore);

void ac97_init(void)
{
	int ret;

	DPRINTK("Enter ac97 init \n");
	/*
	 * Enable AC'97 controller interface.
	 */
	REG32_VAL(0xD8130250) |= 0x00080000;	/*Bit19  enable ac97 clk*/
	/*
	 * Assert a cold reset.
	 */
	udelay(2);      //zhf-ac97
	ac97.regs->ACGC = ACGC_ACR;

	/*
	 * Delay at least 1 us
	 */
	udelay(2);      /* temp*/

	/*
	 * Deassert ac97_rst_x signal.
	 */
	ac97.regs->ACGC = 0;

#ifdef CONFIG_WMT_AC97_DEBUG

	if (++ac97.ref > 1)
		return;

#ifdef CONFIG_PROC_FS
/*
	proc_ac97 = proc_mkdir("driver/ac97", NULL);
	proc_ac97->owner = THIS_MODULE;
	create_proc_info_entry("interrupts", 0, proc_ac97,
			       ac97_irq_read);
	create_proc_info_entry("registers", 0, proc_ac97,
			       ac97_reg_read);
*/
#endif

	ret = request_irq(ac97.irq,
			wmt_ac97_interrupt,
			IRQF_DISABLED,
			"ac97",
			NULL);

#ifndef CONFIG_SKIP_DRIVER_MSG
	DPRINTK(KERN_INFO "WMT AC'97 controller driver v" DRIVER_VERSION \
		" initialized: %s\n", (ret == 0) ? "ok" : "failed");
#endif

#endif
}

void ac97_exit(void)
{
	DPRINTK("ac97.ref = %d\n", ac97.ref);
	if (--ac97.ref)
		return;

	DPRINTK("Enter ac97 exit \n");

#ifdef CONFIG_WMT_AC97_DEBUG
#ifdef CONFIG_PROC_FS
/*
	remove_proc_entry("registers", proc_ac97);
	remove_proc_entry("interrupts", proc_ac97);
	remove_proc_entry("driver/ac97", NULL);
*/
#endif
	free_irq(ac97.irq, NULL);
#endif
	/*
	 * Turn off ahb_clk to AC'97 controller.
	 */
	REG32_VAL(0xD8130250) &= ~0x00080000;	/*Bit19  enable ac97 clk //Dean:tmp remove*/
						/*BA_PMC = 0xD8130000
						Power Management Control Base Address*/



	/*
	 * Reset software counters.
	 */
	memset(&ac97.ints, 0, sizeof(struct ac97_ints_s));
}

/* aclink_status()
 *
 * To check AC-link status from AC'97 controller,
 * return 1 for power-down, 0 for alive
 */
static int aclink_down(void)
{
	return (ac97.regs->ACGS & ACGS_CPD) ? (1) : (0);
}

static int vt1613_enable(void)
{
	int err = 0;


		//if (ac97.regs->ACGS & ACGS_CPD) {
		unsigned short ctrl;

		/*if (ac97.regs->ACGS & ACGS_CPD)
			printk("ac link is power down mode... ... ...");
		else
			printk("ac link isn't power down mode... ... ...");*/

		DPRINTK("Enter vt1613_enable %x\n", (ac97.regs->ACGS & ACGS_CPD));
		/*
		 * Perform a warm reset:
		 * 1. Assert AC97_SYNC high.
		 * 2. Dealy at least 1us.
		 * 3. Deassert AC97_SYNC
		 */
		ac97.regs->ACGC |= ACGC_AWR;
		udelay(2);
		ac97.regs->ACGC &= ~ACGC_AWR;
				DPRINTK("Proc vt1613_enable\n");
		/*
		 * Following read and write power down
		 * status reg could be skip.
		 */
		err = codec_read(VTAC_PDCS, &ctrl);
		if (err)
			return err;
		ctrl &= ~VTAC_PDCS_PR4;
		err = codec_write(VTAC_PDCS, ctrl);
	 	//}
		DPRINTK("Exit vt1613_enable\n");
	return err;
}

static int vt1613_disable(void)
{
	int err = 0;

	DPRINTK("Enter vt1613_disable %x\n", (ac97.regs->ACGS & ACGS_CPD));

	if ((ac97.regs->ACGS & ACGS_CPD) == 0) {
		/*
		 * Power-down AC-Link interface
		 */
		unsigned short ctrl;

		DPRINTK("Proc disable\n");
		err = codec_read(VTAC_PDCS, &ctrl);
		if (err)
			return err;
		ctrl |= VTAC_PDCS_PR4|BIT8|BIT9|BIT10|BIT14|BIT11|BIT13;
		err = codec_write(VTAC_PDCS, ctrl);

		err = codec_read(0x24, &ctrl);
		if (err)
			return err;
		//ctrl |= VTAC_PDCS_PR4;
		ctrl |= BIT15|BIT14|BIT13|BIT12|BIT11|BIT10|BIT9|BIT8|BIT7|BIT6|BIT5|BIT4|BIT3;
		err = codec_write(0x24, ctrl);


		
	}

	DPRINTK("Exit vt1613_disable %x\n", (ac97.regs->ACGS & ACGS_CPD));
	return err;
}

/* wait_crdy()
 *
 */
inline int wait_crdy(void)
{
	int ret = -EAGAIN;
	unsigned int crdy = ac97.ints.crdy;

	DECLARE_WAITQUEUE(wait, current);
	add_wait_queue(&codec_wq, &wait);
	ac97.regs->ACGC |= ACGC_CRIE;

	for (; ;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if ((ac97.ints.crdy > crdy) || (ret == 0))
			break;
		if (signal_pending(current))
			break;
		ret = schedule_timeout(20 * HZ / 1000);
	}

	/*
	 * The CODEC interrupt disable is in isr.
	 * We don't need to disable it here.
	 */
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&codec_wq, &wait);

	/*
	 * Check if it is timeout.
	 */
	if (ac97.ints.crdy == crdy)
		ret = -EAGAIN;
	else
		ret = 0;

	return ret;
}

/* wait_crd()
 *
 */
inline int wait_crd(void)
{
	int ret = -EAGAIN;
	unsigned int crd = ac97.ints.crd;
	unsigned int cst = ac97.ints.cst;

	DECLARE_WAITQUEUE(wait, current);
	add_wait_queue(&codec_wq, &wait);
	ac97.regs->ACGC |= ACGC_CRDIE;

	for (; ;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if ((ac97.ints.crd > crd) || \
		     (ac97.ints.cst > cst) || \
		     (ret == 0))
			break;
		if (signal_pending(current))
			break;
		ret = schedule_timeout(20 * HZ / 1000);
	}

	/*
	 * The CODEC interrupt disable is in isr.
	 * We don't need to disable it here.
	 */
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&codec_wq, &wait);

	/*
	 * Check if it is timeout
	 */
	if (ac97.ints.cst > cst)
		ret = -EAGAIN;
	else
	if (ac97.ints.crd == crd)
		ret = -EAGAIN;
	else
		ret = 0;

	return ret;
}

/* wait_cwd()
 *
 */
inline int wait_cwd(void)
{
	int ret = -EAGAIN;
	unsigned int cwd = ac97.ints.cwd;

	DECLARE_WAITQUEUE(wait, current);
	add_wait_queue(&codec_wq, &wait);
	ac97.regs->ACGC |= ACGC_CWDIE;

	for (; ;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if ((ac97.ints.cwd > cwd) || (ret == 0))
			break;
		if (signal_pending(current))
			break;
		ret = schedule_timeout(20 * HZ / 1000);
	}

	/*
	 * The CODEC interrupt disable is in isr.
	 * We don't need to disable it here.
	 */
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&codec_wq, &wait);

	/*
	 * Check if it is timeout, errno
	 */
	if (ac97.ints.cwd == cwd)
		ret = -EAGAIN;
	else
		ret = 0;

	return ret;
}


int codec_read(u16 addr, u16 *data)
{
	int err = 0;

	DPRINTK("ac97 read addr = 0x%x\n", addr);
	if (addr > VTAC_MAX || data == NULL)
		return -EINVAL;

		if (down_interruptible(&l_sem_wrrdcodec))
		{
			return -ERESTARTSYS;
		}
	if (in_interrupt()) {
		unsigned long flags, cnt;

		local_irq_save(flags);

		/*
		 * Wait for CODEC ready.
		 */
		cnt = 10000;
		while (!(ac97.regs->ACGS & ACGS_CRDY) && --cnt)
			;
		if (!cnt) {
			local_irq_restore(flags);
			printk("read ac97 codec failed\n");
                        err = -EBUSY;
                        goto codec_read_out;
		}

		/*
		 * Issue a read command.
		 */
		ac97.regs->CCMD = CCMD_CCA(addr) | CCMD_CCRW;

		/*
		 * Wait for CODEC read done or hw timeout.
		 */
		cnt = 10000;
		while (!(ac97.regs->ACGS & (ACGS_CRD|ACGS_CST)) && --cnt)
			;
		if (cnt)
			ac97.regs->ACGS = (ACGS_CRD|ACGS_CST);
		else
			err = -EAGAIN;

		local_irq_restore(flags);
	} else {
		if ((err = wait_crdy()) != 0)
			goto codec_read_out;

		/*
		 * Issue a read command.
		 */
		ac97.regs->CCMD = CCMD_CCA(addr) | CCMD_CCRW;

		if ((err = wait_crd()) != 0)
			goto codec_read_out;
	}

	if (!err) {
		*data = (u16)(ac97.regs->CRSD & CRSD_CSDMASK);
		DPRINTK("read success\n");
	} else {
		*data = (u16)(CODEC_FAIL);
		DPRINTK("read failed\n");
	}

	DPRINTK("ac97 read 0x%x = 0x%x\n", addr, *data);


codec_read_out:
	up(&l_sem_wrrdcodec);
	return err;
}

int codec_write(u16 addr, u16 data)
{
	int err = 0;

	DPRINTK("ac97 write 0x%x = 0x%x\n", addr, data);

		if (down_interruptible(&l_sem_wrrdcodec))
		{
			return -ERESTARTSYS;
		}
	if (in_interrupt()) {
		unsigned long flags, cnt;

		local_irq_save(flags);

		/*
		 * Wait for CODEC ready.
		 */
		cnt = 10000;
		while (!(ac97.regs->ACGS & ACGS_CRDY) && --cnt)
			;
		if (!cnt) {
			printk("write_codec failed\n");
			local_irq_restore(flags);
                        err = -EBUSY;
                        goto codec_write_out;
		}

		/*
		 * Issue a write command.
		 */
		ac97.regs->CCMD = CCMD_CCA(addr) | CCMD_CWCD(data);

		/*
		 * Wait for CODEC write done.
		 */
		cnt = 10000;
		while (!(ac97.regs->ACGS & ACGS_CWD) && --cnt)
			;
		if (cnt)
			ac97.regs->ACGS = ACGS_CWD;
		else{
			printk("write_timeout\n");
			err = -EAGAIN;
		}

		local_irq_restore(flags);
	} else {
		if ((err = wait_crdy()) != 0)
			goto codec_write_out;

		/*
		 * Issue a write command.
		 */
		ac97.regs->CCMD = CCMD_CCA(addr) | CCMD_CWCD(data);

		/*
		 * Wait CODEC write done.
		 */
		err = wait_cwd();
	}
codec_write_out:
	up(&l_sem_wrrdcodec);
	return err;
}

static u16 codec_bread(u16 addr)
{
	u16 ret = (u16)(CODEC_FAIL);

	if (addr > VTAC_MAX)
		goto codec_bread_out;

	/*
	 * Busy polling for CODEC ready.
	 */
	while (!(ac97.regs->ACGS & ACGS_CRDY))
		;

	/*
	 * Issue a read command.
	 */
	ac97.regs->CCMD = CCMD_CCA(addr) | CCMD_CCRW;

	/*
	 * Busy polling for CODEC read done or timeout.
	 */
	while (!(ac97.regs->ACGS & (ACGS_CRD|ACGS_CST)))
		;
	if (ac97.regs->ACGS & ACGS_CRD) {
		ac97.regs->ACGS = ACGS_CRD;
		ret = (u16)(ac97.regs->CRSD & CRSD_CSDMASK);
	} else
		ac97.regs->ACGS = ACGS_CST;

codec_bread_out:
	return ret;
}

static void codec_bwrite(u16 addr, u16 data)
{
	/*
	 * Busy polling for CODEC ready.
	 */
	while (!(ac97.regs->ACGS & ACGS_CRDY))
		;

	/*
	 * Issue a write command.
	 */
	ac97.regs->CCMD = CCMD_CCA(addr) | CCMD_CWCD(data);

	/*
	 * Busy polling for CODEC write done.
	 */
	while (!(ac97.regs->ACGS & ACGS_CWD))
		;
		ac97.regs->ACGS = ACGS_CWD;
}

int vt1613_init(void)
{
	int err;

	DPRINTK("Enter init\n");
	printk("init: vt1613_ref=%d\n", vt1613_ref);

	if (++vt1613_ref > 1)
		return 0;
	DPRINTK("Proc init\n");

	/*
	 * Configure PIN Multiplexing[0x0200] as GPIO
	 */
#define	GPIO_PIN_MULTI			REG32_VAL(GPIO_BASE_ADDR + 0x200)
								/*BA_GPIO = 0xD8110000 GPIO Base Address*/
#define MULTI_I2S_AC97                	0x00000002  		/* [bit1]0:I2S*/
								/* [bit1]1:AC97*/
#define GPIO_AC97_ENABLE                REG32_VAL(GPIO_BASE_ADDR + 0x54)/*[4:0]*/
	GPIO_AC97_ENABLE &= ~(0x1F);
	/* IO PIN INIT*/
	GPIO_PIN_MULTI |= (MULTI_I2S_AC97);


	/*
	 * Assert a cold reset.
	 */
	ac97_init();

	/*
	 * Assert a cold reset.
	 */
	ac97.regs->ACGC = ACGC_ACR;

	/*
	 * Delay at least 1 us
	 */
	udelay(2);

	/*
	 * Deassert ac97_rst_x signal.
	 */
	ac97.regs->ACGC = 0;

	/*
	 * Clean all AC'97 global status.
	 */
	ac97.regs->ACGS = (ACGS_CWD | ACGS_CRD | ACGS_CST);
	vt1613_enable();/*Dean add on 2008/7/22 for power management*/

	/*
	 * Read CODEC id1 & id2
	 */
	if ((err = codec_read(VTAC_VID1, &codec->id1)))
		goto init_error;
	if ((err = codec_read(VTAC_VID2, &codec->id2)))
		goto init_error;

	DPRINTK("codec VTAC_VID1 %d\n", codec->id1);
	DPRINTK("codec VTAC_VID2 %d\n", codec->id2);

	return 0;

init_error:

	return err;
}
EXPORT_SYMBOL(vt1613_init);

void vt1613_exit(void)
{
	printk("exit:vt1613_ref = %d\n", vt1613_ref);

	if (--vt1613_ref == 0) {
		vt1613_disable();/*Dean add on 2008/07/22 for power management*/
		ac97_exit();
	}
}
EXPORT_SYMBOL(vt1613_exit);

struct codec_ops_s vt1613_ops = {
	.aclink         = aclink_down,
	.init           = vt1613_init,
	.exit           = vt1613_exit,
	.enable         = vt1613_enable,
	.disable        = vt1613_disable,
	.read           = codec_read,
	.write          = codec_write,
	.bread          = codec_bread,
	.bwrite         = codec_bwrite,
};

struct codec_s *codec_attach(void)
{
	if (!codec) {
		codec = kmalloc(sizeof(struct codec_s), GFP_KERNEL);

		if (codec) {
			memset(codec, 0, sizeof(struct codec_s));
			codec->ops = &vt1613_ops;
			codec->mod = 0;
		}
	}

	return codec;
}

void codec_detach(struct codec_s *codec)
{
	if (codec)
		kfree(codec);
}
EXPORT_SYMBOL(codec_read);
EXPORT_SYMBOL(codec_write);
EXPORT_SYMBOL(ac97_init);
EXPORT_SYMBOL(vt1613_ops);     /* Plan to be local static*/
EXPORT_SYMBOL(codec_attach);
EXPORT_SYMBOL(codec_detach);
MODULE_LICENSE("GPL");
