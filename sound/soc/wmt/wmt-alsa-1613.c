/*
 * sound/arm/wmt/wmt-alsa-1613.c
 * 
 * Alsa codec Driver for VT1613 chip
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *       
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * WonderMedia Technologies, Inc.
 *  10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
 *
 *  History:
 *          2009/06    First Version
 */

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#include <mach/ac97_alsa.h>
#include <mach/vt1613.h>
#include "wmt-alsa-1613.h"

#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define	DRIVER_VERSION		"0.01"
#define AUDIO_RATE_DEFAULT              48000
#define CODEC_FAIL              (-1)
#define CODEC_MASK              (0x0000FFFF)

#define CONFIG_WMT_AC97_DEBUG	0

/*
 * CODEC pointer
 */
struct codec_s *codec;
struct codec_ops_s vt1613_ops;
static int vt1613_ref;

static void ac97_init(void);
static void ac97_exit(void);

/*
 * wait queue head is using for waiting CODEC ready, CODEC read done,
 * and CODEC write done interrups.
 */
static DECLARE_WAIT_QUEUE_HEAD(codec_wq);
/*
 * Change this to static in the future
 */
wmt_ac97_t ac97 = {
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

struct wmt_alsa_codec_config alsa_config = {
	.name = "VT1613",
	.hw_playback_constraints_rates = NULL,
	.hw_capture_constraints_rates = NULL,
	.snd_wmt_alsa_playback = NULL,
	.snd_wmt_alsa_capture = NULL,
	.codec = NULL,
	.codec_configure = NULL,
	.codec_set_dac_samplerate = NULL,
	.codec_set_adc_samplerate = NULL,
	.get_default_samplerate = NULL,
};

/*
 * Hardware capabilities
 */
 
static unsigned int playback_rates[] = {
	8000, 16000, 22050,
	24000, 32000, 44100,
	48000, 88200, 96000,
};
static unsigned int capture_rates[] = {
        8000, 16000, 22050, 24000,
        32000, 44100, 48000,
};

static struct snd_pcm_hw_constraint_list vt1613_hw_playback_constraints_rates = {
        .count = ARRAY_SIZE(playback_rates),
        .list = playback_rates,
	.mask = 0,
};
static struct snd_pcm_hw_constraint_list vt1613_hw_capture_constraints_rates = {
        .count = ARRAY_SIZE(capture_rates),
        .list = capture_rates,
        .mask = 0,
};


static struct snd_pcm_hardware vt1613_snd_wmt_alsa_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 ),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 1,		// 1,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = SZ_16K,
	.period_bytes_max = SZ_16K,	//8 * 1024,
	.periods_min = 2,
	.periods_max = 255,
	.fifo_size = 32,
};

static struct snd_pcm_hardware vt1613_snd_wmt_alsa_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 ),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,		// 1,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = SZ_16K, //SZ_16K,
	.periods_min = 2,
	.periods_max = 255,
	.fifo_size = 32,
};


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
wmt_ac97_interrupt(int irq, void *dev_id, struct pt_regs *regs)
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


#ifdef CONFIG_PROC_FS_Vincent

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

#endif  /* CONFIG_PROC_FS_Vincent */

#endif	 /* CONFIG_WMT_AC97_DEBUG */

/* aclink_status()
 *
 * To check AC-link status from AC'97 controller,
 * return 1 for power-down, 0 for alive
 */
static int aclink_down(void)
{
	return (ac97.regs->ACGS & ACGS_CPD) ? (1) : (0);
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

int vt1613_codec_read(u16 addr, u16 *data)
{
	int err = 0;

	DPRINTK("ac97 read addr = 0x%x\n", addr);

	if (addr > VTAC_MAX || data == NULL)
		return -EINVAL;

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
			return -EBUSY;
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
		DPRINTK("ac97 read 0x%x = 0x%x\n", addr, *data);
	} else {
		*data = (u16)(CODEC_FAIL);
		DPRINTK("read failed: err=%d\n", err);
	}

codec_read_out:
	return err;
}


int vt1613_codec_write(u16 addr, u16 data)
{
	int err = 0;

	DPRINTK("ac97 write 0x%x = 0x%x\n", addr, data);

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
			return -EBUSY;
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
	DPRINTK("err=%d\n", err);
	return err;
}

/* export codec read/write funtions for mixer */
EXPORT_SYMBOL_GPL(vt1613_codec_read);
EXPORT_SYMBOL_GPL(vt1613_codec_write);

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


/*
 * Sample rate changing
 */
static long adc_rate = AUDIO_RATE_DEFAULT;
static long dac_rate = AUDIO_RATE_DEFAULT;
void vt1613_set_dac_samplerate(long val)
{
	unsigned short dac;

	switch (val) {
	case 8000:
		dac = 0x1F40;
		break;
	case 11025:
		dac = 0x2B11;
		break;
	case 12000:
		dac = 0x2EE0;
		break;
	case 16000:
		dac = 0x3E80;
		break;
	case 22050:
		dac = 0x5622;
		break;
	case 24000:
		dac = 0x5DC0;
		break;
	case 32000:
		dac = 0x7D00;
		break;
	case 44100:
		dac = 0xAC44;
		break;
	case 48000:
		dac = 0xBB80;
		break;
	default:
		dac = val; /* dac = 48000; */
		break;
	}

	DPRINTK("val=%lu dac=%d\n", val, dac);

	codec->ops->write(VTAC_DSRC, dac);

	/*
	 * Save audio DAC rate
	 */
	dac_rate = (long)dac;
}


void vt1613_set_adc_samplerate(long val)
{
	unsigned short adc;

	switch (val) {
	case 8000:
		adc = 0x1F40;
		break;
	case 11025:
		adc = 0x2B11;
		break;
	case 12000:
		adc = 0x2EE0;
		break;
	case 16000:
		adc = 0x3E80;
		break;
	case 22050:
		adc = 0x5622;
		break;
	case 24000:
		adc = 0x5DC0;
		break;
	case 32000:
		adc = 0x7D00;
		break;
	case 44100:
		adc = 0xAC44;
		break;
	case 48000:
		adc = 0xBB80;
		break;
	default:
		adc = 48000;
		break;
	}

	DPRINTK("val=%lu adc=%d\n", val, adc);

	/*
	 * ADC are capable of operating at independent rates.
	 */
	codec->ops->write(VTAC_ASRC, adc);

	/*
	 * Save audio ADC rate.
	 */
	adc_rate = (long)adc;
}

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

int vt1613_configure(void)
{
	codec = codec_attach();

	if (!codec)
		return -ENODEV;
	return 0;
}


static void ac97_init(void)
{
#ifdef CONFIG_WMT_AC97_DEBUG
	int ret;
#endif
	DPRINTK("Enter\n");
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

#ifdef CONFIG_PROC_FS_Vincent
	proc_ac97 = proc_mkdir("driver/ac97", NULL);
	proc_ac97->owner = THIS_MODULE;
	create_proc_info_entry("interrupts", 0, proc_ac97,
			       ac97_irq_read);
	create_proc_info_entry("registers", 0, proc_ac97,
			       ac97_reg_read);
#endif

	ret = request_irq(ac97.irq,
			wmt_ac97_interrupt,
			IRQF_DISABLED,
			"wmt_alsa_vt1613",
			NULL);

#ifndef CONFIG_SKIP_DRIVER_MSG
	DPRINTK(KERN_INFO "WMT AC'97 controller driver v" DRIVER_VERSION \
		" initialized: %s\n", (ret == 0) ? "ok" : "failed");
#endif

#endif
}

static void ac97_exit(void)
{
	DPRINTK("ac97.ref = %d\n", ac97.ref);
	if (--ac97.ref)
		return;

	DPRINTK("Enter ac97 exit \n");

#ifdef CONFIG_WMT_AC97_DEBUG
#ifdef CONFIG_PROC_FS_Vincent
	remove_proc_entry("registers", proc_ac97);
	remove_proc_entry("interrupts", proc_ac97);
	remove_proc_entry("driver/ac97", NULL);
#endif
	free_irq(ac97.irq, NULL);
#endif
	/*
	 * Turn off ahb_clk to AC'97 controller.
	 */
	REG32_VAL(0xD8130250) &= ~0x00080000;	/*Bit19  disable ac97 clk //Dean:tmp remove*/
						/*BA_PMC = 0xD8130000
						Power Management Control Base Address*/

	/*
	 * Reset software counters.
	 */
	memset(&ac97.ints, 0, sizeof(struct ac97_ints_s));
}



int vt1613_enable(void)
{
	int err = 0;


	REG32_VAL(0xd8130000+0x250) |= 0x00080000;  /*enable pmc clock*/
	udelay(2);	//zhf-temp delay for 3426
	if (ac97.regs->ACGS & ACGS_CPD) {
		unsigned short ctrl;

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
		err = vt1613_codec_read(VTAC_PDCS, &ctrl);
		if (err)
			return err;
		ctrl &= ~VTAC_PDCS_PR4;
		err = vt1613_codec_write(VTAC_PDCS, ctrl);
	 }
		DPRINTK("Exit vt1613_enable\n");
	return err;
}

int vt1613_disable(void)
{
	int err = 0;

	printk("vt1613_disable: Enter\n");      //zhf-t
	DPRINTK("Enter vt1613_disable %x\n", (ac97.regs->ACGS & ACGS_CPD));

	if ((ac97.regs->ACGS & ACGS_CPD) == 0) {
		/*
		 * Power-down AC-Link interface
		 */
		unsigned short ctrl;

		DPRINTK("Proc disable\n");
		err = vt1613_codec_read(VTAC_PDCS, &ctrl);
		if (err)
			return err;
		ctrl |= VTAC_PDCS_PR4;
		err = vt1613_codec_write(VTAC_PDCS, ctrl);
	}

	DPRINTK("Exit vt1613_disable %x\n", (ac97.regs->ACGS & ACGS_CPD));
	return err;
}


static int vt1613_init(void)
{
	int err;

	DPRINTK("Enter\n");
	DPRINTK("vt1613_ref=%d\n", vt1613_ref);
	printk("vt1613_init: vt1613_ref=%d\n", vt1613_ref);	//zhf-t

	if (++vt1613_ref > 1)
		return 0;

	printk("vt1613_init: Beging init\n");           //zhf-t 
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
	if ((err = vt1613_codec_read(VTAC_VID1, &codec->id1)))
		goto init_error;
	if ((err = vt1613_codec_read(VTAC_VID2, &codec->id2)))
		goto init_error;

	DPRINTK("codec VTAC_VID1 %d\n", codec->id1);
	DPRINTK("codec VTAC_VID2 %d\n", codec->id2);

	return 0;

init_error:
	printk("err=%d\n", err);        //zhf-t1
	return err;
}

static void vt1613_exit(void)
{
	DPRINTK("Enter, vt1613_ref=%d\n", vt1613_ref);
	printk("vt1613_exit, vt1613_ref=%d\n", vt1613_ref);	//zhf-t
	if (--vt1613_ref <= 0)
	{
		printk("vt1613_exit: Beging exit\n");           //zhf-t
		vt1613_disable();/*Dean add on 2008/07/22 for power management*/
		ac97_exit();
	}
}


struct codec_ops_s vt1613_ops = {
	.aclink         = aclink_down,
	.init           = vt1613_init,
	.exit           = vt1613_exit,
	.enable         = vt1613_enable,
	.disable        = vt1613_disable,
	.read           = vt1613_codec_read,
	.write          = vt1613_codec_write,
	.bread          = codec_bread,
	.bwrite         = codec_bwrite,
};


int vt1613_get_default_samplerate(void)
{
	return AUDIO_RATE_DEFAULT;
}

static int __devinit snd_wmt_alsa_vt1613_probe(struct platform_device *pdev)
{
	int	ret;
	struct	wmt_alsa_codec_config *codec_cfg;

	DPRINTK("pdev=%d, codec_cfg=%d\n", pdev, pdev->dev.platform_data);
	vt1613_configure();
	codec_cfg = pdev->dev.platform_data;
	if (codec_cfg != NULL) {
		codec_cfg->hw_playback_constraints_rates        = &vt1613_hw_playback_constraints_rates;
                codec_cfg->hw_capture_constraints_rates = &vt1613_hw_capture_constraints_rates;
		codec_cfg->snd_wmt_alsa_playback  = &vt1613_snd_wmt_alsa_playback;
		codec_cfg->snd_wmt_alsa_capture  = &vt1613_snd_wmt_alsa_capture;
		codec_cfg->codec = codec;
		codec_cfg->codec_configure	= vt1613_configure;
		codec_cfg->codec_set_dac_samplerate	= vt1613_set_dac_samplerate;
		codec_cfg->codec_set_adc_samplerate	= vt1613_set_adc_samplerate;
		codec_cfg->get_default_samplerate = vt1613_get_default_samplerate;
		ret	= snd_wmt_alsa_post_probe(pdev, codec_cfg);
	}
	else
		ret = -ENODEV;
	return ret;
}


static int snd_wmt_alsa_vt1613_remove(struct platform_device *pdev)
{
	int err=0;
	
	err = snd_wmt_alsa_remove(pdev);
	if (codec) {
		codec_detach(codec);
		codec = NULL;
	}
	return err;
}

static struct platform_driver wmt_alsa_driver = {
	.probe	= snd_wmt_alsa_vt1613_probe,
	.remove 	= snd_wmt_alsa_vt1613_remove, 
	.suspend	= snd_wmt_alsa_suspend,
	.resume	= snd_wmt_alsa_resume,
	.driver	= {
	.name =	"wmt_alsa_vt1613",
	},
};


static void ac97_platform_release(struct device *device)
{
}

static struct resource wmt_ac97_resources[] = {
	[0] = {
	.start  = 0xD8290000,
	.end    = 0xD829ffff,
	.flags  = 0x00000200,        },
};

static u64 wmt_ac97_dma_mask = DMA_32BIT_MASK;
static struct platform_device wmt_ac97_device = {
	.name           = "wmt_alsa_vt1613",
	.id             = 0,
	.dev            = {
		.release = ac97_platform_release,
		.dma_mask = &wmt_ac97_dma_mask,
		.coherent_dma_mask = ~0,
		.platform_data	= &alsa_config,
	},
	.num_resources  = ARRAY_SIZE(wmt_ac97_resources),
	.resource       = wmt_ac97_resources,
};

static int __init wmt_alsa_vt1613_init(void)
{
	int err;

	DPRINTK("Enter\n");
	platform_device_register(&wmt_ac97_device);
	err = platform_driver_register(&wmt_alsa_driver);

	return err;
}

static void __exit wmt_alsa_vt1613_exit(void)
{
	platform_driver_unregister(&wmt_alsa_driver);
	platform_device_unregister(&wmt_ac97_device);
}

module_init(wmt_alsa_vt1613_init);
module_exit(wmt_alsa_vt1613_exit);
