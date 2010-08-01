/*++
	sound/arm/wmt_i2s.c

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
		The code was inherit from vt8420
		2009/01/21 First Version

--*/
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/soundcard.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/ioctl.h>
#include <linux/err.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/dma.h>
#include <asm/mach-types.h>
#include <asm/sizes.h>

#include "wmt_i2s.h"
#include "i2s.h"
#include "wmt_swmixer.h"

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __func__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#ifndef memzero
#define memzero(s, n)     memset ((s), 0, (n))
#endif

#define AUDIO_NAME                      "i2s"
#define AUDIO_NBFRAGS_DEFAULT           32
#define AUDIO_FRAGSIZE_DEFAULT          SZ_4K
#define AUDIO_FMT_MASK                  (AFMT_S16_LE | AFMT_U8)
#define AUDIO_FMT_DEFAULT               (AFMT_S16_LE | AFMT_U8)
#define AUDIO_CHANNELS_DEFAULT          2
#define AUDIO_VOLUME_DEFAULT            0x0D
#define AUDIO_ACTIVE(state)             ((state)->rd_ref || (state)->wr_ref)
#define AUDIO_MODE_MONO                 0
#define AUDIO_MODE_STEREO               1
#define AUDIO_RATE_DEFAULT              44100
#define FLUSH_BASE_PHYS                 0xe0000000
#define SPIN_ADDR                       (dma_addr_t)FLUSH_BASE_PHYS
#define SPIN_SIZE                       2048

extern struct i2s_s i2s;
static int sample_rate = AUDIO_RATE_DEFAULT;

static unsigned int channels = AUDIO_CHANNELS_DEFAULT;
static long pcm_fmt = AFMT_S16_LE;
#ifdef CONFIG_PM
static struct i2s_regs_s i2s_pm_regs;
#endif

/*
 * local CODEC pointer
 */
static struct i2s_codec_s *codec;

/*
 * DMA processing
 */
#define DMA_REQUEST(s, cb)      wmt_request_dma(&s->dmach, s->id, s->dma_dev, cb, s)
#define DMA_FREE(s)             {wmt_free_dma(s->dmach); s->dmach = NULL_DMA; }
#define DMA_START(s, d, l)      wmt_start_dma(s->dmach, d, 0, l)
#define DMA_POS(s)              wmt_get_dma_pos(s->dmach)
#define DMA_STOP(s)             wmt_stop_dma(s->dmach)
#define DMA_CLEAR(s)            wmt_clear_dma(s->dmach)
#define DMA_RESET(s)            wmt_reset_dma(s->dmach)
#define SYSTEM_DMA_WAKE(s)      wmt_wake_dma(s->dmach, d, 0, l)

#define IS_NULLDMA(ch)          ((ch) == NULL_DMA)
#define CHK_DMACH(ch)           ((ch) != NULL_DMA)


static u_int audio_get_dma_pos(audio_stream_t *s)
{
	audio_buf_t *b = &s->buffers[s->dma_tail];
	u_int offset;

	if (b->dma_ref) {
		offset = DMA_POS(s) - b->dma_addr;
//		offset = b->dma_addr;
		if (offset >= s->fragsize)
			offset = s->fragsize - 4;
	} else if (s->pending_frags) {
		offset = b->offset;
	} else {
		offset = 0;
	}
	return offset;
}

static void audio_stop_dma(audio_stream_t *s)
{
	u_int pos;
	unsigned long flags;
	audio_buf_t *b;

	if (s->dma_spinref > 0 || !s->buffers)
		return;

	local_irq_save(flags);
	s->stopped = 1;
	DMA_STOP(s);
	pos = audio_get_dma_pos(s);
//	DMA_CLEAR(s);
	if (s->spin_idle) {
		DMA_START(s, SPIN_ADDR, SPIN_SIZE);
		DMA_START(s, SPIN_ADDR, SPIN_SIZE);
		s->dma_spinref = 2;
	} else
		s->dma_spinref = 0;
	local_irq_restore(flags);
#if 0
       /*       
        *  because we have keep the current descriptor info  in DMA driver
        *  we need not  re-transfer the dma buf here
        *  by Vincent  2009/05/19
        */

	/* back up pointers to be ready to restart from the same spot*/
	while (s->dma_head != s->dma_tail) {
		b = &s->buffers[s->dma_head];
		if (b->dma_ref) {
			b->dma_ref = 0;
			b->offset = 0;
		}
		s->pending_frags++;
		if (s->dma_head == 0)
			s->dma_head = s->nbfrags;
		s->dma_head--;
	}
	b = &s->buffers[s->dma_head];
	if (b->dma_ref) {
		b->offset = pos;
		b->dma_ref = 0;
	}
#endif
}

static void audio_reset(audio_stream_t *s)
{
	if (s->buffers) {
		audio_stop_dma(s);
		s->buffers[s->dma_head].offset = 0;
		s->buffers[s->usr_head].offset = 0;
		s->usr_head = s->dma_head;
		s->pending_frags = 0;
		compat_sema_init(&s->sem, s->nbfrags);
	}
	s->active = 0;
	s->stopped = 0;
}

/* audio_setup_dma()
 *
 * Just a simple funcion to control DMA setting for I2S audio
 */
static void audio_setup_dma(audio_stream_t *s)
{

	if (s->direction)
		s->dma_cfg.DefaultCCR = I2S_TX_DMA_CFG;
	else
		s->dma_cfg.DefaultCCR = I2S_RX_DMA_CFG;
	s->dma_cfg.ChunkSize = SZ_4K;
	wmt_setup_dma(s->dmach, s->dma_cfg) ;
}

static void audio_process_dma(audio_stream_t *s)
{
	int ret;

	if (s->stopped)
		goto spin;

	if (s->dma_spinref > 0 && s->pending_frags) {
		s->dma_spinref = 0;
		DMA_CLEAR(s);
	}

	while (s->pending_frags) {
		audio_buf_t *b = &s->buffers[s->dma_head];
		u_int dma_size = s->fragsize - b->offset;
		if (dma_size > MAX_DMA_SIZE)
			dma_size = CUT_DMA_SIZE;
		ret = DMA_START(s, b->dma_addr + b->offset, dma_size);
		if (ret)
			return;
		b->dma_ref++;
		b->offset += dma_size;
		if (b->offset >= s->fragsize) {
			s->pending_frags--;
			if (++s->dma_head >= s->nbfrags)
				s->dma_head = 0;
		}
	}
spin:
	if (s->spin_idle) {
		int spincnt = 0;
		while (DMA_START(s, SPIN_ADDR, SPIN_SIZE) == 0)
			spincnt++;
		/**/
		/* Note: if there is still a data buffer being*/
		/* processed then the ref count is negative.  This*/
		/* allows for the DMA termination to be accounted in*/
		/* the proper order.  Of course dma_spinref can't be*/
		/* greater than 0 if dma_ref is not 0 since we kill*/
		/* the spinning above as soon as there is real data to process.*/
		/**/
		if (s->buffers && s->buffers[s->dma_tail].dma_ref)
			spincnt = -spincnt;
		s->dma_spinref += spincnt;
	}
}

static void audio_dma_callback(void *data)
{
	audio_stream_t *s = data;
	audio_buf_t *b = &s->buffers[s->dma_tail];

	DPRINTK("audio_dma_callback\n");

	if (s->dma_spinref > 0) {
		s->dma_spinref--;
	} else if (!s->buffers) {
		printk(KERN_CRIT "wmt_audio: received DMA IRQ for non existent buffers!\n");
		return;
	} else if (b->dma_ref && --b->dma_ref == 0 && b->offset >= s->fragsize) {
		/* This fragment is done*/
		b->offset = 0;
		s->bytecount += s->fragsize;
		s->fragcount++;
		s->dma_spinref = -s->dma_spinref;
		if (++s->dma_tail >= s->nbfrags)
			s->dma_tail = 0;
		if (!s->mapped)
			compat_up(&s->sem);
		else
			s->pending_frags++;
		wake_up(&s->wq);
	}
	audio_process_dma(s);
}

/*
 * Buffer creation/destruction
 */
static void audio_discard_buf(audio_stream_t *s)
{
	/* ensure DMA isn't using those buffers*/
	audio_reset(s);

	if (s->buffers) {
		int frag;
		for (frag = 0; frag < s->nbfrags; frag++) {
			if (!s->buffers[frag].master)
				continue;
			dma_free_coherent(s->dev,
					  s->buffers[frag].master,
					  s->buffers[frag].data,
					  s->buffers[frag].dma_addr);
		}
		kfree(s->buffers);
		s->buffers = NULL;
	}
}

static int audio_setup_buf(audio_stream_t *s)
{
	int frag;
	int dmasize = 0;
	char *dmabuf = NULL;
	dma_addr_t dmaphys = 0;

	if (s->buffers)
		return -EBUSY;

	/**/
	/* Configure DMA setting*/
	/**/
	audio_setup_dma(s);

	s->buffers = kmalloc(sizeof(audio_buf_t) * s->nbfrags, GFP_KERNEL);
	if (!s->buffers)
		goto err;
	memset(s->buffers, 0, sizeof(audio_buf_t) * s->nbfrags);

	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];

		/* Let's allocate non-cached memory for DMA buffers.*/
		/* We try to allocate all memory at once.*/
		/* If this fails (a common reason is memory fragmentation),*/
		/* then we allocate more smaller buffers.*/
		if (!dmasize) {
			/*dmasize = (s->nbfrags - frag) * s->fragsize;*/

			/*
			 * Since it could not allocate a continuous memory with nbfrag * fragsize size.
			 * Then it would print warning message and would waste time.
                         * So we only allocat one page(4KB) every time.
			 */
			dmasize = s->fragsize;/*Dean revised*/
			do {
				dmabuf = dma_alloc_coherent(s->dev, dmasize, &dmaphys, GFP_KERNEL);
				if (!dmabuf)
					dmasize -= s->fragsize;
			} while (!dmabuf && dmasize);
			if (!dmabuf)
				goto err;
			b->master = dmasize;
			memzero(dmabuf, dmasize);
		}

		b->data = dmabuf;
		b->dma_addr = dmaphys;

		dmabuf += s->fragsize;
		dmaphys += s->fragsize;
		dmasize -= s->fragsize;
	}

	s->usr_head = s->dma_head = s->dma_tail = 0;
	s->bytecount = 0;
	s->fragcount = 0;
	compat_sema_init(&s->sem, s->nbfrags);

	return 0;

err:
	printk(AUDIO_NAME ": unable to allocate audio memory\n ");
	audio_discard_buf(s);
	return -ENOMEM;
}

/*
 * Driver interface functions
 */
static int audio_write(struct file *file, const char *buffer,
			size_t count, loff_t *ppos)
{
	const char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	int chunksize, ret = 0;
	unsigned long flags;

	if (*ppos != file->f_pos)
		return -ESPIPE;
	if (s->mapped)
		return -ENXIO;
	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;

	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_head];

		/* Wait for a buffer to become free*/
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (compat_down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (compat_down_interruptible(&s->sem))
				break;
		}

		/* Feed the current buffer*/
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		DPRINTK("write %d to %d\n", chunksize, s->usr_head);
		if (copy_from_user(b->data + b->offset, buffer, chunksize)) {
			compat_up(&s->sem);
			return -EFAULT;
		}
		/*
		 *if output data was unisnge format
		 *we need to transform unsigned into signed
		 */
		wmt_sw_u2s(pcm_fmt, b->data + b->offset, chunksize);

		buffer += chunksize;
		count -= chunksize;
		b->offset += chunksize;
		if (b->offset < s->fragsize) {
			compat_up(&s->sem);
			break;
		}

		/* Update pointers and send current fragment to DMA*/
		b->offset = 0;
		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;
		local_irq_save(flags);
		s->pending_frags++;
		s->active = 1;
		audio_process_dma(s);
		local_irq_restore(flags);
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;
	DPRINTK("audio_write: return=%d\n", ret);
	return ret;
}

static void audio_prime_rx(audio_state_t *state)
{
	audio_stream_t *is = state->input_stream;
	unsigned long flags;

	local_irq_save(flags);
	if (state->need_tx_for_rx) {
		/**/
		/* UCB1400 gets its clock from the ZAC.*/
		/* While there is no playback data to send, the output DMA*/
		/* will spin with all zeroes.  We use the cache flush special*/
		/* area for that.*/
		/**/
		state->output_stream->spin_idle = 1;
		audio_process_dma(state->output_stream);
	}
	is->pending_frags = is->nbfrags;
	compat_sema_init(&is->sem, 0);
	is->active = 1;
	audio_process_dma(is);
	local_irq_restore(flags);
}

static int audio_read(struct file *file, char *buffer,
			size_t count, loff_t *ppos)
{
	char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->input_stream;
	int chunksize, ret = 0;
	unsigned long flags;

	DPRINTK("audio_read: count=%d\n", count);


	if (*ppos != file->f_pos)
		return -ESPIPE;
	if (s->mapped)
		return -ENXIO;

	if (!s->active) {
		if (!s->buffers && audio_setup_buf(s))
			return -ENOMEM;
		audio_prime_rx(state);
	}

	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_head];

		/* Wait for a buffer to become full*/
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (compat_down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (compat_down_interruptible(&s->sem))
				break;
		}

		/* Grab data from the current buffer*/
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		DPRINTK("read %d from %d\n", chunksize, s->usr_head);
		if (copy_to_user(buffer, b->data + b->offset, chunksize)) {
			compat_up(&s->sem);
			return -EFAULT;
		}
		buffer += chunksize;
		count -= chunksize;
		b->offset += chunksize;
		if (b->offset < s->fragsize) {
			compat_up(&s->sem);
			break;
		}

		/* Update pointers and return current fragment to DMA*/
		b->offset = 0;
		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;
		local_irq_save(flags);
		s->pending_frags++;
		audio_process_dma(s);
		local_irq_restore(flags);
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;
	DPRINTK("audio_read: return=%d\n", ret);
	return ret;
}

static int audio_sync(struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	audio_buf_t *b;
	u_int shiftval = 0;
	unsigned long flags;
	DECLARE_WAITQUEUE(wait, current);

	DPRINTK("audio_sync\n");

	if (!(file->f_mode & FMODE_WRITE) || !s->buffers || s->mapped)
		return 0;

	/* Send current buffer if it contains data.  Be sure to send*/
	/* a full sample count.*/
	b = &s->buffers[s->usr_head];
	b->offset &= ~3;
	if (b->offset) {
		compat_down(&s->sem);
	
		/*
		 * HACK ALERT !
		 * To avoid increased complexity in the rest of the code
		 * where full fragment sizes are assumed, we cheat a little
		 * with the start pointer here and don't forget to restore
		 * it later.
		 */
		shiftval = s->fragsize - b->offset;
		b->offset = shiftval;
		b->dma_addr -= shiftval;
		s->bytecount -= shiftval;
		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;
		local_irq_save(flags);
		s->pending_frags++;
		audio_process_dma(s);
		local_irq_restore(flags);
	}

	/* Let's wait for all buffers to complete*/
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&s->wq, &wait);

	/*
         * we remove quiet condition---s->pending_frags
         * Since pending_frags were countdown
         * while frag was been send to dma channel regardless 
         * the frag hass been finish playing
         * by Dean 2009/4/23
         */
        /*
         * To avoid stop dma early when s->dma_tail == s->dma_head
         * but there are still nbfrags  buffers  to be transfer 
         * When this happens, b->dma_ref != 0 
         * by Vincent 2009/05/19
         */
        b = &s->buffers[s->dma_head];
        while (b->dma_ref || s->dma_tail != s->dma_head &&
               !signal_pending(current)) {
                schedule();
                set_current_state(TASK_INTERRUPTIBLE);
        }

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->wq, &wait);

	/* Undo the pointer hack above*/
	if (shiftval) {
		local_irq_save(flags);
		b->dma_addr += shiftval;
		/* ensure sane DMA code behavior if not yet processed*/
		if (b->offset != 0)
			b->offset = s->fragsize;
		local_irq_restore(flags);
	}

	return 0;
}

static int audio_mmap(struct file *file, struct vm_area_struct *vma)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *s;
	unsigned long size, vma_addr;
	int i, ret;

	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if (vma->vm_flags & VM_WRITE) {
		if (!state->wr_ref)
			return -EINVAL; ;
		s = state->output_stream;
	} else if (vma->vm_flags & VM_READ) {
		if (!state->rd_ref)
			return -EINVAL;
		s = state->input_stream;
	} else
		return -EINVAL;

	if (s->mapped)
		return -EINVAL;
	size = vma->vm_end - vma->vm_start;
	if (size != s->fragsize * s->nbfrags)
		return -EINVAL;
	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;
	vma_addr = vma->vm_start;
	for (i = 0; i < s->nbfrags; i++) {
		audio_buf_t *buf = &s->buffers[i];
		if (!buf->master)
			continue;
		ret = remap_pfn_range(vma, vma_addr, (buf->dma_addr) >> PAGE_SHIFT,
				buf->master, vma->vm_page_prot);
		if (ret)
			return ret;
		vma_addr += buf->master;
	}
	s->mapped = 1;

	return 0;
}

static unsigned int audio_poll(struct file *file,
				struct poll_table_struct *wait)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	unsigned int mask = 0;

	DPRINTK("audio_poll(): mode=%s%s\n",
		(file->f_mode & FMODE_READ) ? "r" : "",
		(file->f_mode & FMODE_WRITE) ? "w" : "");

	if (file->f_mode & FMODE_READ) {
		/* Start audio input if not already active*/
		if (!is->active) {
			if (!is->buffers && audio_setup_buf(is))
				return -ENOMEM;
			audio_prime_rx(state);
		}
		poll_wait(file, &is->wq, wait);
	}

	if (file->f_mode & FMODE_WRITE) {
		if (!os->buffers && audio_setup_buf(os))
			return -ENOMEM;
		poll_wait(file, &os->wq, wait);
	}

	if (file->f_mode & FMODE_READ)
		if ((is->mapped && is->bytecount > 0) ||
		    (!is->mapped && compat_sema_count(&is->sem) > 0))
			mask |= POLLIN | POLLRDNORM;

	if (file->f_mode & FMODE_WRITE)
		if ((os->mapped && os->bytecount > 0) ||
		    (!os->mapped && compat_sema_count(&os->sem) > 0))
			mask |= POLLOUT | POLLWRNORM;

	DPRINTK("audio_poll() returned mask of %s%s\n",
		(mask & POLLIN) ? "r" : "",
		(mask & POLLOUT) ? "w" : "");

	return mask;
}

static loff_t audio_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static int audio_set_fragments(audio_stream_t *s, int val)
{
	DPRINTK("+%s\n", __func__);
	if (s->active)
		return -EBUSY;
	if (s->buffers)
		audio_discard_buf(s);
	s->nbfrags = (val >> 16) & 0x7FFF;
	val &= 0xffff;
	if (val < 4)
		val = 4;
	if (s->wmt_dma && val < 11)
		val = 11;
	if (val > 15)
		val = 15;
	s->fragsize = 1 << val;
	if (s->nbfrags < 2)
		s->nbfrags = 2;
	if (s->nbfrags * s->fragsize > 128 * 1024)
		s->nbfrags = 128 * 1024 / s->fragsize;
	if (audio_setup_buf(s))
		return -ENOMEM;
	return val|(s->nbfrags << 16);

}

static int audio_ioctl(struct inode *inode, struct file *file,
			uint cmd, ulong arg)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	long val;

	/* dispatch based on command*/
	switch (cmd) {
	case OSS_GETVERSION:

		DPRINTK("OSS_GETVERSION\n");

		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_GETBLKSIZE:

		DPRINTK("SNDCTL_DSP_GETBLKSIZE\n");

		if (file->f_mode & FMODE_WRITE)
			return put_user(os->fragsize, (int *)arg);
		else
			return put_user(is->fragsize, (int *)arg);

	case SNDCTL_DSP_GETCAPS:

		DPRINTK("SNDCTL_DSP_GETCAPS\n");

		val = DSP_CAP_REALTIME|DSP_CAP_TRIGGER|DSP_CAP_MMAP;

		if (is && os)
			val |= DSP_CAP_DUPLEX;

		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETFRAGMENT:

		DPRINTK("SNDCTL_DSP_SETFRAGMENT\n");

		if (get_user(val, (long *) arg))
			return -EFAULT;

		if (file->f_mode & FMODE_READ) {
			/* Recording*/
			int ret = audio_set_fragments(is, val);

			if (ret < 0)
				return ret;

			ret = put_user(ret, (int *)arg);

			if (ret)
				return ret;
		}
		if (file->f_mode & FMODE_WRITE) {
			/* Playback*/
			int ret = audio_set_fragments(os, val);

			if (ret < 0)
				return ret;

			ret = put_user(ret, (int *)arg);

			if (ret)
				return ret;
		}
		return 0;

	case SNDCTL_DSP_SYNC:

		DPRINTK("SNDCTL_DSP_SYNC\n");

		return audio_sync(file);

	case SNDCTL_DSP_SETDUPLEX:

		DPRINTK("SNDCTL_DSP_SETDUPLEX\n");

		return 0;

	case SNDCTL_DSP_POST:
		DPRINTK("SNDCTL_DSP_POST\n");
	{
		unsigned int reg;


		codec->ops->read(ANALOG_PATH_CONTROL_ADDR, &reg);
		if (reg & BYPASS_ENABLE)
			reg &= ~BYPASS_ENABLE;
		else
			reg |= BYPASS_ENABLE;

		codec->ops->write(ANALOG_PATH_CONTROL_ADDR, reg);
	}
		return 0;

	case SNDCTL_DSP_GETTRIGGER:

		DPRINTK("SNDCTL_DSP_GETTRIGGER\n");

		val = 0;

		if (file->f_mode & FMODE_READ && is->active && !is->stopped)
			val |= PCM_ENABLE_INPUT;

		if (file->f_mode & FMODE_WRITE && os->active && !os->stopped)
			val |= PCM_ENABLE_OUTPUT;

		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:

		DPRINTK("SNDCTL_DSP_SETTRIGGER\n");

		if (get_user(val, (int *)arg))
			return -EFAULT;

		if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				unsigned long flags;
				if (!is->active) {
					if (!is->buffers && audio_setup_buf(is))
						return -ENOMEM;
					audio_prime_rx(state);
				}
				local_irq_save(flags);
				is->stopped = 0;
				audio_process_dma(is);
				local_irq_restore(flags);
			} else {
				audio_stop_dma(is);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			if (val & PCM_ENABLE_OUTPUT) {
				unsigned long flags;
				if (!os->buffers && audio_setup_buf(os))
					return -ENOMEM;
				local_irq_save(flags);
				if (os->mapped && !os->pending_frags) {
					os->pending_frags = os->nbfrags;
					compat_sema_init(&os->sem, 0);
					os->active = 1;
				}
				os->stopped = 0;
				audio_process_dma(os);
				local_irq_restore(flags);
			} else {
				audio_stop_dma(os);
			}
		}
		return 0;

	case SNDCTL_DSP_GETOPTR:

		DPRINTK("SNDCTL_DSP_GETOPTR\n");

	case SNDCTL_DSP_GETIPTR:
		DPRINTK("SNDCTL_DSP_GETIPTR\n");
	{
		count_info inf = { 0, };
		audio_stream_t *s = (cmd == SNDCTL_DSP_GETOPTR) ? os : is;
		int bytecount, offset;
		unsigned long flags;

		if ((s == is && !(file->f_mode & FMODE_READ)) ||
		    (s == os && !(file->f_mode & FMODE_WRITE)))
			return -EINVAL;
		if (s->active) {
			local_irq_save(flags);
			offset = audio_get_dma_pos(s);
			inf.ptr = s->dma_tail * s->fragsize + offset;
			bytecount = s->bytecount + offset;
			s->bytecount = -offset;
			inf.blocks = s->fragcount;
			s->fragcount = 0;
			local_irq_restore(flags);
			if (bytecount < 0)
				bytecount = 0;
			inf.bytes = bytecount;
		}
		return copy_to_user((void *)arg, &inf, sizeof(inf));
	}

	case SNDCTL_DSP_GETOSPACE:

		DPRINTK("SNDCTL_DSP_GETOSPACE\n");

	case SNDCTL_DSP_GETISPACE:
	{
		DPRINTK("SNDCTL_DSP_GETISPACE\n");
		audio_buf_info inf = { 0, };
		audio_stream_t *s = (cmd == SNDCTL_DSP_GETOSPACE) ? os : is;

		if ((s == is && !(file->f_mode & FMODE_READ)) ||
		    (s == os && !(file->f_mode & FMODE_WRITE)))
			return -EINVAL;
		if (!s->buffers && audio_setup_buf(s))
			return -ENOMEM;
		inf.bytes = compat_sema_count(&s->sem) * s->fragsize;
		inf.bytes -= s->buffers[s->usr_head].offset;
		if (inf.bytes < 0)
			inf.bytes = 0;

		inf.fragments = inf.bytes / s->fragsize;
		inf.fragsize = s->fragsize;
		inf.fragstotal = s->nbfrags;
		return copy_to_user((void *)arg, &inf, sizeof(inf));
	}

	case SNDCTL_DSP_NONBLOCK:

		DPRINTK("SNDCTL_DSP_NONBLOCK\n");

		file->f_flags |= O_NONBLOCK;

		return 0;

	case SNDCTL_DSP_RESET:

		DPRINTK("SNDCTL_DSP_RESET\n");

		if (file->f_mode & FMODE_READ) {
			audio_reset(is);
			if (state->need_tx_for_rx) {
				unsigned long flags;
				local_irq_save(flags);
				os->spin_idle = 0;
				local_irq_restore(flags);
			}
		}
		if (file->f_mode & FMODE_WRITE)
			audio_reset(os);

		return 0;

	default:
		DPRINTK("audio_ioctl default\n");
		/*
		 Let the client of this module handle the
		 non generic ioctls
		*/
		return state->client_ioctl(inode, file, cmd, arg);
	}

	return 0;
}

static int audio_release(struct inode *inode, struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;

	DPRINTK("audio_release\n");

	compat_down(&state->sem);

	if (file->f_mode & FMODE_READ) {
		audio_discard_buf(is);
		DMA_FREE(is);
		is->dma_spinref = 0;
		if (state->need_tx_for_rx) {
			os->spin_idle = 0;
			if (!state->wr_ref) {
				DMA_FREE(os);
				os->dma_spinref = 0;
			}
		}
		state->rd_ref = 0;
	}

	if (file->f_mode & FMODE_WRITE) {
		audio_sync(file);
		audio_discard_buf(os);
		if (!state->need_tx_for_rx || !state->rd_ref) {
			DMA_FREE(os);
			os->dma_spinref = 0;
		}
		state->wr_ref = 0;
	}

	if (!AUDIO_ACTIVE(state)) {
		if (state->hw_shutdown)
			state->hw_shutdown(state->data);
	}

	compat_up(&state->sem);

	return 0;
}

static int wmt_audio_attach(struct inode *inode, struct file *file,
				audio_state_t *state)
{
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	int err, need_tx_dma;

	DPRINTK("wmt_audio_attach\n");

	compat_down(&state->sem);

	/* access control*/
	err = -ENODEV;
	if ((file->f_mode & FMODE_WRITE) && !os)
		goto out;
	if ((file->f_mode & FMODE_READ) && !is)
		goto out;
	err = -EBUSY;
	if ((file->f_mode & FMODE_WRITE) && state->wr_ref)
		goto out;
	if ((file->f_mode & FMODE_READ) && state->rd_ref)
		goto out;
	err = -EINVAL;
	if ((file->f_mode & FMODE_READ) && state->need_tx_for_rx && !os)
		goto out;

	/* request DMA channels*/
	need_tx_dma = ((file->f_mode & FMODE_WRITE) ||
		       ((file->f_mode & FMODE_READ) && state->need_tx_for_rx));
	if (state->wr_ref || (state->rd_ref && state->need_tx_for_rx))
		need_tx_dma = 0;
	if (need_tx_dma) {
		err = DMA_REQUEST(os, audio_dma_callback);
		if (err)
			goto out;
	}
	if (file->f_mode & FMODE_READ) {
		err = DMA_REQUEST(is, audio_dma_callback);
		if (err) {
			if (need_tx_dma)
				DMA_FREE(os);
			goto out;
		}
	}

	/* now complete initialisation*/
	if (!AUDIO_ACTIVE(state)) {
		if (state->hw_init)
			state->hw_init(state->data);
	}

	if ((file->f_mode & FMODE_WRITE)) {
		state->wr_ref = 1;
		audio_reset(os);
		os->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		os->nbfrags = AUDIO_NBFRAGS_DEFAULT;
		os->mapped = 0;
		init_waitqueue_head(&os->wq);
	}
	if (file->f_mode & FMODE_READ) {
		state->rd_ref = 1;
		audio_reset(is);
		is->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		is->nbfrags = AUDIO_NBFRAGS_DEFAULT;
		is->mapped = 0;
		init_waitqueue_head(&is->wq);
	}

	file->private_data      = state;
	file->f_op->release     = audio_release;
	file->f_op->write       = audio_write;
	file->f_op->read        = audio_read;
	file->f_op->mmap        = audio_mmap;
	file->f_op->poll        = audio_poll;
	file->f_op->ioctl       = audio_ioctl;
	file->f_op->llseek      = audio_llseek;
	err = 0;

out:
	compat_up(&state->sem);
	return err;
}

/* static int i2s_mixer_ioctl(int cmd, void *arg)*/
static int i2s_mixer_ioctl(int cmd, ulong arg)
{
	int val, nr = _IOC_NR(cmd), ret = -EINVAL;

	if (_IOC_TYPE(cmd) != 'M')
		return ret;

	if (cmd == SOUND_MIXER_INFO) {

		DPRINTK("SOUND_MIXER_INFO\n");

		struct mixer_info mi;

		strncpy(mi.id, "CODEC", sizeof(mi.id));
		strncpy(mi.name, "TI CODEc", sizeof(mi.name));
		mi.modify_counter = codec->mod;
		return copy_to_user((void *)arg, &mi, sizeof(mi)) ? -EFAULT : 0;
	}

		ret = 0;
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

		switch (nr) {
		case SOUND_MIXER_PCM:
		{
			unsigned short left, right;
			unsigned int reg;

			DPRINTK("SOUND_MIXER_PCM %x \n", val);

			left = (unsigned short)(val & 0xff);            /* left channel vol*/
			right = (unsigned short)((val >> 8) & 0xff);    /* right channel vol*/
			codec->volume = val;
			codec->mod++;

			/* max:100, min:0*/
			if (left > 100)
				left = 100;
			if (right > 100)
				right = 100;

			/* left headphone volume*/
			reg = left;
			reg &= 0x7F; /* mask gain change zero cross*/

			/* right headphone volume*/
			reg = right;
			reg &= 0x7F; /* mask gain change zero cross*/
			codec->ops->set_out_volume(reg);

			break;
		}
		case SOUND_MIXER_VOLUME:
		{
			unsigned short left, right;
			unsigned int reg;

			DPRINTK("SOUND_MIXER_VOLUME %d \n", val);

			left = (unsigned short)(val & 0xff);            /* left channel vol*/
			right = (unsigned short)((val >> 8) & 0xff);    /* right channel vol*/
			codec->volume = val;
			codec->mod++;

			/* max:100, min:0*/
			if (left > 100)
				left = 100;
			if (right > 100)
				right = 100;

			/* left headphone volume*/
			codec->ops->read(HEADPHONE_LEFT_VOLUME_ADDR, &reg);
			reg = (reg & ~HLHV_MASK) | left;
			reg &= 0x7F; /* mask gain change zero cross*/

			/* right headphone volume*/
			codec->ops->read(HEADPHONE_RIGHT_VOLUME_ADDR, &reg);
			reg = (reg & ~HRHV_MASK) | right;
			reg &= 0x7F; /* mask gain change zero cross*/
			codec->ops->set_out_volume(reg);

			break;
		}
		/*
		   FIXME : Following bass and treble need to be verified.
		*/
		case SOUND_MIXER_BASS:
		{
			/*unsigned short reg;*/

			DPRINTK("SOUND_MIXER_BASS\n");

			/* FIXME: I assume val should between 0 and 100*/
			if (val > 100)
				val = 100;
			if (val < 0)
				val = 0;

			codec->bass = val;
			codec->mod++;

			break;
		}
		case SOUND_MIXER_TREBLE:
		{
			/*unsigned short reg;*/

			DPRINTK("SOUND_MIXER_TREBLE\n");

			/* FIXME: I assume val should between 0 and 100*/
			if (val > 100)
				val = 100;
			if (val < 0)
				val = 0;

			codec->treble = val;
			codec->mod++;

			break;
		}
		case SOUND_MIXER_LINE:
		{
			unsigned short left, right;
			unsigned int reg;

			DPRINTK("SOUND_MIXER_LINE %d\n", val);

			left = (unsigned short)(val & 0xff);            /* left channel vol*/
			right = (unsigned short)((val >> 8) & 0xff);    /* right channel vol*/
			codec->line = val;
			codec->mod++;

			/* max:100, min:0*/
			if (left > 100)
				left = 100;
			if (right > 100)
				right = 100;

			/* Convert vol degree to attenuation.*/
			left =  (LLIV_MAX - LLIV_MIN) * left / 100 + LLIV_MIN;/* right channel atte*/
			right = (LRIV_MAX - LRIV_MIN) * right / 100 + LRIV_MIN;/* left line volume*/

			codec->ops->read(LINE_LEFT_VOLUME_ADDR, &reg);
			reg = (reg & ~LLIV_MASK) | left;
			codec->ops->write(LINE_LEFT_VOLUME_ADDR, reg);

			/* right line volume*/
			codec->ops->read(LINE_RIGHT_VOLUME_ADDR, &reg);
			reg = (reg & ~LRIV_MASK) | right;
			codec->ops->write(LINE_RIGHT_VOLUME_ADDR, reg);

			break;
		}
		case SOUND_MIXER_MIC:

			DPRINTK("SOUND_MIXER_MIC %d\n", val);

			codec->mic = val;
			codec->mod++;
			/* TODO*/
			break;

		case SOUND_MIXER_RECSRC:
		{
			unsigned int reg;

			DPRINTK("SOUND_MIXER_RECSRC\n");
			codec->mod++;

			codec->ops->read(ANALOG_PATH_CONTROL_ADDR, &reg);
			if (val & SOUND_MASK_MIC) {
				codec->recsrc = SOUND_MASK_MIC;
				reg |= MIC_MODE;
				reg &= ~MIC_MUTE_ENABLE;
				codec->ops->select_recsrc(0);
			} else {
				codec->recsrc = SOUND_MASK_LINE;
				reg &= ~MIC_MODE;
				reg |= MIC_MUTE_ENABLE + MIC_BOOST_ENABLE;
				codec->ops->select_recsrc(1);
			}
			codec->ops->write(ANALOG_PATH_CONTROL_ADDR, reg);
			break;
		}
		default:
			DPRINTK("i2s_mixer_ioctl default\n");
			ret = -EINVAL;
		}
		goto out;
	}
/* TODO: figure out following 2*/
#define REC_MASK        (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK        (REC_MASK | SOUND_MASK_VOLUME | SOUND_MASK_BASS | SOUND_MASK_TREBLE|SOUND_MASK_PCM)

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		int nr = _IOC_NR(cmd);
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_PCM:
			DPRINTK("SOUND_MIXER_PCM_R\n");
			val = codec->volume;
			break;
		case SOUND_MIXER_VOLUME:
			DPRINTK("SOUND_MIXER_VOLUME_R\n");
			val = codec->volume;
			break;
		case SOUND_MIXER_BASS:
			DPRINTK("SOUND_MIXER_BASE_R\n");
			val = codec->bass;
			break;
		case SOUND_MIXER_TREBLE:
			val = codec->treble;
			break;
		case SOUND_MIXER_LINE:
			DPRINTK("SOUND_MIXER_LINE_R\n");
			val = codec->line;
			break;
		case SOUND_MIXER_MIC:
			DPRINTK("SOUND_MIXER_MIC_R\n");
			val = codec->mic;
			break;
		case SOUND_MIXER_RECSRC:
			DPRINTK("SOUND_MIXER_RECSRC_R\n");
			val = codec->recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			DPRINTK("SOUND_MIXER_RECMASK_R\n");
			val = REC_MASK;
			break;
		case SOUND_MIXER_DEVMASK:
			DPRINTK("SOUND_MIXER_DEVMASK_R\n");
			val = DEV_MASK;
			break;
		case SOUND_MIXER_CAPS:
			DPRINTK("SOUND_MIXER_CAPS_R\n");
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			DPRINTK("SOUND_MIXER_STEREODEVS_R\n");
			val = 0;
			break;
		default:
			val = 0;
			ret = -EINVAL;
			break;
		}

		if (ret == 0) {
			ret = put_user(val, (int *)arg);
			DPRINTK("put user return %d val %d=%d\n", ret, val, *((int *) arg));
		}
	}
out:
	return ret;
}

/*
 * Mixer interface
 */
static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	return i2s_mixer_ioctl(cmd, arg);
}

static struct file_operations wmt_mixer_fops = {
	.owner  = THIS_MODULE,
	.ioctl  = mixer_ioctl,
};


static void wmt_audio_init(void *dummy)
{
	DPRINTK("wmt_audio_init\n");
	i2s.init();
	codec->ops->init();
}

/*
 * Shutdown the audio driver.
 */
static void wmt_audio_shutdown(void *dummy)
{
    DPRINTK("wmt_audio_shutdown \n");
    codec->ops->exit();

    /* Disable CODEC Digital Interface Activation*/
    codec->ops->write(DIGITAL_ACTIVATION_ADDR, 0);

    /* Disable CODEC Power*/
    codec->ops->write(POWERDOWN_CONTROL_ADDR, 0xFF);
}

static int wmt_set_fmt(int rd , int fmt)
{
	if (fmt == AFMT_U8) {
		pcm_fmt = fmt;
		i2s_set_tx_resolution(8);
		i2s_set_rx_resolution(8);
	} else if (fmt == AFMT_S16_LE) {
		pcm_fmt = fmt;
		i2s_set_tx_resolution(16);
		i2s_set_rx_resolution(16);
	} else {
		printk("WMT3426 did not support this format[0x%8X]\n", fmt);
		return -EINVAL;
	}
	return pcm_fmt;
}
static int wmt_audio_ioctl(struct inode *inode, struct file *file,
				uint cmd, ulong arg)
{
	long val = 0;
	int ret = 0;

	/* These are machine dependent ioctls.*/
	switch (cmd) {
	case SNDCTL_DSP_STEREO:

		DPRINTK("SNDCTL_DSP_STEREO\n");

		/* no support both stereo and mono.*/
		ret = get_user(val, (long *) arg);

		if (ret)
			return ret;

		if (val == AUDIO_MODE_MONO) {
			i2s_set_tx_channels(AUDIO_MODE_MONO);
			i2s_set_rx_channels(AUDIO_MODE_MONO);
		} else {
			i2s_set_tx_channels(AUDIO_MODE_STEREO);
			i2s_set_rx_channels(AUDIO_MODE_STEREO);
		}

		DPRINTK("SNDCTL_DSP_STEREO %s \n", (val == 0) ?
			"mono" : "stereo");

		return put_user(val, (int *) arg);

	case SNDCTL_DSP_CHANNELS:

		DPRINTK("SNDCTL_DSP_CHANNELS\n");

		if (get_user(val, (long *) arg))
			return -EFAULT;

		if (val != 1 && val != 2)
			return -EINVAL;
		channels = val;

		return put_user(val, (int *) arg);

	case SOUND_PCM_READ_CHANNELS:

		DPRINTK("SOUND_PCM_READ_CHANNELS\n");
		return put_user(2, (long *) arg);

	case SNDCTL_DSP_SPEED:

		DPRINTK("SNDCTL_DSP_SPEED\n");

		if (get_user(val, (long *) arg))
			return -EFAULT;
		sample_rate = val;
		codec->ops->set_sample_rate(sample_rate);
		return put_user(sample_rate, (long *) arg);

		break;

	case SOUND_PCM_READ_RATE:

		DPRINTK("SOUND_PCM_READ_RATE\n");

		val = sample_rate;
		return put_user(val, (long *) arg);

	case SNDCTL_DSP_SETFMT:

		DPRINTK("SNDCTL_DSP_SETFMT\n");

		if (get_user(val, (long *) arg))
			return -EFAULT;
		if (val != AFMT_QUERY) {
			ret = wmt_set_fmt(file->f_mode, val);
			if (ret == -EINVAL)
				return -EINVAL;
		}
		val = pcm_fmt;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_GETFMTS:

		DPRINTK("SNDCTL_DSP_GETFMTS\n");

		/**/
		/* support unsigned 8-bit and signed 16-bit.*/
		/**/
		return put_user(AFMT_S16_LE, (int *)arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs)*/
		return mixer_ioctl(inode, file, cmd, arg);
	}

	return ret;
}

static audio_stream_t output_stream = {
	.id             = "i2s_out",
	.dma_dev        = I2S_TX_DMA_REQ,
	.dmach          = NULL_DMA,
	.direction      = 1,
};

static audio_stream_t input_stream = {
	.id             = "i2s_in",
	.dma_dev        = I2S_RX_DMA_REQ,
	.dmach          = NULL_DMA,
	.direction      = 0,
};

static audio_state_t audio_state = {
	.output_stream  = &output_stream,
	.input_stream   = &input_stream,
	.need_tx_for_rx = 0,
	.hw_init        = wmt_audio_init,
	.hw_shutdown    = wmt_audio_shutdown,
	.client_ioctl   = wmt_audio_ioctl,
	.sem            = __COMPAT_MUTEX_INITIALIZER(audio_state.sem),
};

static int wmt_audio_open(struct inode *inode, struct file *file)
{
	return wmt_audio_attach(inode, file, &audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to wmt_audio_attach().
 */
static struct file_operations wmt_audio_fops = {
	.owner  = THIS_MODULE,
	.open   = wmt_audio_open,
};


static int audio_dev_id, mixer_dev_id;


#ifdef CONFIG_PM
static int wmt_audio_suspend(struct device *dev, u32 state)
{
	audio_state_t *s = &audio_state;
	audio_stream_t *is = s->input_stream;
	audio_stream_t *os = s->output_stream;
	int stopstate;

	if (is && CHK_DMACH(is->dmach)) {
		i2s.regs->RCR &= ~ RCR_RFADE ;
                udelay(5);
		stopstate = is->stopped;
		audio_stop_dma(is);
	//	DMA_CLEAR(is);
		is->dma_spinref = 0;
		is->stopped = stopstate;
	}
	if (os && CHK_DMACH(os->dmach)) {
		i2s.regs->TCR &= ~ TCR_TFADE ;
                udelay(5);
	//	wmt_dump_dma_regs(os->dmach);//vincent
		stopstate = os->stopped;
		audio_stop_dma(os);
	//	wmt_dump_dma_regs(os->dmach);//vincent
	//	DMA_CLEAR(os);
		os->dma_spinref = 0;
		os->stopped = stopstate;
	}
	i2s_suspend();
	if (AUDIO_ACTIVE(s) && s->hw_shutdown)
		s->hw_shutdown(s->data);


	return 0;
}

static int wmt_audio_resume(struct device *_dev)
{
	audio_state_t *s = &audio_state;
	audio_stream_t *is = s->input_stream;
	audio_stream_t *os = s->output_stream;

	if (AUDIO_ACTIVE(s) && s->hw_init)
		s->hw_init(s->data);


	i2s_resume();
	if (os && CHK_DMACH(os->dmach)) {
		//DMA_RESET(os);
		wmt_setup_dma(os->dmach, os->dma_cfg) ;
		//audio_process_dma(os);
		wmt_resume_dma(os->dmach) ;
	//	wmt_dump_dma_regs(os->dmach);//vincent
	}

	if (is && CHK_DMACH(is->dmach)) {
	//	DMA_RESET(is);
		wmt_setup_dma(is->dmach, is->dma_cfg) ;
		wmt_resume_dma(is->dmach) ;
	//	audio_process_dma(is);
	}
	return 0;
}
#else
#define wmt_audio_suspend    NULL
#define wmt_audio_resume     NULL
#endif

static int wmt_audio_probe(struct device *_dev)
{

	DPRINTK("Enter wmt_audio_probe\r\n");

	codec = codec_attach();

	if (!codec)
		return -ENODEV;

	output_stream.dev = _dev;
	output_stream.dma_cfg = dma_device_cfg_table[I2S_TX_DMA_REQ],
	input_stream.dev = _dev;
	input_stream.dma_cfg = dma_device_cfg_table[I2S_RX_DMA_REQ],

	/**/
	/* register devices*/
	/**/
	audio_dev_id = register_sound_dsp(&wmt_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&wmt_mixer_fops, -1);

	DPRINTK(KERN_INFO "Sound: WMT I2S: dsp id %d mixer id %d\n",
		audio_dev_id, mixer_dev_id);

	return 0;
}

static int wmt_audio_remove(struct device *_dev)
{
	DPRINTK("wmt_audio_remove\n");

	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);
	codec_detach(codec);
	return 0;
}

static struct device_driver wmt_audio_driver = {
	.name           = "i2s",
	.bus            = &platform_bus_type,
	.probe          = wmt_audio_probe,
	.remove         = wmt_audio_remove,
	.suspend        = wmt_audio_suspend,
	.resume         = wmt_audio_resume
};

static int __init wmt_i2s_init(void)
{
	DPRINTK("wmt_i2s_init\r\n");

	return driver_register(&wmt_audio_driver);
}

static void __exit wmt_i2s_exit(void)
{
	DPRINTK("wmt_i2s_exit\r\n");

	return driver_unregister(&wmt_audio_driver);
}

module_init(wmt_i2s_init);
module_exit(wmt_i2s_exit);

MODULE_AUTHOR("WMT SW Team");
MODULE_DESCRIPTION("WMT I2S Audio Driver");
MODULE_LICENSE("GPL");

