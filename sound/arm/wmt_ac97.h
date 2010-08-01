/*++
	sound/arm/wmt_ac97.h

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
#ifndef __WMT_AC97_AUDIO_H
#define __WMT_AC97_AUDIO_H


/*
 * Buffer Management
 */
struct audio_buf_s{
	int offset;		/* current offset */
	char *data;		/* points to actual buffer */
	dma_addr_t dma_addr;	/* physical buffer address */
	int dma_ref;		/* DMA refcount */
	int master;		/* owner for buffer allocation, contain size when true */
};

struct audio_stream_s{
	char *id;		/* identification string */
	struct device *dev;	/* device */
	struct audio_buf_s *buffers;	/* pointer to audio buffer structures */
	u_int usr_head;		/* user fragment index */
	u_int dma_head;		/* DMA fragment index to go */
	u_int dma_tail;		/* DMA fragment index to complete */
	u_int fragsize;		/* fragment i.e. buffer size */
	u_int nbfrags;		/* nbr of fragments i.e. buffers */
	u_int pending_frags;	/* Fragments sent to DMA */
	enum dma_device_e dma_dev;   /* device identifier for DMA */
	struct dma_device_cfg_s dma_cfg;   /* DMA device config */
	dmach_t dmach;	        /* DMA channel number */
	int bytecount;		/* nbr of processed bytes */
	int fragcount;		/* nbr of fragment transitions */
	struct compat_semaphore sem; /* account for fragment usage */
	wait_queue_head_t wq;	/* for poll */
	int dma_spinref;	/* DMA is spinning */
	int mapped:1;		/* mmap()'ed buffers */
	int active:1;		/* actually in progress */
	int stopped:1;		/* might be active but stopped */
	int direction:1;        /* DMA transfer direction 1:out */
	int spin_idle:1;	/* have DMA spin on zeros when idle */
	int wmt_dma:1;	        /* DMA handled by WMT */
};

#define NULL_DMA                ((dmach_t)(-1))

/*
 * State structure for one instance
 */
struct audio_state_s{
	struct audio_stream_s *output_stream;
	struct audio_stream_s *input_stream;
	int rd_ref:1;		/* open reference for recording */
	int wr_ref:1;		/* open reference for playback */
	int need_tx_for_rx:1;	/* if data must be sent while receiving */
	void *data;
	void (*hw_init)(void *);
	void (*hw_shutdown)(void *);
	int (*client_ioctl)(struct inode *, struct file *, uint, ulong);
	struct compat_semaphore sem;/* to protect against races in attach() */
};

extern void ac97_reg_backup(void);
extern void ac97_reg_restore(void);


#endif /* __WMT_AC97_AUDIO_H */
