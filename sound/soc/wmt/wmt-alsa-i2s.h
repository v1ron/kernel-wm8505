/*++
	sound/arm/wmt_i2s.h

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
#ifndef __WMT_I2S_AUDIO_H
#define __WMT_I2S_AUDIO_H

#include <linux/config.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <linux/platform_device.h>
#include <mach/dma.h>
#include <mach/i2s_alsa.h>
/*
 * Buffer Management
 */
typedef struct {
	int offset;		/* current offset */
	char *data;		/* points to actual buffer */
	dma_addr_t dma_addr;	/* physical buffer address */
	int dma_ref;		/* DMA refcount */
	int master;		/* owner for buffer allocation, contain size when true */
} audio_buf_t;

typedef struct {
	char *id;		/* identification string */
	struct device *dev;	/* device */
	audio_buf_t *buffers;	/* pointer to audio buffer structures */
	int stream_id;		/* numeric identification */
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
	struct compat_semaphore sem;	/* account for fragment usage */
	wait_queue_head_t wq;	/* for poll */
	int period;		/* current transfer period */		//????
	int periods;		/* current count of periods registerd in the DMA engine */	//????
	struct snd_pcm_substream *stream;	/* the pcm stream */	//????
	spinlock_t dma_lock;	/* for locking in DMA operations */	//????
	int dma_spinref;	/* DMA is spinning */
	int mapped:1;		/* mmap()'ed buffers */
	int active:1;		/* actually in progress */
	int stopped:1;		/* might be active but stopped */
	int direction:1;        /* DMA transfer direction 1:out */
	int spin_idle:1;	/* have DMA spin on zeros when idle */
	int wmt_dma:1;	/* DMA handled by WMT */
} audio_stream_t;

#define NULL_DMA                ((dmach_t)(-1))

/*
 * State structure for one instance
 */
typedef struct {
	audio_stream_t *output_stream;
	audio_stream_t *input_stream;
	int rd_ref:1;		/* open reference for recording */
	int wr_ref:1;		/* open reference for playback */
	int need_tx_for_rx:1;	/* if data must be sent while receiving */
	void *data;
	void (*hw_init)(void *);
	void (*hw_shutdown)(void *);
	int (*client_ioctl)(struct inode *, struct file *, uint, ulong);
	struct compat_semaphore sem;	/* to protect against races in attach() */
} audio_state_t;

/**********************************************************************************************/

struct audio_stream_a {
	char *id;		/* identification string */
	int stream_id;		/* numeric identification */
	dmach_t dmach;	        /* DMA channel number */
	struct dma_device_cfg_s dma_cfg;   /* DMA device config */
	int dma_dev;		/* dma number of that device */
//	int *lch;		/* Chain of channels this stream is linked to */
//	char started;		/* to store if the chain was started or not */
	int dma_q_head;		/* DMA Channel Q Head */
	int dma_q_tail;		/* DMA Channel Q Tail */
	char dma_q_count;	/* DMA Channel Q Count */
	int active:1;		/* we are using this stream for transfer now */
	int period;		/* current transfer period */
	int periods;		/* current count of periods registerd in the DMA engine */
	spinlock_t dma_lock;	/* for locking in DMA operations */
	struct snd_pcm_substream *stream;	/* the pcm stream */
	unsigned linked:1;	/* dma channels linked */
	int offset;		/* store start position of the last period in the alsa buffer */
//	int (*hw_start)(void);  /* interface to start HW interface, e.g. McBSP */
//	int (*hw_stop)(void);   /* interface to stop HW interface, e.g. McBSP */
};

/*
 * 
 */
struct snd_card_wmt_codec {
	struct snd_card *card;
	struct snd_pcm *pcm;
	long samplerate;
	unsigned int direction;		/* same as member of *s, used for suspend/resume */
	int dmach;		/* same as member of *s, used for suspend/resume */
	int active:1;				/* same as member of *s, used for suspend/resume */
	struct dma_device_cfg_s *dma_cfg;	/* same as member of *s, used for suspend/resume */
	struct audio_stream_a s[2];	/* playback & capture */
};


#endif /* __WMT_I2S_AUDIO_H */
