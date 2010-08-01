/*++
	sound/soc/wmt/wmt-alsa-ac97.h

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
		2009/06    First Version
--*/
#ifndef __WMT_AC97_AUDIO_H
#define __WMT_AC97_AUDIO_H

#include <linux/config.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <linux/platform_device.h>
#include <mach/dma.h>
#include <mach/ac97_alsa.h>


#define NULL_DMA                ((dmach_t)(-1))


/**********************************************************************************************/

struct audio_stream_a {
	char *id;		/* identification string */
	int stream_id;		/* numeric identification */
	dmach_t dmach;	        /* DMA channel number */
	struct dma_device_cfg_s dma_cfg;   /* DMA device config */
	int dma_dev;		/* dma number of that device */
	int active:1;		/* we are using this stream for transfer now */
	int period;		/* current transfer period */
	int periods;		/* current count of periods registerd in the DMA engine */
	spinlock_t dma_lock;	/* for locking in DMA operations */
	struct snd_pcm_substream *stream;	/* the pcm stream */
	int offset;		/* store start position of the last period in the alsa buffer */
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

#endif /* __WMT_AC97_AUDIO_H */
