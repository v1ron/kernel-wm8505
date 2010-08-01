/*
 *       sound/soc/wmt/wmt-alsa-ac97.c
 *       Alsa AC97 Driver for WMT 8510 Chip
 *
 *       Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.
 *
 *       This program is free software: you can redistribute it and/or modify it under the
 *       terms of the GNU General Public License as published by the Free Software Foundation,
 *       either version 2 of the License, or (at your option) any later version.
 *
 *       This program is distributed in the hope that it will be useful, but WITHOUT
 *       ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *       PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *       You should have received a copy of the GNU General Public License along with
 *       this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *       WonderMedia Technologies, Inc.
 *       10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
 *
 *       History:
 *               2009/06    First Version
 * 
 */
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
/*#include <linux/device.h>*/
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
#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/semaphore.h>
#include <asm/dma.h>
#include <asm/mach-types.h>
#include <asm/sizes.h>

#include <mach/ac97_alsa.h>
#include <mach/hardware.h>	/*ac97_reg*/
#include <mach/vt1613.h>

#include "wmt-alsa-ac97.h"

/*#define DEBUG*/
/*#define REC_DEBUG*/
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define AUDIO_VOLUME_DEFAULT            0x1F
#define AUDIO_BYTE(r)           ((((r)>>12)&0x1)?(1):(2))       /* 1= 1 byte, 0 = 2 bytes */
#define AUDIO_CHANNEL(r)        ((((r)>>13)&0x1)?(1):(2))       /* 1= 1 channel, 0 = 2 channels */

#ifdef DEBUG
extern void vt1613_reg_dump(void);
#endif

static char *id	= NULL;	
static struct snd_card_wmt_codec 	*alsa_codec		= NULL;
static struct wmt_alsa_codec_config	*alsa_codec_config	= NULL;
extern wmt_ac97_t ac97;


static void wmt_ac97_enable(void)
{
	REG32_VAL(0xd8130000+0x250) |= 0x00080000;  /*enable pmc clock*/
	udelay(2);	/* temp delay for VT3426, should be removed */
}


void ac97_reg_dump(void)
{
	wmt_ac97_enable();

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


static struct ac97_regs_s pbackup_ac97_reg;
void ac97_reg_backup(void)
{
	wmt_ac97_enable();	/*Bit19  enable ac97 clk*/

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

void ac97_reg_restore(void)
{
	wmt_ac97_enable();	/*Bit19  enable ac97 clk*/
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


static void wmt_alsa_audio_init(struct snd_card_wmt_codec *wmt_alsa)
{
	/* Setup DMA stuff */
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].id = "WMT PCM out";
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].stream_id = SNDRV_PCM_STREAM_PLAYBACK;
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dmach = AC97_PCM_TX_DMA_REQ;
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dma_dev = AC97_PCM_TX_DMA_REQ;
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dma_cfg = dma_device_cfg_table[AC97_PCM_TX_DMA_REQ],

	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].id = "WMT PCM in";
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].stream_id = SNDRV_PCM_STREAM_CAPTURE;
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dmach = AC97_PCM_RX_DMA_REQ;
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dma_cfg = dma_device_cfg_table[AC97_PCM_RX_DMA_REQ],
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dma_dev = AC97_PCM_RX_DMA_REQ;
	DPRINTK("s[playback] pointer: %d, s[capture] pointer: %d\n", &wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK], &wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE]);
}


/* audio_setup_dma()
 *
 * Just a simple funcion to control DMA setting for AC'97 audio
 */
static void ac97_audio_setup_dma(struct audio_stream_a *s, int strream_id)
{
	unsigned int channel, byte;

	DPRINTK("strream_id=%d\n", strream_id);
	if (strream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		/* From memory to device */
		byte = AUDIO_BYTE(ac97.regs->PTFC);
		channel = AUDIO_CHANNEL(ac97.regs->PTFC);
	
	        DPRINTK("Playback: byte=%u, channel=%u\n", AUDIO_BYTE(ac97.regs->PTFC), AUDIO_CHANNEL(ac97.regs->PTFC) );

		switch (byte * channel) {
		case 1:
			s->dma_cfg.DefaultCCR = AC97_PCM_TX_DMA_8BITS_CFG ;     /* setup 1 bytes*/
			break ;
		case 2:
			s->dma_cfg.DefaultCCR = AC97_PCM_TX_DMA_16BITS_CFG ;    /* setup 2 bytes*/
			break ;
		case 4:
			s->dma_cfg.DefaultCCR = AC97_PCM_TX_DMA_32BITS_CFG ;    /* setup 4 byte*/
			break ;
		}
	} else {
		/* From device to memory */
		byte = AUDIO_BYTE(ac97.regs->PRFC);
		channel = AUDIO_CHANNEL(ac97.regs->PRFC);

		DPRINTK("Capture: byte=%u, channel=%u\n", AUDIO_BYTE(ac97.regs->PRFC), AUDIO_CHANNEL(ac97.regs->PRFC) );
		switch (byte * channel) {
		case 1:
			s->dma_cfg.DefaultCCR = AC97_PCM_RX_DMA_8BITS_CFG ;     /* setup 1 bytes*/
			break ;
		case 2:
			s->dma_cfg.DefaultCCR = AC97_PCM_RX_DMA_16BITS_CFG ;    /* setup 2 bytes*/
			break ;
		case 4:
			s->dma_cfg.DefaultCCR = AC97_PCM_RX_DMA_32BITS_CFG ;    /* setup 4 byte*/
			break ;
		}
	}
	DPRINTK("s pointer: %d. audio dma %d cfg.DefaultCCR 0x%x \n", s, s->dmach, s->dma_cfg.DefaultCCR);
	DPRINTK("cfg.ChunkSize 0x%x \n", s->dma_cfg.ChunkSize);
	wmt_setup_dma(s->dmach, s->dma_cfg) ; 
}

/* 
 * DMA functions 
 * 
 */
static int ac97_audio_dma_request(struct audio_stream_a *s, void (*callback) (void *))
{
	int err;

	DPRINTK("s pointer: %d, dmach: %d, id: %s, dma_dev: %d\n", s, s->dmach, s->id, s->dma_dev);
	err = wmt_request_dma(&s->dmach, s->id, s->dma_dev, callback, s);
	if (err < 0)
		printk(KERN_ERR "Unable to grab audio dma 0x%x\n", s->dmach);
	return err;
}

static int ac97_audio_dma_free(struct audio_stream_a *s)
{
	int err = 0;

	wmt_free_dma(s->dmach);
	s->dmach = NULL_DMA;
	return err;
}

/*
 *  This function should calculate the current position of the dma in the
 *  buffer. It will help alsa middle layer to continue update the buffer.
 *  Its correctness is crucial for good functioning.
 */
static u_int ac97_audio_get_dma_pos(struct audio_stream_a *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;
	dma_addr_t ptr;

	ptr = wmt_get_dma_pos(s->dmach);
/*	DPRINTK("ptr=0x%x, dma_area_phy=0x%x\n", ptr, __virt_to_phys((dma_addr_t)runtime->dma_area));*/
	offset = bytes_to_frames(runtime, ptr-__virt_to_phys((dma_addr_t)runtime->dma_area));
/*	DPRINTK("offset: %d\n", offset);*/

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;

}

/*
 * stop the dma and clear the dma ptrs
 */
static void ac97_audio_stop_dma(struct audio_stream_a *s)
{
	unsigned long flags;

	DPRINTK("Enter\n");
	local_irq_save(flags);
	s->active = 0;
	s->period = 0;
	s->periods = 0;
	alsa_codec->active = 0;
	wmt_stop_dma(s->dmach);
	wmt_clear_dma(s->dmach);	
	local_irq_restore(flags);

}


/*
 *  Main dma routine, requests dma according where you are in main alsa buffer
 */
static void ac97_audio_process_dma(struct audio_stream_a *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;
	dma_addr_t dma_base;
	int ret;

	/*DPRINTK("s: %d, dmach: %d. active: %d\n", s, s->dmach, s->active);*/
	if (s->active) {
		substream = s->stream;
		runtime = substream->runtime;
		dma_size = frames_to_bytes(runtime, runtime->period_size);
/*		DPRINTK("frame_bits=%d, period_size=%d, dma_size 1=%d\n", runtime->frame_bits, runtime->period_size, dma_size); */
		if (dma_size > MAX_DMA_SIZE)		//zhf-????
			dma_size = CUT_DMA_SIZE;
		offset = dma_size * s->period;
		dma_base = __virt_to_phys((dma_addr_t)runtime->dma_area);
/*
		DPRINTK("dma address: 0x%x\n", dma_base+offset );
		DPRINTK("hw_ptr_interrupt: 0x%x, state: %d, hwptr: %u, applptr: %u, avail_min: %u\n", 
			runtime->hw_ptr_interrupt, runtime->status->state, runtime->status->hw_ptr,
			runtime->control->appl_ptr, runtime->control->avail_min);
		DPRINTK("dmach: %u, dma_addr: %x, dma_size: %u\n", s->dmach, dma_base+offset, dma_size);
*/
		ret = wmt_start_dma(s->dmach, dma_base+offset, 0, dma_size);
		if (ret) {
			printk(KERN_ERR  "audio_process_dma: cannot queue DMA buffer (%i)\n", ret);
			return;
		}

		s->period++;
		s->period %= runtime->periods;
		s->periods++;
		s->offset = offset;
	}
}


/* 
 *  This is called when dma IRQ occurs at the end of each transmited block
 */
static void ac97_audio_dma_callback(void *data)
{
	struct audio_stream_a *s = data;

	/* 
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	spin_lock(&s->dma_lock);
	if (s->periods > 0) 
		s->periods--;
	
	ac97_audio_process_dma(s);
	spin_unlock(&s->dma_lock);
}
/* 
 * Alsa section
 * PCM settings and callbacks
 */
static int snd_wmt_alsa_trigger(struct snd_pcm_substream * substream, int cmd)
{
	struct snd_card_wmt_codec *chip = snd_pcm_substream_chip(substream);
	int stream_id = substream->pstr->stream;
	struct audio_stream_a *s = &chip->s[stream_id];
	int err = 0;
	
	DPRINTK("Enter, cmd=%d\n", cmd);
	/* note local interrupts are already disabled in the midlevel code */
	spin_lock(&s->dma_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* requested stream startup */
		s->active = 1;
		alsa_codec->active = 1;
		alsa_codec->direction = stream_id;
		alsa_codec->dmach = s->dmach;
		alsa_codec->dma_cfg = &(s->dma_cfg);
/*	wmt_dump_dma_regs(s->dmach);*/
		ac97_audio_process_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* requested stream shutdown */
		ac97_audio_stop_dma(s);
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock(&s->dma_lock);
	
	return err;
}

static int snd_wmt_alsa_prepare(struct snd_pcm_substream * substream)
{
	struct snd_card_wmt_codec *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	struct audio_stream_a *s = &chip->s[substream->pstr->stream];
	
	/* set requested samplerate */
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) 
		alsa_codec_config->codec_set_dac_samplerate(runtime->rate);
	else
		alsa_codec_config->codec_set_adc_samplerate(runtime->rate);
	chip->samplerate = runtime->rate;

	s->period = 0;
	s->periods = 0;
/*
	DPRINTK("avail_max=%d, rate=%d, channels=%d, period_size=%d, periods=%d, buffer_size=%d, tick_time=%d, \
min_align=%d, byte_align=%d, frame_bits=%d, sleep_min=%d, xfer_align=%d, boundary=%d\n", 
		runtime->avail_max, runtime->rate, runtime->channels, runtime->period_size, runtime->periods, 
		runtime->buffer_size, runtime->tick_time, runtime->min_align, runtime->byte_align, runtime->frame_bits, 
		runtime->sleep_min, runtime->xfer_align, runtime->boundary);
*/

	return 0;
}

static snd_pcm_uframes_t snd_wmt_alsa_pointer(struct snd_pcm_substream *substream)
{
	struct snd_card_wmt_codec *chip = snd_pcm_substream_chip(substream);

	return ac97_audio_get_dma_pos(&chip->s[substream->pstr->stream]);
}


static void ac97_fifo_enable(void)
{
	/*
	 * AC'97 controller FIFO(s) presetting.
	 */
	ac97.regs->PTFC = PTFC_PTFT(8);
	ac97.regs->PRFC = PRFC_PRFT(8);
	ac97.regs->MIFC = MIFC_MFT(8);
	/*
	 * Enable PCM_TX, PCM_RX, MIC_IN FIFOs.  stereo , 16bit
	 */
	ac97.regs->PTFC |= PTFC_PTFEN | PTFC_PTFADE;
	ac97.regs->PRFC |= PRFC_PRFEN | PRFC_PRFADE;
	/*ac97.regs->MIFC |= MIFC_MFEN | MIFC_MFADE;*/

}


static void ac97_fifo_disable(void)
{
	/*
	 * Disable PCM_TX, PCM_RX, MIC_IN FIFOs.
	 */
	ac97.regs->PTFC &= ~(PTFC_PTFEN | PTFC_PTFADE);
	ac97.regs->PRFC &= ~(PRFC_PRFEN | PRFC_PRFADE);
	/*ac97.regs->MIFC &= ~(MIFC_MFEN | MIFC_MFADE);*/

}


static int wmt_audio_init_codec(void)
{
	unsigned short reg;

	/*	DPRINTK("Enter\n");*/
	wmt_ac97_enable();
	/*
	 * Enable/wakeup VT1613 CODEC.
	 */
	if ( alsa_codec_config->codec->ops->init() != 0 ) {
		return -ENODEV;
	}   

	/*
	 * Playback presettings.
	 *
	 * Enable audio DAC output and ADC input path.
	 */
	alsa_codec_config->codec->ops->read(VTAC_PDCS, &reg);
	reg &= ~(VTAC_PDCS_PR0 | VTAC_PDCS_PR1);
	alsa_codec_config->codec->ops->write(VTAC_PDCS, reg);

	/*
	 * Wait DAC and ADC sections are ready.
	 */
	do {
		alsa_codec_config->codec->ops->read(VTAC_PDCS, &reg);
	} while ((reg & (VTAC_PDCS_ADC | VTAC_PDCS_DAC)) != (VTAC_PDCS_ADC | VTAC_PDCS_DAC));

	/*
	 * Harry's test to lower volume by adding more attenuation.
	 */

	/*codec->ops->write(VTAC_MASV, (codec->volume | (codec->volume << 8)));*/
	/*alsa_codec_config->codec->ops->write(VTAC_MASV, 0x0000);*/
	alsa_codec_config->codec->ops->write(VTAC_MASV, 0);
	alsa_codec_config->codec->ops->write(VTAC_HPSV, 0);	/* ****/

	alsa_codec_config->codec->ops->write(VRAC_PCMV, 0x0808);
	/*codec->ops->write(VTAC_EINT, 0xFFFF);*/
	/*codec->ops->write(VTAC_PLL, 0x0000);*/

	/*
	 * Enable Variable Rate Audio (VRA) mode.
	 */
	alsa_codec_config->codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);

	/*
	 * Recording presettings.
	 *
	 */
	alsa_codec_config->codec->ops->write(VTAC_RECG, 0x0404);
#ifdef REC_DEBUG        
	alsa_codec_config->codec->ops->write(VTAC_RECS, 0x0404);                //select linein as record source
	alsa_codec_config->codec->ops->write(VTAC_MICV, 0x0808);                //unmute mic in
	alsa_codec_config->codec->ops->write(VTAC_LINV, 0x0808);                //unmute line in
#else   
	alsa_codec_config->codec->ops->write(VTAC_RECS, 0x0000);
	alsa_codec_config->codec->ops->write(VTAC_MICV, 0x8008);
	alsa_codec_config->codec->ops->write(VTAC_LINV, 0x8808);
#endif

	/*
	 * Extra presettings.
	 *
	 * Enable headphone driver and de-emphasis.
	 */
	alsa_codec_config->codec->ops->read(VTAC_FCS1, &reg);
	alsa_codec_config->codec->ops->write(VTAC_FCS1, (reg | 0x60));
	alsa_codec_config->codec->ops->write(VTAC_IODA, 0x0401);

	/*codec->ops->write(VTAC_EINT, 0xFFFF);*/
	/*codec->ops->write(VTAC_PLL, 0x0000);*/
	alsa_codec_config->codec->ops->write(VTAC_EINT, 0x0000);
	alsa_codec_config->codec->ops->write(VTAC_PLL, 0x0084);

	ac97_fifo_enable();

	return 0;
}


/*
 * Shutdown the audio driver.
 */
static void wmt_audio_shutdown(void)
{
	DPRINTK("Enter\n");
	ac97_fifo_disable();
	/*ac97.regs->ACGS |= ACGS_CPD;*/
	alsa_codec_config->codec->ops->exit();

#if 0
	/*
	 * Disable audio DAC output and ADC input path.
	 */
	codec->ops->read(VTAC_PDCS, &reg);
	reg |= (VTAC_PDCS_PR0 | VTAC_PDCS_PR1);
	codec->ops->write(VTAC_PDCS, reg);
#endif
}


static int audio_release(void)
{
	unsigned short reg = 0;

	alsa_codec_config->codec->ops->read(VTAC_MASV, &reg);/*Dean add 2008/07/22 for loopback disable*/
	reg |= 0x8000;
	alsa_codec_config->codec->ops->write(VTAC_MASV, reg);
	alsa_codec_config->codec->ops->read(VTAC_HPSV, &reg);
	reg |= 0x8000;
	alsa_codec_config->codec->ops->write(VTAC_HPSV, reg);

	wmt_audio_shutdown();		//zhf: It seems not necessary If add it, function close is the same with function free
	return 0;
}


static int snd_card_wmt_alsa_open(struct snd_pcm_substream * substream)
{
	struct snd_card_wmt_codec *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	struct audio_stream_a *s = &chip->s[stream_id];
	int err;

	DPRINTK("s[playback]: %d, s[capture]: %d\n", &chip->s[0], &chip->s[1]);
	DPRINTK("s pointer: %d, stream_id: %d, dmach: %d\n", s, stream_id, s->dmach);
	s->stream = substream;

	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = *(alsa_codec_config->snd_wmt_alsa_playback);
	}
	else {
		runtime->hw = *(alsa_codec_config->snd_wmt_alsa_capture);
	}
	ac97_audio_setup_dma(s, stream_id);
	
	if ((err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS)) < 0) 
		return err;
	
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		if ((err = snd_pcm_hw_constraint_list(runtime,
					0,
					SNDRV_PCM_HW_PARAM_RATE,
                                        alsa_codec_config->hw_playback_constraints_rates)) < 0)
                        return err;
        }
        else {
                if ((err = snd_pcm_hw_constraint_list(runtime,
                                        0,
                                        SNDRV_PCM_HW_PARAM_RATE,
                                        alsa_codec_config->hw_capture_constraints_rates)) < 0)
                        return err;
        }

	/*ac97_reg_dump();*/
	/*vt1613_reg_dump();*/

	return 0;
}

static int snd_card_wmt_alsa_close(struct snd_pcm_substream * substream)
{
	struct snd_card_wmt_codec *chip = snd_pcm_substream_chip(substream);

	DPRINTK("Enter\n");
	chip->s[substream->pstr->stream].stream = NULL;
	
	return 0;
}

/* HW params & free */
static int snd_wmt_alsa_hw_params(struct snd_pcm_substream * substream,
				    struct snd_pcm_hw_params * hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_wmt_alsa_hw_free(struct snd_pcm_substream * substream)
{
	return snd_pcm_lib_free_pages(substream);
}




/* pcm operations */
static struct snd_pcm_ops snd_card_wmt_alsa_playback_ops = {
	.open =		snd_card_wmt_alsa_open,
	.close =	snd_card_wmt_alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params =		snd_wmt_alsa_hw_params,
	.hw_free =	snd_wmt_alsa_hw_free,
	.prepare =	snd_wmt_alsa_prepare,
	.trigger =	snd_wmt_alsa_trigger,
	.pointer =	snd_wmt_alsa_pointer,
};

static struct snd_pcm_ops snd_card_wmt_alsa_capture_ops = {
	.open =	snd_card_wmt_alsa_open,
	.close =	snd_card_wmt_alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params =		snd_wmt_alsa_hw_params,
	.hw_free =	snd_wmt_alsa_hw_free,
	.prepare =	snd_wmt_alsa_prepare,
	.trigger =		snd_wmt_alsa_trigger,
	.pointer =	snd_wmt_alsa_pointer,
};

/*
 *  Alsa init and exit section
 *  
 *  Inits pcm alsa structures, allocate the alsa buffer, suspend, resume
 */
static int __init snd_card_wmt_alsa_pcm(struct snd_card_wmt_codec *wmt_alsa, 
					int device)
{
	struct snd_pcm *pcm;
	int err;
	
	DPRINTK("Enter\n");
	if ((err = snd_pcm_new(wmt_alsa->card, "WMT PCM", device, 1, 1, &pcm)) < 0)
		return err;

	/* sets up initial buffer with continuous allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL),
					      128 * 1024, 128 * 1024);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_card_wmt_alsa_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_card_wmt_alsa_capture_ops);
	pcm->private_data = wmt_alsa;
	pcm->info_flags = 0;
	strcpy(pcm->name, "wmt alsa pcm");

	wmt_alsa_audio_init(wmt_alsa);

	/* setup DMA controller */
	ac97_audio_dma_request(&wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK], ac97_audio_dma_callback);
	ac97_audio_dma_request(&wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE], ac97_audio_dma_callback);

	wmt_alsa->pcm = pcm;

	return 0;
}


#ifdef CONFIG_PM
/*
 * Driver suspend/resume - calls alsa functions. Some hints from aaci.c
 */
int snd_wmt_alsa_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card_wmt_codec *chip;
	struct snd_card *card = platform_get_drvdata(pdev);
	
	if (card->power_state != SNDRV_CTL_POWER_D3hot) {
		chip = card->private_data;
/*
DPRINTK("dmach: %d\n", chip->dmach);
if ( chip->dmach >= 0 )
	wmt_dump_dma_regs(chip->dmach);
*/
		if (chip->card->power_state != SNDRV_CTL_POWER_D3hot) {
			DPRINTK("Enter real suspend\n");
#ifdef DEBUG
ac97_reg_dump();
vt1613_reg_dump();
#endif 
			ac97_reg_backup();
			snd_wmt_suspend_mixer();	//call vt1613_reg_backup();
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D3hot);
			snd_pcm_suspend_all(chip->pcm);
			if ( chip->direction == SNDRV_PCM_STREAM_PLAYBACK ) {
				ac97.regs->PTFC &= ~ PTFC_PTFADE;
			}
			else{
				ac97.regs->PTFC &= ~ PTFC_PTFADE;
			}
			udelay(5);
			DPRINTK("alsa_codec->dmach: %d,  alsa_codec->active: %d\n",  chip->dmach, chip->active);
			if ( chip->dmach != NULL_DMA ) {
				ac97_audio_stop_dma(&chip->s[chip->direction]);
			}
			wmt_audio_shutdown();
		}
#ifdef DEBUG
if ( chip->dmach >= 0 )
	wmt_dump_dma_regs(chip->dmach);
#endif 
/*
	struct dma_mem_reg_group_s dma_mem_reg ;
	dma_mem_reg = wmt_get_dma_pos_info(chip->dmach);
	DPRINTK("IF0BAR: %x, IF0CPR: %x, IF0RBR:%x, IF0DAR: %x, IF1BAR: %x, IF1CPR: %x, IF1RBR: %x, IF1DAR: %x\n", 	
		dma_mem_reg.DMA_IF0BAR_CH, dma_mem_reg.DMA_IF0CPR_CH, 
		dma_mem_reg.DMA_IF0RBR_CH, dma_mem_reg.DMA_IF0DAR_CH, 
		dma_mem_reg.DMA_IF1BAR_CH, dma_mem_reg.DMA_IF1CPR_CH, 
		dma_mem_reg.DMA_IF1RBR_CH, dma_mem_reg.DMA_IF1DAR_CH);
*/
	}
	return 0;
}

int snd_wmt_alsa_resume(struct platform_device *pdev)
{
	struct snd_card_wmt_codec *chip;
	struct snd_card *card = platform_get_drvdata(pdev);

	if (card->power_state != SNDRV_CTL_POWER_D0) {				
		chip = card->private_data;
	DPRINTK("dmach: %d\n", chip->dmach);
		if (chip->card->power_state != SNDRV_CTL_POWER_D0) {
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D0);
			wmt_audio_init_codec();
			ac97_reg_restore();
			snd_wmt_resume_mixer();	//call vt1613_reg_restore();

			if ( chip->dmach != NULL_DMA && chip->dma_cfg != NULL ) {
				wmt_setup_dma(chip->dmach, *(chip->dma_cfg) ) ;
			}
		}
#ifdef DEBUG
if ( chip->dmach >= 0 )
	wmt_dump_dma_regs(chip->dmach);
#endif
	}
	return 0;
}
#else
#define snd_wmt_alsa_suspend	NULL
#define snd_wmt_alsa_resume	NULL
#endif	/* CONFIG_PM */

void snd_wmt_alsa_free(struct snd_card * card)
{
	struct snd_card_wmt_codec *chip = card->private_data;
	
	DPRINTK("Enter \n");
	/*
	 * Turn off codec after it is done.
	 * Can't do it immediately, since it may still have
	 * buffered data.
	 */
	schedule_timeout_interruptible(2);

	audio_release();

	ac97_audio_dma_free(&chip->s[SNDRV_PCM_STREAM_PLAYBACK]);
	ac97_audio_dma_free(&chip->s[SNDRV_PCM_STREAM_CAPTURE]);
}

/* module init & exit */

/* 
 * Inits alsa soudcard structure.
 * Called by the probe method in codec after function pointers has been set.
 */
int snd_wmt_alsa_post_probe(struct platform_device *pdev, struct wmt_alsa_codec_config *config)
{
	int err = 0;
	int def_rate;
	struct snd_card *card;

	wmt_ac97_enable();

	alsa_codec_config	= config;
/*	DPRINTK("codec: %d\n", alsa_codec_config->codec);*/
	alsa_codec_config->codec->ops->enable(); 

	/* register the soundcard */
	card = snd_card_new(-1, id, THIS_MODULE, sizeof(alsa_codec));
	if (card == NULL)
		goto nodev1;

	alsa_codec = kcalloc(1, sizeof(*alsa_codec), GFP_KERNEL);
	if (alsa_codec == NULL)
		goto nodev2;

	card->private_data = (void *)alsa_codec;
	card->private_free = snd_wmt_alsa_free;

	alsa_codec->card	= card;
	def_rate		= alsa_codec_config->get_default_samplerate(); 
	alsa_codec->samplerate	= def_rate;
	alsa_codec->dma_cfg = NULL;
	alsa_codec->dmach = NULL_DMA;
	alsa_codec->active = 0;

	spin_lock_init(&alsa_codec->s[0].dma_lock);
	spin_lock_init(&alsa_codec->s[1].dma_lock);

	/* mixer */
	if ((err = snd_wmt_mixer(alsa_codec, alsa_codec->card)) < 0)
		goto nodev3;

	/* PCM */
	if ((err = snd_card_wmt_alsa_pcm(alsa_codec, 0)) < 0)
		goto nodev3;

	strcpy(card->driver, "WMT PCM");
	strcpy(card->shortname, alsa_codec_config->name);
	sprintf(card->longname, alsa_codec_config->name);

        if ( wmt_audio_init_codec() == 0 ) {
                snd_wmt_init_mixer();
                snd_card_set_dev(card, &pdev->dev);
                if ((err = snd_card_register(card)) == 0) {
                        printk(KERN_INFO "audio support initialized\n");
                        platform_set_drvdata(pdev, card);
                        return 0;
                }
        }
        else {
                printk("audio initialization fails!\n");
        }


nodev3:
	kfree(alsa_codec);	
nodev2:	
	snd_card_free(card);
nodev1:

	return err;
}

int snd_wmt_alsa_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_card_wmt_codec *chip = card->private_data;

	audio_release();

	card->private_data = NULL;
	kfree(chip);
	snd_card_free(card);

	if (alsa_codec) {
		kfree(alsa_codec);
		alsa_codec = NULL;
	}
	
	platform_set_drvdata(pdev, NULL);
	
	return 0;
}


MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WMT AC97 driver for ALSA");
MODULE_ALIAS("wmt_alsa_vt1613.0");

