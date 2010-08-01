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

#include <mach/i2s_alsa.h>
#include "wmt-alsa-i2s.h"
#include "wmt-alsa-1602.h"
//#include "wmt_swmixer.h"

#undef DEBUG
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __func__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define AUDIO_VOLUME_DEFAULT            0x0D

static char *id	= NULL;	
static struct snd_card_wmt_codec 	*vt1602_alsa_codec		= NULL;
static struct wmt_alsa_codec_config	*vt1602_alsa_codec_config	= NULL;
extern struct i2s_s i2s;

static long pcm_fmt = AFMT_S16_LE;
#ifdef CONFIG_PM
static struct i2s_regs_s i2s_pm_regs;
#endif

/*
 * local CODEC pointer
 */
static struct i2s_codec_s *codec;

#define PMC_ADDR  (0xD8130000)
#define GPIO_ADDR (0xD8110000)
#define SCC_ADDR (0xD8120000)

static void wmt_i2s_enable(void)
{
	*((volatile unsigned long*)(PMC_ADDR+0x250)) |= 0x00000040;
	*((volatile unsigned long*)(GPIO_ADDR+0x200)) |= 0x00000004;
	*((volatile unsigned long*)(GPIO_ADDR+0x200)) &= 0xFFFFFFFD;
	udelay(2);	/* temp delay, should be removed */
}

void i2s_reg_dump(void)
{
	wmt_i2s_enable();

	printk("AUDCTLCR: 0x%.8x\n", i2s.regs->AUDCTLCR);
	printk("AUDDFCR: 0x%.8x\n", i2s.regs->AUDDFCR);
	printk("TXCLKDIV: 0x%.8x\n", i2s.regs->TXCLKDIV);
	printk("TXFRAMECR: 0x%.8x\n", i2s.regs->TXFRAMECR);
	printk("RXCLKDIV: 0x%.8x\n", i2s.regs->RXCLKDIV);
	printk("RXFRAMECR: 0x%.8x\n", i2s.regs->RXFRAMECR);
	printk("TXEQURXCR: 0x%.8x\n", i2s.regs->TXEQURXCR);
	printk("SPCTLCR: 0x%.8x\n", i2s.regs->SPCTLCR);
	printk("TCR: 0x%.8x\n", i2s.regs->TCR);
	printk("TSR: 0x%.8x\n", i2s.regs->TSR);
	printk("RCR: 0x%.8x\n", i2s.regs->RCR);
	printk("RSR: 0x%.8x\n", i2s.regs->RSR);
}


static struct i2s_regs_s pbackup_i2s_reg;
void i2s_reg_backup(void)
{
	wmt_i2s_enable();	

	pbackup_i2s_reg.AUDCTLCR = i2s.regs->AUDCTLCR;
	pbackup_i2s_reg.AUDDFCR= i2s.regs->AUDDFCR;
	pbackup_i2s_reg.TXCLKDIV= i2s.regs->TXCLKDIV;
	pbackup_i2s_reg.TXFRAMECR= i2s.regs->TXFRAMECR;
	pbackup_i2s_reg.RXCLKDIV= i2s.regs->RXCLKDIV;
	pbackup_i2s_reg.RXFRAMECR= i2s.regs->RXFRAMECR;
	pbackup_i2s_reg.TXEQURXCR= i2s.regs->TXEQURXCR;
	pbackup_i2s_reg.SPCTLCR= i2s.regs->SPCTLCR;
	pbackup_i2s_reg.TCR= i2s.regs->TCR;
	pbackup_i2s_reg.TSR= i2s.regs->TSR;
	pbackup_i2s_reg.RCR= i2s.regs->RCR;
	pbackup_i2s_reg.RSR= i2s.regs->RSR;

}

void i2s_reg_restore(void)
{
	wmt_i2s_enable();	
	i2s.regs->AUDCTLCR = pbackup_i2s_reg.AUDCTLCR; 
	i2s.regs->AUDDFCR = pbackup_i2s_reg.AUDDFCR;
	i2s.regs->TXCLKDIV = pbackup_i2s_reg.TXCLKDIV;
	i2s.regs->TXFRAMECR = pbackup_i2s_reg.TXFRAMECR;
	i2s.regs->RXCLKDIV = pbackup_i2s_reg.RXCLKDIV;
	i2s.regs->RXFRAMECR = pbackup_i2s_reg.RXFRAMECR;
	i2s.regs->TXEQURXCR = pbackup_i2s_reg.TXEQURXCR;
	i2s.regs->SPCTLCR = pbackup_i2s_reg.SPCTLCR;
	i2s.regs->TCR = pbackup_i2s_reg.TCR;
	i2s.regs->TSR = pbackup_i2s_reg.TSR;
	i2s.regs->RCR = pbackup_i2s_reg.RCR;
	i2s.regs->RSR = pbackup_i2s_reg.RSR;
}

static void wmt_alsa_audio_init(struct snd_card_wmt_codec *wmt_alsa)
{
	/* Setup DMA stuff */
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].id = "WMT PCM out";
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].stream_id = SNDRV_PCM_STREAM_PLAYBACK;
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dmach = I2S_TX_DMA_REQ ;
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dma_dev = I2S_TX_DMA_REQ ;
	wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK].dma_cfg = dma_device_cfg_table[I2S_TX_DMA_REQ ],

	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].id = "WMT PCM in";
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].stream_id = SNDRV_PCM_STREAM_CAPTURE;
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dmach = I2S_RX_DMA_REQ ;
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dma_cfg = dma_device_cfg_table[I2S_RX_DMA_REQ ],
	wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE].dma_dev = I2S_RX_DMA_REQ ;
//	DPRINTK("s[playback] pointer: %d, s[capture] pointer: %d\n", &wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK], &wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE]);
}

/* audio_setup_dma()
 *
 * Just a simple funcion to control DMA setting for AC'97 audio
 */
static void i2s_audio_setup_dma(struct audio_stream_a *s, int strream_id)
{
	unsigned int channel, byte;
	struct snd_pcm_runtime *runtime = s->stream->runtime;

	if (strream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		/* From memory to device */
		//byte = AUDIO_BYTE(ac97.regs->PTFC);
		//channel = AUDIO_CHANNEL(ac97.regs->PTFC);

		byte = (runtime->sample_bits)/8;
		channel = runtime->channels; // Vincent To be modified in future
//		printk("i2s_audio_setup_dma byte = %d, channels = %d\n", runtime->sample_bits,runtime->channels);
		switch (byte * channel) {
		case 1:
			s->dma_cfg.DefaultCCR = I2S_TX_DMA_8BITS_CFG;     /* setup 1 bytes*/
			break ;
		case 2:
			s->dma_cfg.DefaultCCR = I2S_TX_DMA_16BITS_CFG  ;    /* setup 2 bytes*/
			break ;
		case 4:
			s->dma_cfg.DefaultCCR = I2S_TX_DMA_32BITS_CFG  ;    /* setup 4 byte*/
			break ;
		}
	} else {
		/* From device to memory */
		//byte = AUDIO_BYTE(ac97.regs->PRFC);
		//channel = AUDIO_CHANNEL(ac97.regs->PRFC);

		byte = (runtime->sample_bits)/8;
		channel = runtime->channels; // Vincent To be modified in future
		switch (byte * channel) {
		case 1:
			s->dma_cfg.DefaultCCR = I2S_RX_DMA_8BITS_CFG   ;     /* setup 1 bytes*/
			break ;
		case 2:
			s->dma_cfg.DefaultCCR = I2S_RX_DMA_16BITS_CFG  ;    /* setup 2 bytes*/
			break ;
		case 4:
			s->dma_cfg.DefaultCCR = I2S_RX_DMA_32BITS_CFG  ;    /* setup 4 byte*/
			break ;
		}
	}
//	DPRINTK("s pointer: %d. audio dma %d cfg.DefaultCCR 0x%x \n", s, s->dmach, s->dma_cfg.DefaultCCR);
//	DPRINTK("cfg.ChunkSize 0x%x \n", s->dma_cfg.ChunkSize);
	wmt_setup_dma(s->dmach, s->dma_cfg) ; /* temp*/
}


/* 
 * DMA functions 
 * 
 */
static int i2s_audio_dma_request(struct audio_stream_a *s, void (*callback) (void *))
{
	int err;

//	DPRINTK("s pointer: %d, dmach: %d, id: %s, dma_dev: %d\n", s, s->dmach, s->id, s->dma_dev);
	err = wmt_request_dma(&s->dmach, s->id, s->dma_dev, callback, s);
	if (err < 0)
		printk(KERN_ERR "Unable to grab audio dma 0x%x\n", s->dmach);
	return err;
}

static int i2s_audio_dma_free(struct audio_stream_a *s)
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
//???? need to change fragment to period?
static u_int i2s_audio_get_dma_pos(struct audio_stream_a *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;
	dma_addr_t ptr;

	ptr = wmt_get_dma_pos(s->dmach);
//	DPRINTK("ptr=0x%x, dma_area_phy=0x%x\n", ptr, __virt_to_phys((dma_addr_t)runtime->dma_area));
	offset = bytes_to_frames(runtime, ptr-__virt_to_phys((dma_addr_t)runtime->dma_area));
//	DPRINTK("offset: %d\n", offset);

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;

}

/*
 * this stops the dma and clears the dma ptrs
 */
//???? need to clear dma after stopping it?
static void i2s_audio_stop_dma(struct audio_stream_a *s)
{
	unsigned long flags;

	DPRINTK("Enter\n");
	local_irq_save(flags);
	s->active = 0;
	s->period = 0;
	s->periods = 0;
	vt1602_alsa_codec->active = 0;
	wmt_stop_dma(s->dmach);
	wmt_clear_dma(s->dmach);	//zhf: Is it necessary to clean dma here? In our OSS driver, it will cause suspend/resume fail
	local_irq_restore(flags);

}


/*
 *  Main dma routine, requests dma according where you are in main alsa buffer
 */
static void i2s_audio_process_dma(struct audio_stream_a *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;
	dma_addr_t dma_base;
	int ret;

//	DPRINTK("Enter\n");
	DPRINTK("s: %d, dmach: %d. active: %d\n", s, s->dmach, s->active);
	if (s->active) {
		substream = s->stream;
		runtime = substream->runtime;
		dma_size = frames_to_bytes(runtime, runtime->period_size);
//		DPRINTK("frame_bits=%d, period_size=%d, dma_size 1=%d\n", runtime->frame_bits, runtime->period_size, dma_size);
		if (dma_size > MAX_DMA_SIZE)		//zhf-????
			dma_size = CUT_DMA_SIZE;
		offset = dma_size * s->period;
//		DPRINTK("offset: 0x%x, ->dma_area: 0x%x, ->dma_addr: 0x%x, final addr: 0x%x\n", offset, runtime->dma_area, runtime->dma_addr, runtime->dma_addr+offset);
		dma_base = __virt_to_phys((dma_addr_t)runtime->dma_area);
//		DPRINTK("dma address: 0x%x\n", dma_base+offset );
//		DPRINTK("hw_ptr_interrupt: 0x%x, state: %d, hwptr: %u, applptr: %u, avail_min: %u\n", 
//			runtime->hw_ptr_interrupt, runtime->status->state, runtime->status->hw_ptr,
//			runtime->control->appl_ptr, runtime->control->avail_min);
//		DPRINTK("dmach: %u, dma_addr: %x, dma_size: %u\n", s->dmach, dma_base+offset, dma_size);
		ret = wmt_start_dma(s->dmach, dma_base+offset, 0, dma_size);
//wmt_dump_dma_regs(s->dmach);
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
static void i2s_audio_dma_callback(void *data)
{
	struct audio_stream_a *s = data;

//	DPRINTK("Enter\n");
	/* 
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	spin_lock(&s->dma_lock);
	if (s->periods > 0) 
		s->periods--;
	
	i2s_audio_process_dma(s);
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
		vt1602_alsa_codec->active = 1;
		vt1602_alsa_codec->direction = stream_id;
		vt1602_alsa_codec->dmach = s->dmach;
		vt1602_alsa_codec->dma_cfg = &(s->dma_cfg);
//	wmt_dump_dma_regs(s->dmach);
		i2s_audio_process_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* requested stream shutdown */
		i2s_audio_stop_dma(s);
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
	
	unsigned int channel, byte;
	byte = (runtime->sample_bits)/8;
	channel = runtime->channels; // Vincent To be modified in future

	//printk("snd_wmt_alsa_prepare byte = %d, channels = %d\n", runtime->sample_bits,runtime->channels);
	/* set requested samplerate */
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) 
		vt1602_alsa_codec_config->codec_set_dac_samplerate(runtime->rate);
	else
		vt1602_alsa_codec_config->codec_set_adc_samplerate(runtime->rate);
	chip->samplerate = runtime->rate;

	s->period = 0;
	s->periods = 0;

#if 1
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		if(byte == 1)
			i2s_set_tx_resolution(8);
		else
			i2s_set_tx_resolution(16);

		if(channel == 1)
			i2s_set_tx_channels(0);
		else
			i2s_set_tx_channels(1);

		s->dma_cfg.DefaultCCR = I2S_TX_DMA_32BITS_CFG  ;    /* setup 4 byte*/
	} else {
		if(byte == 1)
			i2s_set_rx_resolution(8);
		else
			i2s_set_rx_resolution(16);

		if(channel == 1)
			i2s_set_rx_channels(0);
		else
			i2s_set_rx_channels(1);

		
		s->dma_cfg.DefaultCCR = I2S_RX_DMA_32BITS_CFG  ;    /* setup 4 byte*/
	}
//	DPRINTK("s pointer: %d. audio dma %d cfg.DefaultCCR 0x%x \n", s, s->dmach, s->dma_cfg.DefaultCCR);
//	DPRINTK("cfg.ChunkSize 0x%x \n", s->dma_cfg.ChunkSize);
	wmt_setup_dma(s->dmach, s->dma_cfg) ; /* temp*/
#endif
/*
	printk("avail_max=%d, rate=%d, channels=%d, period_size=%d, periods=%d, buffer_size=%d, tick_time=%d, \
min_align=%d, byte_align=%d, frame_bits=%d, sample_bits=%d, sleep_min=%d, xfer_align=%d, boundary=%d\n", 
		runtime->avail_max, runtime->rate, runtime->channels, runtime->period_size, runtime->periods, 
		runtime->buffer_size, runtime->tick_time, runtime->min_align, runtime->byte_align, runtime->frame_bits, runtime->sample_bits,
		runtime->sleep_min, runtime->xfer_align, runtime->boundary);
*/

	return 0;
}

static snd_pcm_uframes_t snd_wmt_alsa_pointer(struct snd_pcm_substream *substream)
{
	struct snd_card_wmt_codec *chip = snd_pcm_substream_chip(substream);

	return i2s_audio_get_dma_pos(&chip->s[substream->pstr->stream]);
}


static void i2s_fifo_enable(void)
{
	/*
	 Enable TX, RX
	*/
	i2s.regs->TCR |= TCR_TFEN | TCR_TFADE | TCR_TFT(8);
	i2s.regs->RCR |= RCR_RFEN | RCR_RFADE | RCR_RFT(8);

}


static void i2s_fifo_disable(void)
{
	i2s.regs->TCR &= (~TCR_TFEN | TCR_TFADE) ;  /* disable Tfifo*/
	i2s.regs->RCR &= (~RCR_RFEN | RCR_RFADE) ;  /* disable Rfifo*/

}

static void wmt_audio_init_codec(void)
{

	DPRINTK("Enter\n");
	wmt_i2s_enable();
	i2s.init();
	/*
	 * Enable/wakeup VT1602 CODEC.
	 */
	vt1602_alsa_codec_config->codec->ops->init();


	return;
}


/*
 * Shutdown the audio driver.
 */
static void wmt_audio_shutdown(void)
{
	i2s_fifo_disable();
	vt1602_alsa_codec_config->codec->ops->exit();

	/* Disable CODEC Digital Interface Activation*/
	vt1602_alsa_codec_config->codec->ops->write(DIGITAL_ACTIVATION_ADDR, 0);

	/* Disable CODEC Power*/
	vt1602_alsa_codec_config->codec->ops->write(POWERDOWN_CONTROL_ADDR, 0xFF);
}


static int audio_release(void)
{

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

	DPRINTK("Enter\n");
//	DPRINTK("s[playback]: %d, s[capture]: %d\n", &chip->s[0], &chip->s[1]);
//	DPRINTK("s pointer: %d, stream_id: %d, dmach: %d\n", s, stream_id, s->dmach);
	s->stream = substream;
//111	vt1602_alsa_codec_config->codec->ops->enable();
//	wmt_audio_init_codec();

	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = *(vt1602_alsa_codec_config->snd_wmt_alsa_playback);
	}
	else {
		runtime->hw = *(vt1602_alsa_codec_config->snd_wmt_alsa_capture);
	}
//	i2s_audio_setup_dma(s, stream_id);
	
	if ((err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS)) < 0) 
		return err;
	
	if ((err = snd_pcm_hw_constraint_list(runtime,
					0,
					SNDRV_PCM_HW_PARAM_RATE,
					vt1602_alsa_codec_config->hw_constraints_rates)) < 0) 
		return err;


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
static struct snd_pcm_ops  snd_card_wmt_alsa_playback_ops = {
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
	i2s_audio_dma_request(&wmt_alsa->s[SNDRV_PCM_STREAM_PLAYBACK], i2s_audio_dma_callback);
	i2s_audio_dma_request(&wmt_alsa->s[SNDRV_PCM_STREAM_CAPTURE], i2s_audio_dma_callback);

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
	
//	DPRINTK("Enter\n");
	if (card->power_state != SNDRV_CTL_POWER_D3hot) {
		chip = card->private_data;
DPRINTK("dmach: %d\n", chip->dmach);
//if ( chip->dmach >= 0 )
//	wmt_dump_dma_regs(chip->dmach);
		if (chip->card->power_state != SNDRV_CTL_POWER_D3hot) {
			DPRINTK("Enter real suspend\n");
			i2s_reg_backup();
			snd_wmt_suspend_mixer();	//call vt1613_reg_backup();
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D3hot);
			snd_pcm_suspend_all(chip->pcm);
			DPRINTK("Enter vt1602_alsa_codec\n");
			if ( chip->direction == SNDRV_PCM_STREAM_PLAYBACK ) {
				i2s.regs->TCR &= ~ TCR_TFADE;
			}
			else{
				i2s.regs->RCR &= ~ RCR_RFADE;
			}
			udelay(5);
			DPRINTK("vt1602_alsa_codec->dmach: %d,  vt1602_alsa_codec->active: %d\n",  chip->dmach, chip->active);
			if ( chip->dmach != NULL_DMA ) {
				i2s_audio_stop_dma(&chip->s[chip->direction]);
			}
//			if ( chip->active )
				wmt_audio_shutdown();
		}
//if ( chip->dmach >= 0 )
//	wmt_dump_dma_regs(chip->dmach);
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

//	DPRINTK("Enter\n");
	if (card->power_state != SNDRV_CTL_POWER_D0) {				
		chip = card->private_data;
DPRINTK("dmach: %d\n", chip->dmach);
		if (chip->card->power_state != SNDRV_CTL_POWER_D0) {
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D0);
//			if ( vt1602_alsa_codec->active )
				wmt_audio_init_codec();
			i2s_reg_restore();
			snd_wmt_resume_mixer();	//call vt1613_reg_restore();

			if ( chip->dmach != NULL_DMA && chip->dma_cfg != NULL ) {
				wmt_setup_dma(chip->dmach, *(chip->dma_cfg) ) ;
//zhf-t				wmt_resume_dma(chip->dmach) ;
			}
		}
//if ( chip->dmach >= 0 )
//	wmt_dump_dma_regs(chip->dmach);
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

	i2s_audio_dma_free(&chip->s[SNDRV_PCM_STREAM_PLAYBACK]);
	i2s_audio_dma_free(&chip->s[SNDRV_PCM_STREAM_CAPTURE]);
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

//	DPRINTK("Enter\n");
	wmt_i2s_enable();

	vt1602_alsa_codec_config	= config;
//	DPRINTK("codec: %d\n", vt1602_alsa_codec_config->codec);
//	vt1602_alsa_codec_config->codec->ops->init(); //Vincent 2009.0715

	/* register the soundcard */
	card = snd_card_new(-1, id, THIS_MODULE, sizeof(vt1602_alsa_codec));
	if (card == NULL)
		goto nodev1;

	vt1602_alsa_codec = kcalloc(1, sizeof(*vt1602_alsa_codec), GFP_KERNEL);
	if (vt1602_alsa_codec == NULL)
		goto nodev2;

	card->private_data = (void *)vt1602_alsa_codec;
	card->private_free = snd_wmt_alsa_free;

	vt1602_alsa_codec->card	= card;
	def_rate		= vt1602_alsa_codec_config->get_default_samplerate(); 
	vt1602_alsa_codec->samplerate	= def_rate;
	vt1602_alsa_codec->dma_cfg = NULL;
	vt1602_alsa_codec->dmach = NULL_DMA;
	vt1602_alsa_codec->active = 0;

	spin_lock_init(&vt1602_alsa_codec->s[0].dma_lock);
	spin_lock_init(&vt1602_alsa_codec->s[1].dma_lock);

#if 1
	/* mixer */
	if ((err = snd_wmt_mixer(vt1602_alsa_codec, vt1602_alsa_codec->card)) < 0)
		goto nodev3;
#endif

	/* PCM */
	if ((err = snd_card_wmt_alsa_pcm(vt1602_alsa_codec, 0)) < 0)
		goto nodev3;

	strcpy(card->driver, "WMT PCM");
	strcpy(card->shortname, vt1602_alsa_codec_config->name);
	sprintf(card->longname, vt1602_alsa_codec_config->name);

	wmt_audio_init_codec();
	snd_wmt_init_mixer();
	snd_card_set_dev(card, &pdev->dev);
	
	if ((err = snd_card_register(card)) == 0) {
		printk(KERN_INFO "audio support initialized\n");
		platform_set_drvdata(pdev, card);
		return 0;
	}


nodev3:
	kfree(vt1602_alsa_codec);	
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

	if (vt1602_alsa_codec) {
		kfree(vt1602_alsa_codec);
		vt1602_alsa_codec = NULL;
	}
	
	platform_set_drvdata(pdev, NULL);
	
	return 0;
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


MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WMT I2S driver for ALSA");
MODULE_ALIAS("wmt_alsa_vt1602.0");
















