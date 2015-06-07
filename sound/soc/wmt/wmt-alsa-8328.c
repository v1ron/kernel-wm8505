/*

	Some descriptions of such software.
    
	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.
	
	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.
	
	Authors: V1ron (Roman I. Volkov) Russian software developer.
	Contacts:
	Web: http://v1ron.ru
	E-mail: v1ron@mail.ru, 1991v1ron@gmail.com

	Please refer to wmt-alsa.h file for more information.

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
#include <linux/interrupt.h>
#include <mach/gpio_if.h>
#include <linux/spinlock.h>

#include "wmt-alsa-8328.h"

#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

static void i2s_audio_dma_tx_callback(void *data);
static void i2s_audio_dma_rx_callback(void *data);

//static struct es8328_instance inst;
static struct snd_card *card;	
static struct snd_pcm *pcm;

#define I2S_BUFFER_SIZE MAX_DMA_SIZE * 4 // 4 periods 16Kb each
#define I2S_BLOCK_SIZE MAX_DMA_SIZE

/* hardware definition */
static struct snd_pcm_hardware snd_es8328_playback_hw = {
	// Data transmitted with interleaved channels, block of samples transfers,
	// DMA have access to physical memory and ALSA will handle this without copy & silence callbacks.
        .info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | 
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_JOINT_DUPLEX),
        .formats =          SNDRV_PCM_FMTBIT_S16_LE,
        .rates =            SNDRV_PCM_RATE_8000_96000 | SNDRV_PCM_RATE_CONTINUOUS,
        .rate_min =         8000,
        .rate_max =         96000,
        .channels_min =     2,
        .channels_max =     2,
        .buffer_bytes_max = I2S_BUFFER_SIZE,
        .period_bytes_min = I2S_BLOCK_SIZE,
        .period_bytes_max = I2S_BLOCK_SIZE,
        .periods_min =      I2S_BUFFER_SIZE / I2S_BLOCK_SIZE,
        .periods_max =      I2S_BUFFER_SIZE / I2S_BLOCK_SIZE,
	.fifo_size =        32,
};

/* hardware definition */
static struct snd_pcm_hardware snd_es8328_capture_hw = {
         .info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | 
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_JOINT_DUPLEX),
        .formats =          SNDRV_PCM_FMTBIT_S16_LE,
        .rates =            SNDRV_PCM_RATE_8000_96000,
        .rate_min =         8000,
        .rate_max =         96000,
        .channels_min =     2,
        .channels_max =     2,
        .buffer_bytes_max = I2S_BUFFER_SIZE,
        .period_bytes_min = I2S_BLOCK_SIZE,
        .period_bytes_max = I2S_BLOCK_SIZE,
        .periods_min =      I2S_BUFFER_SIZE / I2S_BLOCK_SIZE,
        .periods_max =      I2S_BUFFER_SIZE / I2S_BLOCK_SIZE,
	.fifo_size =        32,
};

/* playback open callback */
static int snd_es8328_playback_open(struct snd_pcm_substream *substream)
{
	int status;

	DPRINTK("ES8328: end_es8328_playback_open()\n");
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);
	substream->private_data = card->private_data;
	substream->runtime->hw = snd_es8328_playback_hw;

	pd->pss = substream;
	pd->playback_buf_dev_pos = 0;
        // Allocation of DMA channel for TX transfer
	status = wmt_request_dma(&pd->dmach_tx, "i2s_tx", I2S_TX_DMA_REQ, i2s_audio_dma_tx_callback, substream);
	if (status < 0)
	{
		printk("ALSA: wmt_request_dma() failed with error code %d\n", status);
	}
        return status;
}

/* playback close callback */
static int snd_es8328_playback_close(struct snd_pcm_substream *substream)
{
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);
	DPRINTK("ES8328: end_es8328_playback_close()\n");
	// free DMA channel for future use
	wmt_free_dma(pd->dmach_tx);
	pd->pss = NULL;
	substream->private_data = NULL;
        return 0;
}

/* capture open callback */
static int snd_es8328_capture_open(struct snd_pcm_substream *substream)
{
	int status;	
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);

	DPRINTK("ES8328: end_es8328_capture_open()\n");
	substream->private_data = card->private_data;
	substream->runtime->hw = snd_es8328_capture_hw;

	pd->css = substream;
	pd->capture_buf_dev_pos = 0;
        // Allocation of DMA channel for RX transfer
	status = wmt_request_dma(&pd->dmach_rx, "i2s_rx", I2S_RX_DMA_REQ, i2s_audio_dma_rx_callback, substream);
	if (status < 0)
	{
		printk("ALSA: wmt_request_dma() failed with error code %d\n", status);
	}
	//printk("capture is opened for rx, dma channel is %d\n",pd->dmach_rx);
        return status;
}

/* capture close callback */
static int snd_es8328_capture_close(struct snd_pcm_substream *substream)
{
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);

	DPRINTK("ES8328: end_es8328_capture_close()\n");
	// free DMA channel for future use
	wmt_free_dma(pd->dmach_rx);
	pd->css = NULL;
	substream->private_data = NULL;
	//printk("capture closed for rx\n");
        return 0;
}

/* hw_params callback */
static int snd_es8328_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params)
{
	DPRINTK("ES8328: end_es8328_pcm_hw_params()\n");
	// allocation of DMA buffer (if this buffer is not already allocated)
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

/* hw_free callback */
static int snd_es8328_pcm_hw_free(struct snd_pcm_substream *substream)
{
	DPRINTK("ES8328: end_es8328_pcm_hw_free()\n");
        return snd_pcm_lib_free_pages(substream);
}

static int wmt_i2s_set_clocks(int sysclk, int sysclkdiv, int mclkdiv, int bclkdiv, int lrclkdiv)
{
	DPRINTK("ES8328: wmt_i2s_set_clocks()\n");
	// setting clock rates for i2s bus
	// 16-bit modes!
	// SYSCLK: bit 20:18
	// 000 = 16,3835
	// 010 = 22,5789
	// 100 = 36,8653
	// 001 = 24,5755
	// 011 = 33,8684
	// bit 17:16 divider (1,2,4)
	// tx
	*((volatile unsigned long *)(PMC_ADDR+0x210)) = sysclk | sysclkdiv;
	*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 
		WMT_I2S_MCLK_DIV_MAKE(mclkdiv) | WMT_I2S_BCLK_DIV_MAKE(bclkdiv);
	*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = WMT_I2S_LRCLK_DIV_MAKE(lrclkdiv);
	// rx
	*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_RXCLKDIV)) = 
		WMT_I2S_MCLK_DIV_MAKE(mclkdiv) | WMT_I2S_BCLK_DIV_MAKE(bclkdiv);
	*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_RXFRAMECR)) = WMT_I2S_LRCLK_DIV_MAKE(lrclkdiv);
	//printk("I2S clock settings updated\n");
	return 0;
}

/* prepare callback */
static int snd_es8328_pcm_prepare(struct snd_pcm_substream *substream)
{
	int sysclk, sysclkdiv, mclkdiv, bclkdiv, lrclkdiv, status;

	DPRINTK("ES8328: end_es8328_pcm_prepare()\n");
	// setting of chip DAC rate and retrieving i2s clocks
	if (substream->pstr->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		status = snd_chip_set_dac_rate(substream->runtime->rate,
			&sysclk, &sysclkdiv, &mclkdiv, &bclkdiv, &lrclkdiv);
		//printk("i2s: rate is %d for playback\n",substream->runtime->rate);
	} else if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		status = snd_chip_set_adc_rate(substream->runtime->rate,
			&sysclk, &sysclkdiv, &mclkdiv, &bclkdiv, &lrclkdiv);
		//printk("i2s: rate is %d for capture\n",substream->runtime->rate);
	} else
	{
		status = -EINVAL;
	}
	if (status >= 0)
	{
		status = wmt_i2s_set_clocks(sysclk, sysclkdiv, mclkdiv, bclkdiv, lrclkdiv);
	}
	//printk("sysclk %d sysclkdiv %d mclkdiv %d bclkdiv %d, lrclkdiv %d\n",sysclk,sysclkdiv,mclkdiv,bclkdiv,lrclkdiv);
        return status;
}

/* trigger callback */
static int snd_es8328_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int status;
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);

	pd->playback_buf_dev_pos = 0;
	DPRINTK("ES8328: end_es8328_pcm_trigger()\n");
	if (substream->pstr->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		if (cmd == SNDRV_PCM_TRIGGER_START)
		{
			// Enable i2s transmission
			*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_TCR)) |= 1;
			// DMA
			pd->dma_cfg.DefaultCCR = I2S_TX_DMA_CFG;
			pd->dma_cfg.MIF0addr = (dma_addr_t)__virt_to_phys(substream->runtime->dma_area);
			pd->dma_cfg.MIF1addr = I2S_TX_FIFO;
			pd->dma_cfg.ChunkSize = SIZE_16KB;
			pd->dma_cfg.DeviceReqType = I2S_TX_DMA_REQ;
			
			status = wmt_setup_dma(pd->dmach_tx,pd->dma_cfg);
			if (status < 0) 
			{ 
				printk("ALSA: wmt_setup_dma() failed with error code %d\n",status);
			}
			//printk("ALSA: DMA area is %d\n",__virt_to_phys(substream->runtime->dma_area));
			status = wmt_start_dma(pd->dmach_tx, (dma_addr_t)__virt_to_phys(substream->runtime->dma_area), 
				I2S_TX_FIFO, I2S_BLOCK_SIZE);
			if (status < 0) 
			{
				printk("ALSA: wmt_start_dma() failed with error code %d\n",status); 
			}
			//printk("trigger_start for playback. DMA channel is %d\n",pd->dmach_tx);
		} else if (cmd == SNDRV_PCM_TRIGGER_STOP)
		{
			// Disable i2s transmission
			wmt_stop_dma(pd->dmach_tx);
			wmt_clear_dma(pd->dmach_tx);
			*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_TCR)) &= 0xfffffffe;
			//printk("trigger_stop for playback\n");
		} else
		{
			return -EINVAL;
		}
	} else if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		if (cmd == SNDRV_PCM_TRIGGER_START)
		{
               		 /* do something to start the PCM engine */
			// Enable i2s transmission
			*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_RCR)) |= 1;
			// DMA
			pd->dma_cfg.DefaultCCR = I2S_RX_DMA_CFG;
			pd->dma_cfg.MIF0addr = (dma_addr_t)__virt_to_phys(substream->runtime->dma_area);
			pd->dma_cfg.MIF1addr = I2S_RX_FIFO;
			pd->dma_cfg.ChunkSize = SIZE_16KB;
			pd->dma_cfg.DeviceReqType = I2S_RX_DMA_REQ;
			
			status = wmt_setup_dma(pd->dmach_rx,pd->dma_cfg);
			if (status < 0) 
			{ 
				printk("ALSA: wmt_setup_dma() failed with error code %d\n",status);
			}
			//printk("ALSA: DMA area is %d\n",__virt_to_phys(substream->runtime->dma_area));
			status = wmt_start_dma(pd->dmach_rx, (dma_addr_t)__virt_to_phys(substream->runtime->dma_area), 
				I2S_RX_FIFO, I2S_BLOCK_SIZE);
			if (status < 0) 
			{
				printk("ALSA: wmt_start_dma() failed with error code %d\n",status); 
			}
			//printk("trigger_start for record. DMA channel is %d\n",pd->dmach_rx);
		} else if (cmd == SNDRV_PCM_TRIGGER_STOP)
		{
			/* do something to stop the PCM engine */
			// Disable i2s transmission
			wmt_stop_dma(pd->dmach_rx);
			wmt_clear_dma(pd->dmach_rx);
			*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_RCR)) &= 0xfffffffe;
			//printk("trigger_stop for record\n");
		} else
		{
			return -EINVAL;
		}
	} else
	{
		return -EINVAL;
	}
	return 0;
}

/* pointer callback */
static snd_pcm_uframes_t snd_es8328_pcm_pointer(struct snd_pcm_substream *substream)
{
	DPRINTK("ES8328: end_es8328_pcm_pointer()\n");
        snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);

        /* get the current hardware pointer */
	if (substream->pstr->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		//printk("pointer of TX called\n");
        	return bytes_to_frames(substream->runtime,pd->playback_buf_dev_pos);
	} else if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		//printk("pointer of RX called\n");
		return bytes_to_frames(substream->runtime,pd->capture_buf_dev_pos);
	} else
	{
		return -EINVAL;
	}
}

/* operators */
static struct snd_pcm_ops snd_es8328_playback_ops = {
        .open =        snd_es8328_playback_open,
        .close =       snd_es8328_playback_close,
        .ioctl =       snd_pcm_lib_ioctl,
        .hw_params =   snd_es8328_pcm_hw_params,
        .hw_free =     snd_es8328_pcm_hw_free,
        .prepare =     snd_es8328_pcm_prepare,
        .trigger =     snd_es8328_pcm_trigger,
        .pointer =     snd_es8328_pcm_pointer,
	.copy    =     NULL,
        .silence =     NULL,
};

/* operators */
static struct snd_pcm_ops snd_es8328_capture_ops = {
	.open =        snd_es8328_capture_open,
	.close =       snd_es8328_capture_close,
	.ioctl =       snd_pcm_lib_ioctl,
	.hw_params =   snd_es8328_pcm_hw_params,
	.hw_free =     snd_es8328_pcm_hw_free,
	.prepare =     snd_es8328_pcm_prepare,
	.trigger =     snd_es8328_pcm_trigger,
	.pointer =     snd_es8328_pcm_pointer,
	.copy    =     NULL,
        .silence =     NULL,
};

/* DMA callback (called by WMT's DMA handlers) */
static void i2s_audio_dma_tx_callback(void *data)
{
	int status;
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);

	DPRINTK("ES8328: i2s_audio_dma_tx_callback()\n");
	// increase "hardware buffer" by count of bytes transmitted
	pd->playback_buf_dev_pos += I2S_BLOCK_SIZE;	
	if (pd->playback_buf_dev_pos >= substream->runtime->dma_bytes)
	{
		pd->playback_buf_dev_pos = 0;
	}

	// send new portion of audio data to DMA
	status = wmt_start_dma(pd->dmach_tx, 
		(dma_addr_t)__virt_to_phys(substream->runtime->dma_area) + pd->playback_buf_dev_pos, 
		I2S_TX_FIFO, I2S_BLOCK_SIZE);
	if (status < 0) {
		printk("ALSA: es8328 - fixme, wmt_start_dma() failed with error code %d\n",status);
	}
	// Notify ALSA that period elapsed
	snd_pcm_period_elapsed(pd->pss);
	//printk("ALSA: wmt_start_dma ret = %d, ptr %d\n",status,__virt_to_phys(pd->playback_buffer));
}

/* DMA callback (called by WMT's DMA handlers) */
static void i2s_audio_dma_rx_callback(void *data)
{
	int status;
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	snd_i2s_private_data *pd = snd_pcm_substream_chip(substream);

	DPRINTK("ES8328: i2s_audio_dma_rx_callback()\n");
	// increase "hardware buffer" by count of bytes transmitted
	pd->capture_buf_dev_pos += I2S_BLOCK_SIZE;	
	if (pd->capture_buf_dev_pos >= substream->runtime->dma_bytes)
	{
		pd->capture_buf_dev_pos = 0;
	}
	// send new portion of audio data to DMA
	status = wmt_start_dma(pd->dmach_rx, 
		(dma_addr_t)__virt_to_phys(substream->runtime->dma_area) + pd->capture_buf_dev_pos, 
		I2S_RX_FIFO, I2S_BLOCK_SIZE);
	if (status < 0) {
		printk("ALSA: es8328 - fixme, wmt_start_dma() failed with error code %d\n",status);
	}
	// Notify ALSA that period elapsed
	snd_pcm_period_elapsed(pd->css);
}

/* Driver initialization */
static int __init snd_es8328_new_pcm(void)
{
	int status;
	snd_i2s_private_data* pd;

	card = snd_card_new(-1,"WM8505 I2S", THIS_MODULE, sizeof(snd_i2s_private_data));
	// allocation of sound card, private data structure	
	if (card)
	{
		// probe & initialize the chip
		status = snd_chip_startup(card);
		//card->private_data = card + sizeof(struct snd_card);
		if (status >= 0) 
		{
			// Allocation of PCM instance
			status = snd_pcm_new(card, "I2S PCM", 0, 1, 1, &pcm);
			if (status >= 0) 
			{
				// pre-allocation of continuous physical memory to use as DMA buffer
				status = snd_pcm_lib_preallocate_pages_for_all(pcm,
					SNDRV_DMA_TYPE_CONTINUOUS,
					snd_dma_continuous_data(GFP_KERNEL),
					I2S_BUFFER_SIZE, I2S_BUFFER_SIZE);
				if (status >= 0) 
				{
					// Initialization of misc. fields
					strcpy(pcm->name, "I2S PCM");

					strcpy(card->driver, "WM8505 I2S");
					strcpy(card->shortname, "WM8505 I2S");
					strcpy(card->longname, "Sound chip, connected to WM8505 I2S BUS");
					strcpy(card->mixername, snd_chip_get_name());
					strcpy(card->components, snd_chip_get_name());
					pcm->private_free = NULL;
					pcm->private_data = card->private_data; // extra allocated space reserved for private data
		
					// initialization of private data to handle buffers
					pd = card->private_data;
					pd->playback_buf_dev_pos = 0; // position, where device will read data
					//spin_lock_init(&pd->lock);

					status = snd_card_register(card);
					if (status >= 0)
					{
						/* set operators */
						snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
							&snd_es8328_playback_ops);
						snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
							&snd_es8328_capture_ops);
						//snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
						//                &snd_mychip_capture_ops);
						/* We do not using any interrupts from I2S controller... 
						And our driver does not have any interrupt
						handlers, only DMA callback */
						/* ----Initialization of I2S... */
						/* Enable I2S controller interface. */
						// PMC_ADDR + 0x250 = Clock enables Lower register
						// bit 6: Set. I2C Slave Clock Enable
						//*((volatile unsigned long*)(PMC_ADDR+0x250)) |= 0x10000000; // enable i2s 6-pin mode
						*((volatile unsigned long*)(PMC_ADDR+0x250)) |= 0x00010040;
						// Offset 0x200 = Pin-Sharing Selection register
						// Bit 3: Set. Enable for CLKOUT Pin to Output 24 MHz Clock (1 (Enabled): CLKOUT behaves for CCIR function)
						*((volatile unsigned long*)(GPIO_ADDR+0x200)) |= 0x00000004;
						// Bit 1: Cleared. AC97/I2S Pin Sharing Select. 0 = I2S, 1 = AC97
						*((volatile unsigned long*)(GPIO_ADDR+0x200)) &= 0xFFFFFFFD;

						/* init i2s controller */
						/* Tx & RX FIFO disabling */
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_TCR)) &= 0xfffffffe ;  /* disable Tfifo, clean bit 0 */
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_RCR)) &= 0xfffffffe ;  /* disable Rfifo, clean bit 0 */
						// ??? Resetting bits
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) = 0;
						// Record Right Channel enable & Record Left channel enable (set to 1, bits 29,28)
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 0x30000000;
						// Playback Left & Right Enable (set to 1 13,12 bits)
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 0x3000;
						// Set Tx (playback) sample bit to 011 (16-bit)
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 0x300;
						// set Rx sample bit to 011 (16-bit)
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 0x3000000;
						// Set Tx audio format to I2S format (bit 3,2 to 0,0) ... is already clean
						//*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR) |= (I2S_I2S_MODE << I2S_RX_IF_MODE_SHIFT);
						// Set Rx audio format to I2S format (bit 19,18 to 0,0) ... is already clean
						//i2s.regs->AUDCTLCR |= (I2S_I2S_MODE << I2S_TX_IF_MODE_SHIFT);
						// Select Master Clock Polarity  for TXMCLK and RXMCLK (bit 16 set to 1, Rising edge)
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 0x10000;
						// Rx in slave mode
						//*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 20000;
						// Rx have same configuration as Tx
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_TXEQURXCR)) |= 0x1;
						// Bit 0 = 1. Enable audio transmission between external i2s device
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_AUDCTLCR)) |= 0x1;

						/* Tx FIFO presetting. */
						//*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_TCR)) &= (~1) ;  /* disable Tfifo, clean bit 0 */
						// FIFO interrupts enabling.
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_RCR)) = 0x00000A08;
						*((volatile unsigned long*)(I2S_BASE_ADDR + I2S_TCR)) = 0x00000A08;

						// ??? unknown register...
						//*(volatile unsigned char*)(SCC_ADDR+0x80) = 0x01;

						//i2s.regs->TSR  = (TCR_TFEIE + TCR_TFAIE + TCR_TFADE + TCR_TFUIE + TCR_TFOIE) ;
						//i2s.regs->RSR  = (RCR_RFFIE + RCR_RFAIE + RCR_RFADE + RCR_RFUIE + RCR_RFOIE) ;
						/* Enabling Tx FIFO */	
					}
				}	
			}
		}
	} else
	{
		status = -ENOMEM ;
	}

	if (status < 0)
	{
		printk("ES8328 driver for ALSA: Sound initialization failed with error code %d\n",status);
	} else
	{
		printk("ES8328 driver for ALSA: Sound initialization successful\n");
	}
	return status;
}

/*
CLOCK RATIOS:

Example of configuration:
44100 KHz, 16-bit, Stereo:
1) SYSCLK = 22,5789MHz / 2 = 11,28945MHz (CLKGEN settings)
2) MCLK = 11,28945MHz / 1 = 11,28945MHz (TXCLKDIV settings)
3) SCLK (bit clock) / 4 = 2,82236MHz (TXCLKDIV settings)
4) DLRCLK = 2,82236 / 64 = 0,0441MHz (TXFRAMECR divider = 64)
MCLK/DLRCLK (22,5789 / 0,044100) ratio is 255,9966 (256)
DLRCLK/SCLK (2,82236 / 0.044100) ratio is 63,99909 (64)
*/

static void __exit snd_es8328_delete_pcm(void)
{
	printk("ES8328 driver for ALSA: sound driver unload\n");
	snd_chip_shutdown();
	snd_card_free(card);
}

//---------------------------------------------------
module_init(snd_es8328_new_pcm);
module_exit(snd_es8328_delete_pcm);

MODULE_AUTHOR("V1ron (Roman. I. Volkov)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WMT I2S driver for ALSA");
