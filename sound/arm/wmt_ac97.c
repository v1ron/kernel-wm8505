/*++
	sound/arm/wmt_ac97.c

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


#include <mach/ac97.h>
#include <mach/vt1613.h>

#include "wmt_ac97.h"

//#define DEBUG
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#ifndef memzero
#define memzero(s, n)     memset ((s), 0, (n))
#endif

#define AUDIO_NAME                      "ac97"
#define AUDIO_NBFRAGS_DEFAULT           16 //4
#define AUDIO_FRAGSIZE_DEFAULT          SZ_4K //SZ_16K
#define AUDIO_FMT_MASK                  (AFMT_S16_LE | AFMT_U8)
#define AUDIO_FMT_DEFAULT               (AFMT_S16_LE | AFMT_U8)
#define AUDIO_CHANNELS_DEFAULT          2
#define AUDIO_VOLUME_DEFAULT            0x1F
#define AUDIO_ACTIVE(state)             ((state)->rd_ref || (state)->wr_ref)
#define AUDIO_MODE_MONO                 0
#define AUDIO_MODE_STEREO               1
#define AUDIO_RATE_DEFAULT              48000
#define FLUSH_BASE_PHYS                 0xe0000000
#define SPIN_ADDR                       (dma_addr_t)FLUSH_BASE_PHYS
#define SPIN_SIZE                       2048

//--> pacthed by howayhuo for android 1.6
// whether set the AC97 register at the first time
static int firstset = 1;
// whether the audio is initialized
//static int audio_init_flag = 0;  // 1: initialized.  0: don't initialized.
//<-- end
/*
 * For AC'97 FIFO control registers
 */
#define AUDIO_BYTE(r)           ((((r)>>12)&0x1)?(1):(2))       /* 1= 1 byte, 0 = 2 bytes */
#define AUDIO_CHANNEL(r)        ((((r)>>13)&0x1)?(1):(2))       /* 1= 1 channel, 0 = 2 channels */

#define REC_MASK        (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK        (REC_MASK | SOUND_MASK_VOLUME | SOUND_MASK_BASS | SOUND_MASK_TREBLE | SOUND_MASK_PCM)
//#define DEV_MASK        (REC_MASK | SOUND_MASK_VOLUME | SOUND_MASK_BASS | SOUND_MASK_TREBLE)

extern ac97_t ac97;

extern int register_sound_mixer(const struct file_operations *fops, int dev);
extern int register_sound_dsp(const struct file_operations *fops, int dev);
/*
 * local CODEC pointer
 */
static struct codec_s *codec;

static unsigned int max_volume = 0x1E;

extern int wmt_getsyspara_cache(char *varname, unsigned char *varval, int varlen);
extern int wmt_setsyspara_cache(char *varname, char *varval);

static unsigned int disspdif;

//--> howayhuo add to control GPIO to mute the AMP on 20100412
void audio_amp_powerup(int power_up)
{
    int retval;
    char buf[200]={0};

    if(power_up)
    {
        retval = wmt_getsyspara_cache("amp_powerup", buf, 200);
        if(!retval)
	{
	    printk("amp power up:%s\n", buf);
            excute_gpio_op(buf);
	}
        else
        {
            REG32_VAL(0xD8110064) |= BIT3;  //GPIO3 signal GPIO Enable
            REG32_VAL(0xD811008C) |= BIT3;  //GPIO3 Output Enable
            REG32_VAL(0xD81100B4) &= ~BIT3;  //GPIO3 Output Low
        }
    }
    else
    {
        retval = wmt_getsyspara_cache("amp_powerdown", buf, 200);
	if(!retval)
	{
	    printk("amp power down:%s\n", buf);
            excute_gpio_op(buf);
	}
	else
	{
            REG32_VAL(0xD8110064) |= BIT3;  //GPIO3 signal GPIO Enable
            REG32_VAL(0xD811008C) |= BIT3;  //GPIO3 Output Enable
            REG32_VAL(0xD81100B4) |= BIT3;  //GPIO3 Output High
	}
    }
}
//<-- end add

/*
	kevin add 2010-03-30 for mute mic&line
	call audio_mic_mute(flase) when start dma
	call audio_mic_mute(true)when dam transfer is done
*/
void audio_mic_mute(int enable)
{
unsigned short reg,vid1,vid2;

#define VIA_ID1	0x5649	//VI
#define VIA_ID2	0x4123	//A#
#define WL_ID1	0x574d	//WM
#define WL_ID2	0x4c12	//L12

	printk("audio_mic_mute %s\n",enable?"enable":"disable");

	codec->ops->read(VTAC_VID1, &vid1);
	codec->ops->read(VTAC_VID2, &vid2);
	if(vid1==VIA_ID1&&vid2==VIA_ID2){
		//1613
		//printk("codec :1613\n");
		if(enable){
			codec->ops->read(VTAC_LINV, &reg);
			reg |= (BIT15);
			codec->ops->write(VTAC_LINV, reg);

			codec->ops->read(VTAC_MICV, &reg);
			reg |= (BIT15|BIT7);
			codec->ops->write(VTAC_MICV, reg);

			codec->ops->read(VTAC_RECG, &reg);
			reg |= (BIT15);
			codec->ops->write(VTAC_RECG, reg);

			codec->ops->read(0x0c, &reg);
			reg |= (BIT15);
			codec->ops->write(0x0c, reg);

			codec->ops->read(0x0a, &reg);
			reg |= (BIT15);
			codec->ops->write(0x0a, reg);

		}else{
			codec->ops->read(VTAC_LINV, &reg);
			reg &= (~BIT15);
			codec->ops->write(VTAC_LINV, reg);

			codec->ops->read(VTAC_MICV, &reg);
			reg &= (~BIT15);
			reg &= (~BIT7);
			codec->ops->write(VTAC_MICV, reg);

			codec->ops->read(VTAC_RECG, &reg);
			reg &= (~BIT15);
			codec->ops->write(VTAC_RECG, reg);


			codec->ops->read(0x0c, &reg);
			reg &= (~BIT15);
			codec->ops->write(0x0c, reg);

			codec->ops->read(0x0a, &reg);
			reg &= (~BIT15);
			codec->ops->write(0x0a, reg);
		}
	}else if(vid1==WL_ID1&&vid2==WL_ID2){
		//9715
		//printk("codec :9715\n");
		if(enable){

			codec->ops->read(VTAC_LINV, &reg);
			reg |= (BIT15|BIT14|BIT13);
			codec->ops->write(VTAC_LINV, reg);

			codec->ops->read(VTAC_MICV, &reg);
			reg |= (BIT14|BIT13);
			codec->ops->write(VTAC_MICV, reg);

			codec->ops->read(VTAC_RECG, &reg);
			reg |= (BIT15);
			codec->ops->write(VTAC_RECG, reg);


			codec->ops->read(0x0c, &reg);
			reg |= (BIT15);
			codec->ops->write(0x0c, reg);

			codec->ops->read(0x0a, &reg);
			reg |= (BIT15);
			codec->ops->write(0x0a, reg);


		}else{
			codec->ops->read(VTAC_LINV, &reg);
			reg &= (~BIT15);
			reg &= (~BIT14);
			reg &= (~BIT13);

			codec->ops->write(VTAC_LINV, reg);

			codec->ops->read(VTAC_MICV, &reg);
			reg &= (~BIT14);
			reg &= (~BIT13);
			codec->ops->write(VTAC_MICV, reg);

			codec->ops->read(VTAC_RECG, &reg);
			reg &= (~BIT15);
			codec->ops->write(VTAC_RECG, reg);

			codec->ops->read(0x0c, &reg);
			reg &= (~BIT15);
			codec->ops->write(0x0c, reg);

			codec->ops->read(0x0a, &reg);
			reg &= (~BIT15);
			codec->ops->write(0x0a, reg);
		}
	}else{
		printk("codec id:0x%x:0x%x error\n",vid1,vid2);
	}


}

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
#define DMA_SETUPT(s)           wmt_setup_dma(s->dmach, s->dma_cfg) ; /* temp*/

#define IS_NULLDMA(ch)          ((ch) == NULL_DMA)
#define CHK_DMACH(ch)           ((ch) != NULL_DMA)


static void wm9715_gpioirq_comp2(void)
{
	printk("wm9715_gpioirq_comp2\n");
	codec->ops->write(0x4C, 0xF83A );//pin config
	codec->ops->write(0x4E, 0xBFFF );//int polarity

	codec->ops->write(0x50, 0x4000 );//int sticky
	codec->ops->write(0x52, 0x4000 );//int wakeup

	codec->ops->write(0x5C, 0x0400 );//add fun2

	codec->ops->write(0x54, 0x0820 );//int flags
	codec->ops->write(0x58, 0x000B );//add fun1

	codec->ops->write(0x56, 0xF83A );//pin config 2
	#if 0
	u16 val;
	//mutex_lock(&l_wm9715context->api_mutex);

	codec->ops->read(AC97_ADF2_CTL, &val);
	val &= ~BIT2;
	codec->ops->write(AC97_ADF2_CTL, val );

	codec->ops->read(AC97_GPIO_POLARITY, &val);
	val &= ~BIT14;
	val &= ~BIT15;
	codec->ops->write(AC97_GPIO_POLARITY, val );

	codec->ops->read(AC97_ADF2_CTL, &val);
	//comp2 reference vol and signal source
	val &= ~BIT9;
	val |= BIT10|BIT12;
	val &= ~BIT11;
	codec->ops->write(AC97_ADF2_CTL, val );

	codec->ops->read(AC97_GPIO_CFG, &val);
	val &= ~BIT2;
	codec->ops->write(AC97_GPIO_CFG, val );

	codec->ops->read(AC97_INT_STICKY, &val);
	val |= BIT15|BIT14;
	val &= ~BIT11;
	codec->ops->write(AC97_INT_STICKY, val );

	codec->ops->read(AC97_INT_WAKEUP, &val);
	val |= BIT15|BIT14;
	codec->ops->write(AC97_INT_WAKEUP, val );

	codec->ops->read(AC97_MISC_AFE, &val);
	val &= ~BIT2;
	codec->ops->write(AC97_MISC_AFE, val );

	codec->ops->read(AC97_ADF1_CTL, &val);
	//comp2del and wakeup enable and irqinv
	//val |= BIT14|BIT15;
	val |= BIT1;
	//val |= BIT0;
	codec->ops->write(AC97_ADF1_CTL, val );
	//mutex_unlock(&l_wm9715context->api_mutex);
	#endif
}


static void wm9715_gpioirq_clear()
{
	printk("wm9715_gpioirq_clear\n");
	codec->ops->write(0x4C, 0xF83E );
	codec->ops->write(0x4E, 0xFFFF );
	codec->ops->write(0x5C, 0x0000 );
	codec->ops->write(0x56, 0xF83E );
	codec->ops->write(0x50, 0x0000 );
	codec->ops->write(0x52, 0x0000 );
	codec->ops->write(0x54, 0x0820 );
}

static void wm9715_gpioirq_adcandpen()
{
	u16 val;
	printk("wm9715_gpioirq_adcandpen\n");
	codec->ops->read(0x4C, &val);
	val &= ~BIT3;
	codec->ops->write(0x4C, val );

	codec->ops->read(0x56, &val);
	val &= ~BIT3;
	codec->ops->write(0x56, val );

	codec->ops->read(0x78, &val);
	val |= (BIT15 | BIT14 );
	val &= (~BIT12);
	val |= 0x10;
	codec->ops->write(0x78, val  );
}


static u_int audio_get_dma_pos(struct audio_stream_s *s)
{
	struct audio_buf_s *b = &s->buffers[s->dma_tail];
	u_int offset;

	if (b->dma_ref) {
		offset = DMA_POS(s) - b->dma_addr;
		if (offset >= s->fragsize)
			offset = s->fragsize - 4;
	} else if (s->pending_frags) {
		offset = b->offset;
	} else {
		offset = 0;
	}
	return offset;
}

static void audio_stop_dma(struct audio_stream_s *s)
{
	u_int pos;
	unsigned long flags;

	if (s->dma_spinref > 0 || !s->buffers)
		return;

	DPRINTK("audio_stop_dma\n");

	local_irq_save(flags);
	s->stopped = 1;
	DMA_STOP(s);
	pos = audio_get_dma_pos(s);
/*	DMA_CLEAR(s);	//marked for suspend/resume by zhf */
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
	/*
	 * Back up pointers to be ready to restart from the same spot.
	 */
 	struct audio_buf_s *b;
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

static void audio_reset(struct audio_stream_s *s)
{

	DPRINTK("audio_reset\n");
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
 * Just a simple funcion to control DMA setting for AC'97 audio
 */
static void audio_setup_dma(struct audio_stream_s *s)
{
	unsigned int channel, byte;

	if (s->direction) {
		/* From memory to device */
		byte = AUDIO_BYTE(ac97.regs->PTFC);
		channel = AUDIO_CHANNEL(ac97.regs->PTFC);

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
	printk("audio_setup_dma s->direction = %d, byte =%u, channel = %u\n", s->direction, byte, channel);
	s->dma_cfg.ChunkSize = SZ_4K;
	DPRINTK("audio dma %d cfg 0x%lx \n", s->dmach, s->dma_cfg.DefaultCCR);
	wmt_setup_dma(s->dmach, s->dma_cfg) ; /* temp*/
}

static void audio_process_dma(struct audio_stream_s *s)
{
	int ret;

	if (s->stopped)
		goto spin;

	if (s->dma_spinref > 0 && s->pending_frags) {
		s->dma_spinref = 0;
		DMA_CLEAR(s);
	}

	while (s->pending_frags) {
		struct audio_buf_s *b = &s->buffers[s->dma_head];
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
		/*
		 * Note: if there is still a data buffer being
		 * processed then the ref count is negative.  This
		 * allows for the DMA termination to be accounted in
		 * the proper order.  Of course dma_spinref can't be
		 * greater than 0 if dma_ref is not 0 since we kill
		 * the spinning above as soon as there is real data
		 * to process.
		 */
		if (s->buffers && s->buffers[s->dma_tail].dma_ref)
			spincnt = -spincnt;
		s->dma_spinref += spincnt;
	}
}

static void audio_dma_callback(void *data)
{
	struct  audio_stream_s *s = data;
	struct audio_buf_s *b = &s->buffers[s->dma_tail];

	if (s->dma_spinref > 0) {
		s->dma_spinref--;
	} else if (!s->buffers) {
		printk(KERN_CRIT "wmt_audio: received DMA IRQ for non existent buffers!\n");
		return;
	} else if (b->dma_ref && --b->dma_ref == 0 && b->offset >= s->fragsize) {
		/* This fragment is done */
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
static void audio_discard_buf(struct audio_stream_s *s)
{

	printk("audio_discard_buf\n");
	/*
	 * Ensure DMA isn't using those buffers.
	 */
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

static int audio_setup_buf(struct audio_stream_s *s)
{
	int frag;
	int dmasize = 0;
	char *dmabuf = NULL;
	dma_addr_t dmaphys = 0;

	if (s->buffers)
		return -EBUSY;

	DPRINTK("audio_setup_buf\n");
	/*
	 * Configure DMA setting.
	 */
	audio_setup_dma(s);

	s->buffers = kmalloc(sizeof(struct audio_buf_s) * s->nbfrags, GFP_KERNEL);
	if (!s->buffers) {
		printk("*E* kmalloc\n");
		goto err;
	}
	memset(s->buffers, 0, sizeof(struct audio_buf_s) * s->nbfrags);


	for (frag = 0; frag < s->nbfrags; frag++) {
		struct audio_buf_s *b = &s->buffers[frag];

		/*
		 * Let's allocate non-cached memory for DMA buffers.
		 * We try to allocate all memory at once.
		 * If this fails (a common reason is memory fragmentation),
		 * then we allocate more smaller buffers.
		 */
		if (!dmasize) {
			dmasize = (s->nbfrags - frag) * s->fragsize;
			do {
				dmabuf = dma_alloc_coherent(s->dev, dmasize, &dmaphys, GFP_KERNEL|GFP_DMA);
				if (!dmabuf)
					dmasize -= s->fragsize;

			} while (!dmabuf && dmasize);
			if (!dmabuf) {
				printk("*E* dma_alloc\n");
				goto err;
			}
			b->master = dmasize;
			memzero(dmabuf, dmasize);
		}

		b->data = dmabuf;
		b->dma_addr = dmaphys;
		DPRINTK("buf %d: start %p dma %#08x\n", frag, b->data, b->dma_addr);

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
 * Driver interface functions.
 */
static int audio_write(struct file *file, const char *buffer,
			size_t count, loff_t *ppos)
{
	const char *buffer0 = buffer;
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *s = state->output_stream;
	int chunksize, ret = 0;
	unsigned long flags;
	/*
	unsigned int i ,j;
	unsigned short value;
	printk("vt1613 regs\n");
	for(j = 0; j < 8; j++) {
		for(i = 0; i < 0xF; i = i + 2) {
		  value = (unsigned short) (-1);
		  codec->ops->read((j * 16 + i), &value);
		  printk("0x%.4x  ", value);
		}
		printk("\n");
	}
	*/


	if (*ppos != file->f_pos)
		return -ESPIPE;
	if (s->mapped)
		return -ENXIO;
	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;

//--> added by howayhuo pull down GPIO3 to enable the volume
//        if(REG32_VAL(0xD81100B4)  & BIT3)
//            REG32_VAL(0xD81100B4) &= ~BIT3;  //GPIO3 Output Low
       if(!s->active)
       	{
       	        printk("---- audio write: s->active = 0\n");
	   	audio_amp_powerup(1);
       	}
//<-- end added
 //       printk("--audio_write: count = %d\n", count);
	while (count > 0) {
		struct audio_buf_s *b = &s->buffers[s->usr_head];

		/*
		 * Wait for a buffer to become free.
		 */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (compat_down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (compat_down_interruptible(&s->sem))
				break;
		}

		/*
		 * Feed the current buffer.
		 */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
//		printk("write %d to %d\n", chunksize, s->usr_head); /* harry, trace system performance*/

		if (copy_from_user(b->data + b->offset, buffer, chunksize)) {
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

		/*
		 * Update pointers and send current fragment to DMA.
		 */
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
//	printk("audio_write: return=%d\n", ret);
	return ret;
}

static void audio_prime_rx(struct audio_state_s *state)
{
	struct audio_stream_s *is = state->input_stream;
	unsigned long flags;

	local_irq_save(flags);
	if (state->need_tx_for_rx) {
		/*
		 * VT1613 gets its clock from the ZAC.
		 * While there is no playback data to send, the output DMA
		 * will spin with all zeroes.  We use the cache flush special
		 * area for that.
		 */
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
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *s = state->input_stream;
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
		struct audio_buf_s *b = &s->buffers[s->usr_head];

		/*
		 * Wait for a buffer to become full.
		 */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (compat_down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (compat_down_interruptible(&s->sem))
				break;
		}

		/*
		 * Grab data from the current buffer.
		 */
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

		/*
		 * Update pointers and return current fragment to DMA.
		 */
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
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *s = state->output_stream;
	struct audio_buf_s *b;
	u_int shiftval = 0;
	unsigned long flags;
	DECLARE_WAITQUEUE(wait, current);

	DPRINTK("audio_sync\n");

	if (!(file->f_mode & FMODE_WRITE) || !s->buffers || s->mapped)
		return 0;

	/*
	 * Send current buffer if it contains data.  Be sure to send
	 * a full sample count.
	 */
	b = &s->buffers[s->usr_head];
	if (b->offset &= ~3) {
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

	/*
	 * Let's wait for all buffers to complete.
	 */
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
	while (s->active &&
	       (b->dma_ref || s->dma_tail != s->dma_head)
	       && !signal_pending(current)) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->wq, &wait);

	/*
	 * Undo the pointer hack above.
	 */
	if (shiftval) {
		local_irq_save(flags);
		b->dma_addr += shiftval;
		/*
		 * Ensure sane DMA code behavior if not yet processed.
		 */
		if (b->offset != 0)
			b->offset = s->fragsize;
		local_irq_restore(flags);
	}

	return 0;
}

static int audio_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *s;
	unsigned long size, vma_addr;
	int i, ret;

        printk("audio_mmap\n");
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
		struct audio_buf_s *buf = &s->buffers[i];
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
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *is = state->input_stream;
	struct audio_stream_s *os = state->output_stream;
	unsigned int mask = 0;

	printk("audio_poll(): mode=%s%s\n",
		(file->f_mode & FMODE_READ) ? "r" : "",
		(file->f_mode & FMODE_WRITE) ? "w" : "");

	if (file->f_mode & FMODE_READ) {
		/*
		 * Start audio input if not already active.
		 */
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

static int audio_set_fragments(struct audio_stream_s *s, int val)
{
	DPRINTK("+%s\n", __func__); /* harry*/

	if (s->active)
		return -EBUSY;
	if (s->buffers)
		audio_discard_buf(s);

	/* val ( 32bit:MMMMSSSS ) min size 2^SSSS, max size 2^MMMM (max 0x7FFFF) */
	s->nbfrags = (val >> 16) & 0x7FFF;	/* max size shift bit */
	val &= 0xffff;	/* min size shift bit */

	if (val < 4)	/* non dma min buf size = 16 bytes */
		val = 4;
	if (s->wmt_dma && val < 11)	/* dma min buf size = 2k bytes */
		val = 11;
	if (val > 15)	/* max buf size = 32k bytes */
		val = 15;

	s->fragsize = 1 << val;
	if (s->nbfrags < 2)	/* min buffer nr = 2 */
		s->nbfrags = 2;
	if (s->nbfrags * s->fragsize > 128 * 1024)	/* max total buf size 128k */
		s->nbfrags = 128 * 1024 / s->fragsize;
	if (audio_setup_buf(s))
		return -ENOMEM;

	printk("audio_set_fragment : num %d, size %d \n", s->nbfrags, s->fragsize);
	return val|(s->nbfrags << 16);

}

static int audio_ioctl(struct inode *inode, struct file *file,
			uint cmd, ulong arg)
{
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *os = state->output_stream;
	struct audio_stream_s *is = state->input_stream;
	long val;

	/*
	 * Dispatch based on command.
	 */
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
			/* Recording */
			int ret = audio_set_fragments(is, val);

			if (ret < 0)
				return ret;

			ret = put_user(ret, (int *)arg);

			if (ret)
				return ret;
		}
		if (file->f_mode & FMODE_WRITE) {
			/* Playback */
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

		printk("SNDCTL_DSP_SETTRIGGER\n");

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
		struct audio_stream_s *s = (cmd == SNDCTL_DSP_GETOPTR) ? os : is;
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
		audio_buf_info inf = { 0, };
		struct audio_stream_s *s = (cmd == SNDCTL_DSP_GETOSPACE) ? os : is;

		printk("SNDCTL_DSP_GETISPACE\n");

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
//--> patched by howayhuo for android 1.6
//we do the reset one time
		if(!firstset)
			return 0;
//<-- end patch
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
		DPRINTK("audio_ioctl default %x \n", cmd);
		/*
		 * Let the client of this module handle the
		 * non generic ioctls
		 */
		return state->client_ioctl(inode, file, cmd, arg);
	}

	return 0;
}

static int audio_release(struct inode *inode, struct file *file)
{
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *os = state->output_stream;
	struct audio_stream_s *is = state->input_stream;
	unsigned short reg = 0;

	printk("audio_release\n");

	compat_down(&state->sem);

	codec->ops->read(VTAC_MASV, &reg);/*Dean add 2008/07/22 for loopback disable*/
	reg |= 0x8000;
	codec->ops->write(VTAC_MASV, reg);
	codec->ops->read(VTAC_HPSV, &reg);
	reg |= 0x8000;
	codec->ops->write(VTAC_HPSV, reg);

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

	firstset = 1;
	/*remove by Dean 2008/7/22*/
	/*Must disable before hw_shutdown*/
	/*codec->ops->write(VTAC_GEPR, 0x0080);	// loopback mode enable ;*/
						/*turn off by Dean 2007/12/11*/
	/*codec->ops->read(VTAC_MASV,&reg);//Dean add 2007/12/11 for loopback disable*/
	/*reg |= 0x8000;*/
	/*codec->ops->write(VTAC_MASV,reg);*/
	/*codec->ops->read(VTAC_HPSV,&reg);*/
	/*reg |= 0x8000;*/
	/*codec->ops->write(VTAC_HPSV,reg);*/

	compat_up(&state->sem);
	return 0;
}

static int wmt_audio_attach(struct inode *inode, struct file *file,
			struct audio_state_s *state)
{
	struct audio_stream_s *os = state->output_stream;
	struct audio_stream_s *is = state->input_stream;
	int err, need_tx_dma;

	compat_down(&state->sem);

	/* Access control */
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

	/* Request DMA channels */
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

	/* now complete initialisation */
	if (!AUDIO_ACTIVE(state)) {
		if (state->hw_init)
			state->hw_init(state->data);
	}
	/*codec->ops->write(VTAC_GEPR, 0x0000);	// loopback mode disable*/


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

static int ac97_volume_arg_trans(int val, int max)
{
	unsigned short left, right;

        //--> modified by howayhuo for setting volume on left and right channel
        //-- original
	//left = (unsigned short)(val & 0xff);            /* left channel vol*/
	//right = (unsigned short)((val >> 8) & 0xff);    /* right channel vol*/
        //-- new
	left = right = (unsigned short)(val & 0xff);            /* left and right channel vol*/
        //<-- modification
	codec->mod++;

	/* max:100, min:0 */
	if (left > 100)
		left = 100;


	if (right > 100)
		right = 100;


	/* Convert vol degree to attenuation. */
	right =  max - (max * right / 100);           /* right channel atte*/
	left =  max - (max * left / 100);             /* left channel atte*/

	/* Notice the byte order is different with val */
	return right | (left << 8);

}


static int ac97_mixer_ioctl(int cmd, void *arg)
{
	int val, nr, ret;
        unsigned short left, right;

	ret = -EINVAL;
	nr = _IOC_NR(cmd);

	if (_IOC_TYPE(cmd) != 'M')
		return ret;

	if (cmd == SOUND_MIXER_INFO) {

		struct mixer_info mi;

		printk("SOUND_MIXER_INFO\n");

		strncpy(mi.id, "VT1613", sizeof(mi.id));
		strncpy(mi.name, "VIA VT1613", sizeof(mi.name));
		mi.modify_counter = codec->mod;
		return copy_to_user(arg, &mi, sizeof(mi)) ? -EFAULT : 0;
	}

	ret = 0;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if ((ret = get_user(val, ((int *)arg))) != 0)
			goto out;


		switch (nr) {
		case SOUND_MIXER_MIC_MUTE:
		{
			audio_mic_mute(val);
			break;
		}
		case SOUND_MIXER_PCM:
		{
			unsigned short reg;

			printk("SOUND_MIXER_PCM\n");

			codec->volume = val;

			/* Write to setting control reg. */
			if (val == 0) {	/* mute*/
				codec->ops->write(VTAC_MASV, VTAC_MASV_MM);
				codec->ops->write(VTAC_HPSV, VTAC_MASV_MM);
			} else {
				reg = ac97_volume_arg_trans(val, max_volume);

				codec->ops->write(VTAC_MASV, reg);	/* Stereo Output */
				codec->ops->write(VTAC_HPSV, reg);	/* Headphone Stereo Output */
			}
		}
			break;
		case SOUND_MIXER_VOLUME:
//--> patched by howayhuo for android 1.6
//we don't set the volume register to adjust the volume
//At the AC97 inilization, the volume is set to max. Android use software to adjust the volumn
		printk("SOUND_MIXER_VOLUME. volume val = %d\n", val);
		codec->volume = val;
		ret = 0;
		codec->ops->read(VTAC_MASV, &left);
                codec->ops->read(VTAC_HPSV, &right);
                printk("left = 0x%04x, right = 0x%04x\n", left, right);
#if 0
		{
			unsigned short reg;

			DPRINTK("SOUND_MIXER_VOLUME\n");

			codec->volume = val;

			/* Write to setting control reg. */
			if (val == 0) {	/* mute*/
				codec->ops->write(VTAC_MASV, VTAC_MASV_MM);
				codec->ops->write(VTAC_HPSV, VTAC_MASV_MM);
			} else {
				reg = ac97_volume_arg_trans(val, max_volume);

				codec->ops->write(VTAC_MASV, reg);	/* Stereo Output */
				codec->ops->write(VTAC_HPSV, reg);	/* Headphone Stereo Output */
			}
		}
#endif
//<-- end patch
			break;

		/*
		 * FIXME : Following bass and treble need to be verified.
		 */
		case SOUND_MIXER_BASS:
		{
			unsigned short reg;

			printk("SOUND_MIXER_BASS\n");

			/*
			 * FIXME: I assumed val should between 0 and 100
			 */
			if (val > 100)
				val = 100;
			if (val < 0)
				val = 0;

			codec->bass = val;
			codec->mod++;

			/*
			 * Convert bass degree to BB[3:0]+M[1:0], max:24, min:0.
			 */
			val = (24 * val / 100);

			if (val >= 18)                /* use max mode*/
				val = (((val/2) << 11) | VTAC_FCS1_MAXMODE);
			else
			if (val > 0 && val < 18)      /* use min mode*/
				val = (((val/2) << 11) | VTAC_FCS1_MINMODE);
			/* else val will be 0 (flat mdoe) */

			/* Read then write to vol control reg. */
			codec->ops->read(VTAC_FCS1, &reg);
			codec->ops->write(VTAC_FCS1,
				(reg & ~(VTAC_FCS1_MODEMASK | VTAC_FCS1_BBMASK)) | val);
			/*
			 * Conflict! It will override previous treble setting.
			 */
		}
			break;

		case SOUND_MIXER_TREBLE:
		{
			unsigned short reg;

			printk("SOUND_MIXER_TREBLE\n");

			/*
			 * FIXME: I assumed val should between 0 and 100.
			 */
			if (val > 100)
				val = 100;
			if (val < 0)
				val = 0;

			codec->treble = val;
			codec->mod++;

			/*
			 * Convert bass degree to TR[1:0], max:6, min:0.
			 */
			val = (6 * val / 100 / 2);

			/* Read then write to vol control reg. */
			codec->ops->read(VTAC_FCS1, &reg);
			if ((reg & VTAC_FCS1_MODEMASK) && val) /* keep mode setting*/
				codec->ops->write(VTAC_FCS1,
					((reg & ~VTAC_FCS1_TRMASK) | (val << 9)));
			else                                    /* flat mode*/
				codec->ops->write(VTAC_FCS1, reg & ~VTAC_FCS1_MODEMASK);
			/*
			 * Conflict! If (mode!=0 && val==0) will override previous
			 * bass setting.
			 */
		}
			break;
		case SOUND_MIXER_LINE:	/* Line in volume */
		{
			unsigned short reg = 0;

			codec->line = val;
			/* Write to setting control reg. */
			if (val == 0) {	/* mute*/
				codec->ops->write(VTAC_LINV, VTAC_MASV_MM);
			} else {
				reg = ac97_volume_arg_trans(val, 0x1F);

				codec->ops->write(VTAC_LINV, reg);
			}
			printk("SOUND_MIXER_LINE 0x%x, reg 0x%x\n", val, reg);
		}
			break;
		case SOUND_MIXER_MIC:	/* Mic in volume */
		{
			unsigned short reg = 0;

			codec->mic = val;
			/* Write to setting control reg. */
			if (val == 0)	/* mute*/
				codec->ops->write(VTAC_MICV, VTAC_MASV_MM);

			else {
				reg = ac97_volume_arg_trans(val, 0x1F);
				reg |= VTAC_MICV_BOOST;
				codec->ops->write(VTAC_MICV, reg);
			}
			printk("SOUND_MIXER_MIC 0x%x, reg 0x%x\n", val, reg);
		}
			break;
		case SOUND_MIXER_RECSRC:
			printk("SOUND_MIXER_RECSRC\n");
			codec->mod++;
			if (val & SOUND_MASK_MIC)
				codec->ops->write(VTAC_RECS, 0x0000);
			else
				codec->ops->write(VTAC_RECS, 0x0404);

			break;
		default:
			printk("ac97_mixer_ioctl default\n");
			ret = -EINVAL;
			break;
		}
	} /* IOC_WRITE */

	if ((ret == 0) && (_IOC_DIR(cmd) & _IOC_READ)) {
		switch (nr) {
		case SOUND_MIXER_PCM:
			val = codec->volume;
			break;
		case SOUND_MIXER_VOLUME:
			val = codec->volume;
			break;
		case SOUND_MIXER_BASS:
			val = codec->bass;
			break;
		case SOUND_MIXER_TREBLE:
			val = codec->treble;
			break;
		case SOUND_MIXER_LINE:
			val = codec->line;
			break;
		case SOUND_MIXER_MIC:
		{
			unsigned short reg;

			val = codec->mic;
			codec->ops->read(VTAC_MICV, &reg);
		}
			break;
		case SOUND_MIXER_RECSRC:
		{
			unsigned short reg;

			codec->ops->read(VTAC_RECS, &reg);
			if (reg & 0x04)
				val = SOUND_MASK_LINE;
			else
				val = SOUND_MASK_MIC;
		}
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_CAPS:
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = 0;
			break;
		default:
			val = 0;
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);

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
	return ac97_mixer_ioctl(cmd, (void *)arg);
}

static struct file_operations wmt_mixer_fops = {
	.owner  = THIS_MODULE,
	.ioctl  = mixer_ioctl,
};

/*
 * Audio interface
 */
static long adc_rate = AUDIO_RATE_DEFAULT;
static long dac_rate = AUDIO_RATE_DEFAULT;

static void audio_set_adc(long val)
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

void audio_set_dac(long val)
{
	unsigned short dac;

	switch (val) {
	case 8000:
		if (disspdif == 0)
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
		dac = 0x1F40;
		break;
	case 11025:
		if (disspdif == 0)
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
		dac = 0x2B11;
		break;
	case 12000:
		if (disspdif == 0)
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
		dac = 0x2EE0;
		break;
	case 16000:
		if (disspdif == 0)
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
		dac = 0x3E80;
		break;
	case 22050:
		if (disspdif == 0)
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
		dac = 0x5622;
		break;
	case 24000:
		if (disspdif == 0)
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
		dac = 0x5DC0;
		break;
	case 32000:
		if (disspdif == 0) {
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
			codec->ops->write(VTAC_SPDIF, VTAC_SPDIF_SR0|VTAC_SPDIF_SR1);
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA|VTAC_EASC_SPDIF);
		}
		dac = 0x7D00;
		break;
	case 44100:
		if (disspdif == 0) {
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
			codec->ops->write(VTAC_SPDIF, 0x0000);
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA|VTAC_EASC_SPDIF);
		}
		dac = 0xAC44;
		break;
	case 48000:
		if (disspdif == 0) {
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);
			codec->ops->write(VTAC_SPDIF, VTAC_SPDIF_SR1);
			codec->ops->write(VTAC_EASC, VTAC_EASC_VRA|VTAC_EASC_SPDIF);
		}
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

static void wmt_audio_init(void *dummy)
{
	unsigned short reg;
	int retval;
	unsigned char buf[20]={0};

//--> added by howayhuo pull up GPIO3 to mute the volume
//        REG32_VAL(0xD8110064) |= BIT3;  //GPIO3 signal GPIO Enable
//        REG32_VAL(0xD811008C) |= BIT3;  //GPIO3 Output Enable
//        REG32_VAL(0xD81100B4) |= BIT3;  //GPIO3 Output High
        audio_amp_powerup(0);
//<-- end added
	printk("--wmt_audio_init\n");

	/*
	 * Enable/wakeup VT1613 CODEC.
	 */
	if (codec->ops->aclink)
		codec->ops->init();



	/*
	 * Playback presettings.
	 *
	 * Enable audio DAC output and ADC input path.
	 */
	codec->ops->read(VTAC_PDCS, &reg);
	reg &= ~(VTAC_PDCS_PR0 | VTAC_PDCS_PR1);
	codec->ops->write(VTAC_PDCS, reg);

	/*
	 * Wait DAC and ADC sections are ready.
	 */
		DPRINTK("wait DAC and ADC ready\n");
	do {
		codec->ops->read(VTAC_PDCS, &reg);
	} while ((reg & (VTAC_PDCS_ADC | VTAC_PDCS_DAC)) != (VTAC_PDCS_ADC | VTAC_PDCS_DAC));

	/*
	 * Harry's test to lower volume by adding more attenuation.
	 */
	codec->volume = AUDIO_VOLUME_DEFAULT;

	/*codec->ops->write(VTAC_MASV, (codec->volume | (codec->volume << 8)));*/
	codec->ops->write(VTAC_MASV, 0x0000);

	codec->ops->write(VTAC_HPSV, 0x0000);	/* ****/

//--> modified by howayhuo to increase the  Line Out volume and Head Phone volume
//-- original
//	codec->ops->write(VRAC_PCMV, 0x0808);
//-- new
    if(!wmt_getsyspara_cache("pcmvolum", buf, 200))
	{
	    printk(KERN_ALERT "pcmvolum:%s\n", buf);
        if (!strcmp(buf, "puzhi"))
        {
        	codec->ops->write(VRAC_PCMV, 0x0404); // max volume
        	printk(KERN_ALERT "set pcm volume successfully for puzhi!\n");
        }
	} else{
        codec->ops->write(VRAC_PCMV, 0x0808);
        printk(KERN_ALERT "set normal pcm volume!\n");
    }
//<-- end modification
	/*codec->ops->write(VTAC_EINT, 0xFFFF);*/
	/*codec->ops->write(VTAC_PLL, 0x0000);*/


	/*
	 * Enable Variable Rate Audio (VRA) mode.
	 */
	codec->ops->write(VTAC_SPDIF, VTAC_SPDIF_SR1);
	if (disspdif == 0)
		codec->ops->write(VTAC_EASC, VTAC_EASC_VRA|VTAC_EASC_SPDIF);
	else
	codec->ops->write(VTAC_EASC, VTAC_EASC_VRA);

	/*
	 * Recording presettings.
	 *
	 */
	 retval = wmt_getsyspara_cache("audio.recordselect", buf, 20);
	 if(!retval)
	 {
	     if(!strcmp(buf,"LineIn"))
	     {
	         printk("Audio Line In\n");
		 codec->ops->write(VTAC_RECS, 0x0404);   //select LIne IN
	     }
             else
             {
                 printk("Audio MIC In\n");
		 codec->ops->write(VTAC_RECS, 0x4000);   // select MIC IN
             }
	 }
	 else
	 {
	     printk("Audio MIC In\n");
 	     codec->ops->write(VTAC_RECS, 0x4000);   //Default select MIC IN
	 }

//--> modified by howayhuo to increase the MIC volume
//-- original
//	codec->ops->write(VTAC_RECG, 0x0404);
//	codec->ops->write(VTAC_MICV, 0x8008);
//-- new
      codec->ops->write(VTAC_RECG, 0x7f7f);
      codec->ops->write(VTAC_MICV, 0x00e0);
//<-- end modification

//--> modified by howayhuo to increase the MIC volume
//-- original
//	codec->ops->write(VTAC_LINV, 0x8808);
//-- new
    codec->ops->write(VTAC_LINV, 0x1f1f);
//<--

	/*
	 * Extra presettings.
	 *
	 * Enable headphone driver and de-emphasis.
	 */
	codec->ops->read(VTAC_FCS1, &reg);
	codec->ops->write(VTAC_FCS1, (reg | 0x60));
	codec->ops->write(VTAC_IODA, 0x0401);

	/*codec->ops->write(VTAC_EINT, 0xFFFF);*/
	/*codec->ops->write(VTAC_PLL, 0x0000);*/
	codec->ops->write(VTAC_EINT, 0x0000);
	codec->ops->write(VTAC_PLL, 0x0084);
	/*
	 * Enable PCM_TX, PCM_RX, MIC_IN FIFOs.  stereo , 16bit
	 */
	ac97.regs->PTFC |= PTFC_PTFEN | PTFC_PTFADE;
//--> added by howayhuo
	ac97.regs->PTFC &= ~PTFC_PTFMM;  //stereo
	ac97.regs->PTFC &= ~PTFC_PTF8M; // 16bit
//<-- end add

//--> modified by howayhuo to set RX to Mono 16bit
//-- org
//        ac97.regs->PRFC |= PRFC_PRFEN | PRFC_PRFADE
//-- new
	ac97.regs->PRFC |= PRFC_PRFEN | PRFC_PRFADE | PRFC_PRFMM; //Mono
        ac97.regs->PRFC &= ~PRFC_PRF8M; //16 bit

//<-- end modify
	/*ac97.regs->MIFC |= MIFC_MFEN | MIFC_MFADE;*/
	/*printk("%s    vt1613 regs\n",__FUNCTION__);*/
	/*for(j = 0; j < 8; j++)*/
	/*{*/
	/*  for(i = 0; i < 0xF; i = i + 2)*/
	/*  {*/
	/*    value = (unsigned short) (-1);*/
	/*    codec->ops->read((j * 16 + i), &value);*/
	/*    printk("0x%.4x  ", value);*/
	/*  }*/
	/*  printk("\n");*/
	/*}*/
//--> added by howayhuo pull down GPIO3 to enable the volume
//        REG32_VAL(0xD81100B4) &= ~BIT3;  //GPIO3 Output Low
//<-- end added
	wm9715_gpioirq_adcandpen();
	audio_mic_mute(true);
//        if(audio_init_flag)
//        {
//            printk("wmt audio already init, release audio\n");
	    audio_set_adc(8000);
	    audio_set_dac(44100);
//        }

//	audio_init_flag = 1;
	return;
}

/*
 * Shutdown the audio driver.
 */
static void wmt_audio_shutdown(void *dummy)
{

	DPRINTK("wtm_audio_shutdown \n");
	ac97.regs->PTFC &= ~(PTFC_PTFEN | PTFC_PTFADE);
	ac97.regs->PRFC &= ~(PRFC_PRFEN | PRFC_PRFADE);
	ac97.regs->ACGS |= ACGS_CPD;
	codec->ops->exit();

	/*
	 * Disable audio DAC output and ADC input path.
	 */
#if 0
	codec->ops->read(VTAC_PDCS, &reg);
	reg |= (VTAC_PDCS_PR0 | VTAC_PDCS_PR1);
	codec->ops->write(VTAC_PDCS, reg);
#endif
	/*
	 * Disable PCM_TX, PCM_RX, MIC_IN FIFOs.
	 */
	/*ac97.regs->PTFC &= ~(PTFC_PTFEN | PTFC_PTFADE);*/
	/*ac97.regs->PRFC &= ~(PRFC_PRFEN | PRFC_PRFADE);*/
	/*ac97.regs->ACGS |= ACGS_CPD;*/
	/*ac97.regs->MIFC &= ~(MIFC_MFEN | MIFC_MFADE);*/
}

static int wmt_audio_ioctl(struct inode *inode, struct file *file,
			uint cmd, ulong arg)
{
	long val = 0;
	int ret = 0;
//--> added by howayhuo on 20100210
	struct audio_state_s *state = file->private_data;
	struct audio_stream_s *os = state->output_stream;
	struct audio_stream_s *is = state->input_stream;
//<-- end add

	DPRINTK("cmd 0x%x\n", cmd);

	/*
	 * These are machine dependent ioctls.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:

		printk("SNDCTL_DSP_STEREO\n");

		/*
		 * VT1613 support both stereo and mono.
		 */
		ret = get_user(val, (long *) arg);

		if (ret)
			return ret;

		if (val == AUDIO_MODE_MONO) {
			ac97.regs->PTFC |= PTFC_PTFMM;
			ac97.regs->PRFC |= PRFC_PRFMM;
		} else {
			ac97.regs->PTFC &= ~PTFC_PTFMM;
			ac97.regs->PRFC &= ~PRFC_PRFMM;
		}

		DPRINTK("SNDCTL_DSP_STEREO %s PTFC=0x%x\n", (val == 0) ?
			"mono" : "stereo", ac97.regs->PTFC);

		return put_user(val, (int *) arg);

	case SNDCTL_DSP_CHANNELS:

		if (get_user(val, (long *) arg))
			return -EFAULT;

                printk("SNDCTL_DSP_CHANNELS, value= %ld\n", val);

		if (val != 1 && val != 2)
			return -EINVAL;


//--> patched by howayhuo for android 1.6.  Please fix it if you have good idea
//refer to SPM_WM8505_071  P453
                if(!firstset) //We only set the channel one time
			return put_user(val, (int *) arg);

               if (file->f_mode & FMODE_READ)  //Recording
	           ac97.regs->PRFC |= PRFC_PRFMM;

	       if (file->f_mode & FMODE_WRITE)  //Playback
		   ac97.regs->PTFC &= ~PTFC_PTFMM;

#if 0
		if (file->f_mode & FMODE_READ) {
			/* Recording */
			ac97.regs->PRFC &= ~PRFC_PRFMM;
			if (val == 1)
				ac97.regs->PRFC |= PRFC_PRFMM;
		}
		if (file->f_mode & FMODE_WRITE) {
			/* Playback */
			ac97.regs->PTFC &= ~PTFC_PTFMM;
			if (val == 1)
				ac97.regs->PTFC |= PTFC_PTFMM;
		}
#endif
//<-- end patch
		return put_user(val, (int *) arg);

	case SOUND_PCM_READ_CHANNELS:

		printk("SOUND_PCM_READ_CHANNELS\n");

		if (file->f_mode & FMODE_READ) {
			/* Recording */
			if (ac97.regs->PRFC & PRFC_PRFMM)
				return put_user(1, (long *) arg);
			else
				return put_user(2, (long *) arg);
		}
		if (file->f_mode & FMODE_WRITE) {
			/* Playback */
			if (ac97.regs->PTFC & PTFC_PTFMM)
				return put_user(1, (long *) arg);
			else
				return put_user(2, (long *) arg);
		}

		return -EINVAL;

	case SNDCTL_DSP_SPEED:

		if (get_user(val, (long *) arg))
			return -EFAULT;

                printk("SNDCTL_DSP_SPEED, val=%ld\n", val);

//--> patched by howayhuo for android 1.6.  Please fix it if you have good idea
//refer to SPM_WM8505_071  P453
//WM9715L_Rev3.9 P37

		if(!firstset) //we only set the sample rate one time
			return put_user(val, (long *) arg);
		else //because the sample rate is set at last, change the "firstset" flag to 0
			firstset = 0;

		if (file->f_mode & FMODE_READ)
		{
			//audio_set_adc(val);
			audio_set_adc(8000);
		        audio_discard_buf(is);
			audio_setup_buf(is);
		}
		if (file->f_mode & FMODE_WRITE)
		{
			//audio_set_dac(val);
			audio_set_dac(44100);
	                audio_discard_buf(os);
			audio_setup_buf(os);
		}
		return put_user(val, (long *) arg);
//<-- end patch

	case SOUND_PCM_READ_RATE:

		printk("SOUND_PCM_READ_RATE\n");

		if (file->f_mode & FMODE_READ) {
			/* Recording */
			val = adc_rate;
		}
		if (file->f_mode & FMODE_WRITE) {
			/* Playback */
			val = dac_rate;
		}
		return put_user(val, (long *) arg);

	case SNDCTL_DSP_SETFMT:

		if (get_user(val, (long *) arg))
			return -EFAULT;
                printk("SNDCTL_DSP_SETFMT, val = 0x%lx\n", val);
		if (val != AFMT_U8 && val != AFMT_S16_LE)
			return -EINVAL;
//--> patched by howayhuo for android 1.6
                 if(!firstset)  //We only set the format register one time
			return put_user(val, (int *)arg);
//<-- end patch
		if (val & AUDIO_FMT_MASK) {
			if (file->f_mode & FMODE_READ) {
				/* Recording */
				ac97.regs->PRFC &= ~PRFC_PRF8M;
				if (val == AFMT_U8)
					ac97.regs->PRFC |= PRFC_PRF8M;
			}
			if (file->f_mode & FMODE_WRITE) {
				/* Playback */
				ac97.regs->PTFC &= ~PTFC_PTF8M;
				if (val == AFMT_U8)
					ac97.regs->PTFC |= PTFC_PTF8M;
			}

			return put_user(val, (int *)arg);
		} else
			return -EINVAL;

	case SNDCTL_DSP_GETFMTS:

		printk("SNDCTL_DSP_GETFMTS\n");

		/*
		 * VT1613 support unsigned 8-bit and signed 16-bit.
		 */
		return put_user(AFMT_U8 | AFMT_S16_LE, (int *)arg);
	default:
		/*
		 * Maybe this is meant for the mixer (As per OSS Docs).
		 */
		return mixer_ioctl(inode, file, cmd, arg);
	}

	return ret;
}

static struct audio_stream_s output_stream = {
	.id             = "ac97_out",
	.dma_dev        = AC97_PCM_TX_DMA_REQ,
	.dmach          = NULL_DMA,
	.direction      = 1,
};

static struct audio_stream_s input_stream = {
	.id             = "ac97_in",
	.dma_dev        = AC97_PCM_RX_DMA_REQ,
	.dmach          = NULL_DMA,
	.direction      = 0,
};

static struct audio_state_s audio_state = {
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
static int wmt_audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct audio_state_s *s = &audio_state;
	struct audio_stream_s *is = s->input_stream;
	struct audio_stream_s *os = s->output_stream;
	int stopstate;

	wm9715_gpioirq_clear();
	wm9715_gpioirq_comp2();

	ac97_reg_backup();
        vt1613_reg_backup();

	if (is && CHK_DMACH(is->dmach)) {
		ac97.regs->PRFC &= ~ PRFC_PRFADE ;
                udelay(5);
		DPRINTK("suspend input\n");
		stopstate = is->stopped;
		audio_stop_dma(is);
/*		DMA_CLEAR(is);	*/
		is->dma_spinref = 0;
		is->stopped = stopstate;
	}
	if (os && CHK_DMACH(os->dmach)) {
		ac97.regs->PTFC &= ~ PTFC_PTFADE;
                udelay(5);
		DPRINTK("suspend output\n");
		stopstate = os->stopped;
		audio_stop_dma(os);
/*		DMA_CLEAR(os);	*/
		os->dma_spinref = 0;
		os->stopped = stopstate;
	}
	/* ac97_reg_dump();*/
	/* vt1613_reg_dump();*/

	DPRINTK("state %d, %x\n", state, (ac97.regs->ACGS & ACGS_CPD));
	if (AUDIO_ACTIVE(s) && s->hw_shutdown) {
		DPRINTK("suspend hw\n");
		s->hw_shutdown(s->data);
	}
	 /*REG32_VAL(0xD8110054) |= BIT4;
         REG32_VAL(0xD811007C) |= BIT4;
         REG32_VAL(0xD81100A4) |= BIT4; */

	return 0;
}

//extern void wmt_resume_descript(dmach_t ch);
//extern void wmt_run_dma(dmach_t ch);
static int wmt_audio_resume(struct platform_device *pdev)
{
	struct audio_state_s *s = &audio_state;
	struct audio_stream_s *is = s->input_stream;
	struct audio_stream_s *os = s->output_stream;
	struct audio_buf_s *b;

	if (AUDIO_ACTIVE(s) && s->hw_init) {
		DPRINTK("resume hw\n");
//		audio_init_flag = 0;
		s->hw_init(s->data);
	}
//        REG32_VAL(0xD8110064) |= BIT3;  //GPIO3 signal GPIO Enable
//        REG32_VAL(0xD811008C) |= BIT3;  //GPIO3 Output Enable
//        REG32_VAL(0xD81100B4) |= BIT3;  //GPIO3 Output High

	wm9715_gpioirq_clear();
        audio_amp_powerup(0);
	ac97_reg_restore();
	vt1613_reg_restore();
	codec->ops->write(VTAC_MASV, 0x8000);
	codec->ops->write(VTAC_HPSV, 0x8000);
	/*ac97_reg_dump();*/
	/*vt1613_reg_dump();*/
	if (os && os->buffers && CHK_DMACH(os->dmach)) {
/*		DMA_RESET(os);	*/
                printk("resume os\n");
 		b = &os->buffers[os->dma_head];
		if ((os->dma_head != os->dma_tail) || b->dma_ref) {
		wmt_setup_dma(os->dmach, os->dma_cfg) ;
		wmt_resume_dma(os->dmach) ;
		}
//--> added by howayhuo
		else
		{
		    wmt_setup_dma(os->dmach, os->dma_cfg) ;
		    if(os->active)
		    {
		        wmt_resume_dma(os->dmach) ;
			mdelay(50);
		    }
		}
//<-- end add
/*		audio_process_dma(os);	*/
	}

	if (is && is->buffers && CHK_DMACH(is->dmach)) {
/*		DMA_RESET(is);	*/
                printk("resume is\n");
 		b = &is->buffers[is->dma_head];
		if ((is->dma_head != is->dma_tail) || b->dma_ref) {
		printk("resume is: 1");
		wmt_setup_dma(is->dmach, is->dma_cfg) ;
		wmt_resume_dma(is->dmach) ;
		}
//--> added by howayhuo
		else
		{
		        printk("resume is: 2\n");
			wmt_setup_dma(is->dmach, is->dma_cfg) ;
			if(is->active)
			{
			    printk("resume is: resume dma\n");
			    wmt_resume_dma(is->dmach) ;
			}
		}
//<-- end add
/*		audio_process_dma(is);	*/
	}
	printk("resume complete\n");
        codec->ops->write(VTAC_MASV, 0x0000);
	codec->ops->write(VTAC_HPSV, 0x0000);
//	REG32_VAL(0xD81100B4) &= ~BIT3;  //GPIO3 Output Low
        audio_amp_powerup(1);
	DPRINTK(" ac97.regs->ACGS & ACGS_CPD%x\n", (ac97.regs->ACGS & ACGS_CPD));

	return 0;
}
#else
#define wmt_audio_suspend    NULL
#define wmt_audio_resume     NULL
#endif

static int wmt_audio_probe(struct platform_device *pdev)
{

	unsigned char buf[80];
	int retval;
	char varname[] = "audio_max_gain";
	REG32_VAL(0xd8130000+0x250) |= 0x00080000;  /*enable pmc clock*/

	codec = codec_attach();

	if (!codec)
		return -ENODEV;

	output_stream.dev = &pdev->dev;
	output_stream.dma_cfg = dma_device_cfg_table[AC97_PCM_TX_DMA_REQ],
	input_stream.dev = &pdev->dev;
	input_stream.dma_cfg = dma_device_cfg_table[AC97_PCM_RX_DMA_REQ],

	/*
	 * Register devices to sound core.
	 */
	audio_dev_id = register_sound_dsp(&wmt_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&wmt_mixer_fops, -1);

	printk(KERN_INFO "Sound: WMT AC97: dsp id %d mixer id %d\n",
		audio_dev_id, mixer_dev_id);

	/*
	 * AC'97 controller FIFO(s) presetting.
	 */
	ac97.regs->PTFC = PTFC_PTFT(8);
	ac97.regs->PRFC = PRFC_PRFT(8);
	ac97.regs->MIFC = MIFC_MFT(8);

	/*set codec info*/
	codec->volume = (AUDIO_VOLUME_DEFAULT << 8) | AUDIO_VOLUME_DEFAULT;
	retval = wmt_getsyspara_cache(varname, buf, 80);
	if (retval != 0)
		printk("Max. audio gain was default value(0x%x)\n", max_volume);
	else
		sscanf(buf, "%x", &max_volume);
	sprintf(varname,"wmt_disspdif");
	retval = wmt_getsyspara_cache(varname, buf, 80);
	if (retval != 0)
		disspdif = 1;
	else
		sscanf(buf, "%x", &disspdif);
	/*wmt_audio_init(0);*/
	/*codec->ops->write(VTAC_GEPR, 0x0080);	// loopback mode enable*/

	return 0;
}

static int wmt_audio_remove(struct platform_device *pdev)
{
	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);

	codec_detach(codec);

	return 0;
}

static void ac97_platform_release(struct device *device)
{
}

static struct resource wmt_ac97_resources[] = {
	[0] = {
	.start  = 0xD8290000,
	.end    = 0xD829ffff,
	.flags  = 0x00000200,        },
};


static u64 wmt_ac97_dma_mask = 0xffffffffUL;
static struct platform_device wmt_ac97_device = {


	.name           = "ac97",
	.id             = 0,
	.dev            = {
		.release = ac97_platform_release,
#if 1
		.dma_mask = &wmt_ac97_dma_mask,
		.coherent_dma_mask = ~0,
#endif
	},
	.num_resources  = ARRAY_SIZE(wmt_ac97_resources),
	.resource       = wmt_ac97_resources,
};

/*
static struct device_driver wmt_audio_driver = {
	.name           = "ac97",
	.bus            = &platform_bus_type,
	.probe          = wmt_audio_probe,
	.remove         = wmt_audio_remove,
	.suspend        = wmt_audio_suspend,
	.resume         = wmt_audio_resume
};
*/

struct platform_driver wmt_audio_driver = {
	.probe = wmt_audio_probe,
	.remove = wmt_audio_remove,
	.suspend        = wmt_audio_suspend,
	.resume         = wmt_audio_resume,
	.driver = { .name = "ac97" }
};

static int __init wmt_ac97_init(void)
{
	platform_device_register(&wmt_ac97_device);
	return platform_driver_register(&wmt_audio_driver);
}

static void __exit wmt_ac97_exit(void)
{
	platform_driver_unregister(&wmt_audio_driver);
	platform_device_unregister(&wmt_ac97_device);
}

module_init(wmt_ac97_init);
module_exit(wmt_ac97_exit);

MODULE_AUTHOR("WMT SW Team");
MODULE_DESCRIPTION("Wmt AC'97 Audio Driver");
MODULE_LICENSE("GPL");

