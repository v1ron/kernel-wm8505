/*++
	sound/arm/i2s.c

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
		The code was inherit from vt8420 aic-i2s.c
		2009/01/21 First Version

		2009/02/26 modify playback/record to support diffent sample(below 32k)

--*/

#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include "vt1602.h"
#include "i2s.h"

/*
 *  Debug macros
 */
#undef DEBUG
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define CONFIG_WMT_I2S_DEBUG        1       /* Remove this to static in the future*/
#define CONFIG_I2S_MASTER_MODE

#define	DRIVER_VERSION          "0.01"

#define CODEC_FAIL              (-1)
#define CODEC_MASK              (0x0000FFFF)

#define PMC_ADDR  (0xD8130000)
#define GPIO_ADDR (0xD8110000)
#define SCC_ADDR (0xD8120000)

static void i2s_init(void);
static void i2s_exit(void);


static int codec_read(unsigned short addr, unsigned int *data);
static int codec_write(unsigned short addr, unsigned int data);

static struct i2s_codec_s *codec;

struct codec_shadow_s s_codec_info ;
static unsigned int init_setting;/*check if first init*/
static unsigned int  wmt_i2s_suspend_flag;
#ifdef CONFIG_PM
static struct i2s_regs_s i2s_pm_regs;
#endif

struct codec_shadow_regs_s sreg_reset_value = {
	0x019,/*0x97*/
	0x019,/*0x97*/
	0x0F9,
	0x0F9,
	0x00A,
	0x004,
	0x007,
	0x001,
	0x020,
	0x000,
} ;

struct codec_shadow_regs_s sreg_cfg_value = {
	LINE_VOLUME_DEFAULT,
	LINE_VOLUME_DEFAULT,
	HEADPHONE_VOLUME_DEFAULT,
	HEADPHONE_VOLUME_DEFAULT,
	ANALOG_PATH_DEFAULT,
	DIGITAL_PATH_DEFAULT,
	POWER_ALL_ENABLE,
	DIGITAL_FORMAT_DEFAULT,
	44100 ,
	DIGITAL_ACT,
} ;

/*
 * wait queue head is using for waiting CODEC ready, CODEC read done,
 * and CODEC write done interrups.
 */
static DECLARE_WAIT_QUEUE_HEAD(codec_wq);

/*
 * Change this to static in the future
 */
struct i2s_s i2s = {
	(struct i2s_regs_s *)(io_p2v(I2S_BASE_ADDR)),
	/* interrupt counters*/
	{0, 0, 0, 0, 0, 0, 0, 0},
	/* irq number*/
	IRQ_I2S,
	/* reference counter*/
	0,
	i2s_init,
	i2s_exit,
};


/*
 * Export debug module for AC'97 audio driver.
 */
EXPORT_SYMBOL(i2s);

#ifdef CONFIG_WMT_I2S_DEBUG

/* wmt_i2s_interrupt()
 *
 * It's only interrupt counter now, might be useful to
 * debug or benchmark.
 * The CODEC I/O is asynchronous now, enable interrupt bits
 * in routine and then sleep, interrupts here will detect and
 * turn them off, Apr.18.2005 by Harry.
 */
static irqreturn_t
wmt_i2s_interrupt(int irq, void *dev_id)
{
	unsigned int fifo;

	/* Read and clear global interrupt sources.*/
	/* Read and clear MIFS interrupt sources*/
	fifo = i2s.regs->TSR;
	i2s.regs->TSR = fifo;

	DPRINTK("i2s Tx interrupt 0x%x \n", fifo);

	/* Carefully detect each toggled mic fifo interrupt and then mask it.*/
	if (fifo & TSR_TFE) {        /* Tx Fifo Empty*/
		i2s.ints.tfe++;
		i2s.regs->TCR &= ~TCR_TFEIE;
	}
	if (fifo & TSR_TFA) {        /* Tx FIFO Almost Empty*/
		i2s.ints.tfa++;
		i2s.regs->TCR &= ~TCR_TFAIE;
	}
	if (fifo & TSR_TFUE) {       /* Tx Fifo Underrun*/
		i2s.ints.tfue++;
		i2s.regs->TCR &= ~TCR_TFUIE;
	}
	if (fifo & TSR_TFOE) {       /* Tx Fifo Overrun*/
		i2s.ints.tfoe++;
		i2s.regs->TCR &= ~TCR_TFOIE;
	}

	/* Read and clear PRFS interrupt sources*/
	fifo = i2s.regs->RSR;
	i2s.regs->RSR = fifo;

	/* Carefully detect each toggled mic fifo interrupt and then mask it.*/
	if (fifo & RSR_RFF) {       /* Rx FIFO Full*/
		i2s.ints.rff++;
		i2s.regs->RCR &= ~RCR_RFFIE;
	}
	if (fifo & RSR_RFA) {       /* Rx FIFO Almost Full*/
		i2s.ints.rfa++;
		i2s.regs->RCR &= ~RCR_RFAIE;
	}
	if (fifo & RSR_RFUE) {      /* Rx FIFO Underrun Error*/
		i2s.ints.rfue++;
		i2s.regs->RCR &= ~RCR_RFUIE;
	}
	if (fifo & RSR_RFOE) {      /* Rx FIFO Overrun Error*/
		i2s.ints.rfoe++;
		i2s.regs->RCR &= ~RCR_RFOIE;
	}

	return IRQ_HANDLED;
}

#endif  /* CONFIG_WMT_I2S_DEBUG */

static void i2s_init(void)
{
	/*unsigned int ahb_clk;*/
	int ret;

	DPRINTK("Enter i2s_init \n");
	DPRINTK("i2s_ref = %d \n", i2s.ref);

	if (++i2s.ref > 1)
		return;


	/* Enable I2S controller interface.*/
	*((volatile unsigned long*)(PMC_ADDR+0x250)) |= 0x00000040;
	*((volatile unsigned long*)(GPIO_ADDR+0x200)) |= 0x00000004;
	*((volatile unsigned long*)(GPIO_ADDR+0x200)) &= 0xFFFFFFFD;

//	*((volatile unsigned long*)(PMC_ADDR+0x250)) &= ~0x00010000;
//	printk("i2s_init: (PMC_ADDR+0x250) = 0x%x \n", *((volatile unsigned long*)(PMC_ADDR+0x250)));

	/*
	 hardware initial
	*/
	/*init i2s controller */
	i2s.regs->AUDCTLCR = 0;
	i2s.regs->AUDCTLCR |= (I2S_RRecord_EN|I2S_LRecord_EN);
	i2s.regs->AUDCTLCR |= (I2S_LPLAY_EN|I2S_RPLAY_EN);
	i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT16 << I2S_Rx_SAMPLE_BIT_SHIFT);
	i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT16 << I2S_Tx_SAMPLE_BIT_SHIFT);
	i2s.regs->AUDCTLCR |= (I2S_I2S_MODE << I2S_RX_IF_MODE_SHIFT);
	i2s.regs->AUDCTLCR |= (I2S_I2S_MODE << I2S_TX_IF_MODE_SHIFT);
	i2s.regs->AUDCTLCR |= (I2S_RISINGEDGE_SYNC);
	i2s.regs->AUDCTLCR |= (I2S_AUD_EN);

	/*
	  I2S controller FIFO(s) presetting.
	*/

	i2s.regs->TCR &= (~TCR_TFEN) ;  /* disable Tfifo*/
	i2s.regs->RCR &= (~RCR_RFEN) ;  /* disable Rfifo*/
	*(volatile unsigned char*)(SCC_ADDR+0x80) = 0x01;

	i2s.regs->TSR  = (TCR_TFEIE + TCR_TFAIE + TCR_TFADE + TCR_TFUIE + TCR_TFOIE) ;
	i2s.regs->RSR  = (RCR_RFFIE + RCR_RFAIE + RCR_RFADE + RCR_RFUIE + RCR_RFOIE) ;

	/*
	 request irq
	*/
	ret = request_irq(i2s.irq,
			wmt_i2s_interrupt,
			IRQF_DISABLED,
			"i2s",
			NULL);
	if (ret)
		DPRINTK("%s : unable to request IRQ \n" , __func__);

	/*
	 Enable TX, RX
	*/
	i2s.regs->TCR |= TCR_TFEN | TCR_TFADE | TCR_TFT(8);
	i2s.regs->RCR |= RCR_RFEN | RCR_RFADE | RCR_RFT(8);
}

void i2s_sample_rate(unsigned int rate)
{
	unsigned int txmclk_div;
	unsigned int txbclk_div;
	/*we set 32bit now, it need to get from i2s resigster in the future*/
	unsigned int sample_bit = 0x40;
	unsigned int txclkdiv = 0;

	DPRINTK("%s : set sample rate to %d\n", __func__, rate);
	if (rate >= 92100) {
		s_codec_info.sreg.sample_rate_control_sreg = 96000 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x00060000;
		txmclk_div = 1;
		txbclk_div = 12288000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 92100 && rate >= 68100) {
		s_codec_info.sreg.sample_rate_control_sreg  = 88200 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x000A0000;
		txmclk_div = 1;
		txbclk_div = 11290000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 68100 && rate >= 46050) {
		s_codec_info.sreg.sample_rate_control_sreg  = 48000 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x00060000;
		txmclk_div = 1;
		txbclk_div = 12288000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 46050 && rate >= 38050) {
		s_codec_info.sreg.sample_rate_control_sreg  = 44100 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x000A0000;
		txmclk_div = 1;
		txbclk_div = 11290000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 38050 && rate >= 28000) {
		s_codec_info.sreg.sample_rate_control_sreg  = 32000 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x00060000;
		txmclk_div = 1;
		txbclk_div = 12288000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 28000 && rate >= 23025) {
		s_codec_info.sreg.sample_rate_control_sreg  = 24000 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x00060000;
		txmclk_div = 1;
		txbclk_div = 12288000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 23025 && rate >= 19025) {
		s_codec_info.sreg.sample_rate_control_sreg  = 22050 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x000A0000;
		txmclk_div = 1;
		txbclk_div = 11290000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 19025 && rate >= 13513) {
		s_codec_info.sreg.sample_rate_control_sreg  = 16000 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x00060000;
		txmclk_div = 1;
		txbclk_div = 12288000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else if (rate < 13513 && rate >= 9513) {
		s_codec_info.sreg.sample_rate_control_sreg  = 11025 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x000A0000;
		txmclk_div = 1;
		txbclk_div = 11290000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	} else {
		s_codec_info.sreg.sample_rate_control_sreg  = 8000 ;
		*((unsigned long *)(PMC_ADDR+0x210)) &= 0xFFE0FFFF;
		*((unsigned long *)(PMC_ADDR+0x210)) |= 0x00060000;
		txmclk_div = 1;
		txbclk_div = 12288000/(sample_bit * rate);
		txclkdiv = (txmclk_div << TXMCLK_DIV_SHIFT) | (txbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->TXCLKDIV = txclkdiv ;
	}
}
void i2s_set_tx_channels(unsigned int channel)
{
	if (channel == 0)
		i2s.regs->AUDCTLCR &= ~(I2S_RPLAY_EN);
	else
		i2s.regs->AUDCTLCR |= (I2S_RPLAY_EN|I2S_LPLAY_EN);
}
void i2s_set_rx_channels(unsigned int channel)
{
	if (channel == 0)
		i2s.regs->AUDCTLCR &= ~(I2S_RRecord_EN);
	else
		i2s.regs->AUDCTLCR |= (I2S_RRecord_EN|I2S_LRecord_EN);
}
void i2s_set_tx_resolution(unsigned int resolution)
{
	i2s.regs->AUDCTLCR &= ~(I2S_Tx_SAMPLE_BIT_MASK);

	if (resolution == 8)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT8 << I2S_Tx_SAMPLE_BIT_SHIFT);
	else if (resolution == 13)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT13 << I2S_Tx_SAMPLE_BIT_SHIFT);
	else if (resolution == 14)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT14 << I2S_Tx_SAMPLE_BIT_SHIFT);
	else if (resolution == 20)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT20 << I2S_Tx_SAMPLE_BIT_SHIFT);
	else if (resolution == 24)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT24 << I2S_Tx_SAMPLE_BIT_SHIFT);
	else if (resolution == 32)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT32 << I2S_Tx_SAMPLE_BIT_SHIFT);
	else
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT16 << I2S_Tx_SAMPLE_BIT_SHIFT);
}

void i2s_set_rx_resolution(unsigned int resolution)
{
	i2s.regs->AUDCTLCR &= ~(I2S_Rx_SAMPLE_BIT_MASK);

	if (resolution == 8)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT8 << I2S_Rx_SAMPLE_BIT_SHIFT);
	else if (resolution == 13)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT13 << I2S_Rx_SAMPLE_BIT_SHIFT);
	else if (resolution == 14)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT14 << I2S_Rx_SAMPLE_BIT_SHIFT);
	else if (resolution == 20)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT20 << I2S_Rx_SAMPLE_BIT_SHIFT);
	else if (resolution == 24)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT24 << I2S_Rx_SAMPLE_BIT_SHIFT);
	else if (resolution == 32)
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT32 << I2S_Rx_SAMPLE_BIT_SHIFT);
	else
		i2s.regs->AUDCTLCR |= (I2S_SAMPLE_BIT16 << I2S_Rx_SAMPLE_BIT_SHIFT);
}

void i2s_rx_sample_rate(unsigned int rate)
{
	unsigned int rxmclk_div;
	unsigned int rxbclk_div;
	/*we set 32bit now, it need to get from i2s resigster in the future*/
	unsigned int sample_bit = 0x40;
	unsigned rxclkdiv = 0;
	if (rate >= 92100) {
		rxmclk_div = 1;
		rxbclk_div = 12288000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 92100 && rate >= 68100) {
		rxmclk_div = 1;
		rxbclk_div = 11290000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 68100 && rate >= 46050) {
		rxmclk_div = 1;
		rxbclk_div = 12288000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 46050 && rate >= 38050) {
		rxmclk_div = 1;
		rxbclk_div = 11290000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 38050 && rate >= 28000) {
		rxmclk_div = 1;
		rxbclk_div = 12288000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 28000 && rate >= 23025) {
		rxmclk_div = 1;
		rxbclk_div = 12288000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 23025 && rate >= 19025) {
		rxmclk_div = 1;
		rxbclk_div = 11290000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 19025 && rate >= 13513) {
		rxmclk_div = 1;
		rxbclk_div = 12288000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else if (rate < 13513 && rate >= 9513) {
		rxmclk_div = 1;
		rxbclk_div = 11290000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	} else {
		rxmclk_div = 1;
		rxbclk_div = 12288000/(sample_bit * rate);
		rxclkdiv = (rxmclk_div << TXMCLK_DIV_SHIFT) | (rxbclk_div << TXBCLK_DIV_SHIFT);
		i2s.regs->RXCLKDIV = rxclkdiv ;
	}
}

static void i2s_exit(void)
{
	DPRINTK("Enter i2s_exit \n");

	if (--i2s.ref)
		return;

	DPRINTK("Do i2s_exit \n");

#ifdef CONFIG_WMT_I2S_DEBUG
	free_irq(i2s.irq, NULL);
#endif
	i2s.regs->TCR &= (~TCR_TFEN) ;  /* disable Tfifo*/
	i2s.regs->RCR &= (~RCR_RFEN) ;  /* disable Rfifo*/

	/* Reset counter.*/
	memset(&i2s.ints, 0, sizeof(struct i2s_ints_s));
	return;
}

/*===========================================================================*/
/*  codec_dump_sreg*/
/**/
/*  return:*/
/*===========================================================================*/
void codec_dump_sreg(void)
{
	DPRINTK("\r\n");
	DPRINTK("--- Codec Sregister ---\n");
	DPRINTK("line_left_volume_sreg :         0x%4.4X \r\n", s_codec_info.sreg.line_left_volume_sreg) ;
	DPRINTK("line_right_volume_sreg :        0x%4.4X \r\n", s_codec_info.sreg.line_right_volume_sreg) ;
	DPRINTK("headphone_left_volume_sreg : 0x%4.4X \r\n", s_codec_info.sreg.headphone_left_volume_sreg) ;
	DPRINTK("headphone_right_volume_sreg :0x%4.4X \r\n", s_codec_info.sreg.headphone_right_volume_sreg) ;
	DPRINTK("analog_path_control_sreg :      0x%4.4X \r\n", s_codec_info.sreg.analog_path_control_sreg) ;
	DPRINTK("digital_path_control_sreg :     0x%4.4X \r\n", s_codec_info.sreg.digital_path_control_sreg) ;
	DPRINTK("powerdown_control_sreg :        0x%4.4X \r\n", s_codec_info.sreg.powerdown_control_sreg) ;
	DPRINTK("digital_format_sreg :           0x%4.4X \r\n", s_codec_info.sreg.digital_format_sreg) ;
	DPRINTK("sample_rate_control_sreg :      0x%8.8X \r\n", s_codec_info.sreg.sample_rate_control_sreg) ;
	DPRINTK("digital_activation_sreg :       0x%4.4X \r\n", s_codec_info.sreg.digital_activation_sreg) ;
}

void i2s_dump_reg(void)
{
	DPRINTK("\r\n");
	DPRINTK("--- I2S register ---\n");
	DPRINTK("AUDCTLCR: 0x%.8x \r\n", i2s.regs->AUDCTLCR);
	DPRINTK("AUDDFCR: 0x%.8x \r\n", i2s.regs->AIDDFCR);
	DPRINTK("TXCLKDIV : 0x%.8x \r\n", i2s.regs->TXCLKDIV);
	DPRINTK("TXFRAMECR : 0x%.8x \r\n", i2s.regs->TXFRAMECR);
	DPRINTK("RXCLKDIV : 0x%.8x \r\n", i2s.regs->RXCLKDIV);
	DPRINTK("RXFRAMECR : 0x%.8x \r\n", i2s.regs->RXFRAMECR);
	DPRINTK("TXEQURXCR : 0x%.8x \r\n", i2s.regs->TXEQURXCR);
	DPRINTK("TSR : 0x%.8x \r\n", i2s.regs->TSR);
	DPRINTK("TCR : 0x%.8x \r\n", i2s.regs->TCR);
	DPRINTK("TSR : 0x%.8x \r\n", i2s.regs->TSR);
	DPRINTK("RCR : 0x%.8x \r\n", i2s.regs->RCR);
	DPRINTK("RSR : 0x%.8x \r\n", i2s.regs->RSR);
}

void dump_i2s_codec_sreg(void)
{
	i2s_dump_reg();
	codec_dump_sreg();
}

/*===========================================================================*/
/*  codec_reset*/
/**/
/*  return:*/
/*===========================================================================*/
int codec_reset(void)
{
	DPRINTK("i2s_reset \n");

	/*s_codec_info_write( RESET_ADDR , RESET_CMD) ;*/

	s_codec_info.sreg.line_left_volume_sreg = sreg_reset_value.line_left_volume_sreg;
	s_codec_info.sreg.line_right_volume_sreg = sreg_reset_value.line_right_volume_sreg;
	s_codec_info.sreg.headphone_left_volume_sreg = sreg_reset_value.headphone_left_volume_sreg;
	s_codec_info.sreg.headphone_right_volume_sreg = sreg_reset_value.headphone_right_volume_sreg;
	s_codec_info.sreg.analog_path_control_sreg = sreg_reset_value.analog_path_control_sreg;
	s_codec_info.sreg.digital_path_control_sreg = sreg_reset_value.digital_path_control_sreg;
	s_codec_info.sreg.powerdown_control_sreg = sreg_reset_value.powerdown_control_sreg;
	s_codec_info.sreg.digital_format_sreg = sreg_reset_value.digital_format_sreg;
	s_codec_info.sreg.sample_rate_control_sreg = sreg_reset_value.sample_rate_control_sreg;
	s_codec_info.sreg.digital_activation_sreg = sreg_reset_value.digital_activation_sreg;

	return 0 ;
}

/*===========================================================================*/
/*  codec_set_sample_rate*/
/**/
/*  return:*/
/*===========================================================================*/
int codec_set_sample_rate(unsigned int rate)
{
	int err = 0;
	s_codec_info.sreg.sample_rate_control_sreg  = rate ;
	DPRINTK("set sample rate %d \n", s_codec_info.sreg.sample_rate_control_sreg);
	i2s_sample_rate(rate) ;
	i2s_rx_sample_rate(rate) ;
	vt1602_set_sample_rate(rate);
	codec_write(SAMPLE_RATE_CONTROL_ADDR, rate) ;
	return err ;
}

/*===========================================================================*/
/*  codec_enable*/
/**/
/*  return:*/
/*===========================================================================*/
static int codec_enable(void)
{
	int err = 0;
	DPRINTK("codec_enable \n");
	codec_write(DIGITAL_ACTIVATION_ADDR, CODEC_POWER_ON);
	vt1602_power_up();
	return err;
}

static int codec_disable(void)
{
	int err = 0;
	DPRINTK("codec_disable \n");
	codec_write(DIGITAL_ACTIVATION_ADDR, CODEC_POWER_OFF);
	vt1602_power_down();
	return err;
}

static int codec_read(unsigned short addr, unsigned int *data)
{
	unsigned int *ptr;

	ptr = (unsigned int *)&s_codec_info.sreg;
	*data = ((unsigned int) ptr[addr]);
	return 0;
}

static int codec_write(unsigned short addr, unsigned int data)
{
	int err = 0;
	unsigned int *ptr;
	unsigned int buffer;

	buffer =  data ;

	DPRINTK("codec_write(%d, %d) 0x%x \n", addr, data, buffer);


	if (addr == RESET_ADDR)
		return 0;


	ptr = (unsigned int *)&s_codec_info.sreg;
	(ptr[addr]) = data;

	return err;
}

static int codec_init(void)
{
	DPRINTK("codec init----------\n");
	DPRINTK("s_codec_info.ref = %d\n", s_codec_info.ref);

	if (++s_codec_info.ref > 1)
		return 0;

	if (!init_setting) {
		gpio_enable(GPIO_AUDIO_MUTE,GPIO_OUTPUT_MODE);
                gpio_set_value(GPIO_AUDIO_MUTE, GPIO_OUTPUT_HIGH);
		s_codec_info.sreg = sreg_cfg_value ;
		init_setting++;
		vt1602_init();
		vt1602_set_headphone_volume(HLHV_DEFAULT, 0);
		vt1602_set_line_volume(90, 0);
		s_codec_info.using_sample_rate = s_codec_info.sreg.sample_rate_control_sreg;
		i2s_sample_rate(s_codec_info.sreg.sample_rate_control_sreg);
		i2s_rx_sample_rate(s_codec_info.sreg.sample_rate_control_sreg);
		vt1602_set_sample_rate(s_codec_info.sreg.sample_rate_control_sreg);
		codec->irq = IRQ_I2S;
	}
	if (wmt_i2s_suspend_flag ) {
                vt1602_init();
                vt1602_set_headphone_volume(HLHV_DEFAULT, 0);
                vt1602_set_line_volume(90, 0);
                s_codec_info.using_sample_rate = s_codec_info.sreg.sample_rate_control_sreg;
                i2s_sample_rate(s_codec_info.sreg.sample_rate_control_sreg);
                i2s_rx_sample_rate(s_codec_info.sreg.sample_rate_control_sreg);
                vt1602_set_sample_rate(s_codec_info.sreg.sample_rate_control_sreg);
                codec->irq = IRQ_I2S;
        }

	return 0 ;

}

static void codec_exit(void)
{
	DPRINTK("codec_exit \n");
	DPRINTK("s_codec_info.ref = %d \n", s_codec_info.ref);
	if (--s_codec_info.ref == 0)
		i2s_exit();

}
void i2s_suspend(void)
{
	*((unsigned long *)(PMC_ADDR+0x250)) |= 0x00000040;
#ifdef CONFIG_PM
	/*
	i2s_pm_regs.ICCR = i2s.regs->ICCR;
	i2s_pm_regs.TCR = i2s.regs->TCR;
	i2s_pm_regs.RCR = i2s.regs->RCR;
	*/
#endif
	*((unsigned long *)(PMC_ADDR+0x250)) &= ~(0x00000040);
	wmt_i2s_suspend_flag = 1;

}
void i2s_resume(void)
{
	*((unsigned long *)(PMC_ADDR+0x250)) |= 0x00000040;
//	printk("i2s_resume: (PMC_ADDR+0x250) = 0x%x \n", *((volatile unsigned long*)(PMC_ADDR+0x250)));
//	printk("i2s_resume: (GPIO_ADDR+0x200) = 0x%x \n", *((volatile unsigned long*)(GPIO_ADDR+0x200)));
	gpio_enable(GPIO_AUDIO_MUTE,GPIO_OUTPUT_MODE);
        gpio_set_value(GPIO_AUDIO_MUTE, GPIO_OUTPUT_HIGH);
        if (init_setting || wmt_i2s_suspend_flag) {
		*(volatile unsigned int *)(0xD811000C) |= 0x00000080;/*set to gpio*/
		*(volatile unsigned int *)(0xD811002C) |= 0x00000080;/*set to output*/
		*(volatile unsigned int *)(0xD811004C) &= ~(0x00000080);/*set to 0*/
	}
#ifdef CONFIG_PM
	/*
	i2s.regs->ICCR = i2s_pm_regs.ICCR;
	i2s.regs->TCR = i2s_pm_regs.TCR;
	i2s.regs->RCR = i2s_pm_regs.RCR;
	*/
#endif
	wmt_i2s_suspend_flag = 0;
}
static int codec_set_out_volume(unsigned int vol)
{
	codec_write(HEADPHONE_LEFT_VOLUME_ADDR, vol);
	codec_write(HEADPHONE_RIGHT_VOLUME_ADDR, vol);
	vt1602_set_headphone_volume(vol, 0);
	return 0;
}
static int codec_set_in_volume(unsigned int vol)
{
	codec_write(LINE_LEFT_VOLUME_ADDR, vol);
	codec_write(LINE_RIGHT_VOLUME_ADDR, vol);
	vt1602_set_line_volume(vol, 0);
	return 0;
}
static int codec_select_recsrc(unsigned int src)
{
	vt1602_selectrecord(src);
	return 0;
}
static int codec_loopback_enable(unsigned int en)
{
	vt1602_loopback_enable(en);
	return 0 ;
}

static int codec_mute_en(unsigned int en)
{
	vt1602_mute(en);
	return 0;
}
struct i2s_codec_ops_s codec_ops = {
	.init           = codec_init,
	.exit           = codec_exit,
	.enable         = codec_enable,
	.disable        = codec_disable,
	.read           = codec_read,
	.write          = codec_write,
	.set_sample_rate = codec_set_sample_rate,
	.set_out_volume = codec_set_out_volume,
	.set_in_volume = codec_set_in_volume,
	.select_recsrc = codec_select_recsrc,
	.loopback_enable = codec_loopback_enable,
	.mute_en = codec_mute_en,
};

struct i2s_codec_s *codec_attach(void)
{
	DPRINTK("codec_attach \n");

	if (!codec) {
		codec = kmalloc(sizeof(struct i2s_codec_s), GFP_KERNEL);

		if (codec) {
			memset(codec, 0, sizeof(struct i2s_codec_s));
			codec->ops = &codec_ops;
			codec->mod = 0;
			codec->volume = HLHV_DEFAULT << 8 |
					HLHV_DEFAULT;
		}
	}

	return codec;
}

void codec_detach(struct i2s_codec_s *codec)
{
	DPRINTK("codec_detach \n");
	kfree(codec);
}

EXPORT_SYMBOL(codec_attach);
EXPORT_SYMBOL(codec_detach);
EXPORT_SYMBOL(codec_set_sample_rate);
EXPORT_SYMBOL(dump_i2s_codec_sreg);

