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
	
	Authors: V1ron (Roman I. Volkov) (http://v1ron.ru), Russian software developer.
	
	Please refer to wmt-alsa.h file for more information.

	(c) 2011: Original version for some Chinese kernel.
	(c) 2015: Pushing this shit to the Projectgus kernel. Leaving
		my original comments as is to preserve this important
		historical artifact.
*/

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include "wmt-alsa-8328.h"

#include <linux/config.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/i2s_alsa.h>
#include <linux/i2c.h>
#include <mach/wmt_i2c.h>
#include "es8328.h"

// global variables
static unsigned char chip_i2c_address;
static unsigned char daccontrol;

//
int snd_chip_read_reg_8(unsigned char *buffer);
int snd_chip_write_reg_8(unsigned char index, unsigned char value);
/* ----------------- Helpers ------------*/

int snd_chip_start_play_and_record(void)
{
	int status;
	// set chip to slave mode
	status = snd_chip_write_reg_8(0x08, 0x0);
	// Power down DEM and STM
	status |= snd_chip_write_reg_8(0x02, 0xf3);
	// set chip to Play&record mode
	status |= snd_chip_write_reg_8(0x00, 0x05);
	// Power Up analog and lbias
	status |= snd_chip_write_reg_8(0x01, 0x40);
	// Power up ADC / Analog input / Micbias for record
	status |= snd_chip_write_reg_8(0x03, 0x00);
	// power up DAC and Enable LOUT/ROUT
	status |= snd_chip_write_reg_8(0x04, 0x3c/*0b00111111*/);
	// Select analog input channel for ADC
	//status |= snd_chip_write_reg_8(0x0a, 0x50);
	// Select analog input PGA gain for ADC
	//status |= snd_chip_write_reg_8(0x09, 0x88);
	// Set ADC SFI 16-bit
	status |= snd_chip_write_reg_8(0x0c, 0b00001100);
	// set MCLK\LRCLK ratio
	status |= snd_chip_write_reg_8(0x0d, daccontrol);
	// Set ADC digital volume
	//status |= snd_chip_write_reg_8(0x10, 0x00);
	//status |= snd_chip_write_reg_8(0x11, 0x00);
	// Select ALC for ADC record
	status |= snd_chip_write_reg_8(0x12, 0x16);
	// Set DAC SFI (audio format). no swap, normal polarity,
	// 16-bit, I2S.
	status |= snd_chip_write_reg_8(0x17, 0b00011000);
	// Select MCLK\LRCLK ratio for DAC
	status |= snd_chip_write_reg_8(0x18, daccontrol);
	// Set DAC Digital volume
	//status |= snd_chip_write_reg_8(0x1a, 0x00);
	//status |= snd_chip_write_reg_8(0x1b, 0x00);
	// Set Mixer to DAC output
	status |= snd_chip_write_reg_8(0x26, 0x00);
	status |= snd_chip_write_reg_8(0x29, 0x38);
	status |= snd_chip_write_reg_8(0x2a, 0xB8);
	// Set LOut\ROut vol 0db
	//status |= snd_chip_write_reg_8(0x2e, 0x1e);
	//status |= snd_chip_write_reg_8(0x2f, 0x1e);
	//status |= snd_chip_write_reg_8(0x30, 0x1e);
	//status |= snd_chip_write_reg_8(0x31, 0x1e);
	// power up DEM and STM
	status |= snd_chip_write_reg_8(0x02, 0x00);
	return status;
}

/* ----------------- Exporting functions ------------*/

// Function that reads 8-bit value from the specified address in internal chip memory.
// buffer: pointer to user variable, that contains register offset to read. After returning,
// buffer variable will contain register value.
// Return values: error code. 
int snd_chip_read_reg_8(unsigned char *buffer)
{
	int status;
	struct i2c_msg msg[1];
	// transaction 1: Set read address
	msg[0].addr = chip_i2c_address;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buffer;
	status = wmt_i2c_xfer_if(msg);
	if (status < 0) return status;
	// transaction 2: read data at this address
	msg[0].flags = I2C_M_RD;
	return wmt_i2c_xfer_if(msg);
}

// Function that writes 8-bit value to chip register, over I2C protocol, with verification
int snd_chip_write_reg_8(unsigned char index, unsigned char value) 
{
	int status;
	struct i2c_msg msg[1];
	unsigned char buf[4];
        // transaction 1 - Write data
	buf[0] = index;
	buf[1] = value;
	msg[0].addr = chip_i2c_address;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (char*)&buf;
	status = wmt_i2c_xfer_if(msg);
	if (status < 0) return status;
	// transactions 2 & 3 read data
	// buf[0] already set
	status = snd_chip_read_reg_8(&buf[0]);
	// verify
	//printk("ALSA: reg val = %d, read = %d\n",value,buf[0]);
	if (buf[0] != value) return -EIO; // I/O error
	return status;
}

// performs test write at specified chip address
int i2c_test_addr(unsigned char chip_addr)
{
	unsigned char buf[8];
	struct i2c_msg msg[1];
	buf[0] = 0;
	msg[0].addr = chip_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = (char*)&buf;
	return wmt_i2c_xfer_if(msg);
}

/* this function prepares sound chip for playback and returns clock ratios and divisors for i2s bus,
	i2s layer should set this parameters */
int snd_chip_set_rate(int rate, int *sysclk, int *sysclkdiv, int *mclkdiv, int *bclkdiv, int *lrclkdiv)
{
	if (rate == 8000)
	{
		// SYSCLK = 16,3835 / 4 = 4,09587
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x00050000;
		// MCLK = 4,09587 / 1 = 4,09587; BCLK = 4,09587 / 8 = 0,51198
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x0001000c;
		// DLRCLK = 0,51198 / 64 = 0,008
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 512
		*sysclk = WMT_I2S_SYSCLK_16_3835_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_4;
		*mclkdiv = 1;
		*bclkdiv = 8;
		*lrclkdiv = 64;
		// set mclk\lrclk ratio
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_512, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 11025)
	{
		// SYSCLK = 22,5789 / 4 = 5,64472MHz
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x000A0000;
		// MCLK = 5,64472MHz / 2 = 2,82236MHz; BCLK = 5,64472MHz / 8 = 0,70559MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x00010004;
		// DLRCLK = 0,70559MHz / 64 = 0,01102MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 256
		*sysclk = WMT_I2S_SYSCLK_22_5789_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_4;
		*mclkdiv = 2;
		*bclkdiv = 8;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_256, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 16000)
	{
		// SYSCLK = 16,3835 / 2 = 8,19175
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x00050000;
		// MCLK = 8,19175 / 1 = 8,19175; BCLK = 8,19175 / 8 = 1,02397
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x0001000c;
		// DLRCLK = 1,02397 / 64 = 0,016
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 512
		*sysclk = WMT_I2S_SYSCLK_16_3835_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_2;
		*mclkdiv = 1;
		*bclkdiv = 8;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_512, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 22050)
	{
		// SYSCLK = 22,5789 / 4 = 5,64472MHz
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x000A0000;
		// MCLK = 5,64472MHz / 1 = 5,64472MHz; BCLK = 5,64472MHz / 4 = 1,41118MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x00010004;
		// DLRCLK = 1,41118MHz / 64 = 0,02205MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 256
		*sysclk = WMT_I2S_SYSCLK_22_5789_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_4;
		*mclkdiv = 1;
		*bclkdiv = 4;
		*lrclkdiv = 64;	
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_256, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 32000)
	{
		// SYSCLK = 16,3835 / 1 = 16,3835
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x00050000;
		// MCLK = 16,3835 / 1 = 16,3835; BCLK = 16,3835 / 8 = 2,04794
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x0001000c;
		// DLRCLK = 2,04794 / 64 = 0,032
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 512
		*sysclk = WMT_I2S_SYSCLK_16_3835_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_1;
		*mclkdiv = 1;
		*bclkdiv = 8;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_512, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 44100)
	{
		// SYSCLK = 22,5789 / 2 = 11,28945MHz
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x000A0000;
		// MCLK = 11,28945MHz / 1 = 11,28945MHz; BCLK = 11,28945MHz / 4 = 2,82236MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x00010004;
		// DLRCLK = 2,82236MHz / 64 = 0,0441MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 256
		*sysclk = WMT_I2S_SYSCLK_22_5789_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_2;
		*mclkdiv = 1;
		*bclkdiv = 4;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_256, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 48000)
	{
		// SYSCLK = 24,5755 / 1 = 24,5755
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x00050000;
		// MCLK = 24,5755 / 1 = 24,5755; BCLK = 24,5755 / 8 = 3,07194
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x0001000c;
		// DLRCLK = 3,07194 / 64 = 0,048
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 512
		*sysclk = WMT_I2S_SYSCLK_24_5755_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_1;
		*mclkdiv = 1;
		*bclkdiv = 8;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_512, 
			ES8328_SINGLESPEED_MODE);
	} else if (rate == 64000)
	{
		// SYSCLK = 16,3835 / 1 = 16,3835
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x00050000;
		// MCLK = 16,3835 / 1 = 16,3835; BCLK = 16,3835 / 4 = 4,09587
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x0001000c;
		// DLRCLK = 4,09587 / 64 = 0,064
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 256
		*sysclk = WMT_I2S_SYSCLK_16_3835_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_1;
		*mclkdiv = 1;
		*bclkdiv = 4;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_256, 
			ES8328_DOUBLESPEED_MODE);
	} else if (rate == 88200)
	{
		// SYSCLK = 22,5789 / 1 = 22,5789MHz
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x000A0000;
		// MCLK = 22,5789MHz / 1 = 22,5789MHz; BCLK = 22,5789MHz / 4 = 5,64472MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x00010004;
		// DLRCLK = 5,64472MHz / 64 = 0,0882MHz
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 256
		*sysclk = WMT_I2S_SYSCLK_22_5789_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_1;
		*mclkdiv = 1;
		*bclkdiv = 4;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_256, 
			ES8328_DOUBLESPEED_MODE);
	} else if (rate == 96000)
	{
		// SYSCLK = 24,5755 / 1 = 24,5755
		//*((volatile unsigned long *)(PMC_ADDR+0x210)) = 0x00050000;
		// MCLK = 24,5755 / 1 = 24,5755; BCLK = 24,5755 / 4 = 6,14387
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXCLKDIV)) = 0x0001000c;
		// DLRCLK = 6,14387 / 64 = 0,096
		//*((volatile unsigned long *)(I2S_BASE_ADDR + I2S_TXFRAMECR)) = 0x00000040;
		// ratios: BCLK\DLRCLK = 64, MCLK\DLRCLK = 256
		*sysclk = WMT_I2S_SYSCLK_24_5755_MHZ;
		*sysclkdiv = WMT_I2S_SYSCLK_DIV_1;
		*mclkdiv = 1;
		*bclkdiv = 4;
		*lrclkdiv = 64;
		daccontrol = ES8328_MAKE_DACCONTROL2(ES8328_MCLKTOLRCLK_RATIO_256, 
			ES8328_DOUBLESPEED_MODE);
	} else
	{
		return -EINVAL;
	}
	return 0;
}

/* this function prepares sound chip for capture and returns clock ratios and divisors for i2s bus,
	i2s layer should set this parameters */
/* note: in 5-pin i2s mode, duplex transmission available only on same sample rate, and we simply set
	same rate for capture and playback, i2s layer will just hadle one of two directions. */
int snd_chip_set_adc_rate(int rate, int *sysclk, int *sysclkdiv, int *mclkdiv, int *bclkdiv, int *lrclkdiv)
{
	int status;
	status = snd_chip_set_rate(rate, sysclk, sysclkdiv, mclkdiv, bclkdiv, lrclkdiv);
	if (status >= 0)
	{
		status = snd_chip_start_play_and_record();
	}
	return status;
}

int snd_chip_set_dac_rate(int rate, int *sysclk, int *sysclkdiv, int *mclkdiv, int *bclkdiv, int *lrclkdiv)
{
	int status;
	status = snd_chip_set_rate(rate, sysclk, sysclkdiv, mclkdiv, bclkdiv, lrclkdiv);
	if (status >= 0)
	{
		status = snd_chip_start_play_and_record();
	}
	return status;
}

/* Returns chip name */
char* snd_chip_get_name(void)
{
	return "ES8328";
}

int snd_chip_suspend(void)
{
	return 0;
}

int snd_chip_resume(void)
{
	return 0;
}

// chip initialization. Top-level code may call init\exit dynamically.
int  snd_chip_startup(struct snd_card *card)
{
	// check which address on i2c bus is used by chip.
	// if both 10000 and 10001 is not used, then this system does not have es8328...
	// this chip supports special address selector, which depends on shorting two pins on PCB.
	// send dummy packet to the chip and checking status...
	chip_i2c_address = 0;
	if (i2c_test_addr(es8328_i2c_addr1) > 0)
	{
		chip_i2c_address = es8328_i2c_addr1;
		snd_wmt_mixer(NULL, card);
		printk("es8328: snd_chip_startup(): chip found at address 1\n");
		return 0;
	}
	if (i2c_test_addr(es8328_i2c_addr2) > 0)
	{
		chip_i2c_address = es8328_i2c_addr2;
		snd_wmt_mixer(NULL, card);
		printk("es8328: snd_chip_startup(): chip found at address 2\n");
		return 0;
	}

	printk("es8328: snd_chip_startup(): -ENODEV\n");
	return -ENODEV;
}

// chip finalization.
void snd_chip_shutdown(void)
{
}

EXPORT_SYMBOL_GPL(snd_chip_get_name);
EXPORT_SYMBOL_GPL(snd_chip_startup);
EXPORT_SYMBOL_GPL(snd_chip_shutdown);
EXPORT_SYMBOL_GPL(snd_chip_set_dac_rate);
EXPORT_SYMBOL_GPL(snd_chip_set_adc_rate);
EXPORT_SYMBOL_GPL(snd_chip_suspend);
EXPORT_SYMBOL_GPL(snd_chip_resume);
