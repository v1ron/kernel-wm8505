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

/*
How this device is works?

WM8505 have I2S registers, which can be accessed directly by processor, but this registers does not provide
access to CODEC's configuration registers (AC97 is provide by special COMMAND register).
However, I2S sound devices usually have additional interface, which is attached to peripheral bus.
ES8328 in my WM8505 is attached to the I2C bus at address 0x8 (see es8328 datasheet).
Audio data is transmitted over 5-wire I2S interface.

*/

#include "wmt-8328-mixer.h"

#ifndef _ES8328_H_
#define _ES8328_H_

/* REGISTERS MAP */
#define es8328_control_1          0x0
/*
default    0000 0110
bit
SCPReset   7            0 - normal (default)
                        1 - reset control port register to default
LRCM       6            0 - ALRCK disabled when both ADC disabled; DLRCK disabled when both DAC disabled (default) 
                        1 - ALRCK and DLRCK disabled when all ADC and DAC disabled
DACMCLK    5            0 - when SameFs=1, ADCMCLK is the chip master clock source (default)
                        1 - when SameFs=1, DACMCLK is the chip master clock source
SameFs     4            0 - ADC Fs differs from DAC Fs (default)
                        1 - ADC Fs is the same as DAC Fs
SeqEn      3            0 - Internal power up/down sequence disabled (default)
                        1 - Enabled
EnRef      2            0 - disable reference
                        1 - Enable reference (default)
VMIDSEL    1:0          00 Vmid disabled
                        01 50k (resistance) divider enabled
                        10 500k divider enabled (default)
                        11 5k divider enabled
*/
#define es8328_control_2          0x1
/*
default 0001 1100
TSDEN          7        0 - thermal shutdown disabled
                        1 - enabled
PdnOC          6        0 over current shutdown disabled
                        1 - enabled
LPVcmMod       5        0 - normal
                        1 - low power
LPVrefBuf      4        0 - normal
                        1 - low power
Pdnlbiasgen    2        0 - normal
                        1 - lbiasgen power down
VrefLo         1        0 - normal
                        1 - low power
PdnVrefbuf     0        0 - normal
                        1 - power down
*/
#define es8328_power_man          0x2
/*
default 1100 0011
adc_DigPDN  7           0 - normal
                        1 - resets ADC DEM, filter and serial data port
dac_DigPDN  6           0 - normal
                        1 - resets DAC DSM, DEM filter and serial data port
adc_stm_rst 5           0 - normal
                        1 - reset ADC state machine to power down state
dac_stm_rst 4           0 - normal
                        1 - reset DAC state machine to power down state
ADCDLL_PDN  3           0 - normal
                        1 - ADC_DLL power down, stop ADC clock
DACDLL_PDN  2           0 - normal
                        1 - DAC DLL power down, stop DAC clock
adcVref_PDN 1           0 - ADC analog reference power up
                        1 - power down
dacVref_PDN 0           0 DAC analog reference power up
                        1 - power down
*/
#define es8328_adc_power_man      0x3
/*
default 1111 1100
PdnAINL        7        0 - normal
                        1 - left analog input power down
PdnAINR        6        0 - normal
                        1 - left analog input power down
PdnADCL        5        0 - left ADC power up
                        1 - power down
PdnADCR        4        0 - right ADC power up
                        1 - power down
PdnMICB        3        0 - microphone bias power on
                        1 - power down (high impedance output)
PdnADCBiasgen  2        0 - normal
                        1 - power down
flashLP        1        0 - normal
                        1 - flash ADC low power
int1LP         0        0 - normal
                        1 - int1 low power
*/
#define es8320_dac_power_man      0x4
/*
default 1100 0000
PdnDACL        7        0 - left DAC power up
                        1 - power down
PdnDACR        6        0 - right DAC power up
                        1 - power down
LOUT1          5        0 - LOUT1 disabled
                        1 - enabled
ROUT1          4        0 - disabled
                        1 - enabled
LOUT2          3        0 - disabled
                        1 - enabled
ROUT2          2        0 - disabled
                        1 - enabled
MONO           1        0 - MOUT disabled
                        1 - enabled
OUT3           0        0 - disabled
                        1 - enabled
*/
#define es8320_low_power_1        0x5
/*
normal power by default (non-sleeping mode)
*/
#define es8320_low_power_2        0x6
/* normal power by default */
#define es8320_analog_voltage_man 0x7
/*
VSEL           8:0      01111100 (default)
*/
#define es8320_master_mode_ctrl   0x8
/*
default 1000 0000
MSC            7        0 - slave serial port mode
                        1 - master serial port mode
MCLKDIV2       6        0 - MCLK normal
                        1 - MCLK divide by 2
BCLK_INV       5        0 - normal
                        1 - BCLK inverted
BCLKDIV        4:0      00000 master mode BCLK generated automatically based on the clock table
                        Others - MCLK/N, N=1~31
*/
#define es8320_adc_ctrl_1         0x9
#define es8320_adc_ctrl_2         0x0a
#define es8320_adc_ctrl_3         0x0b
#define es8320_adc_ctrl_4         0x0c
#define es8320_adc_ctrl_5         0x0d
#define es8320_adc_ctrl_6         0x0e
#define es8320_adc_ctrl_7         0x0f
#define es8320_adc_ctrl_8         0x10
#define es8320_adc_ctrl_9         0x11
#define es8320_adc_ctrl_10        0x12
#define es8320_adc_ctrl_11        0x13
#define es8320_adc_ctrl_12        0x14
#define es8320_adc_ctrl_13        0x15
#define es8320_adc_ctrl_14        0x16
#define es8320_dac_ctrl_1         0x17
#define es8320_dac_ctrl_2         0x18
#define es8320_dac_ctrl_3         0x19
#define es8320_dac_ctrl_4         0x1a
#define es8320_dac_ctrl_5         0x1b
#define es8320_dac_ctrl_6         0x1c
#define es8320_dac_ctrl_7         0x1d
#define es8320_dac_ctrl_8         0x1e
#define es8320_dac_ctrl_9         0x1f
#define es8320_dac_ctrl_10        0x20
#define es8320_dac_ctrl_11        0x21
#define es8320_dac_ctrl_12        0x22
#define es8320_dac_ctrl_13        0x23
#define es8320_dac_ctrl_14        0x24
#define es8320_dac_ctrl_15        0x25
#define es8320_dac_ctrl_16        0x26
#define es8320_dac_ctrl_17        0x27
#define es8320_dac_ctrl_18        0x28
#define es8320_dac_ctrl_19        0x29
#define es8320_dac_ctrl_20        0x2a
#define es8320_dac_ctrl_21        0x2b
#define es8320_dac_ctrl_22        0x2c
#define es8320_dac_ctrl_23        0x2d
/*
default 0000 0000
bits
7              0 - ROUT2 no inversion
               1 - ROUT2 inverted
6:5            OUT3 select
               00 - VREF
               01 - ROUT1 (volume controlled by ROUT1VOL)
               10 - MONOOUT
               11 - right mixer output (no volume controlled through ROUT1VOL)
4              0 - 1.5k VREF to analog output resistance
               1 - 40k VREF to analog output resistance
3              0 - headphone switch disabled
               1 - headphone switch enabled
2              0 - HPDETECT high=headphone
               1 - HPDETECT high=speaker
1              0 - MOUT no inversion
               1 - MOUT inverted
*/
#define es8320_dac_ctrl_24        0x2e
/*
default 0000 0000
bits 5:0        LOUT1  Left Channel Out 1 Volume
000000          -30dB
...
100001          3dB
*/
#define es8320_dac_ctrl_25        0x2f
/*
default 0000 0000
bits 5:0        ROUT1  Right Channel Out 1 Volume
000000          -30dB
...
100001          3dB
*/
#define es8320_dac_ctrl_26        0x30
/*
default 0000 0000
bits 5:0        LOUT2  Left Channel Out 2 Volume
000000          -30dB
...
100001          3dB
*/
#define es8320_dac_ctrl_27        0x31
/*
default 0000 0000
bits 5:0        ROUT2  Right Channel Out 2 Volume
000000          -30dB
...
100001          3dB
*/
#define es8320_dac_ctrl_28        0x32
/*
default 0000 0000
bits 5:0        MONOOUTVOL Mono channel Out Volume
000000          -30dB
...
100001          3dB
*/
#define es8320_dac_ctrl_29        0x33
/*
default 0000 0000
bits 0:7 reserved
*/
#define es8320_dac_ctrl_30        0x34
/*
default 0000 0000
 bits 0:7 reserved
*/

/* I2C device address is 0x10 */
// note that this chip supports bit-selector on special CE pin.
// By default, bit is connected to ground and chip address will be 0010000 (binary)
// if CE is connected to Vdd, then address will be 0010001.
// ACK bit IS NOT used and not supported by chip! Program must verify register by performing read afrer write.
// Transaction sequence:
//--------- Write:
// Byte 1:
// bit 0:6 - chip address
// bit 7 - Read/Write (0=write)
// Byte 2:
// Register address
// Byte3:
// Data to write
//--------- Read:
// Transaction 1:
// Byte 1:
// bit 0:6 - chip address
// bit 7 - R\W(0=write)
// Byte 2:
// Register address
// Transaction 2:
// Byte 1:
// bit 0:6 - chip address
// bit 7 - R/W(1=read)
// Byte 2:
// Data returned
#define es8328_i2c_addr1           0b0010000
#define es8328_i2c_addr2           0b0010001

#define     ES8328_MCLKTOLRCLK_RATIO_128     0b00000
#define     ES8328_MCLKTOLRCLK_RATIO_192     0b00001
#define     ES8328_MCLKTOLRCLK_RATIO_256     0b00010
#define     ES8328_MCLKTOLRCLK_RATIO_384     0b00011
#define     ES8328_MCLKTOLRCLK_RATIO_512     0b00100
#define     ES8328_MCLKTOLRCLK_RATIO_576     0b00101
#define     ES8328_MCLKTOLRCLK_RATIO_768     0b00110
#define     ES8328_MCLKTOLRCLK_RATIO_1024    0b00111
#define     ES8328_MCLKTOLRCLK_RATIO_1152    0b01000
#define     ES8328_MCLKTOLRCLK_RATIO_1408    0b01001
#define     ES8328_MCLKTOLRCLK_RATIO_1536    0b01010
#define     ES8328_MCLKTOLRCLK_RATIO_2112    0b01011
#define     ES8328_MCLKTOLRCLK_RATIO_2304    0b01100
#define     ES8328_MCLKTOLRCLK_RATIO_125     0b10000
#define     ES8328_MCLKTOLRCLK_RATIO_136     0b10001
#define     ES8328_MCLKTOLRCLK_RATIO_250     0b10010
#define     ES8328_MCLKTOLRCLK_RATIO_272     0b10011
#define     ES8328_MCLKTOLRCLK_RATIO_375     0b10100
#define     ES8328_MCLKTOLRCLK_RATIO_500     0b10101
#define     ES8328_MCLKTOLRCLK_RATIO_544     0b10110
#define     ES8328_MCLKTOLRCLK_RATIO_750     0b10111
#define     ES8328_MCLKTOLRCLK_RATIO_1000    0b11000
#define     ES8328_MCLKTOLRCLK_RATIO_1088    0b11010
#define     ES8328_MCLKTOLRCLK_RATIO_1496    0b11010
#define     ES8328_MCLKTOLRCLK_RATIO_1500    0b11011

#define	    ES8328_SINGLESPEED_MODE		0
#define     ES8328_DOUBLESPEED_MODE		1

#define     ES8328_MAKE_DACCONTROL2(ratio,mode)		((ratio & 0b11111) | ((mode & 1) << 5))

//---------------------------------------------------------
// Structures
//---------------------------------------------------------

/*
 * wait queue head is using for waiting CODEC ready, CODEC read done,
 * and CODEC write done interrups.
 */
static DECLARE_WAIT_QUEUE_HEAD(codec_wq);

//---------------------------------------------------------
// Import
//---------------------------------------------------------

extern int wmt_i2c_xfer_if(struct i2c_msg *msg);

#endif
