/*++
	sound/soc/wmt/wmt-alsa-1602.h

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

--*/
#ifndef __ALSA_I2S_CODEC
#define __ALSA_I2S_CODEC
#include <linux/interrupt.h>
#define CODEC_INITED            (1 << 0)


#define LLRS_ENABLE                     0x100
#define LLIM_ENABLE                     0x080
#define LLIM_DISABLE                    0x000
#define LLIV_MASK                       0x01F
#define LLIV_MAX                        0x01F   /* 12*/
#define LLIV_DEFAULT                    0x017   /* 0 DB*/
#define LLIV_MIN                        0x000   /* -34.5*/

#define LRLS_ENABLE                     0x100
#define LRIM_ENABLE                     0x080
#define LRIM_DISABLE                    0x000
#define LRIV_MASK                       0x01F
#define LRIV_MAX                        0x01F   /* 12*/
#define LRIV_DEFAULT                    0x017   /* 0 DB*/
#define LRIV_MIN                        0x000   /* -34.5*/

#define LINE_VOLUME_DEFAULT            (LLRS_ENABLE | LLIM_DISABLE | LLIV_DEFAULT)

#define HLRS_ENABLE                     0x100
#define HLZC_ENABLE                     0x080
#define HLHV_MASK                       0x07F
/*#define HLHV_MAX                        0x07F    // 6*/
#define HLHV_DEFAULT                    0x079
/*#define HLHV_MIN                        0x000    // 0x030 -73*/
#define HLHV_MUTE                       0x030

#define HRLS_ENABLE                     0x100
#define HRZC_ENABLE                     0x080
#define HRHV_MASK                       0x07F
/*#define HRHV_MAX                        0x07F    // 6*/
#define HRHV_DEFAULT                    0x099
/*#define HRHV_MIN                        0x000    // 0x030 -73*/
#define HRHV_MUTE                       0x030

#define HEADPHONE_VOLUME_DEFAULT        (HLRS_ENABLE | HLZC_ENABLE | HLHV_DEFAULT)

#define SIDETONE_MASK                   0x1E0
#define SIDETONE_OFF                    0x000
#define SIDETONE_MINUS_0_DB             0x120
#define SIDETONE_MINUS_6_DB             0x020
#define SIDETONE_MINUS_9_DB             0x060
#define SIDETONE_MINUS_12_DB            0x0A0
#define SIDETONE_MINUS_18_DB            0x0E0

#define DAC_ENABLE                      0x010
#define BYPASS_ENABLE                   0x008
#define BYPASS_DISABLE                  0x000
#define LINE_MODE                       0x000
#define MIC_MODE                        0x004
#define MIC_MUTE_ENABLE                 0x002
#define MIC_MUTE_DISABLE                0x000
#define MIC_BOOST_ENABLE                0x001
#define MIC_BOOST_DISABLE               0x000

#define RECORD_MASK                     0x007

#define MIC_DEFAULT                     (LINE_MODE | MIC_MUTE_ENABLE | MIC_BOOST_DISABLE)
#define ANALOG_PATH_DEFAULT             (SIDETONE_OFF | DAC_ENABLE | BYPASS_DISABLE | MIC_DEFAULT)

#define DAC_SOFT_MUTE_ENABLE            0x008
#define DAC_SOFT_MUTE_DISABLE           0x000
#define DE_EMPHASIS_DISABLE             0x000
#define DE_EMPHASIS_32K                 0x002
#define DE_EMPHASIS_44K                 0x004
#define DE_EMPHASIS_48K                 0x006
#define ADCHP_ENABLE                    0x000
#define ADCHP_DISABLE                   0x001

#define DIGITAL_PATH_DEFAULT            (DAC_SOFT_MUTE_DISABLE | DE_EMPHASIS_48K | ADCHP_DISABLE)

#define POWER_LINE_ENABLE               0x000
#define POWER_LINE_DISABLE              0x001
#define POWER_MIC_ENABLE                0x000
#define POWER_MIC_DISABLE               0x002
#define POWER_ADC_ENABLE                0x000
#define POWER_ADC_DISABLE               0x004
#define POWER_DAC_ENABLE                0x000
#define POWER_DAC_DISABLE               0x008
#define POWER_OUT_ENABLE                0x000
#define POWER_OUT_DISABLE               0x010
#define POWER_OSC_ENABLE                0x000
#define POWER_OSC_DISABLE               0x020
#define POWER_CLK_ENABLE                0x000
#define POWER_CLK_DISABLE               0x040
#define POWER_DEV_ENABLE                0x000
#define POWER_DEV_DISABLE               0x080

#define POWER_ALL_ENABLE                0x000
#define POWER_ALL_DISABLE               0x1FF

#define MASTER_MODE                     0x040
#define SLAVE_MODE                      0x000
#define LR_SWAP_ENABLE                  0x020
#define LR_SWAP_DISABLE                 0x000

/**/
#define LRP_LRCIN_HIGH                  0x000
#define LRP_LRCIN_LOW                   0x010
#define LRP_DSP_1st_BCLK                0x000   /* only Data format is DSP*/
#define LRP_DSP_2nd_BCLK                0x010

#define IWL_16_BITS                     0x000
#define IWL_20_BITS                     0x004
#define IWL_24_BITS                     0x008
#define IWL_32_BITS                     0x00C

#define FOR_DSP                         0x003
#define FOR_I2S                         0x002
#define FOR_MSB_LEFT                    0x001
#define FOR_MSM_RIGHT                   0x000

#define DIGITAL_FORMAT_DEFAULT (SLAVE_MODE | LR_SWAP_DISABLE | LRP_LRCIN_HIGH | IWL_16_BITS | FOR_I2S)

#define CLKOUT_HALF_ENABLE              0x080
#define CLKIN_HALF_ENABLE               0x040
#define SAMPLE_MASK                     0x03C
#define USB_250_FS                      0x000
#define USB_272_FS                      0x002
#define NORMAL_256_FS                   0x000
#define NORMAL_384_FS                   0x002
#define NORMAL_MODE                     0x000
#define USB_MODE                        0x001

#define SAMPLE_RATE_DEFAULT             (0x00D) /* KHZ_8_0*/
#define DIGITAL_ACT                     0x001
#define RESET_CMD                       0x000


int vt1602_configure(void);
void i2s_set_dac_samplerate(unsigned int rate);
void i2s_set_adc_samplerate(unsigned int rate);
int vt1602_codec_write(unsigned short addr, unsigned int data);
int codec_set_sample_rate(unsigned int rate);
int vt1602_enable(void);
int vt1602_disable(void);
int vt1602_get_default_samplerate(void);
#endif
