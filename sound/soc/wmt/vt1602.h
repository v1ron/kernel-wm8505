/*++
	sound/arm/vt1602.h	

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
		The code was inherit from vt8500 
		2009/02/03 First Version

--*/
#ifndef _VT1602_H_
#define _VT1602_H_
#include <mach/hardware.h>
#include <linux/i2c.h>

#define VT1602_IS_MASTER 0

#define VT1602_Left_PGA_Volume 0x00
#define VT1602_Right_PGA_Volume 0x01
#define VT1602_LOUTA_Volume 0x02
#define VT1602_ROUTA_Volume 0x03
#define VT1602_ADC_DAC_Control 0x05
#define VT1602_Audio_Port 0x07
#define VT1602_Sample_Rate_Select 0x08
#define VT1602_Left_DAC_Gain 0x0A
#define VT1602_Right_DAC_Gain 0x0B
#define VT1602_Reset 0x0F
#define VT1602_ALC1 0x11
#define VT1602_ALC2 0x12
#define VT1602_ALC3 0x13
#define VT1602_Noise_Gate 0x14
#define VT1602_Left_ADC_Volume 0x15
#define VT1602_Right_ADC_Volume 0x16
#define VT1602_ADD_Control_1 0x17
#define VT1602_ADD_Control_2 0x18
#define VT1602_Pwr_Mgmt_1 0x19
#define VT1602_Pwr_Mgmt_2 0x1A
#define VT1602_ADD_Control_3 0x1B
#define VT1602_ADC_Input_Mode 0x1F
#define VT1602_ADCL_Signal_Path 0x20
#define VT1602_ADCR_Signal_Path 0x21
#define VT1602_Left_Out_Mix_1 0x22
#define VT1602_Left_Out_Mix_2 0x23
#define VT1602_Right_Out_Mix_1 0x24
#define VT1602_Right_Out_Mix_2 0x25
#define VT1602_Mono_Out_Mix_1 0x26
#define VT1602_Mono_Out_Mix_2 0x27
#define VT1602_LOUTB_Volume 0x28
#define VT1602_ROUTB_Volume 0x29
#define VT1602_OUT_MONO_Volume 0x2A

#define VT1602_CACHE_REGNUM  0x2a
#define VT1602_I2C_DEVICE_ADDR      0x1A
#define VT1602_CS_I2C_DEVICE_ADDR      0x1B
#define VT1602_ADC_I2C_DEVICE_ADDR     0x1C
#define VT1602_ADC_CS_I2C_DEVICE_ADDR 0x1E
#define VT1602_MAX_REG_INDEX    0x2A
#define VT1602_HLHV_MAX			0x07F    // 6
#define VT1602_HLHV_MIN			0x03F    // -57
#define VT1602_HRHV_MAX			0x07F    // 6
#define VT1602_HRHV_MIN			0x03F    // -57
#define VT1602_ADCVOL_MAX		0x0FF
#define VT1602_ADCVOL_MIN		0x000
#define HLHV_MAX                        0x07F    // 6
#define HLHV_MIN                        0x040    // -67
#define HRHV_MAX                        0x07F    // 6
#define HRHV_MIN                        0x040    // -67

#define AUDIOIF_FMT_RIGHTJUSTIFIED	0x00
#define AUDIOIF_FMT_LEFTJUSTIFIED	0x01
#define AUDIOIF_FMT_I2S			0x02
#define AUDIOIF_FMT_PCM			0x04

// Function Prototypes
extern int vt1602_init(void);
extern int vt1602_shutdown(void);
extern int vt1602_set_headphone_volume( unsigned int vol , unsigned int mute );
extern int vt1602_set_line_volume( unsigned int vol, unsigned int mute );
extern int vt1602_set_loopback_mode( void );
extern int vt1602_set_record_mode( void );
extern int vt1602_set_sample_rate(unsigned int rate );
extern int vt1602_set_audio_if( unsigned int fmt );
extern int vt1602_reset(void);
extern int vt1602_write_reg(unsigned int index , unsigned short value);
extern int vt1602_set_l_headphone_volume( unsigned int vol , unsigned int mute );
extern int vt1602_set_r_headphone_volume( unsigned int vol , unsigned int mute );
extern int vt1602_mute(unsigned int mute);
extern int vt1602_set_line_out(void);
extern int vt1602_power_down(void);
extern int vt1602_power_up(void);
extern int vt1602_selectrecord(unsigned int recsrc);
extern int vt1602_loopback_enable(unsigned int en);

extern void wmt_i2c_xfer_if(struct i2c_msg *msg);

extern int vt1602_write_reg(unsigned int index , u16 value);
extern u16 vt1602_read_reg(unsigned int index );
#endif /* _VT1602_H_*/

