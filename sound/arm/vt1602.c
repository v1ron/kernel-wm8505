/*++
	sound/arm/vt1602.c	

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

#include "vt1602.h"
#include <linux/slab.h>
#include <linux/config.h>


#define VT1602_ADDR 0x1A

int vt1602_write_reg(unsigned int index , unsigned short value) {
	char* buf ;
	struct i2c_msg msg[1];
	buf = kmalloc(2*sizeof(char*),GFP_KERNEL);
	buf[0] = (index<<1)|((value&0x0100)>>8);
	buf[1] =  (value & 0x00ff);
	msg[0].addr = VT1602_ADDR;
	msg[0].flags = 0 ;  
	msg[0].flags &= ~(I2C_M_RD);
	msg[0].len = 2;
	msg[0].buf = buf;
	wmt_i2c_xfer_if(msg);
	kfree(buf);
	return 0;
}
int vt1602_reset () {
	vt1602_write_reg( VT1602_Reset, 0) ;
	return 0;
}

/*===========================================================================
*  vt1602_set_record_mode
*
*  return: 0: success, -1:fail
* 
*===========================================================================*/
int vt1602_set_record_mode(void) 
{
	unsigned int i = 0;
	vt1602_write_reg(VT1602_ADCL_Signal_Path,0x020);
	vt1602_write_reg(VT1602_ADCR_Signal_Path,0x020);
	vt1602_write_reg(VT1602_Pwr_Mgmt_2,0x1E0);
	vt1602_write_reg(VT1602_Pwr_Mgmt_1,0x0FE);

	for (i = 0;i < 50;++i)
		;
	return 0 ;	
}

/*===========================================================================
*  vt1602_set_audio_if
*
*  return: 0: success, -1:fail
* 
*==========================================================================*/
int vt1602_set_audio_if(unsigned int fmt){
	switch (fmt) {
	case AUDIOIF_FMT_RIGHTJUSTIFIED:
		vt1602_write_reg(VT1602_Audio_Port, 0x000);
		break;
	case AUDIOIF_FMT_LEFTJUSTIFIED:
		vt1602_write_reg(VT1602_Audio_Port, 0x008);
		break;
	case AUDIOIF_FMT_PCM:
		vt1602_write_reg(VT1602_Audio_Port, 0x083);
		break;
	case AUDIOIF_FMT_I2S:
	default:
		vt1602_write_reg(VT1602_Audio_Port, 0x002);
		break;
	}
	return 0;
}


/*===========================================================================
*  vt1602_set_sample_rate
*
*  return: 
*==========================================================================*/
int vt1602_set_sample_rate(unsigned int rate )
{   
	if (rate >=96000)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x01c);
	else if (rate >= 88200)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x03c);
	else if (rate >= 48000)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x000);
	else if (rate >= 44100)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x020);
	else if (rate >= 32000)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x018);
	else if (rate >= 24000)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x038);
	else if (rate >= 22050)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x034);
	else if (rate >= 16000)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x014);
	else if (rate >= 11025)
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x030);
	else
        	vt1602_write_reg(VT1602_Sample_Rate_Select, 0x00C);
	return 0;
}

/*===========================================================================
*  vt1602_mute
*  mute :
*         1:enable
*         0:diaable
*
*  return: 
*==========================================================================*/

int vt1602_mute(unsigned int mute)
{
    if(mute == 1)
        /* SOFTMUTE=1*/
        vt1602_write_reg(VT1602_ADC_DAC_Control, 0x008);
    else
        /* SOFTMUTE=0*/
        vt1602_write_reg(VT1602_ADC_DAC_Control, 0x000);

    return 0 ;

}
int vt1602_set_headphone_volume(unsigned int vol,unsigned int mute){
	unsigned short head_vol ;

	if (vol >= 100)
		vol = 100 ;       

	head_vol = (( VT1602_HLHV_MAX - VT1602_HLHV_MIN) * vol) / 100 + VT1602_HLHV_MIN ;

	if(mute == 1)
		/* SOFTMUTE=1*/
		vt1602_write_reg(VT1602_ADC_DAC_Control, 0x008);
	else
		/* SOFTMUTE=0*/
		vt1602_write_reg(VT1602_ADC_DAC_Control, 0x000);
	    
	vt1602_write_reg(VT1602_LOUTA_Volume, (0x000 | 0x000 | head_vol));

	vt1602_write_reg(VT1602_ROUTA_Volume, (0x100 | 0x000 | head_vol)); 

	return 0 ;    
}

int vt1602_set_l_headphone_volume(unsigned int vol,unsigned int mute){
	unsigned short head_vol ;
	if ( vol >= 100 )
		vol = 100 ;
	head_vol = ( ( VT1602_HLHV_MAX - VT1602_HLHV_MIN ) * vol ) / 100 + VT1602_HLHV_MIN ;
	if(mute == 1)
		/* SOFTMUTE=1*/
		vt1602_write_reg(VT1602_ADC_DAC_Control, 0x008);
	else
		/* SOFTMUTE=0*/
		vt1602_write_reg(VT1602_ADC_DAC_Control, 0x000);

	vt1602_write_reg(VT1602_LOUTA_Volume, (0x000 | 0x000 | head_vol));

	return 0 ;
}

int vt1602_set_r_headphone_volume(unsigned int vol,unsigned int mute)
{
	unsigned short head_vol ;

	if (vol >= 100)
		vol = 100 ;

	head_vol = ((VT1602_HLHV_MAX - VT1602_HLHV_MIN) * vol) / 100 + VT1602_HLHV_MIN ;

	if(mute == 1)
		// SOFTMUTE=1
		vt1602_write_reg(VT1602_ADC_DAC_Control, 0x008);
	else
		// SOFTMUTE=0
		vt1602_write_reg(VT1602_ADC_DAC_Control, 0x000);


	vt1602_write_reg(VT1602_ROUTA_Volume, (0x100 | 0x080 | head_vol));

	return 0 ;
}



int vt1602_init(void)
{
	unsigned int i = 0;
	vt1602_reset() ;

	vt1602_write_reg( VT1602_ADC_DAC_Control, 0x000 );
	vt1602_write_reg(VT1602_Pwr_Mgmt_1,0x0FE);
	vt1602_write_reg(VT1602_Pwr_Mgmt_2,0x1E0);
	vt1602_set_audio_if( AUDIOIF_FMT_I2S ); 
	//vt1602_write_reg(VT1602_ADD_Control_2,0x060);
	vt1602_write_reg(VT1602_ADD_Control_2,0x1E0);
	    
	//vt1602_write_reg(VT1602_Left_Out_Mix_1,0x101);//change to linb
	//vt1602_write_reg(VT1602_Left_Out_Mix_2,0x100);
	//vt1602_write_reg(VT1602_Right_Out_Mix_1,0x101);
	//vt1602_write_reg(VT1602_Right_Out_Mix_2,0x100);
	    
	vt1602_write_reg(VT1602_Left_Out_Mix_1,0x100);
	vt1602_write_reg(VT1602_Left_Out_Mix_2,0x000);
	vt1602_write_reg(VT1602_Right_Out_Mix_1,0x000);
	vt1602_write_reg(VT1602_Right_Out_Mix_2,0x100);
	//vt1602_write_reg(VT1602_Pwr_Mgmt_1,0x0FE);
	//vt1602_write_reg(VT1602_Pwr_Mgmt_2,0x1E0);
	vt1602_write_reg(VT1602_ADD_Control_3,0x040);
	//vt1602_write_reg(VT1602_ALC1,0x1FF);

	for (i = 0; i < 1000; ++i)
		;

	vt1602_write_reg( VT1602_Audio_Port, 0x002 );

	return 0 ;
}
int vt1602_set_line_out()
{
	vt1602_write_reg(VT1602_Pwr_Mgmt_1,0x0FE);
	vt1602_write_reg(VT1602_Pwr_Mgmt_2,0x1E0);
	return 0;
}
/*===========================================================================
*  vt1602_set_line_volume
*
*  return: 
*===========================================================================*/
int vt1602_set_line_volume( unsigned int vol, unsigned int mute )
{
	unsigned short line_vol;
	    
	if (vol >= 100)
		vol = 100;
	if (mute == 1)
		line_vol = 0;
	else
		line_vol = ((( VT1602_ADCVOL_MAX - VT1602_ADCVOL_MIN) * vol) / 100) + VT1602_ADCVOL_MIN;

	vt1602_write_reg(VT1602_Left_ADC_Volume, (0x000 | line_vol));
	vt1602_write_reg(VT1602_Left_PGA_Volume,0x150);
	    
	vt1602_write_reg(VT1602_Right_ADC_Volume, (0x100 | line_vol));
	vt1602_write_reg(VT1602_Right_PGA_Volume,0x150);

	return 0 ;
}
int vt1602_power_down(void)
{
	vt1602_write_reg(VT1602_Pwr_Mgmt_1,0x000);
	vt1602_write_reg(VT1602_Pwr_Mgmt_2,0x000);
	return 0;
}

int vt1602_power_up(void)
{
	vt1602_write_reg(VT1602_Pwr_Mgmt_1,0x0FE);
	vt1602_write_reg(VT1602_Pwr_Mgmt_2,0x1E0);
	return 0;
}

int vt1602_selectrecord(unsigned int recsrc)
{
	/*0:LIN1 1:LIN2 2:LIN3*/
	if (recsrc == 1) {
		vt1602_write_reg(VT1602_ADCL_Signal_Path,0x060);
		vt1602_write_reg(VT1602_ADCR_Signal_Path,0x060);
	} else if (recsrc == 2) {
		vt1602_write_reg(VT1602_ADCL_Signal_Path,0x0A0);
		vt1602_write_reg(VT1602_ADCR_Signal_Path,0x0A0);
	} else {
		vt1602_write_reg(VT1602_ADCL_Signal_Path,0x010);
		vt1602_write_reg(VT1602_ADCR_Signal_Path,0x010);
	} 
	return 0; 
}
int vt1602_loopback_enable(unsigned int en)
{
	if (en == 1) {
		vt1602_write_reg(VT1602_Left_Out_Mix_1,0x193);
		vt1602_write_reg(VT1602_Left_Out_Mix_2,0x010);
		vt1602_write_reg(VT1602_Right_Out_Mix_1,0x013);
		vt1602_write_reg(VT1602_Right_Out_Mix_2,0x190);
	} else {
        	vt1602_write_reg(VT1602_Left_Out_Mix_1,0x100);
        	vt1602_write_reg(VT1602_Left_Out_Mix_2,0x000);
        	vt1602_write_reg(VT1602_Right_Out_Mix_1,0x000);
        	vt1602_write_reg(VT1602_Right_Out_Mix_2,0x100);
	}
	return 0;
}

