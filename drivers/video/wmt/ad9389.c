/**************************************************************		
Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.	
This program is free software: you can redistribute it and/or modify it under the terms 	
of the GNU General Public License as published by the Free Software Foundation, either
 	version 2 of the License, or (at your option) any later version.
	
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the GNU General Public License for more details. You should have received
 a copy of the GNU General Public License along with this program.  If not, see
 <http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.

--*/
#define AD9389_C

/*=== ad9389.c =============================================================
*
* MODULE       : ad9389.c
* AUTHOR       : Sam Shen
* DATE         : 2009/2/5
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Sam Shen, 2009/2/5
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "vout.h"
#include <linux/i2c.h>

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  AD9389_XXXX  xxxx    *//*Example*/

// #define DEBUG

#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBGMSG(fmt, args...)   DPRINT("[AD9389] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...)
#endif

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define AD9389_XXXX    1     *//*Example*/
#define AD9389_ADDR 	0x72
#define AD9389_REG_NUM	203

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx ad9389_xxx_t; *//*Example*/
typedef enum {
	AD9389_VOMODE_HDMI_RGB_12,
	AD9389_VOMODE_HDMI_RGB_24,
	AD9389_VOMODE_HDMI_YUV444_12,
	AD9389_VOMODE_HDMI_YUV444_24,
	AD9389_VOMODE_HDMI_YUV422_24,
	AD9389_VOMODE_MAX
} ad9389_output_mode_t;

/*----------EXPORTED PRIVATE VARIABLES are defined in ad9389.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  ad9389_xxx;        *//*Example*/
const unsigned char ad9389_reg_yuv444_12[AD9389_REG_NUM]={
	3,0,24,0,33,33,192,0,0,0			//0~9
	,9,14,60,24,1,19,37,55,0,0			//10~19
	,0,10,67,2,6,98,4,168,0,0			//20~29
	,28,132,28,191,4,168,30,112,2,30	//30~39
	,0,0,4,168,8,18,27,172,0,0			//40~49
	,0,0,0,0,0,0,0,0,0,128				//50~59
	,0,4,16,0,0,16,224,126,248,32		//60~69
	,0,0,0,0,0,0,0,0,0,0				//70~79
	,0,0,0,0,0,0,0,0,0,0				//80~89
	,0,0,0,0,0,0,0,0,0,0				//90~99
	,0,0,0,0,0,0,0,0,0,0				//100~109
	,0,0,0,0,0,0,0,0,0,0				//110~119
	,0,0,0,0,0,0,0,0,0,0				//120~129
	,0,0,0,0,0,0,0,0,0,0				//130~139
	,0,0,0,0,0,0,0,0,192,0				//140~149
	,32,4,3,2,0,24,56,97,0,0			//150~159	//9e(158) and 9f(159) don't care
	,0,0,135,135,8,192,0,0,0,0			//160~169	//a0(158) don't care
	,0,0,0,0,64,22,147,1,213,56			//170~179
	,218,221,113,240,224,0,96,255,187,165		//180~189
	,144,131,105,220,148,122,0,112,30,0,36,3,	//190~202
};

static vdo_color_fmt ad9389_colfmt;
static vpp_datawidht_t ad9389_dwidth;

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void ad9389_xxx(void); *//*Example*/
int ad9389_set_mode(unsigned int *option);

/*----------------------- Function Body --------------------------------------*/
int ad9389_check_plugin(void)
{
	unsigned char buf[1];
	int plugin;
	unsigned int option[2];
	
	vpp_i2c_read(AD9389_ADDR,0x42,buf,1);
	plugin = (buf[0] & BIT6)? 1:0;
	printk("[AD9389] HDMI plug%s\n",(plugin)?"in":"out");

	buf[0] = 0x80;
	vpp_i2c_write(AD9389_ADDR,0x94,buf,1);	// enable INT
	buf[0] = 0x80;
	vpp_i2c_write(AD9389_ADDR,0x96,buf,1);	// clear HPD flag
	vppif_reg32_write((0xD8110000+0x300), 0x3<<(VPP_VOINT_NO*2),VPP_VOINT_NO*2,2);	// GPIO0 3:rising edge, 2:falling edge

	option[0] = ad9389_colfmt;
	option[1] = ad9389_dwidth;
	if( plugin ){
		do {
			buf[0] = 0x10;
			vpp_i2c_write(AD9389_ADDR,0x41,buf,1);
			vpp_i2c_read(AD9389_ADDR,0x41,buf,1);
			if((buf[0] & BIT6)==0)	// Power up 
				break;

			vpp_i2c_read(AD9389_ADDR,0x42,buf,1);
			plugin = (buf[0] & BIT6)? 1:0;
			if( plugin == 0 ) 
				return 0;			
		} while(1);
		
		ad9389_set_mode(&option[0]);
	}
	return plugin;
}

void ad9389_set_power_down(int enable)
{
	char reg;

	vpp_i2c_read(AD9389_ADDR,0x41,&reg,1);
	if( enable ){
		reg |= 0x40;
	}
	else {
		reg &= ~0x40;
	}
	vpp_i2c_write(AD9389_ADDR,0x41,&reg,1);
}

int ad9389_set_mode(unsigned int *option)
{
	vdo_color_fmt colfmt;
	vpp_datawidht_t dwidth;
	ad9389_output_mode_t mode;
	unsigned char wr_buf[AD9389_REG_NUM];

	colfmt = option[0];
	dwidth = option[1];

	ad9389_colfmt = colfmt;
	ad9389_dwidth = dwidth;
	switch(colfmt){
		case VDO_COL_FMT_ARGB:
			mode = (dwidth == VPP_DATAWIDHT_12)? AD9389_VOMODE_HDMI_RGB_12:AD9389_VOMODE_HDMI_RGB_24;
			break;
		case VDO_COL_FMT_YUV444:
			mode = (dwidth == VPP_DATAWIDHT_12)? AD9389_VOMODE_HDMI_YUV444_12:AD9389_VOMODE_HDMI_YUV444_24;
			break;
		case VDO_COL_FMT_YUV422H:
			mode = (dwidth == VPP_DATAWIDHT_12)? AD9389_VOMODE_MAX:AD9389_VOMODE_HDMI_YUV422_24;
			break;
		default:
			mode = AD9389_VOMODE_MAX;
			break;
	}

	memcpy(wr_buf,ad9389_reg_yuv444_12,AD9389_REG_NUM);
	switch(mode){
		case AD9389_VOMODE_HDMI_RGB_12:
			printk("[AD9389] HDMI RGB 12bit mode\n");
			wr_buf[0x15] = 0x0A;
			wr_buf[0x16] = 0x02;
			wr_buf[0x45] = 0x0;
//			wr_buf[0x97] = 0x4;
//			wr_buf[0x98] = 0x3;
//			wr_buf[0xA5] = 0xC0;
			wr_buf[0xAF] = 0x16;
//			wr_buf[0xBA] = 0x60;
			break;
		case AD9389_VOMODE_HDMI_YUV444_12:
			printk("[AD9389] HDMI YUV444 12bit mode\n");
			wr_buf[0x15] = 0x0A;
			wr_buf[0x16] = 0x43;
			wr_buf[0x45] = 0x20;
//			wr_buf[0x97] = 0x4;
//			wr_buf[0x98] = 0x3;
//			wr_buf[0xA5] = 0xC0;
			wr_buf[0xAF] = 0x16;
//			wr_buf[0xBA] = 0x60;
			break;
		case AD9389_VOMODE_HDMI_RGB_24:
			printk("[AD9389] HDMI RGB 24bit mode\n");			
			wr_buf[0x15] = 0x0;
			wr_buf[0x16] = 0x02;
			wr_buf[0x45] = 0x0;
//			wr_buf[0x97] = 0x4;
//			wr_buf[0x98] = 0x3;
//			wr_buf[0xA5] = 0xC0;
			wr_buf[0xAF] = 0x16;
//			wr_buf[0xBA] = 0x60;
			break;
		case AD9389_VOMODE_HDMI_YUV444_24:
			printk("[AD9389] HDMI YUV444 24bit mode\n");
			wr_buf[0x15] = 0x0;
			wr_buf[0x16] = 0x43;
			wr_buf[0x45] = 0x20;
//			wr_buf[0x97] = 0x4;
//			wr_buf[0x98] = 0x3;
//			wr_buf[0xA5] = 0xC0;
			wr_buf[0xAF] = 0x16;
//			wr_buf[0xBA] = 0x60;
			break;
		case AD9389_VOMODE_HDMI_YUV422_24:
			printk("[AD9389] HDMI YUV422 24bit mode\n");
			wr_buf[0x15] = 0x02;
			wr_buf[0x16] = 0xCB;
			wr_buf[0x45] = 0x10;
//			wr_buf[0x97] = 0x4;
//			wr_buf[0x98] = 0x3;
//			wr_buf[0xA5] = 0xC0;
			wr_buf[0xAF] = 0x16;
//			wr_buf[0xBA] = 0x60;
			break;
		default:
			printk("*E* invalid mode\n");
			return -1;
	}

	wr_buf[0x94] = 0x80;
	wr_buf[0x96] = 0x80;

#if 1
	vpp_i2c_write(AD9389_ADDR, 0x0, wr_buf, AD9389_REG_NUM);
#else
	ad9389_write_reg(0x15,&wr_buf[0x15],1);
	ad9389_write_reg(0x16,&wr_buf[0x16],1);
	ad9389_write_reg(0x45,&wr_buf[0x45],1);
	ad9389_write_reg(0x97,&wr_buf[0x97],1);
	ad9389_write_reg(0x98,&wr_buf[0x98],1);
	ad9389_write_reg(0xA5,&wr_buf[0xA5],1);
	ad9389_write_reg(0xAF,&wr_buf[0xAF],1);
	ad9389_write_reg(0xBA,&wr_buf[0xBA],1);
#endif
	return 0;
}	

int ad9389_config(vout_info_t *info)
{
	return 0;
}

int ad9389_init(void *ptr)
{
	vout_dev_ops_t *ops;
	unsigned char buf[2];

	buf[0] = 0xff;
	vpp_i2c_read(0x72, 0xcf, buf, 1);
	if( buf[0] != 0x70 ){
		return -1;
	}
	ops = (vout_dev_ops_t *) ptr;
	ops->init = ad9389_init;
	ops->set_mode = ad9389_set_mode;
	ops->set_power_down = ad9389_set_power_down;
	ops->check_plugin = ad9389_check_plugin;
	ops->config = ad9389_config;
	return 0;
}
/*--------------------End of Function Body -----------------------------------*/
#undef AD9389_C
