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
#define VT1632_C

/*=== vt1632.c =============================================================
*
* MODULE       : vt1632.c
* AUTHOR       : Sam Shen
* DATE         : 2009/2/26
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Sam Shen, 2009/2/26
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include "vpp.h"
#include <linux/i2c.h>

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  VT1632_XXXX  xxxx    *//*Example*/

// #define DEBUG

#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBGMSG(fmt, args...)   DPRINT("[VT1632] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...)
#endif

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define VT1632_XXXX    1     *//*Example*/
#define VT1632_ADDR 0x10

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx vt1632_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in vt1632.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  vt1632_xxx;        *//*Example*/
static int vt1632_not_ready;

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void vt1632_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/

/*--------------------End of Function Body -----------------------------------*/

/*the define and struct i2c_msg were declared int linux/i2c.h*/
int vt1632_check_plugin(void)
{
	unsigned char buf[1];
	int plugin;
	
	vpp_i2c_read(VT1632_ADDR,0x9,buf,1);
	plugin = (buf[0]&0x4)? 1:0;
	DPRINT("[VT1632] DVI plug%s\n",(plugin)?"in":"out");
	vppif_reg32_write((0xD8110000+0x300), 0x3<<(VPP_VOINT_NO*2),VPP_VOINT_NO*2,(plugin)?2:3);	// GPIO0 3:rising edge, 2:falling edge
	return plugin;
}

int vt1632_init(void)
{
	unsigned char buf[16];

	vt1632_not_ready = 1;
	vpp_i2c_read(VT1632_ADDR, 0x0, buf, 2);
	if( (buf[0] != 0x06) || (buf[1] != 0x11) ){	// check vender id
		DPRINT("vt1632_init 0x%x 0x%x\n",buf[0],buf[1]);
		return -1;
	}
	vt1632_not_ready = 0;

	buf[0x0] = 0x37;
	buf[0x1] = 0x20;
	vpp_i2c_write(VT1632_ADDR,0x8,buf,2);
	return 0;
}

int vt1632_set_mode(vpp_datawidht_t dwidth)
{
	unsigned char buf[1];

	if( vt1632_not_ready )
		return -1;

	DBGMSG("vt1632_set_mode(%d)\n",(dwidth)?24:12);

	vpp_i2c_read(VT1632_ADDR,0x8,buf,1);
	if( dwidth == VPP_DATAWIDHT_12 ){
		buf[0] &= ~BIT2;
		buf[0] |= BIT3;
	}
	else {
		buf[0] |= BIT2;
		buf[0] &= ~BIT3;		
	}
	vpp_i2c_write(VT1632_ADDR,0x8,buf,1);
	return 0;
}

int vt1632_set_power_down(int enable)
{
	unsigned char buf[1];

	if( vt1632_not_ready )
		return -1;

	DBGMSG("vt1632_set_power_down(%d)\n",enable);

	vpp_i2c_read(VT1632_ADDR,0x8,buf,1);
	if( enable ){
		buf[0] &= ~BIT0;
	}
	else {
		buf[0] |= BIT0;
	}
	vpp_i2c_write(VT1632_ADDR,0x8,buf,1);
	return 0;
}

#undef VT1632_C

