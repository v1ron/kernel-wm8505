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
#include "userdef.h"
#include "bus.h"
#define ERR_BUS_NO_ERROR    0

#ifdef CTRL_BY_GPIO
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <linux/delay.h>
#define   SPEED                                        50
//#define OUT_CLOCK   0x08
//#define OUT_DATA    0x80
//#define IN_CLOCK    0x08
//#define IN_DATA     0x80
//#define IN_EVENT    0x40

//unsigned char PortData    = 0x40;
//unsigned char PortControl = 0x00;
//unsigned int  LPT         = 0x378;

#define SCL_BIT		BIT7	// GPIO 1
#define SDA_BIT		BIT9	// GPIO 2

#define delay_time 30

spinlock_t swi2c_irqlock = SPIN_LOCK_UNLOCKED;

#define     ERR_I2C_NO_ERROR            0x00000000ul
#define     ERR_I2C_NULL_HANDLE         0x00000100ul
#define     ERR_I2C_DONT_SUPPORT        0x00000200ul

void swi2c_delay(unsigned int time)
{
    while(time--);
}

//void Init_Clock(void)
//{
//  pGpioReg->OC_SDTV |= 0x04;	// Set VDIN2(DVBT_SCL) as output pin
//  pGpioReg->OD_SDTV |= 0x04;	// Set VDIN2(DVBT_SCL) output 1
//}

void swi2c_SetSDAInput(void)
{
//	REG16_VAL(__GPIO_BASE + 0x480) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x480) &= ~SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x4C0) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x040) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x080) &= ~(SDA_BIT);
}


void swi2c_SetSDAOutput(void)
{
	REG16_VAL(__GPIO_BASE + 0x480) &= ~(SDA_BIT);
	REG16_VAL(__GPIO_BASE + 0x4C0) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x040) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x080) |= SDA_BIT;
}

bool swi2c_GetSDA(void) // bit
{
	if(REG16_VAL(__GPIO_BASE + 0x000) & SDA_BIT)
		return 1;
	return 0;
}

bool swi2c_GetSCL(void) // bit
{
	if(REG16_VAL(__GPIO_BASE + 0x000) & SCL_BIT)
		return 1;
	return 0;
}

//==================================================================//
void    swi2c_SetSDAHigh(void)
{
//        PortData &= ~OUT_DATA;     // 0x40 & ~0x80(0x7F)
//	outportb(LPT,PortData);    // out 0x40
	REG16_VAL(__GPIO_BASE + 0x480) &= ~(SDA_BIT);
	REG16_VAL(__GPIO_BASE + 0x4C0) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x040) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x080) |= SDA_BIT;
	REG16_VAL(__GPIO_BASE + 0x0C0) |= SDA_BIT;
}
void    swi2c_SetSDALow(void)
{
//	PortData |= OUT_DATA;      // 0x40 | 0x80
//	outportb(LPT,PortData);    // out 0xC0
	REG32_VAL(__GPIO_BASE + 0x480) &= ~(SDA_BIT);
	REG32_VAL(__GPIO_BASE + 0x4C0) |= SDA_BIT;
	REG32_VAL(__GPIO_BASE + 0x040) |= SDA_BIT;
	REG32_VAL(__GPIO_BASE + 0x080) |= SDA_BIT;
	REG32_VAL(__GPIO_BASE + 0x0C0) &= ~(SDA_BIT);
}

//void    SetPowHigh(void)
//{
//	PortData |= 0x02;
//	outportb(LPT,PortData);
//}
//void    SetPowLow(void)
//{
//	PortData &= ~0x02;
//	outportb(LPT,PortData);
//}
//void    SetResHigh(void)
//{
//	PortData |= 0x01;
//	outportb(LPT,PortData);
//}
//void    SetResLow(void)
//{
//	PortData &= ~0x01;
//	outportb(LPT,PortData);
//}
void    swi2c_SetSCLHigh(void)
{
//	PortControl |= OUT_CLOCK;      // 0x00 | 0x08
//	outportb(LPT+2,PortControl);   // out 0x08
	REG32_VAL(__GPIO_BASE + 0x480) &= ~(SCL_BIT);
	REG32_VAL(__GPIO_BASE + 0x4C0) |= SCL_BIT;
	REG32_VAL(__GPIO_BASE + 0x040) |= SCL_BIT;
	REG32_VAL(__GPIO_BASE + 0x080) |= SCL_BIT;
	REG32_VAL(__GPIO_BASE + 0x0C0) |= SCL_BIT;
}
void    swi2c_SetSCLLow(void)
{
//	PortControl &= ~OUT_CLOCK;     // 0x00 & 0x08(0xF7)
//	outportb(LPT+2,PortControl);   // out 0x00

	REG32_VAL(__GPIO_BASE + 0x480) &= ~(SCL_BIT);
	REG32_VAL(__GPIO_BASE + 0x4C0) |= SCL_BIT;
	REG32_VAL(__GPIO_BASE + 0x040) |= SCL_BIT;
	REG32_VAL(__GPIO_BASE + 0x080) |= SCL_BIT;
	REG32_VAL(__GPIO_BASE + 0x0C0) &= ~(SCL_BIT);
}
//==================================================================//
tDWORD swi2c_SetDataHigh(void)
{
        unsigned char iWait=1;

	swi2c_SetSDAHigh();
	do
	{
	    if(swi2c_GetSDA())    return 0; // success
	}while(iWait++<SPEED);

	return 1; // fail
}
tDWORD swi2c_SetDataLow(void)
{
	unsigned char iWait=1;

	swi2c_SetSDALow();
	do
	{
	    if(!swi2c_GetSDA())    return 0; // success
	}while(iWait++<SPEED);

	return 1; // fail
}

tDWORD swi2c_SetClockHigh(void)
{
	unsigned char iWait=1;

	udelay(10);

	swi2c_SetSCLHigh();
	do
	{
	    if(swi2c_GetSCL())    return 0; // success
	}while(iWait++<SPEED);

	return 1; // fail
}

tDWORD swi2c_SetClockLow(void)
{
	unsigned char iWait=1;

	udelay(10);

    swi2c_SetSCLLow();
	do
	{
	    if(!swi2c_GetSCL())    return 0; // success
	}while(iWait++<SPEED);

	return 1; // fail
}

//==================================================================//
tDWORD swi2c_StartI2C(void)
{
	if(swi2c_SetDataHigh())    return 1;
	if(swi2c_SetClockHigh())   return 1;
	if(swi2c_SetDataLow())     return 1;
	swi2c_delay(0);
//	swi2c_delay(delay_time);
	if(swi2c_SetClockLow())    return 1;
	return 0; // success
}
tDWORD swi2c_StopI2C(void)
{
	if(swi2c_SetDataLow())     return 1;
	if(swi2c_SetClockLow())    return 1;
	if(swi2c_SetClockHigh())   return 1;
	swi2c_delay(0);
        if(swi2c_SetDataHigh())    return 1;
	return 0; // success
}
//==================================================================//
tDWORD swi2c_WriteAck(unsigned char byte)
{
	int bit;


        for(bit=7; bit>=0; bit--)
        {
            if(byte & 0x80)
            {
                if(swi2c_SetDataHigh())    return 1;
            }
            else
            {
                if(swi2c_SetDataLow())     return 1;
            }
	    byte<<=1;
	    if(swi2c_SetClockHigh())       return 1;
	    if(swi2c_SetClockLow())        return 1;
	}
//	if(SetDataLow())             return 1;
//	////////////////////    Ack    ///////////////////
//	delay(0);
//	SetSDAHigh();
	swi2c_SetSDAInput();
	swi2c_delay(5);
	if(swi2c_SetClockHigh())
	{
		swi2c_SetSDAOutput();
        	return 1;
	}
	if(swi2c_GetSDA())
	{
	    if(swi2c_SetClockLow())
	    {
	    		swi2c_SetSDAOutput();
	    		return 1;
	    
	    }else{
	    		swi2c_SetSDAOutput();
			return 0;
	    }
	}
	if(swi2c_SetClockLow())
	{
		swi2c_SetSDAOutput();
		return 1;
	}
	swi2c_SetSDAOutput();
	return 0;
}
tDWORD swi2c_ReadAck(unsigned char *byte)
{
//	int bit;
	unsigned char ucloop;
	unsigned char ucI2cData = 0;

	swi2c_SetSDAInput();

	for(ucloop = 0; ucloop < 8; ucloop++)
	{
//		SCL_OUT_1(_HIGH);
		if(swi2c_SetClockHigh())
		{
			swi2c_SetSDAOutput();
			return 1;
		}
		ucI2cData <<= 1;
		swi2c_delay(14);
//		ucI2cData |= SDA_IN_1();
		ucI2cData |= swi2c_GetSDA();
//		SCL_OUT_1(_LOW);
		if(swi2c_SetClockLow())
		{
			swi2c_SetSDAOutput();
			return 1;
		}
	}

	*byte = ucI2cData;

//	for(bit=7; bit>=0; bit--)
//	{
//	    (*byte)<<=1;
//	    if(SetClockHigh())       return 1;
//	    if(GetSDA())             (*byte)++;
//	    delay(0);
//	    if(SetClockLow())        return 1;
//	    delay(0);
//	}

	swi2c_delay(0);
	if(swi2c_SetDataHigh())            return 1;
	if(swi2c_SetClockHigh())           return 1;
	swi2c_delay(0);
	if(swi2c_SetClockLow())            return 1;
	swi2c_delay(0);
	return 0;
}

//**********************************************************************
// This function is a template.
// Users have to implement their own BusWrite function
//**********************************************************************
tDWORD  swi2c_BusWrite(
    tHANDLE     handle,
    tBYTE       ucI2cAddr,                  // api_mpeg_only
    tBYTE *     ucpBuffer,
    tDWORD      dwCount,
    tDWORD *    dwpWriteBytes
)
{
    tDWORD  dwError = ERR_BUS_NO_ERROR;
	unsigned int flags;
	unsigned char i;
	
	spin_lock_irqsave(&swi2c_irqlock, flags);

	dwError |= swi2c_StartI2C();
    dwError |= swi2c_WriteAck(ucI2cAddr);

    for (i = 0; i < dwCount; i++)
    {
         dwError |= swi2c_WriteAck(ucpBuffer[i]);
    }
    dwError |= swi2c_StopI2C();

	spin_unlock_irqrestore(&swi2c_irqlock, flags);	
    return (dwError);
}

tDWORD swi2c_BusRead(
    tHANDLE     handle,
    tBYTE       ucI2cAddr,                  // api_mpeg_only
    tBYTE *     ucpBuffer,
    tDWORD      dwCount,
    tDWORD *    dwpReadBytes
)
{
    tDWORD   dwError = ERR_BUS_NO_ERROR;
	unsigned int flags;
//	int i;

	spin_lock_irqsave(&swi2c_irqlock, flags);

	dwError |= swi2c_StartI2C();
    dwError |= swi2c_WriteAck(ucI2cAddr | 0x01);
//    for(i=0;i<dwCount;i++)
		dwError |= swi2c_ReadAck(ucpBuffer);
    dwError |= swi2c_StopI2C();

	spin_unlock_irqrestore(&swi2c_irqlock, flags);	
    return (dwError);
}
#else // CTRL_BY_I2S
tDWORD  BusWrite(
    tHANDLE     handle,
    tBYTE       ucI2cAddr,                  // api_mpeg_only
    tBYTE *     ucpBuffer,
    tDWORD      dwCount,
    tDWORD *    dwpWriteBytes
)
{
	struct i2c_msg_s msgs[1] ;

	msgs[0].addr  = ucI2cAddr ; 	/*slave address*/
	msgs[0].flags = I2C_M_WR ;
	msgs[0].len = dwCount;
	msgs[0].buf   = ucpBuffer ;
	return i2c_transfer(msgs, 1);
}

tDWORD BusRead(
    tHANDLE     handle,
    tBYTE       ucI2cAddr,                  // api_mpeg_only
    tBYTE *     ucpBuffer,
    tDWORD      dwCount,
    tDWORD *    dwpReadBytes
)
{
	struct i2c_msg_s msgs[1] ;
	int ret;

	msgs[0].addr  = ucI2cAddr ; /*slave address*/
	msgs[0].flags = I2C_M_RD ;
	msgs[0].len   = dwCount;/*how many bytes do you want to transfer*/
	msgs[0].buf   = ucpBuffer ;

	ret = i2c_transfer(msgs, 1);
	printf("[VPP] i2c read (0x%x,%d):0x%x 0x%x",ucI2cAddr,dwCount,ucpBuffer[0],ucpBuffer[1]);
	return ret;
}
#endif

tDWORD swi2c_I2C_ReadRegs(
    tHANDLE      handle,
    tBYTE        ucDemod2WireAddr,
    tBYTE        ucSlaveDemod,
    tWORD        wRegAddr,
    tBYTE        ucCount,               // hw only support single read and ucCount must be 1
    tBYTE *      ucpValue
)
{
    tDWORD   dwError = ERR_I2C_NO_ERROR;
//    tBYTE    ucRegLo;
//    tBYTE    ucRegHi;
//    tBYTE    ucCmd;
    tBYTE	 buffer[24];
    tDWORD   dwReadBytes = 0;
    tDWORD   dwWriteBytes = 0;

#if 0
    if (ucCount > 1)
    {
        dwError = ERR_I2C_DONT_SUPPORT;
        return (dwError);
    }
#endif    

#if 1
	buffer[0] = wRegAddr;
#else
    ucRegLo   = (tBYTE)wRegAddr;            // Get low byte of reg. address
    ucRegHi   = (tBYTE)(wRegAddr >> 8);     // Get high byte of reg. address

    ucCmd     = AF901X_EMBX_CMD_READ_REG;
    
    buffer[0] = ucRegHi;
   buffer[1] = ucRegLo;
   buffer[2] = ucCmd;    
#endif

    dwError = swi2c_BusWrite(handle, ucDemod2WireAddr, buffer, 1, &dwWriteBytes);
    if (dwError) goto exit;

    dwError = swi2c_BusRead(handle, ucDemod2WireAddr, ucpValue, ucCount, &dwReadBytes);
    if (dwError) goto exit;

exit:

	return (dwError);
}

#if 0
/*****************************************************************
 * Mercury i2c burst write sequence:
 *
 *      Start / i2c_slave_addr / 0 / Ack / Register_addr_msb / Ack /
 *      Register_addr_lsb / Ack / Register_access_cmd / Ack /
 *      Register_data_0 / Ack / Register_data_1 / Ack ¡K¡K¡K./ Stop
 *
 *****************************************************************/
tDWORD swi2c_I2C_WriteRegs(
    tHANDLE     handle,
    tBYTE       ucDemod2WireAddr,
    tBYTE       ucSlaveDemod,
    tWORD       wReg,
    tBYTE       ucCount,
    tBYTE *     ucpValBuf
)
{
    tDWORD   dwError = ERR_I2C_NO_ERROR;
    tBYTE    ucRegLo;
    tBYTE    ucRegHi;
    tBYTE    ucCmd;
	tBYTE	 i;
    tBYTE    buffer[24];
    tDWORD   dwWriteBytes = 0;
    tBYTE    ucTemp;
    
    
    ucRegLo   = (tBYTE)wReg;            // Get low byte of reg. address
    ucRegHi   = (tBYTE)(wReg >> 8);     // Get high byte of reg. address

    ucCmd     = 0;

    if (ucCount == 1)
    {
        ucCmd     = AF901X_EMBX_CMD_WRITE_REG;
    }
    else
    {
        ucCmd     = AF901X_EMBX_CMD_WRITE_REGS(ucCount);    
    }

    
    buffer[0] = ucRegHi;
    buffer[1] = ucRegLo;
    buffer[2] = ucCmd;
    for (i = 0; i < ucCount; i++)
    {
        buffer[3 + i] = *(ucpValBuf + i);
    }

    dwError = swi2c_BusWrite(handle, ucDemod2WireAddr, buffer, (tDWORD)(ucCount + 3), &dwWriteBytes);
    if (dwError) goto exit;
    
    //
    // Check if write-registere successful
    //
    for (i = 0; i < 200; i++)
    {
        dwError = swi2c_I2C_ReadRegs(handle, ucDemod2WireAddr, ucSlaveDemod, AF901x_REG_2WIRE_CMD_STATUS, 1, &ucTemp);
        if (dwError) goto exit;

        if (!(ucTemp & 0x04))
        {
            break;
        }

        // SysDelay(10);
    }

    if (i == 200)
    {
        dwError = ERR_WRITE_REG_TIMEOUT;
        goto exit;
    }

exit:
	return (dwError);
}
#endif
