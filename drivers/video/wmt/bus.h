#ifndef _BUS_H_
#define _BUS_H_

#include "userdef.h"

// #define CTRL_BY_I2S
#ifndef CTRL_BY_I2S
#define CTRL_BY_GPIO
#endif

tDWORD  BusWrite(
    tHANDLE     handle,
    tBYTE       ucI2cAddr,      // api_mpeg_only
    tBYTE *     ucpBuffer,
    tDWORD      dwCount,
    tDWORD *    dwpWriteBytes
);

tDWORD BusRead(
    tHANDLE     handle,
    tBYTE       ucI2cAddr,      // api_mpeg_only
    tBYTE *     ucpBuffer,
    tDWORD      dwCount,
    tDWORD *    dwpReadBytes
);

tDWORD swi2c_I2C_ReadRegs(
    tHANDLE      handle,
    tBYTE        ucDemod2WireAddr,
    tBYTE        ucSlaveDemod,
    tWORD        wRegAddr,
    tBYTE        ucCount,               // hw only support single read and ucCount must be 1
    tBYTE *      ucpValue
);
#endif

