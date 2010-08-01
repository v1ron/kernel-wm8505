#ifndef _USERDEF_H_
#define _USERDEF_H_


typedef     unsigned char   tBYTE;      // 1 byte
typedef     unsigned short  tWORD;      // 2 bytes
typedef     unsigned long   tDWORD;     // 4 bytes
typedef     long            tINT;       // 4 bytes
typedef     void *          tHANDLE;
#if 0
typedef enum {
     false = 0,
     true
}bool;
#endif

#define IN
#define OUT

#define AF901X_EXT_SWITCH_BUS       0
#define FLOAT_POINT_CFOE_COFF       0
#define AF9013_13S_HW_POWERDOWN     0       // api_mpeg_only

tDWORD SysEnterCriticalSection(void);
tDWORD SysLeaveCriticalSection(void);

tDWORD SysDelay(tDWORD dwMs);

#endif // _USERDEF_H_
