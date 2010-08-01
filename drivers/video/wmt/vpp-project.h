/* vpp-project.h - */

/* PVCS version log
** $Log: POST/vpp/vpp-project.h $
 *
 * 1		08/05/15 Andy Chiu, draft version
 * 2		08/10/01 Andy Chiu, rework for vt8510
 * 3		09/03/11 Sam Shen, add for wm8435
 */

/*=== vpp-comm.h =========================================================
 * Copyright (C) 2009 WonderMedia Tech. Corp.
 *
 * MODULE:	vpp-project.h --
 * AUTHOR:	AndyChiu
 * DATE:		2008/05/15
 * DES:		
 *			1. Only for VPP internal use, please define or declare common information in this file. 
 *======================================================================*/

#ifndef VPP_PROJECT_H
#define VPP_PROJECT_H

/*
* Product ID / Project ID
* 84xx series: 8420/3300, 8430/3357, 8435/3437
* 85xx series: 8500/3400, 8510/3426, 8520/3429
*/
//84xx series, (1-100)
#define VIA_PID_8420	10	// 3300
#define VIA_PID_8430	12	// 3357
#define WMT_PID_8435	14	// 3437
#define WMT_PID_8440	16	// 3392
#define WMT_PID_8425	18	// 3429
//85xx series, (101-200)
#define VIA_PID_8500	110	// 3400
#define WMT_PID_8505	111
#define WMT_PID_8510	112	// 3426*

//current pid
// #define WMT_CUR_PID		WMT_PID_8435
#ifndef WMT_CUR_PID
#ifdef __KERNEL__
	#ifdef CONFIG_VT9043A1
	#define WMT_CUR_PID		WMT_PID_8510
	#endif

	#ifdef CONFIG_WM0001
	#define WMT_CUR_PID		WMT_PID_8435
	#endif
#else
	#define WMT_CUR_PID		WMT_PID_8435
#endif
#endif

#define WMT_SUB_PID		WMT_PID_8505
#ifndef WMT_SUB_PID
	#define WMT_SUB_PID		0
#endif

/* Function Block */
#define WMT_FTBLK_GE
#define WMT_FTBLK_SCL
#define WMT_FTBLK_GOVM
#define WMT_FTBLK_GOVW
#define WMT_FTBLK_GOVRH

#if(WMT_CUR_PID == VIA_PID_8430)	//ProjectID: 3357

#elif(WMT_CUR_PID == WMT_PID_8435)	//ProjectID: wm3437

#ifdef __KERNEL__
#include <asm/arch-wmt/hardware.h>
#include <asm/arch/wmt_mmap.h>
#else
#define __ASM_ARCH_HARDWARE_H
#include "wmt_mmap.h"
#endif

#define WMT_FTBLK_VPU
#define WMT_FTBLK_DISP
#define WMT_FTBLK_PIP

#include "./register/wm8435/wm8435-vpp-reg.h"
#include "./register/wm8435/wm8435-scl-reg.h"
#include "./register/wm8435/wm8435-vpu-reg.h"
#include "./register/wm8435/wm8435-govm-reg.h"
#include "./register/wm8435/wm8435-govw-reg.h"
#include "./register/wm8435/wm8435-govrh-reg.h"
#include "./register/wm8435/wm8435-disp-reg.h"

#define VPP_IRQ_SCL_FINISH	IRQ_VPP_IRQ0
#define VPP_IRQ_SCL			IRQ_VPP_IRQ1
#define VPP_IRQ_SCL444_TG	IRQ_VPP_IRQ2
#define VPP_IRQ_VPPM		IRQ_VPP_IRQ3
#define VPP_IRQ_GOVW_TG		IRQ_VPP_IRQ4
#define VPP_IRQ_GOVW		IRQ_VPP_IRQ5
#define VPP_IRQ_GOVM		IRQ_VPP_IRQ6
#define VPP_IRQ_GE			IRQ_VPP_IRQ7
#define VPP_IRQ_GOVRH_TG	IRQ_VPP_IRQ8
#define VPP_IRQ_DVO			IRQ_VPP_IRQ9
#define VPP_IRQ_VID			IRQ_VPP_IRQ10
#define VPP_IRQ_GOVR		IRQ_VPP_IRQ11
#define VPP_IRQ_GOVRS_TG	IRQ_VPP_IRQ12
#define VPP_IRQ_VPU			IRQ_VPP_IRQ13
#define VPP_IRQ_VPU_TG		IRQ_VPP_IRQ14

#define VPP_DIV_VPP		VPP_DIV_NA12
#define VPP_VOINT_NO	0
#define VPP_BLT_PWM_NUM		0

#elif(WMT_CUR_PID == WMT_PID_8440)	//ProjectID: 3392

#elif(WMT_CUR_PID == WMT_PID_8510)	//ProjectID: wm3426

#ifdef __KERNEL__
//#include <asm/arch-wmt/hardware.h>
//#include <asm/arch/wmt_mmap.h>
#include <mach/hardware.h>
#include <mach/wmt_mmap.h>
#else
#include "wm8510-mmap.h"
#endif
#include "./register/wm8510/wm8510-vpp-reg.h"
#include "./register/wm8510/wm8510-scl-reg.h"
#include "./register/wm8510/wm8510-govm-reg.h"
#include "./register/wm8510/wm8510-govw-reg.h"
#include "./register/wm8510/wm8510-govrh-reg.h"

#define VPP_IRQ_VPU		IRQ_NA12_0
#define VPP_IRQ_VPPM	IRQ_NA12_1
#define VPP_IRQ_SCL		IRQ_NA12_3
#define VPP_IRQ_GOVM	IRQ_NA12_5
#define VPP_IRQ_GOVW	IRQ_NA12_8
#define VPP_IRQ_GOVR	IRQ_NA12_10

#define VPP_DIV_VPP		VPP_DIV_NA12
#define VPP_VOINT_NO	1
#define VPP_BLT_PWM_NUM		0

#elif(WMT_CUR_PID == WMT_PID_8425)	//ProjectID: 3429
#ifdef __KERNEL__
#include <asm/arch-wmt/hardware.h>
#include <asm/arch/wmt_mmap.h>
#else
#define __ASM_ARCH_HARDWARE_H
#include "wmt_mmap.h"
#endif
#undef WMT_FTBLK_GOVRH
#undef WMT_FTBLK_SCL
#define WMT_FTBLK_VPU
#define WMT_FTBLK_LCDC

#include "./register/wm8435/wm8435-vpp-reg.h"
// #include "./register/wm8435/wm8435-scl-reg.h"
#include "./register/wm8435/wm8435-vpu-reg.h"
#include "./register/wm8435/wm8435-govm-reg.h"
#include "./register/wm8435/wm8435-govw-reg.h"
#include "./register/wm8425/wm8425-lcdc-reg.h"


#define VPP_DIV_VPP		VPP_DIV_NA0
#define VPP_VOINT_NO	0

#define VPP_IRQ_SCL_FINISH	IRQ_VPP_IRQ0
// #define VPP_IRQ_SCL		IRQ_VPP_IRQ1
#define VPP_IRQ_SCL444_TG	IRQ_VPP_IRQ2
#define VPP_IRQ_VPPM		IRQ_VPP_IRQ3
#define VPP_IRQ_GOVW_TG		IRQ_VPP_IRQ4
#define VPP_IRQ_GOVW		IRQ_VPP_IRQ5
#define VPP_IRQ_GOVM		IRQ_VPP_IRQ6
#define VPP_IRQ_GE			IRQ_VPP_IRQ7
#define VPP_IRQ_GOVRH_TG	IRQ_VPP_IRQ8
// #define VPP_IRQ_DVO		IRQ_VPP_IRQ9
// #define VPP_IRQ_VID		IRQ_VPP_IRQ10
// #define VPP_IRQ_GOVR		IRQ_VPP_IRQ11
#define VPP_IRQ_GOVRS_TG	IRQ_VPP_IRQ12
#define VPP_IRQ_VPU			IRQ_VPP_IRQ13
#define VPP_IRQ_VPU_TG		IRQ_VPP_IRQ14

#define VPP_IRQ_LCD			IRQ_LCD
#define VPP_BLT_PWM_NUM		1

#endif

/*======================================================================*/
/*======================================================================*/

#if 0
#define VPPPOST_DEBUG
#define VPPIF_DEBUG
#define VPP_DEBUG
#define VPU_DEBUG
#define SCL_DEBUG
#define GOVM_DEBUG
#define GOVW_DEBUG
#define GOVRS_DEBUG
#define GOVRH_DEBUG
#endif

#define BOUNDARY_CHECK

#endif //VPP_PROJECT_H
