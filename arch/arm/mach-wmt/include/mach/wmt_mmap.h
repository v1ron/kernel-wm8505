/*++
	linux/include/asm-arm/arch-wmt/wmt_mmap.h

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
--*/

/* Be sure that virtual mapping is defined right */
#ifndef __ASM_ARCH_HARDWARE_H
#error "You must include hardware.h, not wmt_mmap.h"
#endif

#ifndef __WMT_MMAP_H
#define __WMT_MMAP_H



#define EXTERNAL_AHB_BRIDGE_BASE_ADDR           0xB0000000
#define INTERNAL_AHB_SLAVES_BASE_ADDR           0xD8000000
#define INTERNAL_APB_SLAVES_BASE_ADDR           0xD8100000

/*
 *  Internal AHB Slaves Memory Address Map
 */

#define MEMORY_CTRL_CFG_BASE_ADDR               0xD8000000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8000400 - 0xD8000FFF */
#define DMA_CTRL_CFG_BASE_ADDR                  0xD8001800  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8001400 - 0xD8001FFF */
#define SF_MEM_CTRL_CFG_BASE_ADDR               0xD8002000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8002400 - 0xD8002FFF */
#define SPI_MEM_CTRL_CFG_BASE_ADDR              0xD8003000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8003400 - 0xD8003FFF */
#define ETHERNET_MAC_0_CFG_BASE_ADDR            0xD8004000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8004400 - 0xD8004FFF */
#define ETHERNET_MAC_1_CFG_BASE_ADDR            0xD8005000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8005400 - 0xD8005FFF */
#define SECURITY_ENGINE_CFG_BASE_ADDR           0xD8006000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8006400 - 0xD8006FFF */
#define USB20_HOST_CFG_BASE_ADDR                0xD8007000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8007400 - 0xD8007FFF */
#define PATA_CTRL_CFG_BASE_ADDR                 0xD8008000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8008400 - 0xD8008FFF */
#define NF_CTRL_CFG_BASE_ADDR                   0xD8009000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8009400 - 0xD8009FFF */
#define SD_SDIO_MMC_BASE_ADDR                   0xD800A000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD800A400 - 0xD800AFFF */
#define MS_CTRL_CFG_BASE_ADDR                   0xD800B000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD800B400 - 0xD800BFFF */
#define CF_CTRL_CFG_BASE_ADDR                   0xD800C000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD800C400 - 0xD800CFFF */
#define SATA_CTRL_CFG_BASE_ADDR                 0xD800D000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD800D400 - 0xD800DFFF */
#define XOR_CTRL_CFG_BASE_ADDR                  0xD800E000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD800E400 - 0xD803FFFF */
#define LPC_CTRL_CFG_BASE_ADDR                  0xD8040000  /* 1K , 8/16/32 RW */
/* Reserved                                     0xD8050000 - 0xD80FFFFF */

#define VPP_BASE_ADDR     						0xD8050F00
#define VPU_BASE_ADDR				        	0xD8050000
#define SPU1_BASE_ADDR					       	0xD8050100
#define SPU2_BASE_ADDR      				 	0xD8050200
#define GOVM_BASE_ADDR   				    	0xD8050300
#define GE1_BASE_ADDR       				 	0xD8050400
#define GE2_BASE_ADDR     					   	0xD8050500
#define GE3_BASE_ADDR        					0xD8050600
#define DISP_BASE_ADDR     					  	0xD8050700
#define GOVRH_BASE1_ADDR  					   	0xD8050800
#define GOVRH_BASE2_ADDR   					  	0xD8050900
#define VID_BASE_ADDR      					  	0xD8050A00
#define GOVW_BASE_ADDR       					0xD8050C00
#define SCL_BASE_ADDR    						0xD8050D00
#define HDMI1_BASE_ADDR    					  	0xD806C000
#define HDMI2_BASE_ADDR      					0xD8070000
#define GOVR_BASE_ADDR       					0xD8050B00

/*
 *  Internal APB Slaves Memory Address Map
 */
#define RTC_BASE_ADDR                           0xD8100000  /* 64K  */
#define GPIO_BASE_ADDR                          0xD8110000  /* 64K  */
#define SYSTEM_CFG_CTRL_BASE_ADDR               0xD8120000  /* 64K  */
#define PM_CTRL_BASE_ADDR                       0xD8130000  /* 64K  */
#define INTERRUPT_CTRL_BASE_ADDR                0xD8140000  /* 64K  */
/* Reserved                                     0xD8150000 - 0xD81FFFFF */
#define UART1_BASE_ADDR                         0xD8200000  /* 64K  */
#define UART2_BASE_ADDR                         0xD82b0000  /* 64K  */
#define UART3_BASE_ADDR                         0xD8210000  /* 64K  */
#define UART4_BASE_ADDR                         0xD82c0000  /* 64K  */
#define UART5_BASE_ADDR                         0xD8370000  /* 64K  */
#define UART6_BASE_ADDR                         0xD8380000  /* 64K  */
/* Reserved                                     0xD8220000 - 0xD823FFFF */
#define SPI_BASE_ADDR                           0xD8240000  /* 64K  */
/* Reserved                                     0xD8250000 - 0xD827FFFF */
#define KPAD_BASE_ADDR							0xD8260000  /* 64K  */
#define I2C_BASE_ADDR                           0xD8280000  /* 64K  */
/* Reserved                                     0xD8290000 - 0xD8FFFFFF */
#define I2S_BASE_ADDR                           0xD8330000
#define PCM_BASE_ADDR                           0xD82D0000
#define AC97_BASE_ADDR                          0xD8290000
#endif	/* __WMT_MMAP_H */
