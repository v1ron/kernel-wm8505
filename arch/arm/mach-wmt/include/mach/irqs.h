/*
 *	linux/include/asm-arm/arch-wmt/irqs.h
 *
 *	Some descriptions of such software. Copyright (c) 2008 WonderMedia Technologies, Inc.
 *
 *	This program is free software: you can redistribute it and/or modify it under the
 *	terms of the GNU General Public License as published by the Free Software Foundation,
 *	either version 2 of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful, but WITHOUT
 *	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *	You should have received a copy of the GNU General Public License along with
 *	this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *	WonderMedia Technologies, Inc.
 *	10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/* WM8510 interrupt vectors */
/* Interrupt Controller 0   */
#define IRQ_UHC_F            0
#define IRQ_UHC_H            1
#define IRQ_UDC_MI_DMATC     2
/* IRQ_REVERSED              3 */
#define IRQ_MOUSE            4
#define IRQ_UDC_USBINTR      5
#define IRQ_GPIO0            6
#define IRQ_GPIO1            7
#define IRQ_KPAD             8
#define IRQ_VDMA             9
#define IRQ_ETH0             10
/* IRQ_REVERSED              11 ~ 12 */
#define IRQ_GPIO2            13
#define IRQ_GPIO3            14
#define IRQ_GPIO4            15
#define IRQ_APBB             16
#define IRQ_DMA_CH_0         17
#define IRQ_I2C1             18      /* I2C controller                   */
#define IRQ_I2C0             19      /* I2C controller                   */
#define IRQ_SDC              20      /* SD Host controller               */
#define IRQ_SDC_DMA          21
#define IRQ_PMC_WAKEUP       22      /* PMC wakeup                       */
#define IRQ_KEYBOARD         23
#define IRQ_SPI0             24      /* Serial Peripheral Interface 0    */
#define IRQ_SPI1             25
#define IRQ_SPI2             26
#define IRQ_DMA_CH_1         27
#define IRQ_NFC              28
#define IRQ_NFC_DMA          29
#define IRQ_UART5            30
#define IRQ_UART4            31
#define IRQ_UART0            32
#define IRQ_UART1            33
#define IRQ_DMA_CH_2         34
#define IRQ_I2S              35
#define IRQ_OST0             36      /* OS Timer match 0               	 */
#define IRQ_OST1             37      /* OS Timer match 1               	 */
#define IRQ_OST2             38      /* OS Timer match 2                 */
#define IRQ_OST3             39      /* OS Timer match 3               	 */
#define IRQ_DMA_CH_3         40
#define IRQ_DMA_CH_4         41
#define IRQ_AC97             42
/* IRQ_REVERSED              43 */
#define IRQ_NOR              44
#define IRQ_DMA_CH_5         45
#define IRQ_DMA_CH_6         46
#define IRQ_UART2            47
#define IRQ_RTC1             48      /* PWM Timer Input 0              	 */
#define IRQ_RTC2             49      /* PWM Timer Input 1              	 */
#define IRQ_UART3            50
#define IRQ_DMA_CH_7         51
#define IRQ_GPIO5            52
#define IRQ_GPIO6            53
#define IRQ_GPIO7            54
#define IRQ_CIR              55
#define IRQ_IC1_IRQ0         56
#define IRQ_IC1_IRQ1         57
#define IRQ_IC1_IRQ2         58
#define IRQ_IC1_IRQ3         59
#define IRQ_IC1_IRQ4         60
#define IRQ_IC1_IRQ5         61
#define IRQ_IC1_IRQ6         62
#define IRQ_IC1_IRQ7         63
/* Interrupt Controller 0   */
/* IRQ_REVERSED              64 */
#define IRQ_NA0_1            65
#define IRQ_NA0_2            66
/* IRQ_REVERSED              67 ~ 78 */
#define IRQ_NA12_0           79
#define IRQ_NA12_1           80
#define IRQ_NA12_2           81
#define IRQ_NA12_3           82
#define IRQ_NA12_4           83
#define IRQ_NA12_5           84
#define IRQ_NA12_6           85
#define IRQ_NA12_7           86
/* IRQ_REVERSED              87 ~ 91 */
#define IRQ_DMA_CH_8         92
#define IRQ_DMA_CH_9         93
#define IRQ_DMA_CH_10        94
#define IRQ_DMA_CH_11        95
#define IRQ_DMA_CH_12        96
#define IRQ_DMA_CH_13        97
#define IRQ_DMA_CH_14        98
#define IRQ_DMA_CH_15        99
/* IRQ_REVERSED              100 ~ 110 */
#define IRQ_NA12_8           111
#define IRQ_NA12_9           112
#define IRQ_NA12_10          113
#define IRQ_NA12_11          114
#define IRQ_NA12_12          115
/* IRQ_REVERSED              116 ~ 127 */

#endif
