/*++
	linux/include/asm-arm/arch-wmt/vt1613.h

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

#ifndef __VT1613AC_H
#define __VT1613AC_H

#include <mach/common_def.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

/******************************************************************************
 *
 * VT1613AC Register Definitions.
 *
 ******************************************************************************/
#define VTAC_RSTR        0x00    /* Reset register                         */
#define VTAC_MASV        0x02    /* Master Volume register                 */
#define VTAC_HPSV        0x04	 /* Headphone Volume register              */
#define VTAC_MICV        0x0E    /* MIC Volume register                    */
#define VTAC_LINV        0x10	 /* Line-in Volume register                */
#define VRAC_PCMV        0x18	 /* PCM Volume register                    */
#define VTAC_RECS        0x1A    /* Record Select register                 */
#define VTAC_RECG        0x1C    /* Record Gain register                   */
#define VTAC_GEPR        0x20    /* General Purpose Register               */
#define VTAC_PDCS        0x26    /* Power-down Control/Status register     */
#define VTAC_EAID        0x28    /* Extended Audio ID register             */
#define VTAC_EASC        0x2A    /* Extended Audio Status/Ctrl register    */
#define VTAC_DSRC        0x2C    /* Audio DAC Sample Rate Control register */
#define VTAC_ASRC        0x32    /* Audio ADC Sample Rate Control register */
#define VTAC_SPDIF       0x3A    /* SPDIF Control Register                 */
#define VTAC_IODA        0x5A    /* IO Data register                       */
#define VTAC_IODI        0x5C    /* IO Direction register                  */
#define VTAC_POIE        0x5E    /* Positive INT Enable register           */
#define VTAC_NEIE        0x60    /* Negative INT Enable register           */
#define VTAC_ICSR        0x62    /* INT Clear/Status register              */
#define VTAC_TSCR        0x64    /* Touch Screen Control Register          */
#define VTAC_ADCC        0x66    /* ADC Control register                   */
#define VTAC_ADCD        0x68    /* ADC Data register                      */
#define VTAC_FCS1        0x6A    /* Feature Control/Status register 1      */
#define VTAC_FCS2        0x6C    /* Feature Control/Status register 2      */
#define VTAC_TSTC        0x6E    /* Test Control register                  */
#define VTAC_EINT        0x70    /* Extra Interrupt register               */
#define VTAC_PLL         0x74	 /* PLL Setting                            */
#define VTAC_VID1        0x7C    /* Vendor ID1 register                    */
#define VTAC_VID2        0x7E    /* Vendor ID2 register                    */
/* 25 registers now. */
#define VTAC_MAX         VTAC_VID2    /* Max register offset               */

/******************************************************************************
 *
 * Master Volume register (index 0x02) bits definitions.
 *
 ******************************************************************************/
#define VTAC_MASV_MM             0x8000          /* Master Mute            */
#define VTAC_MASV_MMASK          0x3F            /* 6-bit attenuation Mask */
#define VTAC_MASV_ML(x)          (((x) & VTAC_MASV_MMASK) << 8)   /* D8-13  */
#define VTAC_MASV_MR(x)          ((x) & VTAC_MASV_MMASK)          /* D0-5   */

/******************************************************************************
 *
 * MIC Volume register (index 0x0E) bits definitions.
 *
 ******************************************************************************/
#define VTAC_MICV_BOOST          BIT6            /* 20dB boost */

/******************************************************************************
 *
 * Record Select register (index 0x1A) bits definitions.
 *
 ******************************************************************************/
#define VTAC_RECS_SMASK          0x7              /* Source select mask    */
#define VTAC_RECS_SL(x)          (((x) & VTAC_RECS_SMASK) << 8)   /* D8-10 */
#define VTAC_RECS_SR(x)          ((x) & VTAC_RECS_SMASK)          /* D0-2  */
#define VTAC_RECS_SL_MIC         VTAC_RECS_SL(0x0)        /* MIC           */
#define VTAC_RECS_SL_LINEIN      VTAC_RECS_SL(0x4)        /* Line-in L     */
#define VTAC_RECS_SR_COPY        VTAC_RECS_SR(0x0)        /* Copy left     */
#define VTAC_RECS_SR_LINEIN      VTAC_RECS_SR(0x4)        /* Line-in R     */

/******************************************************************************
 *
 * Record Gain register (index 0x1C) bits definitions.
 *
 ******************************************************************************/
#define VTAC_RECG_GMASK          0xF              /* Record gain mask            */
#define VTAC_RECG_GL(x)          (((x) & VTAC_RECG_GMASK) << 8) /* Left channel  */
#define VTAC_RECG_GR(x)          ((x) & VTAC_RECG_GMASK)        /* Right channel */

/******************************************************************************
 *
 * General Purpose Register (index 0x20) bits definitions.
 *
 ******************************************************************************/
#define VTAC_GEPR_LPBK           BIT7                /* ADC/DAC loopback mode */

/******************************************************************************
 *
 * Power-down Control/Status register (index 0x26) bits definitions.
 *
 ******************************************************************************/
#define VTAC_PDCS_ADC            BIT0            /* Audio ADC ready   */
#define VTAC_PDCS_DAC            BIT1            /* Audio DAC ready   */
#define VTAC_PDCS_REF            BIT3            /* Audio Vref normal */
#define VTAC_PDCS_PR0            BIT8
#define VTAC_PDCS_PR1            BIT9
#define VTAC_PDCS_PR3            BIT11
#define VTAC_PDCS_PR4            BIT12
#define VTAC_PDCS_PR5            BIT13

/******************************************************************************
 *
 * Extended Audio Status/Ctrl register (index 0x2A) bits definitions.
 *
 ******************************************************************************/
#define VTAC_EASC_VRA            BIT0             /* Variable Rate Audio mode */
#define VTAC_EASC_SPDIF          BIT2
#define VTAC_SPSA0               BIT4
#define VTAC_SPSA1               BIT5

/******************************************************************************
 *
 * Audio DAC Sample Rate Control register (index 0x2C) bits definitions.
 *
 ******************************************************************************/
#define VTAC_DSRC_DMASK          0xFFFF          /* Sample rate mask */
#define VTAC_DSRC_DR(x)          ((x) & VTAC_DSRC_DMASK)

/******************************************************************************
 *
 * Audio ADC Sample Rate Control register (index 0x32) bits definitions.
 *
 ******************************************************************************/
#define VTAC_ASRC_AMASK          0xFFFF          /* Sample rate mask */
#define VTAC_ASRC_AR(x)          ((x) & VTAC_ASRC_AMASK)

/******************************************************************************
 *
 * IO Data register (index 0x5A) bits definitions.
 *
 ******************************************************************************/
#define VTAC_IODA_IMASK          0x3FF
#define VTAC_IODA_IO(x)          ((1 << (x)) & VTAC_IODA_IMASK)

/******************************************************************************
 *
 * Positive INT Enable register (index 0x5E) bits definitions.
 *
 ******************************************************************************/
#define VTAC_POIE_IOMASK         0x3FF           /* Bits[0:9] for IO pins   */
#define VTAC_POIE_IOP(x)         (1 << (x) & VTAC_POIE_IOMASK)
/* Enable IO(x) rising edge
 * interrupt
 */
#define VTAC_POIE_IOP0           VTAC_POIE_IOP(0)
#define VTAC_POIE_IOP1           VTAC_POIE_IOP(1)
#define VTAC_POIE_IOP2           VTAC_POIE_IOP(2)
#define VTAC_POIE_IOP3           VTAC_POIE_IOP(3)
#define VTAC_POIE_IOP4           VTAC_POIE_IOP(4)
#define VTAC_POIE_IOP5           VTAC_POIE_IOP(5)
#define VTAC_POIE_IOP6           VTAC_POIE_IOP(6)
#define VTAC_POIE_IOP7           VTAC_POIE_IOP(7)
#define VTAC_POIE_IOP8           VTAC_POIE_IOP(8)
#define VTAC_POIE_IOP9           VTAC_POIE_IOP(9)
/* D10 is reserved */
#define VTAC_POIE_ADCP           BIT11   /* Enable ADC ready rising edge interrupt */

#define VTAC_POIE_TPXP           BIT12   /* Enable TSPX rising edge interrupt */

#define VTAC_POIE_TMXP           BIT13   /* Enable TSMX rising edge interrupt */

#define VTAC_POIE_CLPP           BIT14   /* Enable CLIP rising edge interrupt */

#define VTAC_POIE_OVLP           BIT15   /* Enable OVFL rising edge interrupt */

/******************************************************************************
 *
 * Negative INT Enable register (index 0x60) bits definitions.
 *
 ******************************************************************************/
#define VTAC_NEIE_IOMASK         0x3FF           /* Bits[0:9] for IO pins   */
#define VTAC_NEIE_ION(x)         (1 << (x) & VTAC_NEIE_IOMASK)
/* Enable IO(x) falling edge
 * interrupt
 */
#define VTAC_NEIE_ION0           VTAC_NEIE_ION(0)
#define VTAC_NEIE_ION1           VTAC_NEIE_ION(1)
#define VTAC_NEIE_ION2           VTAC_NEIE_ION(2)
#define VTAC_NEIE_ION3           VTAC_NEIE_ION(3)
#define VTAC_NEIE_ION4           VTAC_NEIE_ION(4)
#define VTAC_NEIE_ION5           VTAC_NEIE_ION(5)
#define VTAC_NEIE_ION6           VTAC_NEIE_ION(6)
#define VTAC_NEIE_ION7           VTAC_NEIE_ION(7)
#define VTAC_NEIE_ION8           VTAC_NEIE_ION(8)
#define VTAC_NEIE_ION9           VTAC_NEIE_ION(9)
/* D10 is reserved */
#define VTAC_NEIE_ADCN           BIT11   /* Enable ADC ready falling edge interrupt */

#define VTAC_NEIE_TPXN           BIT12   /* Enable TSPX falling edge interrupt */

#define VTAC_NEIE_TMXN           BIT13   /* Enable TSMX falling edge interrupt */

#define VTAC_NEIE_CLPN           BIT14   /* Enable CLIP falling edge interrupt */

#define VTAC_NEIE_OVLN           BIT15   /* Enable OVFL falling edge interrupt */

/******************************************************************************
 *
 * INT Clear/Status register (index 0x62) bits definitions.
 *
 ******************************************************************************/
#define VTAC_ICSR_IOMASK         0x3FF           /* Bits[0:9] for IO pins     */
#define VTAC_ICSR_IOS(x)         (1 << (x) & VTAC_ICSR_IOMASK)
/* IO(x) edge interrupt
 * active
 */
#define VTAC_ICSR_IOS0           VTAC_ICSR_IOS(0)
#define VTAC_ICSR_IOS1           VTAC_ICSR_IOS(1)
#define VTAC_ICSR_IOS2           VTAC_ICSR_IOS(2)
#define VTAC_ICSR_IOS3           VTAC_ICSR_IOS(3)
#define VTAC_ICSR_IOS4           VTAC_ICSR_IOS(4)
#define VTAC_ICSR_IOS5           VTAC_ICSR_IOS(5)
#define VTAC_ICSR_IOS6           VTAC_ICSR_IOS(6)
#define VTAC_ICSR_IOS7           VTAC_ICSR_IOS(7)
#define VTAC_ICSR_IOS8           VTAC_ICSR_IOS(8)
#define VTAC_ICSR_IOS9           VTAC_ICSR_IOS(9)
/* D10 is reserved */
#define VTAC_ICSR_ADCP           BIT11           /* ADC ready signal edge interrupt active */
#define VTAC_ICSR_TPXP           BIT12           /* TSPX edge interrupt active */
#define VTAC_ICSR_TMXP           BIT13           /* TSMX edge interrupt active */
#define VTAC_ICSR_CLPP           BIT14           /* CLIP edge interrupt active */
#define VTAC_ICSR_OVLP           BIT15           /* OVFL edge interrupt active */

/******************************************************************************
 *
 * Touch Screen Control register (index 0x64) bits definitions.
 *
 ******************************************************************************/
#define VTAC_TSCR_MXP            BIT0            /* TSMX pin is powered */
#define VTAC_TSCR_PXP            BIT1            /* TSPX pin is powered */
#define VTAC_TSCR_MYP            BIT2            /* TSMY pin is powered */
#define VTAC_TSCR_PYP            BIT3            /* TSPY pin is powered */
#define VTAC_TSCR_MXG            BIT4            /* TSMX pin is grounded */
#define VTAC_TSCR_PXG            BIT5            /* TSPX pin is grounded */
#define VTAC_TSCR_MYG            BIT6            /* TSMY pin is grounded */
#define VTAC_TSCR_PYG            BIT7            /* TSPY pin is grounded */
#define VTAC_TSCR_TMMASK         (BIT8 | BIT9)   /* Operation modes mask */
#define VTAC_TSCR_INT            (0 << 8)        /* Interrupt mode */
#define VTAC_TSCR_PRE            (1 << 8)        /* Pressure measurement mode */
#define VTAC_TSCR_POS            (1 << 9)        /* Position measurement mode */
#define VTAC_TSCR_HYSD           BIT10  /* Hysteresis is deactivated on the Schmitt triggers */
#define VTAC_TSCR_BIAS           BIT11           /* Touch screen bias circuitry is activated */
#define VTAC_TSCR_PX             BIT12           /* State of the TSPX */
#define VTAC_TSCR_MX             BIT13           /* State of the TSMX */
#define VTAC_TSCR_REV            (BIT14 | BIT15) /* Reversed bits */

/******************************************************************************
 *
 * ADC Control register (index 0x66) bits definitions.
 *
 ******************************************************************************/
#define VTAC_ADCC_ASE            BIT0
#define VTAC_ADCC_VREFB          BIT1
#define VTAC_ADCC_AIMASK         0x1C            /* Mask of D4 - D2 */
#define VTAC_ADCC_ADCMASK        0x7             /* Mask of ADC input src */
#define VTAC_ADCC_ADC(x)         (((x) & VTAC_ADCC_ADCMASK) << 2) /* ADC input */
#define VTAC_ADCC_TSPX           VTAC_ADCC_ADC(0)
#define VTAC_ADCC_TSMX           VTAC_ADCC_ADC(1)
#define VTAC_ADCC_TSPY           VTAC_ADCC_ADC(2)
#define VTAC_ADCC_TSMY           VTAC_ADCC_ADC(3)
#define VTAC_ADCC_AD0            VTAC_ADCC_ADC(4)
#define VTAC_ADCC_AD1            VTAC_ADCC_ADC(5)
#define VTAC_ADCC_AD2            VTAC_ADCC_ADC(6)
#define VTAC_ADCC_AD3            VTAC_ADCC_ADC(7)
/* D6 is reserved */
#define VTAC_ADCC_AS             BIT7            /* Start ADC conversion      */
/* D8 - D14 are reserved */
#define VTAC_ADCC_AE             BIT15           /* Enable ADC function       */

/******************************************************************************
 *
 * ADC Data register (index 0x68) bits definitions.
 *
 ******************************************************************************/
#define VTAC_ADCD_DATAMASK       0x3FF           /* Bit[0:9] for ADC data     */
#define VTAC_ADCD_ADV            BIT15           /* ADC conversion is in progress */

/******************************************************************************
 *
 * Feature Control/Status register 1 (index 0x6A) bits definitions.
 *
 ******************************************************************************/
#define VTAC_FCS1_OVFL           BIT0
/* D1 is reserved */
#define VTAC_FCS1_GIEN           BIT2
#define VTAC_FCS1_HIPS           BIT3
#define VTAC_FCS1_DC             BIT4
#define VTAC_FCS1_DE             BIT5
#define VTAC_FCS1_HPEN           BIT6
#define VTAC_FCS1_MODEMASK       (0x03 << 7)
#define VTAC_FCS1_FLATMODE       0
#define VTAC_FCS1_MINMODE        (0x01 << 7)
/* (0x02 << 7) is also minimum mode. */
#define VTAC_FCS1_MAXMODE        (0x03 << 7)
#define VTAC_FCS1_TRMASK         (0x03 << 9)
#define VTAC_FCS1_BBMASK         (0x0F << 11)
/* D15 is reserved */

#define VTAC_SPDIF_SR0           BIT12
#define VTAC_SPDIF_SR1           BIT13

/******************************************************************************
 *
 * Feature Control/Status register 2 (index 0x6C) bits definitions.
 *
 ******************************************************************************/
#define VTAC_FCS2_EVMASK         0x07
#define VTAC_FCS2_SLPMASK        (0x03 << 4)
#define VTAC_FCS2_SLPNONE        0
#define VTAC_FCS2_SLPCODEC       (0x01 << 4)
#define VTAC_FCS2_SLPPLL         (0x02 << 4)
#define VTAC_FCS2_SLPBOTH        (0x03 << 4)
#define VTAC_FCS2_AVENMASK       (0x03 << 10)
#define VTAC_FCS2_AVE            BIT12
#define VTAC_FCS2_SUEVMASK       (0x03 << 13)
#define VTAC_FCS2_SMT            BIT15

#define MAX_VOLUME_GAIN    0x1F
void vt1613_reg_backup(void);
void vt1613_reg_restore(void);


#endif  /* __VT1613AC_H */
