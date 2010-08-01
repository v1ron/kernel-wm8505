/* PVCS version log
** $Log: /VT3357/LinuxKernel_MVL_2.6.10_Integration/drivers/input/touchscreen/vt8430-ts/vt8430-ts.h $
 * 
 * 5     07/09/07 5:10p Paulkwong
 * 
 * 4     07/07/02 6:06p Paulkwong
 * 
 * 3     07/06/25 11:43a Paulkwong
 * 
 * 2     07/03/05 10:32a Paulkwong
 * 
 * 1     07/02/01 12:09p Paulkwong
 * 
 */

#ifndef VT8430_TS_H
/* To assert that only one occurrence is included */
#define VT8430_TS_H



#include <linux/module.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/completion.h>
//#include <asm-arm/arch-vt8430/vt8430_ac97.h>
/*
extern void FASTCALL(__sema_init(struct semaphore *sem, int val, int debug, char *name, char *file, int line));
#define sema_init_nocheck(sem, val) \
                __sema_init(sem, val, 0, #sem, __FILE__, __LINE__)
*/
//typedef struct semaphore	mutex_t;

#define MUTEX_DEFAULT		0x0

//#define mutex_init(lock)		sema_init_nocheck(lock, 1)
//#define mutex_destroy(lock)		sema_init_nocheck(lock, -99)
//#define mutex_lock(lock)		down(lock)
//#define mutex_trylock(lock)		(down_trylock(lock) ? 0 : 1)
//#define mutex_unlock(lock)		up(lock)




/* ID numbers */
#define WM97XX_ID1			0x574d
#define WM9712_ID2			0x4c12
#define WM9705_ID2			0x4c05
#define WM9713_ID2			0x4c13



/*
 * WM97xx register bits
 */
#define WM97XX_POLL			0x8000	/* initiate a polling measurement */
#define WM97XX_ADCSEL_X		0x1000	/* x coord measurement */
#define WM97XX_ADCSEL_Y		0x2000	/* y coord measurement */
#define WM97XX_ADCSEL_PRES	0x3000	/* pressure measurement */
#define WM97XX_ADCSEL_MASK	0x7000
#define WM97XX_COO			0x0800	/* enable coordinate mode */
#define WM97XX_CTC			0x0400	/* enable continuous mode */
#define WM97XX_CM_RATE_93	0x0000	/* 93.75Hz continuous rate */
#define WM97XX_CM_RATE_187	0x0100	/* 187.5Hz continuous rate */
#define WM97XX_CM_RATE_375	0x0200	/* 375Hz continuous rate */
#define WM97XX_CM_RATE_750	0x0300	/* 750Hz continuous rate */
#define WM97XX_CM_RATE_8K	0x00f0	/* 8kHz continuous rate */
#define WM97XX_CM_RATE_12K	0x01f0	/* 12kHz continuous rate */
#define WM97XX_CM_RATE_24K	0x02f0	/* 24kHz continuous rate */
#define WM97XX_CM_RATE_48K	0x03f0	/* 48kHz continuous rate */
#define WM97XX_CM_RATE_MASK	0x03f0
#define WM97XX_RATE(i)		(((i & 3) << 8) | ((i & 4) ? 0xf0 : 0))
#define WM97XX_DELAY(i)		((i << 4) & 0x00f0)	/* sample delay times */
#define WM97XX_DELAY_MASK	0x00f0
#define WM97XX_SLEN			0x0008	/* slot read back enable */
#define WM97XX_SLT(i)		((i - 5) & 0x7)	/* touchpanel slot selection (5-11) */
#define WM97XX_SLT_MASK		0x0007
#define WM97XX_PRP_DETW		0x4000	/* pen detect on, digitiser off, wake up */
#define WM97XX_PRP_DET		0x8000	/* pen detect on, digitiser off, no wake up */
#define WM97XX_PRP_DET_DIG	0xc000	/* pen detect on, digitiser on */
#define WM97XX_RPR			0x2000	/* wake up on pen down */
#define WM97XX_PEN_DOWN		0x8000	/* pen is down */
#define WM97XX_ADCSRC_MASK	0x7000	/* ADC source mask */

#define WM97XX_AUX_ID1		0x8001
#define WM97XX_AUX_ID2		0x8002
#define WM97XX_AUX_ID3		0x8003
#define WM97XX_AUX_ID4		0x8004

#define WM97XX_ADCSEL_COMP1	0x4000	/* comp1/AUX1 measurement */
#define WM97XX_ADCSEL_COMP2	0x5000	/* comp2/AUX2 measurement */
#define WM97XX_ADCSEL_BMON	0x6000	/* BMON/AUX3 measurement */
#define WM97XX_ADCSEL_WIPER	0x7000	/* WIPER/AUX4 measurement */

/*
 * WM97xx AC97 Touchscreen registers
 */
#define AC97_WM97XX_DIGITISER1		0x76
#define AC97_WM97XX_DIGITISER2		0x78
#define AC97_WM97XX_DIGITISER_RD 	0x7a



#define AC97_LINK_FRAME		21	/* time in uS for AC97 link frame */


/*---------------- Return codes from sample reading functions ---------------*/

/* More data is available; call the sample gathering function again */
#define RC_AGAIN			0x00000001
/* The returned sample is valid */
#define RC_VALID			0x00000002
/* The pen is up (the first RC_VALID without RC_PENUP means pen is down) */
#define RC_PENUP			0x00000004
/* The pen is down (RC_VALID implies RC_PENDOWN, but sometimes it is helpful
   to tell the handler that the pen is down but we don't know yet his coords,
   so the handler should not sleep or wait for pendown irq) */
#define RC_PENDOWN			0x00000008
#define RC_TIMEOUT			0x00000010  // error for timeout
#define RC_ERROR			0x00000020  // error for other reseason


/* WM9712/9715 Bits */
#define WM9712_45W			0x1000	/* set for 5-wire touchscreen */
#define WM9712_PDEN			0x0800	/* measure only when pen down */
#define WM9712_WAIT			0x0200	/* wait until adc is read before next sample */
#define WM9712_PIL			0x0100	/* current used for pressure measurement. set 400uA else 200uA */
#define WM9712_MASK_HI		0x0040	/* hi on mask pin (47) stops conversions */
#define WM9712_MASK_EDGE	0x0080	/* rising/falling edge on pin delays sample */
#define	WM9712_MASK_SYNC	0x00c0	/* rising/falling edge on mask initiates sample */
#define WM9712_RPU(i)		(i&0x3f)	/* internal pull up on pen detect (64k / rpu) */
#define WM9712_PD(i)		(0x1 << i)	/* power management */

#define WM9715_VRA			BIT0	//variable rate audio, in reg WM9715_SPDIF

/*WM9715 power A bit in 0x24, 0 on, 1 off*/
#define WM9715_XTAL					BIT15		
#define WM9715_LDAC					BIT14		
#define WM9715_RDAC					BIT13		
#define WM9715_LADC					BIT12		
#define WM9715_RADC					BIT11		
#define WM9715_MIC_BIAS				BIT10		
#define WM9715_HP_LMIXER			BIT9		
#define WM9715_HP_RMIXER			BIT8		
#define WM9715_SPK_MIXER			BIT7		
#define WM9715_MONO_PHONE_MIXER		BIT6		
#define WM9715_OUT3					BIT5		
#define WM9715_HP_BUF				BIT4		
#define WM9715_SPK_BUF				BIT3		
#define WM9715_LINE_IN_PGA			BIT2		
#define WM9715_PHONE_IN_PGA			BIT1		
#define WM9715_MIC_IN_PGA			BIT0	

/*WM9715 power B bit in 0x26, 0 on, 1 off*/
#define WM9715_HP_OUT3_BUF			BIT14		
#define WM9715_INTERNAL_CLK			BIT13		
#define WM9715_ACLINK				BIT12		
#define WM9715_VREF					BIT11		
#define WM9715_ANALOG_OUT			BIT10		
#define WM9715_DAC_AUX				BIT9		
#define WM9715_ADC_AUX				BIT8		


/* WM9712 Registers */
#define AC97_WM9712_POWER				0x24
#define AC97_WM9712_REV					0x58

/* WM9715 Registers */
#define WM9715_OUT2_VOLUME			0x02
#define WM9715_HP_VOLUME			0x04
#define WM9715_MONO_VOLUME			0x06
#define WM9715_OUT3_VOLUME			0x16
#define WM9715_DAC_VOLUME			0x18

#define WM9715_POWER_A				0x24
#define WM9715_POWER_B				0x26

#define WM9715_AUDIO_STAT_CTRL		0x2a
#define WM9715_DAC_SAMPLE_RATE		0x2c
#define WM9715_AUX_SAMPLE_RATE		0x2e
#define WM9715_ADC_SAMPLE_RATE		0x32


/*
 * Register bits and API for Wolfson WM97xx series of codecs
 */

/* AUX ADC ID's */
#define TS_COMP1			0x0
#define TS_COMP2			0x1
#define TS_BMON				0x2
#define TS_WIPER			0x3

/*
 * Digitiser ioctl commands
 */
#define WM97XX_DIG_START	0x1
#define WM97XX_DIG_STOP		0x2
#define WM97XX_PHY_INIT		0x3
#define WM97XX_AUX_PREPARE	0x4
#define WM97XX_DIG_RESTORE	0x5

#if 0

/*--- vt8430-ts.h---------------------------------------------------------------
*   Copyright (C) 2006 VIA Tech. Inc.
*
* MODULE       : vt8430-ts.h -- 
* AUTHOR       : Paul Kwong
* DATE         : 2007/1/17
* DESCRIPTION  : 
*------------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Paul Kwong, 2007/1/17
*	First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/
#include <linux/interrupt.h>
#include "wm9715-ts-io.h"
#ifdef __KERNEL__
#include <asm/arch-vt8430/vt8430-hardware.h>
#else
#include "vt8430-hardware.h"
#endif
/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/


/* ID numbers */
#define WM97XX_ID1			0x574d
#define WM9712_ID2			0x4c12
#define WM9705_ID2			0x4c05
#define WM9713_ID2			0x4c13



/*
 * WM97xx register bits
 */
#define WM97XX_POLL			0x8000	/* initiate a polling measurement */
#define WM97XX_ADCSEL_X		0x1000	/* x coord measurement */
#define WM97XX_ADCSEL_Y		0x2000	/* y coord measurement */
#define WM97XX_ADCSEL_PRES	0x3000	/* pressure measurement */
#define WM97XX_ADCSEL_MASK	0x7000
#define WM97XX_COO			0x0800	/* enable coordinate mode */
#define WM97XX_CTC			0x0400	/* enable continuous mode */
#define WM97XX_CM_RATE_93	0x0000	/* 93.75Hz continuous rate */
#define WM97XX_CM_RATE_187	0x0100	/* 187.5Hz continuous rate */
#define WM97XX_CM_RATE_375	0x0200	/* 375Hz continuous rate */
#define WM97XX_CM_RATE_750	0x0300	/* 750Hz continuous rate */
#define WM97XX_CM_RATE_8K	0x00f0	/* 8kHz continuous rate */
#define WM97XX_CM_RATE_12K	0x01f0	/* 12kHz continuous rate */
#define WM97XX_CM_RATE_24K	0x02f0	/* 24kHz continuous rate */
#define WM97XX_CM_RATE_48K	0x03f0	/* 48kHz continuous rate */
#define WM97XX_CM_RATE_MASK	0x03f0
#define WM97XX_RATE(i)		(((i & 3) << 8) | ((i & 4) ? 0xf0 : 0))
#define WM97XX_DELAY(i)		((i << 4) & 0x00f0)	/* sample delay times */
#define WM97XX_DELAY_MASK	0x00f0
#define WM97XX_SLEN			0x0008	/* slot read back enable */
#define WM97XX_SLT(i)		((i - 5) & 0x7)	/* touchpanel slot selection (5-11) */
#define WM97XX_SLT_MASK		0x0007
#define WM97XX_PRP_DETW		0x4000	/* pen detect on, digitiser off, wake up */
#define WM97XX_PRP_DET		0x8000	/* pen detect on, digitiser off, no wake up */
#define WM97XX_PRP_DET_DIG	0xc000	/* pen detect on, digitiser on */
#define WM97XX_RPR			0x2000	/* wake up on pen down */
#define WM97XX_PEN_DOWN		0x8000	/* pen is down */
#define WM97XX_ADCSRC_MASK	0x7000	/* ADC source mask */

#define WM97XX_AUX_ID1		0x8001
#define WM97XX_AUX_ID2		0x8002
#define WM97XX_AUX_ID3		0x8003
#define WM97XX_AUX_ID4		0x8004

#define WM97XX_ADCSEL_COMP1	0x4000	/* comp1/AUX1 measurement */
#define WM97XX_ADCSEL_COMP2	0x5000	/* comp2/AUX2 measurement */
#define WM97XX_ADCSEL_BMON	0x6000	/* BMON/AUX3 measurement */
#define WM97XX_ADCSEL_WIPER	0x7000	/* WIPER/AUX4 measurement */

/*
 * WM97xx AC97 Touchscreen registers
 */
#define AC97_WM97XX_DIGITISER1		0x76
#define AC97_WM97XX_DIGITISER2		0x78
#define AC97_WM97XX_DIGITISER_RD 	0x7a



#define AC97_LINK_FRAME		21	/* time in uS for AC97 link frame */


/*---------------- Return codes from sample reading functions ---------------*/

/* More data is available; call the sample gathering function again */
#define RC_AGAIN			0x00000001
/* The returned sample is valid */
#define RC_VALID			0x00000002
/* The pen is up (the first RC_VALID without RC_PENUP means pen is down) */
#define RC_PENUP			0x00000004
/* The pen is down (RC_VALID implies RC_PENDOWN, but sometimes it is helpful
   to tell the handler that the pen is down but we don't know yet his coords,
   so the handler should not sleep or wait for pendown irq) */
#define RC_PENDOWN			0x00000008


/* WM9712/9715 Bits */
#define WM9712_45W			0x1000	/* set for 5-wire touchscreen */
#define WM9712_PDEN			0x0800	/* measure only when pen down */
#define WM9712_WAIT			0x0200	/* wait until adc is read before next sample */
#define WM9712_PIL			0x0100	/* current used for pressure measurement. set 400uA else 200uA */
#define WM9712_MASK_HI		0x0040	/* hi on mask pin (47) stops conversions */
#define WM9712_MASK_EDGE	0x0080	/* rising/falling edge on pin delays sample */
#define	WM9712_MASK_SYNC	0x00c0	/* rising/falling edge on mask initiates sample */
#define WM9712_RPU(i)		(i&0x3f)	/* internal pull up on pen detect (64k / rpu) */
#define WM9712_PD(i)		(0x1 << i)	/* power management */

#define WM9715_VRA			BIT0	//variable rate audio, in reg WM9715_SPDIF

/*WM9715 power A bit in 0x24, 0 on, 1 off*/
#define WM9715_XTAL					BIT15		
#define WM9715_LDAC					BIT14		
#define WM9715_RDAC					BIT13		
#define WM9715_LADC					BIT12		
#define WM9715_RADC					BIT11		
#define WM9715_MIC_BIAS				BIT10		
#define WM9715_HP_LMIXER			BIT9		
#define WM9715_HP_RMIXER			BIT8		
#define WM9715_SPK_MIXER			BIT7		
#define WM9715_MONO_PHONE_MIXER		BIT6		
#define WM9715_OUT3					BIT5		
#define WM9715_HP_BUF				BIT4		
#define WM9715_SPK_BUF				BIT3		
#define WM9715_LINE_IN_PGA			BIT2		
#define WM9715_PHONE_IN_PGA			BIT1		
#define WM9715_MIC_IN_PGA			BIT0	

/*WM9715 power B bit in 0x26, 0 on, 1 off*/
#define WM9715_HP_OUT3_BUF			BIT14		
#define WM9715_INTERNAL_CLK			BIT13		
#define WM9715_ACLINK				BIT12		
#define WM9715_VREF					BIT11		
#define WM9715_ANALOG_OUT			BIT10		
#define WM9715_DAC_AUX				BIT9		
#define WM9715_ADC_AUX				BIT8		


/* WM9712 Registers */
#define AC97_WM9712_POWER				0x24
#define AC97_WM9712_REV					0x58

/* WM9715 Registers */
#define WM9715_OUT2_VOLUME			0x02
#define WM9715_HP_VOLUME			0x04
#define WM9715_MONO_VOLUME			0x06
#define WM9715_OUT3_VOLUME			0x16
#define WM9715_DAC_VOLUME			0x18

#define WM9715_POWER_A				0x24
#define WM9715_POWER_B				0x26

#define WM9715_AUDIO_STAT_CTRL		0x2a
#define WM9715_DAC_SAMPLE_RATE		0x2c
#define WM9715_AUX_SAMPLE_RATE		0x2e
#define WM9715_ADC_SAMPLE_RATE		0x32


/*
 * Register bits and API for Wolfson WM97xx series of codecs
 */

/* AUX ADC ID's */
#define TS_COMP1			0x0
#define TS_COMP2			0x1
#define TS_BMON				0x2
#define TS_WIPER			0x3

/*
 * Digitiser ioctl commands
 */
#define WM97XX_DIG_START	0x1
#define WM97XX_DIG_STOP		0x2
#define WM97XX_PHY_INIT		0x3
#define WM97XX_AUX_PREPARE	0x4
#define WM97XX_DIG_RESTORE	0x5


//************************WM9715 register end*************************


/* range 0x3c-0x58 - MODEM */
#define AC97_EXTENDED_MID	0x3c	/* Extended Modem ID */
#define AC97_EXTENDED_MSTATUS	0x3e	/* Extended Modem Status and Control */
#define AC97_LINE1_RATE		0x40	/* Line1 DAC/ADC Rate */
#define AC97_LINE2_RATE		0x42	/* Line2 DAC/ADC Rate */
#define AC97_HANDSET_RATE	0x44	/* Handset DAC/ADC Rate */
#define AC97_LINE1_LEVEL	0x46	/* Line1 DAC/ADC Level */
#define AC97_LINE2_LEVEL	0x48	/* Line2 DAC/ADC Level */
#define AC97_HANDSET_LEVEL	0x4a	/* Handset DAC/ADC Level */
#define AC97_GPIO_CFG		0x4c	/* GPIO Configuration */
#define AC97_GPIO_POLARITY	0x4e	/* GPIO Pin Polarity/Type, 0=low, 1=high active */
#define AC97_GPIO_STICKY	0x50	/* GPIO Pin Sticky, 0=not, 1=sticky */
#define AC97_GPIO_WAKEUP	0x52	/* GPIO Pin Wakeup, 0=no int, 1=yes int */
#define AC97_GPIO_STATUS	0x54	/* GPIO Pin Status, slot 12 */
#define AC97_MISC_AFE		0x56	/* Miscellaneous Modem AFE Status and Control */


//#define TS_NO_SPI_MODE

#define DEVICE_NAME "VT8430-TS" /* appear in /proc/devices & /proc/vt8430-ts */
#define BUFSIZE                 128
#define TS_XLIMIT           	0x16
#define TS_YLIMIT           	0x16

#define TS_MAX_X 3700 //3500
#define TS_MAX_Y 3600 //3800
#define TS_MIN_X 350  //341
#define TS_MIN_Y 500  //335

#define TS_MAX 0xFFF
#define TS_MIN 0x0

#define TS_SAMPLE 4

/* Touch Screen Type */
#define TS_LCD_43INCH  			0


/* Touch Screen Read SPI Sizes */
#define TS_READ_SPI_SIZE	16
#define TS_WRITE_SPI_SIZE	1
#define TS_READ_RET_BYTE	2
#define TS_DATA_BYTE_MASK	0x7FF8



/* Control register bits */
#define TS_START_BIT		( 1 << 7 )
#define TS_CH_A2_HIGH	( 1 << 6 )
#define TS_CH_A1_HIGH	( 1 << 5 )
#define TS_CH_A0_HIGH	( 1 << 4 )
#define TS_12B_MODE	( 0 << 3 )
#define TS_08B_MODE	( 1 << 3 )
#define TS_SER_MODE		( 1 << 2 )
#define TS_DEF_MODE	( 0 << 2 )
#define TS_PD1_HIGH		( 1 << 1 )
#define TS_PD0_HIGH		( 1 )


#define TS_VT8430_TS		IRQ_PMC_WAKEUP 		//IRQ_GPIO4

/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
struct ts_dev_s {
	/* module parameters */
	char *buf;

	/* char dev struct */
	struct cdev cdev;
};

typedef struct ts_panel_info_s {
	int res_x;
	int res_y;
	int raw_min_x;
	int raw_max_x;
	int x_rev;	
	int raw_min_y;
	int raw_max_y;
	int y_rev;	
	int xyswap;
} ts_panel_info_t;
/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef VT8430_TS_C /* allocate memory for variables only in vt8430-ts.c */
#define EXTERN
#else
#define EXTERN   extern
#endif /* ifdef VT8430_TS_C */


#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/


/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/

#endif

#endif /* ifndef VT8430_TS_H */

/*=== END vt8430-ts.h ==========================================================*/

