/*++
	linux/include/asm-arm/arch-wmt/wmt_i2s.h

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

	History:
		2009/01/21 Dean Created

--*/
/* Be sure that virtual mapping is defined right */
#ifndef __ASM_ARCH_HARDWARE_H
#error "You must include hardware.h, not wmt_i2s.h"
#endif

#ifndef _WMT_I2S_H_
#define _WMT_I2S_H_

/*
 *   Refer WMT i2s v2 register 1.60
 *
 */
/*
 * Address
 */
#define I2S_AUDCTLCR_ADDR               (0x0000+I2S_BASE_ADDR)
#define I2S_AUDDFCR_ADDR                (0x0004+I2S_BASE_ADDR)
#define I2S_TXCLKDIV_ADDR               (0x0008+I2S_BASE_ADDR)
#define I2S_TXFRAMECR_ADDR              (0x000C+I2S_BASE_ADDR)
#define I2S_RXCLKDIV_ADDR               (0x0010+I2S_BASE_ADDR)
#define I2S_RXFRAMECR_ADDR              (0x0014+I2S_BASE_ADDR)
#define I2S_TXEQURXCR_ADDR              (0x0018+I2S_BASE_ADDR)
#define I2S_SPCTLCR_ADDR                (0x001C+I2S_BASE_ADDR)
#define I2S_TCR_ADDR                    (0x0020+I2S_BASE_ADDR)
#define I2S_TSR_ADDR                    (0x0024+I2S_BASE_ADDR)
#define I2S_RCR_ADDR                    (0x0028+I2S_BASE_ADDR)
#define I2S_RSR_ADDR                    (0x002C+I2S_BASE_ADDR)
/* Reserved 0x30 ~ 0x7F*/
#define I2S_TFIFO_ADDR                  (0x0080+I2S_BASE_ADDR)
#define I2S_TFIFO_1_ADDR                (0x0084+I2S_BASE_ADDR)

#define I2S_RFIFO_ADDR                  (0x00C0+I2S_BASE_ADDR)
#define I2S_RFIFO_1_ADDR                (0x00C4+I2S_BASE_ADDR)
/* Reserved 0x0100 ~ 0xFFFF */


/*
 * Registers
 */
#define I2S_AUDCTRL_REG                 REG32_PTR(0x0000+I2S_BASE_ADDR)
#define I2S_AUDDFCR_REG                 REG32_PTR(0x0004+I2S_BASE_ADDR)
#define I2S_TXCLKDIV_REG                REG32_PTR(0x0008+I2S_BASE_ADDR)
#define I2S_TXFRAMECR_REG               REG32_PTR(0x000C+I2S_BASE_ADDR)
#define I2S_RXCLKDIV_REG                REG32_PTR(0x0010+I2S_BASE_ADDR)
#define I2S_RXFRAMECR_REG               REG32_PTR(0x0014+I2S_BASE_ADDR)
#define I2S_TXEQURXCR_REG               REG32_PTR(0x0018+I2S_BASE_ADDR)
#define I2S_SPCTLCR_REG                 REG32_PTR(0x001C+I2S_BASE_ADDR)
#define I2S_TCR_REG                     REG32_PTR(0x0020+I2S_BASE_ADDR)
#define I2S_TSR_REG                     REG32_PTR(0x0024+I2S_BASE_ADDR)
#define I2S_RCR_REG                     REG32_PTR(0x0028+I2S_BASE_ADDR)
#define I2S_RSR_REG                     REG32_PTR(0x002C+I2S_BASE_ADDR)
/* Reserved 0x0030 ~ 0x007F */
#define I2S_TFIFO_REG                   REG32_PTR(0x0080+I2S_BASE_ADDR)
#define I2S_TFIFO_1_REG                 REG32_PTR(0x0084+I2S_BASE_ADDR)

#define I2S_RFIFO_REG                   REG32_PTR(0x00C0+I2S_BASE_ADDR)
#define I2S_RFIFO_1_REG                 REG32_PTR(0x00C4+I2S_BASE_ADDR)
/* Reserved 0x0100 ~ 0xFFFF */

/*
 * VAL Registers
 */
#define I2S_AUDCTRL_VAL                 REG32_VAL(0x0000+I2S_BASE_ADDR)
#define I2S_AUDDFCR_VAL                 REG32_VAL(0x0004+I2S_BASE_ADDR)
#define I2S_TXCLKDIV_VAL                REG32_VAL(0x0008+I2S_BASE_ADDR)
#define I2S_TXFRAMECR_VAL               REG32_VAL(0x000C+I2S_BASE_ADDR)
#define I2S_RXCLKDIV_VAL                REG32_VAL(0x0010+I2S_BASE_ADDR)
#define I2S_RXFRAMECR_VAL               REG32_VAL(0x0014+I2S_BASE_ADDR)
#define I2S_TXEQURXCR_VAL               REG32_VAL(0x0018+I2S_BASE_ADDR)
#define I2S_SPCTLCR_VAL                 REG32_VAL(0x001C+I2S_BASE_ADDR)
#define I2S_TCR_VAL                     REG32_VAL(0x0020+I2S_BASE_ADDR)
#define I2S_TSR_VAL                     REG32_VAL(0x0024+I2S_BASE_ADDR)
#define I2S_RCR_VAL                     REG32_VAL(0x0028+I2S_BASE_ADDR)
#define I2S_RSR_VAL                     REG32_VAL(0x002C+I2S_BASE_ADDR)
/* Reserved 0x0030 ~ 0x007F */
#define I2S_TFIFO_VAL                   REG32_VAL(0x0080+I2S_BASE_ADDR)
#define I2S_TFIFO_1_VAL                 REG32_VAL(0x0084+I2S_BASE_ADDR)

#define I2S_RFIFO_VAL                   REG32_VAL(0x00C0+I2S_BASE_ADDR)
#define I2S_RFIFO_1_VAL                 REG32_VAL(0x00C4+I2S_BASE_ADDR)
/* Reserved 0x0100 ~ 0xFFFF */
/*
#define AUDIOIF_FMT_I2S 0
#define AUDIOIF_FMT_RIGHTJUSTIFIED 1
#define AUDIOIF_FMT_LEFTJUSTIFIED 2
#define AUDIOIF_FMT_PCM 3
*/

/*
 *  I2S_AUDCTLCR_REG
 *
 */
/*  only is2 is master mode, i2s_sck will be send*/
/*Rx*/
#define I2S_RRecord_EN BIT29
#define I2S_LRecord_EN BIT28
#define I2S_RxSYNC_OFF BIT27
#define I2S_Rx_SAMPLE_BIT_SHIFT 24
#define I2S_Rx_SAMPLE_BIT_MASK (BIT26|BIT25|BIT24)
#define I2S_RX_POLINV_EN BIT23
#define I2S_RX_MODE_SEL_SHIFT 18
#define I2S_RX_SLAVE_EN BIT17
/*Tx*/
#define I2S_RPLAY_EN BIT13
#define I2S_LPLAY_EN BIT12
#define I2S_TxSYNC_OFF BIT11
#define I2S_Tx_SAMPLE_BIT_SHIFT 8
#define I2S_Tx_SAMPLE_BIT_MASK (BIT10|BIT9|BIT8)
#define I2S_TX_POLINV_EN BIT7
#define I2S_TX_MODE_SEL_SHIFT 2
#define I2S_TX_SLAVE_EN BIT1

#define I2S_SAMPLE_BIT32 0x6
#define I2S_SAMPLE_BIT24 0x5
#define I2S_SAMPLE_BIT20 0x4
#define I2S_SAMPLE_BIT16 0x3
#define I2S_SAMPLE_BIT14 0x2
#define I2S_SAMPLE_BIT13 0x1
#define I2S_SAMPLE_BIT8 0x0

#define I2S_RX_IF_MODE_SHIFT 18
#define I2S_TX_IF_MODE_SHIFT 2
#define I2S_I2S_MODE 0x0
#define I2S_RJ_MODE 0x1
#define I2S_LJ_MODE 0x2
#define I2S_DSP_MODE 0x3

#define I2S_RISINGEDGE_SYNC BIT16

#define I2S_AUD_EN BIT0

/*
* TXCLKDIV
*/
#define TXMCLK_DIV_SHIFT 16
#define TXBCLK_DIV_SHIFT 0

/*
* RXCLKDIV
*/
#define RXMCLK_DIV_SHIFT 16
#define RXBCLK_DIV_SHIFT 0

/*
 *  I2S_TCR_REG
 *
 */
/* BIT31 - BIT12 Reserved */
#define TCR_TFT_1                       0x00000100
#define TCR_TFT_2                       0x00000200
#define TCR_TFT_3                       0x00000300
#define TCR_TFT_4                       0x00000400
#define TCR_TFT_5                       0x00000500
#define TCR_TFT_6                       0x00000600
#define TCR_TFT_7                       0x00000700
#define TCR_TFT_8                       0x00000800
#define TCR_TFT_9                       0x00000900
#define TCR_TFT_10                      0x00000A00
#define TCR_TFT_11                      0x00000B00
#define TCR_TFT_12                      0x00000C00
#define TCR_TFT_13                      0x00000D00
#define TCR_TFT_14                      0x00000E00
#define TCR_TFT_15                      0x00000F00
#define TCR_TFT_16                      0x00000000
#define TCR_TFT_MASK	(BIT8 | BIT9 | BIT10 | BIT11)
#define TCR_TFT(x)    (((x) << 8) & TCR_TFT_MASK)    /* x = [0-16] valid */
/* BIT7 - BIT6 Reserved  */
#define TCR_TFOIE                       BIT5
#define TCR_TFUIE                       BIT4
#define TCR_TFADE                       BIT3
#define TCR_TFAIE                       BIT2
#define TCR_TFEIE                       BIT1
#define TCR_TFEN                        BIT0

/*
 *  I2S_TSR_REG
 *
 */
/* BIT31 - BIT4 Reserved   */
#define TSR_TFOE                        BIT3
#define TSR_TFOE_WRITE_CLEAR            BIT3
#define TSR_TFUE                        BIT2
#define TSR_TFUE_WRITE_CLEAR            BIT2
#define TSR_TFA                         BIT1
#define TSR_TFE                         BIT0

#define TSR_ALL_INT_ENABLE              0x3E

/*
 *  I2S_RCR_REG
 *
 */
/* BIT31 - BIT12 Reserved */
#define RCR_RFT_1                       0x00000100
#define RCR_RFT_2                       0x00000200
#define RCR_RFT_3                       0x00000300
#define RCR_RFT_4                       0x00000400
#define RCR_RFT_5                       0x00000500
#define RCR_RFT_6                       0x00000600
#define RCR_RFT_7                       0x00000700
#define RCR_RFT_8                       0x00000800
#define RCR_RFT_9                       0x00000900
#define RCR_RFT_10                      0x00000A00
#define RCR_RFT_11                      0x00000B00
#define RCR_RFT_12                      0x00000C00
#define RCR_RFT_13                      0x00000D00
#define RCR_RFT_14                      0x00000E00
#define RCR_RFT_15                      0x00000F00
#define RCR_RFT_16                      0x00000000
#define RCR_RFT_MASK	(BIT8 | BIT9 | BIT10 | BIT11)
#define RCR_RFT(x)    (((x) << 8) & RCR_RFT_MASK)    /* x = [0-16] valid */
/* BIT7 - BIT6 Reserved  */
#define RCR_RFOIE                       BIT5
#define RCR_RFUIE                       BIT4
#define RCR_RFADE                       BIT3
#define RCR_RFAIE                       BIT2
#define RCR_RFFIE                       BIT1
#define RCR_RFEN                        BIT0

#define RCR_ALL_INT_ENABLE              0x3E

/*
 *  I2S_RSR_REG
 *
 */
/* BIT31 - BIT4 Reserved */
#define RSR_RFOE                        BIT3
#define RSR_RFOE_WRITE_CLEAR            BIT3
#define RSR_RFUE                        BIT2
#define RSR_RFUE_WRITE_CLEAR            BIT2
#define RSR_RFA                         BIT1
#define RSR_RFF                         BIT0

/*
 *  I2S_TFIFO_REG
 *
 */
#define TFIFO_TLC_MASK                  0xFFFF0000
#define TFIFO_TRC_MASK                  0x0000FFFF

/*
 *  I2S_RFIFO_REG
 *
 */
#define RFIFO_TLC_MASK                  0xFFFF0000
#define RFIFO_TRC_MASK                  0x0000FFFF

struct i2s_regs_s {
	volatile unsigned int AUDCTLCR;	/* [Rx00-03] Audio Controller Control Reigster*/
	volatile unsigned int AUDDFCR;	/* [Rx04-07] Audio Data Format Control Register*/
	volatile unsigned int TXCLKDIV;	/* [Rx08-0B] Clock Divisior of TX*/
	volatile unsigned int TXFRAMECR; /* [Rx0C-0F] offset of the data beginning per tx frame*/
	volatile unsigned int RXCLKDIV;	/* [Rx10-13] Clock Divisor of RX*/
	volatile unsigned int RXFRAMECR; /* [Rx14-17] offset of the data beginning per rx frame*/
	volatile unsigned int TXEQURXCR; /* [Rx18-1B] common setting of RX and TX*/
	volatile unsigned int SPCTLCR; /* [Rx1C-1F] SPDIF TX Control (only in APB system)*/
	volatile unsigned int TCR; /*[Rx20-23] Tx FIFO Control Register*/
	volatile unsigned int TSR; /*[Rx24-27] Tx FIFO Statis Register*/
	volatile unsigned int RCR; /*[Rx28-2B] Rx FIFO Control Register*/
	volatile unsigned int RSR; /*[Rx2C-2F] Rx FIFO Status Register*/
	volatile unsigned int Reserved[0x14]; /*[Rx30-7F]*/
	volatile unsigned int TFIFO;		/* [Rx80-83] Stereo Tx FIFO*/
	volatile unsigned int TFIFO_Alias[15]; /* [Rx84-BF] Stereo Tx FIFO Alias*/
	volatile unsigned int RFIFO;	 /* [RxC0-C3] Stereo Rx FIFO*/
	volatile unsigned int RFIFO_Alias[15]; /* [RxC4-FF] Stereo Rx FIFO Alias*/
};

struct i2s_ints_s {
	/* Tx FIFO Status. */
	unsigned int tfoe;      /* Tx FIFO Overrun Error */
	unsigned int tfue;      /* Tx FIFO Underrun Error */
	unsigned int tfa;       /* Tx FIFO Almost Empty */
	unsigned int tfe;       /* Tx FIFO Empty */

	/* Tx FIFO Status. */
	unsigned int rfoe;      /* Tx FIFO Overrun Error */
	unsigned int rfue;      /* Tx FIFO Underrun Error */
	unsigned int rfa;       /* Tx FIFO Almost Full */
	unsigned int rff;       /* Tx FIFO Full */

};

struct i2s_s {
	/* I2S Controller regsiter set.*/
	struct i2s_regs_s *const regs;    /* Register set*/
	/* Interrupt status counters.*/
	struct i2s_ints_s             ints;
	/* I2S Controller info. */
	const unsigned int      irq;            /* I2S controller irq*/
	unsigned int            ref;            /* I2S reference counter*/
	/* Basic handlers.*/
	void                    (*init)(void);
	void                    (*exit)(void);
};

/*
 * This ops was inherit form ac 97.
 * It need to reivse to meet i2s ops.
 *
 */
struct i2s_codec_ops_s {
	/* Generic operations */
	int	    (*init)(void);
	void	    (*exit)(void);
	int	    (*enable)(void);
	int	    (*disable)(void);
	/* CODEC I/O with interrupts */
	int	    (*read)(u16 addr, u32 *data);
	int	    (*write)(u16 addr, u32 data);
	int         (*set_sample_rate)(unsigned int sample_rate);
	int         (*set_out_volume)(unsigned int vol);
	int         (*set_in_volume)(unsigned int vol);
	int         (*select_recsrc)(unsigned int src);
	int         (*loopback_enable)(unsigned int en);
	int         (*mute_en)(unsigned int en);
};


struct i2s_codec_s {
	struct i2s_codec_ops_s *ops;
	/* IDs */
	unsigned short  id1;
	unsigned short  id2;
	/* audio generic properties */
	unsigned short  volume;
	unsigned short  bass;
	unsigned short  treble;
	unsigned short  line;
	unsigned short  mic;
	/* modified counter for mixer_ioctl. */
	unsigned int    mod;
	/* IRQ number. */
	unsigned int    irq;
	unsigned int 	recsrc;
};

/* codec shadow register*/
#define LINE_LEFT_VOLUME_ADDR          0x0
#define LINE_RIGHT_VOLUME_ADDR         0x1
#define HEADPHONE_LEFT_VOLUME_ADDR     0x2
#define HEADPHONE_RIGHT_VOLUME_ADDR    0x3
#define ANALOG_PATH_CONTROL_ADDR       0x4
#define DIGITAL_PATH_CONTROL_ADDR      0x5
#define POWERDOWN_CONTROL_ADDR         0x6
#define DIGITAL_FORMAT_ADDR            0x7
#define SAMPLE_RATE_CONTROL_ADDR       0x8
#define DIGITAL_ACTIVATION_ADDR        0x9
#define RESET_ADDR                     0xF

struct codec_shadow_regs_s {
	unsigned int line_left_volume_sreg ;
	unsigned int line_right_volume_sreg ;
	unsigned int headphone_left_volume_sreg ;
	unsigned int headphone_right_volume_sreg ;
	unsigned int analog_path_control_sreg ;
	unsigned int digital_path_control_sreg ;
	unsigned int powerdown_control_sreg ;
	unsigned int digital_format_sreg ;
	unsigned int sample_rate_control_sreg ;
	unsigned int digital_activation_sreg ;
};

struct codec_shadow_s {
	int ref;
	int spi_no  ;
	unsigned int  using_sample_rate ;
	int clock_out_half  ;
	struct codec_shadow_regs_s sreg ;
};

#define CODEC_POWER_ON 0x0001
#define CODEC_POWER_OFF 0x0000

void i2s_suspend(void);
void i2s_resume(void);
extern struct i2s_codec_s *i2s_codec_attach(void);
extern void i2s_codec_detach(struct i2s_codec_s *codec);
extern void i2s_sample_rate(unsigned int rate);
extern void i2s_rx_sample_rate(unsigned int rate);
extern void i2s_set_rx_resolution(unsigned int resolution);
extern void i2s_set_tx_resolution(unsigned int resolution);
extern void i2s_set_rx_channels(unsigned int channel);
extern void i2s_set_tx_channels(unsigned int channel);

#endif /* __WMT_I2S_H */
