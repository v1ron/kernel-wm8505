/*++
	linux/include/asm-arm/arch-wmt/ac97.h

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

#ifndef __I2S_ALSA_H
#define __I2S_ALSA_H

#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/core.h>


#if 0
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
#endif



struct wmt_alsa_codec_config {
	char 	*name;
	struct snd_pcm_hw_constraint_list *hw_constraints_rates;
	struct snd_pcm_hardware *snd_wmt_alsa_playback;
	struct snd_pcm_hardware *snd_wmt_alsa_capture;
	struct i2s_codec_s *codec;
	int (*codec_configure)(void);
	void (*codec_set_dac_samplerate)(long);
	void (*codec_set_adc_samplerate)(long);
/*
	void	(*codec_clock_setup)(void);
	int	(*codec_clock_on)(void);
	int 	(*codec_clock_off)(void);
*/
	int	(*get_default_samplerate)(void);
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

/*struct codec_shadow_regs_s {
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
};*/

/*struct codec_shadow_s {
	int ref;
	int spi_no  ;
	unsigned int  using_sample_rate ;
	int clock_out_half  ;
	struct codec_shadow_regs_s sreg ;
};*/

#define CODEC_POWER_ON 0x0001
#define CODEC_POWER_OFF 0x0000

//void i2s_suspend(void);
//void i2s_resume(void);
//extern struct i2s_codec_s *i2s_codec_attach(void);
//extern void i2s_codec_detach(struct i2s_codec_s *codec);
//extern void i2s_sample_rate(unsigned int rate);
//extern void i2s_rx_sample_rate(unsigned int rate);
//extern void i2s_set_rx_resolution(unsigned int resolution);
//extern void i2s_set_tx_resolution(unsigned int resolution);
//extern void i2s_set_rx_channels(unsigned int channel);
//extern void i2s_set_tx_channels(unsigned int channel);

/*********** Mixer function prototypes *************************/
int snd_wmt_mixer(void *, struct snd_card *);
void snd_wmt_init_mixer(void);


extern int snd_wmt_alsa_post_probe(struct platform_device *pdev, struct wmt_alsa_codec_config *config);
extern int snd_wmt_alsa_remove(struct platform_device *pdev);

#ifdef CONFIG_PM
int snd_wmt_alsa_suspend(struct platform_device *pdev, pm_message_t state);
int snd_wmt_alsa_resume(struct platform_device *pdev);
void snd_wmt_suspend_mixer(void);
void snd_wmt_resume_mixer(void);
#else
#define snd_wmt_alsa_suspend	NULL
#define snd_wmt_alsa_resume	NULL
#define snd_wmt_suspend_mixer	NULL
#define snd_wmt_resume_mixer	NULL
#endif

#endif  /* ___I2S_ALSA_H_*/
