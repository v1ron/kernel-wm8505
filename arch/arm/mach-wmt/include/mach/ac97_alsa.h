/*++
	linux/include/asm-arm/arch-wmt/ac97_alsa.h

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
                        The code was inherit from vt8430
--*/

#ifndef __AC97_CODEC
#define __AC97_CODEC

#include <linux/platform_device.h>
//#include <sound/driver.h>
#include <sound/pcm.h>
#include <sound/core.h>

/*
 * AC'97 registers set structure.
 */
struct ac97_regs_s {
	unsigned int volatile ACGC;
	unsigned int volatile ACGS;
	unsigned int volatile CCSL;
	unsigned int volatile CCMD;
	unsigned int volatile CRSD;
	unsigned int volatile REV0[(0x0020-0x0014)/4];  /* 3 words reserved  */
	unsigned int volatile PTFC;
	unsigned int volatile PTFS;
	unsigned int volatile PRFC;
	unsigned int volatile PRFS;
	unsigned int volatile MIFC;
	unsigned int volatile MIFS;
	/*
	 * Due to issue of read FIFO, following section should not use.
	 */
/*        unsigned int volatile REV1[(0x0080-0x0038)/4];   18 words reserved */ 
/*        unsigned int volatile PTFP[(0x00C0-0x0080)/4];   16 words FIFO     */ 
/*        unsigned int volatile PRFP[(0x0100-0x00C0)/4];   16 words FIFO     */ 
/*        unsigned int volatile MIRP[(0x0140-0x0100)/4];   16 words FIFO     */ 
};

/*
 * AC'97 interrupt event counters.
 */
struct ac97_ints_s {
	/*
	 * Global Status.
	 */
	unsigned int crdy;      /* CODEC Ready Interrupt */
	unsigned int cwd;       /* CODEC Write Done Interrupt */
	unsigned int crd;       /* CODEC Read Done Interrupt */
	unsigned int cst;       /* CODEC Status Timeout */

	/*
	 * PCM Tx FIFO Status.
	 */
	unsigned int ptfe;      /* PCM Tx FIFO Empty */
	unsigned int ptfa;      /* PCM Tx FIFO Almost Empty */
	unsigned int ptfue;     /* PCM Tx FIFO Underrun Error */
	unsigned int ptfoe;     /* PCM Tx FIFO Overrun Error */

	/*
	 * PCM Rx FIFO Status.
	 */
	unsigned int prff;      /* PCM Rx FIFO Full */
	unsigned int prfa;      /* PCM Rx FIFO Almost Full */
	unsigned int prfue;     /* PCM Rx FIFO Underrun Error */
	unsigned int prfoe;     /* PCM Rx FIFO Overrun Error */

	/*
	 * Mic FIFO Status.
	 */
	unsigned int mff;       /* Mic FIFO Full */
	unsigned int mfa;       /* Mic FIFO Almost Full */
	unsigned int mfue;      /* Mic FIFO Underrun Error */
	unsigned int mfoe;      /* Mic FIFO Overrun Error */

};

typedef struct wmt_ac97_s {
	struct ac97_regs_s *const regs;    /* AC'97 Controller regsiter set. */
	struct ac97_ints_s ints;	/* Interrupt status counters. */
	/** AC'97 Controller info. */
	const unsigned int irq;      /* AC'97 controller irq     */
	unsigned int ref;            /* AC'97 reference counter  */
	/** Basic handlers. */
	void (*init)(void);
	void (*exit)(void);
	char *name;
	struct snd_pcm_hw_constraint_list *hw_constraints_rates;
	struct snd_pcm_hardware *snd_wmt_alsa_playback;
	struct snd_pcm_hardware *snd_wmt_alsa_capture;
} wmt_ac97_t;


/*
 * CODEC status flags.
 */
#define CODEC_INITED            (1 << 0)

struct codec_ops_s {
	/* AC-link status */
	int         (*aclink)(void);
	/* Generic operations */
	int	        (*init)(void);
	void	    (*exit)(void);
	int	        (*enable)(void);
	int	        (*disable)(void);
	/* CODEC I/O with AC'97 interrupts */
	int	        (*read)(u16 addr, u16 *data);
	int	        (*write)(u16 addr, u16 data);
	/* CODEC I/O without AC'97 interrupts */
	u16         (*bread)(u16 addr);
	void        (*bwrite)(u16 addr, u16 data);

};

struct codec_s {
	struct codec_ops_s *ops;
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

struct wmt_alsa_codec_config {
	char 	*name;
        struct snd_pcm_hw_constraint_list *hw_playback_constraints_rates;
        struct snd_pcm_hw_constraint_list *hw_capture_constraints_rates;
	struct snd_pcm_hardware *snd_wmt_alsa_playback;
	struct snd_pcm_hardware *snd_wmt_alsa_capture;
	struct codec_s *codec;
	int (*codec_configure)(void);
	void (*codec_set_dac_samplerate)(long);
	void (*codec_set_adc_samplerate)(long);
	int	(*get_default_samplerate)(void);
};

/*********** Mixer function prototypes *************************/
int snd_wmt_mixer(void *, struct snd_card *);
void snd_wmt_init_mixer(void);

extern struct codec_s *codec_attach(void);
extern void codec_detach(struct codec_s *codec);
extern void ac97_reg_backup(void);
extern void ac97_reg_restore(void);

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

#endif  /* __AC97_CODEC */
