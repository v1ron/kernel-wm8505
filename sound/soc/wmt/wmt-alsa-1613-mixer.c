/*
 *      sound/soc/wmt/wmt-alsa-1613-mixer.c
 *       Alsa AC97 Mixer Driver for VT1613 codecs for WMT chip
 *
 *       Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.
 *
 *       This program is free software: you can redistribute it and/or modify it under the
 *       terms of the GNU General Public License as published by the Free Software Foundation,
 *       either version 2 of the License, or (at your option) any later version.
 *
 *       This program is distributed in the hope that it will be useful, but WITHOUT
 *       ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *       PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *       You should have received a copy of the GNU General Public License along with
 *       this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *       WonderMedia Technologies, Inc.
 *       10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
 *
 *       History:
 *                2009/06    First Version
 * 
 * 
 * Alsa Driver Mixer for 1613 codecs for WMT chip
 */

#include <linux/wait.h>
#include <sound/initval.h>
#include <sound/control.h>

#include <mach/ac97_alsa.h>
#include <mach/vt1613.h>
#include "wmt-alsa-1613.h"

MODULE_AUTHOR("WMT");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WMT Alsa AC97 Mixer Driver");

/*#define DEBUG*/
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

/*
 * Codec dependent region
 */

extern int vt1613_codec_read(u16 , u16 *);
extern int vt1613_codec_write(u16, u16);

#define MIXER_NAME		     "Mixer VT1613"

/* Callback Functions */
#define WMT_BOOL(xname, xindex, reg, mask) \
{ \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_wmt_info_bool, \
	.get = snd_wmt_get_bool, \
	.put = snd_wmt_put_bool, \
	.private_value = reg  | (mask<< 16) \
}

#define WMT_MUX(xname, reg) \
{ \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.info = snd_wmt_info_mux, \
	.get = snd_wmt_get_mux, \
	.put = snd_wmt_put_mux, \
	.private_value = reg \
}

#define WMT_DOUBLE(xname, xindex, reg) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = xname, \
	.index = xindex, \
	.info = snd_wmt_info_double, \
	.get = snd_wmt_get_double, \
	.put = snd_wmt_put_double, \
	.private_value = reg \
}


/* Begin Bool Functions */

static int snd_wmt_info_bool(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	
	return 0;
}

static int snd_wmt_get_bool(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	u16 reg, val, mask;
	int err;
	
	reg = kcontrol->private_value & 0xff;
	err = vt1613_codec_read(reg, &val);
	if ( err ) {
		printk("snd_wmt_get_bool: read codec error, %d\n", err);
		return err;
	}
	mask = (kcontrol->private_value >> 16) & 0xf;	/* 4 bits for mask value: 15 for mute, 6 for mic boost  */	  ucontrol->value.integer.value[0] = (~(val>>mask))&0x1;  /* 1=mute=disable, 0=unmute=enable */
        DPRINTK("reg=0x%x, mask=%u, value=%d, value=0x%x\n", val, mask, ucontrol->value.integer.value[0], ucontrol->value.integer.value[0]);
	
	return 0;
}

static int snd_wmt_put_bool(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	u16 reg, val, mask, newval;
	int changed = 1, err;

	reg = kcontrol->private_value & 0xff;
	err = vt1613_codec_read(reg, &val);
	if ( err ) {
		printk("snd_wmt_put_bool: read codec error, %d\n", err);
		changed = 0;
		goto put_bool_fail;
	}
	mask = (kcontrol->private_value >> 16) & 0xf;
	newval = val & (~(1<<mask));
	newval |= ((ucontrol->value.integer.value[0]&0x1)^0x1) << mask;
	err = vt1613_codec_write(reg, newval);
	if ( err ) {
		printk("snd_wmt_put_bool: write codec error, %d\n", err);
		changed = 0;
	}
	DPRINTK("reg=0x%x, value=%d, newval=0x%x\n", val, ucontrol->value.integer.value[0], newval);

put_bool_fail:
	return changed;
}

/* End Bool Functions */

/* Begin Mux Functions */

static int snd_wmt_info_mux(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	static char *texts[2] =	{ "Mic", "Line"	};		/* Mic = 0; Line = 1 */

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	
	strcpy(uinfo->value.enumerated.name, texts[uinfo->value.enumerated.item]);
	
	return 0;
}

static int snd_wmt_get_mux(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	u16 reg, val;
	int err;

	reg = kcontrol->private_value & 0xff;
	err = vt1613_codec_read(reg, &val);
	if ( err ) {
		printk("snd_wmt_get_mux: read codec error, %d\n", err);
		return err;
	}

	val = (val>>REC_LEFT_CH_SHIFT) & REC_SEL_MASK;	/* Currently, we suppose the settings are same of left & right channel */
	switch(val) {
	case REC_WITH_MIC:
		ucontrol->value.enumerated.item[0] = 0 /* Mic */;
		break;
	case REC_WITH_LINEIN:
		ucontrol->value.enumerated.item[0] = 1 /* Line */;
		break;
	default:
		break;
	}
	
	return 0;
}

static int snd_wmt_put_mux(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	u16 reg, newval;
	int changed=1, err;

	reg = kcontrol->private_value & 0xff;
	/*** do not need to read old value
	if ( err=vt1613_codec_read(reg, &val) ) {
		printk("snd_wmt_put_mux: read codec error, %d\n", err);
		changed = 0;
		goto put_mux_fail;
	}
	*/
	switch ( ucontrol->value.enumerated.item[0] ) {
	case 1:
		newval = REC_WITH_LINEIN;
		break;
	default:
		newval = REC_WITH_MIC;
		break;

	}
	/* Currently, we use the same setting for left and right channel for recording */
	newval = newval | (newval << REC_LEFT_CH_SHIFT);	/* combine left and right settings */
	err = vt1613_codec_write(reg, newval);
	if ( err ) {
		printk("snd_wmt_put_mux: write codec error, %d\n", err);
		changed = 0;
	}

	return changed;
}

/* End Mux Functions */

/* Begin Double Functions */

static int snd_wmt_info_double(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = VOLUME_VALUE_MASK;
	
	return 0;
}

static int snd_wmt_get_double(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	u16 reg, val;
	int err;

	reg = kcontrol->private_value & 0xff;
	err = vt1613_codec_read(reg, &val);
	if ( err ) {
		printk("snd_wmt_get_double: read codec error, %d\n", err);
		return err;
	}
	ucontrol->value.integer.value[0] =  (val>>VOLUME_LEFT_CH_SHIFT) & VOLUME_VALUE_MASK;	/* left channel */
	ucontrol->value.integer.value[1] = val & VOLUME_VALUE_MASK;		/* right channel */

	return 0;
}

static int snd_wmt_put_double(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	u16 reg, val, left_val, right_val, newval;
	int changed=1, err;

	reg = kcontrol->private_value & 0xff;
	err = vt1613_codec_read(reg, &val);
	if ( err ) {
		printk("snd_wmt_get_double: read codec error, %d\n", err);
		changed = 0;
		goto put_double_fail;
	}
	left_val = (ucontrol->value.integer.value[0] & VOLUME_VALUE_MASK) << VOLUME_LEFT_CH_SHIFT;
	right_val = ucontrol->value.integer.value[1] & VOLUME_VALUE_MASK;
	newval = (val & (1<<MUTE_BIT_MASK)) | left_val | right_val;
	err = vt1613_codec_write(reg, newval);
	if ( err ) {
		printk("snd_wmt_put_double: write codec error, %d\n", err);
		changed = 0;
	}

put_double_fail:
	return changed;
}

/* End Double Functions */

static struct snd_kcontrol_new snd_wmt_controls[] = {
	WMT_BOOL("Master Playback Switch", 0, VTAC_MASV, MUTE_BIT_MASK),	//bit15 for mute
	WMT_DOUBLE("Master Playback Volume", 0, VTAC_MASV),
/*	WMT_DOUBLE("Master Balance Playback Volume", ), */
	WMT_BOOL("PCM Playback Switch", 0, VRAC_PCMV, MUTE_BIT_MASK),	//18h reg: PCM Output Volume, bit15 for mute
/*	WMT_DOUBLE("PCM Balance Playback Volume", ), */
	WMT_DOUBLE("PCM Playback Volume", 0, VRAC_PCMV),
	WMT_BOOL("Line Capture Switch", 0, VTAC_LINV, MUTE_BIT_MASK),	//10h reg: Line In Volume, bit15 for mute
	WMT_DOUBLE("Line Capture Volume", 0, VTAC_LINV),
	/*Stereo Micphone is not supported yet */
	WMT_BOOL("Mic Capture Switch", 0, VTAC_MICV, MUTE_BIT_MASK),	//0Eh reg: Mic In Volume, bit15 for mute
	WMT_BOOL("Mic Booster Capture Switch", 0, 0x0e, MIC_BOOST_BIT_MASK),	//0Eh reg: Mic In Volume, bit6 for boost
	WMT_DOUBLE("Mic Capture Volume", 0, VTAC_MICV),	
	WMT_MUX("Capture Source", VTAC_RECS),
};


static DEFINE_SPINLOCK(vt1613_lock);
#define vt1613_lock_enable(flags) \
		spin_lock(&vt1613_lock); \
		flags = 0;

#define vt1613_unlock_disable(flags) \
		spin_unlock(&vt1613_lock);

void vt1613_reg_dump(void)
{
	int i;
	unsigned long flags;
	u16 data;

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i += 2) {
		vt1613_codec_read(i, &data);
		printk("0x%.2x: 0x%.4x\n", i, data);
	}

	vt1613_unlock_disable(flags);
}


static u16 reg_vt1613_backup[0x7F];
void vt1613_reg_backup(void)
{
	unsigned short i;
	unsigned long flags;

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i++) {
		vt1613_codec_read(i, &reg_vt1613_backup[i]);
		DPRINTK("vt1613_backup = %x\n", reg_vt1613_backup[i]);
	}

	vt1613_unlock_disable(flags);
}


void vt1613_reg_restore(void)
{
	int i;
	unsigned long flags;

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i++) {
		DPRINTK("vt1613_backup = %x\n", reg_vt1613_backup[i]);
		vt1613_codec_write(i, reg_vt1613_backup[i]);
	}

	vt1613_unlock_disable(flags);
}

#ifdef CONFIG_PM
void snd_wmt_suspend_mixer(void)
{
	vt1613_reg_backup();
}

void snd_wmt_resume_mixer(void)
{
	vt1613_reg_restore();
}
#endif

void snd_wmt_init_mixer(void)
{
	/* Set default value of mixer here */
}

int snd_wmt_mixer(void *chip, struct snd_card *card)
{
	unsigned int idx;
	int err;

	if (snd_BUG_ON(chip != NULL && card != NULL))
	       return -EINVAL;

	strcpy(card->mixername, MIXER_NAME);

	/* Registering alsa mixer controls */
	for (idx = 0; idx < ARRAY_SIZE(snd_wmt_controls); idx++) 
		if ((err = snd_ctl_add(card, snd_ctl_new1(&snd_wmt_controls[idx], chip))) < 0)
			return err;

	return 0;
}
