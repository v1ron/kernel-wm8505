/*
	sound/soc/wmt/wmt-1602-mixer.c
 	Alsa I2A Mixer Driver for VT1602 codecs for WMT chip
 
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
		2009/06    First Version

--*/

#include <linux/wait.h>
#include <sound/initval.h>
#include <sound/control.h>

#include <sound/soc.h>
#include <sound/asound.h>

#include <mach/i2s_alsa.h>
#include "vt1602.h"
#include "wmt-alsa-1602.h"

MODULE_AUTHOR("WMT");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WMT Alsa I2S Mixer Driver");

/*#define DEBUG*/
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

static DEFINE_MUTEX(vt1602_mutex);
/*
 * Codec dependent region
 */

extern int vt1602_write_reg(unsigned int index , u16 value);
extern u16 vt1602_read_reg(unsigned int index );

#define MIXER_NAME		     "Mixer VT1602"


/*
 * VT1602 Controls
 */
static const char *vt1602_alc_func[] = {"Off", "Right", "Left", "Stereo"};
static const char *vt1602_ng_type[] = {"Constant PGA Gain", "Mute ADC Output"};
static const char *vt1602_line_mux[] = {"Line 1", "Line 2", "Line 3", "PGA", "Differential"};
static const char *vt1602_pga_sel[] = {"Line 1", "Line 2", "Line 3", "Differential"};
static const char *vt1602_out3[] = {"VREF", "ROUT1 + Vol", "MonoOut", "ROUT1"};
static const char *vt1602_diff_sel[] = {"Line 1", "Line 2"};
static const char *vt1602_adcpol[] = {"Normal", "L Invert", "R Invert", "L + R Invert"};
static const char *vt1602_mono_mux[] = {"Stereo", "Mono (Left)", "Mono (Right)", "Digital Mono"};

static const struct soc_enum vt1602_enum[] = {
SOC_ENUM_SINGLE(VT1602_ALC1, 7, 4, vt1602_alc_func),
SOC_ENUM_SINGLE(VT1602_Noise_Gate, 1, 2, vt1602_ng_type),
SOC_ENUM_SINGLE(VT1602_Left_Out_Mix_1, 0, 5, vt1602_line_mux),
SOC_ENUM_SINGLE(VT1602_Right_Out_Mix_1, 0, 5, vt1602_line_mux),
SOC_ENUM_SINGLE(VT1602_ADCL_Signal_Path, 6, 4, vt1602_pga_sel), 
SOC_ENUM_SINGLE(VT1602_ADCR_Signal_Path, 6, 4, vt1602_pga_sel),
SOC_ENUM_SINGLE(VT1602_ADD_Control_2, 7, 4, vt1602_out3),
SOC_ENUM_SINGLE(VT1602_ADC_Input_Mode, 8, 2, vt1602_diff_sel),
SOC_ENUM_SINGLE(VT1602_ADC_DAC_Control, 5, 4, vt1602_adcpol),
SOC_ENUM_SINGLE(VT1602_ADC_Input_Mode, 6, 4, vt1602_mono_mux), /* 9 */

};


/* Callback Functions */
#define WMT_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_wmt_info_volsw_2r, \
	.get = snd_wmt_get_volsw_2r, .put = snd_wmt_put_volsw_2r, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
                {.reg = reg_left, .rreg = reg_right, .shift = xshift, \
                .max = xmax, .invert = xinvert} }

#define WMT_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_wmt_info_enum_double, \
	.get = snd_wmt_get_enum_double, .put = snd_wmt_put_enum_double, \
	.private_value = (unsigned long)&xenum }

#define WMT_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_wmt_info_volsw, .get = snd_wmt_get_volsw,\
	.put = snd_wmt_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

/* Begin Double Functions */
/**
 * snd_wmt_info_volsw_2r - double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double mixer control that
 * spans 2 codec registers.
 *
 * Returns 0 for success.
 */
static int snd_wmt_info_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        int max = mc->max;

        if (max == 1)
                uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
        else
                uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}


/**
 * snd_wmt_get_volsw_2r - double mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
static int snd_wmt_get_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        unsigned int reg = mc->reg;
        unsigned int reg2 = mc->rreg;
        unsigned int shift = mc->shift;
        int max = mc->max;
        unsigned int mask = (1<<fls(max))-1;
        unsigned int invert = mc->invert;

	ucontrol->value.integer.value[0] =
		(vt1602_read_reg( reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(vt1602_read_reg( reg2) >> shift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] =
			max - ucontrol->value.integer.value[1];
	}

	return 0;
}

/**
 * snd_wmt_update_bits - update codec register bits
 * @codec: audio codec
 * @reg: codec register
 * @mask: register mask
 * @value: new value
 *
 * Writes new register value.
 *
 * Returns 1 for change else 0.
 */
static int snd_wmt_update_bits( unsigned short reg, unsigned short mask, unsigned short value)
{
	int change;
	unsigned short old, new;

	mutex_lock(&vt1602_mutex);
	old = vt1602_read_reg( reg);
	new = (old & ~mask) | value;
	change = old != new;
	if (change)
		vt1602_write_reg( reg, new);

	mutex_unlock(&vt1602_mutex);
	return change;
}


/**
 * snd_wmt_put_volsw_2r - double mixer set callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
static int snd_wmt_put_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        unsigned int reg = mc->reg;
        unsigned int reg2 = mc->rreg;
        unsigned int shift = mc->shift;
        int max = mc->max;
        unsigned int mask = (1 << fls(max)) - 1;
        unsigned int invert = mc->invert;
        int err;
        unsigned short val, val2, val_mask;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	val = val << shift;
	val2 = val2 << shift;

	if ((err = snd_wmt_update_bits( reg, val_mask, val)) < 0)
		return err;

	err = snd_wmt_update_bits( reg2, val_mask, val2);
	return err;
}



/* End Double Functions */

/* Begin Enum Functions */
/**
 * snd_wmt_info_enum_double - enumerated double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double enumerated
 * mixer control.
 *
 * Returns 0 for success.
 */
static int snd_wmt_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = e->shift_l == e->shift_r ? 1 : 2;
	uinfo->value.enumerated.items = e->max;

	if (uinfo->value.enumerated.item > e->max - 1)
		uinfo->value.enumerated.item = e->max - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}

/**
 * snd_wmt_get_enum_double - enumerated double mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a double enumerated mixer.
 *
 * Returns 0 for success.
 */
static int snd_wmt_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned short val, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	val = vt1602_read_reg( e->reg);
	ucontrol->value.enumerated.item[0] = (val >> e->shift_l) & (bitmask - 1);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}

/**
 * snd_wmt_put_enum_double - enumerated double mixer put callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a double enumerated mixer.
 *
 * Returns 0 for success.
 */
static int snd_wmt_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned short val;
	unsigned short mask, bitmask;

	for (bitmask = 1; bitmask < e->mask; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return snd_wmt_update_bits( e->reg, mask, val);
}

/* End Enum Functions */

/* Begin Single Functions */
/**
 * snd_wmt_info_volsw - single mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a single mixer control.
 *
 * Returns 0 for success.
 */
static int snd_wmt_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        int max = mc->max;
        unsigned int shift = mc->shift;
        unsigned int rshift = mc->rshift;

        if (max == 1)
                uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
        else
                uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

/**
 * snd_wmt_get_volsw - single mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a single mixer control.
 *
 * Returns 0 for success.
 */
static int snd_wmt_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        unsigned int reg = mc->reg;
        unsigned int shift = mc->shift;
        unsigned int rshift = mc->rshift;
        int max = mc->max;
        unsigned int mask = (1 << fls(max)) - 1;
        unsigned int invert = mc->invert;


	ucontrol->value.integer.value[0] =
		(vt1602_read_reg( reg) >> shift) & mask;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
			(vt1602_read_reg( reg) >> rshift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		if (shift != rshift)
			ucontrol->value.integer.value[1] =
				max - ucontrol->value.integer.value[1];
	}

	return 0;
}

/**
 * snd_wmt_put_volsw - single mixer put callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a single mixer control.
 *
 * Returns 0 for success.
 */
static int snd_wmt_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        unsigned int reg = mc->reg;
        unsigned int shift = mc->shift;
        unsigned int rshift = mc->rshift;
        int max = mc->max;
        unsigned int mask = (1 << fls(max)) - 1;
        unsigned int invert = mc->invert;
        unsigned short val, val2, val_mask;
        int err;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert)
			val2 = max - val2;
		val_mask |= mask << rshift;
		val |= val2 << rshift;
	}
	err = snd_wmt_update_bits( reg, val_mask, val);
	return err;
}

/* End Double Functions */


static const struct snd_kcontrol_new vt1602_snd_controls[] = {

WMT_DOUBLE_R("Capture Volume", VT1602_Left_PGA_Volume, VT1602_Right_PGA_Volume, 0, 63, 0),
WMT_DOUBLE_R("Capture ZC Switch", VT1602_Left_PGA_Volume, VT1602_Right_PGA_Volume, 6, 1, 0),
WMT_DOUBLE_R("Capture Switch", VT1602_Left_PGA_Volume, VT1602_Right_PGA_Volume, 7, 1, 1),

WMT_DOUBLE_R("Headphone Playback ZC Switch", VT1602_LOUTA_Volume, VT1602_ROUTA_Volume, 7, 1, 0),
WMT_DOUBLE_R("Speaker Playback ZC Switch", VT1602_LOUTA_Volume, VT1602_ROUTA_Volume, 7, 1, 0),

WMT_ENUM("Capture Polarity", vt1602_enum[8]),
WMT_SINGLE("Playback 6dB Attenuate", VT1602_ADC_DAC_Control , 7, 1, 0),
WMT_SINGLE("Capture 6dB Attenuate", VT1602_ADC_DAC_Control , 8, 1, 0),

WMT_DOUBLE_R("PCM Volume", VT1602_Left_DAC_Gain, VT1602_Right_DAC_Gain, 0, 255, 0),

WMT_SINGLE("ALC Capture Target Volume", VT1602_ALC1, 0, 7, 0),
WMT_SINGLE("ALC Capture Max Volume", VT1602_ALC1, 4, 7, 0),
WMT_ENUM("ALC Capture Function", vt1602_enum[0]),
WMT_SINGLE("ALC Capture ZC Switch", VT1602_ALC2, 7, 1, 0),
WMT_SINGLE("ALC Capture Hold Time", VT1602_ALC2, 0, 15, 0),
WMT_SINGLE("ALC Capture Decay Time", VT1602_ALC3, 4, 15, 0),
WMT_SINGLE("ALC Capture Attack Time", VT1602_ALC3, 0, 15, 0),
WMT_SINGLE("ALC Capture NG Threshold", VT1602_Noise_Gate, 3, 31, 0),
WMT_SINGLE("ALC Capture NG Switch", VT1602_Noise_Gate, 0, 1, 0),

WMT_SINGLE("Left ADC Capture Volume", VT1602_Left_ADC_Volume, 0, 255, 0),
WMT_SINGLE("Right ADC Capture Volume", VT1602_Right_ADC_Volume, 0, 255, 0),

WMT_SINGLE("ZC Timeout Switch", VT1602_ADD_Control_1, 0, 1, 0),
WMT_SINGLE("Playback Invert Switch", VT1602_ADD_Control_1, 1, 1, 0),

WMT_SINGLE("Right Speaker Playback Invert Switch", VT1602_ADD_Control_2, 4, 1, 0),

/* Unimplemented */
/* ADCDAC Bit 0 - ADCHPD */
/* ADCDAC Bit 4 - HPOR */
/* ADCTL1 Bit 2,3 - DATSEL */
/* ADCTL1 Bit 4,5 - DMONOMIX */
/* ADCTL1 Bit 6,7 - VSEL */
/* ADCTL2 Bit 2 - LRCM */
/* ADCTL2 Bit 3 - TRI */
/* ADCTL3 Bit 5 - HPFLREN */
/* ADCTL3 Bit 6 - VROI */
/* ADCTL3 Bit 7,8 - ADCLRM */
/* ADCIN Bit 4 - LDCM */
/* ADCIN Bit 5 - RDCM */

WMT_DOUBLE_R("Mic Boost", VT1602_ADCL_Signal_Path, VT1602_ADCR_Signal_Path, 4, 3, 0),

WMT_DOUBLE_R("Bypass Left Playback Volume", VT1602_Left_Out_Mix_1,VT1602_Left_Out_Mix_2 , 4, 7, 1),
WMT_DOUBLE_R("Bypass Right Playback Volume", VT1602_Right_Out_Mix_1, VT1602_Right_Out_Mix_2, 4, 7, 1),
WMT_DOUBLE_R("Bypass Mono Playback Volume", VT1602_Mono_Out_Mix_1, VT1602_Mono_Out_Mix_2, 4, 7, 1),

WMT_SINGLE("Mono Playback ZC Switch", VT1602_OUT_MONO_Volume, 7, 1, 0),

WMT_DOUBLE_R("Headphone Playback Volume", VT1602_LOUTA_Volume,VT1602_ROUTA_Volume, 0, 127, 0),
WMT_DOUBLE_R("Speaker Playback Volume", VT1602_LOUTB_Volume, VT1602_ROUTB_Volume, 0, 127, 0),

WMT_SINGLE("Mono Playback Volume", VT1602_OUT_MONO_Volume, 0, 127, 0),

};

static DEFINE_SPINLOCK(vt1602_lock);
#define vt1602_lock_enable(flags) \
		spin_lock(&vt1602_lock); \
		flags = 0;

#define vt1602_unlock_disable(flags) \
		spin_unlock(&vt1602_lock);

void vt1602_reg_dump(void)
{
	int i;
	unsigned long flags;
	u16 data;

	vt1602_lock_enable(flags);

	for (i = 0; i <= 0x2A; i ++) {
		data = vt1602_read_reg(i );
		printk("0x%.2x: 0x%.4x\n", i, data);
	}

	vt1602_unlock_disable(flags);
}


static u16 reg_vt1602_backup[0x2A];
void vt1602_reg_backup(void)
{
	unsigned short i;
	unsigned long flags;

	vt1602_lock_enable(flags);

	for (i = 0; i <= 0x2A; i++) {
		reg_vt1602_backup[i] = vt1602_read_reg(i );
		DPRINTK("vt1602_backup = %x\n", reg_vt1602_backup[i]);
	}

	vt1602_unlock_disable(flags);
}


void vt1602_reg_restore(void)
{
	int i;
	unsigned long flags;

	vt1602_lock_enable(flags);

	for (i = 0; i <= 0x2A; i++) {
		DPRINTK("vt1602_backup = %x\n", reg_vt1602_backup[i]);
		vt1602_write_reg(i, reg_vt1602_backup[i]);
	}

	vt1602_unlock_disable(flags);
}

#ifdef CONFIG_PM
void snd_wmt_suspend_mixer(void)
{
	vt1602_reg_backup();
}

void snd_wmt_resume_mixer(void)
{
	vt1602_reg_restore();
}
#endif

void snd_wmt_init_mixer(void)
{
	/* Set default value of mixer here */
}
#if 0
/* add non dapm controls */
static int wm8750_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8750_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8750_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}
#endif
int snd_wmt_mixer(void *chip, struct snd_card *card)
{
	unsigned int idx;
	int err;

	if (snd_BUG_ON(chip != NULL && card != NULL))
	       return -EINVAL;

	strcpy(card->mixername, MIXER_NAME);

	/* Registering alsa mixer controls */
	for (idx = 0; idx < ARRAY_SIZE(vt1602_snd_controls); idx++) 
		if ((err = snd_ctl_add(card, snd_ctl_new1(&vt1602_snd_controls[idx], chip))) < 0)
			return err;

	return 0;
}
