/*

	Some descriptions of such software.
    
	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.
	
	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.
	
	Authors: V1ron (Roman I. Volkov) (http://v1ron.ru), Russian software developer.
	
	Please refer to wmt-alsa.h file for more information.

	(c) 2011: Original version for some Chinese kernel.
	(c) 2015: Pushing this shit to the Projectgus kernel. Leaving
		my original comments as is to preserve this important
		historical artifact.
*/

#include "es8328_ctrl.h"

struct ES8328_CONTROL_FUNC *chip_func;

/* Control Callbacks */

/* Master Playback Volume 1 */
static int es8328_master_playback_volume_1_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
          uinfo->count = 2;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 33;
          return 0;
}

static int es8328_master_playback_volume_1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 46;
	buffer[2] = 47;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			//printk("mixer read values %d %d\n",buffer[0],buffer[2]);
			ucontrol->value.integer.value[0] = (int)buffer[0];
			ucontrol->value.integer.value[1] = (int)buffer[2];
		}
	}
	return status;
}

static int es8328_master_playback_volume_1_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	if ((*chip_func->snd_chip_write_reg_8)(46, (unsigned char)ucontrol->value.integer.value[0]) >= 0)
	{
		if ((*chip_func->snd_chip_write_reg_8)(47, (unsigned char)ucontrol->value.integer.value[1]) >= 0)
		{
			return 1;
		}
	}
	return 0;
}

static struct snd_kcontrol_new es8328_master_playback_volume_1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_master_playback_volume_1_info,
	.get = es8328_master_playback_volume_1_get,
	.put = es8328_master_playback_volume_1_put
};

/* Master Playback Switch 1 */

static int es8328_master_playback_switch_1_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
          uinfo->count = 2;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 1;
          return 0;
}

static int es8328_master_playback_switch_1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 39;
	buffer[2] = 42;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			ucontrol->value.integer.value[0] = (int)(buffer[0] >> 7) & 1;
			ucontrol->value.integer.value[1] = (int)(buffer[2] >> 7) & 1;
		}
	}
	return status;
}

static int es8328_master_playback_switch_1_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 39;
	buffer[2] = 42;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			buffer[0] &= 0b01111111;
			buffer[0] |= (unsigned char)((ucontrol->value.integer.value[0] << 7) & 0x80);
			buffer[2] &= 0b01111111;
			buffer[2] |= (unsigned char)((ucontrol->value.integer.value[1] << 7) & 0x80);
			if ((*chip_func->snd_chip_write_reg_8)(39, buffer[0]) >= 0)
			{
				if ((*chip_func->snd_chip_write_reg_8)(42, buffer[2]) >= 0)
				{
					return 1;
				}
			}
		}
	}
	return 0;
}


static struct snd_kcontrol_new es8328_master_playback_switch_1 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Switch",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_master_playback_switch_1_info,
	.get = es8328_master_playback_switch_1_get,
	.put = es8328_master_playback_switch_1_put
};

/* Master Playback Volume 2 */
static int es8328_master_playback_volume_2_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
          uinfo->count = 2;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 33;
          return 0;
}

static int es8328_master_playback_volume_2_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 48;
	buffer[2] = 49;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			//printk("mixer read values %d %d\n",buffer[0],buffer[2]);
			ucontrol->value.integer.value[0] = (int)buffer[0];
			ucontrol->value.integer.value[1] = (int)buffer[2];
		}
	}
	return status;
}

static int es8328_master_playback_volume_2_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	if ((*chip_func->snd_chip_write_reg_8)(48, (unsigned char)ucontrol->value.integer.value[0]) >= 0)
	{
		if ((*chip_func->snd_chip_write_reg_8)(49, (unsigned char)ucontrol->value.integer.value[1]) >= 0)
		{
			return 1;
		}
	}
	return 0;
}

static struct snd_kcontrol_new es8328_master_playback_volume_2 __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.index = 1,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_master_playback_volume_2_info,
	.get = es8328_master_playback_volume_2_get,
	.put = es8328_master_playback_volume_2_put
};


/* Master Mono Playback Volume */
static int es8328_master_mono_playback_volume_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
          uinfo->count = 1;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 33;
          return 0;
}

static int es8328_master_mono_playback_volume_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 50;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		ucontrol->value.integer.value[0] = (int)buffer[0];
	}
	return status;
}

static int es8328_master_mono_playback_volume_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if ((*chip_func->snd_chip_write_reg_8)(50, (unsigned char)ucontrol->value.integer.value[0]) >= 0)
	{
		return 1;
	}
	return 0;
}

static struct snd_kcontrol_new es8328_master_mono_playback_volume __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Mono Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_master_mono_playback_volume_info,
	.get = es8328_master_mono_playback_volume_get,
	.put = es8328_master_mono_playback_volume_put
};

/* Master Mono Playback Switch */

static int es8328_master_mono_playback_switch_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
          uinfo->count = 1;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 1;
          return 0;
}

static int es8328_master_mono_playback_switch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	// Reading value one of register (left & right to mono mix).
	int status;
	unsigned char buffer[4];
	buffer[0] = 43;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		ucontrol->value.integer.value[0] = (int)(buffer[0] >> 7) & 1;
	}
	return status;
}

static int es8328_master_mono_playback_switch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	// Writing one value to two regs
	int status;
	unsigned char buffer[4];
	buffer[0] = 43;
	buffer[2] = 44;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			buffer[0] &= 0b01111111;
			buffer[0] |= (unsigned char)((ucontrol->value.integer.value[0] << 7) & 0x80);
			buffer[2] &= 0b01111111;
			buffer[2] |= (unsigned char)((ucontrol->value.integer.value[0] << 7) & 0x80);
			if ((*chip_func->snd_chip_write_reg_8)(43, buffer[0]) >= 0)
			{
				if ((*chip_func->snd_chip_write_reg_8)(44, buffer[0]) >= 0)
				{
					return 1;
				}
			}
		}
	}

	return 0;
}


static struct snd_kcontrol_new es8328_master_mono_playback_switch __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Mono Playback Switch",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_master_mono_playback_switch_info,
	.get = es8328_master_mono_playback_switch_get,
	.put = es8328_master_mono_playback_switch_put
};


/* PCM Playback Volume */
static int es8328_pcm_playback_volume_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
          uinfo->count = 2;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 192;
          return 0;
}

static int es8328_pcm_playback_volume_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 26;
	buffer[2] = 27;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			//printk("mixer read values %d %d\n",buffer[0],buffer[2]);
			ucontrol->value.integer.value[0] = (int)192-buffer[0];
			ucontrol->value.integer.value[1] = (int)192-buffer[2];
		}
	}
	return status;
}

static int es8328_pcm_playback_volume_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	if ((*chip_func->snd_chip_write_reg_8)(26, (unsigned char)192-ucontrol->value.integer.value[0]) >= 0)
	{
		if ((*chip_func->snd_chip_write_reg_8)(27, (unsigned char)192-ucontrol->value.integer.value[1]) >= 0)
		{
			return 1;
		}
	}
	return 0;
}

static struct snd_kcontrol_new es8328_pcm_playback_volume __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "PCM Playback Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_pcm_playback_volume_info,
	.get = es8328_pcm_playback_volume_get,
	.put = es8328_pcm_playback_volume_put
};

/* Mic Capture Volume */
static int es8328_mic_capture_volume_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
          uinfo->count = 2;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 8;
          return 0;
}

static int es8328_mic_capture_volume_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 9;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		//printk("mixer read values %d %d\n",buffer[0],buffer[2]);
		ucontrol->value.integer.value[0] = (int)((buffer[0] >> 4) & 0b00001111);
		ucontrol->value.integer.value[1] = (int)(buffer[0] & 0b00001111);
	}
	return status;
}

static int es8328_mic_capture_volume_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	unsigned char value = ((unsigned char)ucontrol->value.integer.value[0] &0b00001111) << 4 |
		((unsigned char)ucontrol->value.integer.value[1] & 0b00001111);
	if ((*chip_func->snd_chip_write_reg_8)(9, value) >= 0)
	{
		return 1;
	}
	return 0;
}


static struct snd_kcontrol_new es8328_mic_capture_volume __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Mic Capture Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_mic_capture_volume_info,
	.get = es8328_mic_capture_volume_get,
	.put = es8328_mic_capture_volume_put
};

/* Capture Volume (global capture volume) */

static int es8328_capture_volume_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
          uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
          uinfo->count = 2;
          uinfo->value.integer.min = 0;
          uinfo->value.integer.max = 192;
          return 0;
}

static int es8328_capture_volume_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 16;
	buffer[2] = 17;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[2]);
		if (status >= 0)
		{
			//printk("mixer read values %d %d\n",buffer[0],buffer[2]);
			ucontrol->value.integer.value[0] = (int)192-buffer[0];
			ucontrol->value.integer.value[1] = (int)192-buffer[2];
		}
	}
	return status;
}

static int es8328_capture_volume_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	if ((*chip_func->snd_chip_write_reg_8)(16, (unsigned char)192-ucontrol->value.integer.value[0]) >= 0)
	{
		if ((*chip_func->snd_chip_write_reg_8)(17, (unsigned char)192-ucontrol->value.integer.value[1]) >= 0)
		{
			return 1;
		}
	}
	return 0;
}

static struct snd_kcontrol_new es8328_capture_volume __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Capture Volume",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_capture_volume_info,
	.get = es8328_capture_volume_get,
	.put = es8328_capture_volume_put
};

/* Mic Input Select */

static char *mic_select_names[4] = {
	"INPUT1","INPUT2","INPUT3","DIFFERENTIAL"
};

static int es8328_mic_select_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 4;
	if (uinfo->value.enumerated.item > 3)
	{
		uinfo->value.enumerated.item = 3;
	}
	strcpy(uinfo->value.enumerated.name,
                 mic_select_names[uinfo->value.enumerated.item]);

	return 0;
}

static int es8328_mic_select_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status;
	unsigned char buffer[4];
	buffer[0] = 10;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		ucontrol->value.enumerated.item[0] = (unsigned int)(buffer[0] >> 6) & 0b00000011;
	}
	return status;
}

static int es8328_mic_select_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned char buffer[4];
	int status;
	buffer[3] = ((unsigned char)ucontrol->value.enumerated.item[0]);
	buffer[3] = buffer[3] << 2;
	buffer[3] |= ((unsigned char)ucontrol->value.enumerated.item[0]);
	buffer[3] = buffer[3] << 4;
	buffer[0] = 10;
	status = (*chip_func->snd_chip_read_reg_8)(&buffer[0]);
	buffer[0] &= 0b00111111;
	buffer[0] |= (buffer[3] & 0b11000000);
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	if (status >= 0)
	{
		if ((*chip_func->snd_chip_write_reg_8)(10, buffer[0]) >= 0)
		{
			return 1;
		}
	}
	return 0;
}

static struct snd_kcontrol_new es8328_mic_select __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Mic Select",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_mic_select_info,
	.get = es8328_mic_select_get,
	.put = es8328_mic_select_put
};

/* Differential Select */

static char *diff_select_names[2] = {
	"L1-R1","L2-R2"
};

static int es8328_diff_select_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
	{
		uinfo->value.enumerated.item = 1;
	}
	strcpy(uinfo->value.enumerated.name,
                 diff_select_names[uinfo->value.enumerated.item]);

	return 0;
}

static int es8328_diff_select_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int status = 0;
	unsigned char buffer[4];
	buffer[0] = 11;
	status = (*chip_func->snd_chip_read_reg_8)((unsigned char*)&buffer[0]);
	if (status >= 0)
	{
		ucontrol->value.enumerated.item[0] = (unsigned int)(buffer[0] >> 7) & 0b00000001;
	}
	return status;
}

static int es8328_diff_select_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned char buffer[4];
	buffer[3] = ((unsigned char)ucontrol->value.enumerated.item[0]) << 7;
	buffer[0] = 11;
	//printk("mixer write value %d\n",ucontrol->value.integer.value[0]);
	if ((*chip_func->snd_chip_read_reg_8)(&buffer[0]) >= 0)
	{
		buffer[0] &= 0b01111111;
		buffer[0] |= (buffer[3] & 0b10000000);
		if ((*chip_func->snd_chip_write_reg_8)(11, buffer[0]) >= 0)
		{
			return 1;
		}
	}
	return 0;
}

static struct snd_kcontrol_new es8328_diff_select __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Diff Select",
	.index = 0,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0xffff,
	.info = es8328_diff_select_info,
	.get = es8328_diff_select_get,
	.put = es8328_diff_select_put
};

/* Initialization */

int snd_controls_new(struct snd_card *card, struct ES8328_CONTROL_FUNC *func)
{
	int status;
	status = snd_ctl_add(card, snd_ctl_new1(&es8328_master_playback_volume_1, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_master_playback_switch_1, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_master_playback_volume_2, NULL));
	//status |= snd_ctl_add(card, snd_ctl_new1(&es8328_master_playback_switch_2, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_master_mono_playback_volume, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_master_mono_playback_switch, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_pcm_playback_volume, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_mic_capture_volume, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_capture_volume, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_mic_select, NULL));
	status |= snd_ctl_add(card, snd_ctl_new1(&es8328_diff_select, NULL));
	chip_func = func;
	return status;
}

void snd_controls_delete(void)
{
	chip_func = NULL;
}

EXPORT_SYMBOL_GPL(snd_controls_new);
EXPORT_SYMBOL_GPL(snd_controls_delete);
