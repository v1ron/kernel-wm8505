/*
 * sound/arm/wmt/wmt-alsa-1613.h
 * 
 * Alsa Driver for VT1613 codec WMT platform board
 *
 * Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * WonderMedia Technologies, Inc.
 * 10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
 *
 * History:
 *               2009/06    First Version
 */

#ifndef __WMT_ALSA_vt1613_H
#define __WMT_ALSA_vt1613_H


#define DEFAULT_OUTPUT_VOLUME		0x1F
#define MUTE_BIT_MASK					15		/* bit 15 for mute of Master, PCM, Linein and Mic volume*/
#define MIC_BOOST_BIT_MASK				6		/* bit 6 for mic boost of Mic, reg 0Eh*/
#define REC_SEL_MASK					0x7		/* 3 bits for recode select: bit10~8/bit2~0, reg 1Ah*/
#define REC_WITH_MIC					0		/* 000 for bit10~8/bit2~0, reg 1Ah */
#define REC_WITH_LINEIN				4		/* 100 for bit10~8/bit2~0, reg 1Ah */
#define REC_LEFT_CH_SHIFT				8		/* left channle value at bit10~8, reg 1Ah */
#define OUTPUT_VOLUME_MAX				0x1f	/* VT1613 volume value is from 0 to 31 */
#define INPUT_VOLUME_MAX				0x1f	/* VT1613 volume value is from 0 to 31 */
#define VOLUME_VALUE_MASK				0x1f	/* volume value need 5 bits: bit12~8, bit4~0 */
#define VOLUME_LEFT_CH_SHIFT			8		/* left channle value at bit12~8 */


int vt1613_configure(void);
void vt1613_set_dac_samplerate(long val);
void vt1613_set_adc_samplerate(long val);
int vt1613_enable(void);
int vt1613_disable(void);
int vt1613_get_default_samplerate(void);

#endif
