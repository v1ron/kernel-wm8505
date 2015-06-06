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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/soundcard.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <mach/gpio_if.h>
#include <linux/spinlock.h>
#include <sound/control.h>

int snd_wmt_mixer(void *chip, struct snd_card *card);
