/*++
	sound/arm/vt1613_generic.c

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
		2009/02/24 First Version
--*/
#include <linux/config.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sound.h>

#include <mach/hardware.h>

#include <mach/ac97.h>
#include <mach/vt1613.h>


/*
 *  Debug macros
 */
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#define vt1613_irq_lock        vt1613_lock

/*
 * local CODEC pointer
 */
static struct codec_s *vt1613_codec;

static DEFINE_SPINLOCK(vt1613_lock);

#define vt1613_lock_enable(flags) \
		spin_lock(&vt1613_lock); \
		flags = 0;
		/* spin_lock_irqsave(&vt1613_lock, flags);*/

#define vt1613_unlock_disable(flags) \
		spin_unlock(&vt1613_lock);
		/* spin_unlock_irqrestore(&vt1613_lock, flags);*/

#define vt1613_irq_lock_enable() \
		spin_lock(&vt1613_irq_lock);

#define vt1613_irq_lock_disable() \
		spin_unlock(&vt1613_irq_lock);

inline u16 vt1613_read_reg(u16 reg)
{
	u16 data = (u16)(-1);
	DPRINTK("vt1613_read_reg\n");

	vt1613_codec->ops->read(reg, &data);
	DPRINTK("vt1613_read_reg data = %x\n", data);

	return data;
}

inline void vt1613_write_reg(u16 reg, u16 data)
{
	vt1613_codec->ops->write(reg, data);
}

#ifdef CONFIG_PROC_FS

#if 0
struct proc_dir_entry *proc_vt1613;

static int vt1613_reg_read_proc(char *buf, char **start, off_t offset, int len)
{
	char *p = buf;
	int i;
	unsigned long flags;

	p += sprintf(p, "reg : value\n");

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i += 2)
		p += sprintf(p, "0x%.2x: 0x%.4x\n", i, vt1613_read_reg(i));

	vt1613_unlock_disable(flags);

	return p - buf;
}
#endif
#endif

void vt1613_reg_dump(void)
{
	int i;
	unsigned long flags;

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i += 2)
/*		DPRINTK("0x%.2x: 0x%.4x\n", i, vt1613_read_reg(i)); */
		printk("0x%.2x: 0x%.4x\n", i, vt1613_read_reg(i));

	vt1613_unlock_disable(flags);
}
/*
static u16 reg_vt1613_backup[0x7F];

void vt1613_reg_backup(void)
{
	unsigned short i;
	unsigned long flags;

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i++) {
		reg_vt1613_backup[i] = vt1613_read_reg(i);
		DPRINTK("vt1613_backup = %x\n", reg_vt1613_backup[i]);
	}

	vt1613_unlock_disable(flags);
}

void vt1613_reg_restore(void)
{
	int i;
	unsigned long flags;

	vt1613_lock_enable(flags);

	for (i = 0; i <= 0x7E; i++)
		vt1613_write_reg(i, reg_vt1613_backup[i]);

	vt1613_unlock_disable(flags);
}
*/

static u16 *vt1613_reg_ptr;

void vt1613_reg_backup(void)
{
	u16 *ptr;
	u16 size = 0x7E;
	int i;

	size += 2;
	ptr = (u16*) kmalloc(size,GFP_KERNEL);
	for(i=0;i<size;i+=2){
		ptr[i/2] = vt1613_read_reg(i);
	}
	vt1613_reg_ptr = ptr;
	return;
}

void vt1613_reg_restore(void)
{
	int i;
	u16 size = 0x7E;

	if( vt1613_reg_ptr == NULL )
		return;

	size += 2;
	for(i=0;i<size;i+=2){
		vt1613_write_reg(i, vt1613_reg_ptr[i/2]);
	}
	kfree(vt1613_reg_ptr);
	vt1613_reg_ptr = 0;
}
/*
 * VT1613 CODEC power management support for APM driver.
 */
#ifdef CONFIG_PM
#if 0
static int
vt1613_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{

	/*
	 * We just care about resume case, especially in the case
	 * of power-off suspend.
	 */
	/*
	if ( req == PM_SUSPEND )
		vt1613_codec->ops->disable();
	else
		vt1613_codec->ops->enable();
	*/

	return 0;
}
#endif
#endif

static int __init vt1613_init(void)
{

	vt1613_codec = codec_attach();

	if (!vt1613_codec)
		return -ENODEV;

	/*
	if ( vt1613_codec->ops->aclink )
		vt1613_codec->ops->init();
	*/
#ifdef CONFIG_PROC_FS
/*
	proc_vt1613 = proc_mkdir("driver/vt1613", NULL);
	proc_vt1613->owner = THIS_MODULE;
	create_proc_info_entry("registers", 0, proc_vt1613,
				vt1613_reg_read_proc);
*/
#endif

#ifdef CONFIG_PM
//	pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, vt1613_pm_callback);
#endif

#ifndef CONFIG_SKIP_DRIVER_MSG
	printk(KERN_INFO "VT1613 generic driver installed\n");
#endif
	return 0;
}

static void __exit vt1613_exit(void)
{
	free_irq(vt1613_codec->irq, NULL);

	vt1613_codec->ops->exit();

#ifdef CONFIG_PROC_FS
/*
	remove_proc_entry("io", proc_vt1613);
	remove_proc_entry("interrupts", proc_vt1613);
	remove_proc_entry("registers", proc_vt1613);
	remove_proc_entry("driver/vt1613", NULL);
*/
#endif

#ifdef CONFIG_PM
//	pm_unregister_all(vt1613_pm_callback);
#endif
}

module_init(vt1613_init);
module_exit(vt1613_exit);
EXPORT_SYMBOL(vt1613_read_reg);
EXPORT_SYMBOL(vt1613_write_reg);
EXPORT_SYMBOL(vt1613_reg_backup);
EXPORT_SYMBOL(vt1613_reg_restore);
//EXPORT_SYMBOL(proc_vt1613);

MODULE_AUTHOR("WMT SW Team");
MODULE_DESCRIPTION("WMT VT1613 Generic Driver");
MODULE_LICENSE("GPL");
