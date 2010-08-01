/*******************************************************************************
  linux/arch/arm/mach-wmt/regmon.c
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

  Module Name:

    $Workfile: regmon.c $

  Abstract:

    This program designed to be a monitor of VT8610 registers.
    It is just for debugging and testing purpose under the runtime kernel,
    but notice that I did not implement it with full supporting to all
    registers because there are some registers are not appropriate to show
    to users, such as registers with read to clean behavior.

  Usage:

    If you configured this driver as module, you have to load this module
    before using it, as following example:

    $ insmod regmon.o

    Following examples show how to simply control the read and write
    interfaces:

    $ cd /proc/vt8610/registers/
    $ cat PMC_008C
    $ 0x00000001
    $ echo 0x12345678 > PMC_008C
    $ cat PMC_008C
    $ 0x12345678

  Revision History:

    Apr.11.2007 First created by Richard Hsu.

    $JustDate: 05/12/14 $
    $Author: Admin $

*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>

#define NO_PARENT       NULL

#define MOD_NAME        "regmon"
#define SOC_NAME        "wmt"
#define REG_NAME        "registers"

#define RBUF_SIZE       128             /* 128 bytes should be enough */

struct soc_regs_s {
	u32 addr;
       char *name;
       char *desc;
       /*u16 low_ino;*/
       unsigned int low_ino;
       struct proc_dir_entry *entry;
};

static struct soc_regs_s wmt_regs[] =
{
	/* addr, name, desc */

	/*
	 * GPIO
	 */
	{ GPIO_ENABLE_CTRL_1_ADDR, "GPIO_0040", "GPIO Enable Control Register 01" },
	{ GPIO_ENABLE_CTRL_2_ADDR, "GPIO_0044", "GPIO Enable Control Register 02" },
	{ GPIO_ENABLE_CTRL_3_ADDR, "GPIO_0048", "GPIO Enable Control Register 03" },
	{ GPIO_ENABLE_CTRL_4_ADDR, "GPIO_004C", "GPIO Enable Control Register 04" },
	{ GPIO_ENABLE_CTRL_5_ADDR, "GPIO_0050", "GPIO Enable Control Register 05" },
	{ GPIO_ENABLE_CTRL_6_ADDR, "GPIO_0054", "GPIO Enable Control Register 06" },
	{ GPIO_ENABLE_CTRL_7_ADDR, "GPIO_0058", "GPIO Enable Control Register 07" },
	{ GPIO_ENABLE_CTRL_8_ADDR, "GPIO_005C", "GPIO Enable Control Register 08" },
	{ GPIO_ENABLE_CTRL_9_ADDR, "GPIO_0060", "GPIO Enable Control Register 09" },
	{ GPIO_ENABLE_CTRL_10_ADDR, "GPIO_0064", "GPIO Enable Control Register 10" },
	/* 0018-001F Reserved */

	{ GPIO_OUTPUT_ENABLE_1_ADDR, "GPIO_0068", "GPIO Output Control Register 01" },
	{ GPIO_OUTPUT_ENABLE_2_ADDR, "GPIO_006C", "GPIO Output Control Register 02" },
	{ GPIO_OUTPUT_ENABLE_3_ADDR, "GPIO_0070", "GPIO Output Control Register 03" },
	{ GPIO_OUTPUT_ENABLE_4_ADDR, "GPIO_0074", "GPIO Output Control Register 04" },
	{ GPIO_OUTPUT_ENABLE_5_ADDR, "GPIO_0078", "GPIO Output Control Register 05" },
	{ GPIO_OUTPUT_ENABLE_6_ADDR, "GPIO_007C", "GPIO Output Control Register 06" },
	{ GPIO_OUTPUT_ENABLE_7_ADDR, "GPIO_0080", "GPIO Output Control Register 07" },
	{ GPIO_OUTPUT_ENABLE_8_ADDR, "GPIO_0084", "GPIO Output Control Register 08" },
	{ GPIO_OUTPUT_ENABLE_9_ADDR, "GPIO_0088", "GPIO Output Control Register 09" },
	{ GPIO_OUTPUT_ENABLE_10_ADDR, "GPIO_008C", "GPIO Output Control Register 10" },

	{ GPIO_OUTPUT_DATA_1_ADDR, "GPIO_0090", "GPIO Output Data Register 01" },
	{ GPIO_OUTPUT_DATA_2_ADDR, "GPIO_0094", "GPIO Output Data Register 02" },
	{ GPIO_OUTPUT_DATA_3_ADDR, "GPIO_0098", "GPIO Output Data Register 03" },
	{ GPIO_OUTPUT_DATA_4_ADDR, "GPIO_009C", "GPIO Output Data Register 04" },
	{ GPIO_OUTPUT_DATA_5_ADDR, "GPIO_00A0", "GPIO Output Data Register 05" },
	{ GPIO_OUTPUT_DATA_6_ADDR, "GPIO_00A4", "GPIO Output Data Register 06" },
	{ GPIO_OUTPUT_DATA_7_ADDR, "GPIO_00A8", "GPIO Output Data Register 07" },
	{ GPIO_OUTPUT_DATA_8_ADDR, "GPIO_00AC", "GPIO Output Data Register 08" },
	{ GPIO_OUTPUT_DATA_9_ADDR, "GPIO_00B0", "GPIO Output Data Register 09" },
	{ GPIO_OUTPUT_DATA_10_ADDR, "GPIO_00B4", "GPIO Output Data Register 10" },

	{ GPIO_INPUT_DATA_1_ADDR, "GPIO_00B8", "GPIO Iutput Data Register 01" },
	{ GPIO_INPUT_DATA_2_ADDR, "GPIO_00BC", "GPIO Iutput Data Register 02" },
	{ GPIO_INPUT_DATA_3_ADDR, "GPIO_00C0", "GPIO Iutput Data Register 03" },
	{ GPIO_INPUT_DATA_4_ADDR, "GPIO_00C4", "GPIO Iutput Data Register 04" },
	{ GPIO_INPUT_DATA_5_ADDR, "GPIO_00C8", "GPIO Iutput Data Register 05" },
	{ GPIO_INPUT_DATA_6_ADDR, "GPIO_00CC", "GPIO Iutput Data Register 06" },
	{ GPIO_INPUT_DATA_7_ADDR, "GPIO_00D0", "GPIO Iutput Data Register 07" },
	{ GPIO_INPUT_DATA_8_ADDR, "GPIO_00D4", "GPIO Iutput Data Register 08" },
	{ GPIO_INPUT_DATA_9_ADDR, "GPIO_00D8", "GPIO Iutput Data Register 09" },
	{ GPIO_INPUT_DATA_10_ADDR, "GPIO_00DC", "GPIO Iutput Data Register 10" },

	{ GPIO_STRAP_STATUS_ADDR, "GPIO_0100", "GPIO Strapping Status Register" },
	{ GPIO_PMC_RTC_STATUS_ADDR, "GPIO_0104", "PMC Suspend RTC Clock Exist Status Register" },
	{ GPIO_AHB_CTRL_ADDR, "GPIO_0108", "GPIO AHB Control Register" },
	{ GPIO_SF_EDGE_CTRL_ADDR, "GPIO_010C", "SF Negative Edge Sampling Control Register" },
	{ GPIO_BINDING_STATUS_ADDR, "GPIO_0110", "Binding Option Status Register" },

	{ GPIO_PIN_SELECT_ADDR, "GPIO_0200", "GPIO Pin Selection Register" },
	{ GPIO_INT_REQ_TYPE_ADDR, "GPIO_0300", "GPIO Interrupt Request Type Register" },
	{ GPIO_INT_REQ_STATUS_ADDR, "GPIO_0304", "GPIO Interrupt Request Status Register" },

	{ GPIO_SD_STRENGTH_ADDR, "GPIO_0400", "Secure Digital I/O Drive Strength and Slew Rate Register" },
	{ GPIO_VID_STRENGTH_ADDR, "GPIO_0404", "VID I/O Drive Strength and Slew Rate Register" },
	{ GPIO_SPI_STRENGTH_ADDR, "GPIO_0408", "SPI I/O Drive Strength and Slew Rate Register" },
	{ GPIO_NOR_STRENGTH_ADDR, "GPIO_040C", "NOR I/O Drive Strength and Slew Rate Register" },
	{ GPIO_MMC_STRENGTH_ADDR, "GPIO_0410", "MMC I/O Drive Strength and Slew Rate Register" },
	{ GPIO_CLKTST_STRENGTH_ADDR, "GPIO_0414", "CLKTST I/O Drive Strength and Slew Rate Register" },
	{ GPIO_AC97_I2S_STRENGTH_ADDR, "GPIO_0418", "AC97/I2S I/O Drive Strength and Slew Rate Register" },

	{ GPIO_ENABLE_CTRL_11_ADDR, "GPIO_0500", "GPIO Enable Control Register 11" },
	{ GPIO_OUTPUT_ENABLE_11_ADDR, "GPIO_0504", "GPIO Output Control Register 11" },
	{ GPIO_OUTPUT_DATA_11_ADDR, "GPIO_0508", "GPIO Output Data Register 11" },
	{ GPIO_INPUT_DATA_11_ADDR, "GPIO_050C", "GPIO Iutput Data Register 11" },

	/* Add more registers depend on you */
	/* ..... */
};

#define NR_wmt_REGS    (sizeof(wmt_regs)/sizeof(struct soc_regs_s))

static int proc_read_reg(struct file *file, char *buf,
		size_t nbytes, loff_t *ppos)
{
	char output[RBUF_SIZE];
	int i, count = 0;
	int i_ino = (file->f_dentry->d_inode)->i_ino;

	struct soc_regs_s *p = NULL;

	if (*ppos > 0)        /* Assume reading completed in previous read */
		return 0;

	/*
	* Get the register entry.
	*/
	for (i = 0; i < NR_wmt_REGS; i++) {
		if (wmt_regs[i].low_ino == i_ino) {
			p = &wmt_regs[i];
			break;
		}
	}

	if (p == NULL)
		return -EINVAL;

	count += sprintf(output, "0x%.8x\n", REG32_VAL(p->addr));

#ifdef CONFIG_WMT_REGMON_RF_B
	count += sprintf((output + count), "%s\n", p->desc);
#endif

	*ppos += count;

	if (count > nbytes)   /* Assume output can be read at one time */
		return -EINVAL;

	if (copy_to_user(buf, output, count))
		return -EFAULT;

	return count;
}

#ifdef CONFIG_WMT_REGMON_WF
/*
 * Write interface
 */
static ssize_t proc_write_reg(struct file *file, const char *buffer,
		size_t count, loff_t *ppos)
{
	char *endp;
	int i;
	unsigned long new;
	int i_ino = (file->f_dentry->d_inode)->i_ino;
	ssize_t result;
	struct soc_regs_s *p = NULL;

	/*
	* Get the register entry.
	*/
	for (i = 0; i < NR_wmt_REGS; i++) {
		if (wmt_regs[i].low_ino == i_ino) {
			p = &wmt_regs[i];
			break;
		}
	}

	if (p == NULL)
		return -EINVAL;

	/*
	* Convert input to new value;
	*/
	new = simple_strtoul(buffer, &endp, 0);

	/*
	* Now assign new value to register.
	*/
	REG32_VAL(p->addr) = new;
	result = count + endp - buffer;
	return result;
}

/*
 * Provide both read and write interfaces
 */
static struct file_operations reg_fops = {
	.read   = proc_read_reg,
	.write  = proc_write_reg,
};
#else
/*
 * Provide only read interface
 */
static struct file_operations reg_fops = {
	.read   = proc_read_reg,
};
#endif

static struct proc_dir_entry *proc_soc;
static struct proc_dir_entry *proc_reg;

static int __init monitor_init(void)
{
	int i, err = 0;
	struct soc_regs_s *p = NULL;

	/*
	* Make /proc/wmt directory.
	*/
	proc_soc = proc_mkdir("wmt", &proc_root);

	if (proc_soc == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR MOD_NAME ": can't create /proc/" SOC_NAME "\n");
		goto monitor_err;
	}

	proc_soc->owner = THIS_MODULE;

	/*
	* Make /proc/wmt/registers directory.
	*/
	proc_reg = proc_mkdir(REG_NAME, proc_soc);

	if (proc_reg == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR MOD_NAME ": can't create /proc/" SOC_NAME \
			"/" REG_NAME "\n");
		goto monitor_err;
	}

	/*
	* Make /proc/wmt/registers/XXXX entries.
	*/
	for (i = 0, p = &wmt_regs[0]; i < NR_wmt_REGS; i++, p++) {
		p->entry = NULL;
		p->entry = create_proc_entry(p->name,
			S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH,
			proc_reg);

		if (p->entry == NULL) {
			err = -ENOMEM;
			printk(KERN_ERR MOD_NAME \
			": can't create /proc/" SOC_NAME \
			"/" REG_NAME "/%s\n", p->name);
			goto monitor_err;
		}

		p->low_ino = p->entry->low_ino;
		p->entry->proc_fops = &reg_fops;
	}

	return err;

monitor_err:
	/*
	* Free all allocated entries if we got error.
	*/
	for (i = 0, p = &wmt_regs[0]; i < NR_wmt_REGS; i++, p++) {
		if (p->entry != NULL) {
			remove_proc_entry(p->name, proc_reg);
			p->entry = NULL;
		}
	}

	remove_proc_entry(REG_NAME, proc_soc);
	remove_proc_entry(SOC_NAME, &proc_root);

	proc_reg = NULL;
	proc_soc = NULL;

	return err;
}

static void __exit monitor_exit(void)
{
	int i;
	struct soc_regs_s *p = NULL;

	/*
	* Free all allocated entries if we got error.
	*/
	for (i = 0, p = &wmt_regs[0]; i < NR_wmt_REGS; i++, p++) {
		if (p->entry != NULL) {
			remove_proc_entry(p->name, proc_reg);
			p->entry = NULL;
		}
	}

	remove_proc_entry(REG_NAME, proc_soc);
	remove_proc_entry(SOC_NAME, &proc_root);

	proc_reg = NULL;
	proc_soc = NULL;
}

module_init(monitor_init);
module_exit(monitor_exit);

MODULE_AUTHOR("VIA RISC & DSP SW Team");
MODULE_DESCRIPTION("wmt registers monitor driver");
MODULE_LICENSE("GPL");
