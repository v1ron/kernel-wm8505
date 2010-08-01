/*++
	Copyright (c) 2008  WonderMedia Technologies, Inc.   All Rights Reserved.

	This PROPRIETARY SOFTWARE is the property of WonderMedia Technologies, Inc.
	and may contain trade secrets and/or other confidential information of
	WonderMedia Technologies, Inc. This file shall not be disclosed to any third
	party, in whole or in part, without prior written consent of WonderMedia.

	THIS PROPRIETARY SOFTWARE AND ANY RELATED DOCUMENTATION ARE PROVIDED AS IS,
	WITH ALL FAULTS, AND WITHOUT WARRANTY OF ANY KIND EITHER EXPRESS OR IMPLIED,
	AND WonderMedia TECHNOLOGIES, INC. DISCLAIMS ALL EXPRESS OR IMPLIED WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR
	NON-INFRINGEMENT.

	Module Name:

		$Workfile: post_nand.c $

	Abstract:

		POST functions, called by main().

	Revision History:

		Dec.04.2008 First created
		Dec.19.2008 Dannier change coding style and support spi flash boot with nor accessible.
	$JustDate: 2008/12/19 $
--*/

#include <linux/config.h>
#include <linux/module.h>
/*#include <linux/types.h>*/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/ioport.h>
/*#include <linux/platform_device.h>*/
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
/*#include <linux/clk.h>*/

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
/*#include <linux/mtd/partitions.h>*/

#include <asm/io.h>
#include <asm/dma.h>
#include <asm/sizes.h>

#include <mach/hardware.h>
#include "vt8500_nand.h"


#ifndef memzero
#define memzero(s, n)     memset ((s), 0, (n))
#endif

/*
* NAND_PAGE_SIZE = 2048 or 4096:
* support Harming ECC and BCH ECC
*
* NAND_PAGE_SIZE = 512:
* Only support Harming ECC according to the demand of filesystem
* so need open macro:
* #define NAND_HARMING_ECC
*
*/

/*#define NAND_DEBUG*/
unsigned int vt8500_version;
#ifdef CONFIG_MTD_PARTITIONS
#include <linux/mtd/partitions.h>

struct mtd_partition nand_partitions[] = {
/*	{
		.name		= "filesystem-NAND",
		.offset		= 0x00000000,
		.size		= MTDPART_SIZ_FULL,
	},
*/
	{
		.name		= "kernel-NAND",
		.offset		= 0x00000000,
		.size		= 0x00300000,
	},
	{
		.name		= "filesystem-NAND",
		.offset		= 0x00300000,
		//.size		= 0x00300000,
		.size		= 0x12c00000,
	},
	{
		.name		= "logo-NAND",
		.offset		= 0x12f00000,
		.size		= 0x00600000,
	},	
	{
		.name		= "user-data-NAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},

};
EXPORT_SYMBOL(nand_partitions);
#endif

#ifdef CONFIG_MTD_NAND_VT8500_HWECC

	static int MAX_CHIP = CONFIG_MTD_NAND_CHIP_NUM;
	static int hardware_ecc = 1;

	#if (CONFIG_MTD_NAND_PAGE_SIZE == 2048)
		#define NAND_PAGE_SIZE 2048   /*  4096 or 512  */
	#else
		#if (CONFIG_MTD_NAND_PAGE_SIZE == 4096)
			#define NAND_PAGE_SIZE 4096   /*  2048 or 512  */
			/*#define ECC8BIT_ENGINE*/
		#else
			#define NAND_PAGE_SIZE 512   /*  4096 or 2048  */
		#endif
	#endif

	#if (CONFIG_MTD_NAND_HM_ECC == 1)
		#define NAND_HARMING_ECC
	#else
		#undef NAND_HARMING_ECC
	#endif

	#if 0
		#ifdef CONFIG_MTD_NAND_vt8500_HW_BCH_ECC
		static int eccmode = 1;    /* BCH ECC */
		#else
		static int eccmode = 0;    /* Harming ECC */
		#endif
	#endif

#else		/* if CONFIG_MTD_NAND_VT8500_HWECC else */
	#define NAND_HARMING_ECC
	#define NAND_PAGE_SIZE 2048   /*  4096 or 512  */
	#define MAX_CHIP 1
	static int hardware_ecc = 0;
#endif


#if (NAND_PAGE_SIZE == 512)
	#define PAGE_ADDR  /* support sequential read  */
	#ifdef PAGE_ADDR
		#undef PAGE_READ_COUNTER
	#else
		#define PAGE_READ_COUNTER  /*  read 256 bytes twice */
	#endif
#endif



/*
*  ecc_type = 0 : Harming ECC
*  ecc_type = 1 : BCH ECC
*/
#ifndef NAND_HARMING_ECC
	static int ecc_type  = 1;
#else
	static int ecc_type  = 0;
#endif

#ifndef NAND_HARMING_ECC
	/*static int nandtype = 1;*/    /* support nand new data structure */
#endif


/*
 * hardware specific Out Of Band information
*/

/*
* new oob placement block for use with hardware ecc generation
*/

#if (NAND_PAGE_SIZE == 2048)
	static struct nand_ecclayout vt8500_oobinfo_2048 = {
	/* nand flash new structure and BCH ECC oob info */
		.eccbytes = 7,
		.eccpos = { 24, 25, 26, 27, 28, 29, 30},
		.oobavail = 24,
		.oobfree = {{0, 24} }
	};
#endif

#if (NAND_PAGE_SIZE == 4096)
/*	#ifdef ECC8BIT_ENGINE*/
	static struct nand_ecclayout vt8500_8bit_oobinfo_4096 = {
	/* nand flash old structure and Harming ECC oob info */
		.eccbytes = 13,
		.eccpos = { 24, 25, 26, 27, 28, 29, 30,
								31, 32, 33, 34, 35, 36},
		.oobavail = 24,
		.oobfree = {{0, 24} }
	};
/*	#else*/
	static struct nand_ecclayout vt8500_oobinfo_4096 = {
	/* nand flash old structure and Harming ECC oob info */
		.eccbytes = 7,
		.eccpos = { 24, 25, 26, 27, 28, 29, 30},
		.oobavail = 24,
		.oobfree = {{0, 24} }
	};
	/*#endif*/
#endif

#ifdef NAND_HARMING_ECC
	#if (NAND_PAGE_SIZE == 2048)
		static struct nand_ecclayout vt8500_hm_oobinfo_2048 = {
		/*  nand flash old structure and Harming ECC oob info */
			.eccbytes = 14,
			.eccpos = { 32, 33, 34, 36, 37, 38, 40, 41, 42, 44, 45, 46, 48, 49},
			.oobavail = 32,
			.oobfree = {{0, 32} }
		};
	#endif

	#if (NAND_PAGE_SIZE == 4096)
		static struct nand_ecclayout vt8500_hm_oobinfo_4096 = {
			/*  nand flash old structure and Harming ECC oob info */
			.eccbytes = 27,
			.eccpos = {
				64, 65, 66, 68, 69, 70, 72, 73, 74, 76, 77, 78,
				80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94,
				96, 97, 98},
			.oobavail = 64,
			.oobfree = {{0, 64} }
		};
	#endif

	#if (NAND_PAGE_SIZE == 512)
		static struct nand_ecclayout vt8500_oobinfo_512 = {
		#if 0
			/* #if eccmode */   /*  nand flash new structure and BCH ECC oob info */
			.eccbytes = 10,
			.eccpos = { 0, 1, 2, 3, 4, 5, 6, 13, 14, 15},
			.oobfree = {{7, 6} }
			/* #else */        /*  nand flash old structure and Harming ECC oob info */
		#endif
			.eccbytes = 8,
			.eccpos = { 4, 5, 6, 8, 9, 10, 12, 13},
			.oobfree = {{0, 4}, {7, 1}, {11, 1}, {14, 2} }
			 /* #endif */
		};
	#endif
#endif

/*#if (NAND_PAGE_SIZE == 2048)
EXPORT_SYMBOL(vt8500_oobinfo_2048);
EXPORT_SYMBOL(vt8500_hm_oobinfo_2048);
#else
 #if (NAND_PAGE_SIZE == 4096)
 EXPORT_SYMBOL(vt8500_oobinfo_4096);
 EXPORT_SYMBOL(vt8500_hm_oobinfo_4096);
 #endif
#endif*/
/* Ick. The BBT code really ought to be able to work this bit out
	 for itself from the above, at least for the 2KiB case
*/
#if 1
	static uint8_t vt8500_bbt_pattern_2048[] = { 'B', 'b', 't', '0' };
	static uint8_t vt8500_mirror_pattern_2048[] = { '1', 't', 'b', 'B' };
#endif
/*static uint8_t vt8500_bbt_pattern_2048[] = { 0xFF, 0xFF };*/
/*static uint8_t vt8500_mirror_pattern_2048[] = { 0xFF, 0xFF };*/

#if (NAND_PAGE_SIZE == 512)
	/*static uint8_t vt8500_bbt_pattern_512[] = { 0xBB };*/
	/*static uint8_t vt8500_mirror_pattern_512[] = { 0xBC };*/
#endif


#if (NAND_PAGE_SIZE != 512)
static struct nand_bbt_descr vt8500_bbt_main_descr_2048 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	9,
	.len = 4,
	.veroffs = 25,
	.maxblocks = 4,
	.pattern = vt8500_bbt_pattern_2048,
	.wince_tlb_flag = 0
};

static struct nand_bbt_descr vt8500_bbt_mirror_descr_2048 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	9,
	.len = 4,
	.veroffs = 25,
	.maxblocks = 4,
	.pattern = vt8500_mirror_pattern_2048,
	.wince_tlb_flag = 0
};
#endif

#if (NAND_PAGE_SIZE == 512)
static struct nand_bbt_descr vt8500_bbt_main_descr_512 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	0,
	.len = 4,
	.veroffs = 14,
	.maxblocks = 4,
	.pattern = vt8500_bbt_pattern_2048,
	.wince_tlb_flag = 0
};

static struct nand_bbt_descr vt8500_bbt_mirror_descr_512 = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	0,
	.len = 4,
	.veroffs = 14,
	.maxblocks = 4,
	.pattern = vt8500_mirror_pattern_2048,
	.wince_tlb_flag = 0
};
#endif


/* controller and mtd information */

static void print_register(struct mtd_info *mtd);

struct vt8500_nand_set {
	int	nr_chips;
	int	nr_partitions;
	char *name;
	int *nr_map;
	struct mtd_partition *partitions;
};

#if 0
	struct vt8500_platform_nand {
		/* timing information for controller, all times in nanoseconds */

		int	tacls;	/* time for active CLE/ALE to nWE/nOE */
		int	twrph0;	/* active time for nWE/nOE */
		int	twrph1;	/* time for release CLE/ALE from nWE/nOE inactive */

		int nr_sets;
		struct vt8500_nand_set *sets;
		void (*select_chip)(struct s3c2410_nand_set *, int chip);
	}
#endif

struct vt8500_nand_info;

struct vt8500_nand_mtd {
	struct mtd_info	mtd;
	struct nand_chip chip;
	/*struct vt8500_nand_set* set;*/
	struct vt8500_nand_info *info;
	int	scan_res;
};

/* overview of the vt8500 nand state */

struct vt8500_nand_info {
	/* mtd info */
	struct nand_hw_control controller;
	struct vt8500_nand_mtd *mtds;
	struct vt8500_platform_nand *platform;

	/* device info */
	struct device *device;
	struct resource *area;
	void __iomem *reg;
	int cpu_type;
	int datalen;
	int nr_data;
	int data_pos;
	int page_addr;
	dma_addr_t dmaaddr;
	unsigned char *dmabuf;
};

/* conversion functions */

static struct vt8500_nand_mtd *vt8500_nand_mtd_toours(struct mtd_info *mtd)
{
	return container_of(mtd, struct vt8500_nand_mtd, mtd);
}

static struct vt8500_nand_info *vt8500_nand_mtd_toinfo(struct mtd_info *mtd)
{
	return vt8500_nand_mtd_toours(mtd)->info;
}

/*
static struct vt8500_nand_info *to_nand_info(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}
*/

static struct platform_device *to_platform(struct device *dev)
{
	return container_of(dev, struct platform_device, dev);
}

#if 0
static struct vt8500_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}
#endif

/*
 * type : HW or SW ECc
 *
*/
static void nfc_ecc_set(struct vt8500_nand_info *info, int type)
{
/* struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);*/

	if (type == hardware_ecc)
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) & 0xfb, info->reg + VT8500_NFC_MISC_CTRL);
	else
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x04, info->reg + VT8500_NFC_MISC_CTRL);
}


static void vt8500_nfc_init(struct vt8500_nand_info *info)
{
	*(volatile unsigned char *)PMNAND_ADDR  = (NFC_ClockDivisor&NFC_ClockMask); /*add by vincent*/
	while ((*(volatile unsigned long *)PMCS_ADDR)&(1 << 25))
		;
	/*  struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);*/
	/* enable chip select */
	/* new structure  */

	/* old structure */
	/* writeb(PAGE_2K|WIDTH_8|WP_DISABLE|DIRECT_MAP|OLDDATA_EN,
	info->reg + VT8500_NFC_NAND_TYPE_SEL);*/
	/* writeb((PAGE_2K|WIDTH_8|WP_DISABLE|DIRECT_MAP)&(~OLDDATA_EN),
	info->reg + VT8500_NFC_NAND_TYPE_SEL);*/
#if (NAND_PAGE_SIZE == 2048)
	writel((2 << 5) | 0x1F, info->reg + VT8500_NFC_PAGESIZE_DIVIDER_SEL); /*64 page per block*/
	#ifndef NAND_HARMING_ECC
		writeb((PAGE_2K|WIDTH_8|WP_DISABLE|DIRECT_MAP|CHECK_ALLFF)&(~OLDDATA_EN),
		info->reg + VT8500_NFC_NAND_TYPE_SEL);
	#else
		if (ecc_type == 1)
			writeb((PAGE_2K|WIDTH_8|WP_DISABLE|DIRECT_MAP|CHECK_ALLFF) & (~OLDDATA_EN),
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
		else
			writeb(PAGE_2K|WIDTH_8|WP_DISABLE|DIRECT_MAP|CHECK_ALLFF|OLDDATA_EN,
			info->reg + VT8500_NFC_NAND_TYPE_SEL);/*old structure*/
	#endif
#else
	#if (NAND_PAGE_SIZE == 4096)
		/*writel((2 << 5) | 0x1F, info->reg + VT8500_NFC_PAGESIZE_DIVIDER_SEL);*/ /*64 page per block*/
		writel((3 << 5) | 0x1F, info->reg + VT8500_NFC_PAGESIZE_DIVIDER_SEL); /*128 page per block*/
		#ifndef NAND_HARMING_ECC
			writeb((PAGE_4K|WIDTH_8|WP_DISABLE|DIRECT_MAP|CHECK_ALLFF)&(~OLDDATA_EN),
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
		#else
			if (ecc_type == 1)
				writeb((PAGE_4K|WIDTH_8|WP_DISABLE|DIRECT_MAP|CHECK_ALLFF) & (~OLDDATA_EN),
				info->reg + VT8500_NFC_NAND_TYPE_SEL);
			else
				writeb(PAGE_4K|WIDTH_8|WP_DISABLE|DIRECT_MAP|CHECK_ALLFF|OLDDATA_EN,
				info->reg + VT8500_NFC_NAND_TYPE_SEL);
		#endif
	#else
		/*  #else if (NAND_PAGE_SIZE == 512)*/
		/*  writeb(PAGE_512|WIDTH_8|WP_DISABLE|DIRECT_MAP|OLDDATA_EN,
		info->reg + VT8500_NFC_NAND_TYPE_SEL); */ /*old structure*/
		writel((1 << 5) | 0x1F, info->reg + VT8500_NFC_PAGESIZE_DIVIDER_SEL); /*32 page per block*/
		writeb((PAGE_512|WIDTH_8|WP_DISABLE|DIRECT_MAP)&(~OLDDATA_EN),
		info->reg + VT8500_NFC_NAND_TYPE_SEL); /*new structure*/
	#endif
#endif

	writel(readl(info->reg + VT8500_NFC_READ_CYCLE_PULE_CTRL) & 0xffff0000,
	info->reg + VT8500_NFC_READ_CYCLE_PULE_CTRL);
	writel(readl(info->reg + VT8500_NFC_READ_CYCLE_PULE_CTRL) | NFC_RWTimming,
	info->reg + VT8500_NFC_READ_CYCLE_PULE_CTRL);

}


#if (NAND_PAGE_SIZE != 512)
static void disable_redunt_out_bch_ctrl(struct vt8500_nand_info *info, int flag)
{
	if (flag == 1)
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL)|0x02, info->reg + VT8500_NFC_CALC_CTRL);
	else
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL)&0xfd, info->reg + VT8500_NFC_CALC_CTRL);
}
#endif

#if 0
static void redunt_read_hm_ecc_ctrl(struct vt8500_nand_info *info, int flag)
{
	if (flag == 1)
		writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) | 0x02, info->reg + VT8500_NFC_SMC_ENABLE);
	else
		writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd, info->reg + VT8500_NFC_SMC_ENABLE);
}
#endif

static void set_ecc_engine(struct vt8500_nand_info *info, int type)
{
	/*struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);*/
	writel(readl(info->reg + VT8500_NFC_ECC_BCH_CTRL) & 0xfffffffc, info->reg + VT8500_NFC_ECC_BCH_CTRL);
	writel(readl(info->reg + VT8500_NFC_ECC_BCH_CTRL) | type, info->reg + VT8500_NFC_ECC_BCH_CTRL);

	/* enable 4bit ecc interrupt and new structure */
	if (type == ECC8bit) {
		/*printk(KERN_ERR "set_ecc_engine for bch 8 bit\n");*/
		writew(ecc4bit_inetrrupt_enable, info->reg + VT8500_NFC_ECC_BCH_INT_MASK);
		writel(readl(info->reg + VT8500_NFC_ECC_BCH_CTRL) | READ_RESUME,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		if (ecc_type == 1)
			writel(readl(info->reg + VT8500_NFC_NAND_TYPE_SEL) & (~OLDDATA_EN),
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
		else
			writel(readl(info->reg + VT8500_NFC_NAND_TYPE_SEL) | OLDDATA_EN,
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
	} else if (type == ECC4bit) {
		/*printk(KERN_ERR "set_ecc_engine for bch 4 bit\n");*/
		writew(ecc4bit_inetrrupt_enable, info->reg + VT8500_NFC_ECC_BCH_INT_MASK);
		writel(readl(info->reg + VT8500_NFC_ECC_BCH_CTRL) | READ_RESUME,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		if (ecc_type == 1)
			writel(readl(info->reg + VT8500_NFC_NAND_TYPE_SEL) & (~OLDDATA_EN),
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
		else
			writel(readl(info->reg + VT8500_NFC_NAND_TYPE_SEL) | OLDDATA_EN,
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
	}	else { /*disable 4bit ecc interrupt and old structure*/
		writew(ecc4bit_inetrrupt_disable, info->reg + VT8500_NFC_ECC_BCH_INT_MASK);
		writel(readl(info->reg + VT8500_NFC_ECC_BCH_CTRL) | READ_RESUME,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/*printk(KERN_ERR "set_ecc_engine for harmming\n");*/
		if (ecc_type == 1)
			writel(readl(info->reg + VT8500_NFC_NAND_TYPE_SEL) & (~OLDDATA_EN),
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
		else
			writel(readl(info->reg + VT8500_NFC_NAND_TYPE_SEL) | OLDDATA_EN,
			info->reg + VT8500_NFC_NAND_TYPE_SEL);
	}
}


static int vt8500_nand_ready(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	unsigned int b2r_stat;
	int i = 0;

	while (1)	{
		if (readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE) & B2R)
			break;
		if ((++i>>20)) {
			printk(KERN_ERR "nand flash is not ready\n");
			/*print_register(mtd);*/
			/*    while (1);*/
			return -1;
		}
	}
	b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	if (readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE) & B2R)	{
		printk(KERN_ERR "NFC err : B2R status not clean\n");
		return -2;
	}
	return 0;
}


static int vt8500_nfc_transfer_ready(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int i = 0;

	while (1)	{
		if (!(readb(info->reg + VT8500_NFC_MISC_STAT_PORT) & NFC_BUSY))
			break;

		if (++i>>20)
			return -3;
	}
	return 0;
}
/* Vincent  2008.11.3*/
static int vt8500_wait_chip_ready(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int i = 0;

	while (1) {
		if ((readb(info->reg + VT8500_NFC_MISC_STAT_PORT) & FLASH_RDY))
			break;
		if (++i>>20)
			return -3;
	}
	return 0;
}
static int vt8500_wait_cmd_ready(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int i = 0;

	while (1)	{
		if (!(readb(info->reg + VT8500_NFC_MISC_STAT_PORT) & NFC_CMD_RDY))
			break;
		if (++i>>20)
			return -3;
	}
	return 0;
}

/* #if (NAND_PAGE_SIZE == 512) Vincent 2008.11.4
static int vt8500_wait_dma_ready(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int i = 0;

	while (1) {
		if (!(readb(info->reg + NFC_IDLE) & 0x02))
			break;
		if (++i>>20) {
			printk(KERN_ERR"\r DMA NOT Ready!\n");
			print_register(mtd);
			return -3;
		}
	}
	return 0;
}
#endif  Vincent 2008.11.4*/

static void vt8500_wait_nfc_ready(struct vt8500_nand_info *info)
{
	unsigned int bank_stat1, i = 0;
	while (1) {
		bank_stat1 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);/* Vincent 2008.11.14 */
		if (!(readb(info->reg + VT8500_NFC_MISC_STAT_PORT) & NFC_BUSY))
			break;
		else if ((bank_stat1 & 0x101) == (ERR_CORRECT | BCH_ERR))
			break;

		if (i>>20)
			return;
		i++;
	}
}

static void bit_correct(uint8_t *c, uint8_t pos)
{
	c[0] = (((c[0] ^ (0x01<<pos)) & (0x01<<pos)) | (c[0] & (~(0x01<<pos))));
	#if 0
	temp = info->dmabuf[bch_err_idx[0] >> 3];
	temp >>= ((bch_err_idx[0] & 0x07) - 1);
	#endif
}

/*
 * flag = 0, need check BCH ECC
 * flag = 1, don't check ECC
 * flag = 2, need check Harming ECC
 *
*/

static int vt8500_nfc_wait_idle(struct mtd_info *mtd, unsigned int flag, int command,
int column, unsigned int page_addr)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int i = 0;

	#ifndef NAND_HARMING_ECC/*Vincent 2008.11.14*/
	int k, page_step, oob_step;
	unsigned int bank_stat1, bank_stat2;  /* redunt_stat;*/
	unsigned int /*bank, */data_redunt_flag;  /* bank_sel;*/
	/*unsigned int bch_err_pos[4], bch_ecc_idx;*/
	page_step = 1 + NAND_PAGE_SIZE/512;
	oob_step = 1;
	#endif

	while (1) {
		#ifdef NAND_DEBUG
		printk(KERN_WARNING "waiting NFC idle......\n");
		#endif
		if (readb(info->reg + VT8500_NFC_IDLE_STAT) & NFC_IDLE)
			break;

		if (flag == 1 || flag == 2) {
			#ifdef NAND_DEBUG
			printk(KERN_NOTICE "\r in wait idle ECC OOB MODE(9)0x024\n");
			print_register(mtd);
			#endif
			;
			#if 0
			else if (flag == 2) {
				/* bad block check: don't need HM ECC correct
				* and read oob don't need check ecc */
				if (readb(info->reg + VT8500_NFC_REDUNT_ECC_STAT) & 0x07) {
					printk(KERN_ERR "There are ecc err in reduntant area---------\n");
					mtd->ecc_stats.failed++;
					return -1;
				} else if (readb(info->reg + VT8500_NFC_BANK18_ECC_STAT) & 0xffffffff) {
					printk(KERN_ERR "There are ecc err in data area--------------\n");
					mtd->ecc_stats.failed++;
					return -1;
				}
			}
			#endif
		} else {
			/* check BCH */
			#ifndef NAND_HARMING_ECC/*Vincent 2008.11.14*/
			if (command == NAND_CMD_READ0) {
				#ifdef NAND_DEBUG
				printk(KERN_NOTICE "in nfc_wait_idle(): Read data \n");
				#endif
				/*if(page_addr == 0x343fb) {
					printk(KERN_NOTICE "page_step:%x\n", page_step);
					for (j = 0; j < 0xA8; j += 16)
						printk(KERN_NOTICE "NFCR%x ~ NFCR%x = 0x%8.8x 0x%8.8x 0x%8.8x 0x%8.8x\r\n",
						j/4, (j+12)/4,
						readl(info->reg + j + 0),
						readl(info->reg + j + 4),
						readl(info->reg + j + 8),
						readl(info->reg + j + 12));
				}*/
				for (k = 0; k < page_step; k++) {
					vt8500_wait_nfc_ready(info);
					bank_stat1 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
					if ((bank_stat1 & 0x101) == (ERR_CORRECT | BCH_ERR)) {
						bank_stat2 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT2);
						/* 0: data, 1: reduntant area */
						data_redunt_flag = bank_stat2 & 0x800;
						printk(KERN_NOTICE" BCH Read data ecc eror command: %x"
						" column:%x, page_addr:%x\n", command, column, page_addr);
						#ifdef NAND_DEBUG
						printk(KERN_NOTICE "in test check bch 4/8 bit ecc correct \n");
						#endif
						if (data_redunt_flag) {
							bch4bit_redunt_ecc_correct(mtd);
							if (redunt_err_mark == 0) {
								redunt_err_mark = 1;
								/*printk(KERN_NOTICE " read page redundant mark set to 1\n");*/
							}

						} else {
							if (redunt_err_mark == 1) {
								redunt_err_mark = 2;
								/*printk(KERN_NOTICE "read page redundant mark set to 2\n");*/
							}
							bch4bit_data_ecc_correct(mtd);
						}
					}
				}
			} else if (command == NAND_CMD_READOOB) {
				/* for reduntant area */
				#ifdef NAND_DEBUG
				printk(KERN_NOTICE "in nfc_wait_idle(): Read oob data \n");
				#endif
				bank_stat1 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
				if ((bank_stat1 & 0x101) == (ERR_CORRECT | BCH_ERR)) {
					if (redunt_err_mark == 0) {
						redunt_err_mark = 1;
						/*printk(KERN_NOTICE " read redunt redundant mark set to 1\n");*/
					}
					bank_stat2 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT2);
					printk(KERN_NOTICE" BCH Read oob ecc eror command: %x"
					" column:%x, page_addr:%x\n", command, column, page_addr);

					vt8500_wait_nfc_ready(info);
					bch4bit_redunt_ecc_correct(mtd);
				}
			}
			#endif
		}
		if (i>>20)
			return -1;
		i++;
	}
	#ifndef NAND_HARMING_ECC/*Vincent 2008.11.14*/
	/* continue read next bank and calc BCH ECC */
	if (ecc_type == 1)
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
	#endif

	return 0;
}

void bch4bit_data_ecc_correct(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	unsigned int bch_err_pos[8], bank_stat1, bank_stat2, bch_ecc_idx, bank, ecc_engine;

	bank_stat1 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
	if ((bank_stat1 & 0x101) == (ERR_CORRECT | BCH_ERR)) {
		/* BCH ECC err process */
		bank_stat2 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT2);
		bch_ecc_idx = bank_stat2 & 0x0f;
		bank = (bank_stat2 & 0x700) >> 8;
		/* for data area */
		#ifdef NAND_DEBUG
		printk(KERN_NOTICE "in nfc_wait_idle(): Read data \n");
		#endif
		ecc_engine = readl(info->reg + VT8500_NFC_ECC_BCH_CTRL)&3;
		if (bch_ecc_idx > (ecc_engine == 1 ? 4 : 8) ) {
			/* BCH ECC code of 512 bytes data which is all "FF" */
			if ((readl((unsigned int *)(info->reg+ECC_FIFO_0) + bank * 4) == 0xffffffff) &&
			((readl((unsigned int *)(info->reg+ECC_FIFO_1) + bank * 4) & 0xffffff) == 0xffffff)) {
				#ifdef NAND_DEBUG
				printk(KERN_WARNING "in nfc_wait_idle(): BCH ECC code is all FF\n");
				#endif
				writew(readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1)|(ERR_CORRECT|BCH_ERR),
				info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
				writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
				info->reg + VT8500_NFC_ECC_BCH_CTRL);
				return;
			}
			#ifdef NAND_DEBUG
			printk(KERN_NOTICE "reg1 is %x\n",
			readl((unsigned int *)(info->reg+ECC_FIFO_0) + bank * 4));
			printk(KERN_NOTICE "reg2 is %x\n",
			readl((unsigned int *)(info->reg+ECC_FIFO_1) + bank * 4));
			printk(KERN_ERR "in nfc_wait_idle(): data uncorrected err \n");
			#endif
			writew(readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1)|(ERR_CORRECT | BCH_ERR),
			info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
			writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
			info->reg + VT8500_NFC_ECC_BCH_CTRL);
			mtd->ecc_stats.failed++;
			return; /* uncorrected err */
		}

		/* mtd->ecc_stats.corrected += (bank_stat2 & 0x0f);*/
		/* BCH ECC correct */
		/*#ifdef NAND_DEBUG*/
		printk(KERN_NOTICE "data area %d bit corrected err on bank %d \n", bch_ecc_idx, bank);
		/*#endif*/
		if (bch_ecc_idx == 1) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);
			//#ifdef NAND_DEBUG
			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			//#endif
		}	else if (bch_ecc_idx == 2) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
		}	else if (bch_ecc_idx == 3) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff;
			if((bch_err_pos[2] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[2] >> 3)], bch_err_pos[2] & 0x07);

			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area third ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
		}	else if (bch_ecc_idx == 4) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff;
			if((bch_err_pos[2] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[2] >> 3)], bch_err_pos[2] & 0x07);

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff0000) >> 16;
			if((bch_err_pos[3] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[3] >> 3)], bch_err_pos[3] & 0x07);
			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area third ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fourth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			}	else if (bch_ecc_idx == 5) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff;
			if((bch_err_pos[2] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[2] >> 3)], bch_err_pos[2] & 0x07);

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff0000) >> 16;
			if((bch_err_pos[3] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[3] >> 3)], bch_err_pos[3] & 0x07);

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff;
			if((bch_err_pos[4] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[4] >> 3)], bch_err_pos[4] & 0x07);

			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area third ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fourth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fiveth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
		}	else if (bch_ecc_idx == 6) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff;
			if((bch_err_pos[2] >> 3) < 512)
			bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[2] >> 3)], bch_err_pos[2] & 0x07);

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff0000) >> 16;
			if((bch_err_pos[3] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[3] >> 3)], bch_err_pos[3] & 0x07);

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff;
			if((bch_err_pos[4] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[4] >> 3)], bch_err_pos[4] & 0x07);

			bch_err_pos[5] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff0000) >> 16;
			if((bch_err_pos[5] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[5] >> 3)], bch_err_pos[5] & 0x07);

			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area third ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fourth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fiveth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area sixth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[5] >> 3), (bch_err_pos[5] & 0x07));
		}	else if (bch_ecc_idx == 7) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff;
			if((bch_err_pos[2] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[2] >> 3)], bch_err_pos[2] & 0x07);

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff0000) >> 16;
			if((bch_err_pos[3] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[3] >> 3)], bch_err_pos[3] & 0x07);

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff;
			if((bch_err_pos[4] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[4] >> 3)], bch_err_pos[4] & 0x07);

			bch_err_pos[5] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff0000) >> 16;
			if((bch_err_pos[5] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[5] >> 3)], bch_err_pos[5] & 0x07);

			bch_err_pos[6] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS4) & 0x1fff;
			if((bch_err_pos[6] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[6] >> 3)], bch_err_pos[6] & 0x07);

			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area third ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fourth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fiveth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area sixth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[5] >> 3), (bch_err_pos[5] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area seventh ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[6] >> 3), (bch_err_pos[6] & 0x07));
		/*}	else if (bch_ecc_idx == 8) {*/
		}	else {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff;
			if((bch_err_pos[0] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[0] >> 3)], bch_err_pos[0] & 0x07);

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1fff0000) >> 16;
			if((bch_err_pos[1] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[1] >> 3)], bch_err_pos[1] & 0x07);

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff;
			if((bch_err_pos[2] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[2] >> 3)], bch_err_pos[2] & 0x07);

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1fff0000) >> 16;
			if((bch_err_pos[3] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[3] >> 3)], bch_err_pos[3] & 0x07);

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff;
			if((bch_err_pos[4] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[4] >> 3)], bch_err_pos[4] & 0x07);

			bch_err_pos[5] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1fff0000) >> 16;
			if((bch_err_pos[5] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[5] >> 3)], bch_err_pos[5] & 0x07);

			bch_err_pos[6] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS4) & 0x1fff;
			if((bch_err_pos[6] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[6] >> 3)], bch_err_pos[6] & 0x07);

			bch_err_pos[7] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS4) & 0x1fff0000) >> 16;
			if((bch_err_pos[7] >> 3) < 512)
				bit_correct(&info->dmabuf[512 * bank + (bch_err_pos[7] >> 3)], bch_err_pos[7] & 0x07);
			printk(KERN_NOTICE "in nfc_wait_idle(): data area first ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area second ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area third ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fourth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area fiveth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area sixth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[5] >> 3), (bch_err_pos[5] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area seventh ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[6] >> 3), (bch_err_pos[6] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): data area eighth ecc error position is byte%d bit%d\n",
			512 * bank + (bch_err_pos[7] >> 3), (bch_err_pos[7] & 0x07));
		}
	} /* end of if ((bank_stat1 & 0x101) */
	/* continue read next bank and calc BCH ECC */
	writew(readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1)|(ERR_CORRECT | BCH_ERR),
	info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
	writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
	info->reg + VT8500_NFC_ECC_BCH_CTRL);
}

void bch4bit_redunt_ecc_correct(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	unsigned int bch_err_pos[8], bank_stat1, bank_stat2, bch_ecc_idx, ecc_engine;

	/* BCH ECC err process */
	bank_stat2 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT2);
	bch_ecc_idx = bank_stat2 & 0x0f;

	/* bank = (bank_stat2 & 0x700) >> 8; */
	bank_stat1 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
	if ((bank_stat1 & 0x101) == (ERR_CORRECT | BCH_ERR)) {
		/* mtd->ecc_stats.corrected += (bank_stat2 & 0x0f);*/
		/* BCH ECC correct */
		/* for reduntant area */
		ecc_engine = readl(info->reg + VT8500_NFC_ECC_BCH_CTRL)&3;
		if (bch_ecc_idx > (ecc_engine == 1 ? 4 : 8)) {
			writew(readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1)|(ERR_CORRECT | BCH_ERR),
			info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
			writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
			info->reg + VT8500_NFC_ECC_BCH_CTRL);
			printk(KERN_ERR
			"in nfc_wait_idle(): redunt area uncorrected err \n");
			mtd->ecc_stats.failed++;
			return;
			/* return -4;*/  /* uncorrected err */
		}
		/* mtd->ecc_stats.corrected += (bank_stat2 & 0x0f);*/
		/* BCH ECC correct */
		/*#ifdef NAND_DEBUG*/
		printk(KERN_NOTICE "reduntant %d bit corrected error\n", bch_ecc_idx);
		/*#endif*/

		if (bch_ecc_idx == 1) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));
			/*#ifdef NAND_DEBUG*/
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			/*#endif*/
		}	else if (bch_ecc_idx == 2) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
		}	else if (bch_ecc_idx == 3) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[2] >> 3),
			(bch_err_pos[2] & 0x07));

			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt third ecc error position is byte%d bit%d\n",
			(bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
		}	else if (bch_ecc_idx == 4) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[2] >> 3),
			(bch_err_pos[2] & 0x07));

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[3] >> 3),
			(bch_err_pos[3] & 0x07));

			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt third ecc error position is byte%d bit%d\n",
			(bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fourth ecc error position is byte%d bit%d\n",
			(bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
		}	else if (bch_ecc_idx == 5) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[2] >> 3),
			(bch_err_pos[2] & 0x07));

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[3] >> 3),
			(bch_err_pos[3] & 0x07));

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[4] >> 3),
			(bch_err_pos[4] & 0x07));

			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt third ecc error position is byte%d bit%d\n",
			(bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fourth ecc error position is byte%d bit%d\n",
			(bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fiveth ecc error position is byte%d bit%d\n",
			(bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
		}	else if (bch_ecc_idx == 6) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[2] >> 3),
			(bch_err_pos[2] & 0x07));

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[3] >> 3),
			(bch_err_pos[3] & 0x07));

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[4] >> 3),
			(bch_err_pos[4] & 0x07));

			bch_err_pos[5] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[5] >> 3),
			(bch_err_pos[5] & 0x07));

			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt third ecc error position is byte%d bit%d\n",
			(bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fourth ecc error position is byte%d bit%d\n",
			(bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fiveth ecc error position is byte%d bit%d\n",
			(bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt sixth ecc error position is byte%d bit%d\n",
			(bch_err_pos[5] >> 3), (bch_err_pos[5] & 0x07));
		}	else if (bch_ecc_idx == 7) {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[2] >> 3),
			(bch_err_pos[2] & 0x07));

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[3] >> 3),
			(bch_err_pos[3] & 0x07));

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[4] >> 3),
			(bch_err_pos[4] & 0x07));

			bch_err_pos[5] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[5] >> 3),
			(bch_err_pos[5] & 0x07));

			bch_err_pos[6] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS4) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[6] >> 3),
			(bch_err_pos[6] & 0x07));

			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt third ecc error position is byte%d bit%d\n",
			(bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fourth ecc error position is byte%d bit%d\n",
			(bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fiveth ecc error position is byte%d bit%d\n",
			(bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt sixth ecc error position is byte%d bit%d\n",
			(bch_err_pos[5] >> 3), (bch_err_pos[5] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt seventh ecc error position is byte%d bit%d\n",
			(bch_err_pos[6] >> 3), (bch_err_pos[6] & 0x07));
		}	else {
			bch_err_pos[0] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[0] >> 3),
			(bch_err_pos[0] & 0x07));

			bch_err_pos[1] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[1] >> 3),
			(bch_err_pos[1] & 0x07));

			bch_err_pos[2] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[2] >> 3),
			(bch_err_pos[2] & 0x07));

			bch_err_pos[3] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[3] >> 3),
			(bch_err_pos[3] & 0x07));

			bch_err_pos[4] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[4] >> 3),
			(bch_err_pos[4] & 0x07));

			bch_err_pos[5] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS3) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[5] >> 3),
			(bch_err_pos[5] & 0x07));

			bch_err_pos[6] = readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS4) & 0x1ff;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[6] >> 3),
			(bch_err_pos[6] & 0x07));

			bch_err_pos[7] = (readw(info->reg + VT8500_NFC_ECC_BCH_ERR_POS4) & 0x1ff0000) >> 16;
			bit_correct((uint8_t *)(info->reg+ECC_FIFO_0)+(bch_err_pos[7] >> 3),
			(bch_err_pos[7] & 0x07));

			printk(KERN_NOTICE "in nfc_wait_idle(): redunt first ecc error position is byte%d bit%d\n",
			(bch_err_pos[0] >> 3), (bch_err_pos[0] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt second ecc error position is byte%d bit%d\n",
			(bch_err_pos[1] >> 3), (bch_err_pos[1] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt third ecc error position is byte%d bit%d\n",
			(bch_err_pos[2] >> 3), (bch_err_pos[2] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fourth ecc error position is byte%d bit%d\n",
			(bch_err_pos[3] >> 3), (bch_err_pos[3] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt fiveth ecc error position is byte%d bit%d\n",
			(bch_err_pos[4] >> 3), (bch_err_pos[4] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt sixth ecc error position is byte%d bit%d\n",
			(bch_err_pos[5] >> 3), (bch_err_pos[5] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt seventh ecc error position is byte%d bit%d\n",
			(bch_err_pos[6] >> 3), (bch_err_pos[6] & 0x07));
			printk(KERN_NOTICE "in nfc_wait_idle(): redunt eighth ecc error position is byte%d bit%d\n",
			(bch_err_pos[7] >> 3), (bch_err_pos[7] & 0x07));
		}
	}
	/* continue read next bank and calc BCH ECC */
	writew(readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1)|(ERR_CORRECT | BCH_ERR),
	info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
	writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
	info->reg + VT8500_NFC_ECC_BCH_CTRL);
}

/*
*   [Routine Description]
*	read status
*   [Arguments]
*	cmd : nand read status command
*   [Return]
*	the result of command
*/
static int vt8500_read_nand_status(struct mtd_info *mtd, unsigned char cmd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int cfg = 0, status = -1;
	unsigned int b2r_stat;

	writeb(cmd, info->reg + VT8500_NFC_COMPORT0);
	cfg = DPAHSE_DISABLE|NFC2NAND|(1<<1);

	b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

	writeb(cfg|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);
	status = vt8500_wait_cmd_ready(mtd);
	if (status) {
		printk(KERN_ERR "NFC command transfer1 is not ready\n");
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		return status;
	}
	b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

	cfg = SING_RW|NAND2NFC;
	writeb(cfg|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);

	status = vt8500_wait_cmd_ready(mtd);
	if (status) {
		printk(KERN_ERR "NFC command transfer2 is not ready\n");
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		return status;
	}
	status = vt8500_nfc_transfer_ready(mtd);
	/* status = vt8500_nand_wait_idle(mtd);*/
	if (status) {
		printk(KERN_ERR "NFC IO transfer is not ready\n");
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/*print_register(mtd);*/
		return status;
	}
		 /* return read status  */
	/*   return readb(info->reg + VT8500_NFC_DATAPORT) & 0xff;*/
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "read status is %x\n", readb(info->reg + VT8500_NFC_DATAPORT) & 0xff);
	#endif
	info->datalen = 0;
	info->dmabuf[0] = readb(info->reg + VT8500_NFC_DATAPORT) & 0xff;
	status = info->dmabuf[0];
	return status;
}


/* data_flag = 0:  set data ecc fifo */
static int vt8500_nfc_dma_cfg(struct mtd_info *mtd, unsigned int len, unsigned int wr,
int data_flag, int Nbank)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	int status;
	unsigned long *ReadDesc, *WriteDesc;
	ReadDesc = (unsigned long *)(info->dmabuf + NAND_PAGE_SIZE + 0x100);
	WriteDesc = (unsigned long *)(info->dmabuf + NAND_PAGE_SIZE + 0x200);
	/*
	printk(KERN_ERR "info->dmabuf = 0x%x\r\n", (unsigned int) info->dmabuf);
	printk(KERN_ERR "info->dmaaddr = 0x%x\r\n", (unsigned int) info->dmaaddr);
	printk(KERN_ERR "ReadDesc addr = 0x%x\r\n", (unsigned int) ReadDesc);
	printk(KERN_ERR "WriteDesc addr = 0x%x\r\n", (unsigned int) WriteDesc);
	*/

	if (len == 0)	{
		printk(KERN_ERR "DMA transfer length = 0\r\n");
		return 1;
	}
	if (data_flag == 0) {
		/* data:  set data ecc fifo */
		#if (NAND_PAGE_SIZE == 512)
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_0));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_1));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_2));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_3));
		#else /* pagesize = 2048 or 4096 */
		#if (NAND_PAGE_SIZE == 2048)
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);
		#else /*if (NAND_PAGE_SIZE == 4096)*/
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_0));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_1));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_2));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_3));

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_4));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_5));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_6));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_7));

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_8));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_9));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_a));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_b));

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_c));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_d));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_e));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_f));
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) | 0x08, info->reg + VT8500_NFC_CALC_CTRL);
		#endif
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_0));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_1));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_2));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_3));

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_4));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_5));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_6));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_7));

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_8));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_9));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_a));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_b));

		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_c));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_d));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_e));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_f));
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);
		#endif
	} else if (data_flag == 1) {
		/* reduntant area:  set reduntant data ecc fifo  BCH ECC */
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_0));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_1));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_2));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_3));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_4));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_5));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_6));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_7));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_8));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_9));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_a));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_b));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_c));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_d));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_e));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_f));
	#if (NAND_PAGE_SIZE == 4096)
	#ifdef NAND_HARMING_ECC
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) | 0x08, info->reg + VT8500_NFC_CALC_CTRL);
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_0));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_1));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_2));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_3));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_4));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_5));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_6));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_7));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_8));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_9));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_a));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_b));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_c));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_d));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_e));
		writel(0xffffffff, (unsigned int *)(info->reg + ECC_FIFO_f));
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);
	#endif
	#endif
#if 0 /*Vincent 2008.11.4*/
	#if (NAND_PAGE_SIZE == 2048)
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);
		if (Nbank == 0)
			writel(readl(info->reg + ECC_FIFO_3) | 0xffffff00, info->reg + ECC_FIFO_3);
		else if (Nbank == 1)
			writel(readl(info->reg + ECC_FIFO_7) | 0xffffff00, info->reg + ECC_FIFO_7);
		else if (Nbank == 2)
			writel(readl(info->reg + ECC_FIFO_b) | 0xffffff00, info->reg + ECC_FIFO_b);
		else if (Nbank == 3)
			writel(readl(info->reg + ECC_FIFO_f) | 0xffffff00, info->reg + ECC_FIFO_f);
	#else
	#if (NAND_PAGE_SIZE == 4096)
	writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);

	if (Nbank == 0)
		writel(readl(info->reg + ECC_FIFO_3) | 0xffffff00, info->reg + ECC_FIFO_3);
	else if (Nbank == 1)
		writel(readl(info->reg + ECC_FIFO_7) | 0xffffff00, info->reg + ECC_FIFO_7);
	else if (Nbank == 2)
		writel(readl(info->reg + ECC_FIFO_b) | 0xffffff00, info->reg + ECC_FIFO_b);
	else if (Nbank == 3)
		writel(readl(info->reg + ECC_FIFO_f) | 0xffffff00, info->reg + ECC_FIFO_f);

	writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) | 0x08, info->reg + VT8500_NFC_CALC_CTRL);

	if (Nbank == 0)
		writel(readl(info->reg + ECC_FIFO_3) | 0xffffff00, info->reg + ECC_FIFO_3);
	else if (Nbank == 1)
		writel(readl(info->reg + ECC_FIFO_7) | 0xffffff00, info->reg + ECC_FIFO_7);
	else if (Nbank == 2)
		writel(readl(info->reg + ECC_FIFO_b) | 0xffffff00, info->reg + ECC_FIFO_b);
	else if (Nbank == 3)
		writel(readl(info->reg + ECC_FIFO_f) | 0xffffff00, info->reg + ECC_FIFO_f);
	#else   /* pagesize = 512  BCH ECC */
	writel(readl(info->reg + ECC_FIFO_3) | 0xffffff00, info->reg + ECC_FIFO_3);
	#endif
	#endif
#endif /* end of #if1 */
	} else if (data_flag == 2) {
		/* reduntant area:  set reduntant data ecc fifo  Harming ECC */
	#if 0
	if (Nbank == 0)
		writel(readl(info->reg + ECC_FIFO_3) | 0xffffff00, info->reg + ECC_FIFO_3);

		else if (Nbank == 1)
			writel(readl(info->reg + ECC_FIFO_7) | 0xffffff00, info->reg + ECC_FIFO_7);
		else if (Nbank == 2)
			writel(readl(info->reg + ECC_FIFO_b) | 0xffffff00, info->reg + ECC_FIFO_b);
		else if (Nbank == 3)
			writel(readl(info->reg + ECC_FIFO_f) | 0xffffff00, info->reg + ECC_FIFO_f);
	#endif
		#if (NAND_PAGE_SIZE == 2048)
			writel(readl(info->reg + ECC_FIFO_c) | 0xffff0000, info->reg + ECC_FIFO_c);
		#else   /* pagesize = 512  Harming ECC */
			writel(readl(info->reg + ECC_FIFO_3) | 0xfffff00, info->reg + ECC_FIFO_3);
		#endif
	}
	writew(len - 1, info->reg + VT8500_NFC_DMA_COUNTER);
	if (readl(info->reg + NFC_DMA_ISR) & NAND_PDMA_IER_INT_STS)
		writel(NAND_PDMA_IER_INT_STS, info->reg + NFC_DMA_ISR);

	if (readl(info->reg + NFC_DMA_ISR) & NAND_PDMA_IER_INT_STS) {
		printk(KERN_ERR "PDMA interrupt status can't be clear ");
		printk(KERN_ERR "NFC_DMA_ISR = 0x%8.8x \n", (unsigned int)readl(info->reg + NFC_DMA_ISR));
	}

	status = nand_init_pdma(mtd);
	if (status)
		printk(KERN_ERR "nand_init_pdma fail status = 0x%x", status);
	nand_alloc_desc_pool((wr) ? WriteDesc : ReadDesc);
	/*nand_init_short_desc((wr)?WriteDesc : ReadDesc, len, (unsigned long *)buf);*/
	nand_init_long_desc((wr) ? WriteDesc : ReadDesc, len, (unsigned long *)info->dmaaddr, 0, 1);
	nand_config_pdma(mtd,
	(wr) ? (unsigned long *)(info->dmaaddr + NAND_PAGE_SIZE + 0x200)
	: (unsigned long *)(info->dmaaddr + NAND_PAGE_SIZE + 0x100), wr);

	return 0;
}

int nand_init_pdma(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	writel(NAND_PDMA_GCR_SOFTRESET, info->reg + NFC_DMA_GCR);
	writel(NAND_PDMA_GCR_DMA_EN, info->reg + NFC_DMA_GCR);
	if (readl(info->reg + NFC_DMA_GCR) & NAND_PDMA_GCR_DMA_EN)
		return 0;
	else
		return 1;
}


int nand_free_pdma(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	writel(0, info->reg + NFC_DMA_DESPR);
	writel(0, info->reg + NFC_DMA_GCR);
	return 0;
}


int nand_alloc_desc_pool(unsigned long *DescAddr)
{
	memset(DescAddr, 0x00, 0x100);
	return 0;
}

int nand_init_short_desc(unsigned long *DescAddr, unsigned int ReqCount, unsigned long *BufferAddr)
{
	struct _NAND_PDMA_DESC_S *CurDes_S;
	CurDes_S = (struct _NAND_PDMA_DESC_S *) DescAddr;
	CurDes_S->ReqCount = ReqCount;
	CurDes_S->i = 1;
	CurDes_S->end = 1;
	CurDes_S->format = 0;
	CurDes_S->DataBufferAddr = (unsigned long)BufferAddr;
	return 0;
}

int nand_init_long_desc(unsigned long *DescAddr, unsigned int ReqCount, unsigned long *BufferAddr,
unsigned long *BranchAddr, int End)
{
	struct _NAND_PDMA_DESC_L *CurDes_L;
	CurDes_L = (struct _NAND_PDMA_DESC_L *) DescAddr;
	CurDes_L->ReqCount = ReqCount;
	CurDes_L->i = 0;
	CurDes_L->format = 1;
	CurDes_L->DataBufferAddr = (unsigned long)BufferAddr;
	CurDes_L->BranchAddr = (unsigned long)BranchAddr;
	if (End) {
		CurDes_L->end = 1;
		CurDes_L->i = 1;
	}

	return 0;
}
/*
int nand_config_desc(unsigned long *DescAddr, unsigned long *BufferAddr, int Blk_Cnt)
{
	int i = 0 ;
	unsigned long *CurDes = DescAddr;

	nand_alloc_desc_pool(CurDes);


	for (i = 0 ; i < 3 ; i++) {
		nand_init_short_desc(CurDes, 0x80, BufferAddr);
		BufferAddr += (i * 0x80);
		CurDes += (i * sizeof(NAND_PDMA_DESC_S));
	}
	if (Blk_Cnt > 1) {
		nand_init_long_desc(CurDes, 0x80, BufferAddr, CurDes + sizeof(NAND_PDMA_DESC_L), 0);
		BufferAddr += (i * 0x80);
		CurDes += (i * sizeof(NAND_PDMA_DESC_L));

		nand_init_long_desc(CurDes, (Blk_Cnt - 1) * 512, BufferAddr,
		CurDes + sizeof(NAND_PDMA_DESC_L), 1);
	} else {
		nand_init_long_desc(CurDes, 0x80, BufferAddr, CurDes + sizeof(NAND_PDMA_DESC_L), 1);
	}

	return 0;
}
*/

int nand_config_pdma(struct mtd_info *mtd, unsigned long *DescAddr, unsigned int dir)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	/*writel(NAND_PDMA_IER_INT_EN, info->reg + NFC_DMA_IER);*/
	writel((unsigned long)DescAddr, info->reg + NFC_DMA_DESPR);
	if (dir == NAND_PDMA_READ)
		writel(readl(info->reg + NFC_DMA_CCR)|NAND_PDMA_CCR_peripheral_to_IF,
		info->reg + NFC_DMA_CCR);
	else
		writel(readl(info->reg + NFC_DMA_CCR)&(~NAND_PDMA_CCR_IF_to_peripheral),
		info->reg + NFC_DMA_CCR);

	/*mask_interrupt(IRQ_NFC_DMA);*/
	writel(readl(info->reg + NFC_DMA_CCR)|NAND_PDMA_CCR_RUN, info->reg + NFC_DMA_CCR);
	/*printk(KERN_ERR "NFC_DMA_CCR = 0x%8.8x\r\n", readl(info->reg + NFC_DMA_CCR));*/
	/*print_register(mtd);*/
	return 0;
}

int nand_pdma_handler(struct mtd_info *mtd)
{
	unsigned long status = 0;
	unsigned long count = 0;
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	count = 0x100000;
	/*	 polling CSR TC status	*/
	do {
		count--;
		if (readl(info->reg + NFC_DMA_ISR) & NAND_PDMA_IER_INT_STS) {
			status = readl(info->reg + NFC_DMA_CCR) & NAND_PDMA_CCR_EvtCode;
			writel(readl(info->reg + NFC_DMA_ISR)&NAND_PDMA_IER_INT_STS, info->reg + NFC_DMA_ISR);
			break;
		}
		if (count == 0) {
			printk(KERN_ERR "PDMA Time Out!\n");
			printk(KERN_ERR "NFC_DMA_CCR = 0x%8.8x\r\n",
			(unsigned int)readl(info->reg + NFC_DMA_CCR));
			/*print_register(mtd);*/
			count = 0x100000;
			/*break;*/
		}
	} while (1);
	if (status == NAND_PDMA_CCR_Evt_ff_underrun)
		printk(KERN_ERR "PDMA Buffer under run!\n");

	if (status == NAND_PDMA_CCR_Evt_ff_overrun)
		printk(KERN_ERR "PDMA Buffer over run!\n");

	if (status == NAND_PDMA_CCR_Evt_desp_read)
		printk(KERN_ERR "PDMA read Descriptor error!\n");

	if (status == NAND_PDMA_CCR_Evt_data_rw)
		printk(KERN_ERR "PDMA read/write memory descriptor error!\n");

	if (status == NAND_PDMA_CCR_Evt_early_end)
		printk(KERN_ERR "PDMA read early end!\n");

	if (count == 0) {
		printk(KERN_ERR "PDMA TimeOut!\n");
		while (1)
			;
	}
	return 0;
}


static int vt8500_nand_readID(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	unsigned int cfg = 0, i = 0;
	int status = -1;

	writeb(NAND_CMD_READID, info->reg + VT8500_NFC_COMPORT0);
	writeb(0x00, info->reg + VT8500_NFC_COMPORT1_2);
	cfg = DPAHSE_DISABLE|(0x02<<1);
	writeb(cfg|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);

	status = vt8500_wait_cmd_ready(mtd);
	/*	status = vt8500_nfc_ready(mtd);*/

	if (status) {
		printk(KERN_ERR "in vt8500_nand_readID(): wait cmd is not ready\n");
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		return status;
	}
	cfg = NAND2NFC|SING_RW;
	for (i = 0; i < 5; i++) {
		writeb(cfg|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);
		status = vt8500_wait_cmd_ready(mtd);
		/*	status = vt8500_nfc_ready(mtd);*/
		if (status)
				return status;
		status = vt8500_nfc_transfer_ready(mtd);
		/* status = vt8500_nand_wait_idle(mtd);*/
		if (status) {
			printk(KERN_ERR "in vt8500_nand_readID(): wait transfer cmd is not ready\n");
			return status;
		}
		info->dmabuf[i] = readb(info->reg + VT8500_NFC_DATAPORT) & 0xff;

		#ifdef NAND_DEBUG
			printk(KERN_NOTICE "readID is %x\n", readb(info->reg + VT8500_NFC_DATAPORT));
		#endif
	}
	info->datalen = 0;
	return 0;
}


static int vt8500_device_ready(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	return readb(info->reg + VT8500_NFC_MISC_STAT_PORT) & 0x01;
}


static void vt8500_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	if (mode == hardware_ecc)
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) & 0xfb, info->reg + VT8500_NFC_MISC_CTRL);
	else
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x04, info->reg + VT8500_NFC_MISC_CTRL);
}

static void print_register(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	printk(KERN_NOTICE "\r NFC_MISC_STAT_PORT: %8.8x\n", readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
	printk(KERN_NOTICE "\r NFC_DMA_COUNTER: %8.8x\n", readw(info->reg + VT8500_NFC_DMA_COUNTER));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_GCR   0x100 %8.8x\n", readl(info->reg + NFC_DMA_GCR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_IER   0x104 %8.8x\n", readl(info->reg + NFC_DMA_IER));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_ISR   0x108 %8.8x\n", readl(info->reg + NFC_DMA_ISR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_DESPR 0x10C %8.8x\n", readl(info->reg + NFC_DMA_DESPR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_RBR   0x110 %8.8x\n", readl(info->reg + NFC_DMA_RBR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_DAR   0x114 %8.8x\n", readl(info->reg + NFC_DMA_DAR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_BAR   0x118 %8.8x\n", readl(info->reg + NFC_DMA_BAR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_CPR   0x11C %8.8x\n", readl(info->reg + NFC_DMA_CPR));
	printk(KERN_NOTICE "\rDMA Register: NFC_DMA_CCR   0x120 %8.8x\n", readl(info->reg + NFC_DMA_CCR));
	printk(KERN_NOTICE "\rECC Register: ECC MODE(23)  0x08C %8.8x\n",
	readl(info->reg + VT8500_NFC_ECC_BCH_CTRL));
	printk(KERN_NOTICE "\rECC Register: OOB MODE(9)   0x024 %8.8x\n",
	readl(info->reg + VT8500_NFC_SMC_ENABLE));

	printk(KERN_NOTICE "\rHarming ECC Register: VT8500_NFC_REDUNT_ECC_STAT(0x7C) %8.8x\n",
	readb(info->reg + VT8500_NFC_REDUNT_ECC_STAT));
	printk(KERN_NOTICE "\rHarming ECC Register: VT8500_NFC_BANK18_ECC_STAT(0x80) %8.8x\n",
	readl(info->reg + VT8500_NFC_BANK18_ECC_STAT));

	printk(KERN_NOTICE "\rBCH ECC Register: VT8500_NFC_ECC_BCH_CTRL(0x8c) %8.8x\n",
	readl(info->reg + VT8500_NFC_ECC_BCH_CTRL));
	printk(KERN_NOTICE "\rBCH ECC Register: VT8500_NFC_ECC_BCH_INT_STAT1(0x94) %8.8x\n",
	readl(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1));
	printk(KERN_NOTICE "\rBCH ECC Register: VT8500_NFC_ECC_BCH_INT_STAT2(0x98) %8.8x\n",
	readl(info->reg + VT8500_NFC_ECC_BCH_INT_STAT2));
	printk(KERN_NOTICE "\rBCH ECC Register: VT8500_NFC_ECC_BCH_ERR_POS1(0x9c) %8.8x\n",
	readl(info->reg + VT8500_NFC_ECC_BCH_ERR_POS1));
	printk(KERN_NOTICE "\rBCH ECC Register: VT8500_NFC_ECC_BCH_ERR_POS2(0xa0) %8.8x\n",
	readl(info->reg + VT8500_NFC_ECC_BCH_ERR_POS2));

	printk(KERN_NOTICE "\rNFC Register: VT8500_NFC_NAND_TYPE_SEL(0x48) %8.8x\n",
	readl(info->reg + VT8500_NFC_NAND_TYPE_SEL));
	printk(KERN_NOTICE "\rNFC Register: VT8500_NFC_MISC_CTRL(0x54) %8.8x\n",
	readl(info->reg + VT8500_NFC_MISC_CTRL));
	printk(KERN_NOTICE "\rNFC Register: VT8500_NFC_PAGESIZE_DIVIDER_SEL(0x5c) %8.8x\n",
	readl(info->reg + VT8500_NFC_PAGESIZE_DIVIDER_SEL));
}


/*
 * vt8500_nand_cmdfunc - Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void vt8500_nand_cmdfunc(struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned int addr_cycle = 0, b2r_stat;
	int status = -1;
	unsigned int ecc_err_pos, bank_stat, redunt_stat, bank_stat1;
	#if (NAND_PAGE_SIZE  == 512)
	int readcmd, bank_stat1; /*add by vincent 20080805*/ /*Vincent 2008.11.3*/
	#endif
	int mycolumn = column, mypage_addr = page_addr; /*add by vincent 20080805*/
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "enter in vt8500_nand_cmdfunc() command: %x column:%x, page_addr:%x\n",
	command, column, page_addr);
	#endif
	/*printk(KERN_NOTICE "\rVT8500_NFC_MISC_STAT_PORT: %x\n",
	readb(info->reg + VT8500_NFC_MISC_STAT_PORT));*/
	/* Emulate NAND_CMD_READOOB and oob layout need to deal with specially */


#if 1

	if (command == NAND_CMD_READOOB || command == NAND_CMD_READ0) {

	#if (NAND_PAGE_SIZE == 512)
	#ifdef PAGE_READ_COUNTER
read_again:
	#endif
	#endif
	/* bug patch in 4bit or 8bit ecc engine when read two page consectively with error*/
	/* and the error is data area error(page 2) follows redundant area error(page 1)*/
	/* it will report wrong bank error, need to read again*/
	#if (NAND_PAGE_SIZE == 4096 || NAND_PAGE_SIZE == 2048)
read_page_again:
	addr_cycle = 0;
	column = mycolumn;
	page_addr = mypage_addr;
	#endif

		if (command == NAND_CMD_READOOB) {
			#ifdef NAND_DEBUG
			printk(KERN_NOTICE "in vt8500_nand_cmdfunc(): readoob column %x\n", column);
			#endif
			/*  memcpy(info->dmabuf + info->datalen, 0x00, 64);*/
			#ifndef NAND_HARMING_ECC
			if (ecc_type == 1) {
				#if (NAND_PAGE_SIZE == 2048)
					/*vt8500_nfc_dma_cfg(mtd, 16, 0, -1, -1);*/
				/*printk(KERN_NOTICE "
				(ecc_type = 1 BCH support): page size 2048 readoob column %x\n", column);*/
				#else /*if (NAND_PAGE_SIZE == 4096)*/
					/*vt8500_nfc_dma_cfg(mtd, 32, 0, -1, -1);*/
					/*printk(KERN_NOTICE " (ecc_type = 1 BCH support):
					page size 4096 readoob column %x\n", column);*/
				#endif
			}	else {
			#endif
				/*printk(KERN_NOTICE
				"(ecc_type = 0 HARMING support): readoob column %x\n", column);*/
				/* redunt_read_hm_ecc_ctrl(info, 1);*/
				#if (NAND_PAGE_SIZE == 2048)
				column += mtd->writesize;
				vt8500_nfc_dma_cfg(mtd, 64, 0, -1, -1);
				#else

					#if (NAND_PAGE_SIZE == 4096)
				column += mtd->writesize;
				vt8500_nfc_dma_cfg(mtd, 128, 0, -1, -1);
					#else /* page_size == 512 */
				/*  chip enable:  enable CE0*/
				/* write to clear B2R */
				b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
				writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);
				/* printk(KERN_NOTICE "READOOB: RB is %d\n", b2r_stat & 0x02);*/

				writeb(0xfe, info->reg + VT8500_NFC_CHIP_ENABLE_CTRL);  /* select CE0 */

				/* printk(KERN_NOTICE "read oob--in vt8500_nand_cmdfunc(): config dma\n");*/
						#ifdef PAGE_READ_COUNTER
				/* not use DMA, but must config transfer length */
				writew(16 - 1, info->reg + VT8500_NFC_DMA_COUNTER);
				writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) | 0x02,
				info->reg + VT8500_NFC_SMC_ENABLE);
						#else
				vt8500_nfc_dma_cfg(mtd, 16, 0, -1, -1);
						#endif
				/* info->datalen = mtd->writesize;*/
					#endif

				#endif

			#ifndef NAND_HARMING_ECC
			}
			#endif
		} else {
		/* nand cmd read data area */
			#if (NAND_PAGE_SIZE == 512)
			/* write to clear B2R */
			b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
			writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);
			/* chip enable:  enable CE0*/
			writeb(0xfe, info->reg + VT8500_NFC_CHIP_ENABLE_CTRL);
			#endif

			#ifdef PAGE_READ_COUNTER
			if (command == NAND_CMD_READ0)
				vt8500_nfc_dma_cfg(mtd, 256, 0, -1, -1);  /* 1: read, 0:data, -1: */
			else
				vt8500_nfc_dma_cfg(mtd, 256, 0, -1, 0xff);  /* 1: read, 0:data, -1: */
			#else
			/*printk(KERN_NOTICE "config dma: read page_addr:%x\n", page_addr);*/
			/* 1: read, 0:data, -1:  */
			/*printk(KERN_NOTICE "read data area column %x writesize %x\n", column, mtd->writesize);*/
			vt8500_nfc_dma_cfg(mtd, mtd->writesize, 0, -1, -1);
			#endif
		}

		info->datalen = 0;
		/* write to clear B2R */
		b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		/* printk(KERN_NOTICE "RB is %d\n", b2r_stat & 0x02);*/

		if (column != -1) {
			writeb(column, info->reg + VT8500_NFC_COMPORT1_2);
			addr_cycle++;
			#ifndef PAGE_ADDR
			writeb(column >> 8, (unsigned char *)(info->reg + VT8500_NFC_COMPORT1_2) + 1);
			addr_cycle++;
			#endif
			if (page_addr != -1) {
				#ifndef PAGE_ADDR
				writeb(page_addr, info->reg + VT8500_NFC_COMPORT3_4);
				page_addr >>= 8;
				writeb(page_addr, (unsigned char *)(info->reg + VT8500_NFC_COMPORT3_4) + 1);
				addr_cycle += 2;
				#else
				writeb(page_addr, (unsigned char *)(info->reg + VT8500_NFC_COMPORT1_2) + 1);
				page_addr >>= 8;
				writeb(page_addr, info->reg + VT8500_NFC_COMPORT3_4);
				addr_cycle += 2;
				#endif

				#if (NAND_PAGE_SIZE == 2048)
				/* One more address cycle for devices > 128MiB */
				if (chip->chipsize > (128 << 20)) {
				#else
					#if (NAND_PAGE_SIZE == 4096)
				/* One more address cycle for devices > 256MiB */
				if (chip->chipsize > (256 << 20)) {
					#else
				/* One more address cycle for devices > 32MiB */
				if (chip->chipsize > (32 << 20)) {
					#endif
				#endif
					page_addr >>= 8;
					#ifndef PAGE_ADDR
					writeb(page_addr, info->reg + VT8500_NFC_COMPORT5_6);
					#else
					writeb(page_addr,
					(unsigned char *)(info->reg + VT8500_NFC_COMPORT3_4) + 1);
					#endif
					addr_cycle++;
				}
			}
		/* } else if (page_addr != -1) {*/
		} else if ((page_addr != -1) && (column == -1)) {
			writeb(page_addr & 0xff, info->reg + VT8500_NFC_COMPORT1_2);
			page_addr >>= 8;
			writeb(page_addr & 0xff, (unsigned char *)(info->reg + VT8500_NFC_COMPORT1_2) + 1);
			addr_cycle += 2;

			#if (NAND_PAGE_SIZE == 2048)
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20)) {
			#else
				#if (NAND_PAGE_SIZE == 4096)
			/* One more address cycle for devices > 256MiB */
			if (chip->chipsize > (256 << 20)) {
				#else
			/* One more address cycle for devices > 32MiB */
			if (chip->chipsize > (32 << 20)) {
				#endif
			#endif
				/* One more address cycle for devices > 128MiB */
			/* if (chip->chipsize > (128 << 20)) {*/
			page_addr >>= 8;
			/*  writeb(page_addr,
			info->reg + VT8500_NFC_COMPORT3_4 + 1); */
			/* before, may be a little error */
			writeb(page_addr & 0xff,
			info->reg + VT8500_NFC_COMPORT3_4);
			addr_cycle++;
		}
	} /* end of if (command == NAND_CMD_READOOB || command == NAND_CMD_READ0) { */
	#ifdef NAND_HARMING_ECC /* HAMMing ECC */
	writeb(0x07, info->reg + VT8500_NFC_REDUNT_ECC_STAT);
	writel(0xffffffff, info->reg + VT8500_NFC_BANK18_ECC_STAT);
	#else  /*Vincent 2008.11.3*/
	bank_stat1 = readw(info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
	writew(bank_stat1|0x101, info->reg + VT8500_NFC_ECC_BCH_INT_STAT1);
	#endif

	#if (NAND_PAGE_SIZE == 512)
	/* printk(KERN_NOTICE "pagesize=512 command mode\n");*/
	/* printk(KERN_NOTICE "command is %x\n", command);*/
	writeb(command, info->reg + VT8500_NFC_COMPORT0);
	writeb(NAND2NFC | ((addr_cycle + 1)<<1)|NFC_TRIGGER,
	info->reg + VT8500_NFC_COMCTRL);

	status = vt8500_nand_ready(mtd);
	if (status)	{
		printk(KERN_ERR "nand flash is not ready : %x\n",
		readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
		/*print_register(mtd);*/
		writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd,
		info->reg + VT8500_NFC_SMC_ENABLE);
		/* return;*/
	}
	status = vt8500_nfc_transfer_ready(mtd);
	/*status = vt8500_wait_dma_ready(mtd);*/
	if (status) {
		printk(KERN_ERR "wait transfer command and data is not finished : %x\n",
		readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
		/*print_register(mtd);*/
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd,
		info->reg + VT8500_NFC_SMC_ENABLE);
		/*  while (1);*/
		/*  printk(KERN_NOTICE "dma transfer is not ready\n");*/
		/*  return;*/
	}
	/*  chip disable:  disable CE*/
	/* printk(KERN_NOTICE "Disable CE0-------------------\n");*/
	/* writeb(0xff, info->reg + VT8500_NFC_CHIP_ENABLE_CTRL);*/  /* disable CE0 */

	status = vt8500_nfc_wait_idle(mtd, 1, -1, -1, -1);
	if (status) {
		printk(KERN_ERR "wait transfer data and nfc is not idle : %x\n",
		readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
		/*print_register(mtd);*/
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd,
		info->reg + VT8500_NFC_SMC_ENABLE);
		/* return;*/
	}
	/* disable CE0 */
	writeb(0xff, info->reg + VT8500_NFC_CHIP_ENABLE_CTRL);


	#else /* else of page size = 512 */
	/* printk(KERN_NOTICE "\r ECC OOB MODE(9)0x024 %8.8x\n",
	readl(info->reg + VT8500_NFC_SMC_ENABLE));*/
	#ifdef NAND_HARMING_ECC/*Vincent 2008.11.4*/

	/*printk( "page2k/4k, command =0x%x\n", command);*/
	status = vt8500_wait_chip_ready(mtd); /*Vincent 2008.11.3*/
	if (status)
		printk(KERN_ERR "The chip is not ready\n");

	writeb(NAND_CMD_READ0, info->reg + VT8500_NFC_COMPORT0);
	writeb(DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER,
	info->reg + VT8500_NFC_COMCTRL);
	/* wait all command + address sequence finish status */
	status = vt8500_wait_cmd_ready(mtd);
	/* status = vt8500_nfc_ready(mtd); */
	if (status)	{
		/* printk(KERN_ERR "wait transfer command is not ready : %x\n",
		readb(info->reg + VT8500_NFC_MISC_STAT_PORT));*/
		/*print_register(mtd);*/
		/* writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);*/
		/* while (1);*/
		printk(KERN_NOTICE "dma transfer is not ready 2k or 4k page\n");
		/* return;*/
	}
	/* wait device idle 1: don't check ecc*/
	status = vt8500_nfc_wait_idle(mtd, 1, -1, -1, -1);
	/*print_register(mtd);*/
	if (status)	{
		printk(KERN_ERR "wait transfer command and nfc is not idle : %x\n",
		readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
		/*print_register(mtd);*/
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/* while (1);*/
		/* printk(KERN_NOTICE "dma transfer is not ready\n");*/
		/* return;*/
	}

	/* write to clear B2R */
	b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

	writeb(NAND_CMD_READSTART, info->reg + VT8500_NFC_COMPORT0);
	writeb(NAND2NFC|(1<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);
	/*print_register(mtd);*/
	/*wait busy to ready int status*/
	status = vt8500_nand_ready(mtd);
	if (status) {
		printk(KERN_ERR "readstart: nand flash is not ready\n");
		/*print_register(mtd);*/
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/* while (1);*/
		/* return;*/
	}
	vt8500_wait_nfc_ready(info); /*Vincent 2008.11.3*/
	/* printk(KERN_NOTICE "\rreadstart: nand flash is ready\n");*/
	#if 0
	status = vt8500_nfc_transfer_ready(mtd);
	if (status)	{
		printk(KERN_ERR "wait transfer data is not ready: %x\n",
		readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
		print_register(mtd);
		while (1)
			;
		return;
	}
	#endif

	#ifdef NAND_BBT_BCH_ECC
	if (ecc_type == 0) {
	#endif
		/* use HAMMING ECC */
		/*printk(KERN_ERR "use HAMMing to read command = %x \n", command);*/
		status = vt8500_nfc_wait_idle(mtd, 2, command, mycolumn, mypage_addr);

	#ifdef NAND_BBT_BCH_ECC
	}	else {
		/* use BCH ECC */
		/*print_register(mtd);*/
		/*printk(KERN_ERR "use BCH to read command = %x \n", command);*/
		status = vt8500_nfc_wait_idle(mtd, 0, command, mycolumn, mypage_addr);
		if (command == NAND_CMD_READOOB) {
			/* disable side info read operation */
			/* disable_redunt_out_bch_ctrl(info, 0);*/
			writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd,
			info->reg + VT8500_NFC_SMC_ENABLE);
			/* writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) &0xFFFFFFFD,
			info->reg + VT8500_NFC_CALC_CTRL);*/ /*Vincent 2008.11.3*/
		}
	}
	#endif

	if (status) {
		if (status == -4)
			return;
		printk(KERN_ERR "vt8500_nfc_wait_idle status =%d\n", status);
		printk(KERN_ERR "command =0x%x\n", command);
		printk(KERN_ERR "Read ERR ,NFC is not idle\n");
		print_register(mtd);
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/* while (1);*/
		return;
	}

	/*print_register(mtd);*/
	if (!(readw(info->reg + VT8500_NFC_SMC_ENABLE)&2)) {
		status = nand_pdma_handler(mtd);
		/*printk(KERN_ERR "check status pdma handler status= %x \n", status);*/
		nand_free_pdma(mtd);
		if (status)
			return;
	}

	#else /* Vincent 2008.11.4 #else of #ifdef NAND_HARMING_ECC */


	status = vt8500_wait_chip_ready(mtd); /*Vincent 2008.11.3*/
	if (status)
		printk(KERN_ERR "The chip is not ready\n");
	writeb(NAND_CMD_READ0, info->reg + VT8500_NFC_COMPORT0);
	if (addr_cycle == 4)
		writeb(NAND_CMD_READSTART, info->reg + VT8500_NFC_COMPORT5_6);
	else if (addr_cycle == 5)
		writeb(NAND_CMD_READSTART, (unsigned char *)(info->reg + VT8500_NFC_COMPORT5_6) + 1);

	writeb(NAND2NFC|MUL_CMDS|((addr_cycle + 2)<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);


	status = vt8500_nand_ready(mtd);
	if (redunt_err_mark == 2) {
		redunt_err_mark = 3;
		disable_redunt_out_bch_ctrl(info, 1); /* disable redundant output */
	}

	if (status) {
		printk(KERN_ERR "readstart: nand flash is not ready\n");
		/*print_register(mtd);*/
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/* while(1);*/
		/* return;*/
	}
	vt8500_wait_nfc_ready(info); /*Vincent 2008.11.3*/
	/*Vincent 2008.11.14*/
	/*if (command == NAND_CMD_READOOB) {
		status = vt8500_wait_dma_ready(mtd);
		if (status) {
			printk(KERN_ERR "dma transfer data is not ready: %x\n",
			readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
		}
	}*/
	/* print_register(mtd);*/
	/*if (command != NAND_CMD_READOOB) {
		for (j = 0; j < 0xA8; j += 16)
			printk(KERN_NOTICE "NFCR%x ~ NFCR%x = 0x%8.8x 0x%8.8x 0x%8.8x 0x%8.8x\r\n",
			j/4, (j+12)/4,
			readl(info->reg + j + 0),
			readl(info->reg + j + 4),
			readl(info->reg + j + 8),
			readl(info->reg + j + 12));
	} else if(mypage_addr == 0x343fb) {
		for (j = 0; j < 0xA8; j += 16)
			printk(KERN_NOTICE "NFCR%x ~ NFCR%x = 0x%8.8x 0x%8.8x 0x%8.8x 0x%8.8x\r\n",
			j/4, (j+12)/4,
			readl(info->reg + j + 0),
			readl(info->reg + j + 4),
			readl(info->reg + j + 8),
			readl(info->reg + j + 12));
	}*/
	status = vt8500_nfc_wait_idle(mtd, 0, command, mycolumn, mypage_addr);
	if (command == NAND_CMD_READOOB) {
		/* disable_redunt_out_bch_ctrl(info, 0);*/
		/* writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) &0xFFFFFFFD ,
		info->reg + VT8500_NFC_CALC_CTRL);*/ /*Vincent 2008.11.3*/
	}

	if (status) {
		if (status == -4)
			return;
		printk(KERN_ERR "vt8500_nfc_wait_idle status =%d\n", status);
		printk(KERN_ERR "command =0x%x\n", command);
		printk(KERN_ERR "Read ERR ,NFC is not idle\n");
		/*print_register(mtd);*/
		writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
		info->reg + VT8500_NFC_ECC_BCH_CTRL);
		/*while(1);*/
		return;
	}

	if (!(readw(info->reg + VT8500_NFC_SMC_ENABLE)&2)) {
		status = nand_pdma_handler(mtd);
		/*printk(KERN_ERR "check status pdma handler status= %x \n", status);*/
		nand_free_pdma(mtd);
		if (status)
			printk(KERN_ERR "dma transfer data time out: %x\n",
			readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
	}

	if (redunt_err_mark == 2) {
		/*redunt_err_mark = 0;*/
		/*printk(KERN_ERR "go to read page again\n");*/
		goto read_page_again;
	}
	if(redunt_err_mark == 3) {
		redunt_err_mark = 0;
		disable_redunt_out_bch_ctrl(info, 0); /* Enable redundant output */
	}

	#endif/*Vincent 2008.11.4*/

	#endif /* end of #else of page size = 512 */

	#if (NAND_PAGE_SIZE == 512)
	#ifdef PAGE_READ_COUNTER
	addr_cycle = 0;
	if (command == NAND_CMD_READ0) {
		command = NAND_CMD_READ1;
		goto read_again;
	}	else if (command == NAND_CMD_READ1) {
		command = NAND_CMD_READOOB;
		goto read_again;
	}
	#endif
	#endif


	#ifdef NAND_BBT_BCH_ECC
	if (ecc_type == 0) {
	#endif
		#if (NAND_PAGE_SIZE != 512)
		if (command == NAND_CMD_READOOB) {
		#endif
			/* use HAMMING ECC but page not 512 and read oob area */
			#ifdef NAND_DEBUG
			printk(KERN_NOTICE "in vt8500_nand_cmdfunc(): Read oob data \n");
			#endif
			redunt_stat = readb(info->reg + VT8500_NFC_REDUNT_ECC_STAT);

			if (redunt_stat) {
				printk(KERN_NOTICE
				" Read OOB redundant ecc eror command: %x column:%x, page_addr:%x\n",
				command, mycolumn, mypage_addr);
				printk(KERN_NOTICE "redunt_stat:%x\n", redunt_stat);
			}
			if (redunt_stat & 0x05) {
					printk(KERN_ERR "There are uncorrected ecc error in reduntant area--\n");
					mtd->ecc_stats.failed++;
					return;
			} else if (redunt_stat & 0x02) {
				#ifdef NAND_DEBUG
				printk(KERN_WARNING "There are 1 bit ecc error in reduntant data area--\n");
				#endif
				/* Vincent  modify 2008.10.13*/
				ecc_err_pos = readw(info->reg + VT8500_NFC_REDUNT_AREA_PARITY_STAT);
				bit_correct((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f),
				(ecc_err_pos >> 8) & 0x07);
				/* mtd->ecc_stats.corrected++;*/
			}

		#if (NAND_PAGE_SIZE != 512)
		}	else {
		#endif
			/* read data area with hamming ecc correct */
			/* use HAMMING ECC but page not 512 and read data area */
			#ifdef NAND_DEBUG
			printk(KERN_NOTICE "in vt8500_nand_cmdfunc(): Read data \n");
			#endif

			bank_stat = readl(info->reg + VT8500_NFC_BANK18_ECC_STAT);
			redunt_stat = readb(info->reg + VT8500_NFC_REDUNT_ECC_STAT);
			/* memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, 64);*/
			if (bank_stat || redunt_stat) {
				printk(KERN_NOTICE " Read data ecc eror command: %x column:%x, page_addr:%x\n",
				command, column, page_addr);
				printk(KERN_NOTICE "error block addr: 0x%x page_addr:0x%x\n",
				mypage_addr>>6, mypage_addr&0x3F);
				printk(KERN_NOTICE " bank_stat:0x%x, redunt_stat:0x%x\n",
				bank_stat, redunt_stat);
			}
			nand_hamming_ecc_1bit_correct(mtd);

		#if (NAND_PAGE_SIZE != 512)
		}
		#endif

	#ifdef NAND_BBT_BCH_ECC
	}
	#endif
	return;
	}
#endif /* end of #if 1 */

	switch (command) {
	case NAND_CMD_SEQIN:
		/*add by vincent 20080805*/
		#if (NAND_PAGE_SIZE  == 512)
		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		writeb(readcmd, info->reg + VT8500_NFC_COMPORT0);
		writeb(DPAHSE_DISABLE | (1<<1) | NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);
		vt8500_wait_nfc_ready(info);
		#endif

	/*add by vincent 20080805*/
	case NAND_CMD_ERASE1:
		/* printk(KERN_NOTICE "command is %x\n", command);*/
		if (column != -1) {
			writeb(column, info->reg + VT8500_NFC_COMPORT1_2);
			addr_cycle++;
			#ifndef PAGE_ADDR
			writeb(column >> 8, (unsigned char *)(info->reg + VT8500_NFC_COMPORT1_2) + 1);
			addr_cycle++;
			#endif
			if (page_addr != -1) {
				#ifndef PAGE_ADDR
				writeb(page_addr, info->reg + VT8500_NFC_COMPORT3_4);
				page_addr >>= 8;
				writeb(page_addr, (unsigned char *)(info->reg + VT8500_NFC_COMPORT3_4) + 1);
				addr_cycle += 2;
				#else
				writeb(page_addr, (unsigned char *)(info->reg + VT8500_NFC_COMPORT1_2) + 1);
				page_addr >>= 8;
				writeb(page_addr, info->reg + VT8500_NFC_COMPORT3_4);
				addr_cycle += 2;
				#endif

				#if (NAND_PAGE_SIZE == 2048)
				/* One more address cycle for devices > 128MiB */
				if (chip->chipsize > (128 << 20)) {
				#else
					#if (NAND_PAGE_SIZE == 4096)
					/* One more address cycle for devices > 256MiB */
				if (chip->chipsize > (256 << 20)) {
					#else
					/* One more address cycle for devices > 32MiB */
				if (chip->chipsize > (32 << 20)) {
					#endif
				#endif
					/* One more address cycle for devices > 128MiB */
				/* if (chip->chipsize > (128 << 20)) {*/
			page_addr >>= 8;
			#ifndef PAGE_ADDR
			writeb(page_addr, info->reg + VT8500_NFC_COMPORT5_6);
			#else
			writeb(page_addr, (unsigned char *)(info->reg + VT8500_NFC_COMPORT3_4) + 1);
			#endif
			addr_cycle++;
				}
			}
		/*} else if (page_addr != -1) {*/
		} else if ((page_addr != -1) && (column == -1)) {
			writeb(page_addr & 0xff, info->reg + VT8500_NFC_COMPORT1_2);
			page_addr >>= 8;
			writeb(page_addr & 0xff, (unsigned char *)(info->reg + VT8500_NFC_COMPORT1_2) + 1);
			addr_cycle += 2;

			#if (NAND_PAGE_SIZE == 2048)
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20)) {
			#else
				#if (NAND_PAGE_SIZE == 4096)
			/* One more address cycle for devices > 256MiB */
			if (chip->chipsize > (256 << 20)) {
				#else
				/* One more address cycle for devices > 32MiB */
			if (chip->chipsize > (32 << 20)) {
				#endif
			#endif
				/* One more address cycle for devices > 128MiB */
				/* if (chip->chipsize > (128 << 20)) {*/
				page_addr >>= 8;
				/* writeb(page_addr, info->reg + VT8500_NFC_COMPORT3_4 + 1);*/
				/* before, may be a little error */
				writeb(page_addr, info->reg + VT8500_NFC_COMPORT3_4);
				addr_cycle++;
			}
		}

		/* set command 1 cycle */
		writeb(command, info->reg + VT8500_NFC_COMPORT0);
		if (command == NAND_CMD_SEQIN)
			writeb(((addr_cycle + 1)<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);
		else {
			/* writeb(read(info->reg + VT8500_NFC_NAND_TYPE_SEL) | WP_DISABLE ,
			info->reg + VT8500_NFC_NAND_TYPE_SEL);*/
			writeb(DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER,
			info->reg + VT8500_NFC_COMCTRL);
		}

		if (command == NAND_CMD_ERASE1) {
			status = vt8500_wait_cmd_ready(mtd);
			/* status = vt8500_nfc_ready(mtd); */
			if (status)
					printk(KERN_ERR "command is not ready\n");
					writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
					info->reg + VT8500_NFC_ECC_BCH_CTRL);
		}	else {
			/* vt8500_nfc_transfer_ready(mtd);*/
			#ifdef NAND_HARMING_ECC
			status = 0;
			status = vt8500_nfc_transfer_ready(mtd);
			#else /*Vincent 2008.11.4*/
			vt8500_wait_nfc_ready(info);
			status = vt8500_nfc_transfer_ready(mtd);
			/*status = vt8500_wait_dma_ready(mtd);*/ /*dannier mask*/
			#endif
			if (status)	{
				printk(KERN_ERR "dma transfer data is not ready: %x\n",
				readb(info->reg + VT8500_NFC_MISC_STAT_PORT));
				writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
				info->reg + VT8500_NFC_ECC_BCH_CTRL);
				/*printk(KERN_NOTICE "\rwait transfer data is not ready: %x\n",
				readb(info->reg + VT8500_NFC_MISC_STAT_PORT));*/
				/*print_register(mtd);*/
				/* while (1);*/
				/* return;*/
			}
			/* if (command == NAND_CMD_SEQIN)*/
			/* vt8500_wait_dma_ready(mtd);*/
		}
		return;

	case NAND_CMD_PAGEPROG:
		/* case NAND_CMD_READSTART:*/
	case NAND_CMD_ERASE2:
		/* printk(KERN_NOTICE "command is %x\n", command);*/
		writeb(command, info->reg + VT8500_NFC_COMPORT0);
		b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		status = vt8500_wait_chip_ready(mtd); /*Vincent 2008.11.3*/
		if (status)
			printk(KERN_NOTICE"The chip is not ready\n");
		/* if (command == NAND_CMD_READSTART)*/
		/* writeb(NAND2NFC|(1<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);*/
		/* else*/
		writeb(DPAHSE_DISABLE|(1<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);

		info->datalen = 0;
		status = vt8500_nand_ready(mtd);
		if (status) {
			printk(KERN_ERR "program or erase: nand flash is not ready\n");
			writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
			info->reg + VT8500_NFC_ECC_BCH_CTRL);
			/*print_register(mtd);*/
			/*  while (1);*/
			/*    return;*/
		}
		#if 0  /* for debug */
		if (command == NAND_CMD_ERASE2) {
			vt8500_read_nand_status(mtd, NAND_CMD_STATUS);
			if ((readb(info->reg + VT8500_NFC_DATAPORT) & 0xff) == 0xc0) {
				printk(KERN_NOTICE "vt8500_func: erase block OK\n");
				printk(KERN_NOTICE "read nand status is %x\n",
				readb(info->reg + VT8500_NFC_DATAPORT) & 0xff);
			}	else
				printk(KERN_NOTICE "vt8500_func: erase block failed\n");
		}
		#endif

		status = vt8500_nfc_wait_idle(mtd, 1, 1, -1, -1); /* write page, don't check ecc */
		if (status < 0) {
			printk(KERN_ERR "page program or erase err, nand controller is not idle\n");
			/*print_register(mtd);*/
			/* writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
			info->reg + VT8500_NFC_ECC_BCH_CTRL);*/
			/* while (1);*/
			/* return;*/
			/* }*/
			#if 0
			status = vt8500_read_nand_status(mtd, NAND_CMD_STATUS);
			if (status < 0)
				printk(KERN_NOTICE "\rNFC or NAND is not ready\n");
			else if (status & NAND_STATUS_FAIL)
				printk(KERN_NOTICE "\r status : fail\n");
			else if (!(status & NAND_STATUS_READY))
				printk(KERN_NOTICE "\r status : busy\n");
			else if (!(status & NAND_STATUS_WP))
				printk(KERN_NOTICE "\r status : protect\n");
			#endif
			return;
		}

		return;

	case NAND_CMD_RESET:

		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		writeb(command, info->reg + VT8500_NFC_COMPORT0);
		/* write to clear B2R */
		b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

		writeb(DPAHSE_DISABLE|(0x01<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);
		status = vt8500_nand_ready(mtd);
		if (status) {
			printk(KERN_ERR "Reset err, nand device is not ready\n");
			writew(readw(info->reg + VT8500_NFC_ECC_BCH_CTRL) | 0x100,
			info->reg + VT8500_NFC_ECC_BCH_CTRL);
		}

		vt8500_read_nand_status(mtd, NAND_CMD_STATUS);
		/*  while (!(chip->read_byte(mtd) & NAND_STATUS_READY));*/
		while (!(readb(info->reg + VT8500_NFC_DATAPORT) & 0xff) & NAND_STATUS_READY)
			;
		#ifdef NAND_DEBUG
		printk(KERN_NOTICE "Reset status is ok\n");
		#endif
		return;

	case NAND_CMD_READID:

		status = vt8500_nand_readID(mtd);
		#ifdef NAND_DEBUG
		printk(KERN_NOTICE "readID status is %d\n", status);
		#endif
		return;

	case NAND_CMD_STATUS:

		vt8500_read_nand_status(mtd, command);
		return;

	case NAND_CMD_RNDIN:
		#if 1
		if (column != -1) {
			writeb(column, info->reg + VT8500_NFC_COMPORT1_2);
			writeb(column, info->reg + VT8500_NFC_COMPORT1_2 + 1);
			addr_cycle += 2;
		}
		#endif

		/* set command 1 cycle */
		writeb(command, info->reg + VT8500_NFC_COMPORT0);

		writeb(DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);

		status = vt8500_nfc_wait_idle(mtd, 1, -1, -1, -1); /* don't check ecc, wait nfc idle */
		/*  status = vt8500_wait_cmd_ready(mtd);*/
		/* status = vt8500_nfc_ready(mtd);*/
		if (status)
			printk(KERN_ERR "Ramdom input err: nfc is not idle\n");

		return;

	case NAND_CMD_RNDOUT:

		if (column != -1) {
			writeb(column, info->reg + VT8500_NFC_COMPORT1_2);
			writeb(column, info->reg + VT8500_NFC_COMPORT1_2 + 1);
			addr_cycle += 2;
		}

		/* CLEAR ECC BIT */
		writeb(0x07, info->reg + VT8500_NFC_REDUNT_ECC_STAT);
		writel(0xffffffff, info->reg + VT8500_NFC_BANK18_ECC_STAT);
		/* write to clear B2R */
		b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

		/* set command 1 cycle */
		writeb(command, info->reg + VT8500_NFC_COMPORT0);

		writeb(DPAHSE_DISABLE|((addr_cycle + 1)<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);

		status = vt8500_wait_cmd_ready(mtd);
		/* status = vt8500_nfc_ready(mtd);*/
		if (status) {
			printk(KERN_ERR "Ramdom output err: nfc command is not ready\n");
			/* return;*/
		}

		writeb(NAND_CMD_RNDOUTSTART, info->reg + VT8500_NFC_COMPORT0);
		/* write to clear B2R */
		b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
		writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

		writeb(NAND2NFC|(1<<1)|NFC_TRIGGER, info->reg + VT8500_NFC_COMCTRL);

		status = vt8500_wait_cmd_ready(mtd);
		/* status = vt8500_nand_ready(mtd);*/
		if (status) {
			printk(KERN_ERR "Ramdom output err: nfc io transfer is not finished\n");
			/* return;*/
		}
		/* reduntant aera check ecc, wait nfc idle */
		status = vt8500_nfc_wait_idle(mtd, 0, -1, -1, -1);
		/* status = vt8500_nand_wait_idle(mtd);*/
		if (status)
			printk(KERN_ERR "Ramdom output err: nfc is not idle\n");
		return;


	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
		udelay(chip->chip_delay);
		return;


	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */

		/* trigger command and addrress cycle */

		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}
	/* Apply this short delay always to ensure that we do wait tWB in */
	/* any case on any machine.*/
	/* ndelay(100);*/
	vt8500_device_ready(mtd);
}


static void vt8500_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	unsigned int b2r_stat;
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "\r enter in vt8500_nand_select_chip()\n");
	#endif
	if (chipnr > 1)
		printk(KERN_WARNING "There are only support two chip sets\n");

	b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

	/* select CE0 */
	writeb(~(1<<chipnr), info->reg + VT8500_NFC_CHIP_ENABLE_CTRL);
	/* select CE1 */
	/* writeb(~(2<<chipnr), info->reg + VT8500_NFC_CHIP_ENABLE_CTRL); */
}

void nand_hamming_ecc_1bit_correct(struct mtd_info *mtd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	unsigned int ecc_err_pos, bank_stat, redunt_stat;

	/* use HAMMING ECC but page not 512 and read data area */
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "in vt8500_nand_cmdfunc(): Read data \n");
	#endif

	bank_stat = readl(info->reg + VT8500_NFC_BANK18_ECC_STAT);
	redunt_stat = readb(info->reg + VT8500_NFC_REDUNT_ECC_STAT);

	if (bank_stat & 0x5555) {
		printk(KERN_ERR "There are uncorrected ecc error in data area--\n");
		mtd->ecc_stats.failed++;
		return;
	} else if (redunt_stat & 0x05) {
		printk(KERN_ERR "There are uncorrected ecc error in reduntant area--\n");
		mtd->ecc_stats.failed++;
		return;
	#if (NAND_PAGE_SIZE == 2048)
	} else if (bank_stat & 0x2222) {
	#else
		#if (NAND_PAGE_SIZE == 4096)
	} else if (bank_stat & 0x22222222) {
		#else
	} else if (bank_stat & 0x22) {
		#endif
	#endif
		/*#ifdef NAND_DEBUG*/
		/*printk(KERN_NOTICE
		"There are 1 bit ecc error in data area----at column = %d,	page_addr = %d\n",
		mycolumn, mypage_addr);*/
		/*#endif*/

		/*bank_sel = readb(info->reg + VT8500_NFC_MISC_CTRL);*/
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) & 0xfc, info->reg + VT8500_NFC_MISC_CTRL);

	if (bank_stat & 0x02) {
		ecc_err_pos = readw(info->reg + VT8500_NFC_ODD_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 1 error BYTE: %x bit:%x\n",
		ecc_err_pos & 0x1ff, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[ecc_err_pos & 0x1ff]);
		bit_correct(&info->dmabuf[ecc_err_pos & 0x1ff], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n", *(uint8_t *)&info->dmabuf[ecc_err_pos & 0x1ff]);
		/* mtd->ecc_stats.corrected++;*/
	} else if (bank_stat & 0x20) {
		ecc_err_pos = readw(info->reg + VT8500_NFC_EVEN_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 2 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+512, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[512 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[512 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[512 + (ecc_err_pos & 0x1ff)]);
		/* mtd->ecc_stats.corrected++;*/
	}
	#if (NAND_PAGE_SIZE == 2048)
	writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x1, info->reg + VT8500_NFC_MISC_CTRL);

	if (bank_stat & 0x0200) {
		ecc_err_pos = readw(info->reg + VT8500_NFC_ODD_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 3 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+1024, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
		/* mtd->ecc_stats.corrected++;*/
	} else if (bank_stat & 0x2000) {
		ecc_err_pos = readw(info->reg + VT8500_NFC_EVEN_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 4 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+1536, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
		/* mtd->ecc_stats.corrected++;*/
	}
	#endif

	#if (NAND_PAGE_SIZE == 4096)
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x1 , info->reg + VT8500_NFC_MISC_CTRL);

	if (bank_stat & 0x0200) {
		ecc_err_pos = readw(info->reg + VT8500_NFC_ODD_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 3 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+1024, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[1024 + (ecc_err_pos & 0x1ff)]);
		/*   mtd->ecc_stats.corrected++;*/
	} else if (bank_stat & 0x2000) {
		ecc_err_pos = readw(info->reg + VT8500_NFC_EVEN_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 4 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+1536, (ecc_err_pos >> 9) & 0x7);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[1536 + (ecc_err_pos & 0x1ff)]);
	} else if (bank_stat & 0x020000) {
		writeb((readb(info->reg + VT8500_NFC_MISC_CTRL) & 0xFC) | 0x2,
		info->reg + VT8500_NFC_MISC_CTRL);
		ecc_err_pos = readw(info->reg + VT8500_NFC_ODD_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 5 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+2048, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[2048 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[2048 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[2048 + (ecc_err_pos & 0x1ff)]);
	} else if (bank_stat & 0x200000) {
		writeb((readb(info->reg + VT8500_NFC_MISC_CTRL) & 0xFC) | 0x2,
		info->reg + VT8500_NFC_MISC_CTRL);
		ecc_err_pos = readw(info->reg + VT8500_NFC_EVEN_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 6 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+2560, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[2560 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[2560 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[2560 + (ecc_err_pos & 0x1ff)]);
	} else if (bank_stat & 0x02000000) {
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x3, info->reg + VT8500_NFC_MISC_CTRL);
		ecc_err_pos = readw(info->reg + VT8500_NFC_ODD_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 7 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+3072, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[3072 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[3072 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[3072 + (ecc_err_pos & 0x1ff)]);
	} else if (bank_stat & 0x20000000) {
		writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x3, info->reg + VT8500_NFC_MISC_CTRL);
		ecc_err_pos = readw(info->reg + VT8500_NFC_EVEN_BANK_PARITY_STAT);
		printk(KERN_NOTICE "bank 8 error BYTE: %x bit:%x\n",
		(ecc_err_pos & 0x1ff)+3584, (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*(uint8_t *)&info->dmabuf[3584 + (ecc_err_pos & 0x1ff)]);
		bit_correct(&info->dmabuf[3584 + (ecc_err_pos & 0x1ff)], (ecc_err_pos >> 9) & 0x07);
		printk(KERN_NOTICE "correct value :%x\n",
		*(uint8_t *)&info->dmabuf[3584 + (ecc_err_pos & 0x1ff)]);
	}
	#endif

	} else if (redunt_stat & 0x02) {
		/* printk(KERN_WARNING "There are 1 bit ecc error in reduntant data area--\n");*/
		/* memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, 64);*/
		ecc_err_pos = readw(info->reg + VT8500_NFC_REDUNT_AREA_PARITY_STAT);
		printk(KERN_NOTICE "oob area error BYTE: %x bit:%x\n",
		ecc_err_pos & 0x3f, (ecc_err_pos >> 8) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f)));
		bit_correct((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f),
		(ecc_err_pos >> 8) & 0x07);
		printk(KERN_NOTICE "error value :%x\n",
		*((unsigned char *)info->reg+ECC_FIFO_0 + (ecc_err_pos & 0x3f)));
		/* mtd->ecc_stats.corrected++;*/
	}
}
static void vt8500_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "enter in vt8500_nand_write_buf()\n");
	#endif
	memcpy(info->dmabuf + info->datalen, buf, len);

	info->datalen += len;
}

static void vt8500_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "enter in vt8500_nand_read_buf() len: %x infoDatalen :%x\n", len, info->datalen);
	#endif

	memcpy(buf, info->dmabuf + info->datalen, len);
	info->datalen += len;
}

static uint8_t vt8500_read_byte(struct mtd_info *mtd)
{
	/* struct vt8500_nand_mtd *nmtd = mtd->priv;*/
	/* struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);*/
	uint8_t d;
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "enter in vt8500_nand_read_byte()\n");
	#endif

	/* d = readb(info->reg + VT8500_NFC_DATAPORT) & 0xff;*/
	 vt8500_nand_read_buf(mtd, &d, 1);
	/* via_dev_dbg(&nmtd->info->platform->dev, "Read %02x\n", d);*/
	/* via_dev_dbg(info->platform->dev, "Read %02x\n", d);*/

	return d;
}

static int vt8500_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip, int page, int sndcmd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	uint8_t *buf = chip->oob_poi;
	/* int length = mtd->oobsize;  */ /* prepad = chip->ecc.prepad, bytes = chip->ecc.bytes;*/
	/* int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;*/
	/* int eccsize = chip->ecc.size;*/
	uint8_t *bufpoi = buf;
	/* struct nand_oobfree *free = chip->ecc.layout->oobfree;*/
	/* uint32_t boffs;*/
	/* int pos;   */ /* toread, sndrnd = 1;*/

	#ifndef NAND_HARMING_ECC
	/*int i;*/
	int pos;   /* toread, sndrnd = 1;*/
	int eccsize = chip->ecc.size;
	/*int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;*/
	#endif

	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "\r enter in vt8500_nand_read_oob()\n");
	#endif
	/* info->datalen = mtd->writesize;*/  /* oob data is placed in after info->dmabuf[2047]  */

	#ifdef NAND_HARMING_ECC
	/*info->datalen = 0;*/
	/* pos = mtd->writesize;*/
	/* chip->cmdfunc(mtd, NAND_CMD_READOOB, pos, page);*/
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	/* #if (NAND_PAGE_SIZE == 512)*/
	/*       ;*/
	/* writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) | 0x02, info->reg + VT8500_NFC_SMC_ENABLE);*/
	/* memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, mtd->oobsize);*/
	/* #else*/
	chip->read_buf(mtd, bufpoi, mtd->oobsize);
	/* #endif*/

	#else
	/*  for (i = 0; i < chip->ecc.steps; i++) {*/
	/*for (i = 0; i < 4; i++) {*/
	writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) | 0x2,
	info->reg + VT8500_NFC_SMC_ENABLE);
	pos = (eccsize + chip->ecc.bytes) * chip->ecc.steps;/*+ i * (eccsize + chunk);*/
	chip->cmdfunc(mtd, NAND_CMD_READOOB, pos, page);
	/*chip->read_buf(mtd, bufpoi, 32);*/
	memcpy(bufpoi, info->reg+ECC_FIFO_0, mtd->oobsize);
	writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd,
	info->reg + VT8500_NFC_SMC_ENABLE);
		/*chip->read_buf(mtd, bufpoi + i * 16, 16);*/
	/*}*/
	#endif

	return 1;
}


/*
 * vt8500_nand_read_bb_oob - OOB data read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @sndcmd:	flag whether to issue read command or not
 */
static int vt8500_nand_read_bb_oob(struct mtd_info *mtd, struct nand_chip *chip,
int page, int sndcmd)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "\r enter in vt8500_nand_read_bb_oob()\n");
	#endif
	/* disable hardware ECC  */
	writeb(readb(info->reg + VT8500_NFC_MISC_CTRL) | 0x04, info->reg + VT8500_NFC_MISC_CTRL);
	ecc_type = 0;
	nfc_ecc_set(info, 0);   /* off hardware ecc  */
	set_ecc_engine(info, 0);  /* harming ECC structure for bad block check*/
	/* chip->ecc.layout = &vt8500_hm_oobinfo_2048;*/

	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	nfc_ecc_set(info, 1);   /* on hardware ecc  */
	#ifndef NAND_HARMING_ECC
	ecc_type = 1;
	if (ECC8BIT_ENGINE == 1)
		set_ecc_engine(info, 2);  /* BCH ECC structure 8bit ecc engine*/
	else
		set_ecc_engine(info, 1);  /* BCH ECC structure 4bit ecc engine*/

	#else
		#if (NAND_PAGE_SIZE != 512)
		set_ecc_engine(info, 0);  /* harming ECC structure for bad block check*/
		#endif

	#endif
	return sndcmd;
}



static int vt8500_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	#ifndef NAND_HARMING_ECC
	/*int i;*/
	unsigned int b2r_stat;
	/*int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;*/
	int eccsize = chip->ecc.size; /* length = mtd->oobsize;  */
	/* prepad = chip->ecc.prepad, bytes = chip->ecc.bytes;*/
	#endif

	int pos, status = 0;
	/*int steps = chip->ecc.steps;*/  /* Vincent 2008.11.4*/
	const uint8_t *bufpoi = chip->oob_poi;
	/* struct nand_oobfree *free = chip->ecc.layout->oobfree;*/
	/* uint32_t boffs;*/
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "\r enter in vt8500_nand_write_oob()\n");
	#endif
	/*
	 * data-ecc-data-ecc ... ecc-oob
	 * or
	 * 512  7     1     5    0    3
	 * data-ecc-prepad-data-pad-oobecc ....
	 */

	/* info->datalen = mtd->writesize; */ /* oob data is placed in after info->dmabuf[2047]  */
#ifdef NAND_HARMING_ECC
	info->datalen = 0;
	pos = mtd->writesize;
	chip->write_buf(mtd, bufpoi, mtd->oobsize);
	vt8500_nfc_dma_cfg(mtd, mtd->oobsize, 1, 2, -1);  /* 64 or 16 bytes   */
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos, page);
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
#else
	/* 	for (i = 0; i < steps; i++) {*/
	/*for (i = 0; i < 4; i++) {*/
	b2r_stat = readb(info->reg + VT8500_NFC_HOST_STAT_CHANGE);
	writeb(B2R|b2r_stat, info->reg + VT8500_NFC_HOST_STAT_CHANGE);

	info->datalen = 0;
	/*chip->write_buf(mtd, bufpoi, 32);*/
	memcpy(info->reg+ECC_FIFO_0, bufpoi, 32);
	pos = eccsize * chip->ecc.steps + 8*4;
	/*pos = eccsize + i * (eccsize + chunk);*/
	/*vt8500_nfc_dma_cfg(mtd, 32, 1, 1, i);*/
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos, page);

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	/* printk(KERN_NOTICE "\r in vt8500_nand_write_oob_new(): waitfunc_1\n");*/
	status = chip->waitfunc(mtd, chip);
	/* printk(KERN_NOTICE "\r in vt8500_nand_write_oob_new(): waitfunc_2\n");*/
	if (status & NAND_STATUS_FAIL)
		return -EIO;
	/* } */
	return 0;
#endif
}


/**
 * vt8500_nand_read_page - hardware ecc syndrom based page read
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * The hw generator calculates the error syndrome automatically. Therefor
 * we need a special oob layout and handling.
 */
static int vt8500_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
						 uint8_t *buf)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);

	#ifdef NAND_DEBUG
		printk(KERN_NOTICE "\r enter in vt8500_nand_read_page()\n");
	#endif
	info->datalen = 0;
	chip->read_buf(mtd, buf, mtd->writesize);
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "\r enter in nand_read_page(): mtd->writesize is %d and oobsize is %d\n",
	mtd->writesize, mtd->oobsize);
	#endif
	/* memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, mtd->oobsize);*/
	/* memset(chip->oob_poi, 0xff, mtd->oobsize);*/
	#if (NAND_PAGE_SIZE == 2048)
	/* writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7,
	info->reg + VT8500_NFC_CALC_CTRL);*/
	memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, mtd->oobsize);
	#else /* dannier test 0x34 are used or not when not hamming mode*/
		#if (NAND_PAGE_SIZE == 4096)
			writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7,
			info->reg + VT8500_NFC_CALC_CTRL);
			memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, 64);
			#ifdef NAND_HARMING_ECC
			writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) | 0x08,
			info->reg + VT8500_NFC_CALC_CTRL);
			memcpy(chip->oob_poi+64, info->reg+ECC_FIFO_0, 64);
			#endif
		#else   /* pagesize = 512 */
			/* only reduntant area read enable */
			memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, mtd->oobsize);
			#ifndef PAGE_READ_COUNTER
				writeb(readb(info->reg + VT8500_NFC_SMC_ENABLE) & 0xfd,
				info->reg + VT8500_NFC_SMC_ENABLE);
			#endif
			/*   memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, mtd->oobsize);*/
		#endif
	#endif

	#if 0
	/* for debug  */
	int i;
	/* printk(KERN_NOTICE "data aera is -------------------------\n");*/
	/* for (i = 0; i < mtd->writesize; i += 4) {*/
	/* printk(KERN_NOTICE "%x %x %x %x\n",
	info->dmabuf[i], info->dmabuf[i+1], info->dmabuf[i+2], info->dmabuf[i+3]);*/
	/* }*/
	printk(KERN_NOTICE "spare aera is -------------------------\n");
	for (i = 0; i < mtd->oobsize; i += 4)
		printk(KERN_NOTICE "%x %x %x %x\n",
		chip->oob_poi[i], chip->oob_poi[i+1], chip->oob_poi[i+2], chip->oob_poi[i+3]);
	#endif

 /*   memcpy(chip->oob_poi, info->reg+ECC_FIFO_0, mtd->oobavail);*/
 /*   chip->read_buf(mtd, chip->oob_poi, mtd->oobavail); */  /* ????  */
	return 0;
}

/**
 *  vt8500_nand_write_page_lowlevel - hardware ecc syndrom based page write
 *  @mtd:    mtd info structure
 *  @chip:  nand chip info structure
 *  @buf:  data buffer
 *
 *  The hw generator calculates the error syndrome automatically. Therefor
 *  we need a special oob layout and handling.
 *
 */
static void vt8500_nand_write_page_lowlevel(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t *buf)
{
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "enter in vt8500_nand_page_write_lowlevel() writesize %x\n", mtd->writesize);
	#endif
	info->datalen = 0;
	chip->write_buf(mtd, buf, mtd->writesize);
	vt8500_nfc_dma_cfg(mtd, mtd->writesize, 1, 0, -1);  /*  2048bytes  */

	/* for debug  */
	#if 0
	int i;
	/* printk(KERN_NOTICE "data aera is -------------------------\n");*/
	/* for (i = 0; i < mtd->writesize; i += 4) {*/
	/* printk(KERN_NOTICE "%x %x %x %x\n",
	info->dmabuf[i], info->dmabuf[i+1], info->dmabuf[i+2], info->dmabuf[i+3]);*/
	/* info->dmabuf[i] = 0;*/
	/* info->dmabuf[i+1] = 0;*/
	/* info->dmabuf[i+2] = 0;*/
	/* info->dmabuf[i+3] = 0;*/
	/*  }*/
	printk(KERN_NOTICE "spare aera is -------------------------\n");
	for (i = 0; i < mtd->oobsize; i += 4)
		printk(KERN_NOTICE "%x %x %x %x\n",
		chip->oob_poi[i], chip->oob_poi[i+1], chip->oob_poi[i+2], chip->oob_poi[i+3]);
	#endif

	#if (NAND_PAGE_SIZE == 2048)
	/* writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7, info->reg + VT8500_NFC_CALC_CTRL);*/
	memcpy(info->reg+ECC_FIFO_0, chip->oob_poi, mtd->oobsize);
	/* solve a hardware bug --- bank 3, byte 7, bit 7 error  */
	/*     writel(0xfeffffff, info->reg + ECC_FIFO_e);*/
	#else
		#if (NAND_PAGE_SIZE == 4096)
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) & 0xF7,
		info->reg + VT8500_NFC_CALC_CTRL);
		memcpy(info->reg+ECC_FIFO_0, chip->oob_poi, 64);
			#ifdef NAND_HARMING_ECC
		writeb(readb(info->reg + VT8500_NFC_CALC_CTRL) | 0x08,
		info->reg + VT8500_NFC_CALC_CTRL);
		memcpy(info->reg+ECC_FIFO_0, chip->oob_poi+64, 64);
			#endif
		/* solve a hardware bug --- bank 7, byte 7, bit 7 error  */
		/* writel(0xfeffffff, info->reg + ECC_FIFO_e);*/
		#else
		memcpy(info->reg+ECC_FIFO_0, chip->oob_poi, mtd->oobsize);
		#endif
	#endif

	/* memcpy(info->reg+ECC_FIFO_0, chip->oob_poi, mtd->oobavail);*/
	/* chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);*/
}


static int vt8500_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
const uint8_t *buf, int page, int cached, int raw)
{
	int status;
	#ifdef NAND_BBT_BCH_ECC
	struct vt8500_nand_info *info = vt8500_nand_mtd_toinfo(mtd);
	#endif
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "enter in vt8500_nand_write_page()\n");
	printk(KERN_NOTICE "raw = %d, and ecc_type = %d\n", raw, ecc_type);
	#endif


	#ifdef NAND_BBT_BCH_ECC
	if (raw == 1 && ecc_type == 1) {  /* nand old structure  */
		/*#ifdef NAND_DEBUG*/
		printk(KERN_NOTICE "old structure: enter in vt8500_nand_write_page()\n");
		/*#endif*/
		ecc_type = 0;
		set_ecc_engine(info, 0);  /* old structure for file system write*/
		#if (NAND_PAGE_SIZE == 2048)
			chip->ecc.layout = &vt8500_hm_oobinfo_2048;
		#else
			chip->ecc.layout = &vt8500_hm_oobinfo_4096;
		#endif
	}	else if (raw == 0 && ecc_type == 0) {  /* nand new structure  */
		/*#ifdef NAND_DEBUG*/
		printk(KERN_NOTICE "new structure: enter in vt8500_nand_write_page()\n");
		/*#endif*/
		ecc_type = 1;
		if (ECC8BIT_ENGINE == 1)
		set_ecc_engine(info, 2); /* new structure for bad block check*/
		else
		set_ecc_engine(info, 1); /* new structure for bad block check*/

		#if (NAND_PAGE_SIZE == 2048)
			chip->ecc.layout = &vt8500_oobinfo_2048;
		#else
		if (ECC8BIT_ENGINE == 1)
			chip->ecc.layout = &vt8500_8bit_oobinfo_4096;
		else
			chip->ecc.layout = &vt8500_oobinfo_4096;

		#endif
	}
	#endif


	#if 0
	#ifndef NAND_HARMING_ECC
	if (raw == 1 && ecc_type == 0) {  /* nand new structure  */
		#ifdef NAND_DEBUG
		printk(KERN_NOTICE "New structure: enter in vt8500_nand_write_page()\n");
		#endif
		ecc_type = 1;
		if (ECC8BIT_ENGINE == 1)
		set_ecc_engine(info, 2);
		else
		set_ecc_engine(info, 1);  /* new structure */

		chip->ecc.layout = &vt8500_oobinfo_2048;
	}	else if (raw == 0 && ecc_type == 1) {  /* nand old structure  */
		#ifdef NAND_DEBUG
		printk(KERN_NOTICE "Old structure: enter in vt8500_nand_write_page()\n");
		#endif
		ecc_type = 0;
		set_ecc_engine(info, 0);  /* old structure for bad block check*/
		chip->ecc.layout = &vt8500_hm_oobinfo_2048;
	}
	#endif
	#endif

	/*printk(KERN_NOTICE "vt8500_nand_write_page() write begin\n");*/
	chip->ecc.write_page(mtd, chip, buf);
		/*   }*/
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	/*
	 * *   * Cached progamming disabled for now, Not sure if its worth the
	 * *       * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 * *           */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		/*
		* * See if operation failed and additional status checks are
		* * available
		* *      */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,	page);

		if (status & NAND_STATUS_FAIL)
			return -EIO;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}


	#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	if (chip->verify_buf(mtd, buf, mtd->writesize))
		return -EIO;
	#endif
	return 0;
}


/* vt8500_nand_init_chip
 *
 * init a single instance of an chip
 */

static void vt8500_nand_init_chip(struct vt8500_nand_info *info, struct vt8500_nand_mtd *nmtd)
{
	struct nand_chip *chip = &nmtd->chip;

	#if 0
	chip->cmdfunc      = vt8500_nand_cmdfunc;
	chip->dev_ready    = vt8500_device_ready;
	chip->read_byte    = vt8500_read_byte;
	chip->write_buf    = vt8500_nand_write_buf;
	chip->read_buf     = vt8500_nand_read_buf;
	chip->select_chip  = vt8500_nand_select_chip;
	chip->chip_delay   = 20;
	chip->priv	   = nmtd;
	chip->options	   = 0;
	chip->controller   = &info->controller;
	#endif

	/* chip->cmd_ctrl  = vt8500_nand_hwcontrol;*/
	#if 0
	switch (info->cpu_type) {
	case TYPE_vt8500:
		break;

	case TYPE_vt8620:
		break;

	case TYPE_vt8610:
		break;
	}
	#endif

	/* nmtd->set	   = set;*/
	if (hardware_ecc) {
		/*	chip->ecc.calculate = vt8500_nand_calculate_ecc;*/
		/*	chip->ecc.correct   = vt8500_nand_correct_data;*/

		#if (NAND_PAGE_SIZE == 2048)
		chip->ecc.size      = 512;
		chip->ecc.bytes     = 8;
		chip->ecc.steps     = 4;
		/*chip->ecc.layout    = &vt8500_oobinfo_2048;*/
		chip->ecc.prepad    = 1;
		chip->ecc.postpad   = 8;
		#else
			#if (NAND_PAGE_SIZE == 4096)
			chip->ecc.size      = 512;
			if (ECC8BIT_ENGINE == 1)
			chip->ecc.bytes     = 16;
			else
			chip->ecc.bytes     = 8;

			chip->ecc.steps     = 8;
			/*chip->ecc.layout    = &vt8500_oobinfo_4096;*/
			chip->ecc.prepad    = 1;
			chip->ecc.postpad   = 8;

			#else    /*  512 page   */
			chip->ecc.size      = 512;
			chip->ecc.bytes      = 3;
			chip->ecc.steps     = 1;
			/*chip->ecc.layout    = &vt8500_oobinfo_512;*/
			chip->ecc.prepad    = 4;
			chip->ecc.postpad   = 9;
			#endif
		#endif


		chip->write_page = vt8500_nand_write_page;
		chip->ecc.write_page = vt8500_nand_write_page_lowlevel;
		chip->ecc.write_oob = vt8500_nand_write_oob;
		chip->ecc.read_page = vt8500_nand_read_page;
		chip->ecc.read_oob = vt8500_nand_read_oob;

		chip->ecc.read_bb_oob = vt8500_nand_read_bb_oob;

		/*	switch (info->cpu_type) {*/
		/*	case TYPE_vt8500:*/
		chip->ecc.hwctl	    = vt8500_nand_enable_hwecc;
		/*	chip->ecc.calculate = vt8500_nand_calculate_ecc;*/
		/*	break;*/
	#if 0
	case TYPE_vt8620:
		chip->ecc.hwctl     = vt8620_nand_enable_hwecc;
		chip->ecc.calculate = vt86203_nand_calculate_ecc;
		break;

	case TYPE_vt8610:
		chip->ecc.hwctl     = vt8610_nand_enable_hwecc;
		chip->ecc.calculate = vt8610_nand_calculate_ecc;
		break;
		}
	#endif
	} else
		chip->ecc.mode	    = NAND_ECC_SOFT;
}


static int vt8500_nand_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform(dev);
	struct vt8500_nand_info *info = dev_get_drvdata(&pdev->dev);

	/*  struct mtd_info *mtd = dev_get_drvdata(pdev);*/
	dev_set_drvdata(&pdev->dev, NULL);
	/*  platform_set_drvdata(pdev, NULL);*/
	/*  dev_set_drvdata(pdev, NULL);*/
	if (info == NULL)
		return 0;

	/* first thing we need to do is release all our mtds
	 * and their partitions, then go through freeing the
	 * resources used
	 */

	if (info->mtds != NULL) {
		struct vt8500_nand_mtd *ptr = info->mtds;
	/* int mtdno;*/

	/* for (mtdno = 0; mtdno < info->mtd_count; mtdno++, ptr++) {*/
	/*     pr_debug("releasing mtd %d (%p)\n", mtdno, ptr);*/
		nand_release(&ptr->mtd);
	/*  }*/
		kfree(info->mtds);
	}

	/* free the common resources */

	if (info->reg != NULL) {
		iounmap(info->reg);
		info->reg = NULL;
	}

	if (info->area != NULL) {
		release_resource(info->area);
		kfree(info->area);
		info->area = NULL;
	}
	kfree(info);
	return 0;
}



static int vt8500_nand_probe(struct device *dev)
{
	/* struct vt8500_platform_nand *plat = to_nand_plat(pdev);*/
	struct platform_device *pdev = to_platform_device(dev);
	struct vt8500_nand_info *info;
	struct vt8500_nand_mtd *nmtd;
	/*	struct vt8500_nand_set *sets; */ /*  extend more chips and partitions structure*/
	struct resource *res;

	int err = 0;
	int size;
	int maf_id, dev_id;
	/*	int nr_sets;*/
	/*	int setno;*/
	*(volatile unsigned int *)0xd8110200 &= ~(1<<4); /*PIN_SHARE_NOR_NAND*/
	vt8500_version = *(unsigned int *)(0xD8120000);
	/*	printk(KERN_NOTICE "CHIP version is %x\n", vt8500_version);*/
	if (vt8500_version == 0x34000101)
		return -1;  /* A0 chip not support nand flash */
	/*end vt8500_revision: VT3400 A1 and Later...  */

	pr_debug("vt8500_nand_probe(%p)\n", pdev);

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "no memory for flash info\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memzero(info, sizeof(*info));
	dev_set_drvdata(&pdev->dev, info);
	platform_get_resource(pdev, IORESOURCE_MEM, 0);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	/* allocate and map the resource */

	/* currently we assume we have the one resource */
	res  = pdev->resource;
	size = res->end - res->start + 1;

	info->area = request_mem_region(res->start, size, pdev->name);

	if (info->area == NULL) {
		dev_err(&pdev->dev, "cannot reserve register region\n");
		err = -ENOENT;
		goto exit_error;
	}

	info->device     = &pdev->dev;
	/*	info->platform   = plat;*/
	info->reg       = ioremap(res->start, size);
	/*	info->cpu_type   = cpu_type;*/

	if (info->reg == NULL) {
		dev_err(&pdev->dev, "cannot reserve register region\n");
		err = -EIO;
		goto exit_error;
	}

	/* initialise the hardware */

	vt8500_nfc_init(info);
	nfc_ecc_set(info, 1);  /* on hw ecc */

/*
 * * extend more partitions
 *
	err = vt8500_nand_inithw(info, pdev);
		if (err != 0)
		goto exit_error;

	sets = (plat != NULL) ? plat->sets : NULL;
	nr_sets = (plat != NULL) ? plat->nr_sets : 1;

	info->mtd_count = nr_sets;
*/
	/* allocate our information */

/*	size = nr_sets * sizeof(*info->mtds);*/
	size = sizeof(*info->mtds);
	info->mtds = kmalloc(size, GFP_KERNEL);
	if (info->mtds == NULL) {
		dev_err(&pdev->dev, "failed to allocate mtd storage\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memzero(info->mtds, size);

	/* initialise all possible chips */

	nmtd = info->mtds;

	/*	info->dmabuf = dma_alloc_coherent(&pdev->dev, 2112 + sizeof(struct nand_buffers), */
	/*  &info->dmaaddr, GFP_KERNEL);*/
	#if (NAND_PAGE_SIZE == 2048)
	/*info->dmabuf = dma_alloc_coherent(&pdev->dev, 2112, &info->dmaaddr, GFP_KERNEL);*/
	info->dmabuf = dma_alloc_coherent(&pdev->dev, 2112 + 0x300, &info->dmaaddr, GFP_KERNEL);
	#else
		#if (NAND_PAGE_SIZE == 4096)
		info->dmabuf = dma_alloc_coherent(&pdev->dev, 4224 + 0x300, &info->dmaaddr, GFP_KERNEL);
		#else
		info->dmabuf = dma_alloc_coherent(&pdev->dev, 528 + 0x300, &info->dmaaddr, GFP_KERNEL);
		#endif
	#endif
	if (!info->dmabuf && (info->dmaaddr & 0x0f)) {
		err = -ENOMEM;
		goto out_free_dma;
	}
	/*	nmtd->chip.buffers = (void *)info->dmabuf + 2112;*/

	nmtd->chip.cmdfunc      = vt8500_nand_cmdfunc;
	nmtd->chip.dev_ready    = vt8500_device_ready;
	nmtd->chip.read_byte    = vt8500_read_byte;
	nmtd->chip.write_buf    = vt8500_nand_write_buf;
	nmtd->chip.read_buf     = vt8500_nand_read_buf;
	nmtd->chip.select_chip  = vt8500_nand_select_chip;
	nmtd->chip.chip_delay   = 20;
	nmtd->chip.priv	   = nmtd;
	nmtd->chip.options	   = NAND_BBT_LASTBLOCK | NAND_USE_FLASH_BBT | NAND_BBT_PERCHIP;
	/*	nmtd->chip.options	   = 0;*/
	/*	nmtd->chip.controller   = &info->controller;*/

	/*nmtd->chip.ecc.steps     = 1;
		nmtd->chip.ecc.prepad    = 1;
		nmtd->chip.ecc.postpad   = 8;*/

	nmtd->chip.ecc.mode	    = NAND_ECC_HW;
	/*nmtd->chip.ecc.mode	    = 0;*/


	/*	for (setno = 0; setno < nr_sets; setno++, nmtd++)*/
	#ifdef NAND_DEBUG
	printk(KERN_NOTICE "initialising (%p, info %p)\n", nmtd, info);
	#endif
	/*	vt8500_nand_init_chip(info, nmtd, sets);*/

	/* Set up DMA address */
	/*writel(info->dmaaddr & 0xffffffff, info->reg + NFC_DMA_DAR);*/

	/*info->dmabuf = readl(info->reg + VT8500_NFC_DMA_TRANS_CONFIG);*/

	/* nmtd->nand.chip_delay = 0;*/

	/* Enable the following for a flash based bad block table */
	/*	nmtd->nand.options = NAND_USE_FLASH_BBT | NAND_NO_AUTOINCR | NAND_OWN_BUFFERS;*/

#if 1
	#if (NAND_PAGE_SIZE == 512)
	printk(KERN_NOTICE "vt8500_oobinfo_512 \n");
	nmtd->chip.ecc.layout = &vt8500_oobinfo_512;
	nmtd->chip.bbt_td = &vt8500_bbt_main_descr_512;
	nmtd->chip.bbt_md = &vt8500_bbt_mirror_descr_512;
	#else /*if (NAND_PAGE_SIZE == 4096 or 2048)*/
	nmtd->chip.bbt_td = &vt8500_bbt_main_descr_2048;
	nmtd->chip.bbt_md = &vt8500_bbt_mirror_descr_2048;
	#endif
#endif

	/*nmtd->scan_res = nand_scan(&nmtd->mtd, (sets) ? sets->nr_chips : 1);*/

	nmtd->info	   = info;
	nmtd->mtd.priv	   = &nmtd->chip;
	nmtd->mtd.owner    = THIS_MODULE;

	set_ecc_engine(info, 1);
	writeb(0xfe, info->reg + VT8500_NFC_CHIP_ENABLE_CTRL); /* chip0 enable */
	/* Send the command for reading device ID */
	if (!(vt8500_nand_readID(&nmtd->mtd))) {
		/* Read manufacturer and device IDs */
		maf_id = nmtd->chip.read_byte(&nmtd->mtd);
		dev_id = nmtd->chip.read_byte(&nmtd->mtd);
		/* printk("nand chip device maf_id is %x, and dev_id is %x\n", maf_id,dev_id);*/
		if (maf_id == 0x98 && dev_id == 0xd5) {
			ECC8BIT_ENGINE = 1;
			/*NFC_RWTimming = 0x4848;*/
			/*vt8500_nfc_init(info);*/
		}
	}
	writeb(0xff, info->reg + VT8500_NFC_CHIP_ENABLE_CTRL); /*chip disable */
	vt8500_nand_init_chip(info, nmtd);
	#ifndef NAND_HARMING_ECC
	if (ECC8BIT_ENGINE == 1) {
		printk(KERN_NOTICE "BCH ECC 8BIT \n");
		set_ecc_engine(info, 2);  /* BCH ECC new structure */
	} else
		/*printk(KERN_NOTICE "BCH ECC 4BIT \n");*/
		set_ecc_engine(info, 1);  /* BCH ECC new structure */

	#else
	#ifdef NAND_BBT_BCH_ECC
		ecc_type = 1;
		if (ECC8BIT_ENGINE == 1) {
		printk(KERN_NOTICE "BBT BCH ECC 8BIT \n");
		set_ecc_engine(info, 2); /* write bbt with BCH ECC new structure */
	 } else
		/*printk(KERN_NOTICE "BBT BCH ECC 4BIT \n");*/
		set_ecc_engine(info, 1); /* write bbt with BCH ECC new structure */

	#else
		set_ecc_engine(info, 0);  /* Harming ECC  */
	#endif
	#endif

	#if (NAND_PAGE_SIZE == 2048)
		#ifndef NAND_HARMING_ECC
			nmtd->chip.ecc.layout = &vt8500_oobinfo_2048;
		#else
			/* printk(KERN_NOTICE "hm_oob_2048 \n");*/
			#ifdef NAND_BBT_BCH_ECC
			nmtd->chip.ecc.layout = &vt8500_oobinfo_2048;
			#else
			nmtd->chip.ecc.layout = &vt8500_hm_oobinfo_2048;
			#endif
		#endif
	#else /*if (NAND_PAGE_SIZE == 4096)*/
		#if (NAND_PAGE_SIZE == 4096)
			#ifndef NAND_HARMING_ECC
			if(ECC8BIT_ENGINE == 1)
				nmtd->chip.ecc.layout = &vt8500_8bit_oobinfo_4096;
			else
				nmtd->chip.ecc.layout = &vt8500_oobinfo_4096;
			#else
				#ifdef NAND_BBT_BCH_ECC
				if(ECC8BIT_ENGINE == 1)
					nmtd->chip.ecc.layout = &vt8500_8bit_oobinfo_4096;
				else
					nmtd->chip.ecc.layout = &vt8500_oobinfo_4096;
				#else
				nmtd->chip.ecc.layout = &vt8500_hm_oobinfo_4096;
				#endif
			#endif

		#else
			nmtd->chip.ecc.layout = &vt8500_oobinfo_512;
		#endif
	#endif



	nmtd->scan_res = nand_scan(&nmtd->mtd, MAX_CHIP);


	if (nmtd->scan_res == 0) {
		/* vt8500_nand_add_partition(info, nmtd, sets);*/

		#ifndef NAND_HARMING_ECC
		/* set nand flash new structure */
		/* writew(readw(info->reg + VT8500_NFC_ECC_BCH_INT_MASK) | 0x101,
		info->reg + VT8500_NFC_ECC_BCH_INT_MASK);*/

		/* ecc_type = 1;*/
		/* for test */
		#if 0
		/* vt8500_bch_ecc_format_nandflash(&nmtd->mtd, -1);*/
		printk(KERN_NOTICE "formating ok\n");
		read__ecccode_test(&nmtd->mtd, &nmtd->chip, 0);
		#endif
		/* set_ecc_engine(info, nandtype);*/
		/* nmtd->chip.ecc.layout = &vt8500_oobinfo_2048;*/
		#endif

		#ifdef NAND_HARMING_ECC
		ecc_type = 0;
		#if (NAND_PAGE_SIZE == 2048)
		nmtd->chip.ecc.layout = &vt8500_hm_oobinfo_2048;
		#else /*if (NAND_PAGE_SIZE == 4096)*/
			#if (NAND_PAGE_SIZE == 4096)
			nmtd->chip.ecc.layout = &vt8500_hm_oobinfo_4096;
			#endif
		#endif

		*(volatile unsigned long *)PMCEU_ADDR |= (0x0010000);/*add by vincent*/
		set_ecc_engine(info, 0);  /* Harming ECC  */
		*(volatile unsigned long *)PMCEU_ADDR &= ~(0x0010000);/*add by vincent*/
		#endif


		#ifdef CONFIG_MTD_PARTITIONS
			add_mtd_partitions(&nmtd->mtd, nand_partitions, ARRAY_SIZE(nand_partitions));
		#else
			add_mtd_device(&nmtd->mtd);
		#endif
	}

	printk(KERN_NOTICE "nand initialised ok\n");
	return 0;

out_free_dma:
	#if (NAND_PAGE_SIZE == 2048)
	dma_free_coherent(&pdev->dev, 2112 + 0x300, info->dmabuf, info->dmaaddr);
	#else /*if (NAND_PAGE_SIZE == 4096)*/
		#if (NAND_PAGE_SIZE == 4096)
	dma_free_coherent(&pdev->dev, 4224 + 0x300, info->dmabuf, info->dmaaddr);
		#else
	dma_free_coherent(&pdev->dev, 528 + 0x300, info->dmabuf, info->dmaaddr);
		#endif
	#endif
exit_error:
	vt8500_nand_remove(dev);
	/* vt8500_nand_remove(pdev);*/

	if (err == 0)
		err = -EINVAL;
	return err;
}

/* PM Support */
#ifdef CONFIG_PM
int vt8500_nand_suspend(struct device *dev, u32 state)
{
	unsigned int boot_value = GPIO_STRAP_STATUS_VAL;

	/*Judge whether boot from SF in order to implement power self management*/
	if(boot_value & 0x4)
	{
		REG32_VAL(PMCEU_ADDR) |= BIT16;
	}

	printk(KERN_NOTICE "vt8500_nand_suspend\n");

	return 0;
}

int vt8500_nand_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform(dev);
	struct vt8500_nand_info *info = dev_get_drvdata(&pdev->dev);
	*(volatile unsigned long *)PMCEU_ADDR |= (0x0010000);/*add by vincent*/
	if (info) {
		/* initialise the hardware */
		vt8500_nfc_init(info);
		nfc_ecc_set(info, 1);  /* on hw ecc */
		/* Set up DMA address */
		/*writel(info->dmaaddr & 0xffffffff, info->reg + NFC_DMA_DAR);*/

		if (ecc_type == 1) { /* nand new structure  */
			if (ECC8BIT_ENGINE == 1)
				set_ecc_engine(info, 2); /* BCH ECC */
			else
				set_ecc_engine(info, 1); /* BCH ECC */
		} else
			set_ecc_engine(info, 0);  /* Harmming ECC */

		printk(KERN_NOTICE "vt8500_nand_resume OK\n");
	} else
		printk(KERN_NOTICE "vt8500_nand_resume error\n");

	*(volatile unsigned long *)PMCEU_ADDR &= ~(0x0010000);/*add by vincent*/
	return 0;
}

#else /* else of #define PM */
#define vt8500_nand_suspend NULL
#define vt8500_nand_resume NULL
#endif

/*struct platform_driver vt8500_nand_driver = {*/
struct device_driver vt8500_nand_driver = {
	.name	= "nand",
	.bus  = &platform_bus_type,
	.probe = vt8500_nand_probe,
	.remove = vt8500_nand_remove,
	.suspend = vt8500_nand_suspend,
	.resume = vt8500_nand_resume
	/*
	.driiver = {
	.name	= "vt8500-nand",
	.owner	= THIS_MODULE,
	},
	*/
};

static int __init vt8500_nand_init(void)
{
	printk(KERN_NOTICE "NAND Driver, (c) 2008 WMT Electronics ltd.\n");
	return driver_register(&vt8500_nand_driver);
}

static void __exit vt8500_nand_exit(void)
{
	driver_unregister(&vt8500_nand_driver);
}

module_init(vt8500_nand_init);
module_exit(vt8500_nand_exit);

MODULE_DESCRIPTION("Nand Flash Interface Driver");
MODULE_LICENSE("GPL");
