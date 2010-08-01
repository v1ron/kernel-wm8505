/*
  linux/arch/arm/mach-wmt/generic.c

  wmt generic architecture level codes
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
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/cpufreq.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include "generic.h"

/* TODO*/
static void wmt_power_off(void)
{
}

static struct resource wmt_uart0_resources[] = {
	[0] = {
		.start  = 0xd8200000,
		.end    = 0xd820ffff,
		.flags  = IORESOURCE_MEM,
	},
};

static struct resource wmt_uart1_resources[] = {
	[0] = {
		.start  = 0xd82b0000,
		.end    = 0xd82bffff,
		.flags  = IORESOURCE_MEM,
	},
};
static struct resource wmt_uart2_resources[] = {
	[0] = {
		.start  = 0xd8210000,
		.end    = 0xd821ffff,
		.flags  = IORESOURCE_MEM,
	},
};

static struct resource wmt_uart3_resources[] = {
	[0] = {
		.start  = 0xd82c0000,
		.end    = 0xd82cffff,
		.flags  = IORESOURCE_MEM,
	},
};

static struct resource wmt_uart4_resources[] = {
	[0] = {
		.start  = 0xd8370000,
		.end    = 0xd837ffff,
		.flags  = IORESOURCE_MEM,
	},
};

static struct resource wmt_uart5_resources[] = {
	[0] = {
		.start  = 0xd8380000,
		.end    = 0xd838ffff,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device wmt_uart0_device = {
    .name           = "uart",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(wmt_uart0_resources),
    .resource       = wmt_uart0_resources,
};

static struct platform_device wmt_uart1_device = {
    .name           = "uart",
    .id             = 1,
    .num_resources  = ARRAY_SIZE(wmt_uart1_resources),
    .resource       = wmt_uart1_resources,
};
static struct platform_device wmt_uart2_device = {
    .name           = "uart",
    .id             = 2,
    .num_resources  = ARRAY_SIZE(wmt_uart2_resources),
    .resource       = wmt_uart2_resources,
};
static struct platform_device wmt_uart3_device = {
    .name           = "uart",
    .id             = 3,
    .num_resources  = ARRAY_SIZE(wmt_uart3_resources),
    .resource       = wmt_uart3_resources,
};
static struct platform_device wmt_uart4_device = {
    .name           = "uart",
    .id             = 4,
    .num_resources  = ARRAY_SIZE(wmt_uart4_resources),
    .resource       = wmt_uart4_resources,
};
static struct platform_device wmt_uart5_device = {
    .name           = "uart",
    .id             = 5,
    .num_resources  = ARRAY_SIZE(wmt_uart5_resources),
    .resource       = wmt_uart5_resources,
};

static struct resource wmt_sf_resources[] = {
	[0] = {
		.start  = 0xd8002000,
		.end    = 0xd80023ff,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device wmt_sf_device = {
    .name           = "sf",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(wmt_sf_resources),
    .resource       = wmt_sf_resources,
};

static struct resource wmt_sdmmc_resources[] = {
	[0] = {
		.start	= 0xd800a000,
		.end	= 0xd800a3FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SDC,
		.end	= IRQ_SDC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_SDC_DMA,
		.end    = IRQ_SDC_DMA,
		.flags  = IORESOURCE_IRQ,
	},
	/*2008/10/6 RichardHsu-s*/
	 [3] = {
	     .start    = IRQ_PMC_WAKEUP,
	     .end      = IRQ_PMC_WAKEUP,
	     .flags    = IORESOURCE_IRQ,
	 },
	 /*2008/10/6 RichardHsu-e*/
};

static u64 wmt_sdmmc_dma_mask = 0xffffffffUL;

static struct platform_device wmt_sdmmc_device = {
	.name			= "sdmmc",
	.id				= 0,
	.dev            = {
		.dma_mask = &wmt_sdmmc_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources  = ARRAY_SIZE(wmt_sdmmc_resources),
	.resource       = wmt_sdmmc_resources,
};

static struct resource wmt_nand_resources[] = {
	[0] = {
		.start  = 0xd8009000,
		.end    = 0xd80093FF,
		.flags  = IORESOURCE_MEM,
	},
};

static u64 wmt_nand_dma_mask = 0xffffffffUL;

static struct platform_device wmt_nand_device = {
	.name           = "nand",
	.id             = 0,
	.dev            = {
		.dma_mask = &wmt_nand_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources  = ARRAY_SIZE(wmt_nand_resources),
	.resource       = wmt_nand_resources,
};

static struct resource wmt_i2s_resources[] = {
    [0] = {
		.start  = 0xD8330000,
		.end    = 0xD833ffff,
		.flags  = IORESOURCE_MEM,
    },
};

static u64 wmt_i2s_dma_mask = 0xffffffffUL;

static struct platform_device wmt_i2s_device = {
	.name           = "i2s",
	.id             = 0,
	.dev            = {
		.dma_mask = &wmt_i2s_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources  = ARRAY_SIZE(wmt_i2s_resources),
	.resource       = wmt_i2s_resources,
};

static struct resource wmt_pcm_resources[] = {
    [0] = {
		.start  = 0xD82D0000,
		.end    = 0xD82Dffff,
		.flags  = IORESOURCE_MEM,
    },
};

static u64 wmt_pcm_dma_mask = 0xffffffffUL;

static struct platform_device wmt_pcm_device = {
	.name           = "pcm",
	.id             = 0,
	.dev            = {
		.dma_mask = &wmt_pcm_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources  = ARRAY_SIZE(wmt_pcm_resources),
	.resource       = wmt_pcm_resources,
};

static struct platform_device *wmt_devices[] __initdata = {
    &wmt_uart0_device,
    &wmt_uart1_device,
    &wmt_uart2_device,
    &wmt_uart3_device,
    &wmt_uart4_device,
    &wmt_uart5_device,
    &wmt_sdmmc_device,
    &wmt_sf_device,
    &wmt_nand_device,
    &wmt_i2s_device,
    &wmt_pcm_device,
};

static int __init wmt_init(void)
{
    pm_power_off = wmt_power_off;
    GPIO_INT_TYPE_VAL = GIRQ_TYPE(2, GIRQ_HIGH);    /* Ethernet preset*/
    return platform_add_devices(wmt_devices, ARRAY_SIZE(wmt_devices));
}

arch_initcall(wmt_init);
