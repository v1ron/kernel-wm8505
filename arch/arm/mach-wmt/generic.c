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

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <linux/power_supply.h>
#include <linux/wm97xx_batt.h>
#include <asm/io.h>

#include "generic.h"

/* TODO*/
#define PMHC_HIBERNATE 0x05
#define LOADER_ADDR												0xffff0000
#define HIBERNATION_ENTER_EXIT_CODE_BASE_ADDR	0xFFFFFFC0
#define DO_DVO_PLL_SET							      (HIBERNATION_ENTER_EXIT_CODE_BASE_ADDR + 0x34)

#ifdef wmt_debug
#define WMprintf(fmt, args...) printk("[%s]: " fmt, __FUNCTION__ , ## args)
#else 
#define WMprintf(fmt, args...)
#endif

int dvo_pll_set (unsigned int multi, unsigned char divisor, unsigned short resx, unsigned short resy)
{
	volatile unsigned int base = 0;
	unsigned int exec_at = (unsigned int)-1;
	int (*theKernel_dvo)(int from, unsigned int multi, unsigned char divisor, unsigned short resx, unsigned short resy);
	int retval = 0;

	WMprintf("entry 0x%8.8X 0x%8.8X %d %d\n",multi, divisor, resx, resy);
	/*enble SF clock*/
	REG32_VAL(PMCEU_ADDR) |= 0x00800000;

	/*jump to loader api to do something*/
	base = (unsigned int)ioremap_nocache(LOADER_ADDR, 0x10000);
	exec_at = base + (DO_DVO_PLL_SET - LOADER_ADDR);
	theKernel_dvo = (int (*)(int from, unsigned int multi, unsigned char divisor, unsigned short resx, unsigned short resy))exec_at;
	
	retval = theKernel_dvo(1, multi, divisor, resx, resy);

	iounmap((void *)base);

	/*disable SF clock*/
	REG32_VAL(PMCEU_ADDR) &= ~0x00800000;

	WMprintf("exit!!(%d)\n",retval);

	return retval;
}
static void wmt_power_off(void)
{
printk("wmt_power_off\n");
#if 1 /*fan*/
    mdelay(100);
    local_irq_disable();

    /*
     * Set scratchpad to zero, just in case it is used as a restart
     * address by the bootloader. Since PB_RESUME button has been
     * set to be one of the wakeup sources, clean the resume address
     * will cause zacboot to issue a SW_RESET, for design a behavior
     * to let PB_RESUME button be a power on button.
     *
     * Also force to disable watchdog timer, if it has been enabled.
     */
    HSP0_VAL = 0;
    OSTW_VAL &= ~OSTW_WE;

    /*
     * Well, I cannot power-off myself,
     * so try to enter power-off suspend mode.
     */
    PMHC_VAL = PMHC_HIBERNATE;
    asm("mcr%? p15, 0, %0, c7, c0, 4" : : "r" (0));		/* Force ARM to idle mode*/
#endif
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

static struct resource wmt_nor_resources[] = {
	[0] = {
		.start  = 0xd8009400,
		.end    = 0xd8009F00,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device wmt_nor_device = {
    .name           = "nor",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(wmt_nor_resources),
    .resource       = wmt_nor_resources,
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
	.dma_mask 		= &wmt_sdmmc_dma_mask,
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
	.dma_mask 		= &wmt_nand_dma_mask,
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
	.dma_mask 		= &wmt_i2s_dma_mask,
	.coherent_dma_mask = ~0,
	},
	.num_resources  = ARRAY_SIZE(wmt_i2s_resources),
	.resource       = wmt_i2s_resources,
};



static struct wm97xx_batt_info wm97xx_batt_pdata = {
        .charge_gpio    = 5,
        .max_voltage    = 4000,
        .min_voltage    = 3000,
        .batt_mult      = 1000,
        .batt_div       = 414,
        .temp_mult      = 1,
        .temp_div       = 1,
        .batt_tech      = POWER_SUPPLY_TECHNOLOGY_LION,
        .batt_name      = "battery",
};


static struct platform_device wmt_battery_device = {
	.name           = "wmt-battery",
	.id             = -1,
	.dev            = {
	.platform_data = &wm97xx_batt_pdata,
	
	},
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
	.dma_mask 		= &wmt_pcm_dma_mask,
	.coherent_dma_mask = ~0,
	},
	.num_resources  = ARRAY_SIZE(wmt_pcm_resources),
	.resource       = wmt_pcm_resources,
};

static struct resource wmt_kpad_resources[] = {
	[0] = {
		.start  = 0xd8260000,
		.end    = 0xd826ffff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_KPAD,
		.end    = IRQ_KPAD,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device wmt_kpad_device = {
	.name			= "kpad",
	.id				= 0,
	.num_resources  = ARRAY_SIZE(wmt_kpad_resources),
	.resource		= wmt_kpad_resources,
};

#ifdef CONFIG_GPIO0_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio_irq_data = {
	.id		= 0,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio_input_device = {
	.name			= "gpio_irq",
	.id				= 0,
	.dev		= {
		.platform_data = &gpio_irq_data,
	},
};
#endif

#ifdef CONFIG_GPIO1_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio1_irq_data = {
	.id		= 1,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio1_input_device = {
	.name			= "gpio_irq",
	.id				= 1,
	.dev		= {
		.platform_data = &gpio1_irq_data,
	},

};
#endif

#ifdef CONFIG_GPIO2_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio2_irq_data = {
	.id		= 2,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio2_input_device = {
	.name			= "gpio_irq",
	.id				= 2,
	.dev		= {
		.platform_data = &gpio2_irq_data,
	},

};
#endif

#ifdef CONFIG_GPIO3_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio3_irq_data = {
	.id		= 3,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio3_input_device = {
	.name			= "gpio_irq",
	.id				= 3,
	.dev		= {
		.platform_data = &gpio3_irq_data,
	},

};
#endif

#ifdef CONFIG_GPIO4_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio4_irq_data = {
	.id		= 4,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio4_input_device = {
	.name			= "gpio_irq",
	.id				= 4,
	.dev		= {
		.platform_data = &gpio3_irq_data,
	},

};
#endif

#ifdef CONFIG_GPIO5_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio5_irq_data = {
	.id		= 5,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio5_input_device = {
	.name			= "gpio_irq",
	.id				= 5,
	.dev		= {
		.platform_data = &gpio3_irq_data,
	},

};
#endif

#ifdef CONFIG_GPIO6_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio6_irq_data = {
	.id		= 6,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio6_input_device = {
	.name			= "gpio_irq",
	.id				= 6,
	.dev		= {
		.platform_data = &gpio3_irq_data,
	},

};
#endif

#ifdef CONFIG_GPIO7_IRQ_WMT
static struct wmt_gpio_irq_platform_data gpio7_irq_data = {
	.id		= 7,
	.rep		= 1,
	.delay		= 100,
	.period		= 100,
	.gpio_irq_mode	= 0,
};

static struct platform_device wmt_gpio7_input_device = {
	.name			= "gpio_irq",
	.id				= 7,
	.dev		= {
		.platform_data = &gpio3_irq_data,
	},

};
#endif

static struct platform_device *wmt_devices[] __initdata = {
    &wmt_uart0_device,
    &wmt_uart1_device,
    &wmt_uart2_device,
    &wmt_uart3_device,
    &wmt_uart4_device,
    &wmt_uart5_device,
    &wmt_sdmmc_device,
    &wmt_sf_device,
    &wmt_nor_device,
    &wmt_nand_device,
    &wmt_i2s_device,
    &wmt_pcm_device,
#ifdef CONFIG_GPIO0_IRQ_WMT
    &wmt_gpio_input_device,
#endif
#ifdef CONFIG_GPIO1_IRQ_WMT
    &wmt_gpio1_input_device,
#endif
#ifdef CONFIG_GPIO2_IRQ_WMT
    &wmt_gpio2_input_device,
#endif
#ifdef CONFIG_GPIO3_IRQ_WMT
    &wmt_gpio3_input_device,
#endif
#ifdef CONFIG_GPIO4_IRQ_WMT
    &wmt_gpio4_input_device,
#endif
#ifdef CONFIG_GPIO5_IRQ_WMT
    &wmt_gpio5_input_device,
#endif
#ifdef CONFIG_GPIO6_IRQ_WMT
    &wmt_gpio6_input_device,
#endif
#ifdef CONFIG_GPIO7_IRQ_WMT
    &wmt_gpio7_input_device,
#endif
    &wmt_kpad_device,
    &wmt_battery_device,
};

static int __init wmt_init(void)
{
        printk("wmt_init\n");
	volatile unsigned int base = 0;
	unsigned int exec_at = (unsigned int)-1;

	base = (unsigned int)ioremap_nocache(LOADER_ADDR, 0x10000);
	exec_at = base + (DO_DVO_PLL_SET - LOADER_ADDR);
	if (!(*(volatile unsigned short *)exec_at))
		BUG();

    pm_power_off = wmt_power_off;
    return platform_add_devices(wmt_devices, ARRAY_SIZE(wmt_devices));
}

arch_initcall(wmt_init);
