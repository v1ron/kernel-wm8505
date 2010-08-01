/*++
	linux/arch/arm/mach-wmt/irq.c

	IRQ settings for WM8510

	Some descriptions of such software. Copyright (c) 2008 WonderMedia Technologies, Inc.

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
--*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

#include "generic.h"

struct wm8510_irqcfg_t {
	/* IRQ number */
	unsigned char irq;
	/* Routing and enable */
	unsigned char icdc;
};

/*
 * An interrupt initialising table, put GPIOs on the fisrt
 * group. Also route all interrupts to zac_irq decode block.
 * Change the table for future tuning.
 */
/* FIXME:remove it later, james */
static struct wm8510_irqcfg_t wm8510_init_irqcfgs[] __initdata =
{
	{ IRQ_GPIO0,            ICDC_IRQ }, /*6*/
	{ IRQ_GPIO1,            ICDC_IRQ }, /*7*/
	{ IRQ_GPIO2,            ICDC_IRQ }, /*13*/
	{ IRQ_GPIO3,            ICDC_IRQ }, /*14*/
	{ IRQ_GPIO4,            ICDC_IRQ }, /*15*/
	{ IRQ_GPIO5,            ICDC_IRQ }, /*52*/
	{ IRQ_GPIO6,            ICDC_IRQ }, /*53*/
	{ IRQ_GPIO7,            ICDC_IRQ }, /*54*/
	{ IRQ_UHC_F,            ICDC_IRQ }, /*0*/
	{ IRQ_UHC_H,            ICDC_IRQ }, /*1*/
	{ IRQ_UDC_MI_DMATC,     ICDC_IRQ }, /*2*/
	{ IRQ_MOUSE,            ICDC_IRQ }, /*4*/
	{ IRQ_UDC_USBINTR,      ICDC_IRQ }, /*5*/
	{ IRQ_KPAD,             ICDC_IRQ }, /*8*/
	{ IRQ_VDMA,             ICDC_IRQ }, /*9*/
	{ IRQ_ETH0,             ICDC_IRQ }, /*10*/
	{ IRQ_APBB,             ICDC_IRQ }, /*16*/
	{ IRQ_DMA_CH_0,         ICDC_IRQ }, /*17*/
	{ IRQ_I2C1,             ICDC_IRQ }, /*18*/
	{ IRQ_I2C0,             ICDC_IRQ }, /*19*/
	{ IRQ_SDC,              ICDC_IRQ }, /*20*/
	{ IRQ_SDC_DMA,          ICDC_IRQ }, /*21*/
	{ IRQ_PMC_WAKEUP,       ICDC_IRQ }, /*22*/
	{ IRQ_KEYBOARD,         ICDC_IRQ }, /*23*/
	{ IRQ_SPI0,             ICDC_IRQ }, /*24*/
	{ IRQ_SPI1,             ICDC_IRQ }, /*25*/
	{ IRQ_SPI2,             ICDC_IRQ }, /*26*/
	{ IRQ_DMA_CH_1,         ICDC_IRQ }, /*27*/
	{ IRQ_NFC,              ICDC_IRQ }, /*28*/
	{ IRQ_NFC_DMA,          ICDC_IRQ }, /*29*/
	{ IRQ_UART5,            ICDC_IRQ }, /*30*/
	{ IRQ_UART4,            ICDC_IRQ }, /*31*/
	{ IRQ_UART0,            ICDC_IRQ }, /*32*/
	{ IRQ_UART1,            ICDC_IRQ }, /*33*/
	{ IRQ_DMA_CH_2,         ICDC_IRQ }, /*34*/
	{ IRQ_I2S,              ICDC_IRQ }, /*35*/
	{ IRQ_OST0,             ICDC_IRQ }, /*36*/
	{ IRQ_OST1,             ICDC_IRQ }, /*37*/
	{ IRQ_OST2,             ICDC_IRQ }, /*38*/
	{ IRQ_OST3,             ICDC_IRQ }, /*39*/
	{ IRQ_DMA_CH_3,         ICDC_IRQ }, /*40*/
	{ IRQ_DMA_CH_4,         ICDC_IRQ }, /*41*/
	{ IRQ_AC97,             ICDC_IRQ }, /*42*/
	{ IRQ_NOR,              ICDC_IRQ }, /*44*/
	{ IRQ_DMA_CH_5,         ICDC_IRQ }, /*45*/
	{ IRQ_DMA_CH_6,         ICDC_IRQ }, /*46*/
	{ IRQ_UART2,            ICDC_IRQ }, /*47*/
	{ IRQ_RTC1,             ICDC_IRQ }, /*48*/
	{ IRQ_RTC2,             ICDC_IRQ }, /*49*/
	{ IRQ_UART3,            ICDC_IRQ }, /*50*/
	{ IRQ_DMA_CH_7,         ICDC_IRQ }, /*51*/
	{ IRQ_CIR,              ICDC_IRQ }, /*55*/
	{ IRQ_IC1_IRQ0,         ICDC_IRQ }, /*56*/
	{ IRQ_IC1_IRQ1,         ICDC_IRQ }, /*57*/
	{ IRQ_IC1_IRQ2,         ICDC_IRQ }, /*58*/
	{ IRQ_IC1_IRQ3,         ICDC_IRQ }, /*59*/
	{ IRQ_IC1_IRQ4,         ICDC_IRQ }, /*60*/
	{ IRQ_IC1_IRQ5,         ICDC_IRQ }, /*61*/
	{ IRQ_IC1_IRQ6,         ICDC_IRQ }, /*62*/
	{ IRQ_IC1_IRQ7,         ICDC_IRQ }, /*63*/
	{ IRQ_NA0_1,            ICDC_IRQ }, /*65*/
	{ IRQ_NA0_2,            ICDC_IRQ }, /*66*/
	{ IRQ_NA12_0,           ICDC_IRQ }, /*79*/
	{ IRQ_NA12_1,           ICDC_IRQ }, /*80*/
	{ IRQ_NA12_2,           ICDC_IRQ }, /*81*/
	{ IRQ_NA12_3,           ICDC_IRQ }, /*82*/
	{ IRQ_NA12_4,           ICDC_IRQ }, /*83*/
	{ IRQ_NA12_5,           ICDC_IRQ }, /*84*/
	{ IRQ_NA12_6,           ICDC_IRQ }, /*85*/
	{ IRQ_NA12_7,           ICDC_IRQ }, /*86*/
	{ IRQ_DMA_CH_8,         ICDC_IRQ }, /*92*/
	{ IRQ_DMA_CH_9,         ICDC_IRQ }, /*93*/
	{ IRQ_DMA_CH_10,        ICDC_IRQ }, /*94*/
	{ IRQ_DMA_CH_11,        ICDC_IRQ }, /*95*/
	{ IRQ_DMA_CH_12,        ICDC_IRQ }, /*96*/
	{ IRQ_DMA_CH_13,        ICDC_IRQ }, /*97*/
	{ IRQ_DMA_CH_14,        ICDC_IRQ }, /*98*/
	{ IRQ_DMA_CH_15,        ICDC_IRQ }, /*99*/
	{ IRQ_NA12_8,           ICDC_IRQ }, /*111*/
	{ IRQ_NA12_9,           ICDC_IRQ }, /*112*/
	{ IRQ_NA12_10,          ICDC_IRQ }, /*113*/
	{ IRQ_NA12_11,          ICDC_IRQ }, /*114*/
	{ IRQ_NA12_12,          ICDC_IRQ }, /*115*/
};

#define NR_WM8510_IRQS  (sizeof(wm8510_init_irqcfgs)/sizeof(struct wm8510_irqcfg_t))

/*
 * Using to transfer irq number to GPIO index, add GPIO7 later
 */
static const char wm8510_i2g_map[128] =
{
	/*       0   1   2   3   4   5   6   7   8   9 */
	/* 0 */ -1, -1, -1, -1, -1, -1,  0,  1, -1, -1,
	/* 1 */ -1, -1, -1,  2,  3,  4, -1, -1, -1, -1,
	/* 2 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 3 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 4 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 5 */ -1, -1,  5,  6,  7, -1, -1, -1, -1, -1,
	/* 6 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 7 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 8 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 9 */ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 10*/ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 11*/ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	/* 12*/ -1, -1, -1, -1, -1, -1, -1, -1,
};

static const unsigned int wm8510_g2i_map[8] =
{
	IRQ_GPIO0, IRQ_GPIO1, IRQ_GPIO2, IRQ_GPIO3, IRQ_GPIO4,
	IRQ_GPIO5, IRQ_GPIO6, IRQ_GPIO7
};

#define NR_WM8510_GIRQS  (sizeof(wm8510_g2i_map)/sizeof(unsigned int))

static struct wm8510_irq_state_s {
	unsigned int  saved;
	unsigned int  girc;                     /* GPIO Interrupt Request Control register */
	unsigned int  girt;                     /* GPIO Interrupt Request Type register    */
	unsigned int  icpc[ICPC_NUM];           /* 8 decode block                          */
	unsigned char icdc[ICDC_NUM_WM8510];    /* 64 interrupt destination control        */
} wm8510_irq_state;

static int wm8510_gpio_type(unsigned int irq, unsigned int type)
{
	int idx, edge = -1;

	/*
	 * Translate irq number to GPIO index.
	 */
	idx = (int)wm8510_i2g_map[irq];

	/*
	 * irq become index now.
	 */
	if (idx == -1)
		return -EINVAL;

	/*
	 * WM8510 GPIO function can not help to probe.
	 */
	if (type & IRQT_PROBE)        /* IRQT_PROBE = 0x10*/
		return -EINVAL;

	/*
	 * WM8510 GPIO function do not support both edge.
	 */
	if ((type & IRQT_BOTHEDGE) == IRQT_BOTHEDGE)
		return -EINVAL;

	if (type & IRQT_LOW)
		edge = GIRQ_LOW;

	if (type & IRQT_HIGH)
		edge = GIRQ_HIGH;

	if (type & IRQT_RISING)
		edge = GIRQ_RISING;

	if (type & IRQT_FALLING)
		edge = GIRQ_FALLING;

	if (edge != -1) {
		GPIO_INT_REQ_TYPE_VAL &= ~GIRQ_TYPE(idx, GIRQ_TYPEMASK);
		GPIO_INT_REQ_TYPE_VAL |= GIRQ_TYPE(idx, edge);
		wm8510_irq_state.girt = GPIO_INT_REQ_TYPE_VAL;

		/*
		 * Change to level irq_handler if the type is
		 * level senstive.
		 */
		if (type & (IRQT_LOW | IRQT_HIGH))
			set_irq_handler(irq, handle_level_irq);
	}

	return 0;
}

/*
 * Follows are handlers for normal irq_chip
 */
static void wm8510_mask_irq(unsigned int irq)
{
	if (irq < 64)
		ICDC0_VAL(irq) &= ~ICDC_ENABLE;
	else
		ICDC1_VAL(irq - 64) &= ~ICDC_ENABLE;
}

static void wm8510_unmask_irq(unsigned int irq)
{
	if (irq < 64)
		ICDC0_VAL(irq) |= ICDC_ENABLE;
	else
		ICDC1_VAL(irq-64) |= ICDC_ENABLE;
}

static struct irq_chip wm8510_normal_chip = {
	.name   = "normal",
	.ack    = wm8510_mask_irq,
	.mask   = wm8510_mask_irq,
	.unmask = wm8510_unmask_irq,
};

/* wm8510_low_gpio_ack()
 *
 * Low GPIO irqchip handler.
 * This is for IRQs from discontinuous IRQ_GPIO0 to IRQ_GPIO9.
 * Before handling these GPIO IRQs:
 * If this IRQ type is edge triggled, it need to be acknowledged.
 * If this IRQ type is level senstive, it need to be masked.
 */
static void wm8510_low_gpio_ack(unsigned int irq)
{
	int idx;
	unsigned int type;

	/*
	 * Convert irq number to GPIO IRQ index.
	 */
	idx = (int)wm8510_i2g_map[irq];

	if (idx == -1)
		return;

	/*
	 * Get this GPIO irq type.
	 * 0 - low
	 * 1 - high
	 * 2 - falling
	 * 3 - rising
	 */
	type = (GPIO_INT_REQ_TYPE_VAL >> (idx * 2)) & GIRQ_TYPEMASK;

	/*
	 * Edge triggled need ack, level senstive need mask.
	 */
	if (type > 1)
		GPIO_INT_REQ_STATUS_VAL = (1 << idx);
	else {
#ifdef CONFIG_WM8510_IDE2CF
			if (irq < 32)
				*((volatile unsigned long *)0xd8140080) |= (1 << irq);
			else
				*((volatile unsigned long *)0xd8140080) |= (1 << (irq - 32));
#endif
		wm8510_mask_irq(irq);
	}
}

static void wm8510_low_gpio_mask(unsigned int irq)
{
	if (irq < 64)
		ICDC0_VAL(irq) &= ~ICDC_ENABLE;
	else
		ICDC1_VAL(irq-64) &= ~ICDC_ENABLE;
}

static void wm8510_low_gpio_unmask(unsigned int irq)
{
	if (irq < 64)
		ICDC0_VAL(irq) |= ICDC_ENABLE;
	else
		ICDC1_VAL(irq-64) |= ICDC_ENABLE;
}

static int wm8510_low_gpio_wake(unsigned int irq, unsigned int on)
{
	int idx;

	/*
	 * Translate irq number to GPIO index.
	 */
	idx = (int)wm8510_i2g_map[irq];

	/*
	 * Even WM8510 has general wakeup 0 to 7, but notice that
	 * kernel call this functiuon should be irq 0 to 4.
	 */
	if ((idx == -1) || (idx > 4))
		return -ENXIO;  /* useless now*/

	if (on) {
		/*
		 * Clean wakeup detection (to detect zero signal)
		 * before setting new type.
		 */
		PMWT_VAL &= ~(GIRQ_TYPEMASK << (idx * 2));

		/*
		 * Copy GPIO[idx] pin detection to PMC Wakeup[idx]
		 * type register.
		 */
		PMWT_VAL |= (GPIO_INT_REQ_TYPE_VAL & (GIRQ_TYPEMASK << (idx * 2)));

		/*
		 * Enable to detect Wakeup[idx] pin in suspend mode.
		 */
		PMWE_VAL |= PMWE_WAKEUP(idx);
	} else {
		PMWE_VAL &= ~PMWE_WAKEUP(idx);
		/*
		 * Just disable Wakeup[idx] pin detection is enough.
		 * Removing wakeup type detection to zero is falderal,
		 * and waste your time.
		 */
	}

	return 0;
}

/*
 * irq_chip of GPIO[0:4]
 */
static struct irq_chip wm8510_wakeup_gpio_chip = {
	.name     = "wake",
	.ack      = wm8510_low_gpio_ack,
	.mask     = wm8510_low_gpio_mask,
	.unmask   = wm8510_low_gpio_unmask,
	.set_type = wm8510_gpio_type,
	.set_wake = wm8510_low_gpio_wake,
};

/*
 * irqchip of GPIO[5:9]
 */
static struct irq_chip wm8510_select_gpio_chip = {
	.name     = "gpio",
	.ack      = wm8510_low_gpio_ack,
	.mask     = wm8510_low_gpio_mask,
	.unmask   = wm8510_low_gpio_unmask,
	.set_type = wm8510_gpio_type,
};

static int wm8510_alarm_wake(unsigned int irq, unsigned int on)
{
	if (irq != IRQ_RTC1)
		return -ENXIO;  /* useless now*/

	if (on)
		PMWE_VAL |= PMWE_RTC;
	else
		PMWE_VAL &= ~PMWE_RTC;

	return 0;
}

static struct irq_chip wm8510_alarm_chip = {
	.name     = "alarm",
	.ack      = wm8510_mask_irq,      /* Current ack is in ISR */
	.mask     = wm8510_mask_irq,
	.unmask   = wm8510_unmask_irq,
	.set_wake = wm8510_alarm_wake,
};

static struct resource wm8510_irq_resource = {
	.name  = "irqs",
	.start = 0xD8140000,
	.end   = 0xD815ffff,
};

#ifdef CONFIG_PM
static int wm8510_irq_suspend(struct sys_device *dev, pm_message_t state)
{
	int i;
	struct wm8510_irq_state_s *st = &wm8510_irq_state;

	/* printk("%s state=%d\n", __FUNCTION__, state); */

	st->saved = 1;

	/*
	 * Save interrupt priority.
	 */
	for (i = 0; i < ICPC_NUM; i++)
		st->icpc[i] = ICPC_VAL(i);

	/*
	 * Save interrupt routing and mask.
	 */
	for (i = 0; i < ICDC_NUM_WM8510; i++) {
		if (i < 64)
			st->icdc[i] = ICDC0_VAL(i);
		else
			st->icdc[i] = ICDC1_VAL(i-64);
	}

	/*
	 * Disable all GPIO based interrupts.
	 */
	for (i = 0; i < NR_WM8510_GIRQS; i++)
		ICDC0_VAL(wm8510_g2i_map[i]) &= ~ICDC_ENABLE;

	/*
	 * Clear any pending GPIO based interrupts.
	 */
	GPIO_INT_REQ_STATUS_VAL = GPIO_INT_REQ_STATUS_VAL;

	return 0;
}

static int wm8510_irq_resume(struct sys_device *dev)
{
	int i;
	struct wm8510_irq_state_s *st = &wm8510_irq_state;

	if (st->saved) {
		/*
		 * Restore interrupt priority.
		 */
		for (i = 0; i < ICPC_NUM; i++)
			ICPC_VAL(i) = st->icpc[i];

		/*
		 * Restore interrupt routing and mask.
		 */
		for (i = 0; i < ICDC_NUM_WM8510; i++) {
			if (i < 64)
				ICDC0_VAL(i) = st->icdc[i];
			else
				ICDC1_VAL(i-64) = st->icdc[i];
		}
	}
	return 0;
}
#else
#define wm8510_irq_suspend NULL
#define wm8510_irq_resume NULL
#endif  /* CONFIG_PM */

static struct sysdev_class wm8510_irq_sysclass = {
	.name	 = "irq",
	.suspend = wm8510_irq_suspend,
	.resume  = wm8510_irq_resume,
};

static struct sys_device wm8510_irq_device = {
	.id  = 0,
	.cls = &wm8510_irq_sysclass,
};

static int __init wm8510_irq_init_devicefs(void)
{
	sysdev_class_register(&wm8510_irq_sysclass);
	return sysdev_register(&wm8510_irq_device);
}

device_initcall(wm8510_irq_init_devicefs);

void __init wmt_init_irq(void)
{
	unsigned int i;

	request_resource(&iomem_resource, &wm8510_irq_resource);

	/*
	 * Disable all IRQs and route them to zac_irq.
	 */
	for (i = 0; i < NR_WM8510_IRQS; i++) {
		if (wm8510_init_irqcfgs[i].irq < 64)
			ICDC0_VAL(wm8510_init_irqcfgs[i].irq) = wm8510_init_irqcfgs[i].icdc;
		else
			ICDC1_VAL(wm8510_init_irqcfgs[i].irq-64) = wm8510_init_irqcfgs[i].icdc;
	}

	/*
	 * Clear all GPIO status
	 */
	GPIO_INT_REQ_STATUS_VAL = (unsigned int)(-1);

	/*
	 * Install irqchip for GPIO[0:7]
	 */
	for (i = 0; i < 5; i++) {
		set_irq_chip(wm8510_init_irqcfgs[i].irq, &wm8510_wakeup_gpio_chip);
		/*
		 * Default we choose egde triggled policy
		 */
		set_irq_handler(wm8510_init_irqcfgs[i].irq, handle_edge_irq);
		set_irq_flags(wm8510_init_irqcfgs[i].irq, IRQF_VALID);
	}

	for (i = 5; i < 8; i++) {
		set_irq_chip(wm8510_init_irqcfgs[i].irq, &wm8510_select_gpio_chip);
		/*
		 * Default we choose egde triggled policy
		 */
		set_irq_handler(wm8510_init_irqcfgs[i].irq, handle_edge_irq);
		set_irq_flags(wm8510_init_irqcfgs[i].irq, IRQF_VALID);
	}

	/*
	 * Install irqchip for others
	 */
	for (i = 8 ; i < (NR_WM8510_IRQS-1); i++) {
		set_irq_chip(wm8510_init_irqcfgs[i].irq, &wm8510_normal_chip);
		set_irq_handler(wm8510_init_irqcfgs[i].irq, handle_level_irq);
		set_irq_flags(wm8510_init_irqcfgs[i].irq, IRQF_VALID);
	}

	/*
	 * Install irqchip for IRQ_RTC_ALARM
	 */
	i = (NR_WM8510_IRQS - 1);
	set_irq_chip(wm8510_init_irqcfgs[i].irq, &wm8510_alarm_chip);
	set_irq_handler(wm8510_init_irqcfgs[i].irq, handle_level_irq);
	set_irq_flags(wm8510_init_irqcfgs[i].irq, IRQF_VALID);

	/*
	 * Enable irq56~irq58 for IC1
	 */
	wm8510_unmask_irq(56);
	wm8510_unmask_irq(57);
	wm8510_unmask_irq(58);
}
