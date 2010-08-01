/*++
	linux/drivers/input/keyboard/wmt_kpad.c

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
--*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/errno.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/kpad.h>
#include <mach/wmt_kpad.h>
#include <linux/suspend.h>

/* Debug macros */
#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_ALERT "%s: " fmt, __func__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

static unsigned int kpad_matrix_scan(unsigned int *key);
#define row_size  8
#define col_size  8

unsigned int wmt_keycodes[row_size * col_size] = {
/*                         WMT Key Map
 * ----------------------------------------------------------
 *     0      1      2      3	|     4      5      6      7
 *     8      9      A      B	|     C      D      E      F
 *     G      H      I      J	|     K      L      M      N
 *     O      P      Q      R	|     S      T      U      V
 * -----------------------------+----------------------------
 *     W      X      Y      Z	|     /      \      ;      ,
 *     .      -      @    TAB	| SPACE    DEL    ALT  SHIFT
 * ENTER   BACK   MENU CENTER	|    UP   DOWN   LEFT  RIGHT
 *  VOL+   VOL-  POWER SEARCH	|  HOME CAMERA EXPLOR ENVELO
 * ----------------------------------------------------------
 */
	[0] = KEY_0,              /* key 11    0 */
	[1] = KEY_1,              /* key 2     1 */
	[2] = KEY_2,              /* key 3     2 */
	[3] = KEY_3,              /* key 4     3 */
	[4] = KEY_4,              /* key 5     4 */
	[5] = KEY_5,              /* key 6     5 */
	[6] = KEY_6,              /* key 7     6 */
	[7] = KEY_7,              /* key 8     7 */
	[8] = KEY_8,              /* key 9     8 */
	[9] = KEY_9,              /* key 10    9 */
	[10] = KEY_A,             /* key 30    A */
	[11] = KEY_B,             /* key 48    B */
	[12] = KEY_C,             /* key 46    C */
	[13] = KEY_D,             /* key 32    D */
	[14] = KEY_E,             /* key 18    E */
	[15] = KEY_F,             /* key 33    F */
	[16] = KEY_G,             /* key 34    G */
	[17] = KEY_H,             /* key 35    H */
	[18] = KEY_I,             /* key 23    I */
	[19] = KEY_J,             /* key 36    J */
	[20] = KEY_K,             /* key 37    K */
	[21] = KEY_L,             /* key 38    L */
	[22] = KEY_M,             /* key 50    M */
	[23] = KEY_N,             /* key 49    N */
	[24] = KEY_O,             /* key 24    O */
	[25] = KEY_P,             /* key 25    P */
	[26] = KEY_Q,             /* key 16    Q */
	[27] = KEY_R,             /* key 19    R */
	[28] = KEY_S,             /* key 31    S */
	[29] = KEY_T,             /* key 20    T */
	[30] = KEY_U,             /* key 22    U */
	[31] = KEY_V,             /* key 47    V */
	[32] = KEY_W,             /* key 17    W */
	[33] = KEY_X,             /* key 45    X */
	[34] = KEY_Y,             /* key 21    Y */
	[35] = KEY_Z,             /* key 44    Z */
	[36] = KEY_SLASH,         /* key 53    SLASH '/' */
	[37] = KEY_BACKSLASH,     /* key 43    BACKSLASH '\' */
	[38] = KEY_SEMICOLON,     /* key 39    SEMICOLON ';' */
	[39] = KEY_COMMA,         /* key 51    COMMA ',' */
	[40] = KEY_DOT,           /* key 52    PERIOD '.' */
	[41] = KEY_MINUS,         /* key 12    MINUS '-' */
	[42] = KEY_EMAIL,         /* key 215   AT '@' */
	[43] = KEY_TAB,           /* key 15    TAB */
	[44] = KEY_SPACE,         /* key 57    SPACE */
	[45] = KEY_BACKSPACE,     /* key 14    DEL */
	[46] = KEY_LEFTALT,       /* key 56    ALT_LEFT */
	[47] = KEY_LEFTSHIFT,     /* key 42    SHIFT_LEFT */
	[48] = KEY_ENTER,         /* key 28    ENTER */
	[49] = KEY_BACK,          /* key 158   BACK              WAKE_DROPPED */
	[50] = KEY_MENU,          /* key 139   MENU              WAKE_DROPPED */
	[51] = KEY_REPLY,         /* key 232   DPAD_CENTER       WAKE_DROPPED */
	[52] = KEY_UP,            /* key 103   DPAD_UP           WAKE_DROPPED */
	[53] = KEY_DOWN,          /* key 108   DPAD_DOWN         WAKE_DROPPED */
	[54] = KEY_LEFT,          /* key 105   DPAD_LEFT         WAKE_DROPPED */
	[55] = KEY_RIGHT,         /* key 106   DPAD_RIGHT        WAKE_DROPPED */
	[56] = KEY_VOLUMEUP,      /* key 115   VOLUME_UP */
	[57] = KEY_VOLUMEDOWN,    /* key 114   VOLUME_DOWN */
	[58] = KEY_POWER,         /* key 116   POWER             WAKE */
	[59] = KEY_SEARCH,        /* key 217   SEARCH            WAKE_DROPPED */
	[60] = KEY_HOME,          /* key 102   HOME              WAKE */
	[61] = KEY_CAMERA,        /* key 212   CAMERA */
	[62] = KEY_WWW,           /* key 150   EXPLORER */
	[63] = KEY_MAIL           /* key 155   ENVELOPE */

#if 0 /* Reserve */
	[] = KEY_EQUAL,           /* key 13    EQUALS '=' */
	[] = KEY_LEFTBRACE,       /* key 26    LEFT_BRACKET '[' */
	[] = KEY_RIGHTBRACE,      /* key 27    RIGHT_BRACKET ']' */
	[] = KEY_APOSTROPHE,      /* key 40    APOSTROPHE ''' */
	[] = KEY_RIGHTSHIFT,      /* key 54    SHIFT_RIGHT */
	[] = KEY_F1,              /* key 59    MENU              WAKE_DROPPED */
	[] = KEY_F2,              /* key 60    SOFT_RIGHT        WAKE */
	[] = KEY_F3,              /* key 61    CALL              WAKE_DROPPED */
	[] = KEY_F4,              /* key 62    ENDCALL           WAKE_DROPPED */
	[] = KEY_RIGHTALT,        /* key 100   ALT_RIGHT */
	[] = KEY_END,             /* key 107   ENDCALL           WAKE_DROPPED */
	[] = KEY_COMPOSE,         /* key 127   SEARCH            WAKE_DROPPED */
	[] = KEY_SWITCHVIDEOMODE, /* key 227   STAR '*' */
	[] = KEY_KBDILLUMTOGGLE,  /* key 228   POUND '#' */
	[] = KEY_KBDILLUMDOWN,    /* key 229   MENU              WAKE_DROPPED */
	[] = KEY_KBDILLUMUP,      /* key 230   SOFT_RIGHT        WAKE */
	[] = KEY_SEND,            /* key 231   CALL              WAKE_DROPPED */
	[] = 399,                 /* key 399   GRAVE '`' */
#endif
};

static struct input_dev *kpad_dev;
static unsigned int *keymap;
static unsigned int keymap_size;
static struct kpad_saved_s *saved;

static struct wmt_kpad_s kpad = {
	.ref	= 0,
	.res	= NULL,
	.regs   = NULL,
	.irq	= 0,
	.ints   = { 0, 0, 0, 0, 0 },
};

#ifdef CONFIG_CPU_FREQ
/*
 * Well, the debounce time is not very critical while zac2_clk
 * rescaling, but we still do it.
 */

/* kpad_clock_notifier()
 *
 * When changing the processor core clock frequency, it is necessary
 * to adjust the KPMIR register.
 *
 * Returns: 0 on success, -1 on error
 */
static int kpad_clock_notifier(struct notifier_block *nb, unsigned long event,
	void *data)
{

	switch (event) {
	case CPUFREQ_PRECHANGE:
		/*
		 * Disable input.
		 */
		kpad.regs->kpmcr &= ~KPMCR_EN;
		break;

	case CPUFREQ_POSTCHANGE:
		/*
		 * Adjust debounce time then enable input.
		 */
		kpad.regs->kpmir = KPMIR_DI((125 * wm8510_ahb_khz()) / \
			(262144)) | KPMIR_SI(0xFF);
		kpad.regs->kpmcr |= KPMCR_EN;
		break;
	}

	return 0;
}

/*
 * Notify callback while issusing zac2_clk rescale.
 */
static struct notifier_block kpad_clock_nblock = {
	.notifier_call  = kpad_clock_notifier,
	.priority = 1
};
#endif

static irqreturn_t
kpad_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int scan[row_size*col_size];
	unsigned int i, pressed, status;

	/* Disable interrupt */
	disable_irq(kpad.irq);

	kpad.regs->kpmcr &= ~(KPMCR_EN | KPMCR_IEN) ;

	/*
	 * Get keypad interrupt status and clean interrput source.
	 */
	status = kpad.regs->kpstr;
	kpad.regs->kpstr |= status;

	if (kpad.regs->kpstr != 0)
		printk("[kpad] status clear failed! \n");

	memset(scan, 0, (row_size*col_size));

	if (status & KPSTR_ASA) {
		/* Get the number of pressed keys. */
		pressed = kpad_matrix_scan(scan);

		if (pressed > 0) {
			/*
			 * Report all keys is possible.
			 * > cat /dev/tty0 and press the ENTER of kpad.
			 */
			for (i = 0; i < pressed; i++) {
				input_report_key(kpad_dev, scan[i], 1);
				input_report_key(kpad_dev, scan[i], 0);
				DPRINTK(KERN_DEBUG"Key[%d] = %d\n", i, scan[i]);
			}
			input_sync(kpad_dev);

		} else
			printk(KERN_WARNING "\nERROR: PRESS %d KEYS! \n ", pressed);

	} else
		printk(KERN_WARNING "KPSTR=0x%.8x\n", status);

	/* Enable interrupt */
	kpad.regs->kpmcr |= KPMCR_EN | KPMCR_IEN ;
	enable_irq(kpad.irq);

	return IRQ_HANDLED;
}

static unsigned int kpad_matrix_scan(unsigned int *key)
{
	unsigned int idx, reg, pressed, dummy_reg;
	unsigned int count = 0;
	unsigned int i, b = 0, bit_idx, valid;

	if (!key)
		return 0;

	reg = kpad.regs->kpmar;
	pressed = KPMAR_KEY(reg);

	if (!(reg & KPMAR_VALID) || !pressed)
		return 0;

/*
 * Don't get missed, we're tring to remap KPMAR bitmap
 * to the index of keymap.
 */

#define KPAD_COL_SIZE (((kpad.regs->kpmcr & KPMCR_COLMASK) >> 8) + 1)
#define KPAD_ROW_SIZE (((kpad.regs->kpmcr & KPMCR_ROWMASK) >> 12) + 1)
#define KEY_INDEX(reg) (((reg & KPMAR_ROWMASK) >> 4) * KPAD_COL_SIZE) + (reg & KPMAR_COLMASK);

#define COL0_ROW_BYTE ((kpad.regs->kpmr0 & 0x0000000f))
#define COL1_ROW_BYTE ((kpad.regs->kpmr0 & 0x000f0000)>>16)
#define COL2_ROW_BYTE ((kpad.regs->kpmr1 & 0x0000000f))
#define COL3_ROW_BYTE ((kpad.regs->kpmr1 & 0x000f0000)>>16)
#define COL4_ROW_BYTE ((kpad.regs->kpmr2 & 0x0000000f))
#define COL5_ROW_BYTE ((kpad.regs->kpmr2 & 0x000f0000)>>16)
#define COL6_ROW_BYTE ((kpad.regs->kpmr3 & 0x0000000f))
#define COL7_ROW_BYTE ((kpad.regs->kpmr3 & 0x000f0000)>>16)
#define COL_ROW_BYTE(x) (COL##x##_ROW_BYTE)

	if (pressed == 1) {
		idx = KEY_INDEX(kpad.regs->kpmar);
		*(key + count) = keymap[idx];
		count++;

		/* Dummy code, should safely removed. */
		/* It is used to prevent keypad hangs. */
		dummy_reg = kpad.regs->kpmr3;
		dummy_reg = kpad.regs->kpmr2;
		dummy_reg = kpad.regs->kpmr1;
		dummy_reg = kpad.regs->kpmr0;

		return pressed;
	}

	for (i = 0; i < KPAD_COL_SIZE; i++) {

		bit_idx = 0;
		switch (i) {
		case 0:
			valid = ((kpad.regs->kpmr0 & 0x80000000)>>31);
			b = COL_ROW_BYTE(0);
			break;
		case 1:
			valid = ((kpad.regs->kpmr0 & 0x80000000)>>31);
			b = COL_ROW_BYTE(1);
			break;
		case 2:
			valid = ((kpad.regs->kpmr1 & 0x80000000)>>31);
			b = COL_ROW_BYTE(2);
			break;
		case 3:
			valid = ((kpad.regs->kpmr1 & 0x80000000)>>31);
			b = COL_ROW_BYTE(3);
			break;
		case 4:
			valid = ((kpad.regs->kpmr2 & 0x80000000)>>31);
			b = COL_ROW_BYTE(4);
			break;
		case 5:
			valid = ((kpad.regs->kpmr2 & 0x80000000)>>31);
			b = COL_ROW_BYTE(5);
			break;
		case 6:
			valid = ((kpad.regs->kpmr3 & 0x80000000)>>31);
			b = COL_ROW_BYTE(6);
			break;
		case 7:
			valid = ((kpad.regs->kpmr3 & 0x80000000)>>31);
			b = COL_ROW_BYTE(7);
			break;
		}


		/*
		 *  the valid bit seems never work. so don't bother it.
		 */
		while (b != 0) {
			if (b & 0x01) {
				idx = i + bit_idx * KPAD_ROW_SIZE;
				*(key + count) = keymap[idx];
				count++;
			}
			b >>= 1;
			bit_idx++;
		}
	}

	/*
	 *  Prevent kpad hangs by load all registers when multi-keys pressed.
	 */
	if (pressed > 1) {
		dummy_reg = kpad.regs->kpmr3;
		dummy_reg = kpad.regs->kpmr2;
		dummy_reg = kpad.regs->kpmr1;
		dummy_reg = kpad.regs->kpmr0;
	}

	return pressed;
}

static int kpad_open(struct input_dev *dev)
{
	int ret = 0;
	unsigned int i, status;

	if (kpad.ref++) {
		/* Return success, but not initialize again. */
		return 0;
	}

	/*
	 * Turn on keypad clocks.
	 */

	PMCEL_VAL |= (1<<9); /* bit9 is CK_KPAD*/

	/* Clean keypad matrix and keypad direct input control registers. */
	kpad.regs->kpmcr = 0;
	kpad.regs->kpdcr = 0;

	/*
	 * Setup matrix configuration, debounce time, and assign keymap.
	 */
	kpad.regs->kpmcr = KPMCR_COL(col_size-1) | KPMCR_ROW(row_size-1);

	/*
	 * Set keypad debounce time to be about 125 ms.
	 */
	kpad.regs->kpmir = KPMIR_DI(0x0FFF) | KPMIR_SI(0x01);

	/* Load keymap */
	keymap = wmt_keycodes;
	keymap_size = ARRAY_SIZE(wmt_keycodes);

	/*
	 * Clean all previous keypad status. (Previous bug fixed.)
	 */

	status = kpad.regs->kpstr;
	kpad.regs->kpstr |= status;
	if (kpad.regs->kpstr != 0)
		printk(KERN_ERR "[kpad] clear status failed!\n");

	ret = request_irq(kpad.irq, kpad_interrupt, IRQF_DISABLED, "keypad", dev);

	if (ret) {
		printk(KERN_ERR "%s: Can't allocate irq %d\n", __func__, IRQ_KPAD);
		kpad.ref--;
		goto kpad_open_out;
	}

	/*
	 * Enable keypad matrix automatic scan.
	 */
	kpad.regs->kpmcr |= KPMCR_EN | KPMCR_IEN | KPMCR_ASA;

	/* Register an input event device. */


	dev->name = "keypad",
	dev->phys = "keypad",

	/*
	 *  Let kpad to implement key repeat.
	 */

	set_bit(EV_KEY, dev->evbit);

	for (i = 0; i < keymap_size; i++)
		set_bit(keymap[i], dev->keybit);

	dev->keycode = keymap;
	dev->keycodesize = sizeof(unsigned int);
	dev->keycodemax = keymap_size;

	/*
	 * For better view of /proc/bus/input/devices
	 */
	dev->id.bustype = 0;
	dev->id.vendor  = 0;
	dev->id.product = 0;
	dev->id.version = 0;

	input_register_device(dev);

kpad_open_out:
	return ret;
}

static void kpad_close(struct input_dev *dev)
{
	if (--kpad.ref)
		return;

	/*
	 * Free interrupt resource
	 */
	kpad.regs->kpmcr = 0;
	free_irq(kpad.irq, dev);

	/*
	 * Place the device in the low power state
	 */

	PMCEL_VAL &= ~(1<<9); /* bit9 is CK_KPAD*/


	/*
	 * Unregister input device driver
	 */
	input_unregister_device(dev);
}

static int wmt_kpad_probe(struct device *dev)
{
	unsigned long base;
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);

	kpad_dev = input_allocate_device();
	if (kpad_dev == NULL)
		return -1;
	/*
	 * Simply check resources parameters.
	 */
	if (pdev->num_resources < 2 || pdev->num_resources > 3) {
		ret = -ENODEV;
		goto kpad_probe_out;
	}

	base = pdev->resource[0].start;

	kpad.irq = pdev->resource[1].start;
	kpad.res = request_mem_region(base, KPAD_IO_SIZE, "keypad");

	if (!kpad.res || !kpad.irq) {
		ret = -ENODEV;
		goto kpad_probe_out;
	}

	kpad.regs = ioremap(base, KPAD_IO_SIZE);

	if (!kpad.regs) {
		ret = -ENOMEM;
		goto kpad_probe_out;
	}

	kpad_dev->open = kpad_open,
	kpad_dev->close = kpad_close,

	kpad_open(kpad_dev);

kpad_probe_out:

#ifndef CONFIG_SKIP_DRIVER_MSG
	printk(KERN_INFO "WMT keypad driver initialized: %s\n",
		  (ret == 0) ? "ok" : "failed");
#endif
	return ret;
}

static int wmt_kpad_remove(struct device *dev)
{
	kpad_close(kpad_dev);

	/*
	 * Free allocated resource
	 */
	kfree(kpad.res);
	kpad.res = NULL;

	if (kpad.regs) {
		iounmap(kpad.regs);
		kpad.regs = NULL;
	}

	kpad.ref = 0;
	kpad.irq = 0;

	memset(&kpad.ints, 0, KPAD_INTS_SIZE);

	return 0;
}

static int wmt_kpad_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_NOTIFY:
		/*
		 * Suspend transition is about to happen.
		 */
		break;
	case SUSPEND_SAVE_STATE:
		/*
		 * This call depend on the "state" request.
		 * In WMT, we have two power-saving modes,
		 * power-on sleep (standby) and power-off
		 * suspend (suspend).
		 *
		 * We only need to save hardware registers
		 * on power-off suspend.
		 */
		if (state == PM_SUSPEND_MEM) {
			/*
			 * Save the context of the hardware.
			 */
			saved = kmalloc(sizeof(struct kpad_saved_s), GFP_KERNEL);

			if (saved) {
				saved->kpmcr = kpad.regs->kpmcr;
				saved->kpdcr = kpad.regs->kpdcr;
				saved->kpicr = kpad.regs->kpicr;
				saved->kpmir = kpad.regs->kpmir;
				saved->kpdir = kpad.regs->kpdir;
			} else {
				return -ENOMEM;
			}
		}
		break;
	case SUSPEND_DISABLE:
		/*
		 * Stop I/O transactions.
		 */
		kpad.regs->kpmcr &= ~KPMCR_EN;
		break;
	case SUSPEND_POWER_DOWN:
		/*
		 * Place the device in the low power state
		 */
		PMCEL_VAL &= ~(1<<9); /* bit9 is CK_KPAD*/

		break;
	}
	return 0;
}

static int wmt_kpad_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		/*
		 * Set the power state to the state before
		 * the suspend call
		 */
		PMCEL_VAL |= (1<<9); /* bit9 is CK_KPAD*/
		break;
	case RESUME_RESTORE_STATE:
		/*
		 * Restore the state saved by the
		 * SUSPEND_SAVE_STATE suspend call.
		 */
		if (saved) {
			kpad.regs->kpmcr = saved->kpmcr;
			kpad.regs->kpdcr = saved->kpdcr;
			kpad.regs->kpicr = saved->kpicr;
			kpad.regs->kpmir = saved->kpmir;
			kpad.regs->kpdir = saved->kpdir;
			kfree(saved);
			saved = NULL;
		}
		break;
	case RESUME_ENABLE:
		/*
		 * Start accepting I/O transactions again.
		 */
		kpad.regs->kpmcr |= KPMCR_EN;
		break;
	}
	return 0;
}

static struct device_driver wmt_kpad_driver = {
	.name = "kpad",
	.bus = &platform_bus_type,
	.probe = &wmt_kpad_probe,
	.remove = &wmt_kpad_remove,
	.suspend = &wmt_kpad_suspend,
	.resume	= &wmt_kpad_resume
};

static int __init kpad_init(void)
{
	int ret;

#ifdef CONFIG_CPU_FREQ
	ret = cpufreq_register_notifier(&kpad_clock_nblock, \
		CPUFREQ_TRANSITION_NOTIFIER);

	if (ret) {
		printk(KERN_ERR "Unable to register CPU frequency " \
			"change notifier (%d)\n", ret);
	}
#endif

	ret = driver_register(&wmt_kpad_driver);

	return ret;
}

static void __exit kpad_exit(void)
{
	driver_unregister(&wmt_kpad_driver);
}

module_init(kpad_init);
module_exit(kpad_exit);

MODULE_DESCRIPTION("WMT generic keypad driver");
MODULE_LICENSE("GPL");
