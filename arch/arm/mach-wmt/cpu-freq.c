/*
  linux/arch/arm/mach-wmt/cpu-freq.c

  CPU frequency scaling
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>

#include <mach/hardware.h>

#include "generic.h"

#define REG_PM_STATUS *((volatile unsigned int *)0xd8130000)
#define REG_PLL_A   *((volatile unsigned int *)0xd8130200)
#define REG_CPU_DIV *((volatile unsigned int *)0xd8130300)
#define REG_AHB_DIV *((volatile unsigned int *)0xd8130304)

#define PM_UPDATE_MASK 0x1fff9b37
#define PLL_PREDIV_BYPASS 1<<8
#define PLL_MUL_MASK 0x1f
#define CPU_DIV_MASK 0x1f
#define AHB_DIV_MASK 0x07

#define PM_WAIT_SETTLE do { } while (REG_PM_STATUS & PM_UPDATE_MASK)

typedef struct wmt_scale_s {
  unsigned int khz;       /* cpu_clk in khz */
  unsigned char pll;      /* pll mul */
  unsigned char prediv_bypass;  /* pll predivider (/2) bypass */
  unsigned char cpu;      /* cpu div */
  unsigned char ahb;      /* ahb div */

} wmt_scale_t;

wmt_scale_t wm8510_freqs[] = {
    /* khz, pll, cpu, ahb */
  // the khz values here are all a bit wrong
  // Setting PLL below 0x4 (below = no PLL multiplier) seems to break AHB bus

  { 50001,  0x04,  0, 2, 1 },  
  { 75001,  0x06,  0, 2, 1 }, 
  { 100001, 0x08,  0, 2, 1 },
  { 125002, 0x0a,  0, 2, 2 },
  { 150002, 0x0c,  0, 2, 2 }, 
  { 175002, 0x0e,  0, 2, 2 },
  { 200002, 0x10, 0, 2, 2 }, 
  { 225002, 0x12, 0, 2, 2 },
  { 250003, 0x14, 0, 2, 2 },
  { 275002, 0x16, 0, 2, 2 },
  { 300002, 0x18, 0, 2, 2 }, 
  { 325002, 0x1A, 0, 2, 2 },
  { 350002, 0x1C, 0, 2, 2 }, // Stock/boot kernel speed (except RAM)
  { 375003, 0x1E, 0, 2, 3 }, // AHB @ 125Mhz
  { 387003, 0x1F, 0, 2, 3 }, 

  // Setting any faster than 387Mhz locks up nearly instantly. PLL prediv bypass seems to work, though
  //  { 400002, 0x10, 1, 2, 3 }
};

#define NR_FREQS        ARRAY_SIZE(wm8510_freqs)


unsigned int wm8510_arm_khz(void)
{
	unsigned int pll_reg = REG_PLL_A;
	unsigned int pll_prediv = (pll_reg & PLL_PREDIV_BYPASS) ? 1 : 2; 
	return 66*1000*(pll_reg & PLL_MUL_MASK)/ (REG_CPU_DIV & CPU_DIV_MASK) / pll_prediv;
}

unsigned int wm8510_ahb_khz(void)
{
	unsigned int value, cpu_freq;

	cpu_freq = wm8510_arm_khz();
	value = cpu_freq / REG_AHB_DIV;

	return value;
}

unsigned int wmt_freq_to_idx(unsigned int khz)
{
    int i;

    for (i = 0; i < NR_FREQS; i++)
			if (wm8510_freqs[i].khz >= khz)
				break;

    return (i < NR_FREQS) ? (i) : (NR_FREQS-1);
}

wmt_scale_t *wmt_idx_to_parms(unsigned int idx)
{
    wmt_scale_t *ret = NULL;

    if (idx < NR_FREQS)
			ret = &wm8510_freqs[idx];
    else
			ret = &wm8510_freqs[NR_FREQS];

    return ret;
}

unsigned int wmt_idx_to_freq(unsigned int idx)
{
    int freq = 0;

    if (idx < NR_FREQS)
			freq = wm8510_freqs[idx].khz;
    else
			freq = wm8510_freqs[NR_FREQS-1].khz;

    return freq;
}

/*
 * verify cpufreq policy settings
 */
int wmt_verify_speed(struct cpufreq_policy *policy)
{
    unsigned int tmp;

    if (policy->cpu)
			return -EINVAL;

    cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq, policy->cpuinfo.max_freq);

    /*
     * Make sure that at least one frequency is within the policy
     */
    tmp = wm8510_freqs[wmt_freq_to_idx(policy->min)].khz;

    if (tmp > policy->max)
			policy->max = tmp;

    cpufreq_verify_within_limits(policy,
				policy->cpuinfo.min_freq,
				policy->cpuinfo.max_freq);
    return 0;

}

/*
 * Generic macro : restart CPU for reload PLL.
 * PLL-A ,ZAC Clock Divisor,AHB Clock Divisor,PLL-B ,PLL-C
 * need waitting about 5ms for PLL to stable
 */
static void wmt_restart_cpu(void)
{
	unsigned int reg_11c, reg_120;
	const int zero = 0;
	unsigned int reg_time;

	REG8_VAL(0xd8140000 + 0x65) = 0x8;			/* route irq to irq*/
	REG32_VAL(0xd8130000 + 0x120) = 0x03;
	while (REG32_VAL(0xd8130000 + 0x124) != 0)
		;
	reg_time = REG32_VAL(0xd8130000 + 0x110);
	reg_time += 0x0000EA60;
	REG32_VAL(0xd8130000 + 0x104) = reg_time;

	reg_11c = REG32_VAL(0xd8130000 + 0x11C);
	REG32_VAL(0xd8130000 + 0x11C) = 0x02;	/* OSTimer Int Enable*/
	reg_120 = REG32_VAL(0xd8130000 + 0x120);
	REG32_VAL(0xd8130000 + 0x120) = 0x00000001;	/* OSTimer Ctrl*/

    asm("mcr%? p15, 0, %0, c7, c0, 4" : : "r" (zero));		/* Force ARM to idle mode*/
    asm("nop");
    asm("nop");
    asm("nop");

	REG32_VAL(0xd8130000+0x11C) = reg_11c;	/* OSTimer Int Enable*/
	REG32_VAL(0xd8130000+0x120) = reg_120;	/* OSTimer Ctrl*/
	return;
}

static void wm8510_speedstep(unsigned int idx)
{
	wmt_scale_t *np = wmt_idx_to_parms(idx);
	unsigned int pll_desired = (np->pll & PLL_MUL_MASK) | ( np->prediv_bypass ? PLL_PREDIV_BYPASS : 0 );

	if (np->ahb > 1) {
	  if (np->ahb != REG_AHB_DIV) {
		 while (PMCS_VAL & PMCS_BUSY)
			;
		 PM_WAIT_SETTLE;
		 REG_AHB_DIV = np->ahb;
	  }
	  
	  if (pll_desired != REG_PLL_A) {
		 PM_WAIT_SETTLE;
		 REG_PLL_A = pll_desired;
	  }
	  
	  if (np->cpu != REG_CPU_DIV) {
		 PM_WAIT_SETTLE;
		 REG_CPU_DIV = np->cpu;
	  }
	}	else {
	  if (REG_PLL_A) {
		 PM_WAIT_SETTLE;
		 REG_PLL_A = pll_desired;
	  }
	  if (np->cpu != REG_CPU_DIV) {
		 PM_WAIT_SETTLE;
		 REG_CPU_DIV = np->cpu;
	  }
	  if (np->ahb != REG_AHB_DIV) {
		 while (PMCS_VAL & PMCS_BUSY)
			;
		 PM_WAIT_SETTLE;
		 REG_AHB_DIV = np->ahb;
	  }
	}
	PM_WAIT_SETTLE;
	
	wmt_restart_cpu();
}

static int wmt_target(struct cpufreq_policy *policy,
							unsigned int target_freq,
							unsigned int relation)
{
    unsigned int idx;
    unsigned long flags;
    struct cpufreq_freqs freqs;

    idx = wmt_freq_to_idx(target_freq);

		switch (relation) {
		case CPUFREQ_RELATION_L:
			/*
			* Try to select a new_freq higher than or equal target_freq.
			*/
			if (wmt_idx_to_freq(idx) > policy->max)
				idx--;
			break;
		case CPUFREQ_RELATION_H:
			/*
			* Try to select a new_freq lower than or equal target_freq.
			*/
			if ((wmt_idx_to_freq(idx) > target_freq) &&
			  (wmt_idx_to_freq(idx-1) >= policy->min))
				idx--;
			break;
		}

    freqs.old = wm8510_arm_khz();
    freqs.new = wmt_idx_to_freq(idx);
    freqs.cpu = 0;

	 //	 printk("Requested step from %d to %d (idx %d)\n", freqs.old, freqs.new, idx);

    if (freqs.new != freqs.old) {
			/*
			* Because current design we have all differnet ZAC2
			* clock scheme on each entry, so we can do this check
			* to make sure only apply change on differnet clock.
			* The benefit is skip redundent I/O stop and start.
			*
			* But notice that if we have same ZAC2 clock scheme, we
			* must move CPUFREQ_PRECHANGE and CPUFREQ_POSTCHANGE
			* notify call out side this block.
			*/
			cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
			local_irq_save(flags);
			wm8510_speedstep(idx);
			local_irq_restore(flags);
			cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
    }

	 //	 printk("Now running at %d\n", wm8510_arm_khz());

    return 0;
}

static int __init wmt_cpu_init(struct cpufreq_policy *policy)
{
    if (policy->cpu != 0)
			return -EINVAL;

    policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
    policy->cpuinfo.min_freq = wm8510_freqs[0].khz;
    policy->cpuinfo.max_freq = wm8510_freqs[NR_FREQS-1].khz;
    policy->cpuinfo.transition_latency = 1000000; // 1ms, assumed?
    policy->cur = wm8510_arm_khz();
    policy->min =  100001;
    policy->max = 350002;
    return 0;
}

static struct freq_attr* wmt_cpufreq_attr[] = {
        &cpufreq_freq_attr_scaling_available_freqs,
        NULL,
};

static struct cpufreq_driver wmt_cpufreq_driver = {
    .flags			= CPUFREQ_STICKY,
    .verify         = wmt_verify_speed,
    .target         = wmt_target,
    .init           = wmt_cpu_init,
	 .attr           = wmt_cpufreq_attr,
    .name           = "wmt",
};

static int __init wmt_cpufreq_init(void)
{
    return cpufreq_register_driver(&wmt_cpufreq_driver);
}

arch_initcall(wmt_cpufreq_init);

