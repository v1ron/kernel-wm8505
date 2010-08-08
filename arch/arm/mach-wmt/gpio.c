/*
 * VIA WM8505 GPIOlib support
 *
 * Copyright (c) 2010 Angus Gratton
 *
 * Currently only supports "Signal GPIO" 0-7, could probably be extended to
 * replace lots of the other GPIO regs
 *
 * Based on code originally from:
 *  linux/arch/arm/mach-ep93xx/gpio.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <asm/gpio.h>

#include <mach/hardware.h>

#define GPIO_SIGNAL_ENABLE        GPIO_ENABLE_CTRL_10_REG
#define GPIO_SIGNAL_OUTPUT_ENABLE GPIO_OUTPUT_ENABLE_10_REG
#define GPIO_SIGNAL_OUTPUT_DATA   GPIO_OUTPUT_DATA_10_REG
#define GPIO_SIGNAL_INPUT_DATA    GPIO_INPUT_DATA_10_REG

static int wm8505_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
  printk("Set GPIO %u input\n", offset);
  *GPIO_SIGNAL_ENABLE |= 1<<offset;
  *GPIO_SIGNAL_OUTPUT_ENABLE &= ~(1<<offset);
  
  return 0;
}

static int wm8505_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int val)
{
  printk("Set GPIO %u output %d\n", offset, val);
  *GPIO_SIGNAL_ENABLE |= 1<<offset;
  *GPIO_SIGNAL_OUTPUT_ENABLE |= 1<<offset;  
  return 0;
}

static int wm8505_gpio_get(struct gpio_chip *chip, unsigned offset)
{
  printk("Get GPIO %u\n", offset);
  *GPIO_SIGNAL_ENABLE |= 1<<offset;
  return *GPIO_SIGNAL_INPUT_DATA & (1<<offset);
}

static void wm8505_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
  printk("Set GPIO %u %d\n", offset, val);
  *GPIO_SIGNAL_ENABLE |= 1<<offset;
  if(val)
	 *GPIO_SIGNAL_OUTPUT_DATA |= 1<<offset;
  else
	 *GPIO_SIGNAL_OUTPUT_DATA &= ~(1<<offset);
}

static void wm8505_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
  int i;
  u8 data = *GPIO_SIGNAL_INPUT_DATA;
  u8 output = *GPIO_SIGNAL_OUTPUT_ENABLE;

  for(i = 0; i < chip->ngpio; i++) {
	 seq_printf(s, "GPIO %s%d: %s %s\n", chip->label, i,
					( data &(1<<i) ) ? "set" : "clear", 
					( output &(1<<i) ) ? "output" : "input" );					
  }
}

static struct gpio_chip wm8505_gpio = {
  .label = "GPIOs",
  .direction_input = wm8505_gpio_direction_input,
  .direction_output = wm8505_gpio_direction_output,
  .get = wm8505_gpio_get,
  .set = wm8505_gpio_set,
  .dbg_show = wm8505_gpio_dbg_show,
  .base = 0,
  .ngpio = 8
};

void __init wm8505_gpio_init(void)
{
  int i;
  gpiochip_add(&wm8505_gpio);
  // export all for now, allow full userspace control(!)
  for(i = 0; i < wm8505_gpio.ngpio; i++) {
	 gpio_request(i, NULL);
	 gpio_export(i, true);
  }
}
