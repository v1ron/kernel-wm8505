/*
 * Beginnings of a core.c for WM8505
 *
 * Copyright 2010 Angus Gratton
 */
#include <linux/init.h>

extern void wm8505_gpio_init(void);

void __init wm8505_init_machine(void)
{
  wm8505_gpio_init();
}
