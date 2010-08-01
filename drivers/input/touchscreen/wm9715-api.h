#ifndef __WM8505_WM9715_API
#define __WM8505_WM9715_API


extern void wm9715_pendown_detect(void);
extern int wm9715_pen_down(void);
extern int wm9712_poll_sample (int adcsel, int *sample);
extern int wm9712_poll_sample_battery (int *sample);
extern void wm9715_gpio_config(void);


#endif

