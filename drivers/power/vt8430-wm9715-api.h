


#define DEBUG

#ifdef DEBUG
	#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
	#define dbg(format, arg...)
#endif

extern void wm9715_pendown_detect(void);
extern int wm9715_pen_down(void);
extern int wm9712_poll_sample (int adcsel, int *sample);
extern int wm9712_poll_sample_battery (int *sample);



