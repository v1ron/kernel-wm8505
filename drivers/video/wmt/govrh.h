#include "vpp.h"

#ifdef WMT_FTBLK_GOVRH

#ifndef GOVRH_H
#define GOVRH_H

typedef struct {
	VPP_MOD_BASE;
	
	unsigned int vga_dac_sense_cnt;
} govrh_mod_t;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GOVRH_C
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN govrh_mod_t *p_govrh;

EXTERN vpp_flag_t govrh_set_tg_enable(vpp_flag_t enable);
EXTERN void govrh_set_dvo_enable(vpp_flag_t enable);
EXTERN void govrh_set_dvo_sync_polar(vpp_flag_t hsync,vpp_flag_t vsync);
EXTERN vpp_flag_t govrh_set_dvo_outdatw(vpp_datawidht_t width);
EXTERN void govrh_set_dvo_clock_delay(int inverse,int delay);
EXTERN void govrh_set_colorbar(vpp_flag_t enable,int mode,int inv);
EXTERN void govrh_set_vga_enable(vpp_flag_t enable);
EXTERN void govrh_set_VGA_sync(U32 hsync, U32 vsync);
EXTERN void govrh_set_VGA_sync_polar(U32 hsync, U32 vsync);
EXTERN int govrh_monitor_DAC_sense(void);
EXTERN void govrh_set_DAC_pwrdn(vpp_flag_t enable);
EXTERN void govrh_set_contrast(int level);
EXTERN int govrh_get_contrast(void);
EXTERN void govrh_set_brightness(int level);
EXTERN int govrh_get_brightness(void);
EXTERN void govrh_set_MIF_enable(vpp_flag_t enable);
EXTERN vpp_flag_t govrh_set_data_format(vdo_color_fmt format);
EXTERN vdo_color_fmt govrh_get_color_format(void);
EXTERN vpp_flag_t govrh_set_source_format(vpp_display_format_t format);
EXTERN void govrh_set_output_format(vpp_display_format_t field);
EXTERN void govrh_set_fb_addr(U32 y_addr,U32 c_addr);
EXTERN void govrh_get_fb_addr(U32 *y_addr,U32 *c_addr);
//	EXTERN vpp_flag_t govrh_set_output_format(vpp_display_format_t format);
EXTERN void govrh_set_fb_info(U32 width, U32 act_width, U32 x_offset,U32 y_offset);
EXTERN void govrh_get_fb_info(U32 * width, U32 * act_width,U32 * x_offset, U32 * y_offset);
EXTERN void govrh_set_fifo_index(U32 index);
EXTERN vpp_flag_t govrh_set_reg_level(vpp_reglevel_t level);
EXTERN void govrh_set_reg_update(vpp_flag_t enable);
EXTERN void govrh_set_video_mode(unsigned int resx,unsigned int resy,unsigned int pixel_clock,vpp_timing_t *tmr);
EXTERN void govrh_set_csc_mode(vpp_csc_t mode);
EXTERN void govrh_set_framebuffer(vdo_framebuf_t *inbuf);
EXTERN void govrh_get_framebuffer(vdo_framebuf_t *fb);
EXTERN vdo_color_fmt govrh_get_dvo_color_format(void);
EXTERN void govrh_set_dvo_color_format(vdo_color_fmt fmt);
EXTERN vpp_int_err_t govrh_get_int_status(void);
EXTERN void govrh_clean_int_status(vpp_int_err_t int_sts);
EXTERN void govrh_get_timing(vpp_clock_t * tmr);

#ifdef WMT_FTBLK_LVDS
EXTERN vdo_color_fmt govrh_LVDS_get_color_format(void);
#endif
#ifdef WMT_FTBLK_IGS
EXTERN void govrh_IGS_set_mode(int no,int mode_18bit,int msb);
#endif
#ifdef WMT_FTBLK_DISP
EXTERN void govrh_DISP_set_enable(vpp_flag_t enable);
#endif
EXTERN int govrh_mod_init(void);

#undef EXTERN

#ifdef __cplusplus
}
#endif
#endif				//GOVRH_H
#endif				//WMT_FTBLK_GOVRH
