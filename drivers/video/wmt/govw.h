#include "vpp.h"

#ifdef WMT_FTBLK_GOVW

#ifndef GOVW_H
#define GOVW_H

typedef struct {
	VPP_MOD_BASE;
} govw_mod_t;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GOVW_C
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN govw_mod_t *p_govw;

EXTERN void govw_set_tg_enable(vpp_flag_t enable);
EXTERN void govw_set_hd_mif_enable(vpp_flag_t enable);
EXTERN vpp_flag_t govw_set_hd_color_format(vdo_color_fmt format);
EXTERN vdo_color_fmt govw_get_hd_color_format(void);
EXTERN void govw_set_hd_fb_addr(U32 y_addr,U32 c_addr);
EXTERN void govw_get_hd_fb_addr(U32 * y_addr,U32 * c_addr);
EXTERN void govw_set_hd_width(U32 width, U32 fb_width);
EXTERN void govw_get_hd_width(U32 *width,U32 *fb_width);
EXTERN int govw_mod_init(void);
EXTERN void govw_set_hd_framebuffer(vdo_framebuf_t *fb);

#ifdef WMT_FTBLK_GOVW_CSC
EXTERN void govw_set_rgb_mode(unsigned int mode);
#endif
#undef EXTERN

#ifdef __cplusplus
}
#endif
#endif				//GOVW_H
#endif				//WMT_FTBLK_GOVW
