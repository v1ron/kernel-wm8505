#include "vpp.h"

//#ifdef WMT_FTBLK_VPP

#ifndef VPPM_H
#define VPPM_H

typedef struct {
	VPP_MOD_BASE;
} vppm_mod_t;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef VPPM_C
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN vppm_mod_t *p_vppm;

EXTERN void vppm_set_int_enable(vpp_flag_t enable, vpp_int_t int_bit);
EXTERN vpp_int_t vppm_get_int_status(void);
EXTERN void vppm_clean_int_status(vpp_int_t int_sts);
EXTERN void vppm_set_module_reset(vpp_mod_t mod_bit); //ok
EXTERN int vppm_mod_init(void);
#ifdef WMT_FTBLK_DISP
EXTERN void vppm_set_DAC_select(int tvmode);
#endif

#undef EXTERN

#ifdef __cplusplus
}
#endif
#endif				//VPP_H
//#endif				//WMT_FTBLK_VPP
