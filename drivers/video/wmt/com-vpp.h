#ifndef COM_VPP_H
#define COM_VPP_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/fb.h>
#include <asm/io.h>
//#include <asm/arch/common_def.h>
//#include <asm/arch/com-video.h>
#include <mach/common_def.h>
#include <mach/com-video.h>
#else
#include "com-video.h"
#endif

#define VPP_OLD_API

#define VPP_NEW_FBUF_MANAGER

#define VPP_AHB_CLK				250000000

#ifdef CONFIG_MAX_RESX
	#define VPP_HD_MAX_RESX		CONFIG_MAX_RESX
#else
	#define VPP_HD_MAX_RESX		1920
#endif

#ifdef CONFIG_MAX_RESY
	#define VPP_HD_MAX_RESY		CONFIG_MAX_RESY
#else
	#define VPP_HD_MAX_RESY		1200
#endif

#ifdef CONFIG_DEFAULT_RESX
	#define VPP_HD_DISP_RESX	CONFIG_DEFAULT_RESX
#else
	#define VPP_HD_DISP_RESX		1024
#endif	

#ifdef CONFIG_DEFAULT_RESY
	#define VPP_HD_DISP_RESY	CONFIG_DEFAULT_RESY
#else
	#define VPP_HD_DISP_RESY		768
#endif

#ifdef CONFIG_DEFAULT_FPS
	#define VPP_HD_DISP_FPS		CONFIG_DEFAULT_FPS
#else
	#define VPP_HD_DISP_FPS			60
#endif

#ifndef __KERNEL__
#define REG32_VAL(addr) (*(volatile unsigned int *)(addr))
#define REG16_VAL(addr) (*(volatile unsigned short *)(addr))
#define REG8_VAL(addr)  (*(volatile unsigned char *)(addr))

#define U32 unsigned int
#define U16 unsigned short
#define U8 unsigned char

#endif

#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

#define VPP_YUV_BLACK		0x00,0x80,0x80	//Y, Cr, Cb
#define VPP_YUV_WHITE		0xff,0x80,0x80
#define VPP_YUV_RED			0x51,0xf0,0x5a
#define VPP_YUV_GREEN		0x90,0x22,0x35
#define VPP_YUV_BLUE		0x28,0x6d,0xf0
#define VPP_RGB32_BLACK		0x00,0x00,0x00,0x00

#define VPP_COL_RGB32_RED	0xFF0000
#define VPP_COL_BLACK		0x008080	//Y, Cr, Cb
#define VPP_COL_WHITE		0xff8080
#define VPP_COL_RED			0x41d464
#define VPP_COL_GREEN		0x902235
#define VPP_COL_BLUE		0x2372d4

#define VPP_MAGNUM(s, e) ((2^((s)-(e)+1))-1)

typedef enum {
	VPP_FBUF_GOVW_1,
	VPP_FBUF_GOVW_2,
	VPP_FBUF_SCLW_1,
	VPP_FBUF_SCLR_1,
	VPP_FBUF_MAX
} vpp_fbuf_t;

typedef enum {
	VPP_FLAG_NULL = 0,
	VPP_FLAG_ENABLE = 1,
	VPP_FLAG_DISABLE = 0,
	VPP_FLAG_TRUE = 1,
	VPP_FLAG_FALSE = 0,
	VPP_FLAG_ZERO = 0,
	VPP_FLAG_ONE = 1,
	VPP_FLAG_SUCCESS = 1,
	VPP_FLAG_ERROR = 0,
	VPP_FLAG_RD = 1,
	VPP_FLAG_WR = 0,
} vpp_flag_t;

typedef enum {
	VPP_MOD_GOVRS,
	VPP_MOD_GOVRH,
	VPP_MOD_DISP,
	VPP_MOD_GOVW,	
	VPP_MOD_GOVM,	
	VPP_MOD_SCL,
	VPP_MOD_SCLW,
	VPP_MOD_VPU,
	VPP_MOD_VPUW,	
	VPP_MOD_PIP,	
	VPP_MOD_VPPM,
	VPP_MOD_LCDC,	
	VPP_MOD_MAX
} vpp_mod_t;

typedef enum {
	VPP_OUTDEV_TV_NORMAL,	//NTSC 720x480 or PAL 720x576
	VPP_OUTDEV_VGA,
	VPP_OUTDEV_DVO,
	VPP_OUTDEV_MAX
} vpp_output_device_t;

typedef enum {
	VPP_ALPHA_VIDEO,
	VPP_ALPHA_GE,
	VPP_ALPHA_MAX,
} vpp_alpha_t;

typedef enum {
	VPP_DISP_FMT_FRAME,	//Progressive
	VPP_DISP_FMT_FIELD,	//Interlace
	VPP_DISP_FMT_MAX,
} vpp_display_format_t;

typedef enum {
	VPP_MEDIA_FMT_MPEG,
	VPP_MEDIA_FMT_H264,
	VPP_MEDIA_FMT_MAX,
} vpp_media_format_t;

typedef enum {
	VPP_PATH_NULL = 0,
	VPP_PATH_ALL = 0xffffffff,
	//in
	VPP_PATH_GOVM_IN_VPU = BIT0,
	VPP_PATH_GOVM_IN_SCL = BIT1,
	VPP_PATH_GOVM_IN_GE = BIT2,
	VPP_PATH_GOVM_IN_PIP = BIT3,
	VPP_PATH_GOVM_IN_SPU = BIT4,
	VPP_PATH_GOVM_IN_CUR = BIT5,
	//out
	VPP_PATH_SCL_OUT_REALTIME = BIT10,
	VPP_PATH_SCL_OUT_RECURSIVE = BIT11,
} vpp_path_t;

typedef enum {			/* don't change this order */
	VPP_CSC_YUV2RGB2_MIN,
	VPP_CSC_YUV2RGB_SDTV_0_255 = VPP_CSC_YUV2RGB2_MIN,
	VPP_CSC_YUV2RGB_SDTV_16_235,
	VPP_CSC_YUV2RGB_HDTV_0_255,
	VPP_CSC_YUV2RGB_HDTV_16_235,
	VPP_CSC_YUV2RGB_JFIF_0_255,
	VPP_CSC_YUV2RGB_SMPTE170M,
	VPP_CSC_YUV2RGB_SMPTE240M,	
	VPP_CSC_RGB2YUV_MIN,
	VPP_CSC_RGB2YUV_SDTV_0_255 = VPP_CSC_RGB2YUV_MIN,
	VPP_CSC_RGB2YUV_SDTV_16_235,
	VPP_CSC_RGB2YUV_HDTV_0_255,
	VPP_CSC_RGB2YUV_HDTV_16_235,
	VPP_CSC_RGB2YUV_JFIF_0_255,
	VPP_CSC_RGB2YUV_SMPTE170M,
	VPP_CSC_RGB2YUV_SMPTE240M,	
	VPP_CSC_MAX,
	VPP_CSC_BYPASS
} vpp_csc_t;

typedef enum {
	VPP_INT_NULL = 0,
	VPP_INT_ALL = 0xffffffff,

	VPP_INT_SCL_VBIE = BIT0,
	VPP_INT_SCL_VBIS = BIT1,
	VPP_INT_SCL_PVBI = BIT2,

	VPP_INT_GOVW_VBIE = BIT11,
	VPP_INT_GOVW_VBIS = BIT12,
	VPP_INT_GOVW_PVBI = BIT13,
	
	VPP_INT_GOVRS_VBIE = BIT17,
	VPP_INT_GOVRS_VBIS = BIT18,
	VPP_INT_GOVRS_PVBI = BIT19,

	VPP_INT_GOVRH_VBIE = BIT21,
	VPP_INT_GOVRH_VBIS = BIT22,	//write done
	VPP_INT_GOVRH_PVBI = BIT23,

	VPP_INT_DISP_VBIE = BIT25,
	VPP_INT_DISP_VBIS = BIT26,
	VPP_INT_DISP_PVBI = BIT27,
	
	VPP_INT_VPU_VBIE = BIT28,
	VPP_INT_VPU_VBIS = BIT29,
	VPP_INT_VPU_PVBI = BIT30,

	VPP_INT_LCD_EOF = BIT31,
	
} vpp_int_t;

typedef enum {
	/* SCL */
	VPP_INT_ERR_SCL_TG = BIT0,
	VPP_INT_ERR_SCLR1_MIF = BIT1,
	VPP_INT_ERR_SCLR2_MIF = BIT2,
	VPP_INT_ERR_SCLW_MIFRGB = BIT3,
	VPP_INT_ERR_SCLW_MIFY = BIT4,
	VPP_INT_ERR_SCLW_MIFC = BIT5,

	/* VPU */
	VPP_INT_ERR_VPU_TG = BIT6,
	VPP_INT_ERR_VPUR1_MIF = BIT7,
	VPP_INT_ERR_VPUR2_MIF = BIT8,
	VPP_INT_ERR_VPUW_MIFRGB = BIT9,
	VPP_INT_ERR_VPUW_MIFY = BIT10,
	VPP_INT_ERR_VPUW_MIFC = BIT11,

	/* GOVW */
	VPP_INT_ERR_GOVM_VPU = BIT12,
	VPP_INT_ERR_GOVM_GE = BIT13,
	VPP_INT_ERR_GOVM_SPU = BIT14,
	VPP_INT_ERR_GOVM_PIP = BIT15,

	/* GOVW */
	VPP_INT_ERR_GOVW_TG = BIT16,	//def at govw
	VPP_INT_ERR_GOVW_MIFY = BIT17,	//def at govw
	VPP_INT_ERR_GOVW_MIFC = BIT18,	//def at govw

	/* GOVRS */
	VPP_INT_ERR_GOVRS_MIF = BIT19,	

	/* GOVRH */
	VPP_INT_ERR_GOVRH_MIF = BIT20,	

	/* PIP */
	VPP_INT_ERR_PIP_Y = BIT21,
	VPP_INT_ERR_PIP_C = BIT22,

	/* LCD */
	VPP_INT_ERR_LCD_UNDERRUN = BIT23,
	VPP_INT_ERR_LCD_OVERFLOW = BIT24,
} vpp_int_err_t;

typedef enum {
	VPP_REG_LEVEL_1,
	VPP_REG_LEVEL_2,
	VPP_REG_LEVEL_MAX,
} vpp_reglevel_t;

typedef enum {
	VPP_BPP_1,
	VPP_BPP_2,
	VPP_BPP_4,
	VPP_BPP_8,
	VPP_BPP_12,
	VPP_BPP_16_565,
	VPP_BPP_16_555 = VPP_BPP_16_565,
	VPP_BPP_18,
	VPP_BPP_24,
	VPP_BPP_MAX,
} vpp_bpp_t;

typedef enum {
	VPP_GAMMA_DISABLE,
	VPP_GAMMA_22,
	VPP_GAMMA_24,
	VPP_GAMMA_28,
	VPP_GAMMA_MAX,
} vpp_gamma_t;

typedef enum {
	VPP_DATAWIDHT_12,
	VPP_DATAWIDHT_24,
	VPP_DATAWIDHT_MAX,
} vpp_datawidht_t;

typedef enum {
	VPP_ADDR_NORMAL,
	VPP_ADDR_BURST,
	VPP_ADDR_MAX,
} vpp_addr_mode_t;

typedef enum {
	VPP_DEI_WEAVE,
	VPP_DEI_BOB,
	VPP_DEI_ADAPTIVE_ONE,
	VPP_DEI_ADAPTIVE_THREE,
	VPP_DEI_FIELD,
	VPP_DEI_MOTION_VECTOR,
	VPP_DEI_MAX,
	VPP_DEI_DYNAMIC,
} vpp_deinterlace_t;

typedef enum {
	VPP_DIR_THERE,
	VPP_DIR_TWO,
	VPP_DIR_FOUR,
	VPP_DIR_FIVE,
	VPP_DIR_MAX,
} vpp_direction_t;

typedef enum {
	VPP_FIELD_TOP,
	VPP_FIELD_BOTTOM,
	VPP_FIELD_MAX,
} vpp_field_t;

typedef enum {
	VPP_TVSYS_NTSC,
	VPP_TVSYS_NTSCJ,
	VPP_TVSYS_NTSC443,
	VPP_TVSYS_PAL,
	VPP_TVSYS_PALM,
	VPP_TVSYS_PAL60,
	VPP_TVSYS_PALN,
	VPP_TVSYS_PALNC,
	VPP_TVSYS_MAX
} vpp_tvsys_t;

typedef enum {
	VPP_TVCONN_YCBCR,
	VPP_TVCONN_SCART,
	VPP_TVCONN_YPBPR,
	VPP_TVCONN_VGA,
	VPP_TVCONN_SVIDEO,
	VPP_TVCONN_CVBS,
	VPP_TVCONN_MAX
} vpp_tvconn_t;

#define VPP_OPT_INTERLACE			0x01
#define VPP_VGA_HSYNC_POLAR_HI		0x02
#define VPP_VGA_VSYNC_POLAR_HI		0x04
#define VPP_DVO_SYNC_POLAR_HI		0x08
#define VPP_DVO_VSYNC_POLAR_HI		0x10

typedef struct {
	unsigned int pixel_clock;	
	unsigned int option;

	unsigned int hsync;
	unsigned int hbp;
	unsigned int hpixel;
	unsigned int hfp;

	unsigned int vsync;
	unsigned int vbp;
	unsigned int vpixel;
	unsigned int vfp;
} vpp_timing_t;

typedef struct {
	int read_cycle;
	
	unsigned int total_pixel_of_line;
	unsigned int begin_pixel_of_active;
	unsigned int end_pixel_of_active;
	
	unsigned int total_line_of_frame;
	unsigned int begin_line_of_active;
	unsigned int end_line_of_active;

	unsigned int hsync;
	unsigned int vsync;
	unsigned int line_number_between_VBIS_VBIE;
	unsigned int line_number_between_PVBI_VBIS;
} vpp_clock_t;

typedef struct _vpp_image_t {
	U32 y_addr;
	U32 c_addr;
	vdo_color_fmt src_col_fmt;
} vpp_image_t;

typedef struct {
	vdo_framebuf_t src_fb;
	vdo_framebuf_t dst_fb;
} vpp_scale_t;

#define VPP_VOUT_OPT_ENABLE	0x01
#define VPP_VOUT_OPT_ACTIVE	0x02
typedef struct {
	int num;
	unsigned int option;
	char name[10];
} vpp_vout_info_t;

typedef struct {
	int num;
	int arg;
} vpp_vout_parm_t;

typedef struct {
	int num;
	unsigned int option[3];
} vpp_vout_option_t;

typedef struct {
	int num;
	vpp_timing_t tmr;
} vpp_vout_tmr_t;

typedef struct {
	unsigned int yaddr;
	unsigned int caddr;
} vpp_fbaddr_t;

typedef struct {
	unsigned int chip_id;
	unsigned int version;
	unsigned int resx_max;
	unsigned int resy_max;
	unsigned int pixel_clk;
	unsigned int module;
	unsigned int option;
} vpp_cap_t;

typedef struct {
	vpp_flag_t enable;
	vpp_alpha_t mode;
	int A;
	int B;
} vpp_alpha_parm_t;

typedef struct {
	vpp_path_t src_path;
	vpp_flag_t enable;
} vpp_src_path_t;

typedef struct {
	vpp_flag_t enable;
	vpp_flag_t sync_polar;
	vpp_flag_t vsync_polar;
	vdo_color_fmt color_fmt;
	vpp_datawidht_t data_w;
	int clk_inv;
	int clk_delay;
} vdo_dvo_parm_t;

typedef struct {
	unsigned int addr;
	unsigned int index;
	unsigned int val;
} vpp_i2c_t;

#define VPP_FLAG_DISPFB_ADDR	BIT(0)
#define VPP_FLAG_DISPFB_INFO	BIT(1)
#define VPP_FLAG_DISPFB_VIEW	BIT(2)
#define VPP_FLAG_DISPFB_PIP		BIT(3)
typedef struct {
	unsigned int yaddr;
	unsigned int caddr;
	vdo_framebuf_t info;
	vdo_view_t view;

	unsigned int flag;
} vpp_dispfb_t;

typedef struct {
	vpp_mod_t mod;
	int read;
	unsigned int arg1;
	unsigned int arg2;
} vpp_mod_arg_t;

typedef struct {
	vpp_mod_t mod;
	int read;	
	vpp_timing_t tmr;
} vpp_mod_timing_t;

typedef struct {
	vpp_mod_t mod;
	int read;	
	vdo_framebuf_t fb;
} vpp_mod_fbinfo_t;

typedef struct {
	vpp_mod_t mod;
	int read;
	vdo_view_t view;
} vpp_mod_view_t;

typedef struct {
	char pts[8];
} vpp_pts_t;

#define VPPIO_MAGIC		'f'

/* VPP common ioctl command */
#define VPPIO_VPP_BASE				0x0
#define VPPIO_VPPGET_INFO			_IOR(VPPIO_MAGIC,VPPIO_VPP_BASE+0,vpp_cap_t)
#define VPPIO_VPPSET_INFO			_IOW(VPPIO_MAGIC,VPPIO_VPP_BASE+0,vpp_cap_t)
#define VPPIO_I2CSET_BYTE			_IOW(VPPIO_MAGIC,VPPIO_VPP_BASE+1,vpp_i2c_t)
#define VPPIO_I2CGET_BYTE			_IOR(VPPIO_MAGIC,VPPIO_VPP_BASE+1,vpp_i2c_t)
#define VPPIO_MODULE_FRAMERATE		_IOWR(VPPIO_MAGIC,VPPIO_VPP_BASE+2,vpp_mod_arg_t)
#define VPPIO_MODULE_ENABLE			_IOWR(VPPIO_MAGIC,VPPIO_VPP_BASE+3,vpp_mod_arg_t)
#define VPPIO_MODULE_TIMING			_IOWR(VPPIO_MAGIC,VPPIO_VPP_BASE+4,vpp_mod_timing_t)
#define VPPIO_MODULE_FBADDR			_IOWR(VPPIO_MAGIC,VPPIO_VPP_BASE+5,vpp_mod_arg_t)
#define VPPIO_MODULE_FBINFO			_IOWR(VPPIO_MAGIC,VPPIO_VPP_BASE+6,vpp_mod_fbinfo_t)
#define VPPIO_VPPSET_DIRECTPATH		_IO(VPPIO_MAGIC,VPPIO_VPP_BASE+7)
#define VPPIO_VPPSET_FBDISP			_IOW(VPPIO_MAGIC,VPPIO_VPP_BASE+8,vpp_dispfb_t)
#define VPPIO_WAIT_FRAME			_IO(VPPIO_MAGIC,VPPIO_VPP_BASE+9)
#define VPPIO_MODULE_VIEW			_IOWR(VPPIO_MAGIC,VPPIO_VPP_BASE+10,vpp_mod_view_t)
#define VPPIO_VPPGET_PTS			_IOR(VPPIO_MAGIC,VPPIO_VPP_BASE+11,vpp_pts_t)
#define VPPIO_VPPSET_PTS			_IOW(VPPIO_MAGIC,VPPIO_VPP_BASE+11,vpp_pts_t)

/* VOUT ioctl command */
#define VPPIO_VOUT_BASE				0x10
#define VPPIO_VOGET_INFO			_IOR(VPPIO_MAGIC,VPPIO_VOUT_BASE+0,vpp_vout_info_t)
#define VPPIO_VOSET_MODE			_IOW(VPPIO_MAGIC,VPPIO_VOUT_BASE+1,vpp_vout_parm_t)
#define VPPIO_VOSET_BLANK			_IOW(VPPIO_MAGIC,VPPIO_VOUT_BASE+2,vpp_vout_parm_t)
#define VPPIO_VOSET_DACSENSE		_IOW(VPPIO_MAGIC,VPPIO_VOUT_BASE+3,vpp_vout_parm_t)
#define VPPIO_VOSET_BRIGHTNESS		_IOW(VPPIO_MAGIC,VPPIO_VOUT_BASE+4,vpp_vout_parm_t)
#define VPPIO_VOGET_BRIGHTNESS		_IOR(VPPIO_MAGIC,VPPIO_VOUT_BASE+4,vpp_vout_parm_t)
#define VPPIO_VOSET_CONTRAST		_IOW(VPPIO_MAGIC,VPPIO_VOUT_BASE+5,vpp_vout_parm_t)
#define VPPIO_VOGET_CONTRAST		_IOR(VPPIO_MAGIC,VPPIO_VOUT_BASE+5,vpp_vout_parm_t)
#define VPPIO_VOSET_OPTION			_IOW(VPPIO_MAGIC,VPPIO_VOUT_BASE+6,vpp_vout_option_t)
#define VPPIO_VOGET_OPTION			_IOR(VPPIO_MAGIC,VPPIO_VOUT_BASE+6,vpp_vout_option_t)

/* GOVR ioctl command */
#define VPPIO_GOVR_BASE				0x20
#define VPPIO_GOVRSET_DVO			_IOW(VPPIO_MAGIC,VPPIO_GOVR_BASE+0,vdo_dvo_parm_t)

/* GOVW ioctl command */
#define VPPIO_GOVW_BASE				0x30
#define VPPIO_GOVW_ENABLE			_IO(VPPIO_MAGIC,VPPIO_GOVW_BASE+0)

/* GOVM ioctl command */		
#define VPPIO_GOVM_BASE				0x40
#define VPPIO_GOVMSET_SRCPATH		_IOW(VPPIO_MAGIC,VPPIO_GOVM_BASE+0,vpp_src_path_t)
#define VPPIO_GOVMGET_SRCPATH		_IO(VPPIO_MAGIC,VPPIO_GOVM_BASE+0)
#define VPPIO_GOVMSET_ALPHA			_IOW(VPPIO_MAGIC,VPPIO_GOVM_BASE+1,vpp_alpha_parm_t)
#define VPPIO_GOVMSET_GAMMA			_IO(VPPIO_MAGIC,VPPIO_GOVM_BASE+2)
#define VPPIO_GOVMSET_CLAMPING 		_IO(VPPIO_MAGIC,VPPIO_GOVM_BASE+3)

/* VPU ioctl command */
#define VPPIO_VPU_BASE				0x50
#define VPPIO_VPUSET_VIEW			_IOW(VPPIO_MAGIC,VPPIO_VPU_BASE+0,vdo_view_t)
#define VPPIO_VPUGET_VIEW			_IOR(VPPIO_MAGIC,VPPIO_VPU_BASE+0,vdo_view_t)
#define VPPIO_VPUSET_FBDISP			_IOW(VPPIO_MAGIC,VPPIO_VPU_BASE+1,vpp_dispfb_t)
#define VPPIO_VPU_CLR_FBDISP		_IO(VPPIO_MAGIC,VPPIO_VPU_BASE+2)

/* SCL ioctl command */
#define VPPIO_SCL_BASE				0x60
#define VPPIO_SCL_SCALE				_IOWR(VPPIO_MAGIC,VPPIO_SCL_BASE+0,vpp_scale_t)
#define VPPIO_SCL_DROP_LINE_ENABLE  _IO(VPPIO_MAGIC,VPPIO_SCL_BASE+1)

#define VPPIO_MAX					0x70
#endif //COM_VPP_H
