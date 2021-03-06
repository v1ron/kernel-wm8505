/* Product ID: 8510 / Project ID: 3426 */

#ifndef WM8510_GOVRH_REG_H
#define WM8510_GOVRH_REG_H

//Base2
#define REG_GOVRH_TG_ENABLE			(GOVRH_BASE2_ADDR+0x00)	//TG
#define REG_GOVRH_READ_CYC			(GOVRH_BASE2_ADDR+0x04)
#define REG_GOVRH_H_ALLPXL			(GOVRH_BASE2_ADDR+0x08)	//TG_1
#define REG_GOVRH_V_ALLLN			(GOVRH_BASE2_ADDR+0x0c)
#define REG_GOVRH_ACTLN_BG			(GOVRH_BASE2_ADDR+0x10)
#define REG_GOVRH_ACTLN_END			(GOVRH_BASE2_ADDR+0x14)
#define REG_GOVRH_ACTPX_BG			(GOVRH_BASE2_ADDR+0x18)
#define REG_GOVRH_ACTPX_END			(GOVRH_BASE2_ADDR+0x1c)
#define REG_GOVRH_VBIE_LINE			(GOVRH_BASE2_ADDR+0x20)
#define REG_GOVRH_PVBI_LINE			(GOVRH_BASE2_ADDR+0x24)
#define REG_GOVRH_HDMI_VBISW		(GOVRH_BASE2_ADDR+0x28)
#define REG_GOVRH_HDMI_HSYNW		(GOVRH_BASE2_ADDR+0x2c)
#define REG_GOVRH_DVO_SET			(GOVRH_BASE2_ADDR+0x48)
#define REG_GOVRH_CB_ENABLE			(GOVRH_BASE2_ADDR+0x50)
#define REG_GOVRH_H_ALLPXL2			(GOVRH_BASE2_ADDR+0x58)
#define REG_GOVRH_V_ALLLN2			(GOVRH_BASE2_ADDR+0x5c)
#define REG_GOVRH_ACTLN_BG2			(GOVRH_BASE2_ADDR+0x60)
#define REG_GOVRH_ACTLN_END2		(GOVRH_BASE2_ADDR+0x64)
#define REG_GOVRH_ACTPX_BG2			(GOVRH_BASE2_ADDR+0x68)
#define REG_GOVRH_ACTPX_END2		(GOVRH_BASE2_ADDR+0x6c)
#define REG_GOVRH_VBIE_LINE2		(GOVRH_BASE2_ADDR+0x70)
#define REG_GOVRH_PVBI_LINE2		(GOVRH_BASE2_ADDR+0x74)
#define REG_GOVRH_HDMI_VBISW2		(GOVRH_BASE2_ADDR+0x78)
#define REG_GOVRH_HDMI_HSYNW2		(GOVRH_BASE2_ADDR+0x7c)
#define REG_GOVRH_VGA_HSYNW			(GOVRH_BASE2_ADDR+0x90)
#define REG_GOVRH_VGA_VSYNW			(GOVRH_BASE2_ADDR+0x94)
#define REG_GOVRH_VGA_SYNPOLAR		(GOVRH_BASE2_ADDR+0x98)
#define REG_GOVRH_DAC_MOD			(GOVRH_BASE2_ADDR+0x9c)
#define REG_GOVRH_DAC_VAL			(GOVRH_BASE2_ADDR+0xa0)
#define REG_GOVRH_DAC_CON			(GOVRH_BASE2_ADDR+0xa4)
#define REG_GOVRH_DAC_TEST			(GOVRH_BASE2_ADDR+0xa8)
#define REG_GOVRH_DAC_BTEST			(GOVRH_BASE2_ADDR+0xac)
#define REG_GOVRH_DAC_CTEST			(GOVRH_BASE2_ADDR+0xb0)
#define REG_GOVRH_DAC_DBG			(GOVRH_BASE2_ADDR+0xb4)
#define REG_GOVRH_CONTRAST			(GOVRH_BASE2_ADDR+0xb8)
#define REG_GOVRH_BRIGHTNESS		(GOVRH_BASE2_ADDR+0xbc)
#define REG_GOVRH_DMACSC_COEF0		(GOVRH_BASE2_ADDR+0xc0)
#define REG_GOVRH_DMACSC_COEF1		(GOVRH_BASE2_ADDR+0xc4)
#define REG_GOVRH_DMACSC_COEF2		(GOVRH_BASE2_ADDR+0xc8)
#define REG_GOVRH_DMACSC_COEF3		(GOVRH_BASE2_ADDR+0xcc)
#define REG_GOVRH_DMACSC_COEF4		(GOVRH_BASE2_ADDR+0xd0)
#define REG_GOVRH_DMACSC_COEF5		(GOVRH_BASE2_ADDR+0xd8)
#define REG_GOVRH_DMACSC_COEF6		(GOVRH_BASE2_ADDR+0xdc)
#define REG_GOVRH_CSC_MODE			(GOVRH_BASE2_ADDR+0xe0)
#define REG_GOVRH_YUV2RGB			(GOVRH_BASE2_ADDR+0xe4)
#define REG_GOVRH_H264_INPUT_EN		(GOVRH_BASE2_ADDR+0xe8)

#define REG_GOVRH_DVO_PIX			(GOVRH_BASE1_ADDR+0x30)
#define REG_GOVRH_DVO_DLY_SEL		(GOVRH_BASE1_ADDR+0x34)
#define REG_GOVRH_INT				(GOVRH_BASE1_ADDR+0x38)
#define REG_GOVRH_MIF				(GOVRH_BASE1_ADDR+0x80)
#define REG_GOVRH_COLFMT			(GOVRH_BASE1_ADDR+0x84)	//422,420
#define REG_GOVRH_SRCFMT			(GOVRH_BASE1_ADDR+0x88)
#define REG_GOVRH_DSTFMT			(GOVRH_BASE1_ADDR+0x8C)
#define REG_GOVRH_YSA				(GOVRH_BASE1_ADDR+0x90)
#define REG_GOVRH_CSA				(GOVRH_BASE1_ADDR+0x94)
#define REG_GOVRH_PIXWID			(GOVRH_BASE1_ADDR+0x98)
#define REG_GOVRH_BUFWID			(GOVRH_BASE1_ADDR+0x9c)
#define REG_GOVRH_VCROP				(GOVRH_BASE1_ADDR+0xa0)
#define REG_GOVRH_HCROP				(GOVRH_BASE1_ADDR+0xa4)
#define REG_GOVRH_FHI				(GOVRH_BASE1_ADDR+0xa8)
#define REG_GOVRH_COLFMT2			(GOVRH_BASE1_ADDR+0xac)	//444
#define REG_GOVRH_REG_STS			(GOVRH_BASE1_ADDR+0xe4)
#define REG_GOVRH_SWFLD				(GOVRH_BASE1_ADDR+0xe8)

//Base2
//REG_GOVRH_TG_ENABLE,0x00
#define GOVRH_TG_MODE				REG_GOVRH_TG_ENABLE,BIT8,8
#define GOVRH_TG_ENABLE				REG_GOVRH_TG_ENABLE,BIT0,0

//REG_GOVRH_READ_CYC,0x04
#define GOVRH_READ_CYC				REG_GOVRH_READ_CYC, 0x7F, 0

//REG_GOVRH_H_ALLPXL,0x08
#define GOVRH_H_ALLPXL				REG_GOVRH_H_ALLPXL, 0xFFF, 0

//REG_GOVRH_V_ALLLN,0x0c
#define GOVRH_V_ALLLN				REG_GOVRH_V_ALLLN, 0xFFF, 0

//REG_GOVRH_ACTLN_BG,0x10
#define GOVRH_ACTLN_BG				REG_GOVRH_ACTLN_BG, 0xFFF, 0

//REG_GOVRH_ACTLN_END,0x14
#define GOVRH_ACTLN_END				REG_GOVRH_ACTLN_END, 0xFFF, 0

//REG_GOVRH_ACTPX_BG,0x18
#define GOVRH_ACTPX_BG				REG_GOVRH_ACTPX_BG, 0xFFF, 0

//REG_GOVRH_ACTPX_END,0x1c
#define GOVRH_ACTPX_END				REG_GOVRH_ACTPX_END, 0xFFF, 0

//REG_GOVRH_VBIE_LINE,0x20
#define GOVRH_VBIE_LINE				REG_GOVRH_VBIE_LINE, 0x7F, 0

//REG_GOVRH_PVBI_LINE,0x24
#define GOVRH_PVBI_LINE				REG_GOVRH_PVBI_LINE, 0x1F, 0

//REG_GOVRH_HDMI_VBISW,0x28
#define GOVRH_HDMI_VBISW			REG_GOVRH_HDMI_VBISW, 0xFF, 0

//REG_GOVRH_HDMI_HSYNW,0x2c
#define GOVRH_HDMI_HSYNW			REG_GOVRH_HDMI_HSYNW, 0xFF, 0

//REG_GOVRH_DVO_SET,0x48
#define GOVRH_DVO_ENABLE			REG_GOVRH_DVO_SET,BIT2,2
#define GOVRH_DVO_SYNC_POLAR		REG_GOVRH_DVO_SET,BIT1,1
#define GOVRH_DVO_OUTWIDTH			REG_GOVRH_DVO_SET,BIT0,0

//REG_GOVRH_CB_ENABLE,0x50
#define GOVRH_CB_INVERSION			REG_GOVRH_CB_ENABLE,BIT16,16
#define GOVRH_CB_MODE				REG_GOVRH_CB_ENABLE,BIT8,8
#define GOVRH_CB_ENABLE				REG_GOVRH_CB_ENABLE,BIT0,0

//REG_GOVRH_H_ALLPXL2,0x58
#define GOVRH_H_ALLPXL2				REG_GOVRH_H_ALLPXL2, 0xFFF, 0

//REG_GOVRH_V_ALLLN2,0x5c
#define GOVRH_V_ALLLN2				REG_GOVRH_V_ALLLN2, 0x7FF, 0

//REG_GOVRH_ACTLN_BG2,0x60
#define GOVRH_ACTLN_BG2				REG_GOVRH_ACTLN_BG2, 0xFF, 0

//REG_GOVRH_ACTLN_END2,0x64
#define GOVRH_ACTLN_END2			REG_GOVRH_ACTLN_END2, 0x7FF, 0

//REG_GOVRH_ACTPX_BG2,0x68
#define GOVRH_ACTPX_BG2				REG_GOVRH_ACTPX_BG2, 0x1FF, 0

//REG_GOVRH_ACTPX_END2,0x6c
#define GOVRH_ACTPX_END2			REG_GOVRH_ACTPX_END2, 0x1FFF, 0

//REG_GOVRH_VBIE_LINE2,0x70
#define GOVRH_VBIE_LINE2			REG_GOVRH_VBIE_LINE2, 0x7F, 0

//REG_GOVRH_PVBI_LINE2,0x74
#define GOVRH_PVBI_LINE2			REG_GOVRH_PVBI_LINE2, 0x1F, 0

//REG_GOVRH_HDMI_VBISW2,0x78
#define GOVRH_HDMI_VBISW2			REG_GOVRH_HDMI_VBISW2, 0xFF, 0

//REG_GOVRH_HDMI_HSYNW2,0x7c
#define GOVRH_HDMI_HSYNW2			REG_GOVRH_HDMI_HSYNW2, 0xFF, 0

//REG_GOVRH_VGA_HSYNW,0x90
#define GOVRH_VGA_HSYNW				REG_GOVRH_VGA_HSYNW, 0x7FFF, 0

//REG_GOVRH_VGA_VSYNW,0x94
#define GOVRH_VGA_VSYNW				REG_GOVRH_VGA_VSYNW, 0x7FFF, 0

//REG_GOVRH_VGA_SYNPOLAR,0x98
#define GOVRH_VGA_VSYN_POLAR		REG_GOVRH_VGA_SYNPOLAR,BIT1,1
#define GOVRH_VGA_HSYN_POLAR		REG_GOVRH_VGA_SYNPOLAR,BIT0,0

//REG_GOVRH_DAC_CTL,0x9c
#define GOVRH_DAC_NOR_ENABLE		REG_GOVRH_DAC_MOD, 0x700000, 20
#define GOVRH_DAC_RESET				REG_GOVRH_DAC_MOD, 0x70000, 16
#define GOVRH_DAC_MANUAL_SENSE		REG_GOVRH_DAC_MOD,BIT8,8
#define GOVRH_DAC_PWRDN				REG_GOVRH_DAC_MOD,BIT1,1
#define GOVRH_DAC_SCD_ENABLE		REG_GOVRH_DAC_MOD,BIT0,0

//REG_GOVRH_DAC_VAL,0xa0
#define GOVRH_DAC_VAL_LOW			REG_GOVRH_DAC_VAL, 0xFF0000, 16
#define GOVRH_DAC_VAL_NOR			REG_GOVRH_DAC_VAL, 0xFF, 0

//REG_GOVRH_DAC_STS,0xa4
#define GOVRH_DAC_STS				REG_GOVRH_DAC_CON,BIT0,0

//REG_GOVRH_DAC_TEST,0xa8
#define GOVRH_DAC_MODE_SEL			REG_GOVRH_DAC_TEST, 0x7000000, 24
#define GOVRH_DAC_TEST_ENABLE		REG_GOVRH_DAC_TEST,BIT19,19
#define GOVRH_DAC_DC_TEST_MODE		REG_GOVRH_DAC_TEST,BIT18,18
#define GOVRH_DAC_AC_TEST_MODE		REG_GOVRH_DAC_TEST, 0x30000, 16
#define GOVRH_DAC_TEST_A			REG_GOVRH_DAC_TEST, 0xFF, 0

//REG_GOVRH_DAC_BTEST,0xac
#define GOVRH_DAC_TEST_B			REG_GOVRH_DAC_BTEST, 0xFF, 0

//REG_GOVRH_DAC_CTEST,0xb0
#define GOVRH_DAC_TEST_C			REG_GOVRH_DAC_CTEST, 0xFF, 0

//REG_GOVRH_DAC_DBG,0xb4
#define GOVRH_DAC_DAF				REG_GOVRH_DAC_DBG, 0x700, 8
#define GOVRH_DAC_BANDGAP			REG_GOVRH_DAC_DBG,BIT2,2
#define GOVRH_DAC_RD1ENC			REG_GOVRH_DAC_DBG,BIT1,1
#define GOVRH_DAC_RD0ENC			REG_GOVRH_DAC_DBG,BIT0,0

//REG_GOVRH_CONTRAST,0xb8
#define GOVRH_CONTRAST_YAF			REG_GOVRH_CONTRAST, 0xFF0000, 16
#define GOVRH_CONTRAST_PBAF			REG_GOVRH_CONTRAST, 0xFF00, 8
#define GOVRH_CONTRAST_PRAF			REG_GOVRH_CONTRAST, 0xFF, 0

//REG_GOVRH_BRIGHTNESS,0xbc
#define GOVRH_BRIGHTNESS_Y			REG_GOVRH_BRIGHTNESS, 0xFF, 0

//REG_GOVRH_CSC_MODE,0xe0
#define GOVRH_CSC_MOD				REG_GOVRH_CSC_MODE,BIT0,0
#define GOVRH_CSC_CLAMP_ENABLE		REG_GOVRH_CSC_MODE,BIT1,1

//REG_GOVRH_DVO_YUV2RGB,0xe4
#define GOVRH_DVO_YUV2RGB_ENABLE	REG_GOVRH_YUV2RGB,BIT0,0
#define GOVRH_VGA_YUV2RGB_ENABLE	REG_GOVRH_YUV2RGB,BIT1,1
#define GOVRH_RGB_MODE				REG_GOVRH_YUV2RGB,BIT2,2
#define GOVRH_DAC_CLKINV			REG_GOVRH_YUV2RGB,BIT3,3
#define GOVRH_BLANK_ZERO			REG_GOVRH_YUV2RGB,BIT4,4

//REG_GOVRH_H264_INPUT_EN, 0xe8
#define GOVRH_H264_INPUT_ENABLE		REG_GOVRH_H264_INPUT_EN,BIT0,0

//REG_GOVRH_DVO_PIX,0x30
#define GOVRH_DVO_YUV422			REG_GOVRH_DVO_PIX,BIT1,1
#define GOVRH_DVO_RGB				REG_GOVRH_DVO_PIX,BIT0,0

//REG_GOVRH_DVO_DLY_SEL,0x34
#define GOVRH_DVO_CLK_INV			REG_GOVRH_DVO_DLY_SEL,BIT12,12
#define GOVRH_DVO_CLK_DLY			REG_GOVRH_DVO_DLY_SEL, 0xFFF, 0
	
//Base1
//REG_GOVRH_INT,0x38
#define GOVRH_INT_MEM				REG_GOVRH_INT,BIT17,17
#define GOVRH_INT_SCD				REG_GOVRH_INT,BIT16,16
#define GOVRH_INT_MEM_ENABLE		REG_GOVRH_INT,BIT1,1
#define GOVRH_INT_SCD_ENABLE		REG_GOVRH_INT,BIT0,0

//REG_GOVRH_MIF,0x80
#define GOVRH_MIF_ENABLE			REG_GOVRH_MIF,BIT0,0

//REG_GOVRH_COLFMT,0x84
#define GOVRH_COLFMT				REG_GOVRH_COLFMT,BIT0,0

//REG_GOVRH_INFMT,0x88
#define GOVRH_INFMT					REG_GOVRH_SRCFMT,BIT0,0

//REG_GOVRH_DSTFMT,0x8C
#define GOVRH_OUTFMT				REG_GOVRH_DSTFMT,BIT0,0

//REG_GOVRH_YSA,0x90

//REG_GOVRH_CSA,0x94

//REG_GOVRH_AWIDTH,0x98
#define GOVRH_AWIDTH				REG_GOVRH_PIXWID, 0x7FF, 0

//REG_GOVRH_FWIDTH,0x9c
#define GOVRH_FWIDTH				REG_GOVRH_BUFWID, 0x7FF, 0

//REG_GOVRH_VCROP,0xa0
#define GOVRH_VCROP					REG_GOVRH_VCROP, 0x7FF, 0

//REG_GOVRH_HCROP,0xa4
#define GOVRH_HCROP					REG_GOVRH_HCROP, 0x7FF, 0

//REG_GOVRH_FHI,0xa8
#define GOVRH_FIFO_IND				REG_GOVRH_FHI, 0xF, 0

//REG_GOVRH_COLFMT2,0xac
#define GOVRH_COLFMT2				REG_GOVRH_COLFMT2,BIT0,0

//REG_GOVRH_REG_STS,0xe4
#define GOVRH_REG_LEVEL				REG_GOVRH_REG_STS,BIT8,8
#define GOVRH_REG_UPDATE			REG_GOVRH_REG_STS,BIT0,0

//REG_GOVRH_SWFLD,0xe8
#define GOVRH_SWFLD_ENABLE			REG_GOVRH_SWFLD,BIT1,1
#define GOVRH_SWFLD_FIXED			REG_GOVRH_SWFLD,BIT0,0

#endif				/* WM8510_GOVRH_REG_H */
