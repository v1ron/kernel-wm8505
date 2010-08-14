/**************************************************************		
Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.	
This program is free software: you can redistribute it and/or modify it under the terms 	
of the GNU General Public License as published by the Free Software Foundation, either
 	version 2 of the License, or (at your option) any later version.
	
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the GNU General Public License for more details. You should have received
 a copy of the GNU General Public License along with this program.  If not, see
 <http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.

--*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "vpp.h"
#include "vout.h"
#include "bus.h"
#include "edid.h"

// #define DEBUG
#ifdef DEBUG
#define VPPMSG(fmt, args...) DPRINT("[VO] %s: " fmt, __FUNCTION__ , ## args)
#else
#define VPPMSG(fmt, args...) do {} while(0)
#endif

// #define CONFIG_LCD_VGA_DUAL_OUTPUT

static int vo_plug_flag;
int (*vo_plug_func)(void);
static vout_dev_ops_t vout_dev_ops;
#ifdef CONFIG_VPP_EDID
char vout_edid[256];
#endif

static void vo_do_plug
(
	void *ptr		/*!<; // work input data */
)
{
	int plugin;

	if( vo_plug_func == 0 )
		return;

 	govrh_set_dvo_enable(1);
	plugin = vo_plug_func();
	govrh_set_dvo_enable(plugin);
	vo_plug_flag = 0;
	VPPMSG(KERN_DEBUG "vo_do_plug %d\n",plugin);
	return;
}
DECLARE_WORK(vo_plug_work,vo_do_plug);

static irqreturn_t vo_plug_interrupt_routine
(
	int irq, 				/*!<; // irq id */
	void *dev_id 			/*!<; // device id */
)
{
	VPPMSG(KERN_DEBUG "vo_plug_interrupt_routine\n");
	vppif_reg32_out((0xD8110000+0x304), 0x1<<VPP_VOINT_NO);	// clear int status
//	if( vo_plug_flag == 0 ){
		schedule_delayed_work(&vo_plug_work, HZ/5);
		vo_plug_flag = 1;
//	}
	return IRQ_HANDLED;
}

static void vo_plug_enable(int enable,void *func)
{
	VPPMSG(KERN_DEBUG "vo_plug_enable(%d)\n",enable);
	vo_plug_func = func;
	if( enable ){
		vppif_reg32_write(0xd811008c,0x2,1,0x0);			// GPIO1 input mode
		vo_do_plug(0);
		if ( request_irq(IRQ_GPIO1, vo_plug_interrupt_routine, IRQF_DISABLED, "vo plug", (void *) 0) ) {
			VPPMSG("*E* request GPIO ISR fail\n");
		}
		vppif_reg32_write(0xd8140044,0xFF000000,24,0x8);	// enable irq
	}
	else {
		vppif_reg32_write(0xd8140044,0xFF000000,24,0x0);	// disable irq
		free_irq(IRQ_GPIO1,(void *) 0);
	}
}

static int vo_vga_compatible(int arg)
{
	vout_mode_t mode;

	VPPMSG(KERN_DEBUG "vo_vga_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_LCD:
#ifndef CONFIG_LCD_VGA_DUAL_OUTPUT
			return 1;
#endif
		default:
			break;
	}
	return 0;
}

static int vo_vga_visible(int arg)
{
	VPPMSG(KERN_DEBUG "vo_vga_visible(%d)\n",arg);
	if( arg ){
		govrh_set_vga_enable(VPP_FLAG_ENABLE);
	}
	else {
		govrh_set_vga_enable(VPP_FLAG_DISABLE);
	}
	return 0;
}

static int vo_vga_config(int arg)
{
	VPPMSG(KERN_DEBUG "vo_vga_config\n");

#ifdef CONFIG_LCD_VGA_DUAL_OUTPUT
{
	vout_info_t *vo_info;

	vo_info = (vout_info_t *) arg;
	govrh_set_video_mode(vo_info->resx,vo_info->resy,vo_info->pixclock,0);
}
#endif
	return 0;
}

static int vo_vga_init(int arg)
{
	VPPMSG(KERN_DEBUG "vo_vga_init(%d)\n",arg);

	govrh_set_vga_enable(VPP_FLAG_ENABLE);
	vppm_set_int_enable(VPP_FLAG_ENABLE,VPP_INT_GOVRH_VBIS);
	return 0;
}

static int vo_vga_uninit(int arg)
{
	VPPMSG(KERN_DEBUG "vo_vga_uninit(%d)\n",arg);
	govrh_set_vga_enable(VPP_FLAG_DISABLE);
	vppm_set_int_enable(VPP_FLAG_DISABLE,VPP_INT_GOVRH_VBIS);
	govrh_set_DAC_pwrdn(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_vga_suspend(int arg)
{
	VPPMSG(KERN_DEBUG "vo_vga_suspend(%d)\n",arg);
	govrh_set_vga_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_vga_resume(int arg)
{
	VPPMSG(KERN_DEBUG "vo_vga_resume(%d)\n",arg);
	govrh_set_vga_enable(VPP_FLAG_ENABLE);
	return 0;
}

static int vo_vga_chkplug(int arg)
{
	int bk;
	int plugin;

	bk = p_govrh->vga_dac_sense_cnt;
	p_govrh->vga_dac_sense_cnt = 1;
	vppif_reg32_write(GOVRH_DAC_PWRDN, 0);
	govrh_monitor_DAC_sense();
	p_govrh->vga_dac_sense_cnt = bk;
	plugin = (vppif_reg32_read(GOVRH_DAC_PWRDN))? 0:1;
	VPPMSG(KERN_DEBUG "vo_vga_chkplug %d\n",plugin);
	return plugin;
}

static int vo_vga_get_edid(int arg)
{
	vpp_i2c_read(0xA0,0,vout_edid,256);
	return 0;
}

vout_ops_t vo_vga_ops =
{
	.init = vo_vga_init,
	.uninit = vo_vga_uninit,
	.compatible = vo_vga_compatible,
	.visible = vo_vga_visible,
	.config = vo_vga_config,
	.suspend = vo_vga_suspend,
	.resume = vo_vga_resume,
	.chkplug = vo_vga_chkplug,
	.get_edid = vo_vga_get_edid,
};

vout_t vo_vga_parm = {
	.ops = &vo_vga_ops,
	.name = "VGA",
	.option[0] = 0,
	.option[1] = 0,
	.option[2] = 0,
	.resx = VPP_HD_DISP_RESX,
	.resy = VPP_HD_DISP_RESY,
	.pixclk = (VPP_HD_DISP_RESX*VPP_HD_DISP_RESY*VPP_HD_DISP_FPS),
};

static int vo_dvi_compatible(int arg)
{
	vout_mode_t mode;

	VPPMSG(KERN_DEBUG "vo_dvi_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_DVO2HDMI:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_dvi_visible(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvi_visible(%d)\n",arg);
	govrh_set_dvo_enable(arg);
	return 0;
}

static int vo_dvi_config(int arg)
{
	vout_info_t *vo_info;

	VPPMSG(KERN_DEBUG "vo_dvi_config\n");

	vo_info = (vout_info_t *) arg;

	govrh_set_video_mode(vo_info->resx,vo_info->resy,vo_info->pixclock,0);
	return 0;
}

static int vo_dvi_init(int arg)
{
	vout_t *vo;

	VPPMSG(KERN_DEBUG "vo_dvi_init(%d)\n",arg);

	vo = vout_get_info(VOUT_DVI);
#ifdef CONFIG_VPP_VT1632_SW
	vt1632_set_mode(vo->option[1]);
	vt1632_set_power_down(VPP_FLAG_DISABLE);
	vo_plug_enable(VPP_FLAG_ENABLE,vt1632_check_plugin);
#endif
//	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	govrh_set_dvo_color_format(vo->option[0]);
//	govrh_set_dvo_clock_delay(parm.clk_inv,parm.clk_delay);
	govrh_set_dvo_outdatw(vo->option[1]);
//	govrh_set_dvo_sync_polar(parm.sync_polar);
	p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
	return 0;
}

static int vo_dvi_uninit(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvi_uninit(%d)\n",arg);

#ifdef CONFIG_VPP_VT1632_SW
	vo_plug_enable(VPP_FLAG_DISABLE,0);
	vt1632_set_power_down(VPP_FLAG_ENABLE);
#endif
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_dvi_suspend(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvi_suspend(%d)\n",arg);

	vo_plug_enable(VPP_FLAG_DISABLE,vo_plug_func);
#ifdef CONFIG_VPP_VT1632_SW
	vt1632_set_power_down(VPP_FLAG_ENABLE);
#endif
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_dvi_resume(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvi_resume(%d)\n",arg);

#ifdef CONFIG_VPP_VT1632_SW
	vt1632_init();
#endif
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	vo_plug_enable(VPP_FLAG_ENABLE,vo_plug_func);
	return 0;
}

static int vo_dvi_chkplug(int arg)
{
	int plugin;

#ifdef CONFIG_VPP_VT1632_SW
	plugin = vt1632_check_plugin();
#endif
	VPPMSG(KERN_DEBUG "vo_dvi_chkplug %d\n",plugin);
	return plugin;
}

static int vo_dvi_get_edid(int arg)
{
	int i;

	for(i=0;i<128;i++)
		swi2c_I2C_ReadRegs(0,0xA0,0x0,i,1,&vout_edid[i]);

	return 0;
}

vout_ops_t vo_dvi_ops =
{
	.init = vo_dvi_init,
	.uninit = vo_dvi_uninit,
	.compatible = vo_dvi_compatible,
	.visible = vo_dvi_visible,
	.config = vo_dvi_config,
	.suspend = vo_dvi_suspend,
	.resume = vo_dvi_resume,
	.chkplug = vo_dvi_chkplug,
	.get_edid = vo_dvi_get_edid,
};

vout_t vo_dvi_parm = {
	.ops = &vo_dvi_ops,
	.name = "DVI",
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_24,
	.option[2] = 0,
	.resx = VPP_HD_DISP_RESX,
	.resy = VPP_HD_DISP_RESY,
	.pixclk = (VPP_HD_DISP_RESX*VPP_HD_DISP_RESY*VPP_HD_DISP_FPS),
};

static int vo_dvo2hdmi_init(int arg)
{
	vout_t *vo;
	vdo_color_fmt colfmt;

	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_init(%d)\n",arg);

	vo = vout_get_info(VOUT_DVO2HDMI);
	vout_dev_ops.set_mode(&vo->option[0]);
	vo_plug_enable(VPP_FLAG_ENABLE,vout_dev_ops.check_plugin);
//	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	colfmt = (vo->option[0]==VDO_COL_FMT_YUV422V)? VDO_COL_FMT_YUV422H:vo->option[0];
	govrh_set_dvo_color_format(colfmt);
//	govrh_set_dvo_clock_delay(parm.clk_inv,parm.clk_delay);
	govrh_set_dvo_outdatw(vo->option[1]);
//	govrh_set_dvo_sync_polar(parm.sync_polar);
	p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
	return 0;
}

static int vo_dvo2hdmi_uninit(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_uninit(%d)\n",arg);
	vo_plug_enable(VPP_FLAG_DISABLE,0);
	vout_dev_ops.set_power_down(1);
	return 0;
}

static int vo_dvo2hdmi_suspend(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_suspend(%d)\n",arg);
	vo_plug_enable(VPP_FLAG_DISABLE,vo_plug_func);
	vout_dev_ops.set_power_down(1);
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

static int vo_dvo2hdmi_resume(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_resume(%d)\n",arg);
	vout_dev_ops.init(&vout_dev_ops);
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	vo_plug_enable(VPP_FLAG_ENABLE,vo_plug_func);
	return 0;
}

static int vo_dvo2hdmi_compatible(int arg)
{
	vout_mode_t mode;

	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
//		case VOUT_VGA:
		case VOUT_DVI:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_dvo2hdmi_visible(int arg)
{
	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_visible(%d)\n",arg);

	govrh_set_dvo_enable(arg);
	return 0;
}

static int vo_dvo2hdmi_config(int arg)
{
	vout_info_t *vo_info;
	unsigned int reg;
	unsigned int sense;
	extern int vpp_dac_sense_enable;

	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_config\n");

	vo_info = (vout_info_t *) arg;
	if( (vo_info->resx == 1280) && (vo_info->resy == 720) ){
		vo_info->pixclock = 74250060;
	}
	govrh_set_video_mode(vo_info->resx,vo_info->resy,vo_info->pixclock,0);

	sense = vpp_dac_sense_enable;
	vpp_dac_sense_enable = 0;
	reg = vppif_reg32_read(GOVRH_DAC_PWRDN);
	vppif_reg32_write(GOVRH_DAC_PWRDN,0xFF);
	govrh_set_tg_enable(VPP_FLAG_ENABLE);

	vout_dev_ops.config(vo_info);

	govrh_set_tg_enable(VPP_FLAG_DISABLE);
	vppif_reg32_write(GOVRH_DAC_PWRDN,reg);
	vpp_dac_sense_enable = sense;
	return 0;
}

static int vo_dvo2hdmi_chkplug(int arg)
{
	int plugin;

	mdelay(80);	// wait CAT6612 stable
	plugin = vout_dev_ops.check_plugin();
	VPPMSG(KERN_DEBUG "vo_dvo2hdmi_chkplug %d\n",plugin);
	return plugin;
}

static int vo_dvo2hdmi_get_edid(int arg)
{
	extern int GetEDIDData(int EDIDBlockID, unsigned char *pEDIDData);

	return GetEDIDData(0,vout_edid);
}

vout_ops_t vo_dvo2hdmi_ops =
{
	.init = vo_dvo2hdmi_init,
	.uninit = vo_dvo2hdmi_uninit,
	.compatible = vo_dvo2hdmi_compatible,
	.visible = vo_dvo2hdmi_visible,
	.config = vo_dvo2hdmi_config,
	.suspend = vo_dvo2hdmi_suspend,
	.resume = vo_dvo2hdmi_resume,
	.chkplug = vo_dvo2hdmi_chkplug,
	.get_edid = vo_dvo2hdmi_get_edid,
};

vout_t vo_dvo2hdmi_parm = {
	.ops = &vo_dvo2hdmi_ops,
	.name = "DVO2HDMI",
	.option[0] = VDO_COL_FMT_ARGB,
	.option[1] = VPP_DATAWIDHT_24,
	.option[2] = 0,
	.resx = 1280,
	.resy = 720,
	.pixclk = 74250060,
};

#ifdef CONFIG_LCD_WMT
static int vo_lcd_compatible(int arg)
{
	vout_mode_t mode;

	VPPMSG(KERN_DEBUG "vo_lcd_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_DVO2HDMI:
		case VOUT_DVI:
#ifndef CONFIG_LCD_VGA_DUAL_OUTPUT
		case VOUT_VGA:
#endif
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_lcd_visible(int arg)
{
	VPPMSG(KERN_DEBUG "vo_lcd_visible(%d)\n",arg);
	govrh_set_dvo_enable(arg);
	lcd_blt_enable(VPP_BLT_PWM_NUM,arg);
	return 0;
}

static int vo_lcd_config(int arg)
{
	vout_info_t *vo_info;
//	vpp_clock_t clock;

	VPPMSG(KERN_DEBUG "vo_lcd_config\n");

	vo_info = (vout_info_t *) arg;
	vo_info->resx = p_lcd->timing.hpixel;
	vo_info->resy = p_lcd->timing.vpixel;
	vo_info->pixclock = p_lcd->timing.pixel_clock;

#if 1
	govrh_set_video_mode(vo_info->resx,vo_info->resy,vo_info->pixclock,&p_lcd->timing);
#else
	vpp_trans_timing(VPP_MOD_GOVRH,&p_lcd->timing, &clock,1);
	clock.read_cycle = vpp_set_base_clock(VPP_MOD_GOVRH,p_lcd->timing.pixel_clock);
	g_vpp.govr->set_tg(&clock);
#endif
	return 0;
}

static int vo_lcd_init(int arg)
{
	vout_t *vo;

	VPPMSG(KERN_DEBUG "vo_lcd_init(%d)\n",arg);

	vo = vout_get_info(VOUT_LCD);
	if( (p_lcd = lcd_get_parm((lcd_panel_t) vo->option[0],vo->option[1])) ){
		printk("[LCD] %s (id %d,bpp %d)\n",p_lcd->name,vo->option[0],vo->option[1]);
	}
	else {
		printk("[LCD] *E* lcd %d not support\n",vo->option[0]);
		return -1;
	}

	if( p_lcd->initial ){
		p_lcd->initial();
	}
	lcd_blt_set_level(VPP_BLT_PWM_NUM,lcd_blt_level);
	lcd_blt_set_freq(VPP_BLT_PWM_NUM,lcd_blt_freq);
	lcd_blt_enable(VPP_BLT_PWM_NUM,0);

	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	govrh_set_dvo_color_format(VDO_COL_FMT_ARGB);
	govrh_set_dvo_outdatw(VPP_DATAWIDHT_24);
	govrh_set_dvo_clock_delay((p_lcd->capability & LCD_CAP_CLK_HI)? 0:1, 0);
	govrh_set_dvo_sync_polar((p_lcd->capability & LCD_CAP_HSYNC_HI)? 0:1,(p_lcd->capability & LCD_CAP_VSYNC_HI)? 0:1);
	p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
	g_vpp.vo_enable = 1;
	return 0;
}

static int vo_lcd_uninit(int arg)
{
	VPPMSG(KERN_DEBUG "vo_lcd_uninit(%d)\n",arg);
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	lcd_blt_enable(VPP_BLT_PWM_NUM,0);
	return 0;
}

extern unsigned int pwm_level[2];
extern unsigned int pwm_freq[2] ;
extern void pwm_set_freq(int no,unsigned int freq);
extern void pwm_set_level(int no,unsigned int level);
extern  void pwm_set_enable(int no,int enable);

static int vo_lcd_suspend(int arg)
{
	VPPMSG(KERN_DEBUG "vo_lcd_suspend(%d)\n",arg);
        printk("vo_lcd_suspend: lcd_blt_level = %d\n", lcd_blt_level);
	lcd_blt_set_level(VPP_BLT_PWM_NUM,0);
	govrh_set_dvo_enable(VPP_FLAG_DISABLE);
	return 0;
}

extern void send_ESC_key(void);
static int vo_lcd_resume(int arg)
{
	VPPMSG(KERN_DEBUG "vo_lcd_resume(%d)\n",arg);
	printk("vo_lcd_resume\n");

	REG32_VAL(0xd8130250) |= BIT10;
	govrh_set_dvo_enable(VPP_FLAG_ENABLE);
	mdelay(150);
        printk("lcd_blt_level = %d\n", lcd_blt_level);
//--> test
//      if(lcd_blt_level < 2)
//	  lcd_blt_level = 90;
//<--
	pwm_set_freq(VPP_BLT_PWM_NUM, pwm_freq[VPP_BLT_PWM_NUM]);
	 pwm_level[VPP_BLT_PWM_NUM] = lcd_blt_level;
//	lcd_blt_set_level(VPP_BLT_PWM_NUM, lcd_blt_level);
	pwm_set_level(VPP_BLT_PWM_NUM, pwm_level[VPP_BLT_PWM_NUM]);
	pwm_set_enable(VPP_BLT_PWM_NUM, 1);
//--> added by howayhuo on 20100709
//-- if the android lock the screen and close the backlight. we need send  key  to light the backlight
	if(lcd_blt_level < 2)
	    send_ESC_key();
//<-- end add
	return 0;
}

static int vo_lcd_chkplug(int arg)
{
	return (p_lcd)? 1:0;
}

vout_ops_t vo_lcd_ops =
{
	.init = vo_lcd_init,
	.uninit = vo_lcd_uninit,
	.compatible = vo_lcd_compatible,
	.visible = vo_lcd_visible,
	.config = vo_lcd_config,
	.suspend = vo_lcd_suspend,
	.resume = vo_lcd_resume,
	.chkplug = vo_lcd_chkplug,
};

vout_t vo_lcd_parm = {
	.ops = &vo_lcd_ops,
	.name = "LCD",
};
#endif

static int vo_boot_compatible(int arg)
{
	vout_mode_t mode;

	VPPMSG(KERN_DEBUG "vo_boot_compatible(%d)\n",arg);
	mode = (vout_mode_t) arg;
	switch(mode){
		case VOUT_VGA:
		case VOUT_DVI:
		case VOUT_DVO2HDMI:
		case VOUT_LCD:
		case VOUT_SD_ANALOG:
		case VOUT_SD_DIGITAL:
			return 1;
		default:
			break;
	}
	return 0;
}

static int vo_boot_visible(int arg)
{
	VPPMSG(KERN_DEBUG "vo_boot_visible(%d)\n",arg);
#ifdef WMT_FTBLK_GOVRH
	govrh_set_dvo_enable(arg);
#endif
	if( g_vpp.vo_enable ){
		lcd_blt_enable(VPP_BLT_PWM_NUM,1);
	}
	return 0;
}

static int vo_boot_config(int arg)
{
	vout_info_t *vo_info;

	VPPMSG(KERN_DEBUG "vo_boot_config\n");

	vo_info = (vout_info_t *) arg;
	return 0;
}

static int vo_boot_init(int arg)
{
	vout_t *vo;

	vo = vout_get_info(VOUT_BOOT);

	VPPMSG(KERN_DEBUG "vo_boot_init(%d)\n",arg);
#ifdef WMT_FTBLK_GOVRH
	govrh_get_framebuffer(&g_vpp.govr->fb_p->fb);
	p_govw->fb_p->fb.col_fmt = g_vpp.govr->fb_p->fb.col_fmt;
	if( g_vpp.direct_path ){
		g_vpp.direct_path_colfmt = g_vpp.govr->fb_p->fb.col_fmt;
		govw_set_tg_enable(VPP_FLAG_DISABLE);
		vppm_set_int_enable(VPP_FLAG_ENABLE,VPP_INT_GOVRH_PVBI+VPP_INT_GOVRH_VBIS);
		if( (p_govw->fb_p->capability & BIT(p_govrh->fb_p->fb.col_fmt)) == 0 ){
			g_vpp.govr->fb_p->fb.col_fmt = VDO_COL_FMT_YUV444;
			p_govw->fb_p->fb.col_fmt = VDO_COL_FMT_YUV444;
		}
	}
	else {
		if( (p_govw->fb_p->capability & BIT(p_govrh->fb_p->fb.col_fmt)) == 0 ){
			lcd_blt_enable(VPP_BLT_PWM_NUM,0);
			g_vpp.govr->fb_p->fb.col_fmt = VDO_COL_FMT_YUV444;
			p_govw->fb_p->fb.col_fmt = VDO_COL_FMT_YUV444;
			g_vpp.govr->fb_p->set_color_fmt(VDO_COL_FMT_YUV444);
			govrh_set_csc_mode(p_govrh->fb_p->csc_mode);
			g_vpp.vo_enable = 1;
		}
	}
#else
#endif
	return 0;
}
//extern int wmt_getsyspara(char *varname, unsigned char *varval, int varlen);
static int vo_boot_uninit(int arg)
{
	VPPMSG(KERN_DEBUG "vo_boot_uninit(%d)\n",arg);
	return 0;
}

static int vo_boot_suspend(int arg)
{
           vout_t *parm;
  //         unsigned char buf[40] = {0};
           int val;

           printk("vo_boot_suspend 1\n");
	   VPPMSG(KERN_DEBUG "vo_boot_suspend(%d)\n",arg);

           val = 1;

//           if( wmt_getsyspara("LCD_ENABLE",buf,40) == 0 ){
//                     sscanf(buf,"%d",&val);
//           }

           if( val ){        // LCD
/*
                     lcd_panel_id = 0;
                     if( wmt_getsyspara("LCD_ID",buf,40) == 0 ){
                                sscanf(buf,"%d",&val);
                                lcd_panel_id = val;
                     }
*/
                     parm = &vo_lcd_parm;
                     parm->option[0] = lcd_panel_id;    /* [LCD] option1 : panel id */
                     parm->option[1] = 24;           /* [LCD] option2 : bit per pixel */
                     p_lcd = lcd_get_parm(lcd_panel_id,24);
                     parm->resx = p_lcd->timing.hpixel;
                     parm->resy = p_lcd->timing.vpixel;
                     parm->pixclk = p_lcd->timing.pixel_clock;
                     printk("vo_boot_suspend 2\n");
                     vout_set_mode(VOUT_LCD,1);
           }
           else {           // DVI
                     vout_set_mode(VOUT_DVI,1);
           }
           printk("vo_boot_suspend 3\n");
           vout_suspend(VOUT_MODE_ALL,arg);
	    printk("vo_boot_suspend 4\n");

	return 0;
}

static int vo_boot_resume(int arg)
{
	VPPMSG(KERN_DEBUG "vo_boot_resume(%d)\n",arg);
	return 0;
}

static int vo_boot_chkplug(int arg)
{
	return 1;
}

vout_ops_t vo_boot_ops =
{
	.init = vo_boot_init,
	.uninit = vo_boot_uninit,
	.compatible = vo_boot_compatible,
	.visible = vo_boot_visible,
	.config = vo_boot_config,
	.suspend = vo_boot_suspend,
	.resume = vo_boot_resume,
	.chkplug = vo_boot_chkplug,
};

vout_t vo_boot_parm = {
	.ops = &vo_boot_ops,
	.name = "BOOT",
	.option[0] = 0,
	.option[1] = 0,
	.option[2] = 0,
	.resx = 0,
	.resy = 0,
	.pixclk = 0,
};

vout_mode_t vout_extdev_probe(void)
{
	vout_mode_t mode;
	vout_t *parm;

#ifdef CONFIG_LCD_WMT
	vout_register(VOUT_LCD,&vo_lcd_parm);
	if( lcd_init() == 0 ){
		printk("[VOUT] DVO to LCD\n");
		parm = &vo_lcd_parm;
		parm->option[0] = lcd_panel_id;			/* [LCD] option1 : panel id */
		parm->option[1] = 24;					/* [LCD] option2 : bit per pixel */
		p_lcd = lcd_get_parm(lcd_panel_id,24);
		parm->resx = p_lcd->timing.hpixel;
		parm->resy = p_lcd->timing.vpixel;
		parm->pixclk = p_lcd->timing.pixel_clock;
		return VOUT_LCD;
	}
#endif

	if( ad9389_init(&vout_dev_ops) == 0 ){
		mode = VOUT_DVO2HDMI;
		parm = &vo_dvo2hdmi_parm;
		printk("[VOUT] AD9389 DVO to HDMI\n");
		goto probe_end;
	}

#ifdef CONFIG_HDMI_CAT6610_WMT
	if( cat6612_init(&vout_dev_ops) == 0 ){
		mode = VOUT_DVO2HDMI;
		parm = &vo_dvo2hdmi_parm;
		printk("[VOUT] CAT6612 DVO to HDMI\n");
		goto probe_end;
	}
#endif

#ifdef CONFIG_VPP_VT1632_SW
	mode = VOUT_DVI;
	parm = &vo_dvi_parm;
	if( vt1632_init() == 0 ){
		printk("[VOUT] VT1632 sw mode\n");
	}
	else {
		printk("[VOUT] VT1632 hw mode\n");
	}
#endif
probe_end:
	vout_register(mode,parm);
	printk("[VOUT] ext dev : %s\n",parm->name);
	return mode;
}

vout_mode_t vout_intdev_probe(void)
{
	vout_mode_t mode = VOUT_VGA;
//	vout_t *parm = &vo_vga_parm;

	vout_register(VOUT_VGA,&vo_vga_parm);
//	printk("[VOUT] int dev : %s\n",parm->name);
	return mode;
}

vout_mode_t vout_priority[] = { VOUT_LCD, VOUT_DVO2HDMI, VOUT_DVI, VOUT_SD_DIGITAL, VOUT_VGA, VOUT_SD_ANALOG, VOUT_MODE_MAX };

int vout_init(struct fb_var_screeninfo *var)
{
	vout_t *vo;
	vout_mode_t dvo_vout_mode;
	vout_mode_t int_vout_mode;

	VPPMSG(KERN_ALERT "vo_init_wm8510\n");

#ifdef CONFIG_VPP_EDID
	memset(vout_edid,0,128);
#endif
	/* check video out device */
	dvo_vout_mode = vout_extdev_probe();
	int_vout_mode = vout_intdev_probe();

	if( vpp_vo_boot_arg[0] < VOUT_MODE_MAX ){	// boot argument
		vpp_timing_t *timing;

		vo = vout_get_info(vpp_vo_boot_arg[0]);
		vo->option[0] = vpp_vo_boot_arg[1];
		vo->option[1] = vpp_vo_boot_arg[2];
		timing = vpp_get_video_mode(vpp_vo_boot_arg[3], vpp_vo_boot_arg[4], vpp_vo_boot_arg[5]);
		var->xres = timing->hpixel;
		var->yres = timing->vpixel;
		var->pixclock = timing->pixel_clock;

		if( vpp_vo_boot_arg2[0] < VOUT_MODE_MAX ){
			vout_t *vo2;
			vo2 = vout_get_info(vpp_vo_boot_arg2[0]);
			vo2->option[0] = vpp_vo_boot_arg2[1];
			vo2->option[1] = vpp_vo_boot_arg2[2];
			vout_set_mode(vpp_vo_boot_arg2[0],1);
		}
		vout_set_mode(vpp_vo_boot_arg[0],1);
		return 0;
	}

	if( g_vpp.govrh_preinit ){	// boot logo
		vpp_clock_t tmr;
		vout_register(VOUT_BOOT,&vo_boot_parm);
		g_vpp.govr->get_tg(&tmr);
		var->xres = tmr.end_pixel_of_active - tmr.begin_pixel_of_active;
		var->yres = tmr.end_line_of_active - tmr.begin_line_of_active;
		var->pixclock = vpp_get_base_clock(VPP_PLL_C,VPP_DIV_DVO);
		vout_set_mode(VOUT_BOOT,1);
		return 0;
	}

	{
		int i;
		int match = 0;
		unsigned int user_resx,user_resy,user_freq;
//		unsigned char buf[40];

		//if( wmt_getsyspara("user_res",buf,40) ){
			user_resx = 1024;
			user_resy = 768;
			user_freq = 60;
		/*}
		else {
			sscanf(buf,"%dx%d@%d",&user_resx,&user_resy,&user_freq);
		}*/
		printk("user res %dx%d@%d\n",user_resx,user_resy,user_freq);

		// default
		vo = vout_get_info(VOUT_VGA);
		var->xres = vo->resx;
		var->yres = vo->resy;
		var->pixclock = vo->pixclk;
		vout_set_mode(dvo_vout_mode,1);
		vout_set_mode(VOUT_VGA,1);

		// detect plugin
		for(i=0;vout_priority[i]!=VOUT_MODE_MAX;i++){
			if( vout_get_info(vout_priority[i]) ){
				if( vout_chkplug(vout_priority[i]) ){
					vo = vout_get_info(vout_priority[i]);
#ifdef CONFIG_VPP_EDID
					if( vo->ops->get_edid ){
						if( vo->ops->get_edid(0) == 0 ){
							if( parse_edid(vout_edid) ){
								printk("*E* read EDID fail\n");
							}
							else {
								if( vout_priority[i]==VOUT_DVO2HDMI ){
									extern edid_info_t edid_info;
									if((edid_info.detail_timing[i].resx >= user_resx) && (edid_info.detail_timing[i].resy >= user_resy)){
										match = 1;
									}
								}
								else if( edid_find_support(user_resx,user_resy,user_freq) ){
									match = 1;
								}
							}
						}
					}
#endif

					if( match ){
						var->xres = user_resx;
						var->yres = user_resy;
						var->pixclock = user_freq;
					}
					else { 	// use default
						var->xres = vo->resx;
						var->yres = vo->resy;
						var->pixclock = vo->pixclk;
					}
					vout_set_mode(vout_priority[i],1);
					break;
				}
			}
		}
	}
	printk(KERN_ALERT "vo_init_wm8510 (%s %dx%d,%d)\n",vo->name,var->xres,var->yres,var->pixclock);
	return 0;
}

int vout_exit(void)
{
	vout_enable(VOUT_MODE_ALL,0);
	govrh_set_MIF_enable(VPP_FLAG_DISABLE);

	return 0;
}
