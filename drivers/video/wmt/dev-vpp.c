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


#define DEV_VPP_C

/*=== vt3392_gov.c =============================================================
*
* MODULE       : vpp-common.c --
* AUTHOR       : Sam Shen
* DATE         : 2009/01/06
*-----------------------------------------------------------------------------*/

// #include <fcntl.h>
// #include <unistd.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <asm/page.h>
#include <linux/mm.h>
//#include <asm/arch-wmt/memblock.h>
#include <mach/memblock.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/sysctl.h>
#include <linux/delay.h>

#include "vpp.h"
#include "govrh.h"
#include "vout.h"

#ifdef VPP_DEBUG
#define VPPMSG(fmt, args...) DPRINT("[VPP] %s: " fmt, __FUNCTION__ , ## args)
#else
#define VPPMSG(fmt, args...) do {} while(0)
#endif

#define THE_MB_USER			"VPP-MB"
#define VPP_PROC_NUM		10
#define VPP_DISP_FB_MAX		10
#define VPP_DISP_FB_NUM		4

typedef struct {
	void (*func)(void *arg);
	void *arg;
	struct list_head list;
} vpp_proc_t;

typedef struct {
	vpp_dispfb_t parm;
	vpp_pts_t pts;
	struct list_head list;
} vpp_dispfb_parm_t;

spinlock_t vpp_irqlock = SPIN_LOCK_UNLOCKED;
struct list_head vpp_free_list;
struct list_head vpp_govrh_vbis_list;
struct list_head vpp_govw_pvbi_list;
struct list_head vpp_govw_vbis_list;
struct list_head vpp_govw_vbie_list;
struct list_head vpp_scl_pvbi_list;
struct list_head vpp_scl_vbis_list;
struct list_head vpp_scl_vbie_list;
struct list_head vpp_vpu_vbie_list;

struct list_head vpp_disp_fb_list;
struct list_head vpp_disp_free_list;
#ifdef WMT_FTBLK_PIP
struct list_head vpp_pip_fb_list;
#endif

DECLARE_MUTEX(vpp_govrh_vbis_sem);
DECLARE_MUTEX(vpp_govw_pvbi_sem);
DECLARE_MUTEX(vpp_govw_vbis_sem);
DECLARE_MUTEX(vpp_govw_vbie_sem);
DECLARE_MUTEX(vpp_scl_pvbi_sem);
DECLARE_MUTEX(vpp_scl_vbis_sem);
DECLARE_MUTEX(vpp_scl_vbie_sem);
DECLARE_MUTEX(vpp_vpu_vbie_sem);
DECLARE_MUTEX(vpp_disp_vbie_sem);

static void vpp_do_tasklet(unsigned long data);
DECLARE_TASKLET(vpp_govrh_vbis_tasklet,vpp_do_tasklet,VPP_INT_GOVRH_VBIS);
DECLARE_TASKLET(vpp_govw_pvbi_tasklet,vpp_do_tasklet,VPP_INT_GOVW_PVBI);
DECLARE_TASKLET(vpp_govw_vbis_tasklet,vpp_do_tasklet,VPP_INT_GOVW_VBIS);
DECLARE_TASKLET(vpp_govw_vbie_tasklet,vpp_do_tasklet,VPP_INT_GOVW_VBIE);
DECLARE_TASKLET(vpp_scl_pvbi_tasklet,vpp_do_tasklet,VPP_INT_SCL_PVBI);
DECLARE_TASKLET(vpp_scl_vbis_tasklet,vpp_do_tasklet,VPP_INT_SCL_VBIS);
DECLARE_TASKLET(vpp_scl_vbie_tasklet,vpp_do_tasklet,VPP_INT_SCL_VBIE);
DECLARE_TASKLET(vpp_vpu_vbie_tasklet,vpp_do_tasklet,VPP_INT_VPU_VBIE);

vpp_proc_t vpp_proc_array[VPP_PROC_NUM];
vpp_dispfb_parm_t vpp_disp_fb_array[VPP_DISP_FB_MAX];

extern int vout_init(struct fb_var_screeninfo *var);
extern int vout_exit(void);
extern void wmt_i2c_xfer_continue_if(struct i2c_msg *msg, unsigned int num);
extern void wmt_i2c_xfer_if(struct i2c_msg *msg);
extern struct fb_var_screeninfo vfb_var;
int vpp_disp_fb_cnt(struct list_head *list);

static unsigned int vpp_pre_dispfb_y_addr;
static unsigned int vpp_pre_dispfb_c_addr;

int vpp_govw_vbis_cnt;
int vpp_govw_pvbi_cnt;
int vpp_vpu_disp_cnt;
int vpp_pip_disp_cnt;
int vpp_govw_tg_err_cnt;
int vpp_vpu_disp_skip_cnt;

int vpp_dac_sense_enable = 1;

char *vpp_colfmt_str[] = {"YUV420","YUV422H","YUV422V","YUV444","YUV411","GRAY","ARGB"};

static int __init vpp_get_boot_arg
(
	char *str			/*!<; // argument string */
)
{
	sscanf(str,"%d:%d:%d:%d:%d:%d",&vpp_vo_boot_arg[0],&vpp_vo_boot_arg[1],&vpp_vo_boot_arg[2],&vpp_vo_boot_arg[3],&vpp_vo_boot_arg[4],&vpp_vo_boot_arg[5]);
	switch( vpp_vo_boot_arg[0] ){
		case VOUT_SD_ANALOG:
		case VOUT_SD_DIGITAL:
		case VOUT_LCD:
		case VOUT_DVI:
		case VOUT_HDMI:
		case VOUT_DVO2HDMI:
		case VOUT_DVO:
		case VOUT_VGA:
			break;
		default:
			vpp_vo_boot_arg[0] = VOUT_MODE_MAX;
			return -1;
	}
	printk("[VPP] vpp boot arg %s opt %d,%d, %dx%d@%d\n",vpp_vout_str[vpp_vo_boot_arg[0]],vpp_vo_boot_arg[1],vpp_vo_boot_arg[2],
														vpp_vo_boot_arg[3],vpp_vo_boot_arg[4],vpp_vo_boot_arg[5]);
  	return 1;
} /* End of lcd_arg_panel_id */
__setup("wmtvo=", vpp_get_boot_arg);

static int __init vpp_get_boot_arg2
(
	char *str			/*!<; // argument string */
)
{
	sscanf(str,"%d:%d:%d",&vpp_vo_boot_arg2[0],&vpp_vo_boot_arg2[1],&vpp_vo_boot_arg2[2]);
	switch( vpp_vo_boot_arg[0] ){
		case VOUT_SD_ANALOG:
		case VOUT_SD_DIGITAL:
		case VOUT_LCD:
		case VOUT_DVI:
		case VOUT_HDMI:
		case VOUT_DVO2HDMI:
		case VOUT_DVO:
		case VOUT_VGA:
			break;
		default:
			vpp_vo_boot_arg2[0] = VOUT_MODE_MAX;
			return -1;
	}
	printk("[VPP] vpp boot arg2 %s opt %d,%d\n",vpp_vout_str[vpp_vo_boot_arg2[0]],vpp_vo_boot_arg2[1],vpp_vo_boot_arg2[2]);
  	return 1;
} /* End of lcd_arg_panel_id */
__setup("wmtvo2=", vpp_get_boot_arg2);

#ifdef CONFIG_PROC_FS
#define CONFIG_VPP_PROC
#ifdef CONFIG_VPP_PROC
unsigned int vpp_proc_value;
static int vpp_do_proc(ctl_table * ctl,int write,struct file *file,void *buffer,size_t * len,loff_t *ppos)
{
	int ret;

	if( !write ){
		switch( ctl->ctl_name ){
			case 1:
				vpp_proc_value = g_vpp.dbg_msg_level;
				break;
			case 5:
				vpp_proc_value = p_vpu->dei_mode;
				break;
#ifdef WMT_FTBLK_DISP
			case 6:
				vpp_proc_value = p_disp->dac_sense_val;
				break;
#endif
			case 7:
				vpp_proc_value = g_vpp.disp_fb_max;
				break;
			case 8:
				vpp_proc_value = g_vpp.govw_tg_dynamic;
				break;
			case 9:
				vpp_proc_value = g_vpp.govw_skip_all;
				break;
			case 10:
				vpp_proc_value = g_vpp.video_quality_mode;
				break;
			case 11:
				vpp_proc_value = g_vpp.scale_keep_ratio;
				break;
			default:
				break;
		}
	}

	ret = proc_dointvec(ctl, write, file, buffer, len, ppos);
	if( write ){
		switch( ctl->ctl_name ){
			case 1:
				g_vpp.dbg_msg_level = vpp_proc_value;
				break;
#ifdef CONFIG_LCD_WMT
			case 3:
				lcd_blt_set_level(VPP_BLT_PWM_NUM,lcd_blt_level);
				break;
			case 4:
				lcd_blt_set_freq(VPP_BLT_PWM_NUM,lcd_blt_freq);
				break;
#endif
#ifdef WMT_FTBLK_VPU
			case 5:
				p_vpu->dei_mode = vpp_proc_value;
				vpu_dei_set_mode(p_vpu->dei_mode);
				break;
#endif
#ifdef WMT_FTBLK_DISP
			case 6:
				p_disp->dac_sense_val = vpp_proc_value;
				break;
#endif
			case 7:
				g_vpp.disp_fb_max = vpp_proc_value;
				break;
			case 8:
				g_vpp.govw_tg_dynamic = vpp_proc_value;
				break;
			case 9:
				g_vpp.govw_skip_all = vpp_proc_value;
				break;
			case 10:
				g_vpp.video_quality_mode = vpp_proc_value;
				vpp_set_video_quality(g_vpp.video_quality_mode);
				break;
			case 11:
				g_vpp.scale_keep_ratio = vpp_proc_value;
				break;
#ifdef CONFIG_VPP_EDID
			case 12:
				{
				vout_t *vo;
				extern char vout_edid[];
				extern int parse_edid( unsigned char * edid );

				vo = vout_get_info(vpp_proc_value);
				if( vo->ops->get_edid ){
					if( vo->ops->get_edid(0) == 0 ){
						if( parse_edid(vout_edid) ){
							printk("*E* read EDID fail\n");
						}
					}
				}
				}
				break;
#endif
			default:
				break;
		}
	}
	return ret;
}

	struct proc_dir_entry *vpp_proc_dir = 0;

	static ctl_table vpp_table[] = {
	    {
			.ctl_name	= 1,
			.procname	= "dbg_msg",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
	    {
			.ctl_name	= 2,
			.procname	= "dac_sense_en",
			.data		= &vpp_dac_sense_enable,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &proc_dointvec,
		},
#ifdef CONFIG_LCD_BACKLIGHT
	    {
			.ctl_name	= 3,
			.procname	= "lcd_blt_level",
			.data		= &lcd_blt_level,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
	    {
			.ctl_name	= 4,
			.procname	= "lcd_blt_freq",
			.data		= &lcd_blt_freq,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
#endif
		{
			.ctl_name 	= 5,
			.procname	= "dei_mode",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 6,
			.procname	= "tv_dac_sense_val",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 7,
			.procname	= "disp_fb_max",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 8,
			.procname	= "govw_dynamic_fps",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 9,
			.procname	= "govw_skip_all",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 10,
			.procname	= "video_quality_mode",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 11,
			.procname	= "scale_keep_ratio",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{
			.ctl_name 	= 12,
			.procname	= "vout_edid",
			.data		= &vpp_proc_value,
			.maxlen		= sizeof(int),
			.mode		= 0666,
			.proc_handler = &vpp_do_proc,
		},
		{ .ctl_name = 0 }
	};

	static ctl_table vpp_root_table[] = {
		{
			.ctl_name	= CTL_DEV,
			.procname	= "vpp",	// create path ==> /proc/sys/vpp
			.mode		= 0555,
			.child 		= vpp_table
		},
		{ .ctl_name = 0 }
	};
	static struct ctl_table_header *vpp_table_header;
#endif
#endif

/*!*************************************************************************
* vpp_dbg_show()
*
* Private Function by Sam Shen, 2009/01/17
*/
/*!
* \brief	show debug message with time period
*
* \retval  None
*/
#define VPP_DBG_TMR_NUM		3
//#define VPP_DBG_DIAG_NUM	100
#ifdef VPP_DBG_DIAG_NUM
char vpp_dbg_diag_str[VPP_DBG_DIAG_NUM][100];
int vpp_dbg_diag_index;
int vpp_dbg_diag_delay;
#endif
void vpp_dbg_show(int level,int tmr,char *str)
{
	static struct timeval pre_tv[VPP_DBG_TMR_NUM];
	struct timeval tv;
	unsigned int tm_usec = 0;

	if( vpp_check_dbg_level(level)==0 )
		return;

	if( tmr && (tmr <= VPP_DBG_TMR_NUM) ){
		do_gettimeofday(&tv);
		if( pre_tv[tmr-1].tv_sec ){
			tm_usec = ( tv.tv_sec == pre_tv[tmr-1].tv_sec )? (tv.tv_usec - pre_tv[tmr-1].tv_usec):(1000000 + tv.tv_usec - pre_tv[tmr-1].tv_usec);
		}
		pre_tv[tmr-1] = tv;
	}

#ifdef VPP_DBG_DIAG_NUM
	if( level == VPP_DBGLVL_DIAG ){
		if( str ){
			char *ptr = &vpp_dbg_diag_str[vpp_dbg_diag_index][0];
			sprintf(ptr,"%s (%d,%d)(T%d %d usec)",str,(int)tv.tv_sec,(int)tv.tv_usec,tmr,(int) tm_usec);
			vpp_dbg_diag_index = (vpp_dbg_diag_index + 1) % VPP_DBG_DIAG_NUM;
		}

		if( vpp_dbg_diag_delay ){
			vpp_dbg_diag_delay--;
			if( vpp_dbg_diag_delay == 0 ){
				int i;

				printk("----- VPP DIAG -----\n");
				for(i=0;i<VPP_DBG_DIAG_NUM;i++){
					printk("%02d : %s\n",i,&vpp_dbg_diag_str[vpp_dbg_diag_index][0]);
					vpp_dbg_diag_index = (vpp_dbg_diag_index + 1) % VPP_DBG_DIAG_NUM;
				}
			}
		}
		return;
	}
#endif

	if( str ) {
		if( tmr ){
			printk("[VPP] %s (T%d period %d usec)\n",str,tmr-1,(int) tm_usec);
		}
		else {
			printk("[VPP] %s\n",str);
		}
	}
} /* End of vpp_dbg_show */

/*!*************************************************************************
* vpp_get_pid()
*
* Private Function by Sam Shen, 2009/03/11
*/
/*!
* \brief	get product id
*
* \retval  pid
*/
unsigned int vpp_get_pid(void)
{
	unsigned int pid;

#if(WMT_CUR_PID == WMT_PID_8510)	//ProjectID: wm3426
	pid = 0x851000A1;
#elif(WMT_CUR_PID == WMT_PID_8435)	//ProjectID: wm3437
	pid = 0x843500A0;
#elif(WMT_CUR_PID == WMT_PID_8425)	//ProjectID: wm3429
	pid = 0x842500A0;
#endif
	// reg 0xd8120000
	return pid;
} /* End of vpp_get_pid */

#ifdef CONFIG_PROC_FS
/*!*************************************************************************
* vpp_sts_read_proc()
*
* Private Function by Sam Shen, 2009/01/17
*/
/*!
* \brief	vpp stauts read proc
*
* \retval  None
*/
static int vpp_sts_read_proc(char *buf, char **start, off_t offset, int len)
{
	unsigned int yaddr,caddr;
	char *p = buf;
	static struct timeval pre_tv;
	struct timeval tv;
	unsigned int tm_usec;

	p += sprintf(p, "--- VPP HW status ---\n");
#ifdef WMT_FTBLK_GOVRH
	p += sprintf(p, "GOVRH memory read underrun error %d\n",vppif_reg32_read(GOVRH_INT_MEM));
	p_govrh->clr_sts(VPP_INT_ALL);
#endif

#ifdef WMT_FTBLK_GOVW
	p += sprintf(p, "GOVW TG error %d\n",vppif_reg32_read(GOVW_INTSTS_TGERR));
	p += sprintf(p, "GOVW Y fifo overflow %d\n",vppif_reg32_read(GOVW_INTSTS_MIFYERR));
	p += sprintf(p, "GOVW C fifo overflow %d\n",vppif_reg32_read(GOVW_INTSTS_MIFCERR));
	p_govw->clr_sts(VPP_INT_ALL);
#endif

#ifdef WMT_FTBLK_GOVM
	p += sprintf(p, "GOVM VPU not ready %d\n",(vppif_reg32_read(GOVM_INTSTS_VPU_READY))?0:1);
	p += sprintf(p, "GOVM GE not ready %d\n",(vppif_reg32_read(GOVM_INTSTS_GE_READY))?0:1);
	p += sprintf(p, "GE not ready G1 %d, G2 %d\n",vppif_reg32_read(0xD80504F4,BIT0,0),vppif_reg32_read(0xD80504F4,BIT1,1));
	REG32_VAL(0xD80504f4) |= 0x3;
#ifdef WMT_FTBLK_PIP
	p += sprintf(p, "GOVM PIP not ready %d\n",(vppif_reg32_read(GOVM_INTSTS_PIP_READY))?0:1);
	p += sprintf(p, "GOVM PIP Y error %d\n",vppif_reg32_read(GOVM_INT_PIP_Y_ERR));
	p += sprintf(p, "GOVM PIP C error %d\n",vppif_reg32_read(GOVM_INT_PIP_C_ERR));
#endif
	p_govm->clr_sts(VPP_INT_ALL);
#endif

#ifdef WMT_FTBLK_SCL
	p += sprintf(p, "SCL TG error %d\n",vppif_reg32_read(SCL_INTSTS_TGERR));
	p += sprintf(p, "SCLR MIF1 read error %d\n",vppif_reg32_read(SCLR_INTSTS_R1MIFERR));
	p += sprintf(p, "SCLR MIF2 read error %d\n",vppif_reg32_read(SCLR_INTSTS_R2MIFERR));
	p += sprintf(p, "SCLW RGB fifo overflow %d\n",vppif_reg32_read(SCLW_INTSTS_MIFRGBERR));
	p += sprintf(p, "SCLW Y fifo overflow %d\n",vppif_reg32_read(SCLW_INTSTS_MIFYERR));
	p += sprintf(p, "SCLW C fifo overflow %d\n",vppif_reg32_read(SCLW_INTSTS_MIFCERR));
	p_scl->clr_sts(VPP_INT_ALL);
#endif

#ifdef WMT_FTBLK_VPU
	p += sprintf(p, "VPU TG error %d\n",vppif_reg32_read(VPU_INTSTS_TGERR));
	p += sprintf(p, "VPUR MIF1 read error %d\n",vppif_reg32_read(VPU_R_INTSTS_R1MIFERR));
	p += sprintf(p, "VPUW Y fifo overflow %d\n",vppif_reg32_read(VPU_W_MIF_YERR));
	p += sprintf(p, "VPUW C fifo overflow %d\n",vppif_reg32_read(VPU_W_MIF_YERR));
	p_vpu->clr_sts(VPP_INT_ALL);
#endif

	if( REG32_VAL(0xd8050650) < vppif_reg32_read(GOVM_DISP_X_CR) ){
		p += sprintf(p, "*E* GE resx %d < GOV resx %d\n",REG32_VAL(0xd8050650),vppif_reg32_read(GOVM_DISP_X_CR));
	}
	if( REG32_VAL(0xd8050654) < vppif_reg32_read(GOVM_DISP_Y_CR) ){
		p += sprintf(p, "*E* GE resy %d < GOV resy %d\n",REG32_VAL(0xd8050654),vppif_reg32_read(GOVM_DISP_Y_CR));
	}
	p += sprintf(p, "G1 Enable %d,G2 Enable %d\n",REG32_VAL(0xd80506a8),REG32_VAL(0xd80506ac));

	p += sprintf(p, "--- VPP fb Address ---\n");
#ifdef WMT_FTBLK_VPU
	vpu_r_get_fb_addr(&yaddr,&caddr);
	p += sprintf(p, "VPU fb addr Y(0x%x) 0x%x, C(0x%x) 0x%x\n",REG_VPU_R_Y1SA,yaddr,REG_VPU_R_C1SA,caddr);
#else
	sclr_get_fb_addr(&yaddr,&caddr);
	p += sprintf(p, "VPU fb addr Y(0x%x) 0x%x, C(0x%x) 0x%x\n",REG_SCLR_YSA,yaddr,REG_SCLR_CSA,caddr);
#endif

#ifdef WMT_FTBLK_GOVW
	govw_get_hd_fb_addr(&yaddr,&caddr);
	p += sprintf(p, "GOVW fb addr Y(0x%x) 0x%x, C(0x%x) 0x%x\n",REG_GOVW_HD_YSA,yaddr,REG_GOVW_HD_CSA,caddr);
#endif
#ifdef WMT_FTBLK_GOVRH
	govrh_get_fb_addr(&yaddr,&caddr);
	p += sprintf(p, "GOVRH fb addr Y(0x%x) 0x%x, C(0x%x) 0x%x\n",REG_GOVRH_YSA,yaddr,REG_GOVRH_CSA,caddr);
#endif
	p += sprintf(p, "--- VPP SW status ---\n");

	do_gettimeofday(&tv);
	tm_usec=(tv.tv_sec==pre_tv.tv_sec)? (tv.tv_usec-pre_tv.tv_usec):(1000000*(tv.tv_sec-pre_tv.tv_sec)+tv.tv_usec-pre_tv.tv_usec);
	p += sprintf(p, "Time period %d usec,fps %d\n",(int) tm_usec,(1000000*g_vpp.dbg_govw_vbis_cnt/tm_usec));
	pre_tv = tv;

	p += sprintf(p, "GOVW VBIS INT cnt %d\n",g_vpp.dbg_govw_vbis_cnt);
	p += sprintf(p, "GOVW PVBI INT cnt %d (toggle dual buf)\n",g_vpp.dbg_govw_pvbi_cnt);
	p += sprintf(p, "GOVW TG ERR INT cnt %d\n",vpp_govw_tg_err_cnt);
	p += sprintf(p, "VPU disp fb cnt %d, skip %d\n",vpp_vpu_disp_cnt,vpp_vpu_disp_skip_cnt);
	p += sprintf(p, "PIP disp fb cnt %d\n",vpp_pip_disp_cnt);
#ifdef WMT_FTBLK_PIP
	p += sprintf(p, "Queue cnt disp:%d,pip %d\n",vpp_disp_fb_cnt(&vpp_disp_fb_list),vpp_disp_fb_cnt(&vpp_pip_fb_list));
#else
	p += sprintf(p, "Queue cnt disp:%d\n",vpp_disp_fb_cnt(&vpp_disp_fb_list));
#endif

	g_vpp.dbg_govw_vbis_cnt = 0;
	g_vpp.dbg_govw_pvbi_cnt = 0;
	vpp_vpu_disp_cnt = 0;
	vpp_pip_disp_cnt = 0;
	vpp_govw_tg_err_cnt = 0;
	vpp_vpu_disp_skip_cnt = 0;

	return (p - buf);
} /* End of vpp_sts_read_proc */

/*!*************************************************************************
* vpp_reg_read_proc()
*
* Private Function by Sam Shen, 2009/01/17
*/
/*!
* \brief	vpp register read proc
*
* \retval  None
*/
static int vpp_reg_read_proc(char *buf,char **start,off_t offset,int len)
{
	char *p = buf;
	vpp_mod_base_t *mod_p;
	int i;

	DPRINT("Product ID:0x%x\n",vpp_get_pid());
	for(i=0;i<VPP_MOD_MAX;i++){
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->dump_reg ){
			mod_p->dump_reg();
		}
	}

	p += sprintf(p, "Dump VPP HW register by kernel message\n");

	return (p-buf);
} /* End of vpp_reg_read_proc */

static char *vpp_show_module(vpp_mod_t mod,char *p)
{
	vpp_mod_base_t *mod_p;
	vpp_fb_base_t *fb_p;
	vdo_framebuf_t *fb;

	mod_p = vpp_mod_get_base(mod);
	p += sprintf(p, "int catch 0x%x\n",mod_p->int_catch);

	fb_p = mod_p->fb_p;
	if( fb_p ){
		fb = &fb_p->fb;
		p += sprintf(p, "----- frame buffer -----\n");
		p += sprintf(p, "Y addr 0x%x, size %d\n",fb->y_addr,fb->y_size);
		p += sprintf(p, "C addr 0x%x, size %d\n",fb->c_addr,fb->c_size);
		p += sprintf(p, "W %d, H %d, FB W %d, H %d\n",fb->img_w,fb->img_h,fb->fb_w,fb->fb_h);
		p += sprintf(p, "bpp %d, color fmt %s\n",fb->bpp,vpp_colfmt_str[fb->col_fmt]);
		p += sprintf(p, "H crop %d, V crop %d\n",fb->h_crop,fb->v_crop);

		p += sprintf(p, "CSC mode %d,frame rate %d\n",fb_p->csc_mode,fb_p->framerate);
		p += sprintf(p, "media fmt %d,wait ready %d\n",fb_p->media_fmt,fb_p->wait_ready);
	}
	return p;
}

/*!*************************************************************************
* vpp_info_read_proc()
*
* Private Function by Sam Shen, 2009/01/17
*/
/*!
* \brief	vpp infomation read proc
*
* \retval  None
*/
static int vpp_info_read_proc(char *buf,char **start,off_t offset,int len)
{
	char *p = buf;

	p += sprintf(p, "========== VPP ==========\n");
	p += sprintf(p, "direct path %d, col fmt %d\n",g_vpp.direct_path,g_vpp.direct_path_colfmt);
	p += sprintf(p, "mb0 0x%x,mb1 0x%x\n",g_vpp.mb[0],g_vpp.mb[1]);

#ifdef WMT_FTBLK_GOVRH
	p += sprintf(p, "========== GOVRH ==========\n");
	p += sprintf(p, "VGA DAC SENSE cnt %d\n",p_govrh->vga_dac_sense_cnt);
	p = vpp_show_module(VPP_MOD_GOVRH,p);
#endif

	p += sprintf(p, "========== GOVW ==========\n");
	p = vpp_show_module(VPP_MOD_GOVW,p);

	p += sprintf(p, "========== GOVM ==========\n");
	p += sprintf(p, "path 0x%x\n",p_govm->path);
	p = vpp_show_module(VPP_MOD_GOVM,p);

	p += sprintf(p, "========== VPU ==========\n");
	p += sprintf(p, "visual res (%d,%d),pos (%d,%d)\n",p_vpu->resx_visual,p_vpu->resy_visual,p_vpu->posx,p_vpu->posy);
	p = vpp_show_module(VPP_MOD_VPU,p);

	p += sprintf(p, "========== SCLR ==========\n");
	p = vpp_show_module(VPP_MOD_SCL,p);

	p += sprintf(p, "========== SCLW ==========\n");
	p = vpp_show_module(VPP_MOD_SCLW,p);
	return (p-buf);
} /* End of vpp_info_read_proc */
#endif

/*!*************************************************************************
* vpp_get_info()
*
* Private Function by Sam Shen, 2009/08/06
*/
/*!
* \brief	get current vpp info
*
* \retval  None
*/
void vpp_get_info(struct fb_var_screeninfo *var)
{
	var->xres = vfb_var.xres;
	var->yres = vfb_var.yres;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	var->pixclock = vfb_var.pixclock;
}

/*!*************************************************************************
* vpp_proc_flag()
*
* Private Function by Sam Shen, 2009/08/14
*/
/*!
* \brief	process vpp flag in specify period
*
* \retval  None
*/
int vpp_proc_flag
(
	vpp_int_t type,
	int *flag,
	int wait
)
{
	int ret;
	struct semaphore *wait_sem = 0;
	int flags;

	ret = 0;
	spin_lock_irqsave(&vpp_irqlock, flags);
	switch(type){
		case VPP_INT_DISP_VBIE:
			*flag = 1;
			wait_sem = &vpp_disp_vbie_sem;
			break;
		default:
			return -1;
	}
	spin_unlock_irqrestore(&vpp_irqlock, flags);
	if( wait ) down_interruptible(wait_sem);
	return ret;
}

/*!*************************************************************************
* vpp_proc_func()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	process vpp function in specify period
*
* \retval  None
*/
int vpp_proc_func
(
	void (*func)(void *argc),	/*!<; // process function pointer */
	void *arg,					/*!<; // process argument pointer */
	vpp_int_t type,				/*!<; // proc type */
	int wait					/*!<; // wait : 0 - no wait, 1 - wait */
)
{
	vpp_proc_t *entry;
	struct list_head *ptr;
	struct semaphore *wait_sem = 0;
	int flags;
	int ret;

//	VPPMSG("Enter vpp_proc_func(type %d,wait %d)\n",type,wait);

	if( list_empty(&vpp_free_list) ){
		VPPMSG("*E* proc array empty\n");
		return -1;
	}

	ret = 0;
	spin_lock_irqsave(&vpp_irqlock, flags);

	ptr = vpp_free_list.next;
	entry = list_entry(ptr,vpp_proc_t,list);
	list_del_init(ptr);
	entry->func = func;
	entry->arg = arg;
	switch(type){
		case VPP_INT_GOVRH_VBIS:
			list_add_tail(&entry->list,&vpp_govrh_vbis_list);
			wait_sem = &vpp_govrh_vbis_sem;
			break;
		case VPP_INT_GOVW_PVBI:
			list_add_tail(&entry->list,&vpp_govw_pvbi_list);
			wait_sem = &vpp_govw_pvbi_sem;
			break;
		case VPP_INT_GOVW_VBIS:
			list_add_tail(&entry->list,&vpp_govw_vbis_list);
			wait_sem = &vpp_govw_vbis_sem;
			break;
		case VPP_INT_GOVW_VBIE:
			list_add_tail(&entry->list,&vpp_govw_vbie_list);
			wait_sem = &vpp_govw_vbie_sem;
			break;
		case VPP_INT_SCL_PVBI:
			list_add_tail(&entry->list,&vpp_scl_pvbi_list);
			wait_sem = &vpp_scl_pvbi_sem;
			break;
		case VPP_INT_SCL_VBIS:
			list_add_tail(&entry->list,&vpp_scl_vbis_list);
			wait_sem = &vpp_scl_vbis_sem;
			break;
		case VPP_INT_SCL_VBIE:
			list_add_tail(&entry->list,&vpp_scl_vbie_list);
			wait_sem = &vpp_scl_vbie_sem;
			break;
		case VPP_INT_VPU_VBIE:
			list_add_tail(&entry->list,&vpp_vpu_vbie_list);
			wait_sem = &vpp_vpu_vbie_sem;
			break;
		default:
			VPPMSG("*E* vpp proc type\n");
			ret = -1;
			wait = 0;
			break;
	}
	spin_unlock_irqrestore(&vpp_irqlock, flags);
	if( wait ) down_interruptible(wait_sem);
//	VPPMSG("Exit vpp_proc_func\n");

	return ret;
} /* End of vpp_proc_func */

/*!*************************************************************************
* vpp_do_tasklet()
*
* Private Function by Sam Shen, 2009/01/17
*/
/*!
* \brief	vpp interrupt routine post process
*
* \retval  None
*/
static void vpp_do_tasklet
(
	unsigned long data		/*!<; // tasklet input data */
)
{
	vpp_proc_t *entry;
	struct semaphore *wait_sem;
	struct list_head *tasklet_list;
	struct list_head *ptr;
	int flags;

	spin_lock_irqsave(&vpp_irqlock, flags);

	switch(data){
		case VPP_INT_GOVRH_VBIS:
			tasklet_list = &vpp_govrh_vbis_list;
			wait_sem = &vpp_govrh_vbis_sem;
			break;
		case VPP_INT_GOVW_PVBI:
			tasklet_list = &vpp_govw_pvbi_list;
			wait_sem = &vpp_govw_pvbi_sem;
			break;
		case VPP_INT_GOVW_VBIS:
			tasklet_list = &vpp_govw_vbis_list;
			wait_sem = &vpp_govw_vbis_sem;
			break;
		case VPP_INT_GOVW_VBIE:
			tasklet_list = &vpp_govw_vbie_list;
			wait_sem = &vpp_govw_vbie_sem;
			break;
		case VPP_INT_SCL_PVBI:
			tasklet_list = &vpp_scl_pvbi_list;
			wait_sem = &vpp_scl_pvbi_sem;
			break;
		case VPP_INT_SCL_VBIS:
			tasklet_list = &vpp_scl_vbis_list;
			wait_sem = &vpp_scl_vbis_sem;
			break;
		case VPP_INT_SCL_VBIE:
			tasklet_list = &vpp_scl_vbie_list;
			wait_sem = &vpp_scl_vbie_sem;
			break;
		case VPP_INT_VPU_VBIE:
			tasklet_list = &vpp_vpu_vbie_list;
			wait_sem = &vpp_vpu_vbie_sem;
			break;
		default:
			return;
	}

	do {
		if( list_empty(tasklet_list) )
			break;

		/* get task from work head queue */
		ptr = tasklet_list->next;
		entry = list_entry(ptr,vpp_proc_t,list);
		if( entry->func ){
			entry->func(entry->arg);
		}
		list_del_init(ptr);
		list_add_tail(&entry->list,&vpp_free_list);
	} while(1);

	spin_unlock_irqrestore(&vpp_irqlock, flags);
	up(wait_sem);
//	VPPMSG("do tasklet\n");

} /* End of vpp_do_tasklet */

/*!*************************************************************************
* vpp_govw_dynamic_tg_set_rcyc()
*
* Private Function by Sam Shen, 2009/11/06
*/
/*!
* \brief	set govw tg
*
* \retval  None
*/
#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
void vpp_govw_dynamic_tg_set_rcyc(int rcyc)
{
#ifdef PATCH_SCL_SCALEDN
	if( sclr_get_color_format() == VDO_COL_FMT_ARGB ){
		if( (vppif_reg32_read(SCL_VSCLUP_ENABLE) == 0) || (vppif_reg32_read(SCL_HSCLUP_ENABLE)==0) ){
			rcyc += 2;
		}
	}
#endif
	rcyc = (rcyc > 0xFF)? 0xFF:rcyc;
	vppif_reg32_write(GOVW_TG_RDCYC,rcyc);
}

/*!*************************************************************************
* vpp_govw_dynamic_tg()
*
* Private Function by Sam Shen, 2009/10/14
*/
/*!
* \brief	check govw tg error and recover status
*
* \retval  None
*/
void vpp_govw_dynamic_tg(int err)
{
	int rcyc;
	int diff;

	if( g_vpp.govw_tg_dynamic == 0 )
		return;

	if( err ){
		g_vpp.govw_tg_rtn_cnt = 0;
		rcyc = vppif_reg32_read(GOVW_TG_RDCYC);
		rcyc = (rcyc >= 0xFF)? 255:(rcyc+1);
//		vppif_reg32_write(GOVW_TG_ENABLE,0x0);
		vpp_govw_dynamic_tg_set_rcyc(rcyc);
//		vppif_reg32_write(GOVW_TG_ENABLE,0x1);
		if( vpp_check_dbg_level(VPP_DBGLVL_TG) ){
			printk("[VPP] adjust GOVW rcyc %d\n",rcyc);
		}
	}
	else {
		g_vpp.govw_tg_rtn_cnt++;
		if( g_vpp.govw_tg_rtn_cnt > g_vpp.govw_tg_rtn_max){
			g_vpp.govw_tg_rtn_cnt = 0;
			rcyc = vppif_reg32_read(GOVW_TG_RDCYC);
			if (rcyc > g_vpp.govw_tg_rcyc){
				diff = rcyc - g_vpp.govw_tg_rcyc + 1;
				rcyc -= (diff/2);
//				vppif_reg32_write(GOVW_TG_ENABLE,0x0);
				vpp_govw_dynamic_tg_set_rcyc(rcyc);
//				vppif_reg32_write(GOVW_TG_ENABLE,0x1);
				if( vpp_check_dbg_level(VPP_DBGLVL_TG) ){
					printk("[VPP] return GOVW rcyc %d\n",rcyc);
				}
			}
		}
	}
} /* End of vpp_govw_dynamic_tg */
#endif


/*!*************************************************************************
* vpp_disp_fb_cnt()
*
* Private Function by Sam Shen, 2009/03/05
*/
/*!
* \brief	clear display frame buffer queue
*
* \retval  0 - success
*/
int vpp_disp_fb_cnt(struct list_head *list)
{
	struct list_head *ptr;
	int cnt;
	int flags;

	spin_lock_irqsave(&vpp_irqlock, flags);
	cnt = 0;
	ptr = list;
	while( ptr->next != list ){
		ptr = ptr->next;
		cnt++;
	}
	spin_unlock_irqrestore(&vpp_irqlock, flags);
	return cnt;
}

/*!*************************************************************************
* vpp_disp_fb_add()
*
* Private Function by Sam Shen, 2009/02/02
*/
/*!
* \brief	add display frame to display queue
*
* \retval  None
*/
static int vpp_disp_fb_add
(
	vpp_dispfb_t *fb		/*!<; // display frame pointer */
)
{
	vpp_dispfb_parm_t *entry;
	struct list_head *ptr;
	int flags;
	unsigned int yaddr,caddr;
	struct list_head *fb_list;

//	vpp_set_dbg_gpio(4,0xFF);
	if( vpp_check_dbg_level(VPP_DBGLVL_PLAYBACK) ){
		static struct timeval pre_tv;
		struct timeval tv;
		unsigned int tm_usec = 0;

		do_gettimeofday(&tv);
		if( pre_tv.tv_sec ){
			tm_usec = ( tv.tv_sec == pre_tv.tv_sec )? (tv.tv_usec - pre_tv.tv_usec):(1000000 + tv.tv_usec - pre_tv.tv_usec);
		}
		pre_tv = tv;
		if( tm_usec >= 1000000 / p_govw->fb_p->framerate ){
			printk("[VPP] *W* add disp fb period %d usec\n",tm_usec);
		}
	}

#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
	if( g_vpp.govw_tg_dynamic ){
		g_vpp.govw_tg_rtn_cnt = 0;
		vpp_govw_dynamic_tg_set_rcyc(g_vpp.govw_tg_rcyc);
	}
#endif

	if( list_empty(&vpp_disp_free_list) ){
		return -1;
	}

	spin_lock_irqsave(&vpp_irqlock, flags);
	if( (fb->flag & VPP_FLAG_DISPFB_PIP) == 0 ){
		fb_list = &vpp_disp_fb_list;
		if( g_vpp.disp_fb_cnt >= g_vpp.disp_fb_max ){
			VPPMSG("*W* disp queue full\n");
			return -1;
		}
		g_vpp.disp_fb_cnt++;
		vpp_dbg_show(VPP_DBGLVL_DISPFB,1,"add disp fb");
	}
#ifdef WMT_FTBLK_PIP
	else {
		fb_list = &vpp_pip_fb_list;
	}
#endif

	ptr = vpp_disp_free_list.next;
	entry = list_entry(ptr,vpp_dispfb_parm_t,list);
	list_del_init(ptr);
	entry->parm = *fb;

	memcpy(&entry->pts,&g_vpp.frame_pts,sizeof(vpp_pts_t));
	list_add_tail(&entry->list,fb_list);
	yaddr = caddr = 0;
	if( entry->parm.flag & VPP_FLAG_DISPFB_ADDR ){
		yaddr = entry->parm.yaddr;
		caddr = entry->parm.caddr;
	}
	else if(entry->parm.flag & VPP_FLAG_DISPFB_INFO){
		yaddr = entry->parm.info.y_addr;
		caddr = entry->parm.info.c_addr;
	}

	if( yaddr ){
		yaddr = (unsigned int)phys_to_virt(yaddr);
		mb_get(yaddr);
	}

	if( caddr ){
		caddr = (unsigned int)phys_to_virt(caddr);
		mb_get(caddr);
	}
	spin_unlock_irqrestore(&vpp_irqlock, flags);
	return 0;
} /* End of vpp_disp_fb_add */

/*!*************************************************************************
* vpp_disp_fb_clr()
*
* Private Function by Sam Shen, 2009/03/05
*/
/*!
* \brief	clear display frame buffer queue
*
* \retval  0 - success
*/
static int vpp_disp_fb_clr(int pip)
{
	vpp_dispfb_parm_t *entry;
	struct list_head *fb_list,*ptr;
	int flags;
	unsigned int yaddr,caddr;

	spin_lock_irqsave(&vpp_irqlock, flags);

#ifdef WMT_FTBLK_PIP
	fb_list = (pip)? &vpp_pip_fb_list:&vpp_disp_fb_list;
	yaddr = (pip)? p_pip->pre_yaddr:vpp_pre_dispfb_y_addr;
	caddr = (pip)? p_pip->pre_caddr:vpp_pre_dispfb_c_addr;

	if( yaddr ) mb_put(yaddr);
	if( caddr )	mb_put(caddr);
	if( pip ){
		p_pip->pre_yaddr = 0;
		p_pip->pre_caddr = 0;
	}
	else {
		vpp_pre_dispfb_y_addr = 0;
		vpp_pre_dispfb_c_addr = 0;
		g_vpp.disp_fb_cnt = 0;
	}
#else
	fb_list = &vpp_disp_fb_list;
	yaddr = vpp_pre_dispfb_y_addr;
	caddr = vpp_pre_dispfb_c_addr;
	vpp_pre_dispfb_y_addr = 0;
	vpp_pre_dispfb_c_addr = 0;
	if( yaddr ) mb_put(yaddr);
	if( caddr ) mb_put(caddr);
	g_vpp.disp_fb_cnt = 0;
#endif

	while( !list_empty(fb_list) ){
		ptr = fb_list->next;
		entry = list_entry(ptr,vpp_dispfb_parm_t,list);
		list_del_init(ptr);
		list_add_tail(&entry->list,&vpp_disp_free_list);
		if( entry->parm.flag & VPP_FLAG_DISPFB_ADDR ){
			yaddr = entry->parm.yaddr;
			caddr = entry->parm.caddr;
		}
		else if(entry->parm.flag & VPP_FLAG_DISPFB_INFO){
			yaddr = entry->parm.info.y_addr;
			caddr = entry->parm.info.c_addr;
		}

		if( yaddr ){
			yaddr = (unsigned int)phys_to_virt(yaddr);
			mb_put(yaddr);
		}

		if( caddr ){
			caddr = (unsigned int)phys_to_virt(caddr);
			mb_put(caddr);
		}
	}
	spin_unlock_irqrestore(&vpp_irqlock, flags);
	return 0;
} /* End of vpp_disp_fb_clr */

/*!*************************************************************************
* vpp_disp_fb_isr()
*
* Private Function by Sam Shen, 2009/02/02
*/
/*!
* \brief	interrupt service for display frame
*
* \retval  status
*/
static int vpp_disp_fb_isr(void)
{
	vpp_mod_t mod;
	vpp_dispfb_parm_t *entry;
	struct list_head *ptr;
	int flags;

	spin_lock_irqsave(&vpp_irqlock, flags);
	ptr = 0;
	if( !list_empty(&vpp_disp_fb_list) ){
		unsigned int yaddr,caddr;

		ptr = vpp_disp_fb_list.next;
		entry = list_entry(ptr,vpp_dispfb_parm_t,list);
		memcpy(&g_vpp.govw_pts,&entry->pts,sizeof(vpp_pts_t));
		mod = (g_vpp.direct_path)? VPP_MOD_GOVRH:VPP_MOD_VPU;
		if( mod == VPP_MOD_VPU ){
#ifdef WMT_FTBLK_VPU
			vpu_set_reg_update(VPP_FLAG_DISABLE);
#else
			scl_set_reg_update(VPP_FLAG_DISABLE);
#endif
			govm_set_reg_update(VPP_FLAG_DISABLE);
		}

		yaddr = caddr = 0;
		if( entry->parm.flag & VPP_FLAG_DISPFB_ADDR ){
			vpp_fb_base_t *mod_fb_p;

			yaddr = entry->parm.yaddr;
			caddr = entry->parm.caddr;
			mod_fb_p = vpp_mod_get_fb_base(mod);
			mod_fb_p->set_addr(yaddr,caddr);
		}

		if( entry->parm.flag & VPP_FLAG_DISPFB_INFO ){
			vpp_fb_base_t *mod_fb_p;

			mod_fb_p = vpp_mod_get_fb_base(mod);
			yaddr = entry->parm.info.y_addr;
			caddr = entry->parm.info.c_addr;
			if( g_vpp.direct_path ){
#ifdef WMT_FTBLK_GOVRH
				vpp_display_format_t field;

				if( entry->parm.info.col_fmt != g_vpp.direct_path_colfmt ){
//					printk("direct path colfmt %s\n",vpp_colfmt_str[entry->parm.info.col_fmt]);
					g_vpp.direct_path_colfmt = entry->parm.info.col_fmt;
					govrh_set_data_format(g_vpp.direct_path_colfmt);
					mod_fb_p->set_csc(mod_fb_p->csc_mode);
				}
				field = (entry->parm.info.flag & VDO_FLAG_INTERLACE)?VPP_DISP_FMT_FIELD:VPP_DISP_FMT_FRAME;
				govrh_set_source_format(field);
				govrh_set_fb_addr(yaddr,caddr);
#endif
			}
			else {
				mod_fb_p->fb = entry->parm.info;
				mod_fb_p->set_framebuf(&mod_fb_p->fb);
			}
		}

		if( entry->parm.flag & VPP_FLAG_DISPFB_VIEW ){
			vpp_set_video_scale(&entry->parm.view);
		}

		if( mod == VPP_MOD_VPU ){
#ifdef WMT_FTBLK_VPU
			vpu_set_reg_update(VPP_FLAG_ENABLE);
#else
			scl_set_reg_update(VPP_FLAG_ENABLE);
#endif
			govm_set_reg_update(VPP_FLAG_ENABLE);
		}

		list_del_init(ptr);
		list_add_tail(&entry->list,&vpp_disp_free_list);
		g_vpp.disp_fb_cnt--;

		if( vpp_pre_dispfb_y_addr ) mb_put(vpp_pre_dispfb_y_addr);
		if( vpp_pre_dispfb_c_addr )	mb_put(vpp_pre_dispfb_c_addr);
		vpp_pre_dispfb_y_addr = (yaddr)? ((unsigned int) phys_to_virt(yaddr)):0;
		vpp_pre_dispfb_c_addr = (caddr)? ((unsigned int) phys_to_virt(caddr)):0;
		vpp_vpu_disp_cnt++;
		vpp_dbg_show(VPP_DBGLVL_DISPFB,2,"show disp fb");
	}

#ifdef CONFIG_VPP_DYNAMIC_DEI
	#define VPP_DEI_CHECK_PERIOD	150

	if( vppif_reg32_read(VPU_DEI_ENABLE) && (p_vpu->dei_mode == VPP_DEI_DYNAMIC) ){
		static int dei_cnt = VPP_DEI_CHECK_PERIOD;
		static unsigned int weave_sum,bob_sum;
		static unsigned int pre_weave_sum,pre_bob_sum;
		unsigned int usum,vsum;
		static vpp_deinterlace_t dei_mode = 0;
		unsigned int weave_diff,bob_diff,cur_diff;

		switch( dei_cnt ){
			case 2:
				if( dei_mode != VPP_DEI_ADAPTIVE_ONE ){
					g_vpp.govw_skip_frame = 1;
				}
				vpu_dei_set_mode(VPP_DEI_ADAPTIVE_ONE);
				break;
			case 1:
				vpu_dei_get_sum(&weave_sum,&usum,&vsum);
				if( dei_mode != VPP_DEI_FIELD ){
					g_vpp.govw_skip_frame = 1;
				}
				vpu_dei_set_mode(VPP_DEI_FIELD);
				break;
			case 0:
				vpu_dei_get_sum(&bob_sum,&usum,&vsum);
				if( (vpp_calculate_diff(bob_sum,pre_bob_sum)<100000)
					&& (vpp_calculate_diff(weave_sum,pre_weave_sum)<100000)){
					dei_mode = VPP_DEI_WEAVE;
				}
				else {
					dei_mode = ( bob_sum > (2*weave_sum) )? VPP_DEI_FIELD:VPP_DEI_ADAPTIVE_ONE;
				}
				bob_diff = vpp_calculate_diff(bob_sum,pre_bob_sum);
				weave_diff = vpp_calculate_diff(weave_sum,pre_weave_sum);
				cur_diff = vpp_calculate_diff(weave_sum,bob_sum);
				pre_bob_sum = bob_sum;
				pre_weave_sum = weave_sum;
				vpu_dei_set_mode(dei_mode);
				dei_cnt = VPP_DEI_CHECK_PERIOD;
				if( vpp_check_dbg_level(VPP_DBGLVL_DEI) ){
					static vpp_deinterlace_t pre_mode = 0;
					printk("[VPP] bob %d,weave %d,diff bob %d,weave %d,cur %d\n",bob_sum,weave_sum,bob_diff,weave_diff,cur_diff);
					if( pre_mode != dei_mode ){
						printk("[VPP] dei mode %d -> %d\n",pre_mode,dei_mode);
						pre_mode = dei_mode;
					}
				}
				break;
			default:
				break;
		}
		dei_cnt--;
	}
#endif

#ifdef WMT_FTBLK_PIP
	if( !list_empty(&vpp_pip_fb_list) ){
		unsigned int yaddr,caddr;

		ptr = vpp_pip_fb_list.next;
		entry = list_entry(ptr,vpp_dispfb_parm_t,list);

		yaddr = caddr = 0;
		if( entry->parm.flag & VPP_FLAG_DISPFB_ADDR ){
			yaddr = entry->parm.yaddr;
			caddr = entry->parm.caddr;
			p_pip->fb_p->set_addr(yaddr,caddr);
		}

		if( entry->parm.flag & VPP_FLAG_DISPFB_INFO ){
			yaddr = entry->parm.info.y_addr;
			caddr = entry->parm.info.c_addr;
			p_pip->fb_p->fb = entry->parm.info;
			p_pip->fb_p->set_framebuf(&p_pip->fb_p->fb);
		}

		if( entry->parm.flag & VPP_FLAG_DISPFB_VIEW ){
			p_pip->fb_p->fn_view(VPP_FLAG_WR,&entry->parm.view);
		}

		list_del_init(ptr);
		list_add_tail(&entry->list,&vpp_disp_free_list);

		if( p_pip->pre_yaddr ) mb_put(p_pip->pre_yaddr);
		if( p_pip->pre_caddr ) mb_put(p_pip->pre_caddr);
		p_pip->pre_yaddr = (yaddr)? ((unsigned int) phys_to_virt(yaddr)):0;
		p_pip->pre_caddr = (caddr)? ((unsigned int) phys_to_virt(caddr)):0;
		vpp_pip_disp_cnt++;
	}
#endif
	spin_unlock_irqrestore(&vpp_irqlock, flags);
	return 0;
} /* End of vpp_disp_fb_isr */

/*!*************************************************************************
* vpp_govw_int_routine()
*
* Private Function by Sam Shen, 2009/01/17
*/
/*!
* \brief	govw interrupt routine
*
* \retval  None
*/
void vpp_govw_int_routine(void)
{
#ifdef CONFIG_VPP_DUAL_BUFFER
	unsigned int govr_y,govr_c;
	unsigned int govw_y,govw_c;

	if( g_vpp.dbg_govw_fb_cnt ){
		g_vpp.dbg_govw_fb_cnt--;
		if( g_vpp.dbg_govw_fb_cnt == 0 ){
			govw_set_tg_enable(VPP_FLAG_DISABLE);
		}
	}

	if( g_vpp.govw_skip_all ){
		return;
	}

	if( g_vpp.govw_skip_frame ){
		g_vpp.govw_skip_frame--;
		vpp_dbg_show(VPP_DBGLVL_DIAG,3,"GOVW skip");
		return;
	}

	govr_y = g_vpp.govr->fb_p->fb.y_addr;
	govr_c = g_vpp.govr->fb_p->fb.c_addr;
	govw_y = p_govw->fb_p->fb.y_addr;
	govw_c = p_govw->fb_p->fb.c_addr;

	g_vpp.govr->fb_p->set_addr(govw_y,govw_c);
	p_govw->fb_p->set_addr(govr_y,govr_c);

	g_vpp.govr->fb_p->fb.y_addr = govw_y;
	g_vpp.govr->fb_p->fb.c_addr = govw_c;
	p_govw->fb_p->fb.y_addr = govr_y;
	p_govw->fb_p->fb.c_addr = govr_c;
	memcpy(&g_vpp.disp_pts,&g_vpp.govw_pts,sizeof(vpp_pts_t));
#endif
} /* End of vpp_govw_int_routine */

/*!*************************************************************************
* vpp_interrupt_routine()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp interrupt routine
*
* \retval  None
*/
static irqreturn_t vpp_interrupt_routine
(
	int irq, 				/*!<; // irq id */
	void *dev_id, 			/*!<; // device id */
	struct pt_regs *regs	/*!<; // reg pointer */
)
{
	vpp_int_t int_sts;
#ifdef PATCH_GE_NOT_READY
	static int ge_not_ready;
	static int ge_not_ready_cnt = 0;
#endif
#ifdef PATCH_GE_NOT_READY_2
	static int ge_not_ready_cnt = 0;
#endif

	switch(irq){
#ifdef WMT_FTBLK_VPU
		case VPP_IRQ_VPU:
			int_sts = p_vpu->get_sts();
			p_vpu->clr_sts(int_sts);
			VPPMSG("VPU isr 0x%x\n",int_sts);
			break;
#endif
		case VPP_IRQ_VPPM:	/* VPP */
			int_sts = p_vppm->get_sts();
			p_vppm->clr_sts(int_sts);
			if( vpp_check_dbg_level(VPP_DBGLVL_INT) ){
				printk("VPP isr 0x%x\n",int_sts);
			}

			if( int_sts & VPP_INT_GOVRH_VBIS ){
#ifdef WMT_FTBLK_GOVRH
//#ifdef PATCH_GOVRH_ASYNC_FIFO
				switch( g_vpp.govrh_async_fifo_patch ){
					default:
					case 1:
						if( g_vpp.govrh_interlace_mode ){
							g_vpp.govrh_field = (g_vpp.govrh_field)? 0:1;
						}
						else {
							g_vpp.govrh_field = 1;
						}
						break;
					case 2:
					case 3:
						g_vpp.govrh_field = (g_vpp.govrh_field)? 0:1;
						break;
				}
//#endif
				if( vpp_dac_sense_enable ){
					if( g_vpp.govrh_field ){
						govrh_monitor_DAC_sense();
					}
				}
#endif
				if( list_empty(&vpp_govrh_vbis_list) == 0 ){
					tasklet_schedule(&vpp_govrh_vbis_tasklet);
				}
			}

			if( int_sts & VPP_INT_GOVRH_PVBI ){
#ifdef PATCH_GOVRH_ASYNC_FIFO
				switch( g_vpp.govrh_async_fifo_patch ){
					case 1:
					case 3:
						if( g_vpp.govrh_async_fifo_cnt ){
							g_vpp.govrh_async_fifo_cnt--;
						}
						else {
							vppif_reg32_write(GOVRH_ACTLN_END,0x0);
						}
						break;
					default:
						break;
				}
#endif
				if( g_vpp.direct_path ){
					vpp_disp_fb_isr();
				}
			}

			if( int_sts & VPP_INT_GOVRH_VBIE ){
#ifdef PATCH_GOVRH_ASYNC_FIFO
				switch( g_vpp.govrh_async_fifo_patch ){
					case 1:
					case 3:
						vppif_reg32_write(GOVRH_ACTLN_END,g_vpp.govrh_async_fifo_reg);
						break;
					default:
						break;
				}
#endif
			}

			if( int_sts & VPP_INT_GOVW_PVBI ){
				vpp_set_dbg_gpio(5,0xFF);
#ifndef CONFIG_GOVW_FBSWAP_VBIE
				vpp_govw_int_routine();
				vpp_disp_fb_isr();
#endif
				if( list_empty(&vpp_govw_pvbi_list) == 0 ){
					tasklet_schedule(&vpp_govw_pvbi_tasklet);
				}
				g_vpp.dbg_govw_pvbi_cnt++;
#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
				vpp_govw_dynamic_tg(0);
#endif
			}

			if( int_sts & VPP_INT_GOVW_VBIS ){
				if( list_empty(&vpp_govw_vbis_list) == 0 ){
					tasklet_schedule(&vpp_govw_vbis_tasklet);
				}
//				vpp_govw_check_tg();
				g_vpp.dbg_govw_vbis_cnt++;
#ifdef PATCH_GE_NOT_READY
				if( ge_not_ready ){
					vppif_reg32_write(GOVM_GE_SOURCE,0x1);
					ge_not_ready = 0;
				}
#endif
			}

			if( int_sts & VPP_INT_GOVW_VBIE ){
#ifdef CONFIG_GOVW_FBSWAP_VBIE
				vpp_govw_int_routine();
				vpp_disp_fb_isr();
#endif
				if( list_empty(&vpp_govw_vbie_list) == 0 ){
					tasklet_schedule(&vpp_govw_vbie_tasklet);
				}
#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
				vpp_govw_dynamic_tg(0);
#endif
				vpp_dbg_show(VPP_DBGLVL_DIAG,3,"GOVW VBIE");
			}

			if( int_sts & VPP_INT_SCL_PVBI ){
				if( list_empty(&vpp_scl_pvbi_list) == 0 ){
					tasklet_schedule(&vpp_scl_pvbi_tasklet);
				}
			}

			if( int_sts & VPP_INT_SCL_VBIS ){
				if( list_empty(&vpp_scl_vbis_list) == 0 ){
					tasklet_schedule(&vpp_scl_vbis_tasklet);
				}
			}

			if( int_sts & VPP_INT_SCL_VBIE ){
				if( list_empty(&vpp_scl_vbie_list) == 0 ){
					tasklet_schedule(&vpp_scl_vbie_tasklet);
				}
			}

			if( int_sts & VPP_INT_VPU_VBIE ){
				if( list_empty(&vpp_vpu_vbie_list) == 0 ){
					tasklet_schedule(&vpp_vpu_vbie_tasklet);
				}
			}

#ifdef WMT_FTBLK_DISP /* sw patch : interlace tv mode field swap */
			if( int_sts & VPP_INT_DISP_VBIS ){
				if( vppif_reg32_read(DISP_INPUT_FIELD) ){
					if( disp_get_cur_field() ){
						vppif_reg32_out(GOVRH_BASE1_ADDR+0xe8,0x2);
					}
					else {
						vppif_reg32_out(GOVRH_BASE1_ADDR+0xe8,0x3);
					}
				}
				if( vpp_dac_sense_enable ){
					disp_DAC_sense();
				}
			}

			if( int_sts & VPP_INT_DISP_VBIE ){
				if( disp_get_cur_field() ){
					if( p_disp->disable_flag ){
						vppif_reg32_write(DISP_EN,0);
						p_disp->disable_flag = 0;
						up(&vpp_disp_vbie_sem);
					}
				}
			}
#endif
			break;
#ifdef WMT_FTBLK_SCL
		case VPP_IRQ_SCL:	/* SCL */
			int_sts = p_scl->get_sts();
			p_scl->clr_sts(int_sts);
			VPPMSG("SCL isr 0x%x\n",int_sts);
			break;
#endif
		case VPP_IRQ_GOVM:	/* GOVM */
			int_sts = p_govm->get_sts();
			p_govm->clr_sts(int_sts);
			VPPMSG("GOVM isr 0x%x\n",int_sts);
			break;
		case VPP_IRQ_GOVW:	/* GOVW */
			int_sts = p_govw->get_sts();
			p_govw->clr_sts(int_sts);
//			VPPMSG("GOVW isr 0x%x\n",int_sts);
			vpp_govw_tg_err_cnt++;
			if( int_sts & VPP_INT_ERR_GOVW_TG ){
#if (WMT_CUR_PID == WMT_PID_8510)	// reset GOVW
				if( vppif_reg32_in(0xd8050dbc) ){
					vppif_reg32_out(0xd8050dbc,0x0);
					vppif_reg32_out(0xd8050dbc,0x1);
				}
#endif
#ifdef PATCH_GE_NOT_READY
				if( vppif_reg32_read(GOVM_INTSTS_GE_READY) == 0 ){
					vppif_reg32_write(GOVM_GE_SOURCE,0x0);
					ge_not_ready = 1;
					ge_not_ready_cnt++;
					printk("[VPP] *W* TG err by GE not ready %d\n",ge_not_ready_cnt);
				}
#endif
#ifdef PATCH_GE_NOT_READY_2
				if( vppif_reg32_read(GOVM_INTSTS_GE_READY) == 0 ){
					int i;
					int back_g1[64],back_g2[64],back_g3[64];

					vppif_reg32_out( GE3_BASE_ADDR + 0xd4, 1);	 		//set GE_AMX read level 2 register

					for(i=0;i<64;i++){
						back_g1[i] = vppif_reg32_in(GE1_BASE_ADDR + i*4);		//back GE_AMX register
						back_g2[i] = vppif_reg32_in(GE2_BASE_ADDR + i*4);		//back GE_AMX register
						back_g3[i] = vppif_reg32_in(GE3_BASE_ADDR + i*4);		//back GE_AMX register
					}

					vppif_reg32_out( VPP_BASE_ADDR+0x10, 0x01000101);		//GE S/W RESET enable
					vppif_reg32_out( VPP_BASE_ADDR+0x10, 0x01010101);		//GE S/W RESET disable

					for(i=0;i<64;i++){
						vppif_reg32_out( GE1_BASE_ADDR + i*4 , back_g1[i]);		//restore GE_AMX register
						vppif_reg32_out( GE2_BASE_ADDR + i*4 , back_g2[i]);		//restore GE_AMX register
						vppif_reg32_out( GE3_BASE_ADDR + i*4 , back_g3[i]);		//restore GE_AMX register
					}

					ge_not_ready_cnt++;
					printk("[VPP] *W* TG err by GE not ready %d\n",ge_not_ready_cnt);
				}
#endif
			}
#ifdef CONFIG_VPP_GOVW_TG_ERR_DROP_FRAME
			if( vpp_disp_fb_cnt(&vpp_disp_fb_list) > 1 ){
				vpp_disp_fb_isr();		// drop display frame when bandwidth not enouth
				vpp_vpu_disp_skip_cnt++;
				vpp_dbg_show(VPP_DBGLVL_DISPFB,0,"skip disp fb");
			}
#endif
#ifdef CONFIG_GOVW_FPS_AUTO_ADJUST
			else {
				vpp_govw_dynamic_tg(1);
			}
#endif
#ifdef VPP_DBG_DIAG_NUM
			vpp_dbg_show(VPP_DBGLVL_DIAG,3,"GOVW Err");
			vpp_dbg_diag_delay = 10;
#endif
			g_vpp.govw_skip_frame = 1;
			break;
#ifdef WMT_FTBLK_GOVRH
		case VPP_IRQ_GOVR:	/* GOVR */
			int_sts = p_govrh->get_sts();
			p_govrh->clr_sts(int_sts);
			VPPMSG("GOVR isr 0x%x\n",int_sts);
#ifdef VPP_DBG_DIAG_NUM
			vpp_dbg_show(VPP_DBGLVL_DIAG,3,"GOVR MIF Err");
			vpp_dbg_diag_delay = 10;
#endif
			break;
#endif
		default:
			VPPMSG("*E* invalid vpp isr\n");
			break;
	}
	return IRQ_HANDLED;
} /* End of vpp_interrupt_routine */

/*!*************************************************************************
* vpp_alloc_framebuffer()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp alloc frame buffer
*
* \retval  None
*/
int vpp_alloc_framebuffer(unsigned int resx,unsigned int resy)
{
	vdo_framebuf_t *fb;
	unsigned int y_size;

	if( g_vpp.mb[0] ){
		if( (g_vpp.govr->fb_p->fb.fb_w == resx) && (g_vpp.govr->fb_p->fb.fb_h == resy) ){
			return 0;
		}
		mb_free((unsigned int) phys_to_virt(g_vpp.mb[0]));
		mb_free((unsigned int) phys_to_virt(g_vpp.mb[1]));
	}

	if( resx % 64 ){
		resx += (64 - (resx % 64));
	}
	y_size = resx * resy;
	g_vpp.mb[0] = (unsigned int) virt_to_phys((void *)mb_allocate(y_size * 4));
#ifdef CONFIG_VPP_DUAL_BUFFER
	g_vpp.mb[1] = (unsigned int) virt_to_phys((void *)mb_allocate(y_size * 4));
#else
	g_vpp.mb[1] = g_vpp.mb[0];
#endif

	fb = &p_govw->fb_p->fb;
	fb->y_addr = g_vpp.mb[0];
	fb->c_addr = g_vpp.mb[0] + y_size;
	fb->y_size = y_size;
	fb->c_size = y_size * 3;
	fb->fb_w = resx;
	fb->fb_h = resy;

	fb = &g_vpp.govr->fb_p->fb;
	fb->y_addr = g_vpp.mb[1];
	fb->c_addr = g_vpp.mb[1] + y_size;
	fb->fb_w = resx;
	fb->fb_h = resy;

//	VPPMSG("alloc frame buf %dx%d, size %d\n",resx,resy,y_size*4);
//	VPPMSG("mb0 0x%x,mb1 0x%x\n",g_vpp.mb[0],g_vpp.mb[1]);
	return 0;
} /* End of vpp_alloc_framebuffer */

/*!*************************************************************************
* vpp_get_sys_parameter()
*
* Private Function by Sam Shen, 2010/01/27
*/
/*!
* \brief	vpp device initialize
*
* \retval  None
*/
void vpp_get_sys_parameter(void)
{
	unsigned char buf[40];
	/*extern int wmt_getsyspara(char *varname, unsigned char *varval, int varlen);

	if( wmt_getsyspara("vpp_direct_path",buf,40) == 0 ){
		sscanf(buf,"%d",&g_vpp.direct_path);
		printk("[VPP] direct path %d\n",g_vpp.direct_path);
	}*/
	g_vpp.direct_path = 1;
} /* End of vpp_get_sys_parameter */

//--> added by howayhuo on 20100623
extern int start_boot_splash(void);
//<-- end add
/*!*************************************************************************
* vpp_dev_init()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp device initialize
*
* \retval  None
*/
int vpp_dev_init(void)
{
	vpp_mod_base_t *mod_p;
	int i;
	unsigned int mod_mask;
//--> added by howayhuo on 20100623
        //start_boot_splash();
//<-- end added
	vpp_get_sys_parameter();

	g_vpp.disp_fb_max = VPP_DISP_FB_NUM;
	g_vpp.disp_fb_cnt = 0;
	g_vpp.govw_tg_dynamic = 1;
	g_vpp.video_quality_mode = 1;
	g_vpp.scale_keep_ratio = 1;

	INIT_LIST_HEAD(&vpp_free_list);
	INIT_LIST_HEAD(&vpp_govrh_vbis_list);
	INIT_LIST_HEAD(&vpp_govw_pvbi_list);
	INIT_LIST_HEAD(&vpp_govw_vbis_list);
	INIT_LIST_HEAD(&vpp_govw_vbie_list);
	INIT_LIST_HEAD(&vpp_scl_pvbi_list);
	INIT_LIST_HEAD(&vpp_scl_vbis_list);
	INIT_LIST_HEAD(&vpp_scl_vbie_list);
	INIT_LIST_HEAD(&vpp_vpu_vbie_list);

	for(i=0;i<VPP_PROC_NUM;i++)
		list_add_tail(&vpp_proc_array[i].list,&vpp_free_list);

	INIT_LIST_HEAD(&vpp_disp_free_list);
	INIT_LIST_HEAD(&vpp_disp_fb_list);
#ifdef WMT_FTBLK_PIP
	INIT_LIST_HEAD(&vpp_pip_fb_list);
#endif

	for(i=0;i<VPP_DISP_FB_MAX;i++)
		list_add_tail(&vpp_disp_fb_array[i].list,&vpp_disp_free_list);

//	p_scl->int_catch = VPP_INT_ERR_SCL_TG | VPP_INT_ERR_SCLR1_MIF | VPP_INT_ERR_SCLR2_MIF
//						| VPP_INT_ERR_SCLW_MIFRGB | VPP_INT_ERR_SCLW_MIFY |	VPP_INT_ERR_SCLW_MIFC;
#ifdef CONFIG_GOVW_FBSWAP_VBIE
	p_vppm->int_catch = VPP_INT_GOVW_VBIS | VPP_INT_GOVW_VBIE;
#else
	p_vppm->int_catch = VPP_INT_GOVW_PVBI | VPP_INT_GOVW_VBIS;
#endif

#ifdef CONFIG_VPP_GOVW_TG_ERR_DROP_FRAME
	p_govw->int_catch = VPP_INT_ERR_GOVW_TG | VPP_INT_ERR_GOVW_MIFY | VPP_INT_ERR_GOVW_MIFC;
#endif
//	p_govm->int_catch = VPP_INT_ERR_GOVM_VPU | VPP_INT_ERR_GOVM_GE;

#ifdef WMT_FTBLK_GOVRH
	g_vpp.govr = (vpp_mod_base_t*) p_govrh;
#elif defined(WMT_FTBLK_LCDC)
	g_vpp.govr = (vpp_mod_base_t*) p_lcdc;
#endif

	vpp_alloc_framebuffer(VPP_HD_MAX_RESX,VPP_HD_MAX_RESY);

	/* check govrh preinit for uboot logo function */
#ifdef WMT_FTBLK_GOVRH
	g_vpp.govrh_preinit = vppif_reg32_read(GOVRH_MIF_ENABLE);
	if( g_vpp.govrh_preinit ){
		p_vppm->int_catch &= ~VPP_INT_GOVW_PVBI;
	}
#else
	g_vpp.govrh_preinit = 0;
#endif
	printk("vpp_init(boot logo %d)\n",g_vpp.govrh_preinit);

	// init video out module first
	if( g_vpp.govrh_preinit == 0 ){
		mod_mask = BIT(VPP_MOD_GOVRS) | BIT(VPP_MOD_GOVRH) | BIT(VPP_MOD_DISP) | BIT(VPP_MOD_LCDC);
		for(i=0;i<VPP_MOD_MAX;i++){
			if( !(mod_mask & (0x01 << i)) ){
				continue;
			}
			mod_p = vpp_mod_get_base(i);
			if( mod_p && mod_p->init ){
				mod_p->init(mod_p);
			}
		}
	}

	// init other module
	mod_mask =  BIT(VPP_MOD_GOVW) | BIT(VPP_MOD_GOVM) | BIT(VPP_MOD_SCL) | BIT(VPP_MOD_SCLW) | BIT(VPP_MOD_VPU) |
				BIT(VPP_MOD_VPUW) | BIT(VPP_MOD_PIP) | BIT(VPP_MOD_VPPM);
	for(i=0;i<VPP_MOD_MAX;i++){
		if( !(mod_mask & (0x01 << i)) ){
			continue;
		}
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->init ){
			mod_p->init(mod_p);
		}
	}

	// init video out device
	vout_init(&vfb_var);
	govm_set_disp_coordinate(vfb_var.xres,vfb_var.yres);
	vpp_set_video_quality(g_vpp.video_quality_mode);

#ifdef CONFIG_VPP_INTERRUPT
	// init interrupt service routine
#ifdef WMT_FTBLK_SCL
	if ( request_irq(VPP_IRQ_SCL, vpp_interrupt_routine, IRQF_DISABLED, "scl", (void *)&g_vpp) ) {
		VPPMSG("*E* request VPP ISR fail\n");
		return -1;
	}
#endif

	if ( request_irq(VPP_IRQ_VPPM, vpp_interrupt_routine, IRQF_DISABLED, "vpp", (void *)&g_vpp) ) {
		VPPMSG("*E* request VPP ISR fail\n");
		return -1;
	}

	if ( request_irq(VPP_IRQ_GOVM, vpp_interrupt_routine, IRQF_DISABLED, "govm", (void *)&g_vpp) ) {
		VPPMSG("*E* request VPP ISR fail\n");
		return -1;
	}

	if ( request_irq(VPP_IRQ_GOVW, vpp_interrupt_routine, IRQF_DISABLED, "govw", (void *)&g_vpp) ) {
		VPPMSG("*E* request VPP ISR fail\n");
		return -1;
	}

#ifdef WMT_FTBLK_GOVRH
	if ( request_irq(VPP_IRQ_GOVR, vpp_interrupt_routine, IRQF_DISABLED, "govr", (void *)&g_vpp) ) {
		VPPMSG("*E* request VPP ISR fail\n");
		return -1;
	}
#endif
#ifdef WMT_FTBLK_VPU
	if ( request_irq(VPP_IRQ_VPU, vpp_interrupt_routine, IRQF_DISABLED, "vpu", (void *)&g_vpp) ) {
		VPPMSG("*E* request VPU ISR fail\n");
		return -1;
	}
#endif
#endif

#ifdef CONFIG_VPP_PROC
	// init system proc
	if( vpp_proc_dir == 0 ){
		vpp_proc_dir = proc_mkdir("driver/vpp", NULL);
		vpp_proc_dir->owner = THIS_MODULE;
		//create_proc_info_entry("sts", 0, vpp_proc_dir, vpp_sts_read_proc);
		//create_proc_info_entry("reg", 0, vpp_proc_dir, vpp_reg_read_proc);
		//create_proc_info_entry("info", 0, vpp_proc_dir, vpp_info_read_proc);

		vpp_table_header = register_sysctl_table(vpp_root_table);
	}
#endif
	return 0;
} /* End of vpp_dev_init */
module_init(vpp_dev_init);

/*!*************************************************************************
* vpp_exit()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp device exit
*
* \retval  None
*/
int vpp_exit(struct fb_info *info)
{
	VPPMSG("vpp_exit\n");

	vout_exit();
	unregister_sysctl_table(vpp_table_header);

	return 0;
} /* End of vpp_exit */

/*!*************************************************************************
* vpp_backup_reg()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp register backup
*
* \retval  None
*/
unsigned int *vpp_backup_reg(unsigned int addr,unsigned int size)
{
	u32 *ptr;
	int i;

	size += 4;
	ptr = (u32*) kmalloc(size,GFP_KERNEL);
	for(i=0;i<size;i+=4){
		ptr[i/4] = REG32_VAL(addr+i);
	}
	return ptr;
} /* End of vpp_backup_reg */

/*!*************************************************************************
* vpp_restore_reg()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp register restore
*
* \retval  None
*/
int vpp_restore_reg(unsigned int addr,unsigned int size,unsigned int *reg_ptr)
{
	int i;

	if( reg_ptr == NULL )
		return 0;

	size += 4;
	for(i=0;i<size;i+=4){
		REG32_VAL(addr+i) = reg_ptr[i/4];
	}
	kfree(reg_ptr);
	reg_ptr = 0;
	return 0;
} /* End of vpp_restore_reg */

static u32 *vpp_ge1_reg_ptr;
static u32 *vpp_ge2_reg_ptr;
static u32 *vpp_ge3_reg_ptr;
/*!*************************************************************************
* vpp_suspend()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp suspend
*
* \retval  None
*/

static int g2_amx_enable_bak;

int	vpp_suspend(int state)
{
	vpp_mod_base_t *mod_p;
	int i;

	printk("vpp_suspend\n");
	g2_amx_enable_bak = REG32_VAL(0xd80506ac);
	
	vout_suspend(VOUT_MODE_ALL,state);

	// disable module
	for(i=0;i<VPP_MOD_MAX;i++){
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->suspend ){
			mod_p->suspend(0);
		}
	}

	// wait
	mdelay(100);

	// disable tg
	for(i=0;i<VPP_MOD_MAX;i++){
		if( i == VPP_MOD_GOVW )
			continue;

		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->suspend ){
			mod_p->suspend(1);
		}
	}

	// backup registers
	for(i=0;i<VPP_MOD_MAX;i++){
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->suspend ){
			mod_p->suspend(2);
		}
	}

	REG32_VAL(0xd80506d4) = 0x1;
	REG32_VAL(0xd80506a8) = 0x0;
	//REG32_VAL(0xd80506ac) = 0x0;

	vpp_ge1_reg_ptr = vpp_backup_reg(GE1_BASE_ADDR+0x00,0xFC);			/* 0x00 - 0xF4 */
	vpp_ge2_reg_ptr = vpp_backup_reg(GE2_BASE_ADDR+0x00,0xFC);			/* 0x00 - 0xF4 */
	vpp_ge3_reg_ptr = vpp_backup_reg(GE3_BASE_ADDR+0x00,0xFC);			/* 0x00 - 0xF4 */

	mdelay(100);
	p_govw->suspend(1);
	return 0;
} /* End of vpp_suspend */

/*!*************************************************************************
* vpp_resume()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp driver resume
*
* \retval  None
*/
int	vpp_resume(void)
{
	vpp_mod_base_t *mod_p;
	int i;

	printk("vpp_resume\n");

	// restore registers
	for(i=0;i<VPP_MOD_MAX;i++){
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->resume ){
			mod_p->resume(0);
		}
	}
	vpp_restore_reg(GE1_BASE_ADDR+0x00,0xFC,vpp_ge1_reg_ptr);			/* 0x00 - 0xFC */
	vpp_restore_reg(GE2_BASE_ADDR+0x00,0xFC,vpp_ge2_reg_ptr);			/* 0x00 - 0xFC */
	vpp_restore_reg(GE3_BASE_ADDR+0x00,0xFC,vpp_ge3_reg_ptr);			/* 0x00 - 0xFC */
	REG32_VAL(0xd80506d4) = 0x0;

	REG32_VAL(0xd80506ac) = g2_amx_enable_bak;

	// enable tg
	for(i=0;i<VPP_MOD_MAX;i++){
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->resume ){
			mod_p->resume(1);
		}
	}

	// wait
	mdelay(100);

	// enable module
	for(i=0;i<VPP_MOD_MAX;i++){
		mod_p = vpp_mod_get_base(i);
		if( mod_p && mod_p->resume ){
			mod_p->resume(2);
		}
	}

	vout_resume(VOUT_MODE_ALL,0);
	return 0;
} /* End of vpp_resume */

/*!*************************************************************************
* vpp_check_mmap_offset()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	check mmap offset
*
* \retval  None
*/
int vpp_check_mmap_offset(dma_addr_t offset)
{
	vdo_framebuf_t *fb;
	int i;

//	VPPMSG("vpp_check_mmap_offset 0x%x\r\n",offset);

	for(i=0;i<VPP_MOD_MAX;i++){
		fb = vpp_mod_get_framebuf(i);
		if( fb ){
			if( (offset >= fb->y_addr) && (offset < (fb->y_addr + fb->y_size))){
//				VPPMSG("mmap to mod %d Y frame buffer\r\n",i);
				return 0;
			}

			if( (offset >= fb->c_addr) && (offset < (fb->c_addr + fb->c_size))){
//				VPPMSG("mmap to mod %d C frame buffer\r\n",i);
				return 0;
			}
		}
	}
	return -1;
} /* End of vpp_check_mmap_offset */

/*!*************************************************************************
* vpp_mmap()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp driver mmap
*
* \retval  None
*/
int vpp_mmap(struct vm_area_struct *vma)
{
//	VPPMSG("vpp_mmap\n");

	/* which buffer need to remap */
	if( vpp_check_mmap_offset(vma->vm_pgoff << PAGE_SHIFT) != 0 ){
		printk("*E* vpp_mmap 0x%x\n",(int) vma->vm_pgoff << PAGE_SHIFT);
		return -EINVAL;
	}

//	VPPMSG("Enter vpp_mmap remap 0x%x\n",(int) (vma->vm_pgoff << PAGE_SHIFT));

	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
} /* End of vpp_mmap */

/*!*************************************************************************
* vpp_wait_vsync()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	wait govw frame ready
*
* \retval  None
*/
void vpp_wait_vsync(void)
{
	if( g_vpp.direct_path ){
		vpp_proc_func(0,0,VPP_INT_GOVRH_VBIS,1);
	}
	else {
		vpp_proc_func(0,0,VPP_INT_GOVW_VBIS,1);
	}
	return;
} /* End of vpp_wait_vsync */

/*!*************************************************************************
* vpp_config()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp config for resolution change
*
* \retval  None
*/
int vpp_config(struct fb_info *info)
{
	vout_info_t vo_info;
	vdo_framebuf_t *fb;

#if(WMT_SUB_PID == WMT_PID_8505)
	if( (info->var.xres > 1024) || (info->var.yres > 600) ){
		printk("*E* WM8505 not support (%dx%d)\n",info->var.xres,info->var.yres);
		return -1;
	}
#endif

	g_vpp.resx = info->var.xres;
	g_vpp.resy = info->var.yres;
	vo_info.resx = info->var.xres;
	vo_info.resy = info->var.yres;
	vo_info.pixclock = info->var.pixclock;
	vo_info.bpp = info->var.bits_per_pixel;
	vo_info.fps = info->var.pixclock / (info->var.xres * info->var.yres);
	if( info->var.pixclock != (info->var.xres * info->var.yres * vo_info.fps ) ){
		vo_info.fps = VPP_VOUT_FRAMERATE_DEFAULT;
	}
	if( vo_info.fps == 0 ){
		vo_info.fps = VPP_VOUT_FRAMERATE_DEFAULT;
	}
	VPPMSG("vpp_config(%dx%d@%d),pixclock %d\n",vo_info.resx,vo_info.resy,vo_info.fps,info->var.pixclock);
	vout_config(VOUT_MODE_ALL,&vo_info);
	if( (vo_info.resx != info->var.xres) || (vo_info.resy != info->var.yres) ){
		VPPMSG("vout mode update (%dx%d)\n",vo_info.resx,vo_info.resy);
		info->var.xres = vo_info.resx;
		info->var.yres = vo_info.resy;
	}
	vpp_alloc_framebuffer(vo_info.resx,vo_info.resy);

	g_vpp.govr->fb_p->fb.img_w = info->var.xres;
	g_vpp.govr->fb_p->fb.img_h = info->var.yres;
	g_vpp.govr->fb_p->framerate = vo_info.fps;
	if ( g_vpp.govrh_preinit ){
		g_vpp.govr->fb_p->fb.fb_w = info->var.xres;
		g_vpp.govr->fb_p->fb.fb_h = info->var.yres;
		p_govw->fb_p->fb.fb_w = info->var.xres;
		p_govw->fb_p->fb.fb_h = info->var.yres;
	}
	else {
		g_vpp.govr->fb_p->set_framebuf(&g_vpp.govr->fb_p->fb);
	}
#ifdef WMT_FTBLK_GOVRH
	p_govrh->vga_dac_sense_cnt = vo_info.fps * VPP_DAC_SENSE_SECOND;
#endif
	p_govw->fb_p->fb.img_w = info->var.xres;
	p_govw->fb_p->fb.img_h = info->var.yres;
	p_govw->fb_p->set_framebuf(&p_govw->fb_p->fb);

	if( g_vpp.direct_path ){
		govw_set_tg_enable(VPP_FLAG_DISABLE);
	}

	fb = &p_vpu->fb_p->fb;
	if( fb->y_addr ){
		p_vpu->resx_visual = (fb->fb_w > info->var.xres)? info->var.xres:fb->fb_w;
		p_vpu->resy_visual = (fb->fb_h > info->var.yres)? info->var.yres:fb->fb_h;
	}
	else {
		fb->fb_w = fb->img_w = p_vpu->resx_visual = info->var.xres;
		fb->fb_h = fb->img_h = p_vpu->resy_visual = info->var.yres;
	}
	p_vpu->fb_p->set_framebuf(fb);
	govm_set_vpu_coordinate(p_vpu->posx,p_vpu->posy,p_vpu->posx+p_vpu->resx_visual_scale-1,p_vpu->posy+p_vpu->resy_visual_scale-1);

	if( g_vpp.govrh_preinit ){
		REG32_VAL(0xd805051c) = 0x132;
		REG32_VAL(0xd805055c) = 0x259;
		REG32_VAL(0xd8050560) = 0x75;
		REG32_VAL(0xd8050564) = 0x1f53;
		REG32_VAL(0xd8050568) = 0x1ead;
		REG32_VAL(0xd805056c) = 0x200;
		REG32_VAL(0xd8050570) = 0x200;
		REG32_VAL(0xd8050574) = 0x1e53;
		REG32_VAL(0xd80505f0) = 0x1fad;

		REG32_VAL(0xd80505f4) = 0x0;
		REG32_VAL(0xd80505f8) = 0x100;
		REG32_VAL(0xd80505fc) = 0x100;

		vpp_wait_vsync();
		vppm_set_int_enable(VPP_FLAG_ENABLE,VPP_INT_GOVW_PVBI);
		g_vpp.govrh_preinit = 0;
	}

#ifdef WMT_FTBLK_GOVRH
	govrh_set_tg_enable(VPP_FLAG_ENABLE);
#endif
#if 0
	if( g_vpp.vo_enable ){
		vpp_wait_vsync();
		vpp_wait_vsync();
		vout_enable(VOUT_MODE_ALL,1);
		g_vpp.vo_enable = 0;
	}
#endif

	return 0;
} /* End of vpp_config */

/*!*************************************************************************
* vpp_i2c_write()
*
* Private Function by Sam Shen, 2009/02/26
*/
/*!
* \brief	write i2c
*
* \retval  None
*/
int vpp_i2c_write(unsigned int addr,unsigned int index,char *pdata,int len)
{
    struct i2c_msg msg[1];
	unsigned char buf[len+1];

	addr = (addr >> 1);
    buf[0] = index;
	memcpy(&buf[1],pdata,len);
    msg[0].addr = addr;
    msg[0].flags = 0 ;
    msg[0].flags &= ~(I2C_M_RD);
    msg[0].len = len+1;
    msg[0].buf = buf;
#ifdef CONFIG_I2C
    wmt_i2c_xfer_if(msg);
#endif

#ifdef DEBUG
{
	int i;

	VPPMSG("vpp_i2c_write(addr 0x%x,index 0x%x,len %d\n",addr,index,len);
	for(i=0;i<len;i+=8){
		VPPMSG("%d : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",i,
			pdata[i],pdata[i+1],pdata[i+2],pdata[i+3],pdata[i+4],pdata[i+5],pdata[i+6],pdata[i+7]);
	}
}
#endif
    return 0;
} /* End of vpp_i2c_write */

/*!*************************************************************************
* vpp_i2c_read()
*
* Private Function by Sam Shen, 2009/02/26
*/
/*!
* \brief	read i2c
*
* \retval  None
*/
int vpp_i2c_read(unsigned int addr,unsigned int index,char *pdata,int len)
{
	struct i2c_msg msg[2];
	unsigned char buf[len+1];

	addr = (addr >> 1);
	memset(buf,0x55,len+1);
    buf[0] = index;
	buf[1] = 0x0;

    msg[0].addr = addr;
    msg[0].flags = 0 ;
	msg[0].flags &= ~(I2C_M_RD);
	msg[0].len = 1;
    msg[0].buf = buf;

	msg[1].addr = addr;
	msg[1].flags = 0 ;
	msg[1].flags |= (I2C_M_RD);
	msg[1].len = len;
	msg[1].buf = buf;

#ifdef CONFIG_I2C
	wmt_i2c_xfer_continue_if(msg, 2);
#endif
	memcpy(pdata,buf,len);
#ifdef DEBUG
{
	int i;

	VPPMSG("vpp_i2c_read(addr 0x%x,index 0x%x,len %d\n",addr,index,len);
	for(i=0;i<len;i+=8){
		VPPMSG("%d : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",i,
			pdata[i],pdata[i+1],pdata[i+2],pdata[i+3],pdata[i+4],pdata[i+5],pdata[i+6],pdata[i+7]);
	}
}
#endif
    return 0;
} /* End of vpp_i2c_read */

/*!*************************************************************************
* vpp_i2c0_read()
*
* Private Function by Sam Shen, 2009/02/26
*/
/*!
* \brief	read i2c bus0
*
* \retval  None
*/
int vpp_i2c0_read(unsigned int addr,unsigned int index,char *pdata,int len)
{
	struct i2c_msg msg[2];
	unsigned char buf[len+1];

	addr = (addr >> 1);
	memset(buf,0x55,len+1);
    buf[0] = index;
	buf[1] = 0x0;

    msg[0].addr = addr;
    msg[0].flags = 0 ;
	msg[0].flags &= ~(I2C_M_RD);
	msg[0].len = 1;
    msg[0].buf = buf;

	msg[1].addr = addr;
	msg[1].flags = 0 ;
	msg[1].flags |= (I2C_M_RD);
	msg[1].len = len;
	msg[1].buf = buf;

#ifdef CONFIG_I2C1_WMT
	wmt_i2c_xfer_continue_if_3(msg, 2, 1);
#endif
	memcpy(pdata,buf,len);
	return 0;
}

/*!*************************************************************************
* vpp_direct_path_switch()
*
* Private Function by Sam Shen, 2009/04/13
*/
/*!
* \brief	direct path switch proc function
*
* \retval  None
*/
static void vpp_direct_path_switch(int enable)
{
	if( enable ){

	}
	else {
		g_vpp.govr->fb_p->set_color_fmt(g_vpp.govr->fb_p->fb.col_fmt);
		g_vpp.govr->fb_p->set_csc(g_vpp.govr->fb_p->csc_mode);
	}
} /* End of vpp_direct_path_switch */

/*!*************************************************************************
* vpp_common_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp driver common ioctl
*
* \retval  None
*/
int vpp_common_ioctl(unsigned int cmd,unsigned long arg)
{
	vpp_mod_base_t *mod_p;
	vpp_fb_base_t *mod_fb_p;
	int retval = 0;

	switch(cmd){
		case VPPIO_VPPGET_INFO:
			{
			vpp_cap_t parm;
			int i;

			parm.chip_id = vpp_get_pid();
			parm.version = 0x01;
			parm.resx_max = VPP_HD_MAX_RESX;
			parm.resy_max = VPP_HD_MAX_RESY;
			parm.pixel_clk = 400000000;
			parm.module = 0x0;
			for(i=0;i<VPP_MOD_MAX;i++){
				mod_p = vpp_mod_get_base(i);
				if( mod_p ){
					parm.module |= (0x01 << i);
				}
			}
			parm.option = 0x0;
			copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_cap_t));
			}
			break;
		case VPPIO_VPPSET_INFO:
			{
			vpp_cap_t parm;

			copy_from_user((void *)&parm,(const void *)arg,sizeof(vpp_cap_t));
			}
			break;

		case VPPIO_I2CSET_BYTE:
			{
			vpp_i2c_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_i2c_t));
			vpp_i2c_write(parm.addr,parm.index,(char *)&parm.val,1);
			}
			break;
		case VPPIO_I2CGET_BYTE:
			{
			vpp_i2c_t parm;
			unsigned char buf[8];

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_i2c_t));
			vpp_i2c_read(parm.addr,parm.index,buf,1);
			parm.val = buf[0];
			copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_i2c_t));
			}
			break;
		case VPPIO_VPPSET_DIRECTPATH:
			printk("ioctl VPPIO_VPPSET_DIRECTPATH!!!!!!!!!!\n");
			g_vpp.direct_path = arg;
			g_vpp.direct_path_colfmt = g_vpp.govr->fb_p->fb.col_fmt;
			vpp_disp_fb_clr(0);
			if( g_vpp.direct_path ){
				govw_set_tg_enable(VPP_FLAG_DISABLE);
				vppm_set_int_enable(VPP_FLAG_ENABLE,VPP_INT_GOVRH_PVBI);
			}
			else {
				vppm_set_int_enable(VPP_FLAG_DISABLE,VPP_INT_GOVRH_PVBI);
				govw_set_tg_enable(VPP_FLAG_ENABLE);
				vpp_proc_func((void *)vpp_direct_path_switch,0,VPP_INT_GOVW_PVBI,1);
			}
			printk("ioctl VPPIO_VPPSET_DIRECTPATH end!!!!!!!!!!\n");
			break;
		case VPPIO_VPPSET_FBDISP:
			{
				vpp_dispfb_t parm;

				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_dispfb_t));

				if( vpp_check_dbg_level(VPP_DBGLVL_IOCTL) ){
					printk("[VPP] set fbdisp, flag 0x%x\n",parm.flag);
				}

				retval = vpp_disp_fb_add(&parm);
				if( retval ){
					vpp_dbg_show(VPP_DBGLVL_DISPFB,1,"add disp fb full");
				}
				else {
					vpp_set_dbg_gpio(4,0xFF);
				}
			}
			break;
		case VPPIO_WAIT_FRAME:
			{
				int i;
				for(i=0;i<arg;i++){
					vpp_wait_vsync();
				}
			}
			break;
		case VPPIO_MODULE_FRAMERATE:
			{
				vpp_mod_arg_t parm;

				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_mod_arg_t));
				mod_fb_p = vpp_mod_get_fb_base(parm.mod);
				if( parm.read ){
					parm.arg1 = mod_fb_p->framerate;
					copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_mod_arg_t));
				}
				else {
					mod_fb_p->framerate = parm.arg1;
					if( parm.mod == VPP_MOD_GOVW ){
						mod_fb_p->set_framebuf(&mod_fb_p->fb);
					}
				}
			}
			break;
		case VPPIO_MODULE_ENABLE:
			{
				vpp_mod_arg_t parm;

				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_mod_arg_t));
				mod_p = vpp_mod_get_base(parm.mod);
				if( parm.read ){

				}
				else {
					mod_p->set_enable(parm.arg1);
				}
			}
			break;
		case VPPIO_MODULE_TIMING:
			{
				vpp_mod_timing_t parm;
				vpp_clock_t clock;

				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_mod_timing_t));
				mod_p = vpp_mod_get_base(parm.mod);
				if( parm.read ){
					mod_p->get_tg(&clock);
					vpp_trans_timing(parm.mod,&parm.tmr,&clock,0);
					copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_mod_timing_t));
				}
				else {
					vpp_alloc_framebuffer(parm.tmr.hpixel,parm.tmr.vpixel);
					vpp_mod_set_timing(parm.mod,&parm.tmr);
//					vpp_trans_timing(parm.mod,&parm.tmr,&clock,1);
//					mod_p->set_tg(&clock);
				}
			}
			break;
		case VPPIO_MODULE_FBADDR:
			{
				vpp_mod_arg_t parm;

				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_mod_arg_t));
				mod_fb_p = vpp_mod_get_fb_base(parm.mod);
				if( parm.read ){
					mod_fb_p->get_addr(&parm.arg1,&parm.arg2);
					copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_mod_arg_t));
				}
				else {
					mod_fb_p->set_addr(parm.arg1,parm.arg2);
				}
			}
			break;
		case VPPIO_MODULE_FBINFO:
			{
				vpp_mod_fbinfo_t parm;

				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_mod_fbinfo_t));
				mod_fb_p = vpp_mod_get_fb_base(parm.mod);
				if( parm.read ){
					parm.fb = mod_fb_p->fb;
					copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_mod_fbinfo_t));
				}
				else {
					mod_fb_p->fb = parm.fb;
					mod_fb_p->set_framebuf(&parm.fb);
				}
			}
			break;
		case VPPIO_MODULE_VIEW:
			{
				vpp_mod_view_t parm;
				copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_mod_view_t));
				mod_fb_p = vpp_mod_get_fb_base(parm.mod);
				if( parm.read ){
					mod_fb_p->fn_view(VPP_FLAG_RD,&parm.view);
					copy_to_user( (void *)arg, (void *) &parm, sizeof(vpp_mod_view_t));
				}
				else {
					mod_fb_p->fn_view(0,&parm.view);
				}
			}
			break;
		case VPPIO_VPPGET_PTS:
			copy_to_user( (void *)arg, (void *) &g_vpp.disp_pts, sizeof(vpp_pts_t));
			break;
		case VPPIO_VPPSET_PTS:
			copy_from_user( (void *) &g_vpp.frame_pts, (const void *)arg, sizeof(vpp_pts_t));
			{
				int i;
				for(i=0;i<sizeof(vpp_pts_t);i++){
					if( g_vpp.frame_pts.pts[i] )
						break;
				}
				if( i == sizeof(vpp_pts_t )){
					memset(&g_vpp.govw_pts,0x0,sizeof(vpp_pts_t));
					memset(&g_vpp.disp_pts,0x0,sizeof(vpp_pts_t));
				}
			}
			break;
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of vpp_common_ioctl */

/*!*************************************************************************
* vout_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	video out ioctl
*
* \retval  None
*/
int vout_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;

//	VPPMSG("vout_ioctl\n");

	switch(cmd){
		case VPPIO_VOGET_INFO:
			{
			vpp_vout_info_t parm;
			vout_t *vo;
			int num;

//			VPPMSG("VPPIO_VOGET_INFO\n");

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_vout_info_t));
			num = parm.num;
			if( num >= VOUT_MODE_MAX ){
				retval = -ENOTTY;
				break;
			}
			memset(&parm,0,sizeof(vpp_vout_info_t));
			vo = vout_get_info(num);
			if( vo ){
				parm.num = num;
				if( vo->active ) parm.option |= VPP_VOUT_OPT_ACTIVE;
				if( vo->enable ) parm.option |= VPP_VOUT_OPT_ENABLE;
				strncpy(parm.name,vo->name,10);
			}
			copy_to_user( (void *)arg, (const void *) &parm, sizeof(vpp_vout_info_t));

			}
			break;
		case VPPIO_VOSET_MODE:
			{
			vpp_vout_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_vout_parm_t));
			retval = vout_set_mode(parm.num, parm.arg);
			}
			break;
		case VPPIO_VOSET_BLANK:
			{
			vpp_vout_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_vout_parm_t));
			retval = vout_control(parm.num,VOCTL_VISIBLE,!parm.arg);
			}
			break;
#ifdef WMT_FTBLK_GOVRH
		case VPPIO_VOSET_DACSENSE:
			{
			vpp_vout_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_vout_parm_t));
			if( parm.num == VOUT_VGA ){
				/* TODO: dac sense timer */
				if( parm.arg == 0 ){
					govrh_set_DAC_pwrdn(VPP_FLAG_DISABLE);
				}
			}
			}
			break;
		case VPPIO_VOSET_BRIGHTNESS:
			{
			vpp_vout_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_vout_parm_t));
			govrh_set_brightness(parm.arg);
			}
			break;
		case VPPIO_VOGET_BRIGHTNESS:
			{
			vpp_vout_parm_t parm;

			parm.num = 0;
			parm.arg = govrh_get_brightness();
			copy_to_user((void *)arg,(void *)&parm, sizeof(vpp_vout_parm_t));
			}
			break;
		case VPPIO_VOSET_CONTRAST:
			{
			vpp_vout_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_vout_parm_t));
			govrh_set_contrast(parm.arg);
			}
			break;
		case VPPIO_VOGET_CONTRAST:
			{
			vpp_vout_parm_t parm;

			parm.num = 0;
			parm.arg = govrh_get_contrast();
			copy_to_user((void *)arg,(void *) &parm, sizeof(vpp_vout_parm_t));
			}
			break;
#endif
		case VPPIO_VOSET_OPTION:
			{
			vpp_vout_option_t option;
			vout_t *vo;
			int num;

			copy_from_user( (void *) &option, (const void *)arg, sizeof(vpp_vout_option_t));
			num = option.num;
			if( num >= VOUT_MODE_MAX ){
				retval = -ENOTTY;
				break;
			}
			vo = vout_get_info(num);
			if( vo ){
				vo->option[0] = option.option[0];
				vo->option[1] = option.option[1];
				vo->option[2] = option.option[2];
				vout_set_mode(num,vo->active);
			}
			}
			break;
		case VPPIO_VOGET_OPTION:
			{
			vpp_vout_option_t option;
			vout_t *vo;
			int num;

			copy_from_user( (void *) &option, (const void *)arg, sizeof(vpp_vout_option_t));
			num = option.num;
			if( num >= VOUT_MODE_MAX ){
				retval = -ENOTTY;
				break;
			}
			memset(&option,0,sizeof(vpp_vout_info_t));
			vo = vout_get_info(num);
			if( vo ){
				option.num = num;
				option.option[0] = vo->option[0];
				option.option[1] = vo->option[1];
				option.option[2] = vo->option[2];
			}
			copy_to_user( (void *)arg, (const void *) &option, sizeof(vpp_vout_option_t));
			}
			break;
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of vout_ioctl */

/*!*************************************************************************
* govr_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	govr module ioctl
*
* \retval  None
*/
int govr_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;

 	switch(cmd){
#ifdef WMT_FTBLK_GOVRH
		case VPPIO_GOVRSET_DVO:
			{
			vdo_dvo_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vdo_dvo_parm_t));
			govrh_set_dvo_enable(parm.enable);
			govrh_set_dvo_color_format(parm.color_fmt);
			govrh_set_dvo_clock_delay(parm.clk_inv,parm.clk_delay);
			govrh_set_dvo_outdatw(parm.data_w);
			govrh_set_dvo_sync_polar(parm.sync_polar,parm.vsync_polar);
			p_govrh->fb_p->set_csc(p_govrh->fb_p->csc_mode);
			}
			break;
#endif
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of govr_ioctl */

/*!*************************************************************************
* govw_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	govw module ioctl
*
* \retval  None
*/
int govw_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;

	switch(cmd){
		case VPPIO_GOVW_ENABLE:
			if( arg ){
				govw_set_tg_enable(VPP_FLAG_ENABLE);
			}
			else {
				vpp_proc_func((void *)govw_set_tg_enable,0,VPP_INT_GOVW_PVBI,1);
			}
			break;
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of govw_ioctl */

/*!*************************************************************************
* govm_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	govm module ioctl
*
* \retval  None
*/
int govm_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;

	switch(cmd){
		case VPPIO_GOVMSET_SRCPATH:
			{
			vpp_src_path_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_src_path_t));
			govm_set_in_path(parm.src_path,parm.enable);
			}
			break;
		case VPPIO_GOVMGET_SRCPATH:
			retval = govm_get_in_path();
			break;
		case VPPIO_GOVMSET_ALPHA:
			{
			vpp_alpha_parm_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_alpha_parm_t));
			govm_set_alpha_mode(parm.enable,parm.mode,parm.A,parm.B);
			}
			break;
		case VPPIO_GOVMSET_GAMMA:
			govm_set_gamma_mode(arg);
			break;
		case VPPIO_GOVMSET_CLAMPING:
			govm_set_clamping_enable(arg);
			break;
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of govm_ioctl */

/*!*************************************************************************
* vpu_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpu module ioctl
*
* \retval  None
*/
int vpu_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;

	switch(cmd){
		case VPPIO_VPUSET_VIEW:
			{
			vdo_view_t view;

			copy_from_user( (void *) &view, (const void *)arg, sizeof(vdo_view_t));
			p_vpu->fb_p->fn_view(0,&view);
			if( vpp_check_dbg_level(VPP_DBGLVL_SCALE) ){
				printk("[VPP] set view\n");
			}
			}
			break;
		case VPPIO_VPUGET_VIEW:
			{
			vdo_view_t view;

			p_vpu->fb_p->fn_view(1,&view);
			copy_to_user( (void *)arg, (void *) &view, sizeof(vdo_view_t));
			}
			break;
		case VPPIO_VPUSET_FBDISP:
			{
			vpp_dispfb_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_dispfb_t));
			retval = vpp_disp_fb_add(&parm);
			}
			break;
		case VPPIO_VPU_CLR_FBDISP:
			retval = vpp_disp_fb_clr(0);
			break;
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of vpu_ioctl */

/*!*************************************************************************
* scl_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	scl module ioctl
*
* \retval  None
*/
int scl_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;

	switch(cmd){
		case VPPIO_SCL_SCALE:
			{
			vpp_scale_t parm;

			copy_from_user( (void *) &parm, (const void *)arg, sizeof(vpp_scale_t));
			retval = vpp_set_recursive_scale(&parm.src_fb,&parm.dst_fb);
			copy_to_user( (void *) arg, (void *) &parm, sizeof(vpp_scale_t));
			}
			break;
#ifdef WMT_FTBLK_SCL
		case VPPIO_SCL_DROP_LINE_ENABLE:
			scl_set_drop_line(arg);
			break;
#endif
		default:
			retval = -ENOTTY;
			break;
	}
	return retval;
} /* End of scl_ioctl */

/*!*************************************************************************
* vpp_ioctl()
*
* Private Function by Sam Shen, 2009/01/06
*/
/*!
* \brief	vpp driver ioctl
*
* \retval  None
*/
int vpp_ioctl(unsigned int cmd,unsigned long arg)
{
	int retval = 0;
	int err = 0;

//	VPPMSG("vpp_ioctl\n");

	switch( _IOC_TYPE(cmd) ){
		case VPPIO_MAGIC:
			break;
		default:
			return -ENOTTY;
	}

	/* check argument area */
	if( _IOC_DIR(cmd) & _IOC_READ )
		err = !access_ok( VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd));
	else if ( _IOC_DIR(cmd) & _IOC_WRITE )
		err = !access_ok( VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd));

	if( err ) return -EFAULT;

	if( vpp_check_dbg_level(VPP_DBGLVL_IOCTL) ){
		printk("[VPP] ioctl cmd 0x%x,arg 0x%x\n",_IOC_NR(cmd),(int)arg);
	}

	switch(_IOC_NR(cmd)){
		case VPPIO_VPP_BASE ... (VPPIO_VOUT_BASE-1):
//			VPPMSG("VPP command ioctl\n");
			retval = vpp_common_ioctl(cmd,arg);
			break;
		case VPPIO_VOUT_BASE ... (VPPIO_GOVR_BASE-1):
//			VPPMSG("VOUT ioctl\n");
			retval = vout_ioctl(cmd,arg);
			break;
		case VPPIO_GOVR_BASE ... (VPPIO_GOVW_BASE-1):
//			VPPMSG("GOVR ioctl\n");
			retval = govr_ioctl(cmd,arg);
			break;
		case VPPIO_GOVW_BASE ... (VPPIO_GOVM_BASE-1):
//			VPPMSG("GOVW ioctl\n");
			retval = govw_ioctl(cmd,arg);
			break;
		case VPPIO_GOVM_BASE ... (VPPIO_VPU_BASE-1):
//			VPPMSG("GOVM ioctl\n");
			retval = govm_ioctl(cmd,arg);
			break;
		case VPPIO_VPU_BASE ... (VPPIO_SCL_BASE-1):
//			VPPMSG("VPU ioctl\n");
			retval = vpu_ioctl(cmd,arg);
			break;
		case VPPIO_SCL_BASE ... (VPPIO_MAX-1):
//			VPPMSG("SCL ioctl\n");
			retval = scl_ioctl(cmd,arg);
			break;
		default:
			retval = -ENOTTY;
			break;
	}

	return retval;
} /* End of vpp_ioctl */

