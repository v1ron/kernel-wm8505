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
#define VOUT_C

/*=== vout.c =============================================================
*
* MODULE       : vout.c
* AUTHOR       : Sam Shen
* DATE         : 2009/2/6
*-----------------------------------------------------------------------------*/

/*--------------------- History -----------------------------------------------* 
Version 0.01 , Sam Shen, 2009/2/6
	First version

------------------------------------------------------------------------------*/
/*----------------------- DEPENDENCE -----------------------------------------*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include "vout.h"

/*----------------------- PRIVATE MACRO --------------------------------------*/
/* #define  VO_XXXX  xxxx    *//*Example*/

// #define DEBUG

#ifdef __KERNEL__
#define DPRINT		printk
#else
#define DPRINT		printf
#endif

#ifdef DEBUG
#define DBG(fmt, args...)   DPRINT("[VO] %s: " fmt, __FUNCTION__ , ## args)
#define DBG_DETAIL(fmt, args...)   DPRINT("[VO] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBG(fmt, args...)
#define DBG_DETAIL(fmt, args...)
#endif

/*----------------------- PRIVATE CONSTANTS ----------------------------------*/
/* #define VO_XXXX    1     *//*Example*/

/*----------------------- PRIVATE TYPE  --------------------------------------*/
/* typedef  xxxx vo_xxx_t; *//*Example*/

/*----------EXPORTED PRIVATE VARIABLES are defined in vout.h  -------------*/
/*----------------------- INTERNAL PRIVATE VARIABLES - -----------------------*/
/* int  vo_xxx;        *//*Example*/
vout_t *vout_array[VOUT_MODE_MAX];

/*--------------------- INTERNAL PRIVATE FUNCTIONS ---------------------------*/
/* void vo_xxx(void); *//*Example*/

/*----------------------- Function Body --------------------------------------*/
/*!*************************************************************************
* vout_op()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_op(vout_t *vo,int cmd,int arg)
{
	int *addr;
	int (*ioctl_proc)(int arg);
	
	DBG(KERN_ALERT "vout_op(%d,0x%x)\n",cmd,arg);

	addr = (void *) vo->ops;
	addr += cmd;
	ioctl_proc = (void *) *addr;
	if( !ioctl_proc ){
		return 0;
	}
	return ioctl_proc(arg);
} /* End of vout_op */

/*!*************************************************************************
* vout_control()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_control(vout_mode_t mode,int cmd,int arg)
{
	vout_t *vo;
	int ret = 0;
	int i;

	if( mode > VOUT_MODE_MAX ){
		printk(KERN_ALERT "*E* vout mode invalid %d\n",mode);
		return -1;
	}

	if( mode == VOUT_MODE_ALL ){
		for(i=0;i<VOUT_MODE_MAX;i++){
			if( !(vo = vout_array[i]) )
				continue;
				
			if( vo->active ){
				vout_op(vo,cmd,arg);
			}
		}
	}
	else {
		if( (vo = vout_array[mode]) ){
			if( vo->active ){
				ret = vout_op(vo,cmd,arg);
			}
		}
	}
	return ret;
} /* End of vout_control */

/*!*************************************************************************
* vout_enable()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_enable(vout_mode_t mode,int on)
{
	return vout_control(mode,VOCTL_VISIBLE,on);
} /* End of vout_enable */

/*!*************************************************************************
* vout_set_mode()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_set_mode(vout_mode_t mode,int on)
{
	vout_t *vo;
	int i;

	DBG(KERN_ALERT "vout_set_mode(%d,%d)\n",mode,on);

	if( mode > VOUT_MODE_MAX ){
		printk(KERN_ALERT "*E* vout mode invalid %d\n",mode);
		return -1;
	}

	if( !(vo = vout_array[mode]) ){
		return -1;
	}

	if( on ){
		if( vo->active ){
			return 0;
		}

		for(i=0;i<VOUT_MODE_MAX;i++){
			if( i == mode )
				continue;
			if( !(vo = vout_array[i]) )
				continue;
			if( vo->active ){
				if( vout_op(vo,VOCTL_COMPATIBLE,mode) ){
					vout_op(vo,VOCTL_UNINIT,0);
					vo->active = 0;
				}
			}
		}
		vo = vout_array[mode];
		vout_op(vo,VOCTL_INIT,0);
		vo->active = 1;
	}
	else {
		vout_op(vo,VOCTL_UNINIT,0);
		vo->active = 0;
	}
	return 0;
} /* End of vout_set_mode */

/*!*************************************************************************
* vout_config()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_config(vout_mode_t mode,vout_info_t *info)
{
	return vout_control(mode,VOCTL_CONFIG,(int)info); 
} /* End of vout_config */

/*!*************************************************************************
* vout_suspend()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_suspend(vout_mode_t mode,int level)
{
	return vout_control(mode,VOCTL_SUSPEND,level); 
} /* End of vout_suspend */

/*!*************************************************************************
* vout_resume()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_resume(vout_mode_t mode,int level)
{
	return vout_control(mode,VOCTL_RESUME,level); 
} /* End of vout_resume */

/*!*************************************************************************
* vout_chkplug()
* 
* Private Function by Sam Shen, 2009/09/24
*/
/*!
* \brief	
*		
* \retval  1 - plug in, 0 - plug out
*/ 
int vout_chkplug(vout_mode_t mode)
{
	vout_t *vo;
	int ret = 0;
	
	if( (vo = vout_array[mode]) ){
		ret = vout_op(vo,VOCTL_CHKPLUG,0);
	}
	return ret; 
} /* End of vout_resume */

/*!*************************************************************************
* vout_register()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_register(vout_mode_t mode,vout_t *vo)
{
	if( mode >= VOUT_MODE_MAX ){
		printk(KERN_ALERT "*E* vout mode invalid %d\n",mode);
		return -1;
	}

	if( vout_array[mode] ){
		printk(KERN_ALERT "*W* vout mode register again %d\n",mode);
	}
	
	vout_array[mode] = vo;
	vo->enable = 0;
	vo->active = 0;
	return 0;
} /* End of vout_register */

/*!*************************************************************************
* vout_unregister()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
int vout_unregister(vout_mode_t mode)
{
	if( mode >= VOUT_MODE_MAX ){
		printk(KERN_ALERT "*E* vout mode invalid %d\n",mode);
		return -1;
	}
	
	if( vout_array[mode] ){
		vout_array[mode] = 0;
	}
	return 0;
} /* End of vout_unregister */

/*!*************************************************************************
* vout_get_info()
* 
* Private Function by Sam Shen, 2008/06/12
*/
/*!
* \brief	
*		
* \retval  0 if success
*/ 
vout_t *vout_get_info(vout_mode_t mode)
{
	return vout_array[mode];
} /* End of vout_get_info */
/*--------------------End of Function Body -----------------------------------*/
#undef VOUT_C

