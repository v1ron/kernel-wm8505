/********************************************************
Author:		Guan Shibo
Time:		2007-09-10
File Name:	init.c
Description:USB modem driver, registered as a tty driver	

Copyright:  (c) 2002-2009 VIA TELECOM Corporation
            All Rights Reserved
*****************************************************/
/*Introduction of the multiple kernel version support, added by Guan Shibo, 2007-10-22
 *Compare the kernel version about 2.6.20, list the result as following
 * 1)kernel version beyond or equal 2.6.20, indicated by macro __VIA_KVER_GE_2620__
 *   a)INIT_WORK has two parameters, and set_termios method used the struct ktermios as parameter
 *   b)the usb_complete_t only has one parameter
 * 2)kernel version equal 2.6.19, indicated by macro __VIA_KVER_EQ_2619__
 *   a)INIT_WORK has three parameters, and set_termios method used the struct termios as parameter
 *   b)the usb_complete_t only has one parameter
 * c)kernel versioni lower than 2.6.19 , indicated by macro __VIA_KVER_LE_2618__   (only checked the version 2.6.28, other versions not sure)
 *   a)see kernel version 2.6.19
 *   b)the usb_complete_t has two parameters
End of introduction of multiple kernel versioin support, added by Guan Shibo, 2007-10-22*/
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/smp_lock.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/list.h>

#define __VIA_KVER_GE_2620__

#if defined(__VIA_KVER_EQ_2615__)
#include <asm/semaphore.h>
#else
#include <linux/mutex.h>
#endif

#include "via_usb_modem.h"

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/tty.h>

#define DRIVER_AUTHOR "Huaping Jiang ,Shibo Guan"
#define DRIVER_DESC "VIA USB CDMA Modem for Delta"
#define DRIVER_VERSION "LinuxVIAUSBD1.0.4.1"

#define VIAUSBModem_MAJOR		167
#define VIAUSBModem_MINORS		32


extern struct usb_driver VIAUSBModem_driver;
extern struct tty_driver *VIAUSBModem_tty_driver;
extern struct tty_operations VIAUSBModem_ops;



/*
 * Version Information
 */
//#define VIA_DEBUG 0

#define DRIVER_VERSION "LinuxVIAUSBD1.0.4"
#define DRIVER_AUTHOR "Huaping Jiang ,Shibo Guan"
#define DRIVER_DESC "VIA USB CDMA Modem"

#define VIA_READY(via_dev)	(via_dev && via_dev->dev && via_dev->used)

#if defined(__VIA_KVER_EQ_2615__)
/* #define mutex_init(lock, type, name)		sema_init(lock, 1) */
//#define mutex_destroy(lock)			sema_init(lock, -99)
#define mutex_lock(lock)			down(lock)
//#define mutex_trylock(lock)			(down_trylock(lock) ? 0 : 1)
#define mutex_unlock(lock)			up(lock)
#endif

struct usb_driver VIAUSBModem_driver;
struct tty_driver *VIAUSBModem_tty_driver;
static void wb_free(struct VIAUSBModem_Dev *via_dev, int wbn);
static int via_write_start(struct VIAUSBModem_Dev *via_dev);

static struct VIAUSBModem_Dev *VIAUSBModem_table[VIAUSBModem_MINORS];

/* Added support for kernel 2.6.15 */
#if defined(__VIA_KVER_EQ_2615__)
static DECLARE_MUTEX(open_mutex);
#else
static DEFINE_MUTEX(open_mutex);
#endif

/* Added by Guan Shibo, for debugging the buffer contents */
void debug_buf_content(const unsigned char *buffer, int length)
#ifdef VIA_DEBUG 
{
    int i = 0;
    int j = 0;

	/* Added by Guan Shibo, for parameters check, 2007-11-01 */
    if(buffer == NULL)
    {
        return;
    }
    /* End of added by Guan Shibo, for parameters check, 2007-11-01 */
    
    for(i = 0; i < length; i ++)
    {
        if((i % 8) == 0)
        {
            printk(KERN_DEBUG "\n<VIAUSB>-line[%d]", j);
            j++;
        }
        printk(KERN_DEBUG " [0x%x]", buffer[i]);
    }
}
#else
{
}
#endif
/* End of added by Guan Shibo, for debugging the buffer contents */

/*****************************************************
 * Function Name: via_read_bulk
 * Input:       1)urb: A pointer of struct urb, pointed to the completed URB
 * Output:      1)None
 * Ret value:   1)None
 * Description: This fucntion is the complete fucntion of URB bulk read, which submitted in via_work_inter_read. If the URB complete successfully, we push the result to tty core directly and reschedule the work_inter_read to read the next data. If something wrong, the result will be dropped and reschedule the work_inter_read only 
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-17
 ******************************************************************/
/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__) || defined(__VIA_KVER_EQ_2619__)
void via_read_bulk(struct urb *urb)
#else
void via_read_bulk(struct urb *urb, struct pt_regs *regs)
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */

{
    struct VIAUSBModem_Dev *via_dev = NULL;
    int status = 0;
    struct tty_struct *tty = NULL;
    unsigned char throttled = 0;/* Stored the throttle status of device */
    unsigned long flags = 0;	/* Stored the irq flags temporarily */

    dbg("Enter via_read_bulk, complete of URB read bulk......");
/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(urb == NULL)
    {
        dbg("Error, urb is NULL");
        return;
    }
/* End of added by Guan Shibo, 2007-10-08 */
    via_dev = urb->context;
    status = urb->status;
   	/* Modified by Guan Shibo, correct the bug, 2007-11-01 */
    /*tty = via_dev->tty;*/

    if(!(VIA_READY(via_dev) && (tty = via_dev->tty)))
    {
        dbg("via_dev or via_dev->dev is NULL");
        return;
    }
   	/* End of modified by Guan Shibo, 2007-11-01 */

    dbg("Bulk rx status <%d>, get <%d> bytes", status, urb->actual_length);

    if(likely(status == 0))	/* The URB completed successfully */
    {
#if defined(__VIA_KVER_EQ_2615__)
	int ti = 0;
	for(ti = 0; ti < urb->actual_length; ti++)
	{
		if(tty->flip.count >= TTY_FLIPBUF_SIZE)
		{
			tty_flip_buffer_push(tty);
		}
		tty_insert_flip_char(tty, via_dev->read_buffer[ti], 0);
	}
	tty_flip_buffer_push(tty);	
#else
        tty_buffer_request_room(tty, urb->actual_length);
        spin_lock_irqsave(&via_dev->throttle_lock, flags);
        throttled = via_dev->throttle;
        spin_unlock_irqrestore(&via_dev->throttle_lock, flags);

/*No read buffer, so we can't handle throttle, only display the status of throttled here */       
        if(throttled)
        {
            dbg("VIA Modem driver is throttled");
        }
        tty_insert_flip_string(tty, via_dev->read_buffer, urb->actual_length);
        dbg("After insert_flip the read content");
        debug_buf_content(via_dev->read_buffer, urb->actual_length);

        tty_flip_buffer_push(tty);

#endif
    }

    dbg("Leave via_read_bulk, and reschedule the work_inter_read");
    schedule_work(&via_dev->work_inter_read);

    return;
}

/*****************************************************
 * Function Name: via_work_inter_read
 * Input:       1)private: A void pointer, pointed to the work_inter_read work_struct 
 * Output:      1)None
 * Ret value:   1)None
 * Description: This is the real routine of work_inter_read work_struct. In this function, we filled the URB with pointed parameters and submitted it to the USB core.
 *              P.S, we did not use buffer in read method. so we cann't and handle the throttle, in fact. We reserved the throttle status check only for the future. 
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-17
 ******************************************************************/ 
/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__) 
void via_work_inter_read(struct work_struct *private)
#else
void via_work_inter_read(void *private)
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
{

    unsigned long flags = 0;	/* Stored the irq flags temporarily */
    unsigned char throttle = 0;	/* Stored the throttle status of the VIA USB modem */
    int ret = -1;

    /* Get the pointer of device from the member  */
    struct VIAUSBModem_Dev *via_dev = NULL;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(private == NULL)
    {
        dbg("Error, private is NULL");
        return;
    }
/* End of added by Guan Shibo, 2007-10-08 */

/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__)
    via_dev = container_of(private, struct VIAUSBModem_Dev, work_inter_read);
#else
    via_dev = (struct VIAUSBModem_Dev *)private;
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */

    dbg("Entering via_work_inter_read............");

    if(!(VIA_READY(via_dev)))
    {
        dbg("via_dev or via_dev->dev is NULL");
        return;
    }

    spin_lock_irqsave(&via_dev->throttle_lock, flags);	/* Access the throttle status */
    throttle = via_dev->throttle;
    spin_unlock_irqrestore(&via_dev->throttle_lock, flags);
    if(throttle != 0)
    {
        dbg("Device throttled..........");
        return;
    }

    usb_fill_bulk_urb(via_dev->read_urb, via_dev->dev, via_dev->rx_endpoint, 
            via_dev->read_buffer, via_dev->readsize, via_read_bulk, via_dev);
    via_dev->read_urb->transfer_dma = via_dev->read_dma;
    via_dev->read_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    ret = usb_submit_urb(via_dev->read_urb, GFP_KERNEL);	
    dbg("Leave via_work_inter_read..........., submit inter_read URB ret=<%d>", ret);

    return;
}

/*****************************************************
 * Function Name: via_poll_status
 * Input:       1)urb: A pointer of struct urb, pointed to the completed URB
 * Output:      1)None
 * Ret value:   1)None
 * Description: This fucntion is the complete fucntion of URB ctrl read, which submitted in via_work_poll_status. If the URB complete successfully, we store the VIA USB modem status and reschedule the work_poll_status to poll the next status. If something wrong but not serious, the result will be dropped and reschedule the work_poll_status only. If meeting serious problem, this function will return and not reschedule the work. 
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-18
 ******************************************************************/
/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__) || defined(__VIA_KVER_EQ_2619__)
void via_poll_status(struct urb *urb)
#else
void via_poll_status(struct urb *urb, struct pt_regs *regs)
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
{
    struct VIAUSBModem_Dev *via_dev = NULL;
    int status = 0;
    unsigned char uc_msr = 0;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(urb == NULL)
    {
        dbg("Error, urb is NULL");
        return;
    }
/* End of added by Guan Shibo, 2007-10-08 */
    via_dev = urb->context;
    status = urb->status;

   /* dbg("Enter via_poll_status, complete of URB poll status......");*/
    if(!(VIA_READY(via_dev)))
    {
        dbg("via_dev or via_dev->dev is NULL");
        return;
    }
    
    /*dbg("Poll status urb's status is  <%d>", status);*/

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dbg("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);
		return;
	default:
		dbg("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
		goto exit;
	}
    if(urb->actual_length != 1)
    {
        dbg("urb actual-length != 1");
        goto exit;
    } 
    else
    {
        /*Added by Guan Shibo, for CR=HCT#960, 2007-10-13*/
        uc_msr = *((unsigned char *)(via_dev->poll_status_buffer));
        if(via_dev->tty && (via_dev->msr & (~uc_msr) & 0x1))
        {
            if(via_dev->cd_high)
            {
                dbg("cd_high set, clear it and continue");
                via_dev->cd_high = 0;
            }
            else
            {
                dbg("CD or DSR not detected, will hangup the tty");
                tty_hangup(via_dev->tty);
            }
        }
        dbg("old msr=<0x%x>, new msr=<0x%x>", via_dev->msr, uc_msr);
        via_dev->msr = uc_msr;
        /*End of added by Guan Shibo, for CR=HCT#960, 2007-10-13*/
    }

exit:
    /*Modified by Guan Shibo, for slow the poll status, 2007-10-26*/
    schedule_delayed_work(&via_dev->work_poll_status, HZ);
    /*End of modified by Guan Shibo, for slow the poll status, 2007-10-26*/
    return;    
}

/*****************************************************
 * Function Name: via_work_poll_status
 * Input:       1)urb: A void pointer, pointed to the work_poll_status work_struct
 * Output:      1)None
 * Ret value:   1)None
 * Description: Fill the poll status URB and submit it to poll the VIA USB modem status.
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-18
 ******************************************************************/
/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__)
void via_work_poll_status(struct work_struct *private)
#else
void via_work_poll_status(void *private)
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
{
    /* Get the pointer of device from the member  */
    struct VIAUSBModem_Dev *via_dev = NULL;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(private == NULL)
    {
        dbg("Error, private is NULL");
        return;
    }
/* End of added by Guan Shibo, 2007-10-08 */

/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__)
    via_dev = container_of(private, struct VIAUSBModem_Dev, work_poll_status.work);
#else
    via_dev = (struct VIAUSBModem_Dev *)private;
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */

    /*    dbg("Entering via_work_poll_status............");*/

    if(!(VIA_READY(via_dev)))
    {
        dbg("via_dev or via_dev->dev is NULL");
        return;
    }

    usb_fill_control_urb(via_dev->poll_status_urb, via_dev->dev, usb_rcvctrlpipe(via_dev->dev, 
                via_dev->ctrl_epAddress), via_dev->ctrl_buffer, via_dev->poll_status_buffer, 
                via_dev->ctrlsize, via_poll_status, via_dev); 
    via_dev->poll_status_urb->actual_length = 0;

    usb_submit_urb(via_dev->poll_status_urb, GFP_KERNEL);

    return;    
}

/*****************************************************
 * Function Name:VIAUSBModem_ctrl_msg
 * Input:       1)via_dev:A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1)0,success, or error state.
 * Description: used to send control command to CBP device.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_ctrl_msg(struct VIAUSBModem_Dev *viaUsbModem, int type,int request, int value, void *buf, int len,int timeout)
{
	int iretval;
	dbg("VIAUSBModem_ctrl_msg:Enter.");
	dbg("receive control pipe is 0x%x",usb_rcvctrlpipe(viaUsbModem->dev,viaUsbModem->ctrl_epAddress));
	dbg("send control pipe is 0x%x",usb_sndctrlpipe(viaUsbModem->dev,viaUsbModem->ctrl_epAddress));
	dbg("send control pipe use 0 is 0x%x",usb_sndctrlpipe(viaUsbModem->dev,0));
        dbg("receive control pipe use 0 is 0x%x",usb_rcvctrlpipe(viaUsbModem->dev,0));

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(!(VIA_READY(viaUsbModem)))
    {
        dbg("Error, via_dev is NULL");
        return -1;
    }
/* End of added by Guan Shibo, 2007-10-08 */
	iretval = usb_control_msg(viaUsbModem->dev, 
		usb_sndctrlpipe(viaUsbModem->dev, viaUsbModem->ctrl_epAddress),
		request, type, value,
		0,
		buf, len, timeout);

	dbg("VIAUSBMODEM_control_msg: rq: 0x%02x val: %#x len: %#x result: %d", request, value, len, iretval);
	dbg("VIAUSBModem_ctrl_msg:Exit");
	return iretval;
}

#define VIAUSBMODEM_SET_DTR(via_dev)\
	VIAUSBModem_ctrl_msg(via_dev,VIAUSBMODEM_CONTROL,VIAUSBMODEM_DTR,1,NULL,0,100)
#define VIAUSBMODEM_CLR_DTR(via_dev)\
	VIAUSBModem_ctrl_msg(via_dev,VIAUSBMODEM_CONTROL,VIAUSBMODEM_DTR,0,NULL,0,100)

/*****************************************************
 * Function Name:via_wb_is_avail
 * Input:       1)via_dev:A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1).
 * Description: .
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int via_wb_is_avail(struct VIAUSBModem_Dev *via_dev)
{
	int i, n;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((via_dev == NULL) || (via_dev->dev == NULL))
    {
        dbg("Error, via_dev is NULL");
        return -1;
    }
/* End of added by Guan Shibo, 2007-10-08 */

	n = VIAUSBMODEM_NW;
	for (i = 0; i < VIAUSBMODEM_NW; i++) 
	{
		n -= via_dev->wb[i].use;
		dbg("In wb_avail, wb[%d].use=<%d>", i, via_dev->wb[i].use);
	}
	return n;
}

/*****************************************************
 * Function Name:via_write_done
 * Input:       1)via_dev:A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1)None.
 * Description: called when one write op is complete, to release all resource.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static void via_write_done(struct VIAUSBModem_Dev *via_dev)
{
	/*unsigned long flags;*/
	int wbn;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((via_dev == NULL) || (via_dev->dev == NULL))
    {
        dbg("Error, via_dev is NULL");
        return;
    }
/* End of added by Guan Shibo, 2007-10-08 */

    dbg("Enter via_write_done.............");
	spin_lock(&via_dev->write_lock);
	via_dev->write_ready = 1;
	wbn = via_dev->write_current;
	wb_free(via_dev, wbn);
	via_dev->write_current = (wbn + 1) % VIAUSBMODEM_NW;
	spin_unlock(&via_dev->write_lock);
    dbg("Leave via_write_done.............");
}


/*****************************************************
 * Function Name:wb_alloc_wb_array
 * Input:       1)via_dev:A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1)return -1 If failed, or return 0.
 * Description: to alloc all write buffers.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int via_alloc_wb_array(struct VIAUSBModem_Dev *via_dev)
{
	int wbn;
	int i;
	/*struct VIAUSBModem_wb *pWb=NULL;*/
	unsigned char *pBuf=NULL;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((via_dev == NULL) || (via_dev->dev == NULL))
    {
        dbg("Error, via_dev is NULL");
        return -1;
    }
/* End of added by Guan Shibo, 2007-10-08 */
	for (wbn=0;wbn<VIAUSBMODEM_NW;wbn++)
	{
		pBuf = usb_buffer_alloc(via_dev->dev,via_dev->writesize,GFP_KERNEL,&via_dev->wb[wbn].dmah);
		if(!pBuf)
		{
			dbg("via_alloc_wb_array:out of memory (write_dma buffer alloc)");
			goto alloc_fail;
		}
		via_dev->wb[wbn].buf = pBuf;
		
	}
	
	return 0;
alloc_fail:
	
	/*pWb = via_dev->wb;*/
	for (i=0;i<wbn;i++)
	{
		usb_buffer_free(via_dev->dev,via_dev->wb[i].len,via_dev->wb[i].buf,via_dev->wb[i].dmah);
	}
	dbg("via_alloc_wb_array: allocate wb array failed.\n");
	return -1;
}


/*****************************************************
 * Function Name:via_free_wb_array
 * Input:       1)via_dev:A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1)None
 * Description: to free all usb write buffers.
 *
 * Author:      hjiang
 * Create data: 2007-09-25
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static void via_free_wb_array(struct VIAUSBModem_Dev *via_dev)
{
	int wbn;
/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((via_dev == NULL) || (via_dev->dev == NULL))
    {
        dbg("Error, via_dev is NULL");
        return;
    }
/* End of added by Guan Shibo, 2007-10-08 */

	for (wbn=0;wbn<VIAUSBMODEM_NW;wbn++)
	{
		usb_buffer_free(via_dev->dev,via_dev->writesize,via_dev->wb[wbn].buf,via_dev->wb[wbn].dmah);
	}
	return;
}

/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__) || defined(__VIA_KVER_EQ_2619__)
static void via_write_bulk(struct urb *urb)
#else
static void via_write_bulk(struct urb *urb, struct pt_regs *regs)
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */

{
	struct VIAUSBModem_Dev  *via_dev= NULL;
	if (!urb)
		return;

	via_dev = (struct VIAUSBModem_Dev*)urb->context;

	dbg("Entering via_write_bulk with status %d", urb->status);
	/* Added by Guan Shibo, for parameters check, 2007-11-01 */
    if(!(VIA_READY(via_dev)))
    {
        dbg("via_dev is NULL");
        return;
    }
	/* End of Added by Guan Shibo, for parameters check, 2007-11-01 */
	via_write_done(via_dev);
	via_write_start(via_dev);

	/* Added by Guan Shibo, for parameters check, 2007-11-01 */
    if(via_dev->tty)
    {
        tty_wakeup(via_dev->tty);	/* Added by Guan Shibo, 2007-09-29 */
    }
	/* End of Added by Guan Shibo, for parameters check, 2007-11-01 */
}

/*****************************************************
 * Function Name: VIAUSBModem_tty_unregister
 * Input:       1)via_dev: the pointer of device
 * Output:      1)None
 * Ret value:   1)None
 * Description: Unregister the tty device and free allocted objects, included URBs and via_dev
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-18
 * Modified history:
 * 1)   Modified by:    Guan Shibo, 2007-09-19
 *      Desciption:     Add the free operatioin of poll status urb
 ******************************************************************/
void VIAUSBModem_tty_unregister(struct VIAUSBModem_Dev *via_dev)
{
    struct tty_struct *st_tty = NULL;

	dbg("Enter VIAUSBModem_tty_unregister............");
    /* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(via_dev == NULL)
    {
        dbg("Error, via_dev is NULL");
        return;
    }
    /* End of added by Guan Shibo, 2007-10-08 */
    st_tty = via_dev->tty;
    tty_unregister_device(VIAUSBModem_tty_driver, via_dev->minor);
	usb_put_intf(via_dev->interface);
	VIAUSBModem_table[via_dev->minor] = NULL;
	usb_free_urb(via_dev->read_urb);
	usb_free_urb(via_dev->write_urb);

    usb_free_urb(via_dev->poll_status_urb); 	/* Free the poll_status URB, added by Guan Shibo, 2007-09-18 */
	kfree(via_dev);
    if(st_tty)
    {
        st_tty->driver_data = NULL;
    }
	
	return;
}

/*****************************************************
 * Function Name:VIAUSBModem_probe 
 * Input:       1)intf: A pointer of usb interface struct
 *              2)id:   The usb id information
 * Output:      1)None
 * Ret value:   1)If success, return 0, otherwise ,return a negative value
 * Description: In probe process, core will call this function. We get the interface and endpoints information ,alloc the urbs and do some initialize works in this function.
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-10
 *
 * Modified history:
 * 1)   Modified by:    Guan Shibo, 2007-09-18
 *      Desciption:     Add the poll status urb and work_struct initialization.
 *
 ******************************************************************/
int VIAUSBModem_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	/*Modified by Shibo Guan, 2007-09-10*/
	/*struct usb_endpoint_descriptor *epread = NULL;*/
	struct usb_endpoint_descriptor *eptmp=NULL, *temp_ep;
	/*End of modified by Shibo Guan, 2007-09-10*/

	struct usb_device *usb_dev =NULL;
	struct VIAUSBModem_Dev *via_dev = NULL;
	/*struct VIAUSBModem_wb *wb=NULL;*/
	int minor;
	int i;

	//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);

    struct usb_ctrlrequest *cr_tmp = NULL;	/* Store the poll status setup packet, added by Guan Shibo, 2007-09-18 */
	struct usb_host_interface *pHostIntf=NULL;
	
	dbg("enter probe..........................");	
	if ((intf == NULL) || (id == NULL))
	{
		dbg("parameter interface is NULL.");
		//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}	
	usb_dev = interface_to_usbdev(intf);
	
	pHostIntf = intf->cur_altsetting;
	if(pHostIntf->desc.bInterfaceNumber!=0x00)
	{
		dbg("probe:This interface is not the VIA telecom USB modem interface.\n");
		//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}
	dbg("probe:Now find the correct VIA telecom USB modem interface.\n");
	
	dbg("interfaces are valid");

	for (minor = 0; minor < VIAUSBModem_MINORS && VIAUSBModem_table[minor]; minor++)
	{
	}

	if (minor == VIAUSBModem_MINORS) {
		err("no more free VIAUSBModem devices");
		//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}
	
	dbg("before kmalloc.\n");	
	via_dev = kzalloc(sizeof(struct VIAUSBModem_Dev), GFP_KERNEL);
	if(via_dev==NULL)
	{
		dbg("out of memory (kmalloc)");
		//printk("%s(): line: %u: via modem~~~~~SIZEOF IS %u\n", __FUNCTION__, __LINE__, sizeof(struct VIAUSBModem_Dev));
		goto alloc_fail;
	}
	
	dbg("probe:Now kmalloc VIAUSBModem_Dev succeed.\n");
	dbg("probe:endpoints=%d\n",intf->cur_altsetting->desc.bNumEndpoints);

	/*save two read and write endpoints.*/
    via_dev->rx_endpoint = 0;
    via_dev->tx_endpoint = 0;
	for(i=0;i<intf->cur_altsetting->desc.bNumEndpoints;i++)
	{
		eptmp = &intf->cur_altsetting->endpoint[i].desc;
		if((eptmp->bEndpointAddress & USB_DIR_IN) &&
			((eptmp->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_BULK))
		{
			via_dev->readsize = le16_to_cpu(eptmp->wMaxPacketSize);
			via_dev->rx_epAddress = eptmp->bEndpointAddress;
			dbg("rx endpoint address=0x%x ",eptmp->bEndpointAddress);
			via_dev->rx_endpoint = usb_rcvbulkpipe(usb_dev,eptmp->bEndpointAddress);
			dbg("rx_endpoint=0x%x\n",via_dev->rx_endpoint);
		}
		if(!(eptmp->bEndpointAddress & USB_DIR_IN) &&	
			((eptmp->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_BULK))
		{
			via_dev->writesize = le16_to_cpu(eptmp->wMaxPacketSize);
			via_dev->tx_epAddress = eptmp->bEndpointAddress;
			dbg("tx endpoint address=0x%x ",eptmp->bEndpointAddress);
			via_dev->tx_endpoint = usb_sndbulkpipe(usb_dev,eptmp->bEndpointAddress);
			dbg("tx_endpoint=0x%x\n",via_dev->tx_endpoint);
		}
		/*if(!(via_dev->tx_endpoint && via_dev->rx_endpoint))
		{
			dbg("probe:Could not find both read and write endpoints.\n");
			goto alloc_fail1;
		}*/
	}
	
	/*Added by Shibo Guan, 2007-09-10*/
	if(!(via_dev->tx_endpoint && via_dev->rx_endpoint))
	{
		dbg("Endpoint is incorrect");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		goto alloc_fail1;
	}
	/*End of Added by Shibo Guan, 2007-09-10*/
	
	dbg("probe:Now kmalloc VIAUSBModem_Dev succeed.");
	via_dev->interface = intf;
	via_dev->minor = minor;
	via_dev->dev = usb_dev;
	via_dev->used = 0;
	via_dev->write_current = 0;
	via_dev->write_ready = 1;
	/*save contrl endpoint*/
	eptmp = &usb_dev->ep0.desc;			// luiszhou
	//eptmp = &(usb_dev->actconfig->interface[0]->cur_altsetting->endpoint[0].desc);	
	

	if(!eptmp){
		printk("~~~~~~~~~~~~~~~~~~~~~`\n");
	}
#if 0	
	int index = 0;
	for(index=0; index<3; index++){
		//temp_ep = &(usb_dev->actconfig->interface[index]->cur_altsetting->endpoint[0]);
		temp_ep = &(usb_dev->actconfig->interface[index]->cur_altsetting->endpoint[0].desc);		

		if(temp_ep && 
			usb_dev->actconfig && 
			usb_dev->actconfig->interface[index] && 
			usb_dev->actconfig->interface[index]->cur_altsetting &&
			usb_dev->actconfig->interface[index]->cur_altsetting->endpoint)


		if(((temp_ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_CONTROL)){
			//dbg("probe:control endpoint isn't correct.\n");
			printk("%s(): line: %u: OK!!!!!!!!!!!!!!!!!!!!!THE INTERFACE IS %d\n", __FUNCTION__, __LINE__, index);
			goto alloc_fail1;		
		}
	}

	printk("%s(): line: %u: via modem~~~~%x~~~~%x %x %x %x %x %x\n", 
			__FUNCTION__, 
			__LINE__,
			usb_dev->actconfig->interface[0]->cur_altsetting->desc.bNumEndpoints,
			eptmp->bLength,
			eptmp->bDescriptorType,
			eptmp->bEndpointAddress,
			eptmp->bmAttributes,
			eptmp->wMaxPacketSize,
			eptmp->bInterval);
#endif
	if(!((eptmp->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_CONTROL))
	{
		dbg("probe:control endpoint isn't correct.\n");
#if 0
		printk("%s(): line: %u: via modem~~~~%x~~~~%x %x %x %x %x %x\n", 
			__FUNCTION__, 
			__LINE__,
			usb_dev->actconfig->interface[0]->cur_altsetting->desc.bNumEndpoints,
			eptmp->bLength,
			eptmp->bDescriptorType,
			eptmp->bEndpointAddress,
			eptmp->bmAttributes,
			eptmp->wMaxPacketSize,
			eptmp->bInterval);
#endif
		goto alloc_fail1;		
	}
	else
	{
		dbg("probe:now control endpoint is found and ep address is <%d>", eptmp->bEndpointAddress);
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		via_dev->ctrl_epAddress = eptmp->bEndpointAddress;
	}
    via_dev->ctrlsize = le16_to_cpu(eptmp->wMaxPacketSize);	/* Get the ctrlsize of ctrl endpoint, added by Guan Shibo, 2007-09-18 */

//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	dbg("probe:now control endpoint is found.\n");

	/* Initialize the work struct, bind the real function to them */
    /* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__)
    INIT_WORK(&via_dev->work_inter_read, via_work_inter_read);
    INIT_DELAYED_WORK(&via_dev->work_poll_status, via_work_poll_status);
#else
    INIT_WORK(&via_dev->work_inter_read, via_work_inter_read, via_dev);
    INIT_WORK(&via_dev->work_poll_status, via_work_poll_status, via_dev);
#endif
    /* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */

	/* Recommended to reserve this endpoint, it will be used in rx routine */

	/* Alloc urb struct and buffer of urb for read bulk, write bulk and poll status */
	
	via_dev->read_buffer = usb_buffer_alloc(usb_dev, via_dev->readsize, GFP_KERNEL, &via_dev->read_dma);
	if(!via_dev->read_buffer)
	{
        dbg("out of memory (read_dma buffer alloc)");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		goto alloc_fail1;
	}
	/* allocate write buffer---wb array*/
	if (via_alloc_wb_array(via_dev))
	{
        dbg("out of memory (write_dma buffer alloc)");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		goto alloc_fail2;
	}
	
	via_dev->read_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!via_dev->read_urb)
	{
        dbg("out of memory (read_urb kmalloc)");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		goto alloc_fail3;
	}
	via_dev->read_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	
	via_dev->write_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!via_dev->write_urb)
	{
        dbg("out of memory (write_urb kmalloc)");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
		goto alloc_fail4;
	}
    /* Initialize the poll_cd command, added by Guan Shibo, 2007-09-19 */
    via_dev->poll_cd_cmd.ucRequestType = 0xc0;
    via_dev->poll_cd_cmd.ucRequest = 0x02;
    via_dev->poll_cd_cmd.usValue = 0;
    via_dev->poll_cd_cmd.usIndex = 0;
    via_dev->poll_cd_cmd.usLength = 1;   
    /* End of added by Guan Shibo, 2007-09-19 */

	/* Alloc the dma buffer and URB for poll status, added by Guan shibo, 2007-09-18 */
    via_dev->poll_status_buffer = kzalloc(via_dev->ctrlsize, GFP_KERNEL);
    if(!via_dev->poll_status_buffer)
    {
        dbg("out of memory (poll_status_dma buffer alloc)");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
        goto alloc_fail5;
    }

    via_dev->poll_status_urb = usb_alloc_urb(0, GFP_KERNEL);
    if(!via_dev->poll_status_urb)
    {
        dbg("out of memory (poll_status_urb kmalloc)");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
        goto alloc_fail6;
    }

    via_dev->ctrl_buffer = NULL;
    cr_tmp = (struct usb_ctrlrequest *) kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
    if(!cr_tmp)
    {
        dbg("Kmalloc failed, for ctrl URB's setup packet");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
        goto alloc_fail7;
    } 
    cr_tmp->bRequestType= via_dev->poll_cd_cmd.ucRequestType;
    cr_tmp->bRequest = via_dev->poll_cd_cmd.ucRequest;
    cr_tmp->wValue = cpu_to_le16p(&via_dev->poll_cd_cmd.usValue);
    cr_tmp->wIndex = cpu_to_le16p(&via_dev->poll_cd_cmd.usIndex);
    cr_tmp->wLength = cpu_to_le16p(&via_dev->poll_cd_cmd.usLength);
    via_dev->ctrl_buffer = (unsigned char *)cr_tmp;
    /* End of added by Guan Shibo, 2007-09-18 */

    usb_set_intfdata (intf, via_dev);	/* Store the current device into the interface's member, so we can get the device later */
	
	/* Fill the write_urb, it will be submitted in write method */
	usb_fill_bulk_urb(via_dev->write_urb, usb_dev, usb_sndbulkpipe(usb_dev, via_dev->tx_epAddress),
			  NULL, via_dev->writesize, via_write_bulk, via_dev);
	via_dev->write_urb->transfer_flags |= URB_NO_FSBR | URB_NO_TRANSFER_DMA_MAP;
	
	dbg("ttyVIA[%d]: VIA USB Modem device\n", minor);
	tty_register_device(VIAUSBModem_tty_driver, minor, &intf->dev);

    VIAUSBModem_table[minor] = via_dev;	/* Store the current device into the global VIA USB modem device array */

//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	dbg("probe:exit successfully.\n");

	return 0;

	/* Error handler, free the allocated URB and buffer */
alloc_fail7:
    dbg("in probe, alloc_fail7");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
    usb_free_urb(via_dev->poll_status_urb);

alloc_fail6:
    dbg("in probe, alloc_fail6");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
    kfree(via_dev->poll_status_buffer);

alloc_fail5:
    dbg("in probe, alloc_fail5");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
    usb_free_urb(via_dev->write_urb);
alloc_fail4:
	dbg("in probe, alloc_fail4");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	usb_free_urb(via_dev->read_urb);
	
alloc_fail3:
	dbg("in probe, alloc_fail3");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	via_free_wb_array(via_dev);
	
alloc_fail2:
	dbg("in probe, alloc_fail2");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	usb_buffer_free(usb_dev, via_dev->readsize, via_dev->read_buffer, via_dev->read_dma);

alloc_fail1:
	dbg("in probe, alloc_fail1");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	kfree(via_dev);
	
alloc_fail:
	dbg("in probe, alloc_fail");
//printk("%s(): line: %u: via modem~~~~~~~~~~~~~~~~~~\n", __FUNCTION__, __LINE__);
	return -ENOMEM;
}

/*****************************************************
 * Function Name:VIAUSBModem_disconnect
 * Input:       1)intf: A pointer of usb interface struct
 * Output:      1)None
 * Ret value:   1)None
 * Description: Do the clean up jos in this fucntion when the device deattath. We will free the allocated objects and unregister device here.
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-10
 *
 * Modified history:
 * 1)   Modified by:    Guan Shibo, 2007-09-18
 *      Desciption:     Add the free operation of poll status urb and work_struct .
 *
 ******************************************************************/
void VIAUSBModem_disconnect(struct usb_interface *intf)
{
	struct VIAUSBModem_Dev *via_dev = NULL;
	struct usb_device *usb_dev = NULL;
    
    dbg("disconnect:Enter");
	
	mutex_lock(&open_mutex);
	via_dev = usb_get_intfdata(intf);	
	usb_set_intfdata(intf,NULL);

	if (!via_dev || !via_dev->dev) {
		dbg("disconnect on nonexisting interface");
		mutex_unlock(&open_mutex);
		return;
	}
	dbg("get dev and usb_dev successfully.\n");
	
	usb_dev = via_dev->dev;
	via_dev->dev = NULL;
	
    /*Added by Guan Shibo, for slow the poll status, 2007-10-26*/
    cancel_delayed_work(&via_dev->work_poll_status);
    /*End of added by Guan Shibo, for slow the poll status, 2007-10-26*/

    flush_scheduled_work(); /* wait for poll and read works*/

	usb_kill_urb(via_dev->read_urb);
	usb_kill_urb(via_dev->write_urb);
    usb_kill_urb(via_dev->poll_status_urb);	/* Cancel the poll_status URB, added by Guan Shibo, 2007-09-18 */

	usb_buffer_free(usb_dev, via_dev->readsize, via_dev->read_buffer, via_dev->read_dma);
	/*free write buffer*/
	via_free_wb_array(via_dev);
	/*usb_buffer_free(usb_dev, via_dev->writesize, via_dev->write_buffer, via_dev->write_dma);*/
	
    kfree(via_dev->ctrl_buffer);
    kfree(via_dev->poll_status_buffer); /* Free the buffer of poll_status URB, added by Guan Shibo, 2007-09-18 */


    if(!via_dev->used)
    {
        VIAUSBModem_tty_unregister(via_dev);
        mutex_unlock(&open_mutex);
        dbg("disconnect:exit successfully, no opened.");
        return;
    }
	mutex_unlock(&open_mutex);

    /*Added by Guan Shibo, for CR=HCT#960, 2007-10-18*/
	if (via_dev->tty)
    {
		tty_hangup(via_dev->tty);
    }
    /*End of added by Guan Shibo, for CR=HCT#960, 2007-10-18*/

	dbg("disconnect:exit successfully, but have opened.");
	return ;
}

/*****************************************************
 * Function Name:VIAUSBModem_ids
 * Description:used to check if it is VIAUSBModem using vendor ID and product ID.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 
 ******************************************************************/
static struct usb_device_id VIAUSBModem_ids[] = {
	/* quirky and broken devices */
	//{ USB_DEVICE(0x15eb, 0x0001), /* VIA USB Modem */
	{ USB_DEVICE(0x0eba, 0x210a), /* VIA USB Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0eba, 0x209e), /* VIA USB Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x16d8, 0x4000), /* VIA USB Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},	
	{ USB_DEVICE(0x15eb, 0x0001), /* VIA USB Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},	
	{ }
};

MODULE_DEVICE_TABLE (usb, VIAUSBModem_ids);

/*****************************************************
 * Function Name:VIAUSBModem_driver   pid 210a vid 0eba 
 * Description:used to register the processing functions for usb core.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 
 ******************************************************************/
struct usb_driver VIAUSBModem_driver = {
	.name =	"VIAUSBModem",
	.probe =	VIAUSBModem_probe,
	.disconnect =	VIAUSBModem_disconnect,
	.id_table =	VIAUSBModem_ids,
};

/*****************************************************
 * Function Name:wb_alloc
 * Input:       1)via_dev:A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1)if all wb buffer is used then return -1, or any wb buffer is free then return the index of array wb[].
 * Description: to check if there is free write buffer for write; if there is free write buffer then return the index.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int wb_alloc(struct VIAUSBModem_Dev *via_dev)
{
	int i, wbn;
	struct VIAUSBModem_wb *wb;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(via_dev == NULL)
    {
        dbg("Error, via_dev is NULL");
        return -1;
    }
/* End of added by Guan Shibo, 2007-10-08 */

	wbn = via_dev->write_current;
	i = 0;
	for (;;) {
		wb = &via_dev->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % VIAUSBMODEM_NW;
		if (++i >= VIAUSBMODEM_NW)
			return -1;
	}
}

/*****************************************************
 * Function Name:wb_free
 * Input:       1)via_dev: A pointer of VIAUSBModem_Dev
 *                 2)wbn:write buffer index.
 * Output:      1)None
 * Ret value:   1)Noe
 * Description: to free the write buffer indexed by the wbn.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static void wb_free(struct VIAUSBModem_Dev *via_dev, int wbn)
{
/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(via_dev == NULL)
    {
        dbg("Error, via_dev is NULL");
        return ;
    }
/* End of added by Guan Shibo, 2007-10-08 */

	via_dev->wb[wbn].use = 0;
}


/*****************************************************
 * Function Name:via_write_start
 * Input:       1)via_dev: A pointer of VIAUSBModem_Dev
 * Output:      1)None
 * Ret value:   1)rc: returned from the usb_submit_urb()
 * Description: to fill the write urb buffer, then start the writing op
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int via_write_start(struct VIAUSBModem_Dev *via_dev)
{
	int wbn;
	struct VIAUSBModem_wb *wb;
	int rc;

    dbg("Enter via_write_start................");

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(via_dev == NULL)
    {
        dbg("Error, via_dev is NULL");
        return -ENODEV;
    }
/* End of added by Guan Shibo, 2007-10-08 */

	spin_lock(&via_dev->write_lock);
	if (!via_dev->dev) {
		spin_unlock(&via_dev->write_lock);
		return -ENODEV;
	}

	if (!via_dev->write_ready) {
		spin_unlock(&via_dev->write_lock);
		return 0;	/* A white lie */
	}

	wbn = via_dev->write_current;
	if (!via_dev->wb[wbn].use)
	{
        dbg("All wb writted, no used left, return 0 directly");
		spin_unlock(&via_dev->write_lock);
		return 0;
	}
	wb = &via_dev->wb[wbn];

	via_dev->write_ready = 0;
	spin_unlock(&via_dev->write_lock);

	via_dev->write_urb->transfer_buffer = wb->buf;
	via_dev->write_urb->transfer_dma = wb->dmah;
	via_dev->write_urb->transfer_buffer_length = wb->len;
	via_dev->write_urb->dev = via_dev->dev;

	if ((rc = usb_submit_urb(via_dev->write_urb, GFP_ATOMIC)) < 0) {
		dbg("usb_submit_urb(write bulk) failed: %d", rc);
		via_write_done(via_dev);
	}
    dbg("Leave via_write_start");
	return rc;
}
/*****************************************************
 * Function Name:VIAUSBModem_write
 * Input:       1)tty: A pointer of struct tty_struct 
 *               2) buffer: pointer to writing buffer
 *                3)count: byte counts of writing buffer
 * Output:      1)None
 * Ret value:   1)if usb_submit_urb() failed then return the error status, else return the writed byte count.
 * Description: tty driver's write dispatch function, to prepare the write buffer end call starting write function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_write(struct tty_struct *tty,const unsigned char *buffer,int count)
{
	struct VIAUSBModem_Dev *via_dev=NULL;
	int wbn;
	int stat;
	struct VIAUSBModem_wb *wb=NULL;
/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(tty == NULL)
    {
        dbg("Error, tty is NULL");
        return -EINVAL;
    }
/* End of added by Guan Shibo, 2007-10-08 */

	dbg("VIAUSBModem_write:Enter.\n");
	if (!count)
	{
		return 0;
	}
    dbg("Write in <%d> bytes", count);
    debug_buf_content(buffer, count);

	/*check input parameters*/
	via_dev = tty->driver_data;
    if(!(VIA_READY(via_dev)))
	{
		dbg("VIAUSBModem_write: tty->driver_data is NULL.\n");
		return -EINVAL;
	}

	/*check if there is free write buffer.*/
	spin_lock(&via_dev->write_lock);
	
	wbn = wb_alloc(via_dev);
	if (wbn<0)
	{
		spin_unlock(&via_dev->write_lock);
		via_write_start(via_dev);
		return 0;
	}
	
	
	/*copy data from input buffer to write urb buffer*/
	wb = &via_dev->wb[wbn];
	count = (count > via_dev->writesize) ? via_dev->writesize : count;
    memcpy(wb->buf, buffer, count);
    wb->len = count;

	spin_unlock(&via_dev->write_lock);

	/*start to send write URB.*/
	if ((stat = via_write_start(via_dev)) < 0)
		return stat;
	return count;
	
}
/*****************************************************
 * Function Name: VIAUSBModem_write_room
 * Input:       1)tty: A pointer of struct tty_struct 
 * Output:      1)None
 * Ret value:   1)return the write buffer size or 0.
 * Description: tty write_room dispatch function, to answer the tty core if there are enough buffer for the next writing.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_write_room(struct tty_struct *tty)
{
	struct VIAUSBModem_Dev *via_dev = NULL;
	int ret = 0;


/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(tty == NULL)
    {
        dbg("Error, tty is NULL");
        return -EINVAL;
    }
/* End of added by Guan Shibo, 2007-10-08 */

    via_dev = tty->driver_data;
	dbg("VIAUSBModem_write_room:Enter.");
	if (!VIA_READY(via_dev))
    {
		return -EINVAL;
    }
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	ret =  (via_dev->write_ready && via_wb_is_avail(via_dev)) ? via_dev->writesize : 0;
	dbg("write_ready = <%d>", via_dev->write_ready);
	dbg("the room size= <%d>", ret);	
	dbg("VIAUSBModem_write_room:Exit.");
	return ret;
}
/*****************************************************
 * Function Name:VIAUSBModem_chars_in_buffer
 * Input:       1)tty: A pointer of struct tty_struct 
 * Output:      1)None
 * Ret value:   1) return the byte count in our buffer ?
 * Description: tty driver's chars_in_buffer dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_chars_in_buffer(struct tty_struct *tty)
{
	struct VIAUSBModem_Dev *via_dev = NULL;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if(tty == NULL)
    {
        dbg("Error, tty is NULL");
        return -EINVAL;
    }
/* End of added by Guan Shibo, 2007-10-08 */

    via_dev = tty->driver_data;
	dbg("VIAUSBModem_chars_in_buffer:Enter.");
	
	if (!VIA_READY(via_dev))
    {
		return -EINVAL;
    }
	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	
	dbg("VIAUSBModem_chars_in_buffer:Exit.");
	
	return (VIAUSBMODEM_NW - via_wb_is_avail(via_dev)) * via_dev->writesize;
}

/*****************************************************
 * Function Name:VIAUSBModem_ioctl
 * Input:       1)tty: A pointer of struct tty_struct 
 *                2)file: pointer to struct file 
 *                3)cmd:ioctl command
 *                 4)arg:ioctl command arguments
 * Output:      1)None
 * Ret value:   1)
 * Description: tty driver's ioctl dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_ioctl(struct tty_struct *tty,struct file *file,unsigned int cmd,unsigned long arg)
{
	//struct VIAUSBModem_Dev *via_dev = tty->driver_data;
	//void __user *uarg = (void __user *)arg;
        //struct ktermios	newTermios;
	int ret = -ENOIOCTLCMD;
	
/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((tty == NULL) || (file == NULL))
    {
        dbg("Error, tty or file is NULL");
        return -EFAULT;
    }
/* End of added by Guan Shibo, 2007-10-08 */
	
	dbg("VIAUSBModem_ioctl:Enter ..............");
	
	dbg("The input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
	/*BUG_ON(!kernel_locked());
	switch(cmd)
	{
		case TCGETS:
			dbg("VIAUSBModem_ioctl:TCGETS, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			if (copy_to_user(uarg,&via_dev->curTermios,sizeof(via_dev->curTermios)))
			{
			    dbg("VIAUSBModem_ioctl:TCGETS failed.");
			    ret = -EFAULT;
			}
			else
			   ret = 0;
			break;
		case TCSETS:
			dbg("VIAUSBModem_ioctl:TCSETS, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			if (copy_from_user(&newTermios, uarg, sizeof(newTermios)))
			    ret = -EFAULT;
			else
			    ret = 0;      
			break;
		default:
			break;
	}*/
	/*
	 * These ioctls don't rely on the hardware to be present.
	 */
	/*switch (cmd) 
	{
		case TIOCGSERIAL:
			//ret = uart_get_info(state, uarg);
			dbg("VIAUSBModem_ioctl:TIOCGSERIAL, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;

		case TIOCSSERIAL:
			//ret = uart_set_info(state, uarg);
			dbg("VIAUSBModem_ioctl:TIOCSSERIAL, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;

		case TIOCSERCONFIG:
			//ret = uart_do_autoconfig(state);
			dbg("VIAUSBModem_ioctl:TIOCSERCONFIG, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;

		//case TIOCSERGWILD: // obsolete 
		//case TIOCSERSWILD: // obsolete 
		//ret = 0;
		default:
		break;
	}*/

	/*if (ret != -ENOIOCTLCMD)
		goto out;

	if (tty->flags & (1 << TTY_IO_ERROR)) 
	{
		ret = -EIO;
		goto out;
	}*/

	/*
	 * The following should only be used when hardware is present.
	 */
	/*switch (cmd) 
	{
		case TIOCMIWAIT:
			//ret = uart_wait_modem_status(state, arg);
			dbg("VIAUSBModem_ioctl:TIOCMIWAIT, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;

		case TIOCGICOUNT:
			//ret = uart_get_count(state, uarg);
			dbg("VIAUSBModem_ioctl: TIOCGICOUNT, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;
	}

	if (ret != -ENOIOCTLCMD)
		goto out;

	*/
	/*
	 * All these rely on hardware being present and need to be
	 * protected against the tty being hung up.
	 */
	/*switch (cmd) 
	{
		case TIOCSERGETLSR: // Get line status register 
			//ret = uart_get_lsr_info(state, uarg);
			dbg("VIAUSBModem_ioctl: TIOCSERGETLSR, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;

		default: 
			{
	//		struct uart_port *port = state->port;TIOCSERGETLSR
	//		if (port->ops->ioctl)
	//			ret = port->ops->ioctl(port, cmd, arg);
			dbg("VIAUSBModem_ioctl: default, input CMD=<0x%x>, ARG=<0x%lx>", cmd, arg);
			break;
			}
	}*/

 //out:
	dbg("VIAUSBModem_ioctl:Exit ..............");	
	return ret;
	
}
/*****************************************************
 * Function Name:VIAUSBModem_tiocmset
 * Input:       1)tty: A pointer of struct tty_struct 
 *                2)file: pointer to struct file 
 *                3)set:tiocmset set 
 *                 4)clear:tiocmset clear
 * Output:      1)None
 * Ret value:   1)
 * Description: tty driver's tiocmset dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_tiocmset(struct tty_struct *tty,struct file *file,unsigned int set,unsigned int clear)
{
	struct VIAUSBModem_Dev *via_dev = NULL;
	unsigned int mcr = 0;
	
	dbg("VIAUSBModem_tiocmset: Enter  ..............");

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((tty == NULL) || (file == NULL) || (tty->driver_data == NULL))
    {
        dbg("Error, tty ,file or via_dev is NULL");
        return -EFAULT;
    }

    via_dev = tty->driver_data;

	if (!VIA_READY(via_dev))
    {
        return -EINVAL;
    }

    mcr = via_dev->mcr;
/* End of added by Guan Shibo, 2007-10-08 */

	if (set & TIOCM_RTS)
	{
		dbg("Now set TIOCM_RTS is received.");
		mcr |= VIA_CTRL_RTS;
	}
	if (set & TIOCM_DTR)
	{
		dbg("Now set TIOCM_DTR is received.");
		mcr |= VIA_CTRL_RTS;
		//send set DTR command to CBP.
		//VIAUSBMODEM_SET_DTR(via_dev);
		VIAUSBMODEM_SET_DTR(via_dev);
	}

	if (clear & TIOCM_RTS)
	{
		dbg("Now clear TIOCM_RTS is received.");
		mcr &= ~VIA_CTRL_RTS;
	}
	if (clear & TIOCM_DTR)
	{
		dbg("Now clear TIOCM_DTR is received.");
		mcr &= ~VIA_CTRL_RTS;
		// send clear DTR command to CBP.
		VIAUSBMODEM_CLR_DTR(via_dev);
	}

	/* set the new MCR value in the device */
	via_dev->mcr = mcr;

	dbg("VIAUSBModem_tiocmset: Exit  ..............");
	return 0;
}
/*****************************************************
 * Function Name:VIAUSBModem_tiocmget
 * Input:       1)tty: A pointer of struct tty_struct 
 *                2)file: pointer to struct file 
 * Output:      1)None
 * Ret value:   1)
 * Description: tty driver's tiocmget dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
static int VIAUSBModem_tiocmget(struct tty_struct *tty,struct file *file)
{
	struct VIAUSBModem_Dev *via_dev = NULL;
	unsigned int mcr = 0;
    unsigned int msr = 0;
    int result = 0;
	
/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((tty == NULL) || (file == NULL) || (tty->driver_data == NULL))
    {
        dbg("Error, tty ,file or via_dev is NULL");
        return -EFAULT;
    }
    via_dev = tty->driver_data;
    if (!VIA_READY(via_dev))
    {
        return -EINVAL;
    }

/* End of added by Guan Shibo, 2007-10-08 */

    mcr = via_dev->mcr;
    msr = via_dev->msr;

	dbg("VIAUSBModem_tiocmget: Enter ..............");
	result = ((mcr & VIA_CTRL_DTR)  ? TIOCM_DTR  : 0) |	// DTR is set 
		((mcr & VIA_CTRL_RTS)  ? TIOCM_RTS  : 0) |	/* RTS is set */
             		0|//((mcr & MCR_LOOP) ? TIOCM_LOOP : 0) |	/* LOOP is set */
		0|//((msr & MSR_CTS)  ? TIOCM_CTS  : 0) |	/* CTS is set */
             		((msr & VIA_CTRL_DCD)   ? TIOCM_CAR  : 0) |	/* Carrier detect is set*/
             		((msr & VIA_CTRL_RI)   ? TIOCM_RI   : 0) |	/* Ring Indicator is set */
             		((msr & VIA_CTRL_DSR)  ? TIOCM_DSR  : 0);	/* DSR is set */

	dbg("VIAUSBModem_tiocmget: Exit............");
	return result;
}

void VIAUSBModem_break_ctrl(struct tty_struct *tty,int state)
{
	dbg("Enter VIAUSBModem_break_ctrl ..............");
	
	dbg("Leave VIAUSBModem_break_ctrl ..............");
	return;
}
/*****************************************************
 * Function Name:VIAUSBModem_throttle
 * Input:       1)tty: A pointer of struct tty_struct 
 * Output:      1)None
 * Ret value:   1)
 * Description: tty driver's throttle dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
void VIAUSBModem_throttle(struct tty_struct *tty)
{
	dbg("Enter VIAUSBModem_throttle ..............");
	
	dbg("Leave VIAUSBModem_throttle ..............");
	return;
}
/*****************************************************
 * Function Name:VIAUSBModem_unthrottle
 * Input:       1)tty: A pointer of struct tty_struct 
 * Output:      1)None
 * Ret value:   1)
 * Description: tty driver's unthrottle dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
void VIAUSBModem_unthrottle(struct tty_struct *tty)
{
	dbg("Enter VIAUSBModem_unthrottle ..............");
	
	dbg("Leave VIAUSBModem_unthrottle ..............");
	return;
}

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))
/*****************************************************
 * Function Name:VIAUSBModem_set_termios
 * Input:       1)tty: A pointer of struct tty_struct 
 *                2)tios_old: old ktermios
 * Output:      1)None
 * Ret value:   1)
 * Description: tty driver's set_termios dispatch function.
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:   
 *      Desciption:    
 *
 ******************************************************************/
/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__)
void VIAUSBModem_set_termios(struct tty_struct *tty,struct ktermios *tios_old)
#else
void VIAUSBModem_set_termios(struct tty_struct *tty, struct termios *tios_old)    
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
{
    struct VIAUSBModem_Dev *via_dev = NULL;
/* Modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */
#if defined(__VIA_KVER_GE_2620__)
	struct ktermios *termios = NULL; 
#else
    struct termios *termios = NULL;
#endif
/* End of modified by Guan Shibo, for the multiple kernel version support, 2007-10-22 */

unsigned int cflag = 0;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((tty == NULL) || (tty->termios == NULL))
    {
        dbg("Error, tty or termios is NULL");
        return ;
    }
    via_dev = tty->driver_data;
    if (!VIA_READY(via_dev))
    {
        return;
    }

/* End of added by Guan Shibo, 2007-10-08 */
    termios = tty->termios;
    cflag = termios->c_cflag;

	dbg("CFLAG value,   old value=<0x%x>,     new vlaue=<0x%x>", tios_old->c_cflag, tty->termios->c_cflag);
	dbg("IFLAG value,   old value=<0x%x>,     new vlaue=<0x%x>", tios_old->c_iflag, tty->termios->c_iflag);
	dbg("OFLAG value,   old value=<0x%x>,     new vlaue=<0x%x>", tios_old->c_oflag, tty->termios->c_oflag);
	dbg("LFLAG value,   old value=<0x%x>,     new vlaue=<0x%x>", tios_old->c_lflag, tty->termios->c_lflag);
	dbg("CLINE value,   old value=<0x%x>,     new vlaue=<0x%x>", tios_old->c_line, tty->termios->c_line);
	
	// check that they really want us to change some setting.
	if(tios_old != NULL)
	{
		if ((cflag == tios_old->c_cflag) &&
		    (RELEVANT_IFLAG(termios->c_iflag) == 
		     RELEVANT_IFLAG(tios_old->c_iflag))) 
		{
			dbg("VIAUSBModem_set_termios:nothing to change...");
			return;
		}
	}
	/*switch(cflag)
	{
	}*/
		
	dbg("VIAUSBModem_set_termios:Leave VIAUSBModem_set_termios ..............");	
	return;
}
/*****************************************************
 * Function Name:VIAUSBModem_close
 * Input:       1)tty:  A pointer of tty struct
 *              2)filp: A pointer of the file struct
 * Output:      1)None
 * Ret value:   1)None
 * Description: Do the clean up operation when the device be closed. 
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-10
 *
 * Modified history:
 * 1)   Modified by:    Guan Shibo, 2007-09-18
 *      Desciption:     Add the poll_status_urb cancel operation
 *
 ******************************************************************/

static void VIAUSBModem_close(struct tty_struct *tty, struct file *filp)
{
	struct VIAUSBModem_Dev *via_dev = NULL;

/* Added by Guan Shibo for parameters check, 2007-10-08 */
    if((tty == NULL) || (filp == NULL))
    {
        dbg("Error, tty or filp is NULL");
        return ;
    }
/* End of added by Guan Shibo, 2007-10-08 */
    via_dev = tty->driver_data;

	dbg("Enter VISUSBModem_close...............");
	
	if (!via_dev || !via_dev->used)
	{
		dbg("Maybe dev is null or not be used");
		return;
	}

	dbg("This device be used <%d> times", via_dev->used);
	mutex_lock(&open_mutex);
	if (!--via_dev->used)
	{
		dbg("The last instance be closed");
		if (via_dev->dev)
		{
			//Add some setup communication here if you need
			usb_kill_urb(via_dev->read_urb);
			usb_kill_urb(via_dev->write_urb);
            usb_kill_urb(via_dev->poll_status_urb);	/* Cancel the poll_status URB, added by Guan Shibo, 2007-09-18 */
		} 
		else
		{
			VIAUSBModem_tty_unregister(via_dev);
		}
	}
	mutex_unlock(&open_mutex);
	
	return;
}
/*****************************************************
 * Function Name:VIAUSBModem_open
 * Input:       1)tty:  A pointer of tty struct
 *              2)filp: A pointer of the file struct
 * Output:      1)None
 * Ret value:   1)If success, return 0, otherwise ,return a negative value
 * Description: Whe the userspace open device, the core will call this method. We do some initialization here and schedule the work queue.
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-10
 *
 * Modified history:
 * 1)   Modified by:    Guan Shibo, 2007-09-18
 *      Desciption:     Add the starting of poll_status work schedule 
 *
 ******************************************************************/
static int VIAUSBModem_open(struct tty_struct *tty, struct file *filp)
{
	struct VIAUSBModem_Dev *via_dev;
	int ret = -EINVAL;
    
    char buf = 0;    /* Added by Guan Shibio, for CR=HCT#960, 2007-10-18 */

	dbg("Entering VISUSBmodem_open.............");

	mutex_lock(&open_mutex);

	via_dev = VIAUSBModem_table[tty->index];
	if (!via_dev || !via_dev->dev)
	{
		dbg("via_dev or his member <dev> is NULL");
		goto err_out;
	}

	tty->driver_data = via_dev;
	via_dev->tty = tty;

	/* force low_latency on so that our tty_push actually forces the data through,
	   otherwise it is scheduled, and with high data rates data can get lost. */
	tty->low_latency = 1;

	if (via_dev->used++) 
    {
        dbg("Not the first open, don't need schedule the work_inter_read");
        goto done;
    }

	/* Added by Guan Shibio, for CR=HCT#960, 2007-10-18 */
    if(usb_control_msg(via_dev->dev, usb_rcvctrlpipe(via_dev->dev, 0), 0x2, 
            0xc0, 0, 0, &buf, 1, USB_CTRL_GET_TIMEOUT) == 1)
    {
        if(buf & 0x1)
        {
            dbg("CD high");
            via_dev->cd_high = 1;
        }
    }
    else
    {
        dbg("get status error");
        goto err_out;
    }
    ret = 0;
	/* End of added by Guan Shibo, fro CR=HCT#960, 2007-10-18 */

	/*init sppin lock*/
	spin_lock_init(&via_dev->write_lock);
    spin_lock_init(&via_dev->throttle_lock);
	
	/*send DTR signal*/
	dbg("Now clear and set DTR signal.");
	VIAUSBMODEM_SET_DTR(via_dev);
	
	dbg("It's first open, schedule the work_inter_read");
	via_dev->throttle = 0;

	schedule_work(&via_dev->work_inter_read);

    /*Modified by Guan Shibo, for slow the poll status, 2007-10-26*/
    schedule_delayed_work(&via_dev->work_poll_status, HZ/10);	/* Start the schedule of work_poll_status, added by Guan Shibo, 2007-09-18 */
    /*End of modified by Guan Shibo, for slow the poll status, 2007-10-26*/

done:
err_out:
	mutex_unlock(&open_mutex);
  	return ret;
}


struct tty_operations VIAUSBModem_ops = {
	.open =		VIAUSBModem_open,
	.close =		VIAUSBModem_close,
	.write =		VIAUSBModem_write,
	.write_room =		VIAUSBModem_write_room,
	.ioctl =		VIAUSBModem_ioctl,
	.throttle =		VIAUSBModem_throttle,
	.unthrottle =		VIAUSBModem_unthrottle,
	.chars_in_buffer =	VIAUSBModem_chars_in_buffer,
	.break_ctl =		VIAUSBModem_break_ctrl,
	.set_termios =		VIAUSBModem_set_termios,
	.tiocmget =		VIAUSBModem_tiocmget,
	.tiocmset =		VIAUSBModem_tiocmset,
};



/*****************************************************
 * Function Name:VIAUSBModem_init
 * Input:       1)none
 * Output:      1)None
 * Ret value:   1)
 * Description: VIA usb modem's initialize module function
 *
 * Author:      Guan Shibo
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:  
 *      Desciption:    
 *
 ******************************************************************/
static int __init VIAUSBModem_init(void)
{
	int retval;
	
	dbg("Enter init................................");
	
	VIAUSBModem_tty_driver = alloc_tty_driver(VIAUSBModem_MINORS);
	
	dbg("tty_driver alloc, pointer= %p", VIAUSBModem_tty_driver);
	
	if (!VIAUSBModem_tty_driver )
		return -ENOMEM;
	VIAUSBModem_tty_driver->owner = THIS_MODULE,
	VIAUSBModem_tty_driver->driver_name = "VIAUSBModem",
	VIAUSBModem_tty_driver ->name = "VIAUSBModem",
	VIAUSBModem_tty_driver ->major = VIAUSBModem_MAJOR,
	VIAUSBModem_tty_driver->minor_start = 0,
	VIAUSBModem_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	VIAUSBModem_tty_driver->subtype = SERIAL_TYPE_NORMAL,
#if defined(__VIA_KVER_EQ_2615__)
	VIAUSBModem_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_NO_DEVFS;
#else
	VIAUSBModem_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
#endif
	VIAUSBModem_tty_driver->init_termios = tty_std_termios;
	VIAUSBModem_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(VIAUSBModem_tty_driver, &VIAUSBModem_ops);

	retval = tty_register_driver(VIAUSBModem_tty_driver);
	if (retval) {
		dbg("tty_register_driver failed");
		put_tty_driver(VIAUSBModem_tty_driver);
		return retval;
	}

	retval = usb_register(&VIAUSBModem_driver);
	if (retval) {
		dbg("usb_register failed");
		tty_unregister_driver(VIAUSBModem_tty_driver);
		put_tty_driver(VIAUSBModem_tty_driver);
		return retval;
	}

	printk(DRIVER_VERSION ":" DRIVER_DESC);
	dbg("Leave init successful.....................");
	return 0;
}
/*****************************************************
 * Function Name:VIAUSBModem_exit
 * Input:       1)none
 * Output:      1)None
 * Ret value:   1)
 * Description: VIA usb modem's exit module function
 *
 * Author:      hjiang
 * Create data: 2007-09-24
 *
 * Modified history:
 * 1)   Modified by:  
 *      Desciption:    
 *
 ******************************************************************/
static void __exit VIAUSBModem_exit(void)
{
	usb_deregister(&VIAUSBModem_driver);
	tty_unregister_driver(VIAUSBModem_tty_driver);
	put_tty_driver(VIAUSBModem_tty_driver);
	dbg("VIA driver exit");
}
module_init(VIAUSBModem_init);
module_exit(VIAUSBModem_exit);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

