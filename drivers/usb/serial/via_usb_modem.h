/********************************************************
Author:		Guan Shibo
Time:		2007-09-10
File Name:	VIAUSBModem.h
Description:Header file for the init.c

Copyright:  (c) 2002-2009 VIA TELECOM Corporation
            All Rights Reserved
*****************************************************/

/*
 * CMSPAR, some architectures can't have space and mark parity.
 */

#ifndef CMSPAR
#define CMSPAR			0
#endif

/*
 * Major and minor numbers.
 */

#define VIAUSBModem_MAJOR		167
#define VIAUSBModem_MINORS		32

/*
 * Requests.
 */

//#define USB_RT_ACM		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define	VIAUSBMODEM_CONTROL	0x40
#define	VIAUSBMODEM_POLL_CD	0xc0

/*
 * Output control lines.
 */
#define	VIAUSBMODEM_DTR		0x01
#define	VIAUSBMODEM_POLLCD	0x02

#define VIA_CTRL_DTR		0x01
#define VIA_CTRL_RTS		0x02

/*
 * Input control lines and line errors.
 */

#define VIA_CTRL_DCD		0x01
#define VIA_CTRL_DSR		0x02
#define VIA_CTRL_BRK		0x04
#define VIA_CTRL_RI		0x08

#define VIA_CTRL_FRAMING	0x10
#define VIA_CTRL_PARITY		0x20
#define VIA_CTRL_OVERRUN	0x40

/*added by shibo guan*/
#define EP_CTRL	0
#define EP_BULK_READ	0x81
#define EP_BULK_WRITE	0x01
/*end- shibo guan*/

/*
 * Internal driver structures.
 */

/*
 * The only reason to have several buffers is to accomodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define VIAUSBMODEM_NW  2
//#define ACM_NR  16

struct VIAUSBModem_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;	//write pipe max size
	int use;	//1 used£¬0 free
};

/*struct acm_rb {
	struct list_head	list;
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
};

struct acm_ru {
	struct list_head	list;
	struct acm_rb		*buffer;
	struct urb		*urb;
	struct acm		*instance;
};*/
/* VIA protocol command struct, added by Guan Shibo, 2007-09-19 */
struct via_usb_cmd
{
    unsigned char ucRequestType;
    unsigned char ucRequest;
    unsigned short usValue;
    unsigned short usIndex;
    unsigned short usLength;
};
/* End of added by Guan Shibo, 2007-09-19 */

struct VIAUSBModem_Dev{
	struct usb_device *dev;			/* the corresponding usb device */
	struct usb_interface *interface;			/* control interface */
	//struct usb_interface *data;			/* data interface */
	struct tty_struct *tty;			/* the corresponding tty */
	struct work_struct work_inter_read;		//internal read data work struct
	//struct timer_list	tmReadModemStatus;// this timer is used to read the  modem status by control pipe.

    /*Added by Guan Shibo, 2007-10-26*/
#if defined(__VIA_KVER_GE_2620__) 
    struct delayed_work work_poll_status;	/* This work used to poll the status of VIA usb modem, added by Guanshibo, 2007-09-18 */
#else
    struct work_struct work_poll_status;	/* This work used to poll the status of VIA usb modem, added by Guanshibo, 2007-09-18 */
#endif
    /*End of added by Guan Shibo, 2007-10-26*/

	struct urb *read_urb, *write_urb;			/* urbs */
    struct urb *set_dtr_urb;		//used to SET DTR
    struct via_usb_cmd ctrl_cmd;	// used to fill control transfer command e.g. SET_DTR
    struct urb *poll_status_urb;	/* Polling the modem status via control pipe ,added by Guan Shibo, 2007-09-18*/
    struct via_usb_cmd poll_cd_cmd;	/* The command define of poll_cd, added by Guan Shibo, 2007-09-19 */
    unsigned char msr;	/* Save the poll_cd result, added by Guan Shibo, 2007-09-19 */
	unsigned char *ctrl_buffer;				/* buffers of ctrl urb, for setup packet */
	//dma_addr_t ctrl_dma;				/* dma handles of buffers */
	dma_addr_t 	read_dma;
    dma_addr_t  poll_status_dma;	/* dma for the polling status URB, added by Guan Shibo, 2007-09-18*/
	char *read_buffer;
    char *poll_status_buffer;	/* Buffer for the polling status URB, added by Guan Shibo, 2007-09-18*/
	struct VIAUSBModem_wb wb[VIAUSBMODEM_NW];
	//int rx_buflimit;
	unsigned int rx_endpoint;
	unsigned int tx_endpoint;
	unsigned int ctrl_endpoint;
	__u8	rx_epAddress;
	__u8	tx_epAddress;
	__u8	ctrl_epAddress;
	spinlock_t read_lock;
	//struct list_head spare_read_urbs;
	//struct list_head spare_read_bufs;
	//struct list_head filled_read_bufs;
	int write_current;				/* current write buffer */
	//int write_used;				/* number of non-empty write buffers */
	int write_ready;				/* write urb is not running */
	spinlock_t write_lock;
	//struct usb_cdc_line_coding line;		/* bits, stop, parity */
	struct work_struct work;			/* work queue entry for line discipline waking up */
	//struct tasklet_struct urb_task;                 /* rx processing */
	spinlock_t throttle_lock;			/* synchronize throtteling and read callback */
	unsigned int ctrlin;				/* input control lines (DCD, DSR, RI, break, overruns) */
	unsigned int ctrlout;				/* output control lines (DTR, RTS) */
	unsigned int writesize;				/* max packet size for the output bulk endpoint */
	unsigned int readsize,ctrlsize;			/* buffer sizes for freeing */
	unsigned int used;				/* someone has this acm's device open */
	unsigned int minor;				/* acm minor number */
	unsigned char throttle;				/* throttled by tty layer */
	unsigned char clocal;				/* termios CLOCAL */
	unsigned int ctrl_caps;				/* control capabilities from the class specific header */
    unsigned int cd_high;               /*Added by Guan Shibo, for CR=HT#960, 2007-10-18*/
	
	//int msr;					/*Modem's MSR shadow*/
	int mcr;					/*Modem's MCR shadow*/

	//struct work_struct work_inter_read;			/*internal read data work_struct*/
};

#define CDC_DATA_INTERFACE_TYPE	0x0a

/* constants describing various quirks and errors */
#define NO_UNION_NORMAL			1
#define SINGLE_RX_URB			2

/*added by guan*/
#ifdef VIA_DEBUG
#undef dbg
#define dbg(format, arg...) printk(KERN_DEBUG "<VIAUSBModem>-in<%s> " format "\n" , __FUNCTION__,## arg)
#else
#undef dbg
#define dbg(format, arg...) do {} while (0)
#endif

/*end of added by guan*/

