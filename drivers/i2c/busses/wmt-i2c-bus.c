/*++
	drivers/i2c/busses/wmt_i2c_bus.c

	Some descriptions of such software. Copyright (c) 2008 WonderMedia Technologies, Inc.

	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.

	WonderMedia Technologies, Inc.
	10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/

/*++    History :

	This code was inherit VT8430

	2009/01/16 DeanHsiao First version

--*/
/* Include your headers here*/
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>

#ifdef __KERNEL__

#ifdef DEBUG
	#define DPRINTK		printk
#else
	#define DPRINTK(x...)
#endif

#else
	#define DPRINTK    	printf

#endif


#define MAX_BUS_READY_CNT		50	  /* jiffy*/
#define MAX_TX_TIMEOUT			500   /* ms*/
#define MAX_RX_TIMEOUT			500	  /* ms*/
#define GPIO_CTRL_SD_IIC_SPI_MS         0xd8110004
#define PMC_ClOCK_ENABLE_LOWER          0xd8130250

struct wmt_i2c_s {
	struct i2c_regs_s *regs;
	int irq_no ;
	enum i2c_mode_e i2c_mode ;
	int volatile isr_nack ;
	int volatile isr_byte_end ;
	int volatile isr_timeout ;
	int volatile isr_int_pending ;
};

static int i2c_wmt_wait_bus_not_busy(void);

/**/
/*  variable*/
/*-------------------------------------------------*/
static volatile struct wmt_i2c_s i2c ;

DECLARE_WAIT_QUEUE_HEAD(i2c_wait);
spinlock_t i2c_wmt_irqlock = SPIN_LOCK_UNLOCKED;
/*!*************************************************************************
* i2c_wmt_set_mode()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief set transfer mode (Fast/Standard/HS)
*
* \retval  NULL
*/
void i2c_wmt_set_mode(enum i2c_mode_e mode  /*!<; //[IN] mode */)
{
	i2c.i2c_mode = mode ;

	if (i2c.i2c_mode == I2C_STANDARD_MODE)  {
		DPRINTK("I2C: set standard mode \n");
		i2c.regs->tr_reg = I2C_TR_STD_VALUE ;   /* 0x8041*/
	} else if (i2c.i2c_mode == I2C_FAST_MODE) {
		DPRINTK("I2C: set fast mode \n");
		i2c.regs->tr_reg = I2C_TR_FAST_VALUE ; /* 0x8011*/
	}
}


/*!*************************************************************************
* i2c_wmt_read_msg()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Read message from slave
*
* \retval 0 if success
*/
int i2c_wmt_read_msg(
	unsigned int slave_addr, 	/*!<; //[IN] Salve address */
	char *buf, 				/*!<; //[OUT] Pointer to data */
	unsigned int length,		/*!<; //Data length */
	int restart, 				/*!<; //Need to restart after a complete read */
	int last 					/*!<; //Last read */
)
{
	unsigned short tcr_value ;
	unsigned int xfer_length ;
	int is_timeout ;
	int ret  = 0 ;
	int wait_event_result  = 0 ;

	if (length <= 0)
		return -1 ;
	xfer_length = 0 ;

	if (restart == 0)
		ret = i2c_wmt_wait_bus_not_busy()  ;
	if (ret < 0)
		return ret ;

	i2c.isr_nack    	= 0 ;
	i2c.isr_byte_end	= 0 ;
	i2c.isr_timeout 	= 0 ;
	i2c.isr_int_pending = 0;

	i2c.regs->cr_reg &= ~(I2C_CR_TX_NEXT_NO_ACK); /*clear NEXT_NO_ACK*/
	if (restart == 0)
		i2c.regs->cr_reg |= (I2C_CR_CPU_RDY); /*release SCL*/

	tcr_value = 0 ;
	if (i2c.i2c_mode == I2C_STANDARD_MODE)  {
		tcr_value = (unsigned short)(I2C_TCR_STANDARD_MODE|I2C_TCR_MASTER_READ |\
				(slave_addr & I2C_TCR_SLAVE_ADDR_MASK)) ;
	} else if (i2c.i2c_mode == I2C_FAST_MODE) {
		tcr_value = (unsigned short)(I2C_TCR_FAST_MODE|I2C_TCR_MASTER_READ |\
				(slave_addr & I2C_TCR_SLAVE_ADDR_MASK)) ;
	}
	i2c.regs->tcr_reg = tcr_value ;

	/*repeat start case*/
	if (restart == 1)
		i2c.regs->cr_reg |= (I2C_CR_CPU_RDY); /*release SCL*/

	if (length == 1)
		i2c.regs->cr_reg |= I2C_CR_TX_NEXT_NO_ACK; /*only 8-bit to read*/

	ret  = 0 ;
	for (; ;) {
		is_timeout = 0 ;
		wait_event_result = wait_event_interruptible_timeout(i2c_wait, i2c.isr_int_pending ,
			(MAX_RX_TIMEOUT * HZ / 1000)) ;
		if (likely(wait_event_result > 0)) {
			DPRINTK("I2C: wait interrupted (rx) \n");
			ret  = 0 ;
		} else if (likely(i2c.isr_int_pending == 0)) {
			DPRINTK("I2C: wait timeout (rx) \n");
			is_timeout = 1 ;
			ret = -ETIMEDOUT ;
		}

		/**/
		/* fail case*/
		/**/
		if (i2c.isr_nack == 1) {
			DPRINTK("i2c_err : write NACK error (rx) \n\r") ;
			ret = -EIO ;
			break ;
		}
		if (i2c.isr_timeout == 1) {
			DPRINTK("i2c_err : write SCL timeout error (rx)\n\r") ;
			ret = -ETIMEDOUT ;
			break ;
		}
		if (is_timeout == 1) {
			DPRINTK("i2c_err: write software timeout error (rx) \n\r") ;
			ret = -ETIMEDOUT ;
			break ;
		}


		/**/
		/* pass case*/
		/**/
		if (i2c.isr_byte_end == 1) {
			buf[xfer_length] = (i2c.regs->cdr_reg >> 8) ;
			++xfer_length ;
			DPRINTK("i2c_test: received BYTE_END\n\r");
		}
		i2c.isr_int_pending = 0;
		i2c.isr_nack    	= 0 ;
		i2c.isr_byte_end	= 0 ;
		i2c.isr_timeout 	= 0 ;

		if (length > xfer_length) {
			if ((length - 1) ==  xfer_length) { /* next read is the last one*/
				i2c.regs->cr_reg |= (I2C_CR_TX_NEXT_NO_ACK | I2C_CR_CPU_RDY);
				DPRINTK("i2c_test: set CPU_RDY & TX_ACK. next data is last.\r\n");
			} else {
				i2c.regs->cr_reg |= I2C_CR_CPU_RDY ;
				DPRINTK("i2c_test: more data to read. only set CPU_RDY. \r\n");
			}
		} else if (length == xfer_length) { /* end rx xfer*/
			if (last == 1) {  /* stop case*/
				DPRINTK("i2c_test: read completed \r\n");
				break ;
			} else {  /* restart case*/
				/* ??? how to handle the restart after read ?*/
				DPRINTK("i2c_test: RX ReStart Case \r\n") ;
				break ;
			}
		} else {
			DPRINTK("i2c_err : read known error\n\r") ;
			ret = -EIO ;
			break ;
		}
	}

	DPRINTK("i2c_test: read sequence completed\n\r");
	return ret ;
}

/*!*************************************************************************
* i2c_wmt_write_msg()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Write message to slave
*
* \retval 0 if success
*/
int i2c_wmt_write_msg(
	unsigned int slave_addr, 	/*!<; //[IN] Salve address */
	char *buf, 				/*!<; //[OUT] Pointer to data */
	unsigned int length,		/*!<; //Data length */
	int restart, 				/*!<; //Need to restart after a complete write */
	int last 					/*!<; //Last read */
)
{
	unsigned short tcr_value ;
	unsigned int xfer_length ;
	int is_timeout ;
	int ret = 0 ;
	int wait_event_result ;

	DPRINTK("length = %d , slave_addr = %x\n", length , slave_addr);
	if (slave_addr == WMT_I2C_API_I2C_ADDR)
		return ret ;

	/* special case allow length:0, for i2c_smbus_xfer*/
	/**/
	if (length < 0)
		return -1 ;
	xfer_length = 0 ; /* for array index and also for checking counting*/
	if (restart == 0)
		ret = i2c_wmt_wait_bus_not_busy()  ;
	if (ret < 0)
		return ret ;

	i2c.isr_nack    	= 0 ;
	i2c.isr_byte_end	= 0 ;
	i2c.isr_timeout 	= 0 ;
	i2c.isr_int_pending = 0;

	/**/
	/* special case allow length:0, for i2c_smbus_xfer*/
	/**/
	if (length == 0)
		i2c.regs->cdr_reg = 0 ;
	else
		i2c.regs->cdr_reg = (unsigned short)(buf[xfer_length] & I2C_CDR_DATA_WRITE_MASK)  ;

	if (restart == 0) {
		i2c.regs->cr_reg &= ~(I2C_CR_TX_END); /*clear Tx end*/
		i2c.regs->cr_reg |= (I2C_CR_CPU_RDY); /*release SCL*/
	}

	/**/
	/* I2C: Set transfer mode [standard/fast]*/
	/**/
	tcr_value = 0 ;
	if (i2c.i2c_mode == I2C_STANDARD_MODE)
		tcr_value = (unsigned short)(I2C_TCR_STANDARD_MODE|I2C_TCR_MASTER_WRITE |\
				(slave_addr & I2C_TCR_SLAVE_ADDR_MASK)) ;
	else if (i2c.i2c_mode == I2C_FAST_MODE)
		tcr_value = (unsigned short)(I2C_TCR_FAST_MODE|I2C_TCR_MASTER_WRITE |\
				(slave_addr & I2C_TCR_SLAVE_ADDR_MASK)) ;

	i2c.regs->tcr_reg = tcr_value ;

	if ( restart == 1 ) {
	    i2c.regs->cr_reg |= I2C_CR_CPU_RDY ;
	}

	ret  = 0 ;
	for (; ;) {

		is_timeout = 0 ;
		/**/
		/* I2C: Wait for interrupt. if ( i2c.isr_int_pending == 1 ) ==> an interrupt exsits.*/
		/**/
		wait_event_result = wait_event_interruptible_timeout(i2c_wait, i2c.isr_int_pending , (MAX_TX_TIMEOUT * HZ / 1000)) ;

		if (likely(wait_event_result > 0)) {
			DPRINTK("I2C: wait interrupted (tx)\n");
			ret  = 0 ;
		} else if (likely(i2c.isr_int_pending == 0)) {
			DPRINTK("I2C: wait timeout (tx) \n");
			is_timeout = 1 ;
			ret = -ETIMEDOUT ;
		}

		/**/
		/* fail case*/
		/**/
		if (i2c.isr_nack == 1) {
			DPRINTK("i2c_err : write NACK error (tx) \n\r") ;
			ret = -EIO ;
			break ;
		}
		if (i2c.isr_timeout == 1) {
			DPRINTK("i2c_err : write SCL timeout error (tx)\n\r") ;
			ret = -ETIMEDOUT ;
			break ;
		}
		if (is_timeout == 1) {
			DPRINTK("i2c_err : write software timeout error (tx)\n\r") ;
			ret = -ETIMEDOUT ;
			break ;
		}

		/**/
		/* pass case*/
		/**/
		if (i2c.isr_byte_end == 1) {
			DPRINTK("i2c: isr end byte (tx)\n\r") ;
			++xfer_length ;
		}
		i2c.isr_int_pending = 0 ;
		i2c.isr_nack    	= 0 ;
		i2c.isr_byte_end	= 0 ;
		i2c.isr_timeout 	= 0 ;


		if ((i2c.regs->csr_reg & I2C_CSR_RCV_ACK_MASK) == I2C_CSR_RCV_NOT_ACK) {
			DPRINTK("i2c_err : write RCV NACK error\n\r") ;
			ret = -EIO ;
			break ;
		}

		/**/
		/* special case allow length:0, for i2c_smbus_xfer*/
		/**/
		if (length == 0) {
			i2c.regs->cr_reg = (I2C_CR_TX_END|I2C_CR_CPU_RDY|I2C_CR_ENABLE) ;
			break ;
		}
		if (length > xfer_length) {
			i2c.regs->cdr_reg = (unsigned short) (buf[xfer_length] & I2C_CDR_DATA_WRITE_MASK) ;
			i2c.regs->cr_reg = (I2C_CR_CPU_RDY | I2C_CR_ENABLE) ;
			DPRINTK("i2c_test: write register data \n\r") ;
		} else if (length == xfer_length) { /* end tx xfer*/
			if (last == 1) {  /* stop case*/
				i2c.regs->cr_reg = (I2C_CR_TX_END|I2C_CR_CPU_RDY|I2C_CR_ENABLE) ;
				DPRINTK("i2c_test: finish write \n\r") ;
				break ;
			} else {  /* restart case*/
				/* handle the restart for first write then the next is read*/
				i2c.regs->cr_reg = (I2C_CR_ENABLE) ;
				DPRINTK("i2c_test: tx restart Case \n\r") ;
				break ;
			}
		} else {
			DPRINTK("i2c_err : write unknown error\n\r") ;
			ret = -EIO ;
			break ;
		}
	} ;

	DPRINTK("i2c_test: write sequence completed\n\r");

	return ret ;
}


/*!*************************************************************************
* i2c_wmt_wait_bus_not_busy
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
*
* \retval 0 if success
*/
static int i2c_wmt_wait_bus_not_busy(void)
{
	int ret ;
	int cnt ;

	DECLARE_WAITQUEUE(wait, current) ;
	add_wait_queue(&i2c_wait, &wait) ;

	ret = 0 ;
	cnt = 0 ;
	while (1) {
		if ((REG16_VAL(I2C_CSR_ADDR) & I2C_STATUS_MASK) == I2C_READY)
			break ;

		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current))
			break;

		schedule_timeout(1);
		cnt++ ;

		if (cnt > MAX_BUS_READY_CNT) {
			ret = (-EBUSY) ;
			printk("i2c_err : wait but not ready time-out\n\r") ;
			break;
		}
	};

	set_current_state(TASK_RUNNING);

	remove_wait_queue(&i2c_wait, &wait);

	return ret ;
}


/*!*************************************************************************
* i2c_wmt_reset()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Reset all back to default setting
*
* \retval  NULL
*/
static void i2c_wmt_reset(void)
{
	unsigned short tmp ;

	/**/
	/* software initial*/
	/**/
	i2c.regs        = (struct i2c_regs_s *)I2C0_BASE_ADDR ;
	i2c.irq_no      = IRQ_I2C0 ;
	i2c.i2c_mode    = I2C_FAST_MODE ;
	i2c.isr_nack    = 0 ;
	i2c.isr_byte_end = 0 ;
	i2c.isr_timeout = 0 ;
	i2c.isr_int_pending = 0;

	/**/
	/* hardware initial*/
	/**/



	i2c.regs->cr_reg  = 0 ;
	i2c.regs->div_reg = APB_96M_I2C_DIV ;
	i2c.regs->isr_reg = I2C_ISR_ALL_WRITE_CLEAR ;   /* 0x0007*/
	i2c.regs->imr_reg = I2C_IMR_ALL_ENABLE ;        /* 0x0007*/

	i2c.regs->cr_reg  = I2C_CR_ENABLE ;
	tmp = i2c.regs->csr_reg ;                     /* read clear*/
	i2c.regs->isr_reg = I2C_ISR_ALL_WRITE_CLEAR ; /* 0x0007*/

	if (i2c.i2c_mode == I2C_STANDARD_MODE)
		i2c.regs->tr_reg = I2C_TR_STD_VALUE ;   /* 0x8041*/
	else if (i2c.i2c_mode == I2C_FAST_MODE)
		i2c.regs->tr_reg = I2C_TR_FAST_VALUE ; /* 0x8011*/

	DPRINTK("Resetting I2C Controller Unit\n");

	return ;
}

/*!*************************************************************************
* i2c_wmt_handler()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Check isr_reg status, then clear  i2c.isr_int_pending
*
* \retval  IRQ_HANDLED
*/
static irqreturn_t i2c_wmt_handler(
	int this_irq, 			/*!<; //[IN] IRQ number */
	void *dev_id 		/*!<; //[IN] Pointer to device ID */
)
{

	int wakeup ;
	unsigned short isr_status ;
	unsigned short tmp ;
	unsigned long flags;

	isr_status = i2c.regs->isr_reg ;
	wakeup = 0 ;

	/*
	* CPU_RDY did not auto clear with 3426 A0
	* it would be fix up.
	*/
	/*i2c.regs->cr_reg = ~(I2C_CR_CPU_RDY) ;*/

	if (isr_status & I2C_ISR_NACK_ADDR) {
		i2c.regs->isr_reg = I2C_ISR_NACK_ADDR_WRITE_CLEAR ;
		tmp = i2c.regs->csr_reg ;  /* read clear*/
		i2c.isr_nack = 1 ;
		wakeup = 1 ;
	}

	if (isr_status & I2C_ISR_BYTE_END) {
		i2c.regs->isr_reg = I2C_ISR_BYTE_END_WRITE_CLEAR ;
		i2c.isr_byte_end = 1 ;
		wakeup = 1;
	}

	if (isr_status & I2C_ISR_SCL_TIME_OUT) {
		i2c.regs->isr_reg = I2C_ISR_SCL_TIME_OUT_WRITE_CLEAR ;
		i2c.isr_timeout = 1 ;
		wakeup = 1;
	}


	if (wakeup) {
		spin_lock_irqsave(&i2c_wmt_irqlock, flags);
		i2c.isr_int_pending = 1;
		spin_unlock_irqrestore(&i2c_wmt_irqlock, flags);
		wake_up_interruptible(&i2c_wait);
	} else
		DPRINTK("i2c_err : unknown I2C ISR Handle 0x%4.4X" , isr_status) ;
	return IRQ_HANDLED;
}

/*!*************************************************************************
* i2c_wmt_resource_init()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Request a IRQ
*
* \retval 0 if success
*/
static int i2c_wmt_resource_init(void)
{
	/* IRQ_I2C  19*/
	if (request_irq(i2c.irq_no , &i2c_wmt_handler, IRQF_DISABLED, "i2c", 0) < 0) {
		DPRINTK(KERN_INFO "I2C: Failed to register I2C irq %i\n", i2c.irq_no);
		return -ENODEV;
	}
	return 0;
}

/*!*************************************************************************
* i2c_wmt_resource_release()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Release interrupt handler
*
* \retval  NULL
*/
static void i2c_wmt_resource_release(void)
{
	free_irq(i2c.irq_no, 0);
}

/*!*************************************************************************
* i2c_wmt_client_register()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Not in use
*
* \retval 0 if success
*/
static int i2c_wmt_client_register(struct i2c_client *client)
{
	return 0;
}

/*!*************************************************************************
* i2c_wmt_client_unregister()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Not in use
*
* \retval 0 if success
*/
static int i2c_wmt_client_unregister(struct i2c_client *client)
{
	return 0;
}
static struct i2c_algo_wmt_data i2c_wmt_data = {
	write_msg:          i2c_wmt_write_msg,
	read_msg:           i2c_wmt_read_msg,
	wait_bus_not_busy:  i2c_wmt_wait_bus_not_busy,
	reset:              i2c_wmt_reset,
	set_mode:		i2c_wmt_set_mode,
	udelay:             I2C_ALGO_UDELAY,
	timeout:            I2C_ALGO_TIMEOUT,
};

static struct i2c_adapter i2c_wmt_ops = {
	.owner      		= THIS_MODULE,
	.id         		= I2C_ALGO_WMT,
	.algo_data  		= &i2c_wmt_data,
	.name       		= "wmt_i2c_adapter",
	.client_register   	= i2c_wmt_client_register,
	.client_unregister 	= i2c_wmt_client_unregister,
	.retries           	= I2C_ADAPTER_RETRIES,
};

extern int wmt_i2c_add_bus(struct i2c_adapter *);
extern int wmt_i2c_del_bus(struct i2c_adapter *);

/*!*************************************************************************
* i2c_adap_wmt_init()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Register driver and interrupt handler to kernel, first initialize controller default register setting
*
* \retval  0 if success
*/
static int __init i2c_adap_wmt_init(void)
{
	unsigned short tmp ;

	/**/
	/* software initial*/
	/**/
	i2c.regs        = (struct i2c_regs_s *) I2C_BASE_ADDR ;
	i2c.irq_no      = IRQ_I2C0 ;
	i2c.i2c_mode    = I2C_STANDARD_MODE ;
	i2c.isr_nack    = 0 ;
	i2c.isr_byte_end = 0 ;
	i2c.isr_timeout = 0 ;
	i2c.isr_int_pending = 0;
	/**/
	/* hardware initial*/
	/**/
	*(unsigned long *)(0xD8110500) &= ~(0x00000003);
	*(unsigned long *)(0xD8130250) |= 0x0020;
	i2c.regs->cr_reg  = 0 ;
	i2c.regs->div_reg = APB_166M_I2C_DIV ;
	i2c.regs->isr_reg = I2C_ISR_ALL_WRITE_CLEAR ;   /* 0x0007*/
	i2c.regs->imr_reg = I2C_IMR_ALL_ENABLE ;        /* 0x0007*/

	i2c.regs->cr_reg  = I2C_CR_ENABLE ;
	tmp = i2c.regs->csr_reg ;                     /* read clear*/
	i2c.regs->isr_reg = I2C_ISR_ALL_WRITE_CLEAR ; /* 0x0007*/

	if (i2c.i2c_mode == I2C_STANDARD_MODE)
		i2c.regs->tr_reg = I2C_TR_STD_VALUE ;   /* 0x8041*/
	else if (i2c.i2c_mode == I2C_FAST_MODE)
		i2c.regs->tr_reg = I2C_TR_FAST_VALUE ; /* 0x8011*/


	if (i2c_wmt_resource_init() == 0) {
		if (wmt_i2c_add_bus(&i2c_wmt_ops) < 0) {
			i2c_wmt_resource_release();
			printk(KERN_INFO "i2c: Failed to add bus\n");
			return -ENODEV;
		}
	} else
		return -ENODEV;


	printk(KERN_INFO "i2c: successfully added bus\n");

#ifdef I2C_REG_TEST
	printk("i2c.regs->cr_reg= 0x%08x\n\r", i2c.regs->cr_reg);
	printk("i2c.regs->tcr_reg= 0x%08x\n\r", i2c.regs->tcr_reg);
	printk("i2c.regs->csr_reg= 0x%08x\n\r", i2c.regs->csr_reg);
	printk("i2c.regs->isr_reg= 0x%08x\n\r", i2c.regs->isr_reg);
	printk("i2c.regs->imr_reg= 0x%08x\n\r", i2c.regs->imr_reg);
	printk("i2c.regs->cdr_reg= 0x%08x\n\r", i2c.regs->cdr_reg);
	printk("i2c.regs->tr_reg= 0x%08x\n\r", i2c.regs->tr_reg);
	printk("i2c.regs->div_reg= 0x%08x\n\r", i2c.regs->div_reg);
#endif

	return 0;
}

/*!*************************************************************************
* i2c_adap_wmt_exit()
*
* Private Function by Paul Kwong, 2007/1/12
*/
/*!
* \brief Reslease all resource and unregister from bus
*
* \retval  NULL
*/
static void i2c_adap_wmt_exit(void)
{
	wmt_i2c_del_bus(&i2c_wmt_ops);
	i2c_wmt_resource_release();

	printk(KERN_INFO "i2c: successfully removed bus\n");
}


MODULE_AUTHOR("VIA RISC & DSP SW Team");
MODULE_DESCRIPTION("WMT I2C Adapter Driver");
MODULE_LICENSE("GPL");

module_init(i2c_adap_wmt_init);
module_exit(i2c_adap_wmt_exit);

EXPORT_SYMBOL(i2c_wmt_read_msg);
EXPORT_SYMBOL(i2c_wmt_write_msg);
