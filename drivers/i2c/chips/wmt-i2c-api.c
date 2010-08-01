/*++
	drivers/i2c/busses/wmt_i2c_api.c

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

// Include your headers here
#include <linux/kernel.h>
#include <linux/module.h>
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
//#include <linux/i2c-sensor.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>

 
#ifdef __KERNEL__

#undef DEBUG

#ifdef DEBUG
	#define DPRINTK		printk
#else
	#define DPRINTK(x...)
#endif 

#else
	#define DPRINTK    	printf

#endif
/*----------------------- INTERNAL PRIVATE MARCOS -------------------------*/
#define WMT_I2C_R 1
#define WMT_I2C_W 0


/*----------------------- INTERNAL PRIVATE FUNCTIONS -------------------------*/
int  wmt_i2c_api_identify(void) ;
int  wmt_i2c_api_attach(struct i2c_adapter *adapter, int address, int kind);
int  wmt_i2c_api_probe(struct i2c_adapter *adapter);
int  wmt_i2c_api_detach(struct i2c_client *client);
int  wmt_i2c_api_init(void) ;
void wmt_i2c_api_exit(void) ;
static int wmt_i2c_api_suspend(struct device *dev, pm_message_t state);
static int wmt_i2c_api_resume(struct device *dev);



/*----------------------- SPECIFY A MINOR NUMBER -------------------------------
* The id should be a unique ID. The range 0xf000 to 0xffff is reserved for local
* use, and you can use one of those until you start distributing the driver.
*/
#define 	I2C_DRIVER_ID_WMT_I2C_API		0xf055		/*default address, Should be changed*/


/*----------------------- EXTRA CLIENT DATA ------------------------------------
* The client structure has a special `data' field that can point to any
* structure at all. You can use this to keep client-specific data.
*/
struct wmt_i2c_api_data 
{		
	unsigned char	chip_rev ;
};


/*----------------------- CLIENT ADDRESS OF I2C SLAVE CHIP -------------------*/

static const unsigned short normal_addr[] = { WMT_I2C_API_I2C_ADDR, I2C_CLIENT_END };
static const unsigned short ignore[] 		= { I2C_CLIENT_END };
/*static unsigned short forces[] 		= { I2C_CLIENT_END };*/

static struct i2c_client_address_data addr_data = 
{
	.normal_i2c			= normal_addr,
	.probe				= ignore,
	.ignore				= ignore,
	.forces				= NULL,
};


/*----------------------- THE DRIVER STRUCTURE ---------------------------------
* A driver structure contains general access routines.
*/
static struct i2c_driver wmt_i2c_api_driver ;



static const struct i2c_device_id wmt_id[] = {
        { "wm8510", 0 },
        { } 
};      

static struct i2c_driver wmt_i2c_api_driver = {
	.id_table		= wmt_id,
	.attach_adapter = wmt_i2c_api_probe,
	.detach_client	= wmt_i2c_api_detach,
 	.command		= NULL,
	.driver = {
                .name   = "wm8510",
 		.suspend = wmt_i2c_api_suspend,
 		.resume = wmt_i2c_api_resume,
        },
};

/*----------------------- DEFINE I2C CLIENT ------------------------------------
* A client structure specific information like the actual I2C address.
*/
static struct i2c_client *i2c_api_client;


/*----------------------- INTERNAL PRIVATE VARIABLES -------------------------*/
static int wmt_i2c_api_i2c_id = 0;
static int wmt_i2c_api_i2c_initialized = 0;
/*static int wmt_i2c_api_client= 0;*/

struct i2c_regs_s wmt_i2c_reg ;
static volatile struct i2c_regs_s *wmt_i2c_reg_addr = (struct i2c_regs_s *) I2C0_BASE_ADDR;
static unsigned int  wmt_i2c_suspend_flag = 0;
/*----------------------- PRIVATE CONSTANTS ----------------------------------*/



/*----------------------- Function Body --------------------------------------*/
/*************************************************************************
* i2c_api_register_write - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
void i2c_api_register_write(int index, u8 data)
{
	int ret, retry ;
	
	unsigned char p_data[2] ;
	struct i2c_msg wr[1] ;

	if( i2c_api_client == NULL ) {		
		DPRINTK("[i2c_api_register_write] i2c_api_client is NULL \n");
		return ;
	}
    
	p_data[0] = (unsigned char)index ;
	p_data[1] = data ;

	wr[0].addr  = i2c_api_client->addr ;
	wr[0].flags = WMT_I2C_W ;
	wr[0].len   = 2 ;
	wr[0].buf   = p_data ;

	retry =3;
	while(retry){
		ret = i2c_transfer(i2c_api_client->adapter, wr, 1);
		if (ret != 1) {
			retry--;
			DPRINTK("[i2c_api_register_write: retry] %d times left \n", retry);
		} else
			retry=0;
	} 
		
	if (ret != 1) {
	    DPRINTK("[i2c_api_register_write] write fail \n");
	    return ;
	}
	DPRINTK("[i2c_api_register_write] OK! \n");
	return ;
}
/*************************************************************************
* i2c_api_page_write - 
* 
* Private Function by Paul Kwong, 10/21/2005
* 
* RETURNS: 
* 
*/
void i2c_api_page_write(int index, u8 data[], int count)
{
	int  i,ret, retry ;

	unsigned char p_data[count +1] ; //pdata[count +1]
	struct i2c_msg wr[1] ;

	if( i2c_api_client == NULL ) {		
		DPRINTK("[i2c_api_page_write] i2c_api_client is NULL \n");
		return ;
	}

	p_data[0] = (unsigned char)index ;
	for (i = 0 ; i < count; i++)
		;
	p_data[1+i] = data[i] ;

	wr[0].addr  = i2c_api_client->addr ;
	//wr[0].addr = addr;
	wr[0].flags = WMT_I2C_W ;
	wr[0].len   = count+1 ;
	wr[0].buf   = p_data ;

	retry =3;
	while(retry){
		ret = i2c_transfer(i2c_api_client->adapter, wr, 1);
		DPRINTK("[i2c_api_page_write: retry] ret =%d \n", ret);
		if (ret != 1) {
			retry--;
			DPRINTK("[i2c_api_page_write: retry] %d times left \n", retry);
		} else
			retry=0;
	}    
	if (ret != 1) {
		DPRINTK("[i2c_api_page_write] write fail \n");
		return ;
	}
	DPRINTK("[i2c_api_page_write] OK! \n");
	return ;

}
/*************************************************************************
* wmt_i2c_xfer_if- 
* 
* Private Function by Dean 2009/2/2
* 
* RETURNS: 
* 
*/
void wmt_i2c_xfer_if(struct i2c_msg *msg)
{
	int  ret;

	if( i2c_api_client == NULL ) {		
		DPRINTK("[%s] i2c_api_client is NULL \n", __func__);
		return ;
	}
	ret = i2c_transfer(i2c_api_client->adapter, msg, 1);
	if (ret != 1) {
		DPRINTK("[%s] fail \n", __func__);
		return ;
	}
	DPRINTK("[%s] OK! \n", __func__);
	return ;

}

/*************************************************************************
* wmt_i2c_xfer_continue_if- 
* 
* Private Function by Dean 2009/2/6
* 
* RETURNS: 
* 
*/
void wmt_i2c_xfer_continue_if(struct i2c_msg *msg, unsigned int num)
{
	int  ret;

	if( i2c_api_client == NULL ) {		
		DPRINTK("[%s] i2c_api_client is NULL \n", __func__);
		return ;
	}
	ret = i2c_transfer(i2c_api_client->adapter, msg, num);
	if (ret != 1) {
		DPRINTK("[%s] fail \n", __func__);
		return ;
	}
	DPRINTK("[%s] OK! \n", __func__);
	return ;

}

/*************************************************************************
* i2c_api_register_read - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
int i2c_api_register_read(int index)
{
	int ret, retry ;

	unsigned char reg_index[1] = { 0 } ;
	unsigned char p_data[1] = { 0 } ;
	struct i2c_msg 	wr[1] ;
	struct i2c_msg 	rd[1] ;

	if( i2c_api_client == NULL ) {		
		DPRINTK("[i2c_api_register_read] i2c_api_client is NULL \n");
		return 0 ;
	}

	/* step1: write address out */
	reg_index[0] = (unsigned char)index ;
	p_data[0]    = 0 ;

	wr[0].addr  = i2c_api_client->addr  ;  
	//wr[0].addr = addr;
	wr[0].flags = WMT_I2C_W ;
	wr[0].len   = 1 ;    
	wr[0].buf   = reg_index ;    

	retry = 3;
	while (retry) {
		ret = i2c_transfer(i2c_api_client->adapter, wr, 1);
		if (ret != 1) {
			retry--;
			DPRINTK("[i2c_api_register_read: retry] %d times left \n", retry);
		} else
			retry=0;
	} 
	    
	if (ret != 1) {
		DPRINTK("[i2c_api_register_read] write fail \n");
		return 0 ;
	}

	/* step2: read data in */
	rd[0].addr  = i2c_api_client->addr  ;
	//rd[0].addr = addr;
	rd[0].flags = WMT_I2C_R ;
	rd[0].len   = 1 ; 
	rd[0].buf   = p_data  ;   

	retry =3;
	while (retry) {
		ret = i2c_transfer(i2c_api_client->adapter, rd, 1);
		if (ret != 1) {
			retry--;
			DPRINTK("[i2c_api_register_read: retry] %d times left \n", retry);
		} else
			retry=0;
	} 
	      
	if (ret != 1) {
		DPRINTK("[i2c_api_register_read] read fail \n");
		return 0 ;
	}

	return( p_data[0] );    

}
/*************************************************************************
* i2c_api_page_read - 
* 
* Private Function by Paul Kwong, 10/21/2005
* 
* RETURNS: 
* 
*/
int i2c_api_page_read(int index, int count)
{
	int i,ret, retry ;

	unsigned char 	reg_index[1] = { 0 } ;
	unsigned char 	p_data[count]   ;
	struct i2c_msg 	wr[1] ;
	struct i2c_msg 	rd[1] ;

	if( i2c_api_client == NULL ) {		
		DPRINTK("[i2c_api_page_read] i2c_api_client is NULL \n");
		return 0 ;
	}

	/* step1: write address out */
	reg_index[0] = (unsigned char)index ;
	for (i=0 ; i < count ; i++)
		;
	p_data[i] = 0 ;

	wr[0].addr = i2c_api_client->addr  ;  
	//wr[0].addr = addr;
	wr[0].flags = WMT_I2C_W ;
	wr[0].len   = 1 ;    
	wr[0].buf   = reg_index ;    
	retry =3;
	while(retry){
		ret = i2c_transfer(i2c_api_client->adapter, wr, 1);
		if (ret != 1) {
			retry--;
			DPRINTK("[i2c_api_page_read: retry] %d times left \n", retry);
		} else
			retry=0;
	}

	if (ret != 1) {
		DPRINTK("[i2c_api_page_read] write fail \n");
		return 0 ;
	}

	/* step2: read data in */
	rd[0].addr  = i2c_api_client->addr  ;
	//rd[0].addr = addr;
	rd[0].flags = WMT_I2C_R ;
	rd[0].len   = count ; 
	rd[0].buf   = p_data  ;    
	retry =3;
	while (retry) {
		ret = i2c_transfer(i2c_api_client->adapter, rd, 1);
		DPRINTK("[i2c_api_page_read: retry] ret =%d \n", ret);				
		if (ret != 1) {
			retry--;
			DPRINTK("[i2c_api_page_read: retry] %d times left \n", retry);
		}
		else
			retry=0;
	}

	if (ret != 1) {
		DPRINTK("[i2c_api_page_read] read fail \n");
		return 0 ;
	}
	return( p_data[0] );    
   
}

/*************************************************************************
* wmt_i2c_api_identify - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
int wmt_i2c_api_identify(void)
{
    //int ver;

    /* ----- TO DO ----------*/
    
	return 1;
}

/*************************************************************************
* wmt_i2c_api_attach - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
int wmt_i2c_api_attach(struct i2c_adapter *adap, int addr, int kind)
{
	int ret;
	int chip_ver;
	struct i2c_client *new_client;
	struct wmt_i2c_api_data *data;
	
	/* step1: malloc */
	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL );
	if ( new_client == NULL )  {
		ret = -ENOMEM;
		DPRINTK("[wmt_i2c_api_attach] new_client = NULL \n") ;
		goto exit ;	    
	}
	memset(new_client, 0, sizeof(struct i2c_client));
	
	data = kmalloc(sizeof(struct wmt_i2c_api_data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		kfree(new_client);
		DPRINTK("[wmt_i2c_api_attach] data = NULL \n") ;
		goto exit;
	}	
	memset(data, 0, sizeof(struct wmt_i2c_api_data));

	/* step2: initial client and data */
	new_client->addr 	= addr;
	i2c_set_clientdata(new_client, data);
	new_client->adapter = adap;
	new_client->driver 	= &wmt_i2c_api_driver;	
//	new_client->flags 	= I2C_CLIENT_ALLOW_USE ;
//	new_client->id 		= wmt_i2c_api_i2c_id++; 	
//	snprintf(new_client->name, sizeof(new_client->name) - 1, 
//			"CHIP NAME[%d]", new_client->id);	
	snprintf(new_client->name, I2C_NAME_SIZE, 
			"CHIP NAME[%s]","wm8510" );// Vincent 20090527 ??	
			
	i2c_api_client = new_client;
		
	/* step3: check chip ID, go to exit_kfree if fail */
	chip_ver = wmt_i2c_api_identify() ;
	
	if (!chip_ver) {
		DPRINTK("[wmt_i2c_api_attach] check chip ID fail \n") ;
		ret = -ENODEV;
		goto exit_kfree;	
	}

	data->chip_rev = chip_ver ;

	/* step4: attach client to I2C Bus */
	ret = i2c_attach_client(new_client);

exit_kfree:	
	if (ret) {
		kfree(new_client) ;
		kfree(data);
		i2c_api_client = NULL ;
		DPRINTK("[wmt_i2c_api_attach] exit_kfree \n") ;
	}
	
	
exit:
	
	return ret;		
}

/*************************************************************************
* wmt_i2c_api_probe - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
int wmt_i2c_api_probe(struct i2c_adapter *adap){
		
	return i2c_probe(adap, &addr_data, wmt_i2c_api_attach);
}

/*************************************************************************
* wmt_i2c_api_detach - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
int wmt_i2c_api_detach(struct i2c_client *client)
{	
	int err;
		
	if ( (err = i2c_detach_client(client) ) ) {
		printk("[wmt_i2c_api_detach] Client deregistration failed, client not detached.\n");
		return err;
	}
	    
	kfree( i2c_get_clientdata(client) );	
	kfree( client );  // Frees client data too, if allocated at the same time 
	i2c_api_client = NULL ;
	return 0;
}
/*!*************************************************************************
* wmt_i2c_api_suspend()
* 
* Private Function by Kenny Chou, 2008/06/27(VT8430)
* Revised by Dean Hsiao, 2008/07/02(VT8500)
*/
/*!
* \brief
*        
* \retval  0 if success
*/ 
static int wmt_i2c_api_suspend
(
        struct device *dev,		/*!<; // a pointer point to struct device */ 
        pm_message_t state					/*!<; // state of suspend */ 
)
{
    printk("i2c suspend\n");
#if 0
    switch ( level ) {
        case SUSPEND_NOTIFY:
            /*
             * Suspend transition is about to happen.
             */
             //if ((!vt8430_i2c_suspend_flag) && (state == PM_SUSPEND_MEM)) {
             if (!vt8430_i2c_suspend_flag ) {
            	vt8430_i2c_reg.IICIMR = vt8430_i2c_reg_addr->IICIMR;
            	vt8430_i2c_reg.IICTR = vt8430_i2c_reg_addr->IICTR;
            	vt8430_i2c_reg.IICDIV = vt8430_i2c_reg_addr->IICDIV;
            	vt8430_i2c_suspend_flag = 1;
             }
            break;
        case SUSPEND_SAVE_STATE:
            /*
             * We only need to save hardware registers
             * on power-off suspend.
             */
            break;
        case SUSPEND_DISABLE:
            break;
        case SUSPEND_POWER_DOWN:
            /*
             * Place the device in the low power state
             */ 
            break;
    }
#endif
    return 0;
} /* End of wmt_i2c_api_suspend() */
/*!*************************************************************************
* wmt_i2c_api_resume()
* 
* Private Function by Kenny Chou, 2008/06/27(VT8430)
* Revised by Dean Hsiao, 2008/07/02(VT8500)
*/
/*!
* \brief
*        
* \retval  0 if success
*/
static int wmt_i2c_api_resume
(
        struct device *dev    /*!<; // a pointer point to struct device */
)
{
	unsigned short tmp ;
	printk("i2c resume\n");
	
//	switch ( level ) {
//	case RESUME_POWER_ON:
		/*
		* Set the power state to the state before
		* the suspend call
		*/
		if (wmt_i2c_suspend_flag) {
			wmt_i2c_suspend_flag = 0;
			/*
			 * hardware initial
			 */
			*(unsigned long *)(0xD8110500) = ~(0x00000003);
			*(unsigned long *)(0xD8130250) |= 0x0020;
			wmt_i2c_reg_addr->cr_reg  = 0 ;
			wmt_i2c_reg_addr->div_reg = wmt_i2c_reg.div_reg;
			wmt_i2c_reg_addr->imr_reg = wmt_i2c_reg.imr_reg;
			wmt_i2c_reg_addr->cr_reg = 0x001 ;
			tmp = wmt_i2c_reg_addr->isr_reg; /* read clear*/
			wmt_i2c_reg_addr->tcr_reg = wmt_i2c_reg.tcr_reg;
		}
//		break;
//	case RESUME_RESTORE_STATE:
		/*
		* Restore the state saved by the
		* SUSPEND_SAVE_STATE suspend call.
		*/
//		break;
//	case RESUME_ENABLE:
//		break;
//	}
	return 0;
} /* End of wmt_i2c_api_resume() */

/*************************************************************************
* wmt_i2c_api_init - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
int wmt_i2c_api_init(void)
{	
	int res;
	
	if (!wmt_i2c_api_i2c_initialized){
		if ( (res = i2c_add_driver(&wmt_i2c_api_driver)) ) {
			printk("[wmt_i2c_api_init] Driver registration failed, module not inserted.\n");
			wmt_i2c_api_exit();
			return res;
		}
	}
	printk(KERN_INFO "[wmt_i2c_api_init] wmt_i2c_api_init.\n");
	wmt_i2c_api_i2c_initialized++;

	/* ----- Doing test ----------*/
	wmt_i2c_api_identify() ;
	
	return 0 ;	
}

/*************************************************************************
* wmt_i2c_api_exit - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
void wmt_i2c_api_exit(void)
{	
	if (wmt_i2c_api_i2c_initialized == 1) {
		i2c_del_driver(&wmt_i2c_api_driver);
		
	}
	wmt_i2c_api_i2c_initialized --;	
}

/*************************************************************************
* wmt_i2c_api_i2c_init - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
static int __init  wmt_i2c_api_i2c_init(void)
{	
	printk("[wmt_i2c_api_i2c_init]\n");
	wmt_i2c_api_init() ;
	return 0 ;	
}

/*************************************************************************
* wmt_i2c_api_i2c_exit - 
* 
* Private Function by Paul Kwong, 2007/5/4
* 
* RETURNS: 
* 
*/
static void __exit  wmt_i2c_api_i2c_exit(void)
{	
	wmt_i2c_api_exit() ;
	printk("[wmt_i2c_api_i2c_exit] Driver delete !\n");
}

/*----------------------- End of Function Body -------------------------------*/

MODULE_AUTHOR("MCE SW Team");
MODULE_DESCRIPTION("WMT I2C wmt_i2c_api Driver");
MODULE_LICENSE("GPL");

module_init(wmt_i2c_api_i2c_init);
module_exit(wmt_i2c_api_i2c_exit);

EXPORT_SYMBOL(i2c_api_register_write);
EXPORT_SYMBOL(i2c_api_register_read);
EXPORT_SYMBOL(i2c_api_page_write);
EXPORT_SYMBOL(i2c_api_page_read);
EXPORT_SYMBOL(wmt_i2c_xfer_if);
EXPORT_SYMBOL(wmt_i2c_xfer_continue_if);
