/*--- memblock-ut.h -------------------------------------------------------------------
*   Copyright (C) 2008 WonderMedia Tech. Inc.
*
* MODULE       : memblock-ut.h
* AUTHOR       : Jason Lin
* DATE         : 2008/12/08
* DESCRIPTION  : 
*-----------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Jason Lin, 2008/12/08
* First version
*
*------------------------------------------------------------------------------*/
struct mem_area_struct{
	unsigned long input;
	unsigned long isize;
	/* MB mapped user space address, 
	   which used to save data copy from input prdt */
	unsigned long output;
	unsigned long osize;
};

#define MBIO_PRDT_LOOKBACK		_IOW (MB_IOC_MAGIC,MBIO_MAXiMUM+1,struct mem_area_struct)

