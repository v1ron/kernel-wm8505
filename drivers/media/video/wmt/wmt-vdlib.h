/*--- wmt-vdlib.h---------------------------------------------------------------
*   Copyright (C) 2008 WonderMedia Tech. Inc.
*
* MODULE       : wmt-vdlib.h -- 
* AUTHOR       : Jason Lin
* DATE         : 2008/12/08
* DESCRIPTION  : 
*------------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Jason Lin, 2008/12/08
*	First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/
#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <mach/memblock.h>  /* For MB driver only */

#include "com-vd.h"
/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/

typedef struct BitContext {
	struct prdt_struct *prd,*curprd;
//	unsigned char *start;
//	unsigned int prdbzs;	// prd bits size
	unsigned int prdbzi;	// prd bits index
	unsigned int bzidx;		// bits index
	unsigned int sibys;		// size in bytes
}BitContext;

void init_get_bits(BitContext *, struct prdt_struct *);
unsigned int get_bits(BitContext *, int);
unsigned int show_bits(BitContext *, int);
void skip_bits(BitContext *, int);
int bits_count(BitContext *);
unsigned int bits_end(BitContext *);

