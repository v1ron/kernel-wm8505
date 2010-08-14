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


/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Jason Lin, 2008/12/08
*	First version
*
*------------------------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/config.h>

#include "wmt-vdlib.h"

#define LIB_NAME "wmt-vdlib"
#define LIB_VERSION 1	// 0.00.01

// #define DEBUG
// #define WORDY

#define INFO(fmt,args...) printk(KERN_INFO "[" DRIVER_NAME "] " fmt , ## args)
#define WARN(fmt,args...) printk(KERN_WARNING "[" DRIVER_NAME "] " fmt , ## args)
#define ERROR(fmt,args...) printk(KERN_ERR "[" DRIVER_NAME "] " fmt , ## args)

#ifdef DEBUG
#define DBG(fmt, args...) printk(KERN_DEBUG "[" DRIVER_NAME "] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBG(fmt, args...)
#undef WORDY
#endif

#ifdef WORDY
#define WDBG(fmt, args...)	DBG(fmt, args...)
#else
#define WDBG(fmt, args...)
#endif

// Bit Context Read u8/u16/u24/u32
#define BC_RU8(x)		((unsigned char*)(x))[0]
#define BC_RU16(x)		((((unsigned char *)(x))[0] << 8) | \
						((unsigned char*)(x))[1])
#define BC_RU24(x)		((((unsigned char *)(x))[0] << 16) | \
						(((unsigned char *)(x))[1] <<  8) | \
						((unsigned char *)(x))[2])
#define BC_RU32(x)		((((unsigned char *)(x))[0] << 24) | \
						(((unsigned char *)(x))[1] << 16) | \
						(((unsigned char *)(x))[2] <<  8) | \
						((unsigned char *)(x))[3])

#define BC_UPDATE_CACHE(p,i,type)	\
						BC_RU##type(((unsigned char *)phys_to_virt(p->addr))+((i)>>3)) << ((i)&0x07)

#define BC_NEXT_PRD(bc)	bc->curprd = bc->curprd->EDT?bc->curprd:bc->curprd+1;	\
						bc->prdbzi = 0

#define BC_UPDATE(bc,n)	bc->bzidx += n;	\
						bc->prdbzi += n

#define END_OF_PRDT(prd,i)	\
						((i >= (prd->size*8)) && (prd->EDT))

#define END_OF_BC(bc)	(END_OF_PRDT(bc->curprd,bc->prdbzi))

inline unsigned int prd_get_bits(
	struct prdt_struct *p, 
	unsigned int i, 
	unsigned int n, 
	unsigned int *v)
{
	unsigned int c = 0;
	unsigned int btz = p->size * 8;

	if(n > 24)
		n = 24;

	*v = 0;
	n = (max(i+n,btz) - btz)?(btz-i):n;
	switch((n + (i & 0x7)) >> 3){
		case 0:	// read in one byte
			c = BC_UPDATE_CACHE(p,i,8) & 0xFF;
			*v = c >> (8-n);
			break;
		case 1:	// read in two bytes
			c = BC_UPDATE_CACHE(p,i,16) & 0xFFFF;
			*v = c >> (16-n);
			break;
		case 2:	// read in three bytes	
			c = BC_UPDATE_CACHE(p,i,24) & 0xFFFFFF;
			*v = c >> (24-n);
			break;
		case 3:	// read in four bytes
			c = BC_UPDATE_CACHE(p,i,32); 
			*v = c >> (32-n);
			break;
		default: break;
	}
	
	DBG("prd %p: %p/%d i %d n %d c %x v %x\n",p,p->addr,p->size*8,i,n,c,*v);

	return n;
}

inline void init_get_bits(BitContext *bc, struct prdt_struct *prd)
{
	struct prdt_struct *prdidx = prd;

	if(!bc || !prd)
		return;

	memset(bc,0x0,sizeof(BitContext));

	bc->curprd = bc->prd = prd;
	bc->prdbzi = 0;

	while(!prdidx->EDT){
		bc->sibys += prdidx->size;
		prdidx++;
	}
}

/**
 * reads 0-32 bits.
 */
inline unsigned int get_bits(BitContext *bc, int n)
{
	unsigned int sum = 0, c = 0;

	if(n > 32) 
		n = 32;

	while(n){
		int r = prd_get_bits(bc->curprd, bc->prdbzi, n, &c);
		BC_UPDATE(bc,r);
		sum += c;

		// read in data satisfied or last prd
		if(n == r || END_OF_BC(bc))
			break;

		n -= r;
		sum <<= n;

		if(bc->prdbzi >= (bc->curprd->size*8)){
			BC_NEXT_PRD(bc);
		}
	}

	return sum;
}

/**
 * shows 0-32 bits.
 */
inline unsigned int show_bits(BitContext *bc, int n)
{
	struct prdt_struct *prd = bc->curprd;
	unsigned int bzi = bc->prdbzi;
	unsigned int sum = 0, c = 0;

	if(n > 32) 
		n = 32;

	while(n){
		int r = prd_get_bits(prd, bzi, n, &c);
		sum += c;
		bzi += r;

		// read in data satisfied or last prd
		if(n == r || END_OF_PRDT(prd, bzi))
			break;

		n -= r;
		sum <<= n;

		if(bzi >= (prd->size*8)){
			// next prd
			prd++;
			bzi = 0;
		}
	}

	return sum;
}

/**
 * skip 0-32 bits.
 */
inline void skip_bits(BitContext *bc, int n)
{

	if(n > 32) 
		n = 32;

	while(n && !END_OF_BC(bc)){
		int r = ((bc->prdbzi+n)>(bc->curprd->size*8))?
				bc->curprd->size*8 - bc->prdbzi:n;
		BC_UPDATE(bc,r);

		// read in data satisfied or last prd
		if(n == r)
			break;

		n -= r;
		BC_NEXT_PRD(bc);
	}
}

/**
 * consumed bits
 */
inline int bits_count(BitContext *bc)
{
	return bc->bzidx;
}

/**
 * is end of bits
 */
inline unsigned int bits_end(BitContext *bc)
{
	return END_OF_BC(bc);
}
