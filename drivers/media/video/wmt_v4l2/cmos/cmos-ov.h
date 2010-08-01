/* PVCS version log
** $Log:  $
 * 
 */
#ifndef CMOS_OV_H
/* To assert that only one occurrence is included */
#define CMOS_OV_H

/*--- wmt-cmos.h ----------------------------------------------------------------
*   Copyright (C) 2010 WonderMedia Tech. Inc.
*
* MODULE       : wmt-cmos.h
* AUTHOR       : Willy Chuang
* DATE         : 2010/05/28
* DESCRIPTION  : 
*------------------------------------------------------------------------------*/

/*--- History ------------------------------------------------------------------- 
*Version 0.01 , Willy Chuang, 2010/05/28
*	First version
*
*------------------------------------------------------------------------------*/
/*-------------------- MODULE DEPENDENCY -------------------------------------*/

/*-------------------- EXPORTED PRIVATE CONSTANTS ----------------------------*/


/*-------------------- EXPORTED PRIVATE TYPES---------------------------------*/
/* typedef  void  viaapi_xxx_t;  *//*Example*/

/*------------------------------------------------------------------------------
    Definitions of structures
------------------------------------------------------------------------------*/

/*-------------------- EXPORTED PRIVATE VARIABLES -----------------------------*/
#ifdef CMOS_OV_C 
    #define EXTERN
#else
    #define EXTERN   extern
#endif 

#undef EXTERN

/*--------------------- EXPORTED PRIVATE MACROS -------------------------------*/

/*--------------------- EXPORTED PRIVATE FUNCTIONS  ---------------------------*/
/* extern void  viaapi_xxxx(vdp_Void); *//*Example*/

int cmos_init_ov7670(int width, int height);
int cmos_exit_ov7670(int width, int height);

    
#endif /* ifndef CMOS_OV_H */

/*=== END cmos-ov.h ==========================================================*/
