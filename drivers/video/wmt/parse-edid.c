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

#ifdef __KERNEL__
#define DPRINT printk
#include <linux/kernel.h>
#include <linux/string.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define DPRINT printf
#endif

#include "edid.h"

const unsigned char edid_v1_header[] = { 
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };

const unsigned char edid_v1_descriptor_flag[] = { 0x00, 0x00 };

#define COMBINE_HI_8LO( hi, lo ) \
        ( (((unsigned)hi) << 8) | (unsigned)lo )

#define COMBINE_HI_4LO( hi, lo ) \
        ( (((unsigned)hi) << 4) | (unsigned)lo )

#define UPPER_NIBBLE( x ) \
        (((128|64|32|16) & (x)) >> 4)

#define LOWER_NIBBLE( x ) \
        ((1|2|4|8) & (x))

#define MONITOR_NAME            0xfc
#define MONITOR_LIMITS          0xfd
#define UNKNOWN_DESCRIPTOR      -1
#define DETAILED_TIMING_BLOCK   -2

#define DEBUG
#ifdef DEBUG
#define DBGMSG(fmt, args...)  DPRINT("[EDID] %s: " fmt, __FUNCTION__ , ## args)
#else
#define DBGMSG(fmt, args...) do {} while(0)
#endif

edid_info_t edid_info;

static int block_type( unsigned char * block )
{
    if ( !memcmp( edid_v1_descriptor_flag, block, 2 ) ) {
//        DBGMSG("# Block type: 2:%x 3:%x\n", block[2], block[3]);

        /* descriptor */
        if ( block[ 2 ] != 0 )
	        return UNKNOWN_DESCRIPTOR;
        return block[ 3 ];
    } 
    /* detailed timing block */
    return DETAILED_TIMING_BLOCK;
} /* End of block_type() */

static char * get_vendor_sign( unsigned char * block, char *sign)
{
//    static char sign[4];
    unsigned short h;

  /*
     08h	WORD	big-endian manufacturer ID (see #00136)
		    bits 14-10: first letter (01h='A', 02h='B', etc.)
		    bits 9-5: second letter
		    bits 4-0: third letter
  */
    h = COMBINE_HI_8LO(block[0], block[1]);
    sign[0] = ((h>>10) & 0x1f) + 'A' - 1;
    sign[1] = ((h>>5) & 0x1f) + 'A' - 1;
    sign[2] = (h & 0x1f) + 'A' - 1;
    sign[3] = 0;
    
    return sign;
} /* End of get_vendor_sign() */

static char * get_monitor_name( unsigned char * block )
{
    #define DESCRIPTOR_DATA         5

    unsigned char *ptr = block + DESCRIPTOR_DATA;
    static char name[ 13 ];
    unsigned i;


    for( i = 0; i < 13; i++, ptr++ ) {
        if ( *ptr == 0xa ) {
	        name[ i ] = 0;
	        return name;
	    }
        name[ i ] = *ptr;
    }
    return name;
} /* End of get_monitor_name() */

static int parse_timing_description( unsigned char* dtd )
{
    #define PIXEL_CLOCK_LO     (unsigned)dtd[ 0 ]
    #define PIXEL_CLOCK_HI     (unsigned)dtd[ 1 ]
    #define PIXEL_CLOCK        (COMBINE_HI_8LO( PIXEL_CLOCK_HI,PIXEL_CLOCK_LO )*10000)
    #define H_ACTIVE_LO        (unsigned)dtd[ 2 ]
    #define H_BLANKING_LO      (unsigned)dtd[ 3 ]
    #define H_ACTIVE_HI        UPPER_NIBBLE( (unsigned)dtd[ 4 ] )
    #define H_ACTIVE           COMBINE_HI_8LO( H_ACTIVE_HI, H_ACTIVE_LO )
    #define H_BLANKING_HI      LOWER_NIBBLE( (unsigned)dtd[ 4 ] )
    #define H_BLANKING         COMBINE_HI_8LO( H_BLANKING_HI, H_BLANKING_LO )
    #define V_ACTIVE_LO        (unsigned)dtd[ 5 ]
    #define V_BLANKING_LO      (unsigned)dtd[ 6 ]
    #define V_ACTIVE_HI        UPPER_NIBBLE( (unsigned)dtd[ 7 ] )
    #define V_ACTIVE           COMBINE_HI_8LO( V_ACTIVE_HI, V_ACTIVE_LO )
    #define V_BLANKING_HI      LOWER_NIBBLE( (unsigned)dtd[ 7 ] )
    #define V_BLANKING         COMBINE_HI_8LO( V_BLANKING_HI, V_BLANKING_LO )
    #define H_SYNC_OFFSET_LO   (unsigned)dtd[ 8 ]
    #define H_SYNC_WIDTH_LO    (unsigned)dtd[ 9 ]
    #define V_SYNC_OFFSET_LO   UPPER_NIBBLE( (unsigned)dtd[ 10 ] )
    #define V_SYNC_WIDTH_LO    LOWER_NIBBLE( (unsigned)dtd[ 10 ] )
    #define V_SYNC_WIDTH_HI    ((unsigned)dtd[ 11 ] & (1|2))
    #define V_SYNC_OFFSET_HI   (((unsigned)dtd[ 11 ] & (4|8)) >> 2)
    #define H_SYNC_WIDTH_HI    (((unsigned)dtd[ 11 ] & (16|32)) >> 4)
    #define H_SYNC_OFFSET_HI   (((unsigned)dtd[ 11 ] & (64|128)) >> 6)
    #define V_SYNC_WIDTH       COMBINE_HI_4LO( V_SYNC_WIDTH_HI, V_SYNC_WIDTH_LO )
    #define V_SYNC_OFFSET      COMBINE_HI_4LO( V_SYNC_OFFSET_HI, V_SYNC_OFFSET_LO )
    #define H_SYNC_WIDTH       COMBINE_HI_4LO( H_SYNC_WIDTH_HI, H_SYNC_WIDTH_LO )
    #define H_SYNC_OFFSET      COMBINE_HI_4LO( H_SYNC_OFFSET_HI, H_SYNC_OFFSET_LO )
    #define H_SIZE_LO          (unsigned)dtd[ 12 ]
    #define V_SIZE_LO          (unsigned)dtd[ 13 ]
    #define H_SIZE_HI          UPPER_NIBBLE( (unsigned)dtd[ 14 ] )
    #define V_SIZE_HI          LOWER_NIBBLE( (unsigned)dtd[ 14 ] )
    #define H_SIZE             COMBINE_HI_8LO( H_SIZE_HI, H_SIZE_LO )
    #define V_SIZE             COMBINE_HI_8LO( V_SIZE_HI, V_SIZE_LO )
    #define H_BORDER           (unsigned)dtd[ 15 ]
    #define V_BORDER           (unsigned)dtd[ 16 ]
    #define FLAGS              (unsigned)dtd[ 17 ]
    #define INTERLACED         (FLAGS&128)
    #define SYNC_TYPE	   (FLAGS&3<<3)  /* bits 4,3 */
    #define SYNC_SEPARATE	   (3<<3)
    #define HSYNC_POSITIVE	   (FLAGS & 4)
    #define VSYNC_POSITIVE     (FLAGS & 2)

    int htotal, vtotal;
    
    htotal = H_ACTIVE + H_BLANKING;
    vtotal = V_ACTIVE + V_BLANKING;
  
    DBGMSG( "\tMode \"%dx%d\"", H_ACTIVE, V_ACTIVE );
	DPRINT("\n");
    DBGMSG( "# vfreq %dHz, hfreq %dkHz\n",
	        PIXEL_CLOCK/(vtotal*htotal),
	        PIXEL_CLOCK/(htotal*1000));
    DBGMSG( "\tDotClock\t%d\n", PIXEL_CLOCK/1000000 );
    DBGMSG( "\tHTimings\t%u %u %u %u\n", H_ACTIVE,
	      H_ACTIVE+H_SYNC_OFFSET, 
	      H_ACTIVE+H_SYNC_OFFSET+H_SYNC_WIDTH,
	      htotal );

    DBGMSG( "\tVTimings\t%u %u %u %u\n", V_ACTIVE,
	    V_ACTIVE+V_SYNC_OFFSET,
	    V_ACTIVE+V_SYNC_OFFSET+V_SYNC_WIDTH,
	    vtotal );

    if ( INTERLACED || (SYNC_TYPE == SYNC_SEPARATE)) {
        DBGMSG( "Flags\t%s\"%sHSync\" \"%sVSync\"\n",
	    INTERLACED ? "\"Interlace\" ": "",
	    HSYNC_POSITIVE ? "+": "-",
	    VSYNC_POSITIVE ? "+": "-");
    }

    DBGMSG( "EndMode\n" );

	{
		int i;
		for(i=0;i<4;i++){
			if( edid_info.detail_timing[i].resx == 0 ){
				edid_info.detail_timing[i].resx = H_ACTIVE;
				edid_info.detail_timing[i].resy = V_ACTIVE;
				edid_info.detail_timing[i].freq = PIXEL_CLOCK/(vtotal*htotal);
			}
		}
	}
    return 0;
} /* End of parse_timing_description() */

static int parse_dpms_capabilities(unsigned char flags)
{
    #define DPMS_ACTIVE_OFF		(1 << 5)
    #define DPMS_SUSPEND		(1 << 6)
    #define DPMS_STANDBY		(1 << 7)

    DBGMSG("# DPMS capabilities: Active off:%s  Suspend:%s  Standby:%s\n\n",
            (flags & DPMS_ACTIVE_OFF) ? "yes" : "no",
            (flags & DPMS_SUSPEND)    ? "yes" : "no",
            (flags & DPMS_STANDBY)    ? "yes" : "no");
    return 0;
} /* End of parse_dpms_capabilities() */

static int parse_monitor_limits( unsigned char * block )
{
    #define V_MIN_RATE              block[ 5 ]
    #define V_MAX_RATE              block[ 6 ]
    #define H_MIN_RATE              block[ 7 ]
    #define H_MAX_RATE              block[ 8 ]
    #define MAX_PIXEL_CLOCK         (((int)block[ 9 ]) * 10)
    #define GTF_SUPPORT             block[10]

    DBGMSG( "\tHorizontal Frequency: %u-%u Hz\n", H_MIN_RATE, H_MAX_RATE );
    DBGMSG( "\tVertical   Frequency: %u-%u kHz\n", V_MIN_RATE, V_MAX_RATE );
    if ( MAX_PIXEL_CLOCK == 10*0xff )
        DBGMSG( "\t# Max dot clock not given\n" );
    else {
        DBGMSG( "\t# Max dot clock (video bandwidth) %u MHz\n", (int)MAX_PIXEL_CLOCK );
		edid_info.pixel_clock_limit = MAX_PIXEL_CLOCK;
    }

    if ( GTF_SUPPORT ) {
        DBGMSG( "\t# EDID version 3 GTF given: contact author\n" );
    }
    return 0;
} /* End of parse_monitor_limits() */

static int get_established_timing( unsigned char * edid )
{
    unsigned char time_1, time_2;
    
    time_1 = edid[ESTABLISHED_TIMING_I];
    time_2 = edid[ESTABLISHED_TIMING_II];
	edid_info.establish_timing = time_1 + (time_2 << 8);

    /*--------------------------------------------------------------------------
        35: ESTABLISHED TIMING I
            bit 7-0: 720กั400@70 Hz, 720กั400@88 Hz, 640กั480@60 Hz, 640กั480@67 Hz,
                     640กั480@72 Hz, 640กั480@75 Hz, 800กั600@56 Hz, 800กั600@60 Hz
    --------------------------------------------------------------------------*/
    DBGMSG("Established Timimgs I:  0x%x\n", time_1);
    if( time_1 & 0x80 )
        DBGMSG("     \t%- dx%d@%dHz\n", 720, 400, 70);
    if( time_1 & 0x40 )
        DBGMSG("     \t%- dx%d@%dHz\n", 720, 400, 88);
    if( time_1 & 0x20 )
        DBGMSG("     \t%- dx%d@%dHz\n", 640, 480, 60);
    if( time_1 & 0x10 )
        DBGMSG("     \t%- dx%d@%dHz\n", 640, 480, 67);
    if( time_1 & 0x08 )
        DBGMSG("     \t%- dx%d@%dHz\n", 640, 480, 72);
    if( time_1 & 0x04 )
        DBGMSG("     \t%- dx%d@%dHz\n", 640, 480, 75);
    if( time_1 & 0x02 )
        DBGMSG("     \t%- dx%d@%dHz\n", 800, 600, 56);
    if( time_1 & 0x01 )
        DBGMSG("     \t%- dx%d@%dHz\n", 800, 600, 60);

    /*--------------------------------------------------------------------------
        36: ESTABLISHED TIMING II
            bit 7-0: 800กั600@72 Hz, 800กั600@75 Hz, 832กั624@75 Hz, 1024กั768@87 Hz (Interlaced),
                     1024กั768@60 Hz, 1024กั768@70 Hz, 1024กั768@75 Hz, 1280กั1024@75 Hz
    --------------------------------------------------------------------------*/
    DBGMSG("Established Timimgs II: 0x%x\n", time_2);
    if( time_2 & 0x80 )
        DBGMSG("     \t%- dx%d@%dHz\n", 800, 600, 72);
    if( time_2 & 0x40 )
        DBGMSG("     \t%- dx%d@%dHz\n", 800, 600, 75);
    if( time_2 & 0x20 )
        DBGMSG("     \t%- dx%d@%dHz\n", 832, 624, 75);
    if( time_2 & 0x10 )
        DBGMSG("     \t%- dx%d@%dHz (Interlace)\n", 1024, 768, 87);
    if( time_2 & 0x08 )
        DBGMSG("     \t%- dx%d@%dHz\n", 1024, 768, 60);
    if( time_2 & 0x04 )
        DBGMSG("     \t%- dx%d@%dHz\n", 1024, 768, 70);
    if( time_2 & 0x02 )
        DBGMSG("     \t%- dx%d@%dHz\n", 1024, 768, 75);
    if( time_2 & 0x01 )
        DBGMSG("     \t%- dx%d@%dHz\n", 1280, 1024, 75);
    
    return 0;
} /* End of get_established_timing() */

static int get_standard_timing( unsigned char * edid )
{
    unsigned char *ptr = edid +STANDARD_TIMING_IDENTIFICATION_START;
    int h_res, v_res, v_freq;
    int byte_1, byte_2, aspect, i;

    /*--------------------------------------------------------------------------
        First byte
            Horizontal resolution.  Multiply by 8, then add 248 for actual value.
        Second byte
            bit 7-6: Aspect ratio. Actual vertical resolution depends on horizontal 
            resolution.
            00=16:10, 01=4:3, 10=5:4, 11=16:9 (00=1:1 prior to v1.3)
            bit 5-0: Vertical frequency. Add 60 to get actual value.
    --------------------------------------------------------------------------*/  
    DBGMSG("Standard Timing Identification \n");
    for(i=0; i< STANDARD_TIMING_IDENTIFICATION_SIZE/2; i++ ) {
        byte_1 = *ptr++;
        byte_2 = *ptr++;
        if( (byte_1 == 0x01) && (byte_2 == 0x01) )
            break;
        h_res = (byte_1 * 8) + 248;
        aspect = byte_2 & 0xC0;
        switch(aspect) {
			default:
            case 0x00:
                v_res = h_res * 10/16;
                break;
            case 0x40:
                v_res = h_res * 3/4;
                break;
            case 0x80:
                v_res = h_res * 4/5;
                break;
            case 0xC0:
                v_res = h_res * 9/16;
                break;
        }
        v_freq = (byte_2 & 0x1F) + 60;
        DBGMSG("Standard Timing: \t%dx%d@%dHz\n", h_res, v_res, v_freq);
		edid_info.standard_timing[i].resx = h_res;
		edid_info.standard_timing[i].resy = h_res;
		edid_info.standard_timing[i].freq = v_freq;
    }    
    return 0;
} /* End of get_standard_timing() */


int parse_edid( unsigned char * edid )
{
    unsigned char * block;
    unsigned char checksum = 0;
    char *monitor_name = 0;
    char  monitor_alt_name[100];
    char  vendor_sign[4];
    int   i, ret = 0;

	memset(&edid_info,0,sizeof(edid_info_t));
    for( i = 0; i < EDID_LENGTH; i++ )
        checksum += edid[ i ];

    if ( checksum != 0 ) {
        DBGMSG("*E* EDID checksum failed - data is corrupt\n" );
        ret = -1;
		goto parse_end;
    }  

    if ( memcmp( edid+EDID_HEADER, edid_v1_header, EDID_HEADER_END+1 ) ) {
        DBGMSG("*E* first bytes don't match EDID version 1 header\n");
        ret = -1;
		goto parse_end;
    }

#ifdef DEBUG
	DBGMSG("dump EDID data\n");
	for(i=0;i<128;i++){
		if( (i%16)==0 ) printk("\n");
		printk("%02x ",edid[i]);
	}
	printk("\n");
#endif
    
    DBGMSG("EDID version:  %d.%d\n", (int)edid[EDID_STRUCT_VERSION],(int)edid[EDID_STRUCT_REVISION] );

    get_vendor_sign( edid + ID_MANUFACTURER_NAME,(char *) &vendor_sign ); 

    /*--------------------------------------------------------------------------
        Parse Monitor name
    --------------------------------------------------------------------------*/
    block = edid + DETAILED_TIMING_DESCRIPTIONS_START;
    for( i = 0; i < NO_DETAILED_TIMING_DESCRIPTIONS; i++,
	     block += DETAILED_TIMING_DESCRIPTION_SIZE ) {
        if ( block_type( block ) == MONITOR_NAME ) {
	        monitor_name = get_monitor_name( block );
	        break;
	    }
    }

    if (!monitor_name) {
        /* Stupid djgpp hasn't snDBGMSG so we have to hack something together */
        if(strlen(vendor_sign) + 10 > sizeof(monitor_alt_name))
            vendor_sign[3] = 0;
    
        sprintf(monitor_alt_name, "%s:%02x%02x",
	            vendor_sign, edid[ID_MODEL], edid[ID_MODEL+1]) ;
        monitor_name = monitor_alt_name;
    }

    DBGMSG( "Identifier \"%s\"\n", monitor_name );
    DBGMSG( "VendorName \"%s\"\n", vendor_sign );
    DBGMSG( "ModelName  \"%s\"\n",  monitor_name );

    parse_dpms_capabilities(edid[DPMS_FLAGS]);

    /*--------------------------------------------------------------------------
        Parse ESTABLISHED TIMING I and II
    --------------------------------------------------------------------------*/
    get_established_timing( edid );

    /*--------------------------------------------------------------------------
        Parse STANDARD TIMING IDENTIFICATION
    --------------------------------------------------------------------------*/
    get_standard_timing( edid );

    block = edid + DETAILED_TIMING_DESCRIPTIONS_START;
    for( i = 0; i < NO_DETAILED_TIMING_DESCRIPTIONS; i++,
	     block += DETAILED_TIMING_DESCRIPTION_SIZE ) {
        if ( block_type( block ) == MONITOR_LIMITS )
	        parse_monitor_limits( block );
    }

    block = edid + DETAILED_TIMING_DESCRIPTIONS_START;
    for( i = 0; i < NO_DETAILED_TIMING_DESCRIPTIONS; i++,
	     block += DETAILED_TIMING_DESCRIPTION_SIZE ) {
        if ( block_type( block ) == DETAILED_TIMING_BLOCK )
	        parse_timing_description( block );
    }
parse_end:
  	return ret;
}

edid_timing_t edid_establish_timing[] = {
	{ 720, 400, 70 }, { 720, 400, 88 }, { 640, 480, 60 }, { 640, 480, 67 }, { 640, 480, 72 }, { 640, 480, 75 },
	{ 800, 600, 56 }, { 800, 600, 60 }, { 800, 600, 72 }, { 800, 600, 75 }, { 832, 624, 75 }, { 1024, 768, 87 },
	{ 1024, 768, 60 }, { 1024, 768, 70 }, { 1024, 768, 75 }, { 1280, 1024, 75 }, { 1152, 870, 75 }
};

int edid_find_support(unsigned int resx,unsigned int resy,int freq)
{
	int ret;
	int i;

	ret = 0;
	if( edid_info.establish_timing == 0 ){
		goto find_end;
	}

	// find established timing
	for(i=0;i<17;i++){
		if( edid_info.establish_timing & (0x1 << i) ){
			if( (resx == edid_establish_timing[i].resx) && (resy == edid_establish_timing[i].resy) ){
				if( freq == edid_establish_timing[i].freq ){
					ret = 1;
					goto find_end;
				}
			}
		}
	}

	// find standard timing
	for(i=0;i<8;i++){
		if( edid_info.standard_timing[i].resx == 0 )
			continue;
		if( (resx == edid_info.standard_timing[i].resx) && (resy == edid_info.standard_timing[i].resy) ){
			if( freq == edid_info.standard_timing[i].freq ){
				ret = 1;
				goto find_end;
			}
		}
	}

	// find detail timing
	for(i=0;i<4;i++){
		if( edid_info.detail_timing[i].resx == 0 )
			continue;
		if( (resx == edid_info.detail_timing[i].resx) && (resy == edid_info.detail_timing[i].resy) ){
			if( freq == edid_info.detail_timing[i].freq ){
				ret = 1;
				goto find_end;
			}
		}
	}
find_end:
	printk("[EDID] %s support %dx%d@%d\n",(ret)? "":"No",resx,resy,freq);
	return ret;
}

