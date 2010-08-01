


/*------------------------------------------------------------------------------
    Following definitions, please refer spec of EDID. You may refer it on 
    http://en.wikipedia.org/wiki/EDID#EDID_1.3_data_format   
------------------------------------------------------------------------------*/

#define EDID_LENGTH                             0x80

/*------------------------------------------------------------------------------
  Offset 00-19: HEADER INFORMATION
------------------------------------------------------------------------------*/
/*  00¡V07: Header information "00h FFh FFh FFh FFh FFh FFh 00h" */  
#define EDID_HEADER                             0x00
#define EDID_HEADER_END                         0x07

/*  08¡V09: Manufacturer ID. These IDs are assigned by Microsoft. 
         "00001=A¡¨; ¡§00010=B¡¨; ... ¡§11010=Z¡¨. Bit 7 (at address 08h) is 0, the first
         character (letter) is located at bits 6 ¡÷ 2 (at address 08h), the second character
         (letter) is located at bits 1 & 0 (at address 08h) and bits 7 ¡÷ 5 (at address 09h),
         and the third character (letter) is located at bits 4 ¡÷ 0 (at address 09h).
*/         
#define ID_MANUFACTURER_NAME                    0x08
#define ID_MANUFACTURER_NAME_END                0x09

/*  10¡V11: Product ID Code (stored as LSB first). Assigned by manufacturer */
#define ID_MODEL				                0x0a

/*  12¡V15: 32-bit Serial Number. No requirement for the format. Usually stored as LSB first. In
         order to maintain compatibility with previous requirements the field should set at
         least one byte of the field to be non-zero if an ASCII serial number descriptor is
         provided in the detailed timing section.
*/         
#define ID_SERIAL_NUMBER			            0x0c

/*  16: Week of Manufacture. This varies by manufacturer. One way is to count January 1-7 as
      week 1, January 8-15 as week 2 and so on. Some count based on the week number
      (Sunday-Saturday). Valid range is 1-54.
    17: Year of Manufacture. Add 1990 to the value for actual year. */
#define MANUFACTURE_WEEK			            0x10
#define MANUFACTURE_YEAR			            0x11

/*  18: EDID Version Number. "01h"
    19: EDID Revision Number "03h"  */    
#define EDID_STRUCT_VERSION                     0x12
#define EDID_STRUCT_REVISION                    0x13

/*------------------------------------------------------------------------------
  Offset 20-24: BASIC DISPLAY PARAMETERS
------------------------------------------------------------------------------*/


#define DPMS_FLAGS				                0x18
#define ESTABLISHED_TIMING_I                    0x23
#define ESTABLISHED_TIMING_II                   0x24
#define MANUFACTURERS_TIMINGS                   0x25

#define STANDARD_TIMING_IDENTIFICATION_START    0x26
#define STANDARD_TIMING_IDENTIFICATION_SIZE     16

#define DETAILED_TIMING_DESCRIPTIONS_START      0x36
#define DETAILED_TIMING_DESCRIPTION_SIZE        18
#define NO_DETAILED_TIMING_DESCRIPTIONS         4

#define DETAILED_TIMING_DESCRIPTION_1           0x36
#define DETAILED_TIMING_DESCRIPTION_2           0x48
#define DETAILED_TIMING_DESCRIPTION_3           0x5a
#define DETAILED_TIMING_DESCRIPTION_4           0x6c

typedef struct {
	unsigned int resx;
	unsigned int resy;
	int freq;
} edid_timing_t;

typedef struct {
	unsigned int establish_timing;
	edid_timing_t standard_timing[8];
	edid_timing_t detail_timing[4];
	unsigned int pixel_clock_limit;
} edid_info_t;

extern int parse_edid( unsigned char * edid );
extern int edid_find_support(unsigned int resx,unsigned int resy,int freq);

