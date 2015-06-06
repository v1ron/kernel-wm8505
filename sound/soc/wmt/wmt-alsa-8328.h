/*

	Some descriptions of such software.
    
	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.
	
	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.
	
	Authors: V1ron (Roman I. Volkov) Russian software developer.
	For bugs reports, questions, contact with me:
	Web: http://v1ron.ru
	E-mail: v1ron@mail.ru, 1991v1ron@gmail.com
	
	History:
	Thanks to WonderMedia for existing sources that helps me to start
	and chinese datasheets ;)

	- Oct 24, 2011: Version 1.0           
	1) Just works... (unreleased, for testing only, will be published in future). Used I2C helpers by Wondermedia.

	- Dec 2, 2011: Version 2.0                  
	1) Buffers is handled now by DMA controller. Used functions declared in dma.h from Wondermedia.

*/

#include <linux/config.h>
#include <sound/core.h>
#include <sound/pcm.h>
//#include <linux/platform_device.h>
#include <asm/dma.h>
#include <mach/dma.h>
//#include <mach/i2s_alsa.h>

///////////////////////////////////////////////////////
// Importing published functions from chip driver
///////////////////////////////////////////////////////

// returns chip name string
extern char* snd_chip_get_name(void);
// chip startup, returns error code
extern int snd_chip_startup(struct snd_card *card);
// chip shutdown
extern void snd_chip_shutdown(void);
// setup chip DAC sample rate, prepare chip for playback and return error code
extern int snd_chip_set_dac_rate(int rate, int *sysclk, int *sysclkdiv, int *mclkdiv, int *bclkdiv, int *lrclkdiv);
// setup chip ADC sample rate, prepare chip for recording and return error code
extern int snd_chip_set_adc_rate(int rate, int *sysclk, int *sysclkdiv, int *mclkdiv, int *bclkdiv, int *lrclkdiv);
// enter chip to standby mode
extern int snd_chip_suspend(void);
// leave chip from standby mode
extern int snd_chip_resume(void);

///////////////////////////////////////////////////////
// Hardware description
///////////////////////////////////////////////////////

#define I2S_INTERRUPT_VECTOR      35

// TX FIFO empty bit
#define I2S_STATUS_TX_TFE         1
// Tx FIFO almost empty bit
#define I2S_STATUS_TX_TFAE        2
// TX FIFO Underrun Error
#define I2S_STATUS_TX_TFUE        4
// TX FIFO Overron Error
#define I2S_STATUS_TX_TFOE        8

#define PMC_ADDR  (0xD8130000)
/* Power Management Control register */
#define GPIO_ADDR (0xD8110000)
/* GPIO Control register */
#define SCC_ADDR (0xD8120000)
/* System Configuration Control Register */
/* This register (offset 0) returns the hardware ID (vt8500\wm8505 platforms) */
/* Where... High 16-bit word is part number value, Low part is Major and metaL masks */

//#define I2S_BASE_ADDR 0xd8330000
/* I2S control registers base */

#define I2S_AUDCTLCR          0x00
/*
Provides control over the Audio Controller module
Bit        Attribute  Def   Description
31:30      RO         0     Reserved
29         RW         1b    Record Right Channel Enable.
                            0: Right Channel is disabled for record
                            1: Audio data for right channel is enabled to receive
28         RW         1b    Record Left Channel Enable
27         RW         0     RxSync_out Disable bit (only used in master mode)
                            0: normal
                            1: stopped and cannot go to high level, causing external 
                            device to power down
26:24      RW         000b  Sample bit for Received Data as record ----------(rxsample_bit)
                            000: 8 bit
                            001: 13 bit
                            010: 14 bit
                            011: 16 bit
                            100: 20 bit
                            101: 24 bit
                            110: 32 bit
                            111: reserved
23         RW         0     Bit clock polarity select for received data as record
                            0: normal polarity
                            1: inverted polarity
22         RO         0     Reserved
21:20      RW         00b   Rx DSP mode.
                            00: Short frame sync, and early mode.
                            01: Short frame sync, and late mode.
                            10: Long frame sync, and early mode
                            11: Long frame sync, and late mode
19:18      RW         00b   Rx Audio Data format
                            00: i2s mode
                            01 Right-justify mode
                            10: Left-justify mode
                            11: DSP mode
17         RW         0     Master/slave mode bit for Received data as record.
                            RxBCLK_in,RxSYNC_in are just active in the slave mode.
                            RxBCLK_out, RxSYNC_out are just active in the master mode
                            0: Rx in master mode 1: Rx in slave mode
16         RW         0     Select master clock polarity for TXMCLK and RXMCLK
                            0: The edge of Frame Sync Clock (txsync_out or rxsync_out)
                            is synchronized with the falling edge of master clock (txmclk or rxmclk)
                            1: On rising edge
15:14      RO         0     Reserved
13         RW         1b    Playback Right channel enable
                            0: Right channel is disabled for playback
                            1: Audio data for right channel is enabled to transmit
12         RW         1b    Playback left channel enable
11         RW         0     TxSync_out Disable bit (only used in master mode)
                            0: normal
                            1: stopped, causing external device to power down
10:8       RW         000b  Sample bit for Transmitted data as playback -------(txsample_bit)
                            000: 8 bit       111: reserved
                            001: 13          100: 20
                            010: 14          101: 24
                            011: 16          110: 32-bit
7          RW         0     Select bit clock polarity for transmitted data as playback
                            0: normal polarity
                            1: inverted polarity
6          RO         0     Reserved
5:4        RW         0     Tx DSP mode (see 21:20)
3:2        RW         0     Tx audio data format (see 19:18)
1          RW         0     Master\slave mode bit for transmitted data as playback
                            TxBCLK_in and TxSYNC_in are lust active in slave mode
                            TxBclk_out and TxSYNC_out are just active in master mode
                            0: tx in master mode      1: tx in slave mode
0          RW         0     I2S_v2 Enable bit
                            Enable to transfer audio data between the audio controller 
                            and external audio device. This bit also used to enable clock divisors
                            0: Disable           1: Enable
*/

#define I2S_AUDDFCR           0x04
/*
Bit        Attribute  Def   Description
31:15      RO         0     Reserved
14         RW         0     RX FIFO First bit setup bit
                            Identifies the order in which the valid data is received 
                            from the ADCIN port. This control bit is NOT affected by 
                            the programming of bit [10]
13         RW         0     RX FIFO Padding control bit ----------------(rx_pad)
                            Controls the no-effect bits of right-aligned data written
                            into the Receive FIFO from the valid data of ADCDIN.
                            The left-aligned data is always zero-padded. This bit is ignored
                            if bit[10] is zero.
12         RW         0     RX_FIFO Alignment control bit. ------------ (rd_al)
                            Controls the alignment of data written into the receive FIFO
                            from the data of the ADCDIN. This bit is ignored if bit[10]
                            is zero.
                            0: right aligned within RX FIFO
                            1: left aligned within RX FIFO
11         RW         0     ADCDIN data alignment setup bit
                            This bit identifies the left or right alignment of data being
                            received in from the ADCDIN port so the data can be properly
                            processed. This bit is ignored if bit [10] is zero.
                            0: Right aligned within received data.
                            1: Left aligned
10         RW         0     Data size in ADCIN per channel -------(rx_size)
                            0: Same with sample bit (no alignment)
                            1: When Rx sample bit is 8, 16 bit bits are received from ADCIN
                            ... Packet size aligned dy 16 bit.
9:7        RO         0     Reserved
6          RW         0     Select DACDOUT First bit
                            Controls the order in which the data is transmitted. This
                            control bit is NOT affected by the programmint of bit [2]
                            0: MSB bit first     1: LSB bit first
5          RW         0     TX FIFO padding control bit      (error in original datasheet...)
                            see [13]
4          RW         0     ADCOUT Data Alignment setup bit (error in original datasheet)
                            see [11]
3          RW         0     TX write data alignment setup bit --------(wr_al)
                            Identifies the left or right alignment of data on APB data
                            bus that will be written into the TX FIFO so the data can be
                            properly processed. This bit is ignored if bit [2] is zero.
                            0: right aligned on APB data bus bit[31:16] or bit [15:0]
                            1: left aligned
2          RW         0     Data size in DACDOUT per channel -----------(txsize)
                            0: Same with sample bit
                            1: With 16-bit padding (alignment)
1:0                         Reserved

Table: Data size of Playback data written from MCU or APB interface for each channel
tx_size          wr_al       Data size of playback
0                x           Defined by txsample_bit
1                0           Defined by txsample_bit
1                1           data size transmitted to DACOUT (as description in tx_size)

Table: Data size of record data read by MCU or APB interface for each channel
rx_size          {rd_al,rx_pad}      Data size of record
0                x                   defined by rxsample_bit
1                2'b00               defined by rxsample_bit
1                others              data size received from ADCDIN (as description in rx_size)
*/

#define I2S_TXCLKDIV          0x8
/*
Provides clock divisor control of the bit block and master clock for the transmitted data
Bit        Attribute  Def   Description
31:24                       Reserved
23:16      RW         0     TXMCLK Division value
                            This value is used to divide SYSCLK to the external master clock
                            output for playback according to the following equations.
                            txmclk_divInteger[ freq(SYSCLK[4])/freq(TXMCLK)+1/2 ]
                            freq(TXMCLK[5]) = freq(SYSCLK[4])/txmclk_div
                            note:
                            (1) The value [0,1] is the same as the value 1, TXMCLK = SYSCLK[4]
                            (2) The software does not need to program this value if TXMCLK is not used
                            (3) TXMCLK will be output once the modules enabled, no
                                matter it is in master mode or in slave mode.
                            (4) In MCU system, SYSCLK means sysclk_ply
                            (5) In SOC system, SYSCLK means audsysclk
                                In SOC system, TXMCLK means master_clk
15:11                       RESERVED
10:0       RW          0    TXBCLK Division Value
                            This value is used to divide SYSCLK[2] to the bit clock
                            TXBCLK[3] for playing according to the following equations.
                            txbclk_div = Integer[ freq(SYSCLK)/freq(TXBCLK) +1/2 ]
                            freq(TXBCLK[3])=freq(SYSCLK[2])/txbclk_div
                            it has following 2 functions:
                            - generating TXBCLK in master mode
                            - Controlling the DACDOUT shift out time to meet its output
                              delay timing
                            Note:
                            (1) The value [0,1] is the same as the value 1, TXBCLK = ~SYSCLK[2]
                            (2) In MCU system, sysclk means sysclk_ply
                                 In SOC system, SYSCLK means audsysclk
                            (3) In SOC system, TXBCLK means tx bit clk
 */

#define I2S_TXFRAMECR         0x0c
/*
This register provides control over the offset bit slot of the data beginning per tx frame,
and also defines the TxSync frequency.
Bit        Attribute  Def   Description
31:24      RO         0     Reserved
23:16      RW         0     Offset bit slot of the data beginning aech tx frame.
                            It defines the offset bit slot number of the data beginning on the
                            DACDOUT port. The default value is 0, which means the offset is 0 
                            and the data beginning in the normal mode.
                            This bit is ignored if txaud_mod = right-justify mode.
15:12      RO         0     Reserved
11:0       RW         040h  TxSYNC Clock Division Value.
                            This value is used to divide RXBCLK to the DAC sync clock (TxSYNC)
                            according to the following equations.
                            txsync_div = Integer[ freq(TXBCLK)/freq(TxSYNC) + 1/2 ]
                            freq(TxSYNC)=freq(TxBCLK)/txsync_div.
                            It has the following two functions:
                               - Generating RxSYNC_out in master mode
                               - Controlling the frequency of DAC sample rate
                            Note:
                              - The value [0,1] is the same with the value 2
                              - The default value is 64
                              - The software must correctly program this value no matter
                                in master mode or slave mode. If in slave mode, the
                                software should use the input TXBCLK and TxSYNC frequency to count this value.
                              - The minimum value of this register is the tx_size*channel
                                number. It is error if the configured value is smaller than
                                the min value.
                              - In WM8505, the bit width of this register is only 8 bit
*/

#define I2S_RXCLKDIV		0x10

#define I2S_RXFRAMECR         0x14
/* This register provides the control over the offset bit slot of the data beginning per rx
frame, and also defines the RxSync frequency 
Bit        Attribute  Def   Description
31:24      RO         0     Reserved
23:16      RW         0     Offset Bit Slot of the data beginning each Rx frame.
                            It defines the offset bit slot number of the data beginning on the ADCIN
                            port. The default value is 0, which means the offset is 0
                            and the data beginning in normal mode.
                            This bit is ignoring if Right justify mode
15:12      RO         0     Reserved
11:0       RW         040h  RxSYNC Clock Division value.
                            This value is used to divide RXBCLK to the ADC sync clock (RxSYNC)
                            according to the following equations.
                            rxsync_div = Integer[ freq(RXBCLK)/freq(RxSYNC) + 1/2 ]
                            freq(RxSYNC)=freq(RxBCLK)/rxsync_div.
                            It has the following two functions:
                               - Generating RxSYNC_out in master mode
                               - Controlling the frequency of ADC sample rate
                            Note:
                              - The value [0,1] is the same with the value 2
                              - The default value is 64
                              - The software must correctly program this value no matter
                                in master mode or slave mode. If in slave mode, the
                                software should use the input BCLK and RxSYNC frequency to count this value.
                              - The minimum value of this register is the rx_size*channel
                                number. It is error if the configured value is smaller than
                                the min value.
                              - In WM8505, the bit width of this register is only 8 bit
*/

#define I2S_TXEQURXCR      0x18
/*
Bit        Attribute  Def   Description
31:8       RO         0     Reserved
7          RW         0     Rx FIFO Software Flush.
                            When i2s_v2 stops receiving recording data from
                            the digital audio interface (determined by rx_en and rx_stop_immed),
                            RX FIFO will has a flush if this bit is 1. After this
                            flush is completed, it can read 1 from rx_rd_empty (0x2c[4]).
                            Then, it can write 0 to this bit to stop the flush.
                            0: End software flush for RxFIFO
                            1: Start
                            This bit is ignored if rxfifo_flush_en=0
6:1        RW         0     Reserved
0          RW         0     Tx and Rx have the Same configuration
                            Tx and Rx part of the i2s bus have the same audio mode, clock divisor 
                            and offset
                            1: When AUDCTLR[15:0], TXCLKDIV and TXFRAMECR are
                            configured, the same value is configured to AUDCTLR[31:16],
                            RXCLKDIV and TXFRAMECR, i.e. Tx part and Rx part has the same
                            audio mode, clock divisor and offset
*/

#define I2S_TCR            0x20
/*
Bit        Attribute  Def   Description
31:12      RO         0     Reserved
11:8       RW         8h    Tx FIFO Threshold
                            0000 = 16 entries
                            0001 = 1 entry
                            ...
                            1111 = 15 entries
7:6        RO         0     Reserved
5          RW         0     Tx FIFO Overrun Interrupt enable
4          RW         0     Tx FIFO underrun interrupt enable
3          RW         0     Tx FIFO Almost empty DMA request enable
2          RW         0     Tx FIFO Almost empty Interrupt enable
1          RW         0     Tx FIFO Empty Interrupt enable
0          RW         0     Tx FIFO enable. To start playback, 
                            please set this bit after all other registers have been set.
*/

#define I2S_TSR            0x24
/*
RSR provides status for the Tx FIFO
Bit        Attribute  Def   Description
31:4       RO         0     reserved
3          RW1C       0     Tx FIFO Overrun
2          RW1C       0     TX Underrun
1          RO         0     TX FIFO almost empty
                            Bit indicates if the number of free entries
                            is equal or greater Threshold
0          RO         0     RX FIFO empty
*/

#define I2S_RCR            0x28
/*
Bit        Attribute  Def   Description
31:12      RO         0     Reserved
11:8       RW         8h    Rx FIFO Threshold
                            0000 = 16 entries
                            0001 = 1 entry
                            ...
                            1111 = 15 entries
7:6        RO         0     Reserved
5          RW         0     Rx FIFO Overrun Interrupt enable
4          RW         0     Rx FIFO underrun interrupt enable
3          RW         0     Rx FIFO Almost full DMA request enable
2          RW         0     Rx FIFO Almost Full Interrupt enable
1          RW         0     Rx FIFO Full Interrupt enable
0          RW         0     Rx FIFO Enable
                            To Start recording, please enable this bit
                            after all other registers have set.
*/
#define I2S_RSR            0x2c
/*
RSR provides status for the Rx FIFO
Bit        Attribute  Def   Description
31:4       RO         0     reserved
3          RW1C       0     Rx FIFO Overrun
2          RW1C       0     RX Underrun
1          RO         0     RX FIFO almost full
                            Bit indicates if the number of occupied entries
                            is equal or greater Threshold
0          RO         0     RX FIFO full
*/

#define I2S_TFIFO          0x80

#define I2S_RFIFO          0xc0

///////////////////////////////////////////////////
// private structures
///////////////////////////////////////////////////

// making of MCLK division value
#define WMT_I2S_MCLK_DIV_MAKE(x)		(0x00FF0000 & (x << 16))

// Making of BCLK division value
#define WMT_I2S_BCLK_DIV_MAKE(x)		(0x000007FF & x)

// Making of LRCLK division value
#define WMT_I2S_LRCLK_DIV_MAKE(x)		(0x00000FFF & x)

typedef struct {
	struct snd_pcm_substream *pss;		/* link to playback stream structure */
	struct snd_pcm_substream *css;		/* link to capture stream structure */
	int playback_buf_dev_pos;		/* positions, where DMA actually read or write data */
	int capture_buf_dev_pos;
	dmach_t dmach_tx;			/* DMA channel numbers */
	dmach_t dmach_rx;
	struct dma_device_cfg_s dma_cfg;	/* DMA device config */
} snd_i2s_private_data;

// SYSCLK(AUDCLK) values
#define WMT_I2S_SYSCLK_16_3835_MHZ		0
#define WMT_I2S_SYSCLK_22_5789_MHZ		0x00080000
#define WMT_I2S_SYSCLK_36_8653_MHZ		0x00100000
#define WMT_I2S_SYSCLK_24_5755_MHZ		0x00040000
#define WMT_I2S_SYSCLK_33_8684_MHZ		0x000C0000

// SYSCLK division values.
#define WMT_I2S_SYSCLK_DIV_1			0x00010000
#define WMT_I2S_SYSCLK_DIV_2			0x00020000
#define WMT_I2S_SYSCLK_DIV_4			0x00030000