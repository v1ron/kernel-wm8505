/*++
	linux/drivers/serial/serial_wmt.c

	Some descriptions of such software. Copyright (c) 2008  WonderMedia Technologies, Inc.

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

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/mach/serial_wmt.h>

#include <linux/serial_core.h>


#define PORT_WMT 54
#if defined(CONFIG_SERIAL_WMT_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

/*
 *  Debug macros
 */
#ifdef DEBUG
#define DPRINTK(fmt, args...)   printk(KERN_ERR "%s: " fmt, __func__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#define UART_BR_115K2 115200

/*
 * This is for saving useful I/O registers
 */

/*
 * RS232 on DB9
 * Pin No.      Name    Notes/Description
 * 1            DCD     Data Carrier Detect
 * 2            RD      Receive Data (a.k.a RxD, Rx)
 * 3            TD      Transmit Data (a.k.a TxD, Tx)
 * 4            DTR     Data Terminal Ready
 * 5            SGND    Ground
 * 6            DSR     Data Set Ready
 * 7            RTS     Request To Send
 * 8            CTS     Clear To Send
 * 9            RI      Ring Indicator
 */

/*
 * We've been assigned a range on the "Low-density serial ports" major
 *
 * dev          Major           Minor
 * ttyVT0       204             90
 * ttyS0        4               64
 */
#ifdef CONFIG_SERIAL_WMT_TTYVT
		#define SERIAL_WMT_MAJOR     204
		#define MINOR_START             90      /* Start from ttyVT0 */
		#define CALLOUT_WMT_MAJOR    205     /* for callout device */
#else
		#define SERIAL_WMT_MAJOR     4
		#define MINOR_START             64      /* Start from ttyS0 */
		#define CALLOUT_WMT_MAJOR    5       /* for callout device */
#endif

#define NR_PORTS                6       /* UART0 to UART5 */
#define WMT_ISR_PASS_LIMIT   256

struct wmt_port {
	struct uart_port	port;
	struct timer_list	timer;
	unsigned int		old_status;
};


/*
 * WMT UART registers set structure.
 */
struct wmt_uart {
	unsigned int volatile urtdr;            /* 0x00*/
	unsigned int volatile urrdr;            /* 0x04*/
	unsigned int volatile urdiv;            /* 0x08*/
	unsigned int volatile urlcr;            /* 0x0C*/
	unsigned int volatile uricr;            /* 0x10*/
	unsigned int volatile urier;            /* 0x14*/
	unsigned int volatile urisr;            /* 0x18*/
	unsigned int volatile urusr;            /* 0x1C*/
	unsigned int volatile urfcr;            /* 0x20*/
	unsigned int volatile urfidx;           /* 0x24*/
	unsigned int volatile urbkr;            /* 0x28*/
	unsigned int volatile urtod;            /* 0x2C*/
	unsigned int volatile resv30_FFF[0x3F4];	/* 0x0030 - 0x0FFF Reserved*/
	unsigned char volatile urtxf[32];       /* 0x1000 - 0x101F*/
	unsigned short volatile urrxf[16];      /* 0x1020 - 0x103F*/
};

struct baud_info_s {
	unsigned int baud;		/* baud rate */
	unsigned int brd;		/* baud rate divisor */
	unsigned int bcv;		/* break counter value at this baud rate
							 * simply be calculated by baud * 0.004096
							 */
};


static struct baud_info_s baud_table[] =
{
	{   3600,  0x100FF,    15 },
	{   7600,  0x1007F,    30 },
	{   9600,  0x2003F,    39 },
	{  14400,  0x1003F,    59 },
	{  19200,  0x2001F,    79 },
	{  28800,  0x1001F,   118 },
	{  38400,  0x2000F,   157 },
	{  57600,  0x1000F,   236 },
	{ 115200,   0x10007,   472 },
	{ 230400,   0x10003,   944 },
	{ 460800,   0x10001,  1920 },
	{ 921600,   0x10000,  3775 },
};

#define BAUD_TABLE_SIZE                 ARRAY_SIZE(baud_table)

/*
 * Macros to put URISR and URUSR into a 32-bit status variable
 * URISR in bit[ 0:15]
 * URUSR in bit[16:31]
 */
#define URISR_TO_SM(x)                  ((x) & URISR_MASK)
#define URUSR_TO_SM(x)                  (((x) & URUSR_MASK) << 16)
#define SM_TO_URISR(x)                  ((x) & 0xffff)
#define SM_TO_URUSR(x)                  ((x) >> 16)

/*
 * Following is a trick if we're interesting to listen break signal,
 * but due to WMT UART doesn't suppout this interrupt status.
 * So I make a fake interrupt status and use URISR_FER event to implement
 * break signal detect.
 */
#ifdef CONFIG_SERIAL_WMT_BKSIG
#define SW_BKSIG                        (BIT31 | URISR_FER)
#endif
/*
 * Macros to manipulate WMT UART module.
 *
 * s = sport, o = offset, v = value
 *
 * registers offset table as follows:
 *
 * URTDR           0x0000
 * URRDR           0x0004
 * URBRD           0x0008
 * URLCR           0x000C
 * URICR           0x0010
 * URIER           0x0014
 * URISR           0x0018
 * URUSR           0x001C
 * URFCR           0x0020
 * URFIDX          0x0024
 * URBKR           0x0028
 *
 * Offset 0x002C-0x002F reserved
 *
 * URTXF           0x0030
 * URRXF           0x0040
 */
#define PORT_TO_BASE(s)                 ((s)->port.membase)
#define WMT_UART_GET(s, o)           __raw_readl((s)->port.membase + o)
#define WMT_UART_PUT(s, o, v)        __raw_writel(v, (s)->port.membase + o)
#define WMT_UART_TXFIFO(s)           (volatile unsigned char *)((s)->port.membase + URTXF)
#define WMT_UART_RXFIFO(s)           (volatile unsigned short *)((s)->port.membase + URRXF)

/*
 * This is the size of our serial port register set.
 */
#define UART_PORT_SIZE  0x1040

/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo incase CTS has been dropped.
 */
#define MCTRL_TIMEOUT   (250*HZ/1000)

/*{2007/11/10 JHT Support the VT8500 Serial Port Driver Because the*/
/*                definition of URSIRT Bit[0] & Bit[3] are different.*/
/*                Before VT8500 these bit are defined as RO, in VT8500*/
/*                they are changed into the W1C. Therefore the xmit function*/
/*                for the FIFO mode should be modified as well.*/
static void wmt_tx_chars(struct wmt_port *sport);
/*}2007/11/10-JHT*/

static void write_ch_to_uart(unsigned char ch)
{
	struct wmt_uart *pUart_Reg = (struct wmt_uart *)0xd8210000;

	/* wayne 2006-11-29*/
	if (ch == '\n')
		write_ch_to_uart('\r');

	while (readl(&pUart_Reg->urusr) & URUSR_TXDBSY)
		;

	writeb(ch, &pUart_Reg->urtdr);
}

#if 0
/**/
/*	[Description]*/
/*		put a string end with null character into the uart0(consol)*/
/**/
/*	[Arguments]*/
/*		str : string pointer for the uart output.*/
/**/
static void uputs(char *str)
{
	unsigned long i, len;
	unsigned char *p;

	for (p = str, len = 0; *p != 0; p++, len++)
		;    /* calculate string size*/

	for (p = str, i = 1; i <= len; p++, i++)
		write_ch_to_uart(*p);
}
#endif
/**/
/*	[Description]*/
/*		print a format string to UART0(console)*/
/**/
/*	[Arguments]*/
/*		format : formatted string*/
/**/
/*
static void uprintf(char *format, ...)
{
	va_list args;
	char buffer[255];
	unsigned int size;

	memset(buffer, 0, 255);
	va_start(args, format);
	size = vsprintf(buffer, format, args);
	if (size > 252) {
		uputs("string too long (>252)\r\n");
		goto end;
	}
	uputs(buffer);
end:
	va_end(args);

}
*/
static void wmt_mctrl_check(struct wmt_port *sport)
{
	unsigned int status, changed;

	status = sport->port.ops->get_mctrl(&sport->port);
	changed = status ^ sport->old_status;

	if (changed == 0)
		return;

	sport->old_status = status;

	if (changed & TIOCM_RI)
		sport->port.icount.rng++;
	if (changed & TIOCM_DSR)
		sport->port.icount.dsr++;
	if (changed & TIOCM_CAR)
		uart_handle_dcd_change(&sport->port, status & TIOCM_CAR);
	if (changed & TIOCM_CTS)
		uart_handle_cts_change(&sport->port, status & TIOCM_CTS);

	wake_up_interruptible(&sport->port.info->delta_msr_wait);
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */
static void wmt_timeout(unsigned long data)
{
	struct wmt_port *sport = (struct wmt_port *)data;

	unsigned long flags;

	if (sport->port.info) {
		spin_lock_irqsave(&sport->port.lock, flags);
		wmt_mctrl_check(sport);
		spin_unlock_irqrestore(&sport->port.lock, flags);
		mod_timer(&sport->timer, jiffies + MCTRL_TIMEOUT);
	}
}

/*
 * Interrupts should be disabled on entry.
 */
static void wmt_stop_tx(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	uart->urier &= ~(URIER_ETXFAE | URIER_ETXFE);
	sport->port.read_status_mask &= ~URISR_TO_SM(URISR_TXFAE | URISR_TXFE);
}

/*
 * Interrupts may not be disabled on entry.
 */
static void wmt_start_tx(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
//	unsigned long flags;

//	spin_lock_irqsave(&sport->port.lock, flags); // Vincent Li mark out for CONFIG_PREEMPT_RT

	/*{2007/11/10 JHT Support the VT8500 Serial Port Driver Because the
	 *                definition of URSIRT Bit[0] & Bit[3] are different.
	 *                Before VT8500 these bit are defined as RO, in VT8500
	 *                they are changed into the W1C. Therefore the xmit function
	 *                for the FIFO mode should be modified as well.
	 */
	uart->urier &= ~(URIER_ETXFAE | URIER_ETXFE);
	wmt_tx_chars(sport);
	/*}2007/11/10-JHT*/

	sport->port.read_status_mask |= URISR_TO_SM(URISR_TXFAE | URISR_TXFE);
	uart->urier |= URIER_ETXFAE | URIER_ETXFE;

//	spin_unlock_irqrestore(&sport->port.lock, flags);// Vincent Li mark out for CONFIG_PREEMPT_RT
}

/*
 * Interrupts enabled
 */
static void wmt_stop_rx(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	uart->urier &= ~URIER_ERXFAF;
}

/*
 * No modem control lines
 */
static void wmt_enable_ms(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;

	mod_timer(&sport->timer, jiffies);
}

/*
 * Inside the UART interrupt service routine dut to following
 * reason:
 *
 * URISR_RXFAF:  RX FIFO almost full (FIFO mode)
 * URISR_RXDF:   RX data register full (Register mode)
 * URISR_RXTOUT: RX timeout
 */
int debug_ch;
static void
wmt_rx_chars(struct wmt_port *sport, unsigned int status)
{
//	struct tty_struct *tty = sport->port.info->tty;
	struct tty_struct *tty = sport->port.info->port.tty;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
	unsigned int  flg, urfidx, ignored = 0;
	int ch;

	urfidx = URFIDX_RXFIDX(uart->urfidx);

	/*
	 * Check if there is data ready to be read.
	 *
	 * Note: We only receive characters.
	 */
	while ((status & URUSR_TO_SM(URUSR_RXDRDY)) && (urfidx > 0)) {
		ch = (unsigned int)(uart->urrxf[0] & 0x3FF);
		debug_ch = ch;
		urfidx--;
#if 0  /* need fix */
		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			goto ignore_char;
#endif
		sport->port.icount.rx++;

		flg = TTY_NORMAL;

		/*
		 * Check interrupt status information using status[URISR_bits].
		 *
		 * Notice that the error handling code is out of
		 * the main execution path and the URISR has already
		 * been read by ISR.
		 */
		if (status & URISR_TO_SM(URISR_PER | URISR_FER | URISR_RXDOVR)) {
			if (urfidx > 1) {
				if (uart_handle_sysrq_char(&sport->port, ch))
					goto ignore_char;
				/*
				 * Pop all TTY_NORMAL data.
				 */
				goto error_return;
			} else {
				/*
				 * Now we have poped up to the data with
				 * parity error or frame error.
				 */
				goto handle_error;
			}
		}

		if (uart_handle_sysrq_char(&sport->port, ch))
			goto ignore_char;

error_return:
#if 0
			*tty->flip.flag_buf_ptr++ = flg;
			*tty->flip.char_buf_ptr++ = ch;
			tty->flip.count++;
#endif
			uart_insert_char(&sport->port, ch, URISR_TO_SM(URISR_RXDOVR) , ch, flg);

ignore_char:
			status &= 0xffff;       /* Keep URISR field*/
			status |= URUSR_TO_SM(uart->urusr);
	}
out:
	tty_flip_buffer_push(tty);
	return;

handle_error:
	/*
	 * Update error counters.
	 */
	if (status & URISR_TO_SM(URISR_PER))
		sport->port.icount.parity++;
	else
		if (status & URISR_TO_SM(URISR_FER)) {

#ifdef CONFIG_SERIAL_WMT_BKSIG
			/*
			 * Experimental software patch for break signal detection.
			 *
			 * When I got there is a frame error in next frame data,
			 * I check the next data to judge if it is a break signal.
			 *
			 * FIXME: Open these if Bluetooth or IrDA need this patch.
			 *        Dec.29.2004 by Harry.
			 */
			if ((ch & RX_PERMASK) == 0) {
				sport->port.icount.brk++;
				uart_handle_break(&sport->port);
			} else
				sport->port.icount.frame++;

#else   /* Don't support break sinal detection */

			sport->port.icount.frame++;

#endif

		}

	/*
	 * RX Over Run event
	 */
	if (status & URISR_TO_SM(URISR_RXDOVR))
		sport->port.icount.overrun++;

	if (status & sport->port.ignore_status_mask) {
		if (++ignored > 100)
			goto out;
		goto ignore_char;
	}

	/*
	 * Second, handle the events which we're interesting to listen.
	 */
	status &= sport->port.read_status_mask;

	if (status & URISR_TO_SM(URISR_PER))
		flg = TTY_PARITY;
	else
		if (status & URISR_TO_SM(URISR_FER)) {

#ifdef CONFIG_SERIAL_WMT_BKSIG
			/* Software patch for break signal detection.
			 *
			 * When I got there is a frame error in next frame data,
			 * I check the next data to judge if it is a break signal.
			 *
			 * FIXME: Open these if Bluetooth or IrDA need this patch.
			 *        Dec.29.2004 by Harry.
			 */
			if (sport->port.read_status_mask & SW_BKSIG) {
				if ((ch & RX_PERMASK) == 0) {
					DEBUG_INTR("handling break....");
					flg = TTY_BREAK;
					/*goto error_return;*/
				} else {
					flg = TTY_FRAME;
					/*goto error_return;*/
				}
			} else {
				flg = TTY_FRAME;
				/*goto error_return;*/
			}

#else   /* Don't support break sinal detection */

			flg = TTY_FRAME;

#endif
		}

	if (status & URISR_TO_SM(URISR_RXDOVR)) {
		/*
		 * Overrun is special, since it's reported
		 * immediately, and doesn't affect the current
		 * character.
		 */
#if 0
		*tty->flip.flag_buf_ptr++ = flg;
		*tty->flip.char_buf_ptr++ = ch;
		tty->flip.count++;
		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			goto ignore_char;
#endif
		ch = 0;
		flg	= TTY_OVERRUN;
	}
#ifdef SUPPORT_SYSRQ
	sport->port.sysrq = 0;
#endif
	goto error_return;
}

/*
 * Inside the UART interrupt service routine dut to following
 * reason:
 *
 * URISR_TXFAE: TX FIFO almost empty (FIFO mode)
 * URISR_TXFE:  TX FIFO empty(FIFO mode)
 */
static void wmt_tx_chars(struct wmt_port *sport)
{
	struct circ_buf *xmit = &sport->port.info->xmit;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	if (sport->port.x_char) {
		/*
		 * Fill character to the TX FIFO entry.
		 */
		uart->urtxf[0] = sport->port.x_char;
		sport->port.icount.tx++;
		sport->port.x_char = 0;
		return;
	}

	/*Check the modem control lines before transmitting anything.*/
	wmt_mctrl_check(sport);

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		wmt_stop_tx(&sport->port);
		return;
	}

	/*{2007/11/10 JHT Support the WMT Serial Port Driver Because the
	 *                definition of URSIRT Bit[0] & Bit[3] are different.
	 *                Before WMT these bit are defined as RO, in WMT
	 *                they are changed into the W1C. Therefore the xmit function
	 *                for the FIFO mode should be modified as well.
	 */
	while ((uart->urfidx & 0x1F) < 16) {
		if (uart_circ_empty(xmit))
			break;

		if (uart->urusr & URUSR_TXDBSY)
			continue;
		uart->urtxf[0] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}
	/*}2007/11/10-JHT*/

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		wmt_stop_tx(&sport->port);
}

static irqreturn_t wmt_int(int irq, void *dev_id)
{
	struct wmt_port *sport = dev_id;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
	unsigned int status, pass_counter = 0;

	spin_lock(&sport->port.lock);

	/*
	 * Put interrupts status information to status bit[0:15]
	 * Put UART status register to status bit[16:31].
	 */
	status = URISR_TO_SM(uart->urisr) | URUSR_TO_SM(uart->urusr);
	uart->urisr |= SM_TO_URISR(status);

	do {
		/*
		 * First, we handle RX events.
		 *
		 * RX FIFO Almost Full.         (URUSR_RXFAF)
		 * RX Timeout.                  (URISR_RXTOUT)
		 * Frame error                  (URISR_FER)
		 *
		 * Note that also allow URISR_FER and URISR_PER event to do rx.
		 */
		if (status & URISR_TO_SM(URISR_RXFAF | URISR_RXFF | URISR_RXTOUT |\
								 URISR_PER | URISR_FER)) {
			wmt_rx_chars(sport,  status);
		}
		/*
		 * Second, we handle TX events.
		 *
		 * If there comes a TX FIFO Almost event, try to fill TX FIFO.
		 */
		if (status & URISR_TO_SM(URISR_TXFAE | URISR_TXFE))
			wmt_tx_chars(sport);

		if (pass_counter++ > WMT_ISR_PASS_LIMIT)
			break;

		/*
		 * Update UART interrupt status and general status information.
		 */
		status = (URISR_TO_SM(uart->urisr) | URUSR_TO_SM(uart->urusr));
		uart->urisr |= SM_TO_URISR(status);

		/*
		 * Inside the loop, we handle events that we're interesting.
		 */
		status &= sport->port.read_status_mask;

		/*
		 * Continue loop while following condition:
		 *
		 * TX FIFO Almost Empty.        (URISR_TXFAE)
		 * RX FIFO Almost Full.         (URISR_RXFAF)
		 * RX Receive Time Out.         (URISR_RXTOUT)
		 */
	} while (status & (URISR_TXFE | URISR_TXFAE | URISR_RXFAF | URISR_RXFF | URISR_RXTOUT));

	spin_unlock(&sport->port.lock);

	return IRQ_HANDLED;
}

/* wmt_tx_empty()
 *
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int wmt_tx_empty(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	return (uart->urusr & URUSR_TXDBSY) ? 0 : TIOCSER_TEMT;
}

/* wmt_get_mctrl()
 *
 * Returns the current state of modem control inputs.
 *
 * Note: Only support CTS now.
 */
static u_int wmt_get_mctrl(struct uart_port *port)
{
	u_int ret = TIOCM_DSR | TIOCM_CAR;
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	ret |= (uart->urusr & URUSR_CTS) ? TIOCM_CTS : 0;

	return ret;
}

/* wmt_set_mctrl()
 *
 * This function sets the modem control lines for port described
 * by 'port' to the state described by mctrl. More detail please
 * refer to Documentation/serial/driver.
 *
 * Note: Only support RTS now.
 */
static void wmt_set_mctrl(struct uart_port *port, u_int mctrl)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	if (mctrl & TIOCM_RTS)
		uart->urlcr |= URLCR_RTS;
	else
		uart->urlcr &= ~URLCR_RTS;
}

/*
 * Interrupts always disabled.
 */
static void wmt_break_ctl(struct uart_port *port, int break_state)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);

	if (break_state == -1) {
		int i;
		unsigned int urbrd = URBRD_BRD(uart->urdiv);

		/*
		 * This looks something tricky.
		 * Anyway, we need to get current baud rate divisor,
		 * search bcv in baud_table[], program it into
		 * URBKR, then generate break signal.
		 */
		for (i = 0; i < BAUD_TABLE_SIZE; i++) {
			if ((baud_table[i].brd & URBRD_BRDMASK) == urbrd)
				break;
		}

		if (i < BAUD_TABLE_SIZE) {
			uart->urbkr = URBKR_BCV(baud_table[i].bcv);
			uart->urlcr |= URLCR_BKINIT;
		}
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static char *wmt_uartname[] = {
	"uart0",
	"uart1",
	"uart2",
	"uart3",
	"uart4",
	"uart5"
};


static int wmt_startup(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
	char *uartname = NULL;
	int retval;
	/*unsigned int tmp_irisr;*/
	int i;

	/*
	 * Disable uart GPIO mode.
	 */
	gpio_disable(GPIO_UART0_RTS);
	gpio_disable(GPIO_UART0_TXD);
	gpio_disable(GPIO_UART0_CTS);
	gpio_disable(GPIO_UART0_RXD);
	gpio_disable(GPIO_UART1_RTS);
	gpio_disable(GPIO_UART1_TXD);
	gpio_disable(GPIO_UART1_CTS);
	gpio_disable(GPIO_UART1_RXD);
	gpio_disable(GPIO_UART2_RTS);
	gpio_disable(GPIO_UART2_TXD);
	gpio_disable(GPIO_UART2_CTS);
	gpio_disable(GPIO_UART2_RXD);
	//gpio_disable(GPIO_UART3_RTS);
	//gpio_disable(GPIO_UART3_TXD);
	//gpio_disable(GPIO_UART3_CTS);
	//gpio_disable(GPIO_UART3_RXD);

	/*
	 * Enable bus to uart clock
	 */

	/*
	 * Find uart name for request_irq
	 */

	switch (sport->port.irq) {

	case IRQ_UART0:
		uartname = wmt_uartname[0];
		break;
	case IRQ_UART1:
		uartname = wmt_uartname[1];
		break;
	case IRQ_UART2:
		uartname = wmt_uartname[2];
		break;
	case IRQ_UART3:
		uartname = wmt_uartname[3];
		break;
	case IRQ_UART4:
		uartname = wmt_uartname[4];
		break;
	case IRQ_UART5:
		uartname = wmt_uartname[5];
		break;
	}

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(sport->port.irq, wmt_int, 0, uartname, sport);
	if (retval)
		return retval;

	/*
	 * Setup the UART clock divisor
	 */
	for (i = 0; i < BAUD_TABLE_SIZE; i++) {
		if (baud_table[i].baud == 115200)
			break;
	}
	uart->urdiv = baud_table[i].brd;


	/* Disable TX,RX*/
	uart->urlcr = 0;
	/* Disable all interrupt*/
	uart->urier = 0;
	while (uart->urfcr) {
		/*Reset TX,RX Fifo*/
		uart->urfcr = URFCR_TXFRST | URFCR_RXFRST;
	}

	/* Disable Fifo*/
	uart->urfcr &= ~(URFCR_FIFOEN);

	uart->urlcr |=  (URLCR_DLEN & ~URLCR_STBLEN & ~URLCR_PTYEN);

	/* Enable Fifo, Tx 8 , Rx 8*/
	uart->urfcr = URFCR_FIFOEN | URFCR_TXFLV(0) | URFCR_RXFLV(0);
	/* Enable Fifo, Tx 8 , Rx 8*/
	uart->urlcr |= URLCR_RXEN | URLCR_TXEN;

	/*
	 * Enable RX FIFO almost full, timeout, and overrun interrupts.
	 */
	uart->urier = URIER_ERXFAF | URIER_ERXTOUT | URIER_EPER | URIER_EFER | URIER_ERXDOVR;

	/*
	 * Enable modem status interrupts
	 */
	spin_lock_irq(&sport->port.lock);
	wmt_enable_ms(&sport->port);
	spin_unlock_irq(&sport->port.lock);

	return 0;
}

static void wmt_shutdown(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	/*
	 * Stop our timer.
	 */
	del_timer_sync(&sport->timer);

	/*
	 * Free the allocated interrupt
	 */
	free_irq(sport->port.irq, sport);

	/*
	 * Disable all interrupts, port and break condition.
	 */
	uart->urier &= ~(URIER_ETXFE | URIER_ETXFAE | URIER_ERXFF | URIER_ERXFAF);
#if 0 /*under tx this will cause data loss, because the FIFO may still have data and this will disable tx*/
#ifdef CONFIG_SERIAL_WMT_CONSOLE
	/* Skip shutdown UART0 */
	if (sport->port.irq != IRQ_UART0)
		uart->urlcr &= ~(URLCR_TXEN | URLCR_RXEN);
#else
	uart->urlcr &= ~(URLCR_TXEN | URLCR_RXEN);
#endif
#endif

}

/* wmt_uart_pm()
 *
 * Switch on/off uart in powersave mode.
 *
 * Hint: Identify port by irq number.
 */
static void wmt_uart_pm(struct uart_port *port, u_int state, u_int oldstate)
{
	int channel = port->irq - IRQ_UART0;

	/* Skip UART0 */
	if (channel) {
		#if 0 /*williamfan*/
		if (state)
			/* Disable bus to uart clock.*/
			PMRS_VAL &= ~(PMAC_UART0 << channel);  /*RichardDBG*/
		else
			/* Enable bus to uart clock.*/
			PMRS_VAL |= (PMAC_UART0 << channel);   /*RichardDBG*/
		#endif
	}
}

static void
wmt_set_termios(struct uart_port *port, struct termios *termios, struct termios *old)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
	unsigned long flags;
	unsigned int new_urlcr, old_urlcr, old_urier, tmp_urisr, baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	int i;

	/*
	 * If we don't support modem control lines, don't allow
	 * these to be set.
	 */
	if (0) {
		termios->c_cflag &= ~(HUPCL | CRTSCTS | CMSPAR);
		termios->c_cflag |= CLOCAL;
	}
	/*
	 * Only support CS7 and CS8.
	 */
	while ((termios->c_cflag & CSIZE) != CS7 && (termios->c_cflag & CSIZE) != CS8) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8)
		new_urlcr = URLCR_DLEN;
	else
		new_urlcr = 0;

	if (termios->c_cflag & CRTSCTS)
		new_urlcr |= URLCR_RTS;
	else
		new_urlcr &= ~URLCR_RTS;

	if (termios->c_cflag & CSTOPB)
		new_urlcr |= URLCR_STBLEN;

	if (termios->c_cflag & PARENB) {
		/*
		 * Enable parity.
		 */
		new_urlcr |= URLCR_PTYEN;

		/*
		 * Parity mode select.
		 */
		if (termios->c_cflag & PARODD)
			new_urlcr |= URLCR_PTYMODE;
	}

	/*
	 * Ask the core to get baud rate, but we need to
	 * calculate quot by ourself.
	 */
	baud = uart_get_baud_rate(port, termios, old, 9600, 921000);

	/*
	 * We need to calculate quot by ourself.
	 *
	 * FIXME: Be careful, following result is not an
	 *        interger quotient, fix it if need.
	 */
	/*quot = port->uartclk / (13 * baud);*/

	spin_lock_irqsave(&sport->port.lock, flags);

	/*
	 * Mask out other interesting to listen expect TX FIFO almost empty event.
	 */
	sport->port.read_status_mask &= URISR_TO_SM(URISR_TXFAE | URISR_TXFE);

	/*
	 * We're also interested in receiving RX FIFO events.
	 */
	sport->port.read_status_mask |= URISR_TO_SM(URISR_RXDOVR | URISR_RXFAF | URISR_RXFF);

	/*
	 * Check if we need to enable frame and parity error events
	 * to be passed to the TTY layer.
	 */
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |= URISR_TO_SM(URISR_FER | URISR_PER);

#ifdef CONFIG_SERIAL_WMT_BKSIG
	/*
	 * check if we need to enable break events to be passed to the TTY layer.
	 */
	if (termios->c_iflag & (BRKINT | PARMRK))
		/*
		 * WMT UART doesn't support break signal detection interrupt.
		 *
		 * I try to implement this using URISR_FER.
		 */
		sport->port.read_status_mask |= SW_BKSIG;
#endif
	/*
	 * Characters to ignore
	 */
		sport->port.ignore_status_mask = 0;

	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= URISR_TO_SM(URISR_FER | URISR_PER);

	if (termios->c_iflag & IGNBRK) {
#ifdef CONFIG_SERIAL_WMT_BKSIG
		/*
		 * WMT UART doesn't support break signal detection interrupt.
		 *
		 * I try to implement this using URISR_FER.
		 */
		sport->port.ignore_status_mask |= BIT31;/*FIXME*/
#endif

		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= URISR_TO_SM(URISR_RXDOVR);
	}

	del_timer_sync(&sport->timer);

	/*
	 * Update the per-port timeout.
	 */
	 uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Disable FIFO request interrupts and drain transmitter
	 */
	old_urlcr = uart->urlcr;
	old_urier = uart->urier;
	uart->urier = old_urier & ~(URIER_ETXFAE | URIER_ERXFAF);

	/*
	 * Two step polling, first step polling the remaining
	 * entries in TX FIFO. This step make it safe to drain
	 * out all of remaining data in FIFO.
	 */
	while (URFIDX_TXFIDX(uart->urfidx))
		barrier();

	/*
	 * Second step to make sure the last one data has been sent.
	 */
	while (uart->urusr & URUSR_TXDBSY)
		barrier();

	/*
	 * Disable this UART port.
	 */
	uart->urier = 0;

	/*
	 * Set the parity, stop bits and data size
	 */
	uart->urlcr = new_urlcr;

	/*
	 * Set baud rate
	 */
	/*quot -= 1;*/
	/*uart->urdiv = quot & 0xff;*/
	for (i = 0; i < BAUD_TABLE_SIZE; i++) {
		if (baud_table[i].baud == baud)
			break;
	}
	uart->urdiv = baud_table[i].brd;
	/*uart->urdiv = UART_BR_115K2; //fan*/
	/*
	 * Read to clean any pending pulse interrupts.
	 */
	tmp_urisr = uart->urisr;

	/*
	 * Restore FIFO interrupt, TXEN bit, RXEN bit settings.
	 */
	uart->urier = old_urier;
	uart->urlcr |= old_urlcr & (URLCR_TXEN | URLCR_RXEN);

	if (UART_ENABLE_MS(&sport->port, termios->c_cflag))
		wmt_enable_ms(&sport->port);

	spin_unlock_irqrestore(&sport->port.lock, flags);
/**/
/* additional checking*/
/**/
	/*printk("%s: baud=%d quot=%d urlcr=0x%x\n\r",*/
	/*        __FUNCTION__, baud, quot, uart->urlcr);     // harry0*/

	/* These checking is in order to make sure quot to be*/
	/* under our control, because it relative to URBKR value.*/

	/**/
	/* These can be removed in the future if nothing goes wrong.*/
	/*
		for (i = 0; i < BAUD_TABLE_SIZE; i++ ) {
			if ( baud_table[i].brd == (quot & 0xff) )
				break;
		}
		if ( i >= BAUD_TABLE_SIZE)
			printk("%s: unexpected quot=%d\n", __func__, quot); // harry0
	*/
}

static const char *wmt_type(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;

	return (sport->port.type == PORT_WMT) ? "wmt serial" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void wmt_release_port(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;

	release_mem_region(sport->port.mapbase, UART_PORT_SIZE);
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int wmt_request_port(struct uart_port *port)
{
	struct wmt_port *sport = (struct wmt_port *)port;

	return request_mem_region(sport->port.mapbase,
							  UART_PORT_SIZE,
							  "uart") != NULL ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void wmt_config_port(struct uart_port *port, int flags)
{
	struct wmt_port *sport = (struct wmt_port *)port;

	if (flags & UART_CONFIG_TYPE && wmt_request_port(&sport->port) == 0)
		sport->port.type = PORT_WMT;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_WMT and PORT_UNKNOWN
 */
static int wmt_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct wmt_port *sport = (struct wmt_port *)port;
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_WMT)
		ret = -EINVAL;
	if (sport->port.irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (sport->port.uartclk / 16 != ser->baud_base)
		ret = -EINVAL;

	if ((void *)sport->port.mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (sport->port.iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops wmt_pops = {
	.tx_empty 		= wmt_tx_empty,
	.set_mctrl		= wmt_set_mctrl,
	.get_mctrl		= wmt_get_mctrl,
	.stop_tx		= wmt_stop_tx,
	.start_tx		= wmt_start_tx,
	.stop_rx		= wmt_stop_rx,
	.enable_ms		= wmt_enable_ms,
	.break_ctl		= wmt_break_ctl,
	.startup		= wmt_startup,
	.shutdown		= wmt_shutdown,
	.pm				= wmt_uart_pm,
	.set_termios	= wmt_set_termios,
	.type			= wmt_type,
	.release_port	= wmt_release_port,
	.request_port	= wmt_request_port,
	.config_port	= wmt_config_port,
	.verify_port	= wmt_verify_port,
};

static struct wmt_port wmt_ports[NR_PORTS];

/* Setup the WMT serial ports.  Note that we don't include the IrDA
 * port here since we have our own SIR/FIR driver (see drivers/net/irda)
 *
 * Note also that we support "console=ttyVTx" where "x" is either 0 to 2.
 * Which serial port this ends up being depends on the machine you're
 * running this kernel on.
 */
static void wmt_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;

	first = 0;

	for (i = 0; i < NR_PORTS; i++) {
		wmt_ports[i].port.uartclk    = 24000000;
		wmt_ports[i].port.ops        = &wmt_pops;
		wmt_ports[i].port.fifosize   = 16;
		wmt_ports[i].port.line       = i;
		wmt_ports[i].port.iotype     = SERIAL_IO_MEM;
		init_timer(&wmt_ports[i].timer);
		wmt_ports[i].timer.function  = wmt_timeout;
		wmt_ports[i].timer.data      = (unsigned long)&wmt_ports[i];
	}

	/*
	 * Make sure all UARTs are not configured as GPIO function.
	 *
	 * This step may be redundant due to bootloader has already
	 * done this for us.
	 */

	/*
	 * Enable UART0 and UART5
	 * Disable uart GPIO mode.
	 */
	gpio_disable(GPIO_UART0_RTS);
	gpio_disable(GPIO_UART0_TXD);
	gpio_disable(GPIO_UART0_CTS);
	gpio_disable(GPIO_UART0_RXD);
	gpio_disable(GPIO_UART1_RTS);
	gpio_disable(GPIO_UART1_TXD);
	gpio_disable(GPIO_UART1_CTS);
	gpio_disable(GPIO_UART1_RXD);
	gpio_disable(GPIO_UART2_RTS);
	gpio_disable(GPIO_UART2_TXD);
	gpio_disable(GPIO_UART2_CTS);
	gpio_disable(GPIO_UART2_RXD);
	//gpio_disable(GPIO_UART3_RTS);
	//gpio_disable(GPIO_UART3_TXD);
	//gpio_disable(GPIO_UART3_CTS);
	//gpio_disable(GPIO_UART3_RXD);
}

void __init wmt_register_uart_fns(struct wmt_port_fns *fns)
{
	if (fns->get_mctrl)
		wmt_pops.get_mctrl = fns->get_mctrl;
	if (fns->set_mctrl)
		wmt_pops.set_mctrl = fns->set_mctrl;

	wmt_pops.pm = fns->pm;
	wmt_pops.set_wake = fns->set_wake;
}

void __init wmt_register_uart(int idx, int port)
{
	if (idx >= NR_PORTS) {
		printk(KERN_ERR "%s: bad index number %d\n", __func__, idx);
		return;
	}

	switch (port) {
	case 1:
		wmt_ports[idx].port.membase = (void *)(REG32_PTR(UART1_URTDR_ADDR));
		wmt_ports[idx].port.mapbase = UART1_URTDR_ADDR;
		wmt_ports[idx].port.irq     = IRQ_UART0;
		wmt_ports[idx].port.flags   = ASYNC_BOOT_AUTOCONF;
		break;
	case 2:
		wmt_ports[idx].port.membase = (void *)(REG32_PTR(UART2_URTDR_ADDR));
		wmt_ports[idx].port.mapbase = UART2_URTDR_ADDR;
		wmt_ports[idx].port.irq     = IRQ_UART1;
		wmt_ports[idx].port.flags   = ASYNC_BOOT_AUTOCONF;
		break;
	case 3:
		wmt_ports[idx].port.membase = (void *)(REG32_PTR(UART3_URTDR_ADDR));
		wmt_ports[idx].port.mapbase = UART3_URTDR_ADDR;
		wmt_ports[idx].port.irq     = IRQ_UART2;
		wmt_ports[idx].port.flags   = ASYNC_BOOT_AUTOCONF;
		break;
	case 4:
		wmt_ports[idx].port.membase = (void *)(REG32_PTR(UART4_URTDR_ADDR));
		wmt_ports[idx].port.mapbase = UART4_URTDR_ADDR;
		wmt_ports[idx].port.irq     = IRQ_UART3;
		wmt_ports[idx].port.flags   = ASYNC_BOOT_AUTOCONF;
		break;
	case 5:
		wmt_ports[idx].port.membase = (void *)(REG32_PTR(UART5_URTDR_ADDR));
		wmt_ports[idx].port.mapbase = UART5_URTDR_ADDR;
		wmt_ports[idx].port.irq     = IRQ_UART4;
		wmt_ports[idx].port.flags   = ASYNC_BOOT_AUTOCONF;
		break;
	case 6:
		wmt_ports[idx].port.membase = (void *)(REG32_PTR(UART6_URTDR_ADDR));
		wmt_ports[idx].port.mapbase = UART6_URTDR_ADDR;
		wmt_ports[idx].port.irq     = IRQ_UART5;
		wmt_ports[idx].port.flags   = ASYNC_BOOT_AUTOCONF;
		break;
	default:
		printk(KERN_ERR "%s: bad port number %d\n", __func__, port);
	}
}

#ifdef CONFIG_SERIAL_WMT_CONSOLE

/*
 * Interrupts are disabled on entering
 *
 * Note: We do console writing with UART register mode.
 */

static void wmt_console_write(struct console *co, const char *s, u_int count)
{
	struct wmt_port *sport = &wmt_ports[co->index];
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);
	unsigned int i, old_urlcr, old_urier;
	/*{JHT*/
	unsigned int old_urfcr;
	/*}JHT*/
	/*
	 * First, save URLCR and URIER.
	 */
	old_urlcr = uart->urlcr;
	old_urier = uart->urier;
	/*{JHT*/
	old_urfcr = uart->urfcr;
	/*}JHT*/

	/*
	 * Second, switch to register mode with follows method:
	 *
	 * Disable FIFO threshold interrupts, and enable transmitter.
	 */
	uart->urier &= ~(URIER_ETXFAE | URIER_ERXFAF);
	uart->urlcr |= URLCR_TXEN;
	/*{JHT*/
	uart->urfcr &= ~URFCR_FIFOEN;
	/*}JHT*/
	/*
	 * Now, do each character
	 */
	for (i = 0; i < count; i++) {
		/*
		 * Polling until free for transmitting.
		 */
		while (uart->urusr & URUSR_TXDBSY)
			;

		uart->urtdr = (unsigned int)s[i];

		/*
		 * Do CR if there comes a LF.
		 */
		if (s[i] == '\n') {
			/*
			 * Polling until free for transmitting.
			 */
			while (uart->urusr & URUSR_TXDBSY)
				;

			uart->urtdr = (unsigned int)'\r';
		}
	}

	/*
	 * Finally, wait for transmitting done and restore URLCR and URIER.
	 */
	while (uart->urusr & URUSR_TXDBSY)
		;

	uart->urlcr = old_urlcr;
	uart->urier = old_urier;
	/*{JHT*/
	uart->urfcr = old_urfcr;
	/*}JHT*/
}

/*
 * If the port was already initialised (eg, by a boot loader), try to determine
 * the current setup.
 */
static void __init wmt_console_get_options(struct wmt_port *sport, int *baud, int *parity, int *bits)
{
	int i;
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

	if ((uart->urlcr & (URLCR_RXEN | URLCR_TXEN)) == (URLCR_RXEN | URLCR_TXEN)) {
		/*
		 * Port was enabled.
		 */
		unsigned quot;

		*parity = 'n';
		/*
		 * Check parity mode, 0:evev 1:odd
		 */
		if (uart->urlcr & URLCR_PTYEN) {
			if (uart->urlcr & URLCR_PTYMODE)
				*parity = 'o';
			else
				*parity = 'e';
		}

		/*
		 * Check data length, 0:7-bit 1:8-bit
		 */
		if (uart->urlcr & URLCR_DLEN)
			*bits = 8;
		else
			*bits = 7;

		/*
		 * Get baud rate divisor.
		 */
		quot = (uart->urdiv & URBRD_BRDMASK);
		/*
		 * FIXME: I didn't trace the console driver want me
		 * report baud rate whether actual baud rate or ideal
		 * target baud rate, current I report baud as actual
		 * one, if it need value as target baud rate, just
		 * creat an array to fix it, Dec.23 by Harry.
		 */

		/*printk("%s: quot=%d\n", __FUNCTION__, quot);  // harry0*/
		for (i = 0; i < BAUD_TABLE_SIZE; i++) {
			if ((baud_table[i].brd & URBRD_BRDMASK) == quot) {
				*baud = baud_table[i].baud;
				break;
			}
		}

		/*
		 * If this condition is true, something might be wrong.
		 * I reprot the actual baud rate temporary.
		 * Check the printk information then fix it.
		 */
		if (i >= BAUD_TABLE_SIZE)
			*baud = sport->port.uartclk / (13 * (quot + 1));
	}
}

#ifndef CONFIG_WMT_DEFAULT_BAUDRATE
#define CONFIG_WMT_DEFAULT_BAUDRATE  115200
#endif

static int __init
wmt_console_setup(struct console *co, char *options)
{
	struct wmt_port *sport;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int baud = CONFIG_WMT_DEFAULT_BAUDRATE;

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	*/
	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	sport = &wmt_ports[co->index];
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		wmt_console_get_options(sport, &baud, &parity, &bits);

	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}

static struct uart_driver wmt_reg;

static struct console wmt_console = {

#ifdef CONFIG_SERIAL_WMT_TTYVT
	.name	= "ttyVT",
#else
	.name	= "ttyS",
#endif

	.write 	= wmt_console_write,
	.device	= uart_console_device,
	.setup	= wmt_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &wmt_reg,
};

static int __init wmt_rs_console_init(void)
{
	wmt_init_ports();
	register_console(&wmt_console);
	return 0;
}

console_initcall(wmt_rs_console_init);

#define WMT_CONSOLE  (&wmt_console)

#else   /* CONFIG_SERIAL_WMT_CONSOLE */

#define WMT_CONSOLE  NULL

#endif

static struct uart_driver wmt_reg = {
	.owner          = THIS_MODULE,

#ifdef CONFIG_SERIAL_WMT_TTYVT
	.driver_name    = "ttyVT",
	.dev_name       = "ttyVT",
#else
	.driver_name    = "ttyS",
	.dev_name       = "ttyS",
#endif
	.major          = SERIAL_WMT_MAJOR,
	.minor          = MINOR_START,
	.nr             = NR_PORTS,
	.cons           = WMT_CONSOLE,
};


static int wmt_serial_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wmt_port *sport = dev_get_drvdata(&pdev->dev);
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

        printk("wmt_serial_suspend\n");
	if (sport)
		uart_suspend_port(&wmt_reg, &sport->port);

	if(!sport)
		return 0;

	/*disable host interrupt*/
	sport->port.old_urier = uart->urier;
	uart->urier = 0;

	switch (sport->port.irq) {

        case IRQ_UART0:
		REG32_VAL(PMCEL_ADDR) &= ~( BIT1 );
                break;
        case IRQ_UART1:
		REG32_VAL(PMCEL_ADDR) &= ~( BIT2 );
                break;
        case IRQ_UART2:
		REG32_VAL(PMCEL_ADDR) &= ~( BIT3 );
                break;
        case IRQ_UART3:
		REG32_VAL(PMCEL_ADDR) &= ~( BIT4 );
                break;
        case IRQ_UART4:
		REG32_VAL(PMCEL_ADDR) &= ~( BIT22 );
                break;
        case IRQ_UART5:
		REG32_VAL(PMCEL_ADDR) &= ~( BIT23);
                break;
        }

	return 0;
}

static int wmt_serial_resume(struct platform_device *pdev)
{
	struct wmt_port *sport = dev_get_drvdata(&pdev->dev);
	struct wmt_uart *uart = (struct wmt_uart *)PORT_TO_BASE(sport);

        printk("wmt_serial_resume\n");
        if(!sport)
		return 0;

        switch (sport->port.irq) {

        case IRQ_UART0:
              REG32_VAL(PMCEL_ADDR) |= ( BIT1 );
                break;
        case IRQ_UART1:
              REG32_VAL(PMCEL_ADDR) |= ( BIT2 );
                break;
        case IRQ_UART2:
              REG32_VAL(PMCEL_ADDR) |= ( BIT3 );
                break;
        case IRQ_UART3:
              REG32_VAL(PMCEL_ADDR) |= ( BIT4 );
                break;
        case IRQ_UART4:
              REG32_VAL(PMCEL_ADDR) |= ( BIT22 );
                break;
        case IRQ_UART5:
              REG32_VAL(PMCEL_ADDR) |= ( BIT23);
                break;
        }

	/*store back the interrupt enable status*/
	uart->urier = sport->port.old_urier;

	if (sport)
		uart_resume_port(&wmt_reg, &sport->port);

	return 0;
}

static int wmt_serial_probe(struct platform_device *pdev)
{
//	struct platform_device *pdev = to_platform_device(_dev);
	struct resource *res = pdev->resource;
	int i;

	for (i = 0; i < pdev->num_resources; i++, res++)
		if (res->flags & IORESOURCE_MEM)
			break;

	if (i < pdev->num_resources) {
		for (i = 0; i < NR_PORTS; i++) {
			if (wmt_ports[i].port.mapbase != res->start)
					continue;

			wmt_ports[i].port.dev = &pdev->dev;// _dev;
			uart_add_one_port(&wmt_reg, &wmt_ports[i].port);
			//dev_set_drvdata(_dev, &wmt_ports[i]);
			dev_set_drvdata(&pdev->dev, &wmt_ports[i]);
			break;
		}
	}
	return 0;
}

static int wmt_serial_remove(struct platform_device *pdev)
{
	struct wmt_port *sport = dev_get_drvdata(&pdev->dev);

//	dev_set_drvdata(_dev, NULL);
        dev_set_drvdata(&pdev->dev, NULL);

	if (sport)
		uart_remove_one_port(&wmt_reg, &sport->port);

	return 0;
}

/*
static struct device_driver wmt_serial_driver = {
	.name           = "uart",
	.bus            = &platform_bus_type,
	.probe          = wmt_serial_probe,
	.remove         = wmt_serial_remove,
	.suspend        = wmt_serial_suspend,
	.resume         = wmt_serial_resume,
};
*/

static struct platform_driver wmt_serial_driver = {
	.probe = wmt_serial_probe,
	.remove = wmt_serial_remove,
	.suspend        = wmt_serial_suspend,
	.resume         = wmt_serial_resume,
	.driver = { .name = "uart" }
};

static int __init wmt_serial_init(void)
{
	int ret;

	wmt_init_ports();
	ret = uart_register_driver(&wmt_reg);

	if (ret == 0) {
//		ret = driver_register(&wmt_serial_driver);
                ret = platform_driver_register(&wmt_serial_driver);
		if (ret)
			uart_unregister_driver(&wmt_reg);
	}

#ifndef CONFIG_SKIP_DRIVER_MSG
	printk(KERN_INFO "WMT Serial driver initialized: %s\n",
			(ret == 0) ? "ok" : "failed");
#endif
	return ret;
}

static void __exit wmt_serial_exit(void)
{
//	driver_unregister(&wmt_serial_driver);
        platform_driver_unregister(&wmt_serial_driver);
	uart_unregister_driver(&wmt_reg);
}

module_init(wmt_serial_init);
module_exit(wmt_serial_exit);

MODULE_AUTHOR("WMT Technologies, Inc.");
MODULE_DESCRIPTION("WMT generic serial port driver");
MODULE_LICENSE("GPL");
