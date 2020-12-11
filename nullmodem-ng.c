/* ########################################################################

   nullmodem-ng - linux null modem emulator (module)

   ########################################################################

	Based on nullmodem driver -  Copyright (c) : 2012 Peter Remmers
	Based on tty0tty driver -  Copyright (c) : 2010  Luis Claudio Gambôa Lopes
	Based on Tiny TTY driver -  Copyright (C) 2002-2004 Greg Kroah-Hartman (greg@kroah.com)

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

   For e-mail suggestions :  celliwig@nym.hush.com
   ######################################################################## */


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <asm/uaccess.h>
#include "nullmodem-ng.h"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

// ########################################################################
// # Module Parameters
// ########################################################################
static unsigned int burst_transfer = 1;
module_param(burst_transfer, int, 0444);
MODULE_PARM_DESC(burst_transfer, "Allow multiple characters to be transfer.\n");
static unsigned int device_pairs = NULLMODEM_PAIRS;
module_param(device_pairs, int, 0444);
MODULE_PARM_DESC(device_pairs, "Number of linked pairs to create.\n");
static unsigned int tx_buffer_size = DEFAULT_BUF_SIZE;
module_param(tx_buffer_size, int, 0444);
MODULE_PARM_DESC(tx_buffer_size, "Size of the Tx buffer.\n");

// ########################################################################
// # Static variables
// ########################################################################
static struct tty_driver *nullmodem_tty_driver;
static struct nullmodem_device	*nullmodem_devices[MAX_DEVICES];
static unsigned int num_devices, scrambled_char = 0xff;

// ########################################################################
// # 
// ########################################################################
static int nullmodem_status_lines_swap(int pins)
{
	int out = 0;
	if (pins & TIOCM_RTS) out |= TIOCM_CTS;
	if (pins & TIOCM_DTR) out |= TIOCM_DSR;
	if (pins & TIOCM_CTS) out |= TIOCM_RTS;
	if (pins & TIOCM_DSR) out |= TIOCM_DTR;
	return out;
}

static int nullmodem_status_lines_get(struct nullmodem_device *nm_device)
{
	int status_lines = 0;

	status_lines |= nm_device->status_register;
	status_lines |= nm_device->control_register;
	if (status_lines & TIOCM_DSR) status_lines |= TIOCM_CD;

	return status_lines;
}

static void nullmodem_status_register_update(struct nullmodem_device *nm_device, unsigned int pins_new)
{
	unsigned int pins_changed, pins_old;

	// Filter for valid bits
	pins_new = pins_new & STATUS_FLAGS;

	// Lock this device
	mutex_lock(&nm_device->rx_mutex);

	pins_old = nm_device->status_register;					// Get current configuration
	pins_changed = pins_old ^ pins_new;					// Is there a difference?

	nm_device->status_register = pins_new;

	if (pins_changed)
	{
		// Update receiver stats
		if (pins_changed & TIOCM_CTS)
		{
			nm_device->icount.cts++;
		}
		if (pins_changed & TIOCM_DSR)
		{
			nm_device->icount.dcd++;
			nm_device->icount.dsr++;
		}

		// Check if CTS has changed
		if (nm_device->tty && (nm_device->tty->termios.c_cflag & CRTSCTS) && (pins_changed & TIOCM_CTS))
		{
			if (pins_new & TIOCM_CTS)
			{
				// CTS = 1, enable receiver
				nm_device->tty->hw_stopped = 0;
				tty_wakeup(nm_device->tty);
			}
			else
			{
				// CTS = 0, so stop receiver
				nm_device->tty->hw_stopped = 1;
			}
		}
	}

	// Release this device
	mutex_unlock(&nm_device->rx_mutex);
}

static void nullmodem_control_register_update(struct nullmodem_device *nm_device, unsigned int rbits_set, unsigned int rbits_clear)
{
	unsigned int pins_changed, pins_new, pins_old;

	// Filter for valid bits
	rbits_set = rbits_set & CONTROL_FLAGS;
	rbits_clear = rbits_clear & CONTROL_FLAGS;

	// Lock this device
	mutex_lock(&nm_device->tx_mutex);

	pins_old = nm_device->control_register;					// Get current configuration
	pins_new = (pins_old & ~rbits_clear) | rbits_set;			// Create new configuration
	pins_changed = pins_old ^ pins_new;					// Is there a difference?

	nm_device->control_register = pins_new;

	// Release this device
	mutex_unlock(&nm_device->tx_mutex);

	if (pins_changed)
	{
		// Update receiver status register
		nullmodem_status_register_update(nm_device->paired_with, nullmodem_status_lines_swap(pins_new));
	}

//	if (change)
//		wake_up_interruptible(&end->pair->control_lines_wait);
}

static void nullmodem_termios_update(struct tty_struct *tty)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	speed_t nm1_rx, nm2_rx, nm1_tx, nm2_tx;
	tcflag_t cflag_device, cflag_paired;
	unsigned int ticks_per_symbol;
	unsigned long flags;

	printd("#%d: %s\n", tty->index, __FUNCTION__);

	// Get this device baud/parity/bits/etc
	spin_lock_irqsave(&nm_device->tport.lock, flags);
	nm1_rx = nm_device->tty->termios.c_ispeed;
	nm1_tx = nm_device->tty->termios.c_ospeed;
	// Filter flow control
	cflag_device = nm_device->tty->termios.c_cflag & ~CRTSCTS;
	spin_unlock_irqrestore(&nm_device->tport.lock, flags);

	nm_device->baud_rate = tty_get_baud_rate(tty);
	if (nm_device->baud_rate == 0)
	{
		nullmodem_control_register_update(nm_device, 0, TIOCM_DTR|TIOCM_RTS);
		// Just return if baud rate zero
		return;
	}
	else
	{
		nullmodem_control_register_update(nm_device, TIOCM_DTR|TIOCM_RTS, 0);
	}

	// Check that the other port is allocated
	if (nm_device->paired_with->tty)
	{
		// Get paired device baud/parity/bits/etc
		spin_lock_irqsave(&nm_device->paired_with->tport.lock, flags);
		nm2_rx = nm_device->paired_with->tty->termios.c_ispeed;
		nm2_tx = nm_device->paired_with->tty->termios.c_ospeed;
		// Filter flow control
		cflag_paired = nm_device->paired_with->tty->termios.c_cflag & ~CRTSCTS;
		spin_unlock_irqrestore(&nm_device->paired_with->tport.lock, flags);

		// Check whether async parameters match on either end
		if ((nm1_tx == nm2_rx) && (nm2_tx == nm1_rx) && (cflag_device == cflag_paired))
		{
			nm_device->tx_rx_matched = true;
			nm_device->paired_with->tx_rx_matched = true;
		}
		else
		{
			nm_device->tx_rx_matched = false;
			nm_device->paired_with->tx_rx_matched = false;
		}
	}

	nm_device->symbol_length = 2;
	switch (cflag_device & CSIZE)
	{
		case CS5:	nm_device->symbol_length +=5;	break;
		case CS6:	nm_device->symbol_length +=6;	break;
		case CS7:	nm_device->symbol_length +=7;	break;
		default:
		case CS8:	nm_device->symbol_length +=8;	break;
	}
	if (cflag_device & PARENB) nm_device->symbol_length += 1;
	if (cflag_device & CSTOPB) nm_device->symbol_length += 1;

	// Calculate the ratio of system timer ticks to symbols per second
	ticks_per_symbol = DIV_ROUND_CLOSEST(TIMER_FREQ, DIV_ROUND_CLOSEST(nm_device->baud_rate, nm_device->symbol_length));
	if (ticks_per_symbol >= 1)
	{
		// If the baud rate is slow enough, transmit just 1 symbol
		// Set the delay between timer execution to emulate baud rate
		nm_device->ticks_per_tx_symbol = ticks_per_symbol;
		nm_device->tx_symbols_per_tick = 1;
	}
	else
	{
		// If the baud rate is faster than the system timer
		// Set minimum time between timer execution
		// Transmit multiple characters to emulate baud rate
		nm_device->ticks_per_tx_symbol = 1;
		nm_device->tx_symbols_per_tick = DIV_ROUND_CLOSEST(DIV_ROUND_CLOSEST(nm_device->baud_rate, nm_device->symbol_length), TIMER_FREQ);
	}
}

// ########################################################################
// # Timer routines
// ########################################################################
static void nullmodem_timer_tx_handle(struct timer_list *tl)
{
	struct nullmodem_device *nm_device;
	unsigned char symbol_mask, tx_drain[DRAIN_BUF_SIZE], tx_transfer_count;
	unsigned int i;

	printd("%s\n", __FUNCTION__);

	nm_device = from_timer(nm_device, tl, tx_timer);

	// Check both ends are active
	if ((nm_device->tty == NULL) || (nm_device->paired_with->tty == NULL)) return;
	if ((nm_device->tty->hw_stopped) || (nm_device->paired_with->tty->hw_stopped)) return;

	// Save the delta between requested and actual timer expiry
	nm_device->delta_jiffies = (unsigned char) (jiffies - nm_device->tx_timer.expires);

	tx_transfer_count = 1;
	// Check whether multi-byte transfers are enabled
	if (burst_transfer)
	{
		// If burst transfers are supported, calculate the maximum number of symbols to transfer
		tx_transfer_count = nm_device->tx_symbols_per_tick * (nm_device->delta_jiffies + 1);
	}

	mutex_lock(&nm_device->tx_mutex);
	// Extract from FIFO a set number of bytes
	// Don't remove, don't know how many can actually be transfered
	tx_transfer_count = kfifo_out_peek(&nm_device->tx_fifo, &tx_drain, tx_transfer_count);

	// Get symbol length
	switch (nm_device->tty->termios.c_cflag & CSIZE)
	{
		case CS5: symbol_mask = 0x1F; break;
		case CS6: symbol_mask = 0x3F; break;
		case CS7: symbol_mask = 0x7F; break;
		default: symbol_mask = 0xFF; break;
	}
	for (i = 0; i < tx_transfer_count; i++)
	{
		// Enforce symbol length
		tx_drain[i] &= symbol_mask;
		// Scramble characters if the ends are mismatched
		if (!nm_device->tx_rx_matched) {
			tx_drain[i] = scrambled_char = (scrambled_char ^ tx_drain[i]) >> 1;
		}
	}

	// Write to receiver buffer
	mutex_lock(&nm_device->paired_with->rx_mutex);
	// Write byte(s)
	tx_transfer_count = tty_insert_flip_string(&nm_device->paired_with->tport, tx_drain, tx_transfer_count);
	if (tx_transfer_count > 0)
	{
		tty_flip_buffer_push(&nm_device->paired_with->tport);		// Push buffer to user
		nm_device->paired_with->icount.rx += tx_transfer_count;		// Update Rx stats
	}
	mutex_unlock(&nm_device->paired_with->rx_mutex);

	// Remove the actual data
	kfifo_out(&nm_device->tx_fifo, &tx_drain, tx_transfer_count);
	nm_device->icount.tx += tx_transfer_count;				// Update Tx stats

	// Schedule Tx timer if Tx FIFO not empty
	if (!kfifo_is_empty(&nm_device->tx_fifo)) {
		if (!timer_pending(&nm_device->tx_timer)) {
			nm_device->tx_timer.expires = jiffies + nm_device->ticks_per_tx_symbol;
			add_timer(&nm_device->tx_timer);
		}
	}
	mutex_unlock(&nm_device->tx_mutex);

//	if (kfifo_len(&nm_device->tx_fifo) < WAKEUP_CHARS)
		tty_wakeup(nm_device->tty);

	printd("%s - Transfered %u bytes\n", __FUNCTION__, tx_transfer_count);
}

static void nullmodem_timer_tx_set(struct timer_list *tl)
{
	struct nullmodem_device *nm_device;

	printd("%s\n", __FUNCTION__);

	nm_device = from_timer(nm_device, tl, tx_timer);

	// Schedule Tx timer, if it is not already active
	if (!timer_pending(&nm_device->tx_timer)) {
		nm_device->tx_timer.expires = jiffies + nm_device->ticks_per_tx_symbol;
		add_timer(&nm_device->tx_timer);
	}
}

// ########################################################################
// # Module TTY routines
// ########################################################################
static int nullmodem_open(struct tty_struct *tty, struct file *file)
{
	struct nullmodem_device *nm_device = nullmodem_devices[tty->index];
	struct tty_port *nm_port = &nm_device->tport;
	int status;

	printd("#%d: %s:- TTY count: %d\n", tty->index, __FUNCTION__, tty->count);

	if (tty->count > 1) return 0;

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	status = tty_port_open(nm_port, tty, file);

	if(!status) {
		/* save our structure within the tty structure */
		tty->driver_data = nm_device;
	}

	// Update termios
	nullmodem_termios_update(tty);

	return status;
}

static void nullmodem_close(struct tty_struct *tty, struct file *file)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	struct tty_port *nm_port;

	printd("#%d: %s:- TTY count: %d\n", tty->index, __FUNCTION__, tty->count);

	if (tty->count > 1) return;

	nullmodem_control_register_update(nm_device, 0, TIOCM_RTS|TIOCM_DTR);
	tty->hw_stopped = 1;

	if (nm_device) {
		nm_port = &nm_device->tport;
		tty_port_close(nm_port, tty, file);
	}

	wake_up_interruptible(&tty->read_wait);
	wake_up_interruptible(&tty->write_wait);
}

static int nullmodem_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	struct tty_port *nm_port;
	int retval;
	unsigned long flags;

	printd("#%d: %s:- TTY count: %d\n", tty->index, __FUNCTION__, tty->count);

	if (!nm_device) return -ENODEV;

	mutex_lock(&nm_device->tx_mutex);

	nm_port = &nm_device->tport;
	// Check that port configured
	spin_lock_irqsave(&nm_port->lock, flags);
	if (!nm_port->count) {
		spin_unlock_irqrestore(&nm_port->lock, flags);
		retval = -EINVAL;
		goto nullmodem_write_exit;
	}
	spin_unlock_irqrestore(&nm_port->lock, flags);

	retval = kfifo_in(&nm_device->tx_fifo, buffer, count);
	nullmodem_timer_tx_set(&nm_device->tx_timer);
	printd("#%d: %s:- %d bytes --> %d written\n", tty->index, __FUNCTION__, count, retval);
nullmodem_write_exit:
	mutex_unlock(&nm_device->tx_mutex);
	return retval;
}

static int nullmodem_write_room(struct tty_struct *tty)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	struct tty_port *nm_port;
	int room = -EINVAL;
	unsigned long flags;

	if (!nm_device) return -ENODEV;

	mutex_lock(&nm_device->tx_mutex);

	nm_port = &nm_device->tport;
	spin_lock_irqsave(&nm_port->lock, flags);
	if (!nm_port->count) {
		spin_unlock_irqrestore(&nm_port->lock, flags);
		goto nullmodem_write_room_exit;
	}
	spin_unlock_irqrestore(&nm_port->lock, flags);

	room = kfifo_avail(&nm_device->tx_fifo);
	printd("#%d: %s:- %d\n", tty->index, __FUNCTION__, room);
nullmodem_write_room_exit:
	mutex_unlock(&nm_device->tx_mutex);
	return room;
}

//static int nullmodem_ioctl_tiocgserial(struct tty_struct *tty, unsigned long arg)
//{
//	struct nullmodem_device *nm_device = tty->driver_data;
//	unsigned long flags;
//	struct serial_struct tmp;
//	struct serial_struct *serial;
//
//	printd("#%d: %s\n", tty->index, __FUNCTION__);
//
//	if (!arg) return -EFAULT;
//
//	serial = &nm_device->serial;
//	memset(&tmp, 0, sizeof(tmp));
//
//	spin_lock_irqsave(&nm_device->slock, flags);
//	tmp.type			= serial->type;
//	tmp.line			= serial->line;
//	tmp.port			= serial->port;
//	tmp.irq				= serial->irq;
//	tmp.flags			= ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
//	tmp.xmit_fifo_size		= serial->xmit_fifo_size;
//	tmp.baud_base			= serial->baud_base;
//	tmp.close_delay			= 5*HZ;
//	tmp.closing_wait		= 30*HZ;
//	tmp.custom_divisor		= serial->custom_divisor;
//	tmp.hub6			= serial->hub6;
//	tmp.io_type			= serial->io_type;
//	spin_unlock_irqrestore(&nm_device->slock, flags);
//
//	if (copy_to_user((void __user *)arg, &tmp, sizeof(struct serial_struct)))
//		return -EFAULT;
//	return 0;
//}

//static int nullmodem_ioctl_tiocmiwait(struct tty_struct *tty, unsigned long arg)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//	DECLARE_WAITQUEUE(wait, current);
//	int pins;
//	int prev;
//	int changed;
//	int ret;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	if ((tty->index&1) == 0)
//	{
//		if (arg & TIOCM_CD) arg |= TIOCM_DSR;
//	}
//	else
//	{
//		int t = 0;
//		if (arg & TIOCM_CTS) t |= TIOCM_RTS;
//		if (arg & TIOCM_DSR) t |= TIOCM_DTR;
//		if (arg & TIOCM_CD)  t |= TIOCM_DTR;
//		arg = t;
//	}
//
//	spin_lock_irqsave(&end->pair->spin, flags);
//	prev = end->pair->control_lines;
//	add_wait_queue(&end->pair->control_lines_wait, &wait);
//	set_current_state(TASK_INTERRUPTIBLE);
//	spin_unlock_irqrestore(&end->pair->spin, flags);
//
//	while (1)
//	{
//		schedule();
//
//		/* see if a signal woke us up */
//		if (signal_pending(current))
//		{
//			ret = -ERESTARTSYS;
//			break;
//		}
//
//		spin_lock_irqsave(&end->pair->spin, flags);
//		pins = end->pair->control_lines;
//		set_current_state(TASK_INTERRUPTIBLE);
//		spin_unlock_irqrestore(&end->pair->spin, flags);
//
//		changed = pins ^ prev;
//		if (changed & arg)
//		{
//			ret = 0;
//			break;
//		}
//
//		prev = pins;
//	}
//	remove_wait_queue(&end->pair->control_lines_wait, &wait);
//	set_current_state(TASK_RUNNING);
//	return ret;
//}

static int nullmodem_ioctl_tiocgicount(struct tty_struct *tty, unsigned long arg)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	struct serial_icounter_struct icount;
	unsigned long flags;

	printd("#%d: %s\n", tty->index, __FUNCTION__);

	spin_lock_irqsave(&nm_device->tport.lock, flags);
	icount.cts			= nm_device->icount.cts;
	icount.dsr			= nm_device->icount.dsr;
	icount.rng			= nm_device->icount.rng;
	icount.dcd			= nm_device->icount.dcd;
	icount.rx			= nm_device->icount.rx;
	icount.tx			= nm_device->icount.tx;
	icount.frame			= nm_device->icount.frame;
	icount.overrun			= nm_device->icount.overrun;
	icount.parity			= nm_device->icount.parity;
	icount.brk			= nm_device->icount.brk;
	icount.buf_overrun		= nm_device->icount.buf_overrun;
	spin_unlock_irqrestore(&nm_device->tport.lock, flags);

	if (copy_to_user((void __user *)arg, &icount, sizeof(icount))) return -EFAULT;
	return 0;
}

static int nullmodem_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
	printd("#%d: %s:- cmd:0x%x\n", tty->index, __FUNCTION__, cmd);

	if (cmd == TCGETS || cmd == TCSETS)
		return -ENOIOCTLCMD;

	switch (cmd)
	{
//	case TIOCGSERIAL:
//		return nullmodem_ioctl_tiocgserial(tty, arg);
//		break;
//	case TIOCMIWAIT:
//		return nullmodem_ioctl_tiocmiwait(tty, arg);
//		break;
	case TIOCGICOUNT:
		return nullmodem_ioctl_tiocgicount(tty, arg);
		break;
	}

	return -ENOIOCTLCMD;
}

static void nullmodem_termios_set(struct tty_struct *tty, struct ktermios *old_termios)
{
	//struct nullmodem_device *nm_device = tty->driver_data;
	unsigned int cflag;

	printd("#%d: %s\n", tty->index, __FUNCTION__);

	cflag = tty->termios.c_cflag;

	// Check if something has changed
	if (old_termios)
	{
		if ((cflag == old_termios->c_cflag) &&
				(RELEVANT_IFLAG(tty->termios.c_iflag) == RELEVANT_IFLAG(old_termios->c_iflag)))
		{
			printd("#%d: %s - nothing to change...\n", tty->index, __FUNCTION__);
			return;
		}
	}

	// Update termios
	nullmodem_termios_update(tty);

#ifdef NM_DEBUG
	speed_t speed = tty_get_baud_rate(tty);
	printd("#%d: %s - baud = %u\n", tty->index, __FUNCTION__, speed);
	printd("#%d: %s - ispeed = %u\n", tty->index, __FUNCTION__, tty->termios.c_ispeed);
	printd("#%d: %s - ospeed = %u\n", tty->index, __FUNCTION__, tty->termios.c_ospeed);

	// Byte size
	switch (cflag & CSIZE)
	{
		case CS5:
			printd("#%d: %s - data bits = 5\n", tty->index, __FUNCTION__);
			break;
		case CS6:
			printd("#%d: %s - data bits = 6\n", tty->index, __FUNCTION__);
			break;
		case CS7:
			printd("#%d: %s - data bits = 7\n", tty->index, __FUNCTION__);
			break;
		default:
		case CS8:
			printd("#%d: %s - data bits = 8\n", tty->index, __FUNCTION__);
			break;
	}

	// Parity
	if (cflag & PARENB)
		if (cflag & PARODD)
			printd("#%d: %s - parity = odd\n", tty->index, __FUNCTION__);
		else
			printd("#%d: %s - parity = even\n", tty->index, __FUNCTION__);
	else
		printd("#%d: %s - parity = none\n", tty->index, __FUNCTION__);

	// Stop bits
	if (cflag & CSTOPB)
		printd("#%d: %s - stop bits = 2\n", tty->index, __FUNCTION__);
	else
		printd("#%d: %s - stop bits = 1\n", tty->index, __FUNCTION__);

	// Hardware flow control
	if (cflag & CRTSCTS)
		printd("#%d: %s - RTS/CTS is enabled\n", tty->index, __FUNCTION__);
	else
		printd("#%d: %s - RTS/CTS is disabled\n", tty->index, __FUNCTION__);

	// Software flow control
	/* If we are implementing INBOUND XON/XOFF */
	if (I_IXOFF(tty))
		printd("#%d: %s - INBOUND XON/XOFF is enabled, XON = %2x, XOFF = %2x\n", tty->index, __FUNCTION__, START_CHAR(tty), STOP_CHAR(tty));
	else
		printd("#%d: %s - INBOUND XON/XOFF is disabled\n", tty->index, __FUNCTION__);
	/* If we are implementing OUTBOUND XON/XOFF */
	if (I_IXON(tty))
		printd("#%d: %s - OUTBOUND XON/XOFF is enabled, XON = %2x, XOFF = %2x\n", tty->index, __FUNCTION__, START_CHAR(tty), STOP_CHAR(tty));
	else
		printd("#%d: %s - OUTBOUND XON/XOFF is disabled\n", tty->index, __FUNCTION__);
#endif
}

static void nullmodem_throttle(struct tty_struct * tty)
{
	struct nullmodem_device *nm_device = tty->driver_data;

	printd("#%d: %s\n", tty->index, __FUNCTION__);

	//if (I_IXOFF(tty)) nullmodem_send_xchar(tty, STOP_CHAR(tty));

	if (tty->termios.c_cflag & CRTSCTS)
	{
		nullmodem_control_register_update(nm_device, 0, TIOCM_RTS);
	}
}

static void nullmodem_unthrottle(struct tty_struct * tty)
{
	struct nullmodem_device *nm_device = tty->driver_data;

	printd("#%d: %s\n", tty->index, __FUNCTION__);

	if (tty->termios.c_cflag & CRTSCTS)
	{
		nullmodem_control_register_update(nm_device, TIOCM_RTS, 0);
	}

	//if (I_IXOFF(tty)) nullmodem_send_xchar(tty, START_CHAR(tty));
}

//static void nullmodem_send_xchar(struct tty_struct *tty, char ch)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	end->xchar = ch;
//}

static int nullmodem_tiocmget(struct tty_struct *tty)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	int retval = -EINVAL;

	retval = nullmodem_status_lines_get(nm_device);

	printd("#%d: %s:- --> 0x%x\n", tty->index, __FUNCTION__, retval);
	return retval;
}

static int nullmodem_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
	struct nullmodem_device *nm_device = tty->driver_data;

	printd("#%d: %s:- set:0x%x clear:0x%x\n", tty->index, __FUNCTION__, set, clear);

	nullmodem_control_register_update(nm_device, set, clear);
	return 0;
}

// ########################################################################
// # Supported TTY operations
// ########################################################################
static struct tty_operations nm_serial_ops =
{
	.open		= nullmodem_open,
	.close		= nullmodem_close,
	.write		= nullmodem_write,
	//.put_char 	= ,
	.write_room 	= nullmodem_write_room,
	.ioctl		= nullmodem_ioctl,
	.set_termios	= nullmodem_termios_set,
	.throttle	= nullmodem_throttle,
	.unthrottle	= nullmodem_unthrottle,
	//.send_xchar	= nullmodem_send_xchar,
	.tiocmget	= nullmodem_tiocmget,
	.tiocmset	= nullmodem_tiocmset,
};

// ########################################################################
// # Module TTY port routines
// ########################################################################
static int nullmodem_port_activate(struct tty_port *tport, struct tty_struct *tty)
{
	struct nullmodem_device *nm_device;
	int err = -ENOMEM;

	printd("#%d: %s:- TTY count: %d\n", tty->index, __FUNCTION__, tport->count);

	nm_device = container_of(tport, struct nullmodem_device, tport);
	nm_device->tty = tty;

	// Create Tx FIFO
	if (kfifo_alloc(&nm_device->tx_fifo, tx_buffer_size, GFP_KERNEL)) goto nullmodem_port_activate_exit;

	// Setup timer
	timer_setup(&nm_device->tx_timer, nullmodem_timer_tx_handle, 0);

	err = 0;
nullmodem_port_activate_exit:
	return err;
}

static void nullmodem_port_shutdown(struct tty_port *tport){
	struct nullmodem_device *nm_device;

	printd("%s\n", __FUNCTION__);

	nm_device = container_of(tport, struct nullmodem_device, tport);

	// Destroy Tx FIFO
	kfifo_free(&nm_device->tx_fifo);

	// Shutdown timer
	del_timer(&nm_device->tx_timer);
}

// ########################################################################
// # TTY port operations
// ########################################################################
static const struct tty_port_operations nm_port_ops = {
	.activate	= nullmodem_port_activate,
	.shutdown	= nullmodem_port_shutdown,
};

// ########################################################################
// # SysFS routines
// ########################################################################

static ssize_t nm_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nullmodem_device *nm_device = dev_get_drvdata(dev);
	int char_count = 0;

	if ((nm_device == NULL) || (nm_device->tty == NULL))
	{
		char_count += sprintf(buf, "Device not initialised!\n");
	}
	else
	{
		char_count += sprintf(buf + char_count, "HW Stopped: %s\n\n", nm_device->tty->hw_stopped ? "true" : "false");

		char_count += sprintf(buf + char_count, "Baud Rate: %u\n", nm_device->baud_rate);
		char_count += sprintf(buf + char_count, "	Input Speed: %u\n", nm_device->tty->termios.c_ispeed);
		char_count += sprintf(buf + char_count, "	Output Speed: %u\n", nm_device->tty->termios.c_ospeed);

		char_count += sprintf(buf + char_count, "Configuration: %u\n", nm_device->tty->termios.c_cflag);

		char_count += sprintf(buf + char_count, "Control Register: 0x%04x\n", nm_device->control_register);
		char_count += sprintf(buf + char_count, "	Request To Send: %s\n", nm_device->control_register & TIOCM_RTS ? "true" : "false");
		char_count += sprintf(buf + char_count, "	Data Terminal Ready: %s\n", nm_device->control_register & TIOCM_DTR ? "true" : "false");

		char_count += sprintf(buf + char_count, "Status Register: 0x%04x\n", nm_device->status_register);
		char_count += sprintf(buf + char_count, "	Carrier Detect: %s\n", nm_device->status_register & TIOCM_CAR ? "true" : "false");
		char_count += sprintf(buf + char_count, "	Clear To Send: %s\n", nm_device->status_register & TIOCM_CTS ? "true" : "false");
		char_count += sprintf(buf + char_count, "	Data Sender Ready: %s\n", nm_device->status_register & TIOCM_DSR ? "true" : "false");
		char_count += sprintf(buf + char_count, "	Ring Indicator: %s\n", nm_device->status_register & TIOCM_RI ? "true" : "false");

		char_count += sprintf(buf + char_count, "Tx FIFO\n");
		char_count += sprintf(buf + char_count, "	FIFO Used: %u\n", kfifo_len(&nm_device->tx_fifo));
		char_count += sprintf(buf + char_count, "	FIFO Available: %u\n", kfifo_avail(&nm_device->tx_fifo));

		char_count += sprintf(buf + char_count, "Tx Symbol Parameters\n");
		char_count += sprintf(buf + char_count, "	Symbol Length: %u\n", nm_device->symbol_length);
		char_count += sprintf(buf + char_count, "	Ticks per Symbol: %u\n", nm_device->ticks_per_tx_symbol);
		char_count += sprintf(buf + char_count, "	Symbols per Tick: %u\n", nm_device->tx_symbols_per_tick);
		char_count += sprintf(buf + char_count, "	Delta Jiffies: %u\n", nm_device->delta_jiffies);

		char_count += sprintf(buf + char_count, "ICOUNT Stats\n");
		char_count += sprintf(buf + char_count, "	Clear To Send: %u\n", nm_device->icount.cts);
		char_count += sprintf(buf + char_count, "	Data Carrier Detect: %u\n", nm_device->icount.dcd);
		char_count += sprintf(buf + char_count, "	Data Set Ready: %u\n", nm_device->icount.dsr);
		char_count += sprintf(buf + char_count, "	Rx Received: %u\n", nm_device->icount.rx);
		char_count += sprintf(buf + char_count, "	Tx Sent: %u\n", nm_device->icount.tx);
	}

	return char_count;
}
static DEVICE_ATTR_RO(nm_stats);

// ########################################################################
// # Module routines
// ########################################################################
static int __init nullmodem_init(void)
{
	struct nullmodem_device *nm_device, *nm_other = NULL;
	int retval = 0;
	int i;

	printd("%s\n", __FUNCTION__);

	// Check device number
	if ((device_pairs * 2) > MAX_DEVICES)
	{
		printe("Can not create more than %u devices (%u pairs).\n", MAX_DEVICES, (MAX_DEVICES / 2));
		return -EINVAL;
	}
	num_devices = 2 * device_pairs;
	if (num_devices == 0) num_devices = 1;					// Test mode loopback

	// Clear device array pointers
	for (i = 0; i < MAX_DEVICES; i++) {
		nullmodem_devices[i] = NULL;
	}

	// Allocate the tty driver
	nullmodem_tty_driver = alloc_tty_driver(num_devices);
	if (!nullmodem_tty_driver)
	{
		printe("Failed to initialise TTY driver.\n");
		return -ENOMEM;
	}

	/* initialize the tty driver */
	nullmodem_tty_driver->owner = THIS_MODULE;
	nullmodem_tty_driver->driver_name = "nullmodem-ng";
	nullmodem_tty_driver->name = "nm-ng";
	/* no more devfs subsystem */
	nullmodem_tty_driver->major = NULLMODEM_MAJOR;
	nullmodem_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	nullmodem_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	nullmodem_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	/* no more devfs subsystem */
	nullmodem_tty_driver->init_termios = tty_std_termios;
	nullmodem_tty_driver->init_termios.c_iflag = 0;
	nullmodem_tty_driver->init_termios.c_oflag = 0;
	nullmodem_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	nullmodem_tty_driver->init_termios.c_lflag = 0;
	nullmodem_tty_driver->init_termios.c_ispeed = 38400;
	nullmodem_tty_driver->init_termios.c_ospeed = 38400;
	tty_set_operations(nullmodem_tty_driver, &nm_serial_ops);

	/* register the tty driver */
	retval = tty_register_driver(nullmodem_tty_driver);
	if (retval)
	{
		printe("Failed to register TTY driver: %d\n", retval);
		goto nullmodem_init_free_driver;
	}

	// Allocate devices
	nm_other = NULL;
	for (i = 0; i < num_devices; i++) {
		// Allocate device
		nm_device = kzalloc(sizeof(struct nullmodem_device), GFP_KERNEL);
		if (!nm_device) {
			printe("Can not allocate memory for device.\n");
			goto nullmodem_init_free_devices;
		}

		// Pair device
		if (nm_other != NULL) {
			nm_device->paired_with = nm_other;			// This device is paired with the other one
			nm_other->paired_with = nm_device;
			nm_other = NULL;
		} else if (num_devices == 1) {
			nm_device->paired_with = nm_device;			// In test mode, this is the other device
		} else {
			nm_other = nm_device;					// Save pointer to create a pair of linked devices
		}

		mutex_init(&nm_device->rx_mutex);				// Initialise Rx mutex
		mutex_init(&nm_device->tx_mutex);				// Initialise Tx mutex

		tty_port_init(&nm_device->tport);				// Initialise TTY port
		nm_device->tport.ops = &nm_port_ops;				// Set TTY port operations

		nullmodem_devices[i] = nm_device;				// Save pointer to device
	}

	// Register devices
	for (i = 0; i < num_devices; i++) {
		nm_device = nullmodem_devices[i];

		nm_device->dev = tty_port_register_device(&nm_device->tport, nullmodem_tty_driver, i, NULL);
		if (IS_ERR(nm_device->dev)) {
			retval = PTR_ERR(nm_device->dev);
			printe("Could not register tty [%i]\n", retval);
			goto nullmodem_init_unreg_devices;
		}
		nm_device->dev->driver_data = nm_device;

		// Register SysFS device stats
		if (device_create_file(nm_device->dev, &dev_attr_nm_stats)) printe("Could not create sysfs file for card_type\n");

		nm_device->registered = true;
		printd("Initialised nullmodem device %u.\n", i);
	}

	printi(DRIVER_DESC " " DRIVER_VERSION "\n");
	return 0;

nullmodem_init_unreg_devices:
	// Unregister devices
	for (i = 0; i < num_devices; ++i) {
		nm_device = nullmodem_devices[i];
		if ((nm_device != NULL) && (nm_device->registered)) {
			if (nm_device->tport.count) nullmodem_port_shutdown(&nm_device->tport);
			tty_unregister_device(nullmodem_tty_driver, i);
		}
	}
nullmodem_init_free_devices:
	// Free devices
	for (i = 0; i < num_devices; ++i) {
		nm_device = nullmodem_devices[i];
		if (nm_device != NULL) {
			tty_port_destroy(&nm_device->tport);
			kfree(nm_device);
		}
	}
	tty_unregister_driver(nullmodem_tty_driver);
nullmodem_init_free_driver:
	put_tty_driver(nullmodem_tty_driver);
	return retval;
}

static void __exit nullmodem_exit(void)
{
	struct nullmodem_device *nm_device;
	int i;

	printd("%s\n", __FUNCTION__);

	// Unregister devices
	for (i = 0; i < num_devices; ++i) {
		nm_device = nullmodem_devices[i];
		// Unregister SysFS device stats
		device_remove_file(nm_device->dev, &dev_attr_nm_stats);
		if ((nm_device != NULL) && (nm_device->registered)) {
			if (nm_device->tport.count) nullmodem_port_shutdown(&nm_device->tport);
			tty_unregister_device(nullmodem_tty_driver, i);
		}
	}
	// Free devices
	for (i = 0; i < num_devices; ++i) {
		nm_device = nullmodem_devices[i];
		if (nm_device != NULL) {
			del_timer_sync(&nm_device->tx_timer);
			tty_port_destroy(&nm_device->tport);
			kfree(nm_device);
			printd("Destroyed nullmodem device %u.\n", i);
		}
	}
	tty_unregister_driver(nullmodem_tty_driver);
	put_tty_driver(nullmodem_tty_driver);

	printd("Unloaded\n");
}

module_init(nullmodem_init);
module_exit(nullmodem_exit);
