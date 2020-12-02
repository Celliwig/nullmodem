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
static unsigned int device_pairs = NULLMODEM_PAIRS;
module_param(device_pairs, int, 0444);
MODULE_PARM_DESC(device_pairs, "Number of linked pairs to create.\n");
static unsigned int rx_buffer_size = DEFAULT_BUF_SIZE;
module_param(rx_buffer_size, int, 0444);
MODULE_PARM_DESC(rx_buffer_size, "Size of the Rx buffer.\n");
static unsigned int tx_buffer_size = DEFAULT_BUF_SIZE;
module_param(tx_buffer_size, int, 0444);
MODULE_PARM_DESC(tx_buffer_size, "Size of the Tx buffer.\n");

// ########################################################################
// # Static variables
// ########################################################################
static struct tty_driver *nullmodem_tty_driver;
static struct nullmodem_device	*nullmodem_devices[MAX_DEVICES];
static unsigned int num_devices;

//static unsigned char drain[TX_BUF_SIZE];
//static unsigned long last_timer_jiffies;
//static unsigned long delta_jiffies;

// ########################################################################
// # 
// ########################################################################
//static int switch_pin_view(int pins)
//{
//	int out = 0;
//	if (pins & TIOCM_RTS) out |= TIOCM_CTS;
//	if (pins & TIOCM_DTR) out |= TIOCM_DSR;
//	if (pins & TIOCM_CTS) out |= TIOCM_RTS;
//	if (pins & TIOCM_DSR) out |= TIOCM_DTR;
//	return out;
//}

//static int get_pins(struct nullmodem_end *end)
//{
//	int pins = end->pair->control_lines;
//	if (end == &end->pair->b)
//		pins = switch_pin_view(pins);
//	if (pins&TIOCM_DSR)
//		pins |= TIOCM_CD;
//	return pins;
//}

//static void change_pins(struct nullmodem_end *end, unsigned int set, unsigned int clear)
//{
//	int is_end_b = (end == &end->pair->b);
//	int old_pins = end->pair->control_lines;
//	if (is_end_b)
//		old_pins = switch_pin_view(old_pins);
//
//	int new_pins = (old_pins & ~clear) | set;
//	int change = old_pins ^ new_pins;
//
//	if (is_end_b)
//		new_pins = switch_pin_view(new_pins);
//
//	end->pair->control_lines = new_pins;
//
//	if (change & TIOCM_RTS)
//	{
//		end->other->icount.cts++;
//	}
//	if (change & TIOCM_DTR)
//	{
//		end->other->icount.dsr++;
//		end->other->icount.dcd++;
//	}
//
//	if (end->other->tty
//	&& (end->other->tty->termios->c_cflag & CRTSCTS)
//	&& (change&TIOCM_RTS))
//	{
//		if (!(new_pins&TIOCM_RTS))
//			end->other->tty->hw_stopped = 1;
//		else
//		{
//			end->other->tty->hw_stopped = 0;
//			tty_wakeup(end->other->tty);
//		}
//	}
//
//	if (change)
//		wake_up_interruptible(&end->pair->control_lines_wait);
//}

//static void nullmodem_timer_proc(unsigned long data)
//{
//	int i;
//	unsigned long flags;
//	//dprint("%s jiffies: %lu\n", __FUNCTION__, jiffies);
//
//	unsigned long current_jiffies = jiffies;
//	delta_jiffies = current_jiffies - last_timer_jiffies;
//	last_timer_jiffies = current_jiffies;
//
//	for (i=0; i<NULLMODEM_PAIRS; ++i)
//	{
//		struct nullmodem_pair *pair = &pair_table[i];
//
//		spin_lock_irqsave(&pair->spin, flags);
//		handle_end(&pair->a);
//		handle_end(&pair->b);
//		spin_unlock_irqrestore(&pair->spin, flags);
//	}
//
//	nullmodem_timer.expires += TIMER_INTERVAL;
//	add_timer(&nullmodem_timer);
//}

//static inline void handle_end(struct nullmodem_end *end)
//{
//	if (!end->tty)
//		return;
//	if (end->tty->hw_stopped)
//	{
//		//dprintf("%s - #%d: hw_stopped\n", __FUNCTION__, end->tty->index);
//		return;
//	}
//	unsigned nominal_bits = end->tty->termios->c_ospeed * FACTOR * delta_jiffies / HZ;
//	unsigned add_bits = end->nominal_bit_count - end->actual_bit_count;
//	unsigned chars = (nominal_bits+add_bits) / end->char_length;
//	unsigned actual_bits = chars * end->char_length;
//
//	end->nominal_bit_count += nominal_bits;
//	end->actual_bit_count  += actual_bits;
//
////	dprintf("%s - #%d: nb %u add %u ab %u ch %u nbc %u abc %u\n", __FUNCTION__,
////			end->tty->index, nominal_bits, add_bits, actual_bits, chars,
////			end->nominal_bit_count, end->actual_bit_count);
//
//	if (chars == 0)
//		return;
//
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
//	int cnt = kfifo_out(&end->fifo, drain, chars);
//#else
//	int cnt = __kfifo_get(end->fifo, drain, chars);
//#endif
//	if (cnt < chars)
//	{
//		end->nominal_bit_count = 0;
//		end->actual_bit_count = 0;
//	}
//	if (cnt <= 0)
//	{
//		//dprintf("%s - #%d: fifo empty\n", __FUNCTION__, end->tty->index);
//		return;
//	}
//
////	dprintf("%s - #%d: drained %d bytes\n", __FUNCTION__, end->tty->index, cnt);
//
//	if (end->other->tty)
//	{
//		if (end->tty->termios->c_ospeed == end->other->tty->termios->c_ispeed
//		&& (end->tty->termios->c_cflag & (CSIZE|PARENB|CSTOPB))
//		 ==(end->other->tty->termios->c_cflag & (CSIZE|PARENB|CSTOPB)))
//		{
//			tcflag_t csize = (end->tty->termios->c_cflag&CSIZE);
//			if (csize != CS8)
//			{
//				int i;
//				unsigned char mask = 0xFF;
//				switch (csize)
//				{
//				case CS7: mask = 0x7F; break;
//				case CS6: mask = 0x3F; break;
//				case CS5: mask = 0x1F; break;
//				}
//				for (i=0; i<cnt; ++i)
//					drain[i] &= mask;
//			}
//			int written = tty_insert_flip_string(end->other->tty, drain, cnt);
//			if (written > 0)
//			{
//				//dprintf("%s - #%d -> #%d: copied %d bytes\n", __FUNCTION__, end->tty->index, end->other->tty->index, written);
//				tty_flip_buffer_push(end->other->tty);
//			}
//		}
//	}
//
////	if (kfifo_len(&end->fifo) < WAKEUP_CHARS)
//		tty_wakeup(end->tty);
//}

//static void handle_termios(struct tty_struct *tty)
//{
//	struct nullmodem_end *end = tty->driver_data;
//
//	speed_t speed = tty_get_baud_rate(tty);
//	if (speed == 0)
//		change_pins(end, 0, TIOCM_DTR|TIOCM_RTS);
//	else
//		change_pins(end, TIOCM_DTR|TIOCM_RTS, 0);
//
//	unsigned int cflag = tty->termios->c_cflag;
//	end->char_length = 2;
//	switch (cflag & CSIZE)
//	{
//	case CS5:	end->char_length +=5;	break;
//	case CS6:	end->char_length +=6;	break;
//	case CS7:	end->char_length +=7;	break;
//	default:
//	case CS8:	end->char_length +=8;	break;
//	}
//	if (cflag & PARENB) end->char_length += 1;
//	if (cflag & CSTOPB) end->char_length += 1;
//	end->char_length *= FACTOR;
//
//	tty->hw_stopped = (tty->termios->c_cflag&CRTSCTS)
//					&& !(get_pins(end) & TIOCM_CTS);
//}

// ########################################################################
// # Module TTY routines
// ########################################################################
static int nullmodem_open(struct tty_struct *tty, struct file *file)
{
	struct nullmodem_device *nm_device = nullmodem_devices[tty->index];
	struct tty_port *nm_port = &nm_device->tport;
//	unsigned long flags;
	int status;

	printd("#%d: %s:- TTY count: %d\n", tty->index, __FUNCTION__, tty->count);

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	status = tty_port_open(nm_port, tty, file);

	if(!status) {
		/* save our structure within the tty structure */
		tty->driver_data = nm_device;
	}

	return status;

//	if (tty->count > 1) return 0;
//
//	spin_lock_irqsave(&nm_device->slock, flags);
//	tty->driver_data = nm_device;
//	end->nominal_bit_count = 0;
//	end->actual_bit_count = 0;
//	handle_termios(tty);
//	spin_unlock_irqrestore(&nm_device->slock, flags);
//
//	return 0;
}

static void nullmodem_close(struct tty_struct *tty, struct file *file)
{
	struct nullmodem_device *nm_device = tty->driver_data;
	struct tty_port *nm_port;
//	unsigned long flags;

	printd("#%d: %s:- TTY count: %d\n", tty->index, __FUNCTION__, tty->count);

	if (nm_device) {
		nm_port = &nm_device->tport;
		tty_port_close(nm_port, tty, file);
	}

//	if (tty->count > 1) return;
//
//	spin_lock_irqsave(&nm_device->slock, flags);
//	change_pins(end, 0, TIOCM_RTS|TIOCM_DTR);
//	tty->hw_stopped = 1;
//	spin_unlock_irqrestore(&nm_device->slock, flags);
//
//	wake_up_interruptible(&tty->read_wait);
//	wake_up_interruptible(&tty->write_wait);
}

//static int nullmodem_write(struct tty_struct *tty, const unsigned char *buffer, int count)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//	int written = 0;
//
//	if (tty->stopped)
//	{
//		dprintf("%s - #%d %d bytes --> 0 (tty stopped)\n", __FUNCTION__, tty->index, count);
//		return 0;
//	}
//
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
//	written = kfifo_in(&end->fifo, buffer, count);
//#else
//	written = __kfifo_put(end->fifo, buffer, count);
//#endif
//	//dprintf("%s - #%d %d bytes --> %d written\n", __FUNCTION__, tty->index, count, written);
//	return written;
//}

//static int nullmodem_write_room(struct tty_struct *tty)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	int room = 0;
//
//	if (tty->stopped)
//	{
//		dprintf("%s - #%d --> %d (tty stopped)\n", __FUNCTION__, tty->index, room);
//		return 0;
//	}
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
//	room = kfifo_avail(&end->fifo);
//#else
//	room = TX_BUF_SIZE - __kfifo_len(end->fifo);
//#endif
//	//dprintf("%s - #%d --> %d\n", __FUNCTION__, tty->index, room);
//	return room;
//}

//static int nullmodem_ioctl_tiocgserial(struct tty_struct *tty, unsigned long arg)
//{
//	struct nullmodem_device *nm_device = &nullmodem_devices[tty->index];
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

//static int nullmodem_ioctl_tiocgicount(struct tty_struct *tty, unsigned long arg)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//	struct serial_icounter_struct icount;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	spin_lock_irqsave(&end->pair->spin, flags);
//	icount.cts			= end->icount.cts;
//	icount.dsr			= end->icount.dsr;
//	icount.rng			= end->icount.rng;
//	icount.dcd			= end->icount.dcd;
//	icount.rx			= end->icount.rx;
//	icount.tx			= end->icount.tx;
//	icount.frame		= end->icount.frame;
//	icount.overrun		= end->icount.overrun;
//	icount.parity		= end->icount.parity;
//	icount.brk			= end->icount.brk;
//	icount.buf_overrun	= end->icount.buf_overrun;
//	spin_unlock_irqrestore(&end->pair->spin, flags);
//
//	if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
//		return -EFAULT;
//	return 0;
//}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
static int nullmodem_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#else
static int nullmodem_ioctl(struct tty_struct *tty, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	printd("#%d: %s:- cmd:0x%x\n", tty->index, __FUNCTION__, cmd);

	if (cmd == TCGETS || cmd == TCSETS)
		return -ENOIOCTLCMD;

//	switch (cmd)
//	{
//	case TIOCGSERIAL:
//		return nullmodem_ioctl_tiocgserial(tty, arg);
//		break;
//	case TIOCMIWAIT:
//		return nullmodem_ioctl_tiocmiwait(tty, arg);
//		break;
//	case TIOCGICOUNT:
//		return nullmodem_ioctl_tiocgicount(tty, arg);
//		break;
//	}

	return -ENOIOCTLCMD;
}

//static void nullmodem_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//	unsigned int cflag;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	cflag = tty->termios->c_cflag;
//
//	/* check that they really want us to change something */
//	if (old_termios)
//	{
//		if (cflag == old_termios->c_cflag
//		&& RELEVANT_IFLAG(tty->termios->c_iflag) == RELEVANT_IFLAG(old_termios->c_iflag))
//		{
//			dprintf(" - nothing to change...\n");
//			return;
//		}
//	}
//	spin_lock_irqsave(&end->pair->spin, flags);
//	handle_termios(tty);
//	spin_unlock_irqrestore(&end->pair->spin, flags);
//
//#ifdef SCULL_DEBUG
//	speed_t speed = tty_get_baud_rate(tty);
//	dprintf(" - baud = %u", speed);
//	dprintf(" - ispeed = %u", tty->termios->c_ispeed);
//	dprintf(" - ospeed = %u", tty->termios->c_ospeed);
//
//	/* get the byte size */
//	switch (cflag & CSIZE)
//	{
//	case CS5:	dprintf(" - data bits = 5\n");	break;
//	case CS6:	dprintf(" - data bits = 6\n");	break;
//	case CS7:	dprintf(" - data bits = 7\n");	break;
//	default:
//	case CS8:	dprintf(" - data bits = 8\n");	break;
//	}
//
//	/* determine the parity */
//	if (cflag & PARENB)
//		if (cflag & PARODD)
//			dprintf(" - parity = odd\n");
//		else
//			dprintf(" - parity = even\n");
//	else
//		dprintf(" - parity = none\n");
//
//	/* figure out the stop bits requested */
//	if (cflag & CSTOPB)
//		dprintf(" - stop bits = 2\n");
//	else
//		dprintf(" - stop bits = 1\n");
//
//	/* figure out the hardware flow control settings */
//	if (cflag & CRTSCTS)
//		dprintf(" - RTS/CTS is enabled\n");
//	else
//		dprintf(" - RTS/CTS is disabled\n");
//
//	/* determine software flow control */
//	/* if we are implementing XON/XOFF, set the start and
//	 * stop character in the device */
//	/* if we are implementing INBOUND XON/XOFF */
//	if (I_IXOFF(tty))
//		dprintf(" - INBOUND XON/XOFF is enabled, "
//			"XON = %2x, XOFF = %2x\n", START_CHAR(tty), STOP_CHAR(tty));
//	else
//		dprintf(" - INBOUND XON/XOFF is disabled\n");
//
//	/* if we are implementing OUTBOUND XON/XOFF */
//	if (I_IXON(tty))
//		dprintf(" - OUTBOUND XON/XOFF is enabled, "
//			"XON = %2x, XOFF = %2x\n", START_CHAR(tty), STOP_CHAR(tty));
//	else
//		dprintf(" - OUTBOUND XON/XOFF is disabled\n");
//#endif
//}

//static void nullmodem_throttle(struct tty_struct * tty)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	if (I_IXOFF(tty))
//		nullmodem_send_xchar(tty, STOP_CHAR(tty));
//
//	if (tty->termios->c_cflag & CRTSCTS)
//	{
//		spin_lock_irqsave(&end->pair->spin, flags);
//		change_pins(end, 0, TIOCM_RTS);
//		spin_unlock_irqrestore(&end->pair->spin, flags);
//	}
//}

//static void nullmodem_unthrottle(struct tty_struct * tty)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	if (tty->termios->c_cflag & CRTSCTS)
//	{
//		spin_lock_irqsave(&end->pair->spin, flags);
//		change_pins(end, TIOCM_RTS, 0);
//		spin_unlock_irqrestore(&end->pair->spin, flags);
//	}
//	if (I_IXOFF(tty))
//		nullmodem_send_xchar(tty, START_CHAR(tty));
//}

//static void nullmodem_send_xchar(struct tty_struct *tty, char ch)
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//
//	dprintf("%s - #%d\n", __FUNCTION__, tty->index);
//
//	end->xchar = ch;
//}

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
//static int nullmodem_tiocmget(struct tty_struct *tty)
//#else
//static int nullmodem_tiocmget(struct tty_struct *tty, struct file *filp)
//#endif
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//	int retval = -EINVAL;
//
//	spin_lock_irqsave(&end->pair->spin, flags);
//	retval = get_pins(end);
//	spin_unlock_irqrestore(&end->pair->spin, flags);
//
//	//dprintf("%s - #%d --> 0x%x\n", __FUNCTION__, tty->index, retval);
//	return retval;
//}

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
//static int nullmodem_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
//#else
//static int nullmodem_tiocmset(struct tty_struct *tty, struct file *filp, unsigned int set, unsigned int clear)
//#endif
//{
//	struct nullmodem_end *end = tty->driver_data;
//	unsigned long flags;
//
//	dprintf("%s - #%d set:0x%x clear:0x%x\n", __FUNCTION__,
//			tty->index, set, clear);
//
//	spin_lock_irqsave(&end->pair->spin, flags);
//	change_pins(end, set, clear);
//	spin_unlock_irqrestore(&end->pair->spin, flags);
//	return 0;
//}

// ########################################################################
// # Supported TTY operations
// ########################################################################
static struct tty_operations nm_serial_ops =
{
	.open		= nullmodem_open,
	.close		= nullmodem_close,
//	.write		= nullmodem_write,
//	//.put_char = ,
//	.write_room = nullmodem_write_room,
	.ioctl		= nullmodem_ioctl,
//	.set_termios	= nullmodem_set_termios,
//	.throttle	= nullmodem_throttle,
//	.unthrottle	= nullmodem_unthrottle,
//	//.send_xchar	= nullmodem_send_xchar,
//	.tiocmget	= nullmodem_tiocmget,
//	.tiocmset	= nullmodem_tiocmset,
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
	if (kfifo_alloc(&nm_device->tx_fifo, tx_buffer_size, GFP_KERNEL))
		goto exit;
#else
	nm_device->tx_fifo = kfifo_alloc(tx_buffer_size, GFP_KERNEL, NULL);
	if (!nm_device->tx_fifo)
		goto exit;
#endif
	err = 0;
exit:
	return err;
}

static void nullmodem_port_shutdown(struct tty_port *tport){
	struct nullmodem_device *nm_device;

	printd("%s\n", __FUNCTION__);

	nm_device = container_of(tport, struct nullmodem_device, tport);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
	kfifo_free(&nm_device->tx_fifo);
#else
	kfifo_free(nm_device->tx_fifo);
	nm_device->tx_fifo = NULL;
#endif
}

// ########################################################################
// # TTY port operations
// ########################################################################
static const struct tty_port_operations nm_port_ops = {
	.activate	= nullmodem_port_activate,
	.shutdown	= nullmodem_port_shutdown,
};

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
		} else if (num_devices == 0) {
			nm_device->paired_with = nm_device;			// In test mode, this is the other device
		} else {
			nm_other = nm_device;					// Save pointer to create a pair of linked devices
		}

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

		nm_device->registered = true;
		printd("Initialised nullmodem device %u.\n", i);
	}


//	init_timer(&nullmodem_timer);
//	setup_timer(&nullmodem_timer, nullmodem_timer_proc, 0);

//	for (i = 0; i < NULLMODEM_PAIRS; ++i)
//	{
//		struct nullmodem_pair *pair = &pair_table[i];
//		memset(pair, 0, sizeof(*pair));
//		pair->a.other = &pair->b;
//		pair->a.pair = pair;
//		pair->b.other = &pair->a;
//		pair->b.pair = pair;
//		pair->a.char_length = 10 * FACTOR;
//		pair->b.char_length = 10 * FACTOR;
//		spin_lock_init(&pair->spin);
//		init_waitqueue_head(&pair->control_lines_wait);
//		dprintf("%s - initialized pair %d -> %p\n", __FUNCTION__, i, pair);
//	}

//	last_timer_jiffies = jiffies;
//	nullmodem_timer.expires = last_timer_jiffies + TIMER_INTERVAL;
//	add_timer(&nullmodem_timer);

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

//	del_timer_sync(&nullmodem_timer);

	// Unregister devices
	for (i = 0; i < num_devices; ++i) {
		nm_device = nullmodem_devices[i];
		if ((nm_device != NULL) && (nm_device->registered)) {
			if (nm_device->tport.count) nullmodem_port_shutdown(&nm_device->tport);
			tty_unregister_device(nullmodem_tty_driver, i);
		}
	}
	// Free devices
	for (i = 0; i < num_devices; ++i) {
		nm_device = nullmodem_devices[i];
		if (nm_device != NULL) {
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
