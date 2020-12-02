
#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR "Celliwig <celliwig@nym.hush.com>"
#define DRIVER_DESC "NullModem Driver"

#define LOG_PREFIX "nullmodem: "
#define DEBUG = 1

#ifdef DEBUG
#       define printd(...) pr_alert(LOG_PREFIX __VA_ARGS__)
#else
#       define printd(...) do {} while (0)
#endif
#define printe(...) pr_err(LOG_PREFIX __VA_ARGS__)
#define printi(...) pr_info(LOG_PREFIX __VA_ARGS__)
#define printn(...) pr_notice(LOG_PREFIX __VA_ARGS__)

#define NULLMODEM_MAJOR		0	/* Auto assign major number */
#define NULLMODEM_PAIRS		1

#define MAX_DEVICES		10
#define DEFAULT_BUF_SIZE	1024
//#define TIMER_INTERVAL	(HZ/20)
////#define TIMER_INTERVAL	HZ
//#define WAKEUP_CHARS		256
//#define FACTOR 10

//#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

struct nullmodem_device
{
	struct device			*dev;
	struct tty_port			tport;
	struct nullmodem_device		*paired_with;		/* Pointer to device paired with this one */
	unsigned int			control_lines;		/* Control lines */
	struct async_icount		icount;			/* Device statistics */
	struct serial_struct		serial;			/* Serial port config */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
	struct kfifo			tx_fifo;
#else
	struct kfifo			*tx_fifo;
#endif
//	unsigned char			xchar;
//	unsigned char			char_length;
//	unsigned			nominal_bit_count;
//	unsigned			actual_bit_count;
	spinlock_t			slock;			/* Locks this structure */
	bool				registered;
};

//struct nullmodem_pair
//{
//	spinlock_t				spin;		/* locks this structure */
//	struct nullmodem_end	a;
//	struct nullmodem_end	b;
//	int						control_lines;	/* control lines as seen from end a */
//	wait_queue_head_t		control_lines_wait;
//};
