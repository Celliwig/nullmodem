
#define DRIVER_VERSION "v1.2"
#define DRIVER_AUTHOR "Celliwig <celliwig@nym.hush.com>"
#define DRIVER_DESC "NullModem Driver"

//#ifdef SCULL_DEBUG
//#define dprintf(fmt, args...) printk(KERN_DEBUG fmt, ##args)
//#else
//#define dprintf(fmt, args...)
//#endif

#define NULLMODEM_MAJOR		240	/* experimental range */
#define NULLMODEM_PAIRS		1

//#define TIMER_INTERVAL (HZ/20)
////#define TIMER_INTERVAL HZ
#define DEFAULT_BUF_SIZE 4096
//#define WAKEUP_CHARS 256

//struct nullmodem_pair;
//struct nullmodem_end
//{
//	struct tty_struct		*tty;		/* pointer to the tty for this device */
//	struct nullmodem_end	*other;
//	struct nullmodem_pair	*pair;
//	struct async_icount		icount;
//	struct serial_struct	serial;
//	unsigned char			xchar;
//	unsigned char			char_length;
//	unsigned				nominal_bit_count;
//	unsigned				actual_bit_count;
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
//	struct kfifo			fifo;
//#else
//	struct kfifo			*fifo;
//#endif
//};

//struct nullmodem_pair
//{
//	spinlock_t				spin;		/* locks this structure */
//	struct nullmodem_end	a;
//	struct nullmodem_end	b;
//	int						control_lines;	/* control lines as seen from end a */
//	wait_queue_head_t		control_lines_wait;
//};
//static struct nullmodem_pair pair_table[NULLMODEM_PAIRS];

//static struct timer_list nullmodem_timer;
