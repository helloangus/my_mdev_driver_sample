#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/vfio.h>
#include <linux/iommu.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/file.h>
#include <linux/mdev.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <uapi/linux/serial_reg.h>
#include <linux/eventfd.h>
#include <linux/anon_inodes.h>
#include <linux/vmalloc.h>

#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/delay.h>

/*
 * #defines
 */

#define VERSION_STRING  "0.1"
#define DRIVER_AUTHOR   "Angus Lee"

#define MMDEV_CLASS_NAME "my_mdev"

#define MMDEV_NAME   "my_mdev_vfio"

#define MMDEV_CONFIG_SPACE_SIZE  0xff
#define MMDEV_IO_BAR_SIZE        0x8
#define MMDEV_MMIO_BAR_SIZE      0x100000

#define STORE_LE16(addr, val)   (*(u16 *)addr = val)
#define STORE_LE32(addr, val)   (*(u32 *)addr = val)

#define MAX_FIFO_SIZE   16

#define CIRCULAR_BUF_INC_IDX(idx)    (idx = (idx + 1) & (MAX_FIFO_SIZE - 1))

#define MMDEV_VFIO_PCI_OFFSET_SHIFT   40

#define MMDEV_VFIO_PCI_OFFSET_TO_INDEX(off)   (off >> MMDEV_VFIO_PCI_OFFSET_SHIFT)
#define MMDEV_VFIO_PCI_INDEX_TO_OFFSET(index) \
				((u64)(index) << MMDEV_VFIO_PCI_OFFSET_SHIFT)
#define MMDEV_VFIO_PCI_OFFSET_MASK    \
				(((u64)(1) << MMDEV_VFIO_PCI_OFFSET_SHIFT) - 1)
#define MAX_MMDEVS	24

#define LOG_DIR "/var/log/mdev"
#define LOG_PREFIX "mdev_dsched"
#define QUEUE_SIZE 4096

/*
 * Global Structures
 */

static struct mmdev_dev {
	dev_t		vd_devt;
	struct class	*vd_class;
	struct cdev	vd_cdev;
	struct idr	vd_idr;
	struct device	dev;
	struct mdev_parent parent;

	struct list_head instances;
    struct mutex list_lock;
	struct task_struct *sched_thread;
    wait_queue_head_t wake_sched;
	bool stop;

	char log_path[256];
} mmdev_dev;

struct mdev_region_info {
	u64 start;
	u64 phys_start;
	u32 size;
	u64 vfio_offset;
};

/* loop back buffer */
struct rxtx {
	u8 fifo[MAX_FIFO_SIZE];
	u8 head, tail;
	u8 count;
};

struct serial_port {
	u8 uart_reg[8];         /* 8 registers */
	struct rxtx rxtx;       /* loop back buffer */
	bool dlab;
	bool overrun;
	u16 divisor;
	u8 fcr;                 /* FIFO control register */
	u8 max_fifo_size;
	u8 intr_trigger_level;  /* interrupt trigger level */
};

/* State of each mdev device */
struct mdev_state {
	struct vfio_device vdev;
	struct eventfd_ctx *intx_evtfd;
	struct eventfd_ctx *msi_evtfd;
	int irq_index;
	u8 *vconfig;
	struct mutex ops_lock;
	struct mdev_device *mdev;
	struct mdev_region_info region_info[VFIO_PCI_NUM_REGIONS];
	u32 bar_mask[VFIO_PCI_NUM_REGIONS];

	struct vfio_device_info dev_info;

	int nr_ports;
	struct serial_port s[2];
	struct mutex rxtx_lock;

	struct list_head list_node;
    struct kfifo queue;            /* 待写入队列 (bytes) */
    unsigned int priority;
    unsigned int timeslice_ms;
    bool running;                  /* 是否当前持有时间片 */
	struct mutex write_lock;
	char uuid[64];

	void *memblk;
	u64 memsize;

	u8 intx_mask:1;
};

static struct mmdev_type {
	struct mdev_type type;
	int nr_ports;
} mmdev_types[3] = {
	{ .nr_ports = 1, .type.sysfs_name = "1",
	  .type.pretty_name = "Single port serial" },
	{ .nr_ports = 2, .type.sysfs_name = "2",
	  .type.pretty_name = "Dual port serial" },
	{ .nr_ports = 0, .type.sysfs_name = "WO",
	  .type.pretty_name = "Write only port" },
};

static struct mdev_type *mmdev_mdev_types[] = {
	&mmdev_types[0].type,
	&mmdev_types[1].type,
	&mmdev_types[2].type,
};

static atomic_t mdev_avail_ports = ATOMIC_INIT(MAX_MMDEVS);

static const struct file_operations vd_fops = {
	.owner          = THIS_MODULE,
};

static const struct vfio_device_ops mmdev_dev_ops;

/* Helper functions */

static void dump_buffer(u8 *buf, uint32_t count)
{
#if defined(DEBUG)
	int i;

	pr_info("Buffer:\n");
	for (i = 0; i < count; i++) {
		pr_info("%2x ", *(buf + i));
		if ((i + 1) % 16 == 0)
			pr_info("\n");
	}
#endif
}

static bool is_intx(struct mdev_state *mdev_state)
{
	return mdev_state->irq_index == VFIO_PCI_INTX_IRQ_INDEX;
}

static bool is_msi(struct mdev_state *mdev_state)
{
	return mdev_state->irq_index == VFIO_PCI_MSI_IRQ_INDEX;
}

static bool is_noirq(struct mdev_state *mdev_state)
{
	return !is_intx(mdev_state) && !is_msi(mdev_state);
}

static void mmdev_trigger_interrupt(struct mdev_state *mdev_state)
{
	lockdep_assert_held(&mdev_state->ops_lock);

	if (is_msi(mdev_state)) {
		if (mdev_state->msi_evtfd)
			eventfd_signal(mdev_state->msi_evtfd);
	} else if (is_intx(mdev_state)) {
		if (mdev_state->intx_evtfd && !mdev_state->intx_mask) {
			eventfd_signal(mdev_state->intx_evtfd);
			mdev_state->intx_mask = true;
		}
	}
}

static void mmdev_create_config_space(struct mdev_state *mdev_state)
{
	/* PCI dev ID */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x0], 0x32534348);

	/* Control: I/O+, Mem-, BusMaster- */
	STORE_LE16((u16 *) &mdev_state->vconfig[0x4], 0x0001);

	/* Status: capabilities list absent */
	STORE_LE16((u16 *) &mdev_state->vconfig[0x6], 0x0200);

	/* Rev ID */
	mdev_state->vconfig[0x8] =  0x10;

	/* programming interface class : 16550-compatible serial controller */
	mdev_state->vconfig[0x9] =  0x02;

	/* Sub class : 00 */
	mdev_state->vconfig[0xa] =  0x00;

	/* Base class : Simple Communication controllers */
	mdev_state->vconfig[0xb] =  0x07;

	/* base address registers */
	/* BAR0: IO space */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x10], 0x000001);
	mdev_state->bar_mask[0] = ~(MMDEV_IO_BAR_SIZE) + 1;

	if (mdev_state->nr_ports == 2) {
		/* BAR1: IO space */
		STORE_LE32((u32 *) &mdev_state->vconfig[0x14], 0x000001);
		mdev_state->bar_mask[1] = ~(MMDEV_IO_BAR_SIZE) + 1;
	}

	/* BAR2: MMIO space (mappable) */
	/* indicate mem BAR, 32-bit prefetchable=0, mem type 32-bit */
	/* Configure BAR2 as a 32-bit memory BAR that is not prefetchable */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x18], 0x000000);
	/* set mask for mmio backing size */
	/* Initialize the BAR mask to determine the size of the MMIO backing space */
	mdev_state->bar_mask[2] = ~(MMDEV_MMIO_BAR_SIZE) + 1;

	/* BAR3: MMIO space (mappable) */
	/* indicate mem BAR, 32-bit prefetchable=0, mem type 32-bit */
	/* Configure BAR3 as a 32-bit memory BAR that is not prefetchable */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x1c], 0x000000);
	/* set mask for mmio backing size */
	/* Initialize the BAR mask to determine the size of the MMIO backing space */
	mdev_state->bar_mask[3] = ~(QUEUE_SIZE) + 1;

	/* Subsystem ID */
	STORE_LE32((u32 *) &mdev_state->vconfig[0x2c], 0x32534348);

	mdev_state->vconfig[0x34] =  0x00;   /* Cap Ptr */
	mdev_state->vconfig[0x3d] =  0x01;   /* interrupt pin (INTA#) */

	/* Vendor specific data */
	mdev_state->vconfig[0x40] =  0x23;
	mdev_state->vconfig[0x43] =  0x80;
	mdev_state->vconfig[0x44] =  0x23;
	mdev_state->vconfig[0x48] =  0x23;
	mdev_state->vconfig[0x4c] =  0x23;

	mdev_state->vconfig[0x60] =  0x50;
	mdev_state->vconfig[0x61] =  0x43;
	mdev_state->vconfig[0x62] =  0x49;
	mdev_state->vconfig[0x63] =  0x20;
	mdev_state->vconfig[0x64] =  0x53;
	mdev_state->vconfig[0x65] =  0x65;
	mdev_state->vconfig[0x66] =  0x72;
	mdev_state->vconfig[0x67] =  0x69;
	mdev_state->vconfig[0x68] =  0x61;
	mdev_state->vconfig[0x69] =  0x6c;
	mdev_state->vconfig[0x6a] =  0x2f;
	mdev_state->vconfig[0x6b] =  0x55;
	mdev_state->vconfig[0x6c] =  0x41;
	mdev_state->vconfig[0x6d] =  0x52;
	mdev_state->vconfig[0x6e] =  0x54;
}

static void handle_pci_cfg_write(struct mdev_state *mdev_state, u16 offset,
				 u8 *buf, u32 count)
{
	u32 cfg_addr, bar_mask, bar_index = 0;

	switch (offset) {
	case 0x04: /* device control */
	case 0x06: /* device status */
		/* do nothing */
		break;
	case 0x3c:  /* interrupt line */
		mdev_state->vconfig[0x3c] = buf[0];
		break;
	case 0x3d:
		/*
		 * Interrupt Pin is hardwired to INTA.
		 * This field is write protected by hardware
		 */
		break;
	case 0x10:  /* BAR0 */
	case 0x14:  /* BAR1 */
		if (offset == 0x10)
			bar_index = 0;
		else if (offset == 0x14)
			bar_index = 1;

		if ((mdev_state->nr_ports == 1) && (bar_index == 1)) {
			STORE_LE32(&mdev_state->vconfig[offset], 0);
			break;
		}

		cfg_addr = *(u32 *)buf;
		pr_info("BAR%d addr 0x%x\n", bar_index, cfg_addr);

		if (cfg_addr == 0xffffffff) {
			bar_mask = mdev_state->bar_mask[bar_index];
			cfg_addr = (cfg_addr & bar_mask);
		}

		cfg_addr |= (mdev_state->vconfig[offset] & 0x3ul);
		STORE_LE32(&mdev_state->vconfig[offset], cfg_addr);
		break;
	case 0x18:  /* BAR2 */
	case 0x1c:  /* BAR3 */
		if (offset == 0x18)
			bar_index = 2;
		else if (offset == 0x1c)
			bar_index = 3;

		/* 
			* 从缓冲区获取guest写入的32位值
			* 注意：当前实现可能存在对齐问题，推荐使用get_unaligned_le32或memcpy后转换
			*/
		u32 guest_val = *(u32 *)buf;
		pr_info("BAR%d addr 0x%x\n", bar_index, guest_val);

		/* 
			* 处理BAR地址探测操作
			* 当guest写入0xffffffff时，表示进行BAR大小探测，
			* 此时返回预先设置的掩码值
			*/
		if (guest_val == 0xffffffff) {
			bar_mask = mdev_state->bar_mask[bar_index];
			guest_val = (guest_val & bar_mask);
		}

		/* 
			* 保留并恢复BAR寄存器的低位标志位
			* 包括IO/MEM标志位和类型位，确保功能标识不被覆盖
			*/
		guest_val |= (mdev_state->vconfig[offset] & 0x3ul);
		STORE_LE32(&mdev_state->vconfig[offset], guest_val);
		break;
	case 0x20:  /* BAR4 */
		STORE_LE32(&mdev_state->vconfig[offset], 0);
		break;
	default:
		pr_info("PCI config write @0x%x of %d bytes not handled\n",
			offset, count);
		break;
	}
}


// TODO
static void handle_bar_write(unsigned int index, struct mdev_state *mdev_state,
				u16 offset, u8 *buf, u32 count)
{
	u8 data = *buf;

	/* Handle data written by guest */
	switch (offset) {
	case UART_TX:
		/* if DLAB set, data is LSB of divisor */
		if (mdev_state->s[index].dlab) {
			mdev_state->s[index].divisor |= data;
			break;
		}

		mutex_lock(&mdev_state->rxtx_lock);

		/* save in TX buffer */
		if (mdev_state->s[index].rxtx.count <
				mdev_state->s[index].max_fifo_size) {
			mdev_state->s[index].rxtx.fifo[
					mdev_state->s[index].rxtx.head] = data;
			mdev_state->s[index].rxtx.count++;
			CIRCULAR_BUF_INC_IDX(mdev_state->s[index].rxtx.head);
			mdev_state->s[index].overrun = false;

			/*
			 * Trigger interrupt if receive data interrupt is
			 * enabled and fifo reached trigger level
			 */
			if ((mdev_state->s[index].uart_reg[UART_IER] &
						UART_IER_RDI) &&
			   (mdev_state->s[index].rxtx.count ==
				    mdev_state->s[index].intr_trigger_level)) {
				/* trigger interrupt */

				mmdev_trigger_interrupt(mdev_state);
			}
		} else {

			mdev_state->s[index].overrun = true;

			/*
			 * Trigger interrupt if receiver line status interrupt
			 * is enabled
			 */
			if (mdev_state->s[index].uart_reg[UART_IER] &
								UART_IER_RLSI)
				mmdev_trigger_interrupt(mdev_state);
		}
		mutex_unlock(&mdev_state->rxtx_lock);
		break;

	case UART_IER:
		/* if DLAB set, data is MSB of divisor */
		if (mdev_state->s[index].dlab)
			mdev_state->s[index].divisor |= (u16)data << 8;
		else {
			mdev_state->s[index].uart_reg[offset] = data;
			mutex_lock(&mdev_state->rxtx_lock);
			if ((data & UART_IER_THRI) &&
			    (mdev_state->s[index].rxtx.head ==
					mdev_state->s[index].rxtx.tail)) {

				mmdev_trigger_interrupt(mdev_state);
			}

			mutex_unlock(&mdev_state->rxtx_lock);
		}

		break;

	case UART_FCR:
		mdev_state->s[index].fcr = data;

		mutex_lock(&mdev_state->rxtx_lock);
		if (data & (UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT)) {
			/* clear loop back FIFO */
			mdev_state->s[index].rxtx.count = 0;
			mdev_state->s[index].rxtx.head = 0;
			mdev_state->s[index].rxtx.tail = 0;
		}
		mutex_unlock(&mdev_state->rxtx_lock);

		switch (data & UART_FCR_TRIGGER_MASK) {
		case UART_FCR_TRIGGER_1:
			mdev_state->s[index].intr_trigger_level = 1;
			break;

		case UART_FCR_TRIGGER_4:
			mdev_state->s[index].intr_trigger_level = 4;
			break;

		case UART_FCR_TRIGGER_8:
			mdev_state->s[index].intr_trigger_level = 8;
			break;

		case UART_FCR_TRIGGER_14:
			mdev_state->s[index].intr_trigger_level = 14;
			break;
		}

		/*
		 * Set trigger level to 1 otherwise or  implement timer with
		 * timeout of 4 characters and on expiring that timer set
		 * Recevice data timeout in IIR register
		 */
		mdev_state->s[index].intr_trigger_level = 1;
		if (data & UART_FCR_ENABLE_FIFO)
			mdev_state->s[index].max_fifo_size = MAX_FIFO_SIZE;
		else {
			mdev_state->s[index].max_fifo_size = 1;
			mdev_state->s[index].intr_trigger_level = 1;
		}

		break;

	case UART_LCR:
		if (data & UART_LCR_DLAB) {
			mdev_state->s[index].dlab = true;
			mdev_state->s[index].divisor = 0;
		} else
			mdev_state->s[index].dlab = false;

		mdev_state->s[index].uart_reg[offset] = data;
		break;

	case UART_MCR:
		mdev_state->s[index].uart_reg[offset] = data;

		if ((mdev_state->s[index].uart_reg[UART_IER] & UART_IER_MSI) &&
				(data & UART_MCR_OUT2)) {

			mmdev_trigger_interrupt(mdev_state);
		}

		if ((mdev_state->s[index].uart_reg[UART_IER] & UART_IER_MSI) &&
				(data & (UART_MCR_RTS | UART_MCR_DTR))) {

			mmdev_trigger_interrupt(mdev_state);
		}
		break;

	case UART_LSR:
	case UART_MSR:
		/* do nothing */
		break;

	case UART_SCR:
		mdev_state->s[index].uart_reg[offset] = data;
		break;

	default:
		break;
	}
}


// TODO
static void handle_bar_read(unsigned int index, struct mdev_state *mdev_state,
			    u16 offset, u8 *buf, u32 count)
{
	/* Handle read requests by guest */
	switch (offset) {
	case UART_RX:
		/* if DLAB set, data is LSB of divisor */
		if (mdev_state->s[index].dlab) {
			*buf  = (u8)mdev_state->s[index].divisor;
			break;
		}

		mutex_lock(&mdev_state->rxtx_lock);
		/* return data in tx buffer */
		if (mdev_state->s[index].rxtx.head !=
				 mdev_state->s[index].rxtx.tail) {
			*buf = mdev_state->s[index].rxtx.fifo[
						mdev_state->s[index].rxtx.tail];
			mdev_state->s[index].rxtx.count--;
			CIRCULAR_BUF_INC_IDX(mdev_state->s[index].rxtx.tail);
		}

		if (mdev_state->s[index].rxtx.head ==
				mdev_state->s[index].rxtx.tail) {
		/*
		 *  Trigger interrupt if tx buffer empty interrupt is
		 *  enabled and fifo is empty
		 */
			if (mdev_state->s[index].uart_reg[UART_IER] &
							 UART_IER_THRI)
				mmdev_trigger_interrupt(mdev_state);
		}
		mutex_unlock(&mdev_state->rxtx_lock);

		break;

	case UART_IER:
		if (mdev_state->s[index].dlab) {
			*buf = (u8)(mdev_state->s[index].divisor >> 8);
			break;
		}
		*buf = mdev_state->s[index].uart_reg[offset] & 0x0f;
		break;

	case UART_IIR:
	{
		u8 ier = mdev_state->s[index].uart_reg[UART_IER];
		*buf = 0;

		mutex_lock(&mdev_state->rxtx_lock);
		/* Interrupt priority 1: Parity, overrun, framing or break */
		if ((ier & UART_IER_RLSI) && mdev_state->s[index].overrun)
			*buf |= UART_IIR_RLSI;

		/* Interrupt priority 2: Fifo trigger level reached */
		if ((ier & UART_IER_RDI) &&
		    (mdev_state->s[index].rxtx.count >=
		      mdev_state->s[index].intr_trigger_level))
			*buf |= UART_IIR_RDI;

		/* Interrupt priotiry 3: transmitter holding register empty */
		if ((ier & UART_IER_THRI) &&
		    (mdev_state->s[index].rxtx.head ==
				mdev_state->s[index].rxtx.tail))
			*buf |= UART_IIR_THRI;

		/* Interrupt priotiry 4: Modem status: CTS, DSR, RI or DCD  */
		if ((ier & UART_IER_MSI) &&
		    (mdev_state->s[index].uart_reg[UART_MCR] &
				 (UART_MCR_RTS | UART_MCR_DTR)))
			*buf |= UART_IIR_MSI;

		/* bit0: 0=> interrupt pending, 1=> no interrupt is pending */
		if (*buf == 0)
			*buf = UART_IIR_NO_INT;

		/* set bit 6 & 7 to be 16550 compatible */
		*buf |= 0xC0;
		mutex_unlock(&mdev_state->rxtx_lock);
	}
	break;

	case UART_LCR:
	case UART_MCR:
		*buf = mdev_state->s[index].uart_reg[offset];
		break;

	case UART_LSR:
	{
		u8 lsr = 0;

		mutex_lock(&mdev_state->rxtx_lock);
		/* atleast one char in FIFO */
		if (mdev_state->s[index].rxtx.head !=
				 mdev_state->s[index].rxtx.tail)
			lsr |= UART_LSR_DR;

		/* if FIFO overrun */
		if (mdev_state->s[index].overrun)
			lsr |= UART_LSR_OE;

		/* transmit FIFO empty and tramsitter empty */
		if (mdev_state->s[index].rxtx.head ==
				 mdev_state->s[index].rxtx.tail)
			lsr |= UART_LSR_TEMT | UART_LSR_THRE;

		mutex_unlock(&mdev_state->rxtx_lock);
		*buf = lsr;
		break;
	}
	case UART_MSR:
		*buf = UART_MSR_DSR | UART_MSR_DDSR | UART_MSR_DCD;

		mutex_lock(&mdev_state->rxtx_lock);
		/* if AFE is 1 and FIFO have space, set CTS bit */
		if (mdev_state->s[index].uart_reg[UART_MCR] &
						 UART_MCR_AFE) {
			if (mdev_state->s[index].rxtx.count <
					mdev_state->s[index].max_fifo_size)
				*buf |= UART_MSR_CTS | UART_MSR_DCTS;
		} else
			*buf |= UART_MSR_CTS | UART_MSR_DCTS;
		mutex_unlock(&mdev_state->rxtx_lock);

		break;

	case UART_SCR:
		*buf = mdev_state->s[index].uart_reg[offset];
		break;

	default:
		break;
	}
}

static int host_write_all(struct mmdev_dev *p,
                          const char *buf, size_t len)
{
    struct file *filp;
    loff_t pos = 0;
    ssize_t written;

    if (!p || !p->log_path[0])
        return -EINVAL;

    filp = filp_open(p->log_path,
                     O_WRONLY | O_CREAT | O_APPEND,
                     0644);
    if (IS_ERR(filp))
        return PTR_ERR(filp);

    pr_info("%s: writing %zu bytes to log file\n",
            MMDEV_NAME, len);
    written = kernel_write(filp, buf, len, &pos);
    filp_close(filp, NULL);

    return written < 0 ? (int)written : 0;
}

static int sched_thread_fn(void *data)
{
    struct mmdev_dev *p = data;

	pr_info("%s: scheduler thread started\n", MMDEV_NAME);

    while (!kthread_should_stop()) {
        struct mdev_state *sel = NULL;
        struct mdev_state *iter;
        unsigned long best_prio = 0;
        bool found = false;

        /* pick the highest priority non-empty instance */
        mutex_lock(&p->list_lock);
        list_for_each_entry(iter, &p->instances, list_node) {
            if (!kfifo_is_empty(&iter->queue)) {
                found = true;
                if (iter->priority >= best_prio) {
                    best_prio = iter->priority;
                    sel = iter;
                }
            }
        }
        mutex_unlock(&p->list_lock);

        if (!found) {
            /* 没有数据，等待唤醒或超时。等待条件：stop 或 instances 不为空（轮询） */
            wait_event_interruptible_timeout(p->wake_sched,
                                             p->stop || !list_empty(&p->instances),
                                             msecs_to_jiffies(200));
            if (p->stop || kthread_should_stop())
                break;
            continue;
        }

        if (!sel)
            continue;

		pr_info("%s: scheduling mdev %s\n", MMDEV_NAME, sel->uuid);

        /* grant timeslice */
        mutex_lock(&sel->write_lock);
        sel->running = true;

        /* 从队列读数据并写到 host 文件（示例行为） */
        {
            unsigned int avail = kfifo_len(&sel->queue);
            unsigned int tocopy = min(avail, (unsigned int)(QUEUE_SIZE - 1));
            char *buf = kmalloc(tocopy + 2, GFP_KERNEL);
            if (buf) {
                unsigned int copied_bytes = kfifo_out(&sel->queue, buf, tocopy);
                if (copied_bytes > 0) {
                    host_write_all(p, "[", 1);
                    host_write_all(p, sel->uuid, strlen(sel->uuid));
                    host_write_all(p, "] \r\n", 4);
                    host_write_all(p, buf, copied_bytes);
                    host_write_all(p, "\n", 1);
                }
                kfree(buf);
            }
        }

        if (sel->timeslice_ms)
            msleep(sel->timeslice_ms);
        else
            msleep(100);

        sel->running = false;

        mutex_unlock(&sel->write_lock);

        if (kthread_should_stop() || p->stop)
            break;
    }

    return 0;
}

static ssize_t mdev_in_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
    struct mdev_state *mdev_state = dev_get_drvdata(dev);
    size_t tocopy;

    if (!mdev_state)
        return -EINVAL;

    /* 限制大小，避免一次放入过多 */
    tocopy = min(count, (size_t)(QUEUE_SIZE - 1));
    /* kfifo_in 接受 kernel buffer */
    tocopy = kfifo_in(&mdev_state->queue, buf, tocopy);

    /* 唤醒调度线程 */
	pr_info("%s: waking scheduler after enqueue %zu bytes to mdev %s\n",
			MMDEV_NAME, tocopy, mdev_state->uuid);
	wake_up_interruptible(&mmdev_dev.wake_sched);
    
    /* 返回写入的字节（note: sysfs expects count or consumed len; return count to mimic user write) */
    return count;
}

static ssize_t mdev_modify_priority_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
    struct mdev_state *mdev_state = dev_get_drvdata(dev);
    unsigned int new_prio;
    int ret;

    if (!mdev_state)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &new_prio);
    if (ret)
        return ret;

    mutex_lock(&mdev_state->write_lock);
    mdev_state->priority = new_prio;
    mutex_unlock(&mdev_state->write_lock);

    pr_info("%s: mdev %s priority changed to %u\n", MMDEV_NAME,
            mdev_state->uuid, mdev_state->priority);

    return count;
}

static void mdev_read_base(struct mdev_state *mdev_state)
{
	int index, pos;
	u32 start_lo, start_hi;
	u32 mem_type;

	pos = PCI_BASE_ADDRESS_0;

	for (index = 0; index <= VFIO_PCI_BAR5_REGION_INDEX; index++) {

		if (!mdev_state->region_info[index].size)
			continue;

		start_lo = (*(u32 *)(mdev_state->vconfig + pos)) &
			PCI_BASE_ADDRESS_MEM_MASK;
		mem_type = (*(u32 *)(mdev_state->vconfig + pos)) &
			PCI_BASE_ADDRESS_MEM_TYPE_MASK;

		switch (mem_type) {
		case PCI_BASE_ADDRESS_MEM_TYPE_64:
			start_hi = (*(u32 *)(mdev_state->vconfig + pos + 4));
			pos += 4;
			break;
		case PCI_BASE_ADDRESS_MEM_TYPE_32:
		case PCI_BASE_ADDRESS_MEM_TYPE_1M:
			/* 1M mem BAR treated as 32-bit BAR */
		default:
			/* mem unknown type treated as 32-bit BAR */
			start_hi = 0;
			break;
		}
		pos += 4;
		mdev_state->region_info[index].start = ((u64)start_hi << 32) |
							start_lo;
	}
}

static ssize_t mdev_access(struct mdev_state *mdev_state, u8 *buf, size_t count,
			   loff_t pos, bool is_write)
{
	unsigned int index;
	loff_t offset;
	int ret = 0;

	if (!buf)
		return -EINVAL;

	mutex_lock(&mdev_state->ops_lock);

	index = MMDEV_VFIO_PCI_OFFSET_TO_INDEX(pos);
	offset = pos & MMDEV_VFIO_PCI_OFFSET_MASK;
	switch (index) {
	case VFIO_PCI_CONFIG_REGION_INDEX:
		pr_info("%s: PCI config space %s at offset 0x%llx\n",
			 __func__, is_write ? "write" : "read", offset);
		if (is_write) {
			dump_buffer(buf, count);
			handle_pci_cfg_write(mdev_state, offset, buf, count);
		} else {
			memcpy(buf, (mdev_state->vconfig + offset), count);
			dump_buffer(buf, count);
		}

		break;

		/**
		 * 处理VFIO PCI设备BAR区域的读写访问
		 * 
		 * 处理PCI设备BAR0到BAR5区域的内存映射I/O访问。对于BAR2区域，
		 * 提供了特殊的处理逻辑来直接暴露原始的MMIO后备缓冲区。
		 */
	case VFIO_PCI_BAR0_REGION_INDEX ... VFIO_PCI_BAR5_REGION_INDEX:
		/* 如果当前BAR区域的起始地址未初始化，则读取基础配置信息 */
		if (!mdev_state->region_info[index].start)
			mdev_read_base(mdev_state);
		
		pr_info("%s: BAR%d %s at offset 0x%llx\n", __func__,
			index, is_write ? "write" : "read", offset);
		
		/* 特殊处理BAR2区域：直接暴露原始的MMIO后备缓冲区 */
		if (index == VFIO_PCI_BAR2_REGION_INDEX) {
			/* 检查访问是否超出内存块边界 */
			if (offset + count > mdev_state->memsize) {
				ret = -EINVAL;
				goto accessfailed;
			}
			if (is_write) {
				pr_info("MMIO BAR write\n");
				/* 打印写入缓冲区的每个字节数据 */
				for (int i = 0; i < count; i++)
					pr_info("write buffer[%d] is 0x%02x\n",
						i, *(buf + i));
				/* 将数据从输入缓冲区复制到设备内存块 */
				memcpy(mdev_state->memblk + offset, buf, count);
				dump_buffer(buf, count);
			} else {
				/* 从设备内存块复制数据到输出缓冲区 */
				memcpy(buf, mdev_state->memblk + offset, count);
				pr_info("MMIO BAR read\n");
				/* 打印读取缓冲区的每个字节数据 */
				for (int i = 0; i < count; i++)
					pr_info("read buffer[%d] is 0x%02x\n",
						i, *(buf + i));
				dump_buffer(buf, count);
			}
			break;
		} else if (index == VFIO_PCI_BAR3_REGION_INDEX) {
			/* Handle BAR3 region */
			if (is_write) {
				pr_info("MMIO BAR3 write\n");
				if (count > QUEUE_SIZE) {
					ret = -EINVAL;
					goto accessfailed;
				}
				/* 打印写入缓冲区的每个字节数据 */
				for (int i = 0; i < count; i++)
					pr_info("write queue[%d] is 0x%02x\n",
						i, *(buf + i));
				/* 将数据从输入缓冲区复制到设备内存块 */
				kfifo_in(&mdev_state->queue, buf, count);
				pr_info("%s: waking scheduler after enqueue %zu bytes to mdev %s\n",
					MMDEV_NAME, count, mdev_state->uuid);
				wake_up_interruptible(&mmdev_dev.wake_sched);
				dump_buffer(buf, count);
			} else {
				/* 从设备内存块复制数据到输出缓冲区 */
				unsigned int avail = kfifo_len(&mdev_state->queue);
				pr_info("MMIO BAR3 read %u bytes\n", avail);
			}
			break;
		}

		if (is_write) {
			dump_buffer(buf, count);
			handle_bar_write(index, mdev_state, offset, buf, count);
		} else {
			handle_bar_read(index, mdev_state, offset, buf, count);
			dump_buffer(buf, count);
		}
		break;

	default:
		ret = -1;
		goto accessfailed;
	}

	ret = count;


accessfailed:
	mutex_unlock(&mdev_state->ops_lock);

	return ret;
}

static int mmdev_init_dev(struct vfio_device *vdev)
{
	struct mdev_state *mdev_state =
		container_of(vdev, struct mdev_state, vdev);
	struct mdev_device *mdev = to_mdev_device(vdev->dev);
	struct mmdev_type *type =
		container_of(mdev->type, struct mmdev_type, type);
	int avail_ports = atomic_read(&mdev_avail_ports);
	int ret;

	do {
		if (avail_ports < type->nr_ports)
			return -ENOSPC;
	} while (!atomic_try_cmpxchg(&mdev_avail_ports,
				     &avail_ports,
				     avail_ports - type->nr_ports));

	mdev_state->nr_ports = type->nr_ports;
	mdev_state->irq_index = -1;
	mdev_state->s[0].max_fifo_size = MAX_FIFO_SIZE;
	mdev_state->s[1].max_fifo_size = MAX_FIFO_SIZE;
	mutex_init(&mdev_state->rxtx_lock);

	mdev_state->vconfig = kzalloc(MMDEV_CONFIG_SPACE_SIZE, GFP_KERNEL);
	if (!mdev_state->vconfig) {
		ret = -ENOMEM;
		goto err_nr_ports;
	}

	mutex_init(&mdev_state->ops_lock);
	mdev_state->mdev = mdev;
	mmdev_create_config_space(mdev_state);

	vdev->migration_flags = 0;
	vdev->mig_ops = NULL;
	vdev->log_ops = NULL;

	/* allocate backing memory for BAR2 (mappable) */
	mdev_state->memsize = MMDEV_MMIO_BAR_SIZE;
	mdev_state->memblk = vmalloc(mdev_state->memsize);
	if (!mdev_state->memblk) {
		ret = -ENOMEM;
		goto err_nr_ports;
	}

    if (kfifo_alloc(&mdev_state->queue, QUEUE_SIZE, GFP_KERNEL)) {
        return -ENOMEM;
    }
    mdev_state->priority = 1;
    mdev_state->timeslice_ms = 100;
    mutex_init(&mdev_state->write_lock);
    INIT_LIST_HEAD(&mdev_state->list_node);

    strscpy(mdev_state->uuid,
        dev_name(mdev_dev(mdev)),
        sizeof(mdev_state->uuid));

	mutex_lock(&mmdev_dev.list_lock);
	list_add_tail(&mdev_state->list_node, &mmdev_dev.instances);
	mutex_unlock(&mmdev_dev.list_lock);
	wake_up_interruptible(&mmdev_dev.wake_sched);

	return 0;

err_nr_ports:
	atomic_add(type->nr_ports, &mdev_avail_ports);
	return ret;
}

static DEVICE_ATTR(in, 0220, NULL, mdev_in_store);
static DEVICE_ATTR(priority, 0220, NULL, mdev_modify_priority_store);

static int mmdev_probe(struct mdev_device *mdev)
{
	pr_info("%s: probe mdev=%p dev=%p\n", MMDEV_NAME,
        mdev, mdev_dev(mdev));
	struct mdev_state *mdev_state;
	int ret;

	mdev_state = vfio_alloc_device(mdev_state, vdev, &mdev->dev,
				       &mmdev_dev_ops);
	if (IS_ERR(mdev_state))
		return PTR_ERR(mdev_state);

	ret = vfio_register_emulated_iommu_dev(&mdev_state->vdev);
	if (ret)
		goto err_put_vdev;
	dev_set_drvdata(&mdev->dev, mdev_state);

    ret = device_create_file(mdev_dev(mdev), &dev_attr_in);
    if (ret) {
        pr_err("%s: failed to create /sys/.../in (%d)\n", MMDEV_NAME, ret);
        return ret;
    }
    ret = device_create_file(mdev_dev(mdev), &dev_attr_priority);
    if (ret) {
        pr_err("%s: failed to create /sys/.../priority (%d)\n", MMDEV_NAME, ret);
        return ret;
    }

	return 0;

err_put_vdev:
	vfio_put_device(&mdev_state->vdev);
	return ret;
}

static void mmdev_release_dev(struct vfio_device *vdev)
{
	struct mdev_state *mdev_state =
		container_of(vdev, struct mdev_state, vdev);

	atomic_add(mdev_state->nr_ports, &mdev_avail_ports);
	if (mdev_state->memblk)
		vfree(mdev_state->memblk);

	mdev_state->running = false;
	mutex_lock(&mmdev_dev.list_lock);
	list_del_init(&mdev_state->list_node);
	mutex_unlock(&mmdev_dev.list_lock);
    kfifo_free(&mdev_state->queue);

	kfree(mdev_state->vconfig);
}


/**
 * mmdev_mmap - 将设备内存映射到用户空间
 * @vdev: VFIO设备结构体指针
 * @vma: 虚拟内存区域结构体指针
 *
 * 该函数实现VFIO设备的mmap操作，将设备的BAR2内存区域映射到用户空间。
 * 主要进行参数验证和内存映射操作。
 *
 * 返回值：
 *   成功时返回0，失败时返回负的错误码
 */
static int mmdev_mmap(struct vfio_device *vdev, struct vm_area_struct *vma)
{
	/* 获取mdev_state结构体指针 */
	struct mdev_state *mdev_state =
		container_of(vdev, struct mdev_state, vdev);

	/* 验证内存映射偏移量是否正确 */
	if (vma->vm_pgoff != MMDEV_VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR2_REGION_INDEX) >> PAGE_SHIFT)
		return -EINVAL;
	
	/* 验证虚拟内存区域的起始和结束地址是否有效 */
	if (vma->vm_end < vma->vm_start)
		return -EINVAL;
	
	/* 验证映射内存大小是否超过设备内存大小 */
	if (vma->vm_end - vma->vm_start > mdev_state->memsize)
		return -EINVAL;
	
	/* 验证是否为共享内存映射 */
	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	/* 执行实际的内存映射操作 */
	return remap_vmalloc_range(vma, mdev_state->memblk, 0);
}

static void mmdev_remove(struct mdev_device *mdev)
{
	pr_info("%s: remove mdev=%p dev=%p\n", MMDEV_NAME,
        mdev, mdev_dev(mdev));

    /* remove attribute and instance */
    device_remove_file(mdev_dev(mdev), &dev_attr_in);
    device_remove_file(mdev_dev(mdev), &dev_attr_priority);

	struct mdev_state *mdev_state = dev_get_drvdata(&mdev->dev);

	vfio_unregister_group_dev(&mdev_state->vdev);
	vfio_put_device(&mdev_state->vdev);

	pr_info("%s: removed mdev instance\n", MMDEV_NAME);
}

static ssize_t mmdev_read(struct vfio_device *vdev, char __user *buf,
			 size_t count, loff_t *ppos)
{
	pr_info("mmdev_dev: %s\n", __func__);
	pr_info("mmdev_dev: read count=%zu, ppos=%lld\n", count, *ppos);
	struct mdev_state *mdev_state =
		container_of(vdev, struct mdev_state, vdev);
	unsigned int done = 0;
	int ret;

	while (count) {
		size_t filled;

		if (count >= 4 && !(*ppos % 4)) {
			u32 val;

			ret =  mdev_access(mdev_state, (u8 *)&val, sizeof(val),
					   *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 4;
		} else if (count >= 2 && !(*ppos % 2)) {
			u16 val;

			ret = mdev_access(mdev_state, (u8 *)&val, sizeof(val),
					  *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 2;
		} else {
			u8 val;

			ret = mdev_access(mdev_state, (u8 *)&val, sizeof(val),
					  *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;

			filled = 1;
		}

		count -= filled;
		done += filled;
		*ppos += filled;
		buf += filled;
	}

	return done;

read_err:
	return -EFAULT;
}

static ssize_t mmdev_write(struct vfio_device *vdev, const char __user *buf,
		   size_t count, loff_t *ppos)
{
	pr_info("mmdev_dev: %s\n", __func__);
	pr_info("mmdev_dev: write count=%zu, ppos=%lld\n", count, *ppos);
	struct mdev_state *mdev_state =
		container_of(vdev, struct mdev_state, vdev);
	unsigned int done = 0;
	int ret;

	while (count) {
		size_t filled;

		if (count >= 4 && !(*ppos % 4)) {
			u32 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev_state, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 4;
		} else if (count >= 2 && !(*ppos % 2)) {
			u16 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev_state, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 2;
		} else {
			u8 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = mdev_access(mdev_state, (u8 *)&val, sizeof(val),
					  *ppos, true);
			if (ret <= 0)
				goto write_err;

			filled = 1;
		}
		count -= filled;
		done += filled;
		*ppos += filled;
		buf += filled;
	}

	return done;
write_err:
	return -EFAULT;
}

static void mmdev_disable_intx(struct mdev_state *mdev_state)
{
	if (mdev_state->intx_evtfd) {
		eventfd_ctx_put(mdev_state->intx_evtfd);
		mdev_state->intx_evtfd = NULL;
		mdev_state->intx_mask = false;
		mdev_state->irq_index = -1;
	}
}

static void mmdev_disable_msi(struct mdev_state *mdev_state)
{
	if (mdev_state->msi_evtfd) {
		eventfd_ctx_put(mdev_state->msi_evtfd);
		mdev_state->msi_evtfd = NULL;
		mdev_state->irq_index = -1;
	}
}

static int mmdev_set_irqs(struct mdev_state *mdev_state, uint32_t flags,
			 unsigned int index, unsigned int start,
			 unsigned int count, void *data)
{
	int ret = 0;

	mutex_lock(&mdev_state->ops_lock);
	switch (index) {
	case VFIO_PCI_INTX_IRQ_INDEX:
		switch (flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
		case VFIO_IRQ_SET_ACTION_MASK:
			if (!is_intx(mdev_state) || start != 0 || count != 1) {
				ret = -EINVAL;
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_NONE) {
				mdev_state->intx_mask = true;
			} else if (flags & VFIO_IRQ_SET_DATA_BOOL) {
				uint8_t mask = *(uint8_t *)data;

				if (mask)
					mdev_state->intx_mask = true;
			} else if (flags &  VFIO_IRQ_SET_DATA_EVENTFD) {
				ret = -ENOTTY; /* No support for mask fd */
			}
			break;
		case VFIO_IRQ_SET_ACTION_UNMASK:
			if (!is_intx(mdev_state) || start != 0 || count != 1) {
				ret = -EINVAL;
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_NONE) {
				mdev_state->intx_mask = false;
			} else if (flags & VFIO_IRQ_SET_DATA_BOOL) {
				uint8_t mask = *(uint8_t *)data;

				if (mask)
					mdev_state->intx_mask = false;
			} else if (flags &  VFIO_IRQ_SET_DATA_EVENTFD) {
				ret = -ENOTTY; /* No support for unmask fd */
			}
			break;
		case VFIO_IRQ_SET_ACTION_TRIGGER:
			if (is_intx(mdev_state) && !count &&
			    (flags & VFIO_IRQ_SET_DATA_NONE)) {
				mmdev_disable_intx(mdev_state);
				break;
			}

			if (!(is_intx(mdev_state) || is_noirq(mdev_state)) ||
			    start != 0 || count != 1) {
				ret = -EINVAL;
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_EVENTFD) {
				int fd = *(int *)data;
				struct eventfd_ctx *evt;

				mmdev_disable_intx(mdev_state);

				if (fd < 0)
					break;

				evt = eventfd_ctx_fdget(fd);
				if (IS_ERR(evt)) {
					ret = PTR_ERR(evt);
					break;
				}
				mdev_state->intx_evtfd = evt;
				mdev_state->irq_index = index;
				break;
			}

			if (!is_intx(mdev_state)) {
				ret = -EINVAL;
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_NONE) {
				mmdev_trigger_interrupt(mdev_state);
			} else if (flags & VFIO_IRQ_SET_DATA_BOOL) {
				uint8_t trigger = *(uint8_t *)data;

				if (trigger)
					mmdev_trigger_interrupt(mdev_state);
			}
			break;
		}
		break;
	case VFIO_PCI_MSI_IRQ_INDEX:
		switch (flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
		case VFIO_IRQ_SET_ACTION_MASK:
		case VFIO_IRQ_SET_ACTION_UNMASK:
			ret = -ENOTTY;
			break;
		case VFIO_IRQ_SET_ACTION_TRIGGER:
			if (is_msi(mdev_state) && !count &&
			    (flags & VFIO_IRQ_SET_DATA_NONE)) {
				mmdev_disable_msi(mdev_state);
				break;
			}

			if (!(is_msi(mdev_state) || is_noirq(mdev_state)) ||
			    start != 0 || count != 1) {
				ret = -EINVAL;
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_EVENTFD) {
				int fd = *(int *)data;
				struct eventfd_ctx *evt;

				mmdev_disable_msi(mdev_state);

				if (fd < 0)
					break;

				evt = eventfd_ctx_fdget(fd);
				if (IS_ERR(evt)) {
					ret = PTR_ERR(evt);
					break;
				}
				mdev_state->msi_evtfd = evt;
				mdev_state->irq_index = index;
				break;
			}

			if (!is_msi(mdev_state)) {
				ret = -EINVAL;
				break;
			}

			if (flags & VFIO_IRQ_SET_DATA_NONE) {
				mmdev_trigger_interrupt(mdev_state);
			} else if (flags & VFIO_IRQ_SET_DATA_BOOL) {
				uint8_t trigger = *(uint8_t *)data;

				if (trigger)
					mmdev_trigger_interrupt(mdev_state);
			}
			break;
		}
		break;
	case VFIO_PCI_MSIX_IRQ_INDEX:
		dev_dbg(mdev_state->vdev.dev, "%s: MSIX_IRQ\n", __func__);
		ret = -ENOTTY;
		break;
	case VFIO_PCI_ERR_IRQ_INDEX:
		dev_dbg(mdev_state->vdev.dev, "%s: ERR_IRQ\n", __func__);
		ret = -ENOTTY;
		break;
	case VFIO_PCI_REQ_IRQ_INDEX:
		dev_dbg(mdev_state->vdev.dev, "%s: REQ_IRQ\n", __func__);
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&mdev_state->ops_lock);
	return ret;
}

static int mmdev_get_region_info(struct mdev_state *mdev_state,
			 struct vfio_region_info *region_info,
			 u16 *cap_type_id, void **cap_type)
{
	unsigned int size = 0;
	u32 bar_index;

	bar_index = region_info->index;
	if (bar_index >= VFIO_PCI_NUM_REGIONS)
		return -EINVAL;

	mutex_lock(&mdev_state->ops_lock);

	switch (bar_index) {
	case VFIO_PCI_CONFIG_REGION_INDEX:
		size = MMDEV_CONFIG_SPACE_SIZE;
		break;
	case VFIO_PCI_BAR0_REGION_INDEX:
		size = MMDEV_IO_BAR_SIZE;
		break;
	case VFIO_PCI_BAR1_REGION_INDEX:
		if (mdev_state->nr_ports == 2)
			size = MMDEV_IO_BAR_SIZE;
		break;
	case VFIO_PCI_BAR2_REGION_INDEX:
			 size = MMDEV_MMIO_BAR_SIZE;
		break;
	case VFIO_PCI_BAR3_REGION_INDEX:
			 size = MMDEV_MMIO_BAR_SIZE;
		break;
	default:
		size = 0;
		break;
	}

	mdev_state->region_info[bar_index].size = size;
	mdev_state->region_info[bar_index].vfio_offset =
		MMDEV_VFIO_PCI_INDEX_TO_OFFSET(bar_index);

	region_info->size = size;
	region_info->offset = MMDEV_VFIO_PCI_INDEX_TO_OFFSET(bar_index);
	if ((bar_index == VFIO_PCI_BAR2_REGION_INDEX) || (bar_index == VFIO_PCI_BAR3_REGION_INDEX))
		region_info->flags = VFIO_REGION_INFO_FLAG_READ |
			VFIO_REGION_INFO_FLAG_WRITE | VFIO_REGION_INFO_FLAG_MMAP;
	else
		region_info->flags = VFIO_REGION_INFO_FLAG_READ |
			VFIO_REGION_INFO_FLAG_WRITE;
	mutex_unlock(&mdev_state->ops_lock);
	return 0;
}

static int mmdev_get_irq_info(struct vfio_irq_info *irq_info)
{
	if (irq_info->index != VFIO_PCI_INTX_IRQ_INDEX &&
	    irq_info->index != VFIO_PCI_MSI_IRQ_INDEX)
		return -EINVAL;

	irq_info->flags = VFIO_IRQ_INFO_EVENTFD;
	irq_info->count = 1;

	if (irq_info->index == VFIO_PCI_INTX_IRQ_INDEX)
		irq_info->flags |= VFIO_IRQ_INFO_MASKABLE |
				   VFIO_IRQ_INFO_AUTOMASKED;
	else
		irq_info->flags |= VFIO_IRQ_INFO_NORESIZE;

	return 0;
}

static int mmdev_get_device_info(struct vfio_device_info *dev_info)
{
	dev_info->flags = VFIO_DEVICE_FLAGS_PCI;
	dev_info->num_regions = VFIO_PCI_NUM_REGIONS;
	dev_info->num_irqs = VFIO_PCI_NUM_IRQS;

	return 0;
}

static long mmdev_ioctl(struct vfio_device *vdev, unsigned int cmd,
			unsigned long arg)
{
	pr_info("mmdev_dev: %s\n", __func__);
	pr_info("mmdev_dev: cmd=0x%x, arg=0x%lx\n", cmd, arg);
	struct mdev_state *mdev_state =
		container_of(vdev, struct mdev_state, vdev);
	int ret = 0;
	unsigned long minsz;

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
	{
		struct vfio_device_info info;

		minsz = offsetofend(struct vfio_device_info, num_irqs);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = mmdev_get_device_info(&info);
		if (ret)
			return ret;

		memcpy(&mdev_state->dev_info, &info, sizeof(info));

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_GET_REGION_INFO:
	{
		struct vfio_region_info info;
		u16 cap_type_id = 0;
		void *cap_type = NULL;

		minsz = offsetofend(struct vfio_region_info, offset);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = mmdev_get_region_info(mdev_state, &info, &cap_type_id,
					   &cap_type);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}

	case VFIO_DEVICE_GET_IRQ_INFO:
	{
		struct vfio_irq_info info;

		minsz = offsetofend(struct vfio_irq_info, count);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if ((info.argsz < minsz) ||
		    (info.index >= mdev_state->dev_info.num_irqs))
			return -EINVAL;

		ret = mmdev_get_irq_info(&info);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_SET_IRQS:
	{
		struct vfio_irq_set hdr;
		u8 *data = NULL, *ptr = NULL;
		size_t data_size = 0;

		minsz = offsetofend(struct vfio_irq_set, count);

		if (copy_from_user(&hdr, (void __user *)arg, minsz))
			return -EFAULT;

		ret = vfio_set_irqs_validate_and_prepare(&hdr,
						mdev_state->dev_info.num_irqs,
						VFIO_PCI_NUM_IRQS,
						&data_size);
		if (ret)
			return ret;

		if (data_size) {
			ptr = data = memdup_user((void __user *)(arg + minsz),
						 data_size);
			if (IS_ERR(data))
				return PTR_ERR(data);
		}

		ret = mmdev_set_irqs(mdev_state, hdr.flags, hdr.index, hdr.start,
				    hdr.count, data);

		kfree(ptr);
		return ret;
	}
	case VFIO_DEVICE_RESET:
	{
		pr_err("%s: VFIO_DEVICE_RESET not supported\n", MMDEV_NAME);
		return -ENOTTY;
	}
	}
	return -ENOTTY;
}

static ssize_t
sample_mdev_dev_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	return sprintf(buf, "This is MDEV %s\n", dev_name(dev));
}

static DEVICE_ATTR_RO(sample_mdev_dev);

static struct attribute *mdev_dev_attrs[] = {
	&dev_attr_sample_mdev_dev.attr,
	NULL,
};

static const struct attribute_group mdev_dev_group = {
	.name  = "vendor",
	.attrs = mdev_dev_attrs,
};

static const struct attribute_group *mdev_dev_groups[] = {
	&mdev_dev_group,
	NULL,
};

static unsigned int mmdev_get_available(struct mdev_type *mtype)
{
	struct mmdev_type *type = container_of(mtype, struct mmdev_type, type);

	return atomic_read(&mdev_avail_ports) / type->nr_ports;
}

static void mmdev_close(struct vfio_device *vdev)
{
	struct mdev_state *mdev_state =
				container_of(vdev, struct mdev_state, vdev);

	mmdev_disable_intx(mdev_state);
	mmdev_disable_msi(mdev_state);
}

static const struct vfio_device_ops mmdev_dev_ops = {
	.name = "vfio-mmdev",
	.init = mmdev_init_dev,
	.release = mmdev_release_dev,
	.read = mmdev_read,
	.write = mmdev_write,
	.mmap = mmdev_mmap,
	.ioctl = mmdev_ioctl,
	.bind_iommufd	= vfio_iommufd_emulated_bind,
	.unbind_iommufd	= vfio_iommufd_emulated_unbind,
	.attach_ioas	= vfio_iommufd_emulated_attach_ioas,
	.detach_ioas	= vfio_iommufd_emulated_detach_ioas,
	.close_device	= mmdev_close,
};

static struct mdev_driver mmdev_driver = {
	.device_api = VFIO_DEVICE_API_PCI_STRING,
	.driver = {
		.name = "mmdev",
		.owner = THIS_MODULE,
		.mod_name = KBUILD_MODNAME,
		.dev_groups = mdev_dev_groups,
	},
	.probe = mmdev_probe,
	.remove	= mmdev_remove,
	.get_available = mmdev_get_available,
};


static int mdev_dsched_pdrv_probe(void)
{
    int ret;

    INIT_LIST_HEAD(&mmdev_dev.instances);
    mutex_init(&mmdev_dev.list_lock);
    init_waitqueue_head(&mmdev_dev.wake_sched);
    mmdev_dev.stop = false;

    {
        struct timespec64 ts;
        struct tm tm;

        ktime_get_real_ts64(&ts);
        time64_to_tm(ts.tv_sec, 0, &tm);

        snprintf(mmdev_dev.log_path, sizeof(mmdev_dev.log_path),
                LOG_DIR "/%s_%04ld%02d%02d_%02d%02d%02d.txt",
                LOG_PREFIX,
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec);

        dev_info(&mmdev_dev.dev, "log file: %s\n", mmdev_dev.log_path);
    }


    mmdev_dev.sched_thread = kthread_run(sched_thread_fn, &mmdev_dev, "mdev_sched");
    if (IS_ERR(mmdev_dev.sched_thread)) {
        ret = PTR_ERR(mmdev_dev.sched_thread);
        return ret;
    }

    dev_info(&mmdev_dev.dev, "parent registered and scheduler thread started\n");
    return 0;
}

static int mdev_dsched_pdrv_remove(void)
{
    mmdev_dev.stop = true;
    if (mmdev_dev.sched_thread)
        kthread_stop(mmdev_dev.sched_thread);
    return 0;
}


static void mmdev_device_release(struct device *dev)
{
	dev_dbg(dev, "mtty: released\n");
	mdev_dsched_pdrv_remove();
}

static int __init mmdev_dev_init(void)
{
	int ret = 0;

	pr_info("mmdev_dev: %s\n", __func__);

	memset(&mmdev_dev, 0, sizeof(mmdev_dev));

	idr_init(&mmdev_dev.vd_idr);

	ret = alloc_chrdev_region(&mmdev_dev.vd_devt, 0, MINORMASK + 1,
				  MMDEV_NAME);

	if (ret < 0) {
		pr_err("Error: failed to register mmdev_dev, err:%d\n", ret);
		return ret;
	}

	cdev_init(&mmdev_dev.vd_cdev, &vd_fops);
	cdev_add(&mmdev_dev.vd_cdev, mmdev_dev.vd_devt, MINORMASK + 1);

	pr_info("major_number:%d\n", MAJOR(mmdev_dev.vd_devt));

	ret = mdev_register_driver(&mmdev_driver);
	if (ret)
		goto err_cdev;

	mmdev_dev.vd_class = class_create(MMDEV_CLASS_NAME);

	if (IS_ERR(mmdev_dev.vd_class)) {
		pr_err("Error: failed to register mmdev_dev class\n");
		ret = PTR_ERR(mmdev_dev.vd_class);
		goto err_driver;
	}

	mmdev_dev.dev.class = mmdev_dev.vd_class;
	mmdev_dev.dev.release = mmdev_device_release;
	dev_set_name(&mmdev_dev.dev, "%s", MMDEV_NAME);

	ret = device_register(&mmdev_dev.dev);
	if (ret)
		goto err_put;
	ret = mdev_dsched_pdrv_probe();
	if (ret)
		goto err_device;

	ret = mdev_register_parent(&mmdev_dev.parent, &mmdev_dev.dev,
				   &mmdev_driver, mmdev_mdev_types,
				   ARRAY_SIZE(mmdev_mdev_types));
	if (ret)
		goto err_device;
	return 0;

err_device:
	device_del(&mmdev_dev.dev);
err_put:
	put_device(&mmdev_dev.dev);
	class_destroy(mmdev_dev.vd_class);
err_driver:
	mdev_unregister_driver(&mmdev_driver);
err_cdev:
	cdev_del(&mmdev_dev.vd_cdev);
	unregister_chrdev_region(mmdev_dev.vd_devt, MINORMASK + 1);
	return ret;
}

static void __exit mmdev_dev_exit(void)
{
	mmdev_dev.dev.bus = NULL;
	mdev_unregister_parent(&mmdev_dev.parent);

	device_unregister(&mmdev_dev.dev);
	idr_destroy(&mmdev_dev.vd_idr);
	mdev_unregister_driver(&mmdev_driver);
	cdev_del(&mmdev_dev.vd_cdev);
	unregister_chrdev_region(mmdev_dev.vd_devt, MINORMASK + 1);
	class_destroy(mmdev_dev.vd_class);
	mmdev_dev.vd_class = NULL;
	pr_info("mmdev_dev: Unloaded!\n");
}

module_init(mmdev_dev_init)
module_exit(mmdev_dev_exit)

MODULE_LICENSE("GPL v2");
MODULE_INFO(supported, "Test driver that simulate serial port over PCI");
MODULE_VERSION(VERSION_STRING);
MODULE_AUTHOR(DRIVER_AUTHOR);
