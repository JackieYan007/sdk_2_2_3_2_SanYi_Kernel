#include <linux/crc16.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/of_gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include "spi_ipc.h"

/*---------------------------------------------------------------------
* MACRO
*--------------------------------------------------------------------*/
/*************************device num*********************************/
#define OFILMIPC_MAJOR                 220
#define OFILMIPC_NAME                  "spi_ch"

/*************************buf set*********************************/
#define MAX_CHNENEL                    32
#define MAX_XFER_SIZE      	           (16 * 1024)
#define IPC_MESSAGE_FLAG                0xa5
#define MAX_CMD_PER_CHNENEL               32

/************************** crc **********************************/
#define WIDTH    (8 * sizeof(unsigned short))
#define TOPBIT   (1 << (WIDTH - 1))
#define POLYNOMIAL			0x1021
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define CRC_TABLE_SIZE		256


/***********************buf point cal ****************************/
#define INDEX_TO_CMD_HEADER(bq, index) \
		(struct cmd_header *) \
		(bq->buffer_array + ((bq->index & (bq->item_nr - 1)) * bq->item_size))

#define DATA_ALIGN_4_CHECK(value) ((value&0x03)?0:1)


typedef enum {
	PACK_CONTINUE_NO_CH_PACKED = 0,
	PACK_CONTINUE_CH_PACKED,
	PACK_STOP_NO_CH_PACKED,
	PACK_STOP_CH_PACKED,
} PACK_ONE_CH_RET_VALUE;

/*---------------------------------------------------------------------
* STRUCT
*--------------------------------------------------------------------*/
struct buffer_queue {
	struct ch_info *ch;
	wait_queue_head_t wq;
    struct mutex lock;
	unsigned int read_index;
	unsigned int write_index;
	int item_nr; // must be like: 2,4,8,16,32,64,128,etc...
	int item_size;
	void *buffer_array; // pointer to buffer arrry, array size is item_nr * item_size
	/*
		buffer arrry item struct:
		item0:	 cmdX0 + cmd_len + cmd_data
		item1:	 cmdX1 + cmd_len + cmd_data
		......
		itemN:	 cmdXN + cmd_len + cmd_data
	*/
};

struct ch_info {
	int id; // channel id
	int  open_count;
    int  rx_missing;
	struct device *dev;
	struct mutex ch_mutex;

	struct buffer_queue tx;
	struct buffer_queue rx;
};

struct spi_ipc_info {
	struct device *dev;
	struct spi_device *spi;
	u32 speed_hz;

	int irq;
	int mcu_request_gpio; /* gpio level is: 0: mcu is not ready, 1: ready */
	int mcu_crc_error_gpio; /* gpio level: 0: mcu no crc error, 1:  crc error */

	unsigned long ch_enabled_bit_flag; // ex: if bit0 =1 means channel 0 is opened, 0 means closed.
	unsigned int ch_total; // actual number of channels
	struct ch_info chi[MAX_CHNENEL];
	unsigned char tx_buf[MAX_XFER_SIZE];
	unsigned char rx_buf[MAX_XFER_SIZE];
	int xfer_size; // xfer_size must be <= MAX_XFER_SIZE
	struct spi_message message;
	struct spi_transfer xfer;
    unsigned short crcTable[CRC_TABLE_SIZE];
	/* below is statistics info */
	unsigned int tx_counter; /* plus 1 when call spi transfer func: spi_sync */

	//unsigned int mcu_counter;
	unsigned short mcu_counter;
	unsigned int rx_good;
	unsigned int rx_crc_error; // soc endpoint crc check error times
	unsigned int rx_missing; // mcu pack message and waiting for soc transfer timeout times,

	unsigned int xfer_error; // spi transfer error times
	unsigned int xfer_good;
	unsigned int mcu_crc_error; // mcu endpoint crc check error times
};


struct msg_header {
	unsigned char ipc_msg_flag;
    unsigned char protocol_version;
    unsigned short counter;
    unsigned char reverse;
    unsigned char channel_num;
    unsigned short msg_len;
};

struct ch_header {
	unsigned char channel_id;
    unsigned char cmd_num;
	unsigned short channel_len; /* cmd_header + cmd_len */
};

struct cmd_header {
	unsigned char cmd_id;
    unsigned char reverse;
	unsigned short cmd_len;
};


/*---------------------------------------------------------------------
* Internal Variable
*--------------------------------------------------------------------*/
static struct spi_ipc_info *g_sii = NULL;
static struct class *spi_ipc_class = NULL;
static unsigned int dump_flag = 0;
static unsigned int dump_channel = 0;


/*---------------------------------------------------------------------
* Test configure
*--------------------------------------------------------------------*/
#define DUMP_CHANNEL_READ_CMD      0
#define DUMP_CHANNEL_WRITE_CMD     1
#define DUMP_IRQ_RECV              2
#define DUMP_IRQ_SEND              3
#define DUMP_RX_BUFFER             4
#define DUMP_TX_BUFFER             5

static const char* dump_str[] = {
	"channel_read",
	"channel_write",
	"irq_revc",
	"irq_send",
	"rx_buf",
	"tx_buf"
};


/******************************************************************************
** define
*******************************************************************************/
unsigned short ipc_pack_crc_cal(unsigned char *message, unsigned short nBytes);


/******************************************************************************
** Code
*******************************************************************************/
static ssize_t ipc_stat_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_ipc_info *sii = dev_get_drvdata(dev);
	int n = 0;
	int i;

	n += sprintf(buf, "global statistics\n");
	n += sprintf(buf + n, "    tx_counter:%u\n", sii->tx_counter);
	n += sprintf(buf + n, "    mcu_counter:%u\n", sii->mcu_counter);
	n += sprintf(buf + n, "    rx_good:%u\n", sii->rx_good);
	n += sprintf(buf + n, "    rx_crc_error:%u\n", sii->rx_crc_error);
	n += sprintf(buf + n, "    rx_missing:%u\n", sii->rx_missing);
	n += sprintf(buf + n, "    xfer_error:%u\n", sii->xfer_error);
	n += sprintf(buf + n, "    xfer_good:%u\n", sii->xfer_good);
	n += sprintf(buf + n, "    mcu_crc_error:%u\n", sii->mcu_crc_error);

	for (i = 0; i < sii->ch_total; i++) {
		n += sprintf(buf + n, "\nchannel%d %s: statistics\n", i,
				(sii->ch_enabled_bit_flag & (1 << i)) ? "enabled" : "disabled");
		n += sprintf(buf + n, "    tx.write_index:%u\n",
					sii->chi[i].tx.write_index);
		n += sprintf(buf + n, "    tx.read_index:%u\n",
					sii->chi[i].tx.read_index);
		n += sprintf(buf + n, "    rx.write_index:%u\n",
					sii->chi[i].rx.write_index);
		n += sprintf(buf + n, "    rx.read_index:%u\n",
					sii->chi[i].rx.read_index);
        n += sprintf(buf + n, "    rx missing:%u\n",
					sii->chi[i].rx_missing);
	}

	return n;
}
static DEVICE_ATTR_RO(ipc_stat);

static ssize_t dump_flag_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int n = 0;
	int i;

	n += sprintf(buf + n, "dump flag:0x%x\n", dump_flag);

	for (i = 0; i < ARRAY_SIZE(dump_str); i++) {
		n += sprintf(buf + n, "%16s:%u\n", dump_str[i], (dump_flag >> i) & 0x1);
	}

	return n;
}

static ssize_t dump_flag_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 16, &val);
	if (ret)
		return ret;

	dump_flag = val;

	return size;
}
static DEVICE_ATTR_RW(dump_flag);

static ssize_t dump_channel_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int n = 0;
	struct spi_ipc_info *sii = dev_get_drvdata(dev);

	n += sprintf(buf + n, "channel enabled:0x%lx\n", sii->ch_enabled_bit_flag);
	n += sprintf(buf + n, "dump channel   :0x%x\n", dump_channel);

	return n;
}

static ssize_t dump_channel_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct spi_ipc_info *sii = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 16, &val);
	if (ret)
		return ret;

	dump_channel = val;

	return size;
}
static DEVICE_ATTR_RW(dump_channel);

static void dump_msg_header(const char* notice, struct msg_header *msg_h)
{
	printk("%s: dump msg header ...\n", notice);
	printk("    ipc message flag:  0X%X\n", msg_h->ipc_msg_flag);
    printk("    protocol_version:  0X%X\n", msg_h->protocol_version);
    printk("    counter:           0X%X\n", msg_h->counter);
    printk("    channel_num:       0X%X\n", msg_h->channel_num);
    printk("    message length:    0X%X\n", msg_h->msg_len);
}

static void dump_ch_header(const char* notice, struct ch_header *ch_h)
{
	printk("%s: dump ch header ...     \n", notice);
	printk("    channel_id:        0X%X\n", ch_h->channel_id);
    printk("    cmd_num:           0X%X\n", ch_h->cmd_num);
	printk("    channel_len:       0X%X\n", ch_h->channel_len);
}

static void dump_cmd(const char* notice, int channel, void *cmd)
{
	int i;
	int n = 0;
    unsigned char temp[64];
    struct cmd_header *cmd_h = (struct cmd_header *)cmd;
    unsigned char *d = (unsigned char *)cmd + sizeof(struct cmd_header);

	printk("%s: dump channel %d: cmd...\n", notice, channel);
	printk("    cmd id:              0X%X\n", cmd_h->cmd_id);
	printk("    cmd len:             0X%X\n", cmd_h->cmd_len);

	for (i = 0; i < cmd_h->cmd_len; i++) {
		if ((i & 0x0f) == 0) {
            n = 0;
            if (0 == i){
                printk("0X%04X:", i);
            }
            else
            {
                printk("  %s\n", temp);
                printk("0X%04X:", i);
            }
        }

		n += sprintf(temp + n, "%02x ", *d++);
	}

	printk("  %s\n", temp);
}

static void dump_buffer(const char* notice, unsigned char *buffer, int len)
{
	int i;
	int n = 0;
    unsigned char temp[64];

	printk("dump %s:\n", notice);
	for (i = 0; i < len; i++) {
        if ((i & 0x0f) == 0) {
            n = 0;
            if (0 == i){
                printk("0X%04X:", i);
            }
            else
            {
                printk("  %s\n", temp);
                printk("0X%04X:", i);
            }
        }
		n += sprintf(temp + n, "%02x ", *buffer++);
	}

	printk("  %s\n", temp);
}



static int alloc_buffer_queue(struct buffer_queue *bq)
{
	if ((!bq->item_nr) || (!bq->item_size)) {
		printk(KERN_ERR "channel%d: item_nr:%d  item_size:%d\n",
				bq->ch->id, bq->item_nr, bq->item_size);
		return -EINVAL;
	}

	bq->read_index = 0;
	bq->write_index = 0;

	bq->buffer_array = kzalloc(bq->item_size * bq->item_nr, GFP_KERNEL);
	if (!bq->buffer_array) {
		printk(KERN_ERR "channel%d: can't alloc memory for buffer array\n", bq->ch->id);
		return -ENOMEM;
	}

	return 0;
}

static int alloc_channel_buffers(struct ch_info *ci)
{
	int ret;

	printk("alloc buffer for channel %d tx [size(%d) x nr(%d)], tx\n", ci->id,
			ci->tx.item_size, ci->tx.item_nr);
	ret = alloc_buffer_queue(&ci->tx);
	if (ret < 0) {
		printk(KERN_ERR "channel%d: alloc tx buffer queue error.\n", ci->id);
		return ret;
	}

	printk("alloc buffer for channel %d rx [size(%d) x nr(%d)], rx\n", ci->id,
			ci->rx.item_size, ci->rx.item_nr);
	ret = alloc_buffer_queue(&ci->rx);
	if (ret < 0) {
		printk(KERN_ERR "channel%d: alloc rx buffer queue error.\n", ci->id);
		return ret;
	}

	return 0;
}

static void free_channel_buffers(struct ch_info *ci)
{
	printk("free buffer for channel %d tx [size(%d) x nr(%d)], tx\n", ci->id,
			ci->tx.item_size, ci->tx.item_nr);
	if (ci->tx.buffer_array) {
		kfree(ci->tx.buffer_array);
		ci->tx.buffer_array = NULL;
	}

	printk("free buffer for channel %d rx [size(%d) x nr(%d)], tx\n", ci->id,
			ci->rx.item_size, ci->rx.item_nr);
	if (ci->rx.buffer_array) {
		kfree(ci->rx.buffer_array);
		ci->rx.buffer_array = NULL;
	}
}

static PACK_ONE_CH_RET_VALUE pack_one_channel_tx_msg(struct spi_ipc_info *sii,
	struct ch_info *ci, int *filled_size)
{
	struct ch_header *ch_h;
	struct cmd_header *cmd_h;
	struct buffer_queue *bq;
	int total_size;
	unsigned char cmds[MAX_CMD_PER_CHNENEL];
	int nr_cmd = 0;
	int i;
	PACK_ONE_CH_RET_VALUE ret;



	if (!((1 << ci->id) & sii->ch_enabled_bit_flag))
		return PACK_CONTINUE_NO_CH_PACKED; /* channel is not enabled, so skip */


	bq = &sii->chi[ci->id].tx;
	if ((bq->write_index - bq->read_index) == 0) {
		/* channel has no cmd to send */
		return PACK_CONTINUE_NO_CH_PACKED;
	} else if ((bq->write_index - bq->read_index) < 0) {
		printk(KERN_ERR "ch:%d write_index:%u read_index:%u", i,
				bq->write_index, bq->read_index);
		BUG();
	}

	/* make sure tx buffer free space can store userdata header and one cmd */
	cmd_h = INDEX_TO_CMD_HEADER(bq, read_index);
	total_size = *filled_size + sizeof(struct ch_header)
					+ sizeof(struct cmd_header)
					+ cmd_h->cmd_len
					+ 2 /* 16 bit crc */;

	if (total_size > sii->xfer_size)
		return PACK_STOP_NO_CH_PACKED; /* no enough free space, so stop packing */

    memset(cmds, 0x00, MAX_CMD_PER_CHNENEL);

    /* channel header */
	ch_h = (struct ch_header *)(sii->tx_buf + *filled_size);
	ch_h->channel_id = ci->id;
	ch_h->channel_len= 0;
	ch_h->cmd_num = 0;


	*filled_size += sizeof(struct ch_header);
	ret = PACK_CONTINUE_CH_PACKED;
	while (1) {
		if (*filled_size + sizeof(struct cmd_header) + cmd_h->cmd_len
				+ 2 /* 16 bit crc */ > sii->xfer_size) {
			ret = PACK_STOP_CH_PACKED; /* no enough free space, so stop packing */
			goto out;
		}

		if (nr_cmd >= ARRAY_SIZE(cmds)) {
			ret = PACK_CONTINUE_CH_PACKED;
			goto out;
		}

		for (i = 0; i < nr_cmd; i++) {
			if (cmd_h->cmd_id == cmds[i]) { /* do not pack the same cmd at a time */
				ret = PACK_CONTINUE_CH_PACKED;
				goto out;
			}
		}
		cmds[nr_cmd++] = cmd_h->cmd_id;

		ch_h->channel_len += sizeof(struct cmd_header) + cmd_h->cmd_len;
		ch_h->cmd_num++;

		memcpy(sii->tx_buf + *filled_size, cmd_h,
				sizeof(struct cmd_header) + cmd_h->cmd_len);
		*filled_size += sizeof(struct cmd_header) + cmd_h->cmd_len;

		bq->read_index++;
		if (0 == (bq->write_index - bq->read_index)) {
			ret = PACK_CONTINUE_CH_PACKED; /* no cmd to send, so return */
			goto out;
		}

		cmd_h = INDEX_TO_CMD_HEADER(bq, read_index);
	}

out:
    wake_up_interruptible(&ci->tx.wq);
	return ret;
}

static void pack_channels_tx_msg(struct spi_ipc_info *sii)
{
	static int first_pack_channel = 0;
	int i;
	struct msg_header *msg_h;
	unsigned int filled_size = 0;
	int next_channel;
	PACK_ONE_CH_RET_VALUE pocrv;
	unsigned short *pcrc;


	msg_h = (struct msg_header *)(sii->tx_buf);
	msg_h->ipc_msg_flag = IPC_MESSAGE_FLAG;
    msg_h->counter = sii->tx_counter++; /* update tx counter */
	msg_h->protocol_version = 1;
	msg_h->msg_len = 0;
	msg_h->channel_num = 0;
	filled_size = sizeof(struct msg_header);

#if 0
	for (i = first_pack_channel; i < sii->ch_total; i++) {
		next_channel = i;

		pocrv = pack_one_channel_tx_msg(sii, &sii->chi[i], &filled_size);
		if (PACK_CONTINUE_NO_CH_PACKED == pocrv) { /* channel is not enabled or no cmd to send */
			continue;
		} else if (PACK_CONTINUE_CH_PACKED == pocrv) {
			mh->channel_num++;
		} else if (PACK_STOP_NO_CH_PACKED == pocrv) {
			goto out;
		} else { /* pocrv == PACK_STOP_CH_PACKED */
			mh->channel_num++;
			goto out;
		}
	}

	for (i = 0; i < first_pack_channel; i++) {
		next_channel = i;

		pocrv = pack_one_channel_tx_msg(sii, &sii->chi[i], &filled_size);
		if (PACK_CONTINUE_NO_CH_PACKED == pocrv) { /* channel is not enabled or no cmd to send */
			continue;
		} else if (PACK_CONTINUE_CH_PACKED == pocrv) {
			mh->channel_num++;
		} else if (PACK_STOP_NO_CH_PACKED == pocrv) {
			goto out;
		} else { /* pocrv == PACK_STOP_CH_PACKED */
			mh->channel_num++;
			goto out;
		}
	}
#else
    for (i = 0; i < sii->ch_total; i++) {
        next_channel = i;

        pocrv = pack_one_channel_tx_msg(sii, &sii->chi[i], &filled_size);
        if (PACK_CONTINUE_NO_CH_PACKED == pocrv) { /* channel is not enabled or no cmd to send */
            continue;
        } else if (PACK_CONTINUE_CH_PACKED == pocrv) {
            msg_h->channel_num++;
        } else if (PACK_STOP_NO_CH_PACKED == pocrv) {
            goto out;
        } else { /* pocrv == PACK_STOP_CH_PACKED */
            msg_h->channel_num++;
            goto out;
        }
    }
#endif
out:
	msg_h->msg_len = filled_size - sizeof(struct msg_header);
	first_pack_channel = next_channel;

    pcrc = (unsigned short*)(sii->tx_buf + sii->xfer_size - 2);
   *pcrc = ipc_pack_crc_cal(sii->tx_buf, sizeof(struct msg_header) + msg_h->msg_len);
}

void ipc_pack_crc_init(void)
{
    unsigned char   bit;
    unsigned short            dividend;
    unsigned short 		      remainder;
    struct spi_ipc_info *sii = g_sii;


    for (dividend = 0; dividend < CRC_TABLE_SIZE; ++dividend)
    {
        remainder = dividend << (WIDTH - 8);

        for (bit = 8; bit > 0; --bit)
        {
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
        sii->crcTable[dividend] = remainder;
    }
}   /* ipc_pack_crc_init() */

unsigned short ipc_pack_crc_cal(unsigned char *message, unsigned short nBytes)
{
    unsigned char  data;
    unsigned short		byte;
    unsigned short check_remainder = INITIAL_REMAINDER;
    struct spi_ipc_info *sii = g_sii;

    for (byte = 0; byte < nBytes; ++byte)
    {
        data = (message[byte]) ^ (check_remainder >> (WIDTH - 8));
  		check_remainder = sii->crcTable[data] ^ (check_remainder << 8);
    }

    return ((check_remainder) ^ FINAL_XOR_VALUE);
}

static irqreturn_t spi_ipc_thread_irq_handler(int irq, void *data)
{
    struct spi_ipc_info *sii = data;
    struct ch_info *ci;
    struct buffer_queue *bq;

	struct msg_header *msg_h;
	struct ch_header *ch_h;
    struct cmd_header *cmd_h;

    struct spi_message message ;
	struct spi_transfer xfer;
    unsigned char i, j;
	unsigned int n;
	int cousumed_size = 0;
    int tmp_cousumed_size;
	int ret;
	unsigned short *pcrc;
	unsigned short crc;


	/* confirm mcu is really ready */
	if (unlikely(0 == gpio_get_value(sii->mcu_request_gpio))) {
		printk(KERN_WARNING "mcu not ready\n");
		return IRQ_HANDLED;
	}

    /* build data packet */
#if 0
	/* confirm mcu crc check error or not */
	if (unlikely(1 == gpio_get_value(sii->mcu_crc_error_gpio))) {
		sii->mcu_crc_error++;
		printk(KERN_ERR "mcu crc error:%u\n", sii->mcu_crc_error);
	} else {
		/* collect channel msg to send when mcu no crc error */
		pack_channels_tx_msg(sii);
	}
#else
	pack_channels_tx_msg(sii);
#endif
    memset((unsigned char*)&xfer, 0x00, sizeof(xfer));
	/* spi transfer */
	xfer.tx_buf = sii->tx_buf;
	xfer.rx_buf = sii->rx_buf;
	xfer.len = sii->xfer_size;
    xfer.bits_per_word = 8;
    xfer.rx_nbits = SPI_NBITS_SINGLE;
    xfer.tx_nbits = SPI_NBITS_SINGLE;
    xfer.speed_hz = 0;

	spi_message_init(&message);
	spi_message_add_tail(&xfer, &message);



    /************ check tx message ****************/
    /* check tx buffer */
	if (unlikely((1 << DUMP_TX_BUFFER) & dump_flag))
		dump_buffer(dump_str[DUMP_TX_BUFFER], sii->tx_buf, sii->xfer_size);

    /* check tx message */
	if (unlikely((1 << DUMP_IRQ_SEND) & dump_flag)) {
		int temp_cousumed_size = sizeof(struct msg_header);

        /* check tx header */
        msg_h = (struct msg_header *)(sii->tx_buf);
		dump_msg_header(dump_str[DUMP_IRQ_SEND], msg_h);

        /* check tx channel */
		for (i = 0; i < msg_h->channel_num; i++) {
            ch_h = (struct ch_header *)(sii->tx_buf + temp_cousumed_size);
			temp_cousumed_size += sizeof(struct ch_header);

			if (((1 << DUMP_IRQ_SEND) & dump_flag)
					&& ((1 << ch_h->channel_id) & dump_channel)) {
				dump_ch_header(dump_str[DUMP_IRQ_SEND], ch_h);
			}

			for (j = 0; j < ch_h->cmd_num; j++) {
                cmd_h = (struct cmd_header *)(sii->tx_buf + temp_cousumed_size);
				temp_cousumed_size += sizeof(struct cmd_header) + cmd_h->cmd_len;


				if (((1 << DUMP_IRQ_SEND) & dump_flag)
						&& ((1 << ch_h->channel_id) & dump_channel)) {
						dump_cmd(dump_str[DUMP_IRQ_SEND], ch_h->channel_id, cmd_h);
				}
			}
		}

		//printk("crc: %02x %02x\n", sii->tx_buf[sii->xfer_size - 2],	sii->tx_buf[sii->xfer_size - 1]);
	}



    /************ ipc communiction ****************/
	if (unlikely((ret = spi_sync(sii->spi, &message)) < 0)) {
		sii->xfer_error++;
		printk(KERN_ERR "spi io error:%u ret:%d\n", sii->xfer_error, ret);
        printk(KERN_ERR "spi bits:%d, rx: %d, tx: %d\n", xfer.bits_per_word, xfer.rx_nbits, xfer.tx_nbits);
		return IRQ_HANDLED;
	}
//    memset(sii->tx_buf, 0x00, sii->xfer_size);  /* clear tx buf */
	sii->xfer_good++;



	/* flag number check */
	msg_h = (struct msg_header *)(sii->rx_buf);
	cousumed_size = sizeof(struct msg_header);
	if (unlikely((IPC_MESSAGE_FLAG != msg_h->ipc_msg_flag)
		|| (sizeof(struct msg_header) + msg_h->msg_len + 2 /* 16bit crc */
		> sii->xfer_size))) {
		sii->rx_crc_error++;
		printk(KERN_ERR "flag number or size error. flag:0x%x, len:%d\n",
			msg_h->ipc_msg_flag, msg_h->msg_len);
		//dump_buffer("rx buf size error", sii->rx_buf, sii->xfer_size);
		return IRQ_HANDLED;
	}



	/* do crc check */
	pcrc = (unsigned short*)(sii->rx_buf + sii->xfer_size - 2);
    crc = ipc_pack_crc_cal(sii->rx_buf, sizeof(struct msg_header) + msg_h->msg_len);
	if (*pcrc != crc) {
		sii->rx_crc_error++;
		printk(KERN_ERR "crc error. mcu:%u  calc crc:%u\n", *pcrc, crc);
		//dump_buffer("rx buf crc error", sii->rx_buf, sii->xfer_size);
		return IRQ_HANDLED;
	}


	sii->rx_good++;

	/* missing mcu packet check */
	//if (unlikely(msg_h->counter != sii->mcu_counter + 1)) {
	if (unlikely(msg_h->counter !=(unsigned short)(sii->mcu_counter + 1))) {
		n = msg_h->counter - sii->mcu_counter - 1;
		sii->rx_missing += n;

		printk(KERN_ERR "missing msgs from mcu total:%u. current missing:%u\n",
				sii->rx_missing, n);
	}
	sii->mcu_counter = msg_h->counter;

	if (unlikely((1 << DUMP_RX_BUFFER) & dump_flag))
		dump_buffer(dump_str[DUMP_RX_BUFFER], sii->rx_buf, sii->xfer_size);

	if ((1 << DUMP_IRQ_RECV) & dump_flag)
		dump_msg_header(dump_str[DUMP_IRQ_RECV], msg_h);


	/* parse rx msg */
	for (i = 0; i < msg_h->channel_num; i++) {
        ch_h = (struct ch_header*)(sii->rx_buf + cousumed_size);
		cousumed_size += sizeof(struct ch_header);
		if (((1 << DUMP_IRQ_RECV) & dump_flag)
				&& ((1 << ch_h->channel_id) & dump_channel))
			dump_ch_header(dump_str[DUMP_IRQ_RECV], ch_h);

		if (unlikely(ch_h->channel_id >= sii->ch_total)) {
			printk(KERN_ERR "invalid channel id:%d\n", ch_h->channel_id);
            cousumed_size += ch_h->channel_len;
			continue;
		}

		if (!((1 << ch_h->channel_id) & sii->ch_enabled_bit_flag)) {
			/* printk(KERN_ERR "recv chan %d msg from mcu: which is not enabled.\n",ch_h->channel_id); */
            cousumed_size += ch_h->channel_len;
			continue;
		}

		ci = &sii->chi[ch_h->channel_id];
		bq = &ci->rx;


		/* copy each cmd to each rx buffer queue */
        tmp_cousumed_size = cousumed_size;
		for (j = 0; j < ch_h->cmd_num; j++) {
			if (unlikely(bq->write_index - bq->read_index >= bq->item_nr)) {
				printk(KERN_ERR "channel %d no space to store data from mcu\n", ci->id);
                ci->rx_missing++;
                cousumed_size = tmp_cousumed_size + ch_h->channel_len;
				break;
			}

            cmd_h = (struct cmd_header *)(sii->rx_buf + cousumed_size);

			if (sizeof(struct cmd_header) + cmd_h->cmd_len > bq->item_size) {
                cousumed_size += sizeof(struct cmd_header) + cmd_h->cmd_len;
				printk(KERN_ERR "channel:%u , cmd_id:%u: cmd len:%lu > item_size:%d\n", ch_h->channel_id, cmd_h->cmd_id,
					sizeof(struct cmd_header) + cmd_h->cmd_len, bq->item_size);
				continue;
			}


			memcpy(INDEX_TO_CMD_HEADER(bq, write_index),
					sii->rx_buf + cousumed_size,
					sizeof(struct cmd_header) + cmd_h->cmd_len);
            bq->write_index++;


			cousumed_size += sizeof(struct cmd_header) + cmd_h->cmd_len;
			if (((1 << DUMP_IRQ_RECV) & dump_flag) && ((1 << ch_h->channel_id) & dump_channel))
				dump_cmd(dump_str[DUMP_IRQ_RECV], ch_h->channel_id, cmd_h);

		}
        wake_up_interruptible(&ci->rx.wq);
	}
	return IRQ_HANDLED;
}

static int channel_open(struct inode *inode, struct file *filp)
{
	struct spi_ipc_info *sii = g_sii;
	struct ch_info *ci;
	int ch = iminor(file_inode(filp));

    printk("open ch:%d\n", ch);
	ci = &sii->chi[ch];

	mutex_lock(&ci->ch_mutex);

	if ((ci->open_count)&&(0 != ch)) {
		printk(KERN_ERR "channel %d open_count: %d\n", ch, ci->open_count);
		mutex_unlock(&ci->ch_mutex);
		return -EBUSY;
	}
	ci->open_count++;

	mutex_unlock(&ci->ch_mutex);
	return 0;
}

static ssize_t channel_read(struct file *filp, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct spi_ipc_info *sii = g_sii;
	int chid = iminor(file_inode(filp));
	struct ch_info *ci = &sii->chi[chid];
	struct buffer_queue *bq = &ci->rx;
	struct cmd_header *cmd_h;
	int ret = 0;

    if (!((1 << chid) & sii->ch_enabled_bit_flag)){
        printk(KERN_ERR "channel%d read: error. ioctl has not been called to enable ch%d\n", 
            chid, chid);
        return -EPERM;
    }

	if (count < bq->item_size) {
		printk(KERN_ERR "channel%d read: error. count:%d < item_size:%d\n", chid, 
				count, bq->item_size);
		return -EINVAL;
	}

	mutex_lock(&ci->rx.lock);

	if (unlikely(bq->write_index < bq->read_index)) {
		printk(KERN_ERR "channel%d read: write index:%u < read index:%u\n",
				chid, bq->write_index, bq->read_index);
		BUG();
	}

	/* if rx buffer is empty, block here, waiting for receive cmds from mcu */
	while (bq->read_index == bq->write_index) {
		ret = wait_event_interruptible(bq->wq, bq->write_index > bq->read_index);
		if (ret < 0) {
			printk(KERN_ERR "channel%d read: error. ret:%d\n", chid, ret);
			goto out;
		}
	}

	cmd_h = INDEX_TO_CMD_HEADER(bq, read_index);
	if (copy_to_user(buf, cmd_h, sizeof(struct cmd_header) + cmd_h->cmd_len)) {
		ret = -EFAULT;
		goto out;
	}

    ret = cmd_h->cmd_len + sizeof(struct cmd_header);
	bq->read_index++;

	if (((1 << DUMP_CHANNEL_READ_CMD) & dump_flag) && ((1 << ci->id) & dump_channel))
		dump_cmd(dump_str[DUMP_CHANNEL_READ_CMD], ci->id, cmd_h);

out:
	mutex_unlock(&ci->rx.lock);
	return ret;
}

static ssize_t channel_write(struct file *filp, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct spi_ipc_info *sii = g_sii;
	int chid = iminor(file_inode(filp));
	struct ch_info *ci = &sii->chi[chid];
	struct buffer_queue *bq = &ci->tx;
	struct cmd_header *cmd_h;
	size_t len;
	int ret = 0;

    if (!((1 << chid) & sii->ch_enabled_bit_flag)){
        printk(KERN_ERR "channel%d write: error. ioctl has not been called to enable ch%d\n",
            chid, chid);
        return -EPERM;
    }

	if (count > bq->item_size) {
		printk(KERN_ERR "channel%d write: error. count:%d > item_size:%d\n", chid,
				count, bq->item_size);
		return -EINVAL;
	}

	mutex_lock(&ci->tx.lock);

	if (unlikely((bq->write_index < bq->read_index) ||
			(bq->write_index - bq->read_index > bq->item_nr))) {
		printk(KERN_ERR "channel%d write: write index:%u  read index:%u\n",
				chid, bq->write_index, bq->read_index);
		BUG();
	}

	/* if tx buffer is full, block here, waiting tx buf cmds send to mcu */
	while (bq->write_index - bq->read_index == bq->item_nr) {
		ret = wait_event_interruptible(bq->wq,
				bq->write_index - bq->read_index < bq->item_nr);
		if (ret < 0) {
			printk(KERN_ERR "channel%d write: error ret:%d\n", chid, ret);
			goto out;
		}
	}

	cmd_h = INDEX_TO_CMD_HEADER(bq, write_index);
	if (copy_from_user(cmd_h, buf, count)) {
		ret = -EFAULT;
		goto out;
	}

	len = sizeof(struct cmd_header) + cmd_h->cmd_len;
	if (len != count) {
		printk(KERN_ERR "channel%d, cmd %d write: error. cmd item len:%d != count:%d\n",
				chid, cmd_h->cmd_id, len, count);
		ret = -EINVAL;
		goto out;
	}

	if (DATA_ALIGN_4_CHECK(cmd_h->cmd_len)   == 0) {
		printk(KERN_ERR "channel %d write data no 4 byte align, len:%d\n",
            chid, cmd_h->cmd_len);
        ret =  -EINVAL;
		goto out;
	}

    ret = cmd_h->cmd_len + sizeof(struct cmd_header);
	bq->write_index++;
    //printk("write ch id %d, cmd %d, len %d\n", ci->id, cmd_h->cmd_id , cmd_h->cmd_len, ci->id);

	if (((1 << DUMP_CHANNEL_WRITE_CMD) & dump_flag) && ((1 << ci->id) & dump_channel))
		dump_cmd(dump_str[DUMP_CHANNEL_WRITE_CMD], ci->id, cmd_h);

out:
	mutex_unlock(&ci->tx.lock);
	return ret;
}

static long channel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct spi_ipc_info *sii = g_sii;
	struct ch_info *ci;
	int ch = iminor(file_inode(filp));
	void __user *argp = (void __user *)arg;
	struct ipc_cfg ic;
	int ret = 0;

	printk("%s: ch%d cmd 0x%.8x (%d), arg 0x%lx\n", __func__, ch, cmd, _IOC_NR(cmd), arg);

	if (_IOC_TYPE(cmd) != OFILM_IPC_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) >= OFILM_IPC_IOC_MAXNR)
		return -ENOTTY;

	ci = &sii->chi[ch];

	switch (cmd) {
	case OFILM_IPC_IOC_SETCFG:
        if (((1 << ci->id) & sii->ch_enabled_bit_flag)){
            return ret;
        }

		if (copy_from_user(&ic, argp, sizeof(ic))){
			return -EFAULT;
		}
        if ((DATA_ALIGN_4_CHECK(ic.tx_buf_size)   == 0)||
            (DATA_ALIGN_4_CHECK(ic.rx_buf_size)   == 0)){
            printk(KERN_ERR "ch%d buf size is not 4 digits. tx buf size: 0X%X; rx buf size:0X%X.\n",
                  ch, ic.tx_buf_size, ic.rx_buf_size);
            return -EINVAL;
        }

		if (ic.tx_buf_nr > MAX_BUF_NR || ic.rx_buf_nr > MAX_BUF_NR
			|| ic.tx_buf_size > MAX_BUF_SIZE || ic.rx_buf_size > MAX_BUF_SIZE
			|| ic.tx_buf_nr < MIN_BUF_NR || ic.rx_buf_nr < MIN_BUF_NR) {
			printk(KERN_ERR "ch%d tx:[%dx%d] rx:[%dx%d] buf nr or size error.\n",
				ch, ic.tx_buf_nr, ic.tx_buf_size, ic.rx_buf_nr, ic.rx_buf_size);
			return -EINVAL;
		}

		if ((ic.tx_buf_nr & (ic.tx_buf_nr -1))
			|| (ic.rx_buf_nr & (ic.rx_buf_nr -1))) {
			printk(KERN_ERR "error: ch%d tx_buf_nr:%d or rx_buf_nr:%d is not a power of 2.\n",
					ch, ic.tx_buf_nr, ic.rx_buf_nr);
			return -EINVAL;
		}

		mutex_lock(&ci->ch_mutex);

		ci->tx.item_size = ic.tx_buf_size;
		ci->tx.item_nr = ic.tx_buf_nr;
		ci->rx.item_size = ic.rx_buf_size;
		ci->rx.item_nr = ic.rx_buf_nr;
        ci->rx_missing = 0;

		ret = alloc_channel_buffers(ci);
		if (!ret) {
			set_bit(ch, &sii->ch_enabled_bit_flag);
		}

		mutex_unlock(&ci->ch_mutex);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	if (ret) {
		printk(KERN_ERR "error in ioctl call:ch%d cmd 0x%.8x (%d), ret %d\n",
			ch, cmd, _IOC_NR(cmd), ret);
	}

	return ret;
}

static int channel_release(struct inode *inode, struct file *filp)
{
	struct spi_ipc_info *sii = g_sii;
	struct ch_info *ci;
	int ch = iminor(file_inode(filp));

	ci = &sii->chi[ch];

	mutex_lock(&ci->ch_mutex);

	clear_bit(ch, &sii->ch_enabled_bit_flag);
	free_channel_buffers(ci);
	ci->open_count = 0;

	mutex_unlock(&ci->ch_mutex);

	return 0;
}

static const struct file_operations spi_ipc_fops = {
	.owner = THIS_MODULE,
	.open = channel_open,
	.llseek = no_llseek,
	.read = channel_read,
	.write = channel_write,
	.unlocked_ioctl = channel_ioctl,
	.release = channel_release,
};

static int spi_ipc_probe(struct spi_device *spi)
{
	struct spi_ipc_info *sii = NULL;
	struct device_node *np = spi->dev.of_node;
	struct ch_info *ci;
	int ret;
	int i;

	/* Allocate driver data */
	sii = kzalloc(sizeof(*sii), GFP_KERNEL);
	if (!sii)
		return -ENOMEM;


    g_sii = sii;
    ipc_pack_crc_init();

	sii->spi = spi;
	sii->dev = &spi->dev;

	sii->mcu_request_gpio = of_get_named_gpio(np, "mcu-request-gpios", 0);
	if (!gpio_is_valid(sii->mcu_request_gpio)) {
		dev_err(sii->dev, "No mcu-request-gpios property\n");
		ret = sii->mcu_request_gpio;
		goto error;
	}

#if 0
	sii->mcu_crc_error_gpio = of_get_named_gpio(np, "mcu-crc-error-gpios", 0);
	if (!gpio_is_valid(sii->mcu_crc_error_gpio)) {
		dev_err(sii->dev, "No mcu-crc-error-gpios property\n");
		ret = sii->mcu_crc_error_gpio;
		goto error;
	}
#endif

	ret = devm_gpio_request_one(sii->dev, sii->mcu_request_gpio,
			GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED, "mcu-request-gpios");
	if (ret) {
		dev_err(sii->dev, "Can't request mcu-request-gpios GPIO: %d\n", ret);
		goto error;
	}

#if 0
	ret = devm_gpio_request_one(sii->dev, sii->mcu_crc_error_gpio,
			GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED, "mcu-crc-error-gpios");
	if (ret) {
		dev_err(sii->dev, "Can't request mcu-crc-error-gpios GPIO: %d\n", ret);
		goto error;
	}
#endif

	ret = of_property_read_u32(np, "xfer-size", &sii->xfer_size);
	if (ret) {
		dev_err(sii->dev, "could not find xfer-size property. ret:%d\n", ret);
		goto error;
	}

	if (sii->xfer_size > MAX_XFER_SIZE)
		sii->xfer_size = MAX_XFER_SIZE;

	ret = of_property_read_u32(np, "nr-channels", &sii->ch_total);
	if (ret) {
		dev_err(sii->dev, "could not find nr-channels property. ret:%d", ret);
		goto error;
	}

	if (sii->ch_total > MAX_CHNENEL)
		sii->ch_total = MAX_CHNENEL;

	for (i = 0; i < sii->ch_total; i++) {
		ci = &sii->chi[i];
		ci->id = i;
		mutex_init(&ci->ch_mutex);
        mutex_init(&ci->tx.lock);
        mutex_init(&ci->rx.lock);

		init_waitqueue_head(&ci->tx.wq);
		init_waitqueue_head(&ci->rx.wq);
		ci->tx.ch = ci;
		ci->rx.ch = ci;

		ci->dev= device_create(spi_ipc_class, &spi->dev, MKDEV(OFILMIPC_MAJOR, i),
				    sii, "spi_ch%d", i);
		if (IS_ERR(ci->dev)) {
			dev_err(sii->dev, "spi ipc: failed to create device. i:%d\n", i);
			return PTR_ERR(ci->dev);
		}
	}

	sii->speed_hz = spi->max_speed_hz;
    sii->ch_enabled_bit_flag = 0;

	spi_set_drvdata(spi, sii);

	sii->irq = gpio_to_irq(sii->mcu_request_gpio);
	ret = devm_request_threaded_irq(sii->dev, sii->irq,
				     NULL, spi_ipc_thread_irq_handler,
				     IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				     "ipc_request_gpio", sii);
	if (ret < 0) {
		dev_err(sii->dev, "Can't allocate irq %d\n", sii->irq);
		goto error;
	}

	device_create_file(&spi->dev, &dev_attr_ipc_stat);
	device_create_file(&spi->dev, &dev_attr_dump_flag);
	device_create_file(&spi->dev, &dev_attr_dump_channel);

	dev_info(sii->dev, "spi ipc: channel:%u  xfer size:%d  speed: %u\n",
			sii->ch_total, sii->xfer_size, sii->speed_hz);

	return 0;

error:
    g_sii = NULL;
	kfree(sii);
	return ret;
}

static int spi_ipc_remove(struct spi_device *spi)
{
	int i;
	struct spi_ipc_info *sii = spi_get_drvdata(spi);

	device_remove_file(&spi->dev, &dev_attr_ipc_stat);
	device_remove_file(&spi->dev, &dev_attr_dump_flag);
	device_remove_file(&spi->dev, &dev_attr_dump_channel);

	for (i = 0; i < sii->ch_total; i++)
		device_destroy(spi_ipc_class, MKDEV(OFILMIPC_MAJOR, i));

	kfree(sii);
	g_sii = NULL;

	return 0;
}

/*
	spi_ipc: spi@0 {
		compatible = "ofilm,spi-ipc";
		reg = <0>;
		spi-max-frequency = <20000000>;
		spi-cpha;
		spi-cpol;
		spi-cs-high;
 		spi-lsb-first;

		nr-channels = <16>;
		xfer-size = <1024>;
 		mcu-request-gpios = <&gpio7 11 GPIO_ACTIVE_LOW>;
 		mcu-crc-error-gpios = <&gpio2 6 GPIO_ACTIVE_LOW>;
	};
*/

static const struct of_device_id spi_ipc_dt_ids[] = {
	{ .compatible = "ofilm,spi-ipc" },
	{},
};
MODULE_DEVICE_TABLE(of, spi_ipc_dt_ids);

static struct spi_driver spi_ipc_driver = {
	.driver = {
		.name =		"spi_ipc",
		.of_match_table = of_match_ptr(spi_ipc_dt_ids),
	},
	.probe = spi_ipc_probe,
	.remove = spi_ipc_remove,
};

static int __init spi_ipc_init(void)
{
	int status;

	printk(KERN_INFO "ofilm ipc init\n");

	status = register_chrdev(OFILMIPC_MAJOR, "spi_ipc", &spi_ipc_fops);
	if (status < 0)
		return status;
	printk(KERN_INFO "register 111111\n");
	spi_ipc_class = class_create(THIS_MODULE, "spi_ipc");
	if (IS_ERR(spi_ipc_class)) {
		unregister_chrdev(OFILMIPC_MAJOR, "spi_ipc");
		return PTR_ERR(spi_ipc_class);
	}
	printk(KERN_INFO "class create 222222222\n");
	status = spi_register_driver(&spi_ipc_driver);
	if (status < 0) {
		class_destroy(spi_ipc_class);
		unregister_chrdev(OFILMIPC_MAJOR, "spi_ipc");
		return status;
	}
	printk(KERN_INFO "register ok 3333333\n");
	return status;
}

static void __exit spi_ipc_exit(void)
{
	spi_unregister_driver(&spi_ipc_driver);
	class_destroy(spi_ipc_class);
	unregister_chrdev(OFILMIPC_MAJOR, "spi_ipc");
}

module_init(spi_ipc_init);
module_exit(spi_ipc_exit);

MODULE_AUTHOR("TuJian <tu.jian@ofilm.com>");
MODULE_DESCRIPTION("intel processor(MCU < --- > SOC) communition driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(OFILMIPC_MAJOR);
