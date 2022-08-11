#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timex.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/utsname.h>

//#include "ion/ion.h"           // libion header
//#include "linux/ion_4.19.h" // struct ion_heap_data
#include "ion.h"

#define ION_BUFFER_LEN	4096
#define MAX_HEAP_COUNT	10

struct ion_buf_info {
	int ionfd;
	int buffd;
	unsigned int heap_type;
	unsigned int flag_type;
	unsigned long heap_size;
	unsigned long buflen;
	unsigned char *buffer;
};

struct ion_share_info {
	int buffd;
	unsigned int buflen;
	unsigned int flag_type;
	unsigned int heap_type;
	int rwopt;
};

struct picp_cfg {
	char rname[32];
	char wname[32];
	u_int32_t timeout;
	u_int32_t rwopt;
	u_int64_t local_head;
	u_int64_t remote_head;
	u_int32_t head_physize;
};

struct ioctl_info {
	int size;
	int rwopt;
	int thread;
	int single;
	int sniffer;
	int quit;
};

/* local record */
struct picp_record {
	u_int32_t rtotal;    /* read total */
	u_int32_t rerrors;   /* read err total */
	u_int32_t rlinkerr;  /* read link err */
	u_int32_t rdmaerr;   /* read dma err */
	u_int32_t rtimeout;  /* read timeout */
	u_int64_t rbytes;

	u_int32_t wtotal;    /* write total */
	u_int32_t werrors;   /* write err total */
	u_int32_t wlinkerr;  /* write link err */
	u_int32_t wdmaerr;   /* write dma err */
	u_int32_t wtimeout;  /* write timeout */
	u_int64_t wbytes;

	u_int32_t ltssm_count;
	u_int32_t type;
	u_int32_t state;
};

#define PICP_IOC_MAGIC    'p'
#define PICP_IOC_QUERY    _IOR(PICP_IOC_MAGIC, 1, struct picp_record)
#define PICP_IOC_SNIFFER  _IOR(PICP_IOC_MAGIC, 2, int)               
#define PICP_IOC_INIT     _IOW(PICP_IOC_MAGIC, 3, struct picp_cfg)
#define PICP_IOC_EXIT     _IOW(PICP_IOC_MAGIC, 4, struct picp_cfg)
#define PICP_IOC_RWTEST   _IOW(PICP_IOC_MAGIC, 5, struct ioctl_info) 
#define PICP_IOC_SIONFD   _IOW(PICP_IOC_MAGIC, 6, struct ion_share_info)
#define PICP_IOC_MAXNR    8

/* begin/end dma-buf functions used for userspace mmap. */
struct dma_buf_sync {
	__u64 flags;
};

#define DMA_BUF_SYNC_READ      (1 << 0)
#define DMA_BUF_SYNC_WRITE     (2 << 0)
#define DMA_BUF_SYNC_RW        (DMA_BUF_SYNC_READ | DMA_BUF_SYNC_WRITE)
#define DMA_BUF_SYNC_START     (0 << 2)
#define DMA_BUF_SYNC_END       (1 << 2)
#define DMA_BUF_SYNC_VALID_FLAGS_MASK \
	(DMA_BUF_SYNC_RW | DMA_BUF_SYNC_END)

#define DMA_BUF_BASE		'b'
#define DMA_BUF_IOCTL_SYNC	_IOW(DMA_BUF_BASE, 0, struct dma_buf_sync)

unsigned char *test_str = 
"(1)ion heap cma buffer test\r\n\
(2)ion heap cma buffer test\r\n\
(3)ion heap cma buffer test\r\n\
(4)ion heap cma buffer test\r\n\
(5)ion heap cma buffer test\r\n\
(6)ion heap cma buffer test\r\n\
(7)ion heap cma buffer test\r\n\
(8)ion heap cma buffer test\r\n\
(9)ion heap cma buffer test\r\n\
(10)ion heap cma buffer test\r\n";

static void usage(char *progname)
{
	fprintf(stderr,
		"usage: %s [options]\n"
		" -g       get data statistics\n"
		" -p 1|0   enable|disable sniffer packet\n"
		" -r       picp read\n"
		" -w       picp write\n"
		" -o       read/write onle\n"
		" -i       use ion buf\n"
		" -s val   buf size\n"
		" -n 1~4   kthread num\n"
		" -q       quit\n",
		progname);
}

static long get_curr_time(void) 
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (ts.tv_sec + ts.tv_nsec);
}

int alloc_ion_buffer(struct ion_buf_info *ion_info)
{
	int i, ret, ionfd;
	unsigned int heap_id;
	struct ion_allocation_data alloc_data;
	struct ion_heap_query query = {0};
	struct ion_heap_data heap_data[MAX_HEAP_COUNT];

	if (!ion_info) {
		printf("%s: Invalid ion info\n", __func__);
		return -1;
	}

	ionfd = open("/dev/ion", O_RDWR | O_CLOEXEC);
	if (ionfd < 0) {
		printf("Failed to open /dev/ion: %d\n", ionfd);
		return -1;
	}

	query.cnt = MAX_HEAP_COUNT;
	query.heaps = (unsigned long int)&heap_data[0];

	/* Query ION heap_id_mask from ION heap */
	ret = ioctl(ionfd, ION_IOC_HEAP_QUERY, &query);
	if (ret < 0) {
		printf("Failed ION_IOC_HEAP_QUERY\n");
		goto err_ion;
	}
	printf("ION HEAP Max:%d cnt:%d\n", MAX_HEAP_COUNT, query.cnt);

	heap_id = MAX_HEAP_COUNT + 1;
	for (i = 0; i < query.cnt; i++) {
		printf(" heap:type=%d, id=%d i=%d name=%s\n", 
			heap_data[i].type, heap_data[i].heap_id, i, heap_data[i].name);
		if (heap_data[i].type == ion_info->heap_type) {
			heap_id = heap_data[i].heap_id;
			//break;
		}
	}
	
	if (heap_id > MAX_HEAP_COUNT) {
		printf("ERROR: heap type does not exists\n");
		goto err_ion;
	}

	alloc_data.len = ion_info->heap_size;
	alloc_data.heap_id_mask = 1 << heap_id; //1 << heap_id;
	alloc_data.flags = ion_info->flag_type;

	/* Allocate memory for this ION client as per heap_type */
	ret = ioctl(ionfd, ION_IOC_ALLOC, &alloc_data);
	if (ret < 0) {
		printf("Failed: ION_IOC_ALLOC: %d\n", ret);
		goto err_ion;
	}

	/* This will return a valid buffer fd */
	if (alloc_data.fd < 0 || alloc_data.len <= 0) {
		printf("%s: Invalid map data, fd: %d, len: %ld\n",
			__func__, alloc_data.fd, alloc_data.len);
		goto err_fd;
	}

	ion_info->ionfd  = ionfd;
	ion_info->buffd  = alloc_data.fd;
	ion_info->buflen = alloc_data.len;

	printf("%s Success heap_id: %d len: %d flag: %d\n", 
		__func__, heap_id, ion_info->buflen, ion_info->flag_type);

	return 0;

err_fd:
	if (ion_info->buffd)
		close(ion_info->buffd);

err_ion:
	if (ionfd)
		close(ionfd);

	return -1;
}

void free_ion_buffer(struct ion_buf_info *ion_info)
{
	if (ion_info->buffd)
		close(ion_info->buffd);

	if (ion_info->ionfd)
		close(ion_info->ionfd);
}

int picp_ion_rwtest(int picp_fd, struct ioctl_info ioc)
{
	int i, ret;
	int map_flags = MAP_SHARED;
	int alloc_flags = 0, recodnum = 0;
	long start, curr;
	struct ion_buf_info info;
	struct dma_buf_sync sync = {0};
	struct ion_share_info ion_share;
	struct picp_cfg config;
	int type = ION_HEAP_TYPE_DMA;
	struct utsname buf;

	ret = uname(&buf);
	if (!ret) {
	#if 0
		printf("sysname:%s\n""nodename:%s\n""release:%s\n"
			"version:%s\n""machine:%s\n",
			buf.sysname, buf.nodename, buf.release,
			buf.version, buf.machine);
	#endif
		
		if (strncmp("5.10", buf.release, 2) == 0)
			type = 2;
		else
			type = 4;
	}

	/* picp init */
	config.timeout = 5000;
	config.rwopt = ioc.rwopt;
	ret = ioctl(picp_fd, PICP_IOC_INIT, &config);
	if (ret < 0) {
		perror("PICP_IOC_INIT\n");
		return -1;
	}

	/* alloc ion */
	info.heap_type = type;
	info.heap_size = 1048576;
	if (ioc.size)
		info.heap_size = ioc.size;
	info.flag_type = 0;
	ret = alloc_ion_buffer(&info);
	if (ret < 0) {
		printf("alloc_ion_buffer fail\n");
		return -1;
	}

	/* share ion fd */
	ion_share.buffd  = info.buffd;
	ion_share.buflen = info.buflen;
	ion_share.flag_type = info.flag_type;
	ion_share.heap_type = info.heap_type;
	ion_share.rwopt = ioc.rwopt;
	ret = ioctl(picp_fd, PICP_IOC_SIONFD, &ion_share);
	if (ret < 0) {
		perror("PICP_IOC_SIONFD");
		return -1;
	}

	/* Create memory mapped buffer for the buffer fd */
	info.buffer = (unsigned char *)mmap(NULL, info.buflen, PROT_READ|PROT_WRITE,
			MAP_SHARED, info.buffd, 0);
	if (info.buffer == MAP_FAILED) {
		printf("%s: Failed: mmap\n", __func__);
		return -1;
	}

	/* picp read/write */
	for (i = 0; i < 100; i++) {
		start = get_curr_time();
		if (!ioc.rwopt) {
			ret = read(picp_fd, NULL, info.heap_size);
			curr = get_curr_time();
			printf(" ion-buf read %d size:%ld time:%ld speed:%ld MB/s\n\n",
				recodnum++, info.heap_size, curr-start,
				((info.heap_size*1000000000)/(curr-start))/(1024*1024));
			continue;
		}
	
		sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
		ret = ioctl(info.buffd, DMA_BUF_IOCTL_SYNC, &sync);
		if (ret)
			printf("sync start failed %d\n", errno);

		memset(info.buffer, i, info.buflen);

		sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
		ret = ioctl(info.buffd, DMA_BUF_IOCTL_SYNC, &sync);
		if (ret)
			printf("sync end failed %d\n", errno);

		start = get_curr_time();
		ret = write(picp_fd, NULL, info.heap_size);
		curr = get_curr_time();
		printf(" ion-buf write %d size:%ld time:%ld speed:%ld MB/s\n\n",
			recodnum++, info.heap_size, curr-start,
			((info.heap_size*1000000000)/(curr-start))/(1024*1024));
	}

	free_ion_buffer(&info);

	ret = ioctl(picp_fd, PICP_IOC_EXIT, &config);
	if (ret < 0) {
		perror("PICP_IOC_EXIT\n");
		return -1;
	}

	munmap(info.buffer, info.buflen);
}

void picp_kthread_rwtest(int picp_fd, struct ioctl_info ioc)
{
	int ret = 0;
	
	ret = ioctl(picp_fd, PICP_IOC_RWTEST, &ioc);
	if (ret < 0)
		perror("PICP_IOC_RWTEST");
}


int main(int argc, char *argv[])
{
	char *progname;
	int c, cnt, picp_fd, ret, test, hold = 0, num = 0;
	int rwopt = 0, transfer = 0, single = 0, sniffer = 0, iontest = 0;
	int query = 0, on = 0;
	struct picp_record rec;

	int64_t t1, t2, tp;
	int64_t interval, offset;
	struct ioctl_info ioc = {0};
	ioc.size = 1048576;
	ioc.thread = 1;

	progname = strrchr(argv[0], '/');
	progname = progname ? 1+progname : argv[0];
	while (EOF != (c = getopt(argc, argv, "gt:ihs:p:drwoqn:"))) {
		switch (c) {
		case 'g':
			query = 1;
			break;
		case 'h':
			hold = 1;
			break;
		case 't':
			test = 1;
			//ioc.txctl = atoi(optarg);
			break;
		case 's':
			ioc.size = atoi(optarg);
			break;
		case 'r':
			transfer = 1;
			ioc.rwopt = 0;
			break;
		case 'w':
			transfer = 1;
			ioc.rwopt = 1;
			break;
		case 'o':
			ioc.single = 1;
			break;
		case 'i':
			iontest = 1;
			break;
		case 'n':
			ioc.thread = atoi(optarg);
			break;
		case 'p':
			sniffer = 1;
			on = atoi(optarg);
			ioc.sniffer = on;
			break;
		case 'q':
			transfer = 1;
			ioc.quit = 1;
			break;
		case '0':
		case '?':
		default:
			usage(progname);
			return -1;
		}
	}

	if (argc < 2) {
		usage(progname);
		return -1;
	}

	picp_fd = open("/dev/picp", O_RDWR);
	if (picp_fd < 0) {
		printf("Failed to open /dev/picp  %d\n", picp_fd);
		return -1;
	}

	if (query) {
		if (ioctl(picp_fd, PICP_IOC_QUERY, &rec) < 0) {
			perror("PICP_IOC_QUERY");
		} else {
		printf("picp: ltssm:%d\n"
		    "\tRX packets :%d  bytes %d (%d KB)\n"
		    "\t   errors  :%d  timeout %d dma-err %d link-err %d \n\n"
		    "\tTX packets :%d  bytes %d (%d KB)\n"
		    "\t   errors  :%d  timeout %d dma-err %d link-err %d \n\n",
			rec.ltssm_count,
		    rec.rtotal, rec.rbytes, rec.rbytes/1024,
		    rec.rerrors, rec.rtimeout, rec.rdmaerr, rec.rlinkerr,
		    rec.wtotal, rec.wbytes, rec.wbytes/1024,
		    rec.werrors, rec.wtimeout, rec.wdmaerr, rec.wlinkerr);
		}
	}

	if (sniffer) {
		ret = ioctl(picp_fd, PICP_IOC_SNIFFER, &on);
		if (ret < 0)
			perror("PICP_IOC_SNIFFER");
	}

	if (transfer) {
		if (iontest)
			picp_ion_rwtest(picp_fd, ioc);
		else
			picp_kthread_rwtest(picp_fd, ioc);
	}

	close(picp_fd);

	return 0;
}
