#define BSTCV_IOCTL_MAGIC 'r'
#define CV_IOCTL_WAIT     	_IO(BSTCV_IOCTL_MAGIC, 8)
#define CV_IOCTL_ALLOC		_IO(BSTCV_IOCTL_MAGIC, 9)
#define CV_IOCTL_FREE		_IO(BSTCV_IOCTL_MAGIC, 10)
#define BST_CV_IOCTL_DMA_BUF_IMPORT	_IO(BSTCV_IOCTL_MAGIC, 11)
#define BST_CV_IOCTL_DMA_BUF_EXPORT	_IO(BSTCV_IOCTL_MAGIC, 12)
#define BST_CV_IOCTL_DMA_BUF_FREE	_IO(BSTCV_IOCTL_MAGIC, 13)

#define MAX_FD_NUM 16
#define MAX_GWARP_CHANNEL 3
enum messions {
	BST_CV_GWARP = 0,
	BST_CV_SCALER,
	BST_CV_GWARP_SCALER,
	BST_CV_SCALER_GWARP
};

enum gwarp_format {
	GWARP_RGB888 = 0,
	GWARP_RGB888_PLANAR,
	GWARP_YUV422_YUYV,
	GWARP_YUV422_UYVY,
	GWARP_NV12,
	GWARP_NV21,
	GWARP_YUV420_PLANAR,
	GWARP_YUV422_PLANAR,
};

struct resolution {
	__u32 width;
	__u32 height;
};

typedef enum {
	BST_CV_BUFFER_USER,
	BST_CV_BUFFER_DMA,
} cv_input_buffer_type_t;
//phy address
struct gwarp_mem_info {
  __u32 in_dma_fd;
  __u32 out_dma_fd;
  __u32 gwarp_src_phy_addr;
  __u32 gwarp_dst_phy_addr;
  __u32 gwarp_lut_phy_addr;
};

typedef struct bst_cv_user_buffer {
  __u32 index;
  __u32 channel_num;
  __u8 pixel_format;
  __u32 byte_used;
  __u32 length;
  __u32 fd;
  __u64 ts_usec;
  void *uaddr;
  __u32 reserve[MAX_GWARP_CHANNEL];
} bst_cv_user_buffer_t;

struct gwarp_param {
	__u8 in_channel_num;
	__u8 out_channel_num;
	int is_bilinear;
	__u8 src_format;
	__u8 dst_format;
	__u32 out_dma_fd;
	struct resolution src_res;
	struct resolution dst_res;
	struct gwarp_mem_info mem_info;
};

struct bst_cv_req {
	cv_input_buffer_type_t in_type;
	cv_input_buffer_type_t out_type;
	struct gwarp_param g_param;
};

struct bst_cv_rsp {
	__u32 cookie;
	bst_cv_user_buffer_t cv_buffer;
};

struct bst_cv_msg {
	__u32 cookie;
	enum messions mission;
    struct bst_cv_req req;
    struct bst_cv_rsp rsp;
};

/* memory */
struct bst_cv_dma_buf {
	int fd;                             // the dma-buf file descriptor
	__u32 bus_addr;               // the dma-buf bus address
	__u32 size;                  		// the dma-buf size
};

typedef struct buf_req {
    int buf_fd;
	int size;
    unsigned long phy_addr;
} buf_req_t;

struct xrp_ioctl_alloc {
    __u32 size;
    __u32 align;
    __u32 addr;
    __u64 ptr;
};