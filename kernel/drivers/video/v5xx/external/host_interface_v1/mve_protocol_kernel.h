/*
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */

#ifndef __FW_INCLUDE__MVE_PROTOCOL_KERNEL_H__
#define __FW_INCLUDE__MVE_PROTOCOL_KERNEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/*
 * Virtual memory regions
 *
 * ..._ADDR_BEGIN gives the starting virtual address of the region,
 * and ..._ADDR_END the (non-inclusive) ending address, such that
 * the size of the region is obtained with the subtraction
 * (..._ADDR_END - ..._ADDR_BEGIN).
 */

/* Memory region for first firmware instance */
#define MVE_MEM_REGION_FW_INSTANCE0_ADDR_BEGIN (0x00000000u)
#define MVE_MEM_REGION_FW_INSTANCE0_ADDR_END   (0x000FFFFFu + 1)

/*
 * Areas for communication between host and MVE are placed in the interval
 * 0x10079000 - 0x1007FFFF, see special defines further down.
 */

/* PROTECTED virtual memory region */
#define MVE_MEM_REGION_PROTECTED_ADDR_BEGIN    (0x20000000u)
#define MVE_MEM_REGION_PROTECTED_ADDR_END      (0x4FFFFFFFu + 1)

/* FRAMEBUF virtual memory region */
#define MVE_MEM_REGION_FRAMEBUF_ADDR_BEGIN       (0x50000000u)
#define MVE_MEM_REGION_FRAMEBUF_ADDR_END         (0x7FFFFFFFu + 1)

/* Memory regions for other firmware instances */
#define MVE_MEM_REGION_FW_INSTANCE1_ADDR_BEGIN (0x80000000u)
#define MVE_MEM_REGION_FW_INSTANCE1_ADDR_END   (0x8FFFFFFFu + 1)
#define MVE_MEM_REGION_FW_INSTANCE2_ADDR_BEGIN (0x90000000u)
#define MVE_MEM_REGION_FW_INSTANCE2_ADDR_END   (0x9FFFFFFFu + 1)
#define MVE_MEM_REGION_FW_INSTANCE3_ADDR_BEGIN (0xA0000000u)
#define MVE_MEM_REGION_FW_INSTANCE3_ADDR_END   (0xAFFFFFFFu + 1)
#define MVE_MEM_REGION_FW_INSTANCE4_ADDR_BEGIN (0xB0000000u)
#define MVE_MEM_REGION_FW_INSTANCE4_ADDR_END   (0xBFFFFFFFu + 1)
#define MVE_MEM_REGION_FW_INSTANCE5_ADDR_BEGIN (0xC0000000u)
#define MVE_MEM_REGION_FW_INSTANCE5_ADDR_END   (0xCFFFFFFFu + 1)
#define MVE_MEM_REGION_FW_INSTANCE6_ADDR_BEGIN (0xD0000000u)
#define MVE_MEM_REGION_FW_INSTANCE6_ADDR_END   (0xDFFFFFFFu + 1)
#define MVE_MEM_REGION_FW_INSTANCE7_ADDR_BEGIN (0xE0000000u)
#define MVE_MEM_REGION_FW_INSTANCE7_ADDR_END   (0xEFFFFFFFu + 1)
/* 0xF0000000 - 0xFFFFFFFF is used internally in MVE */

/* Communication queues between HOST/DRIVER and MVE */
/*
 * Address for queue for messages in to MVE,
 * one struct mve_comm_area_host located here
 */
#define MVE_MSG_INQ     0x10079000u

/*
 * Address for queue for messages out from MVE,
 * one struct mve_comm_area_mve located here
 */
#define MVE_MSG_OUTQ    0x1007A000u

/*
 * Address for queue for input buffers in to MVE,
 * one struct mve_comm_area_host located here
 */
#define MVE_BUF_INQ     0x1007B000u

/*
 * Address for queue for input buffers returned from MVE,
 * one struct mve_comm_area_mve located here
 */
#define MVE_BUF_INRQ    0x1007C000u

/*
 * Address for queue for output buffers in to MVE,
 * one struct mve_comm_area_host located here
 */
#define MVE_BUF_OUTQ    0x1007D000u

/*
 * Address for queue for output buffers returned from MVE,
 * one struct mve_comm_area_mve located here
 */
#define MVE_BUF_OUTRQ   0x1007E000u

/* One struct mve_rpc_comunication_area located here */
#define MVE_RPC_BUF     0x1007F000u

/*
 * One page of memory (4 kB) is used for each queue,
 * so maximum 1024 words, but need room for some counters as well,
 * see structs mve_comm_area_mve and mve_comm_area_host below.
 */
#define MVE_COMM_QUEUE_SIZE_IN_WORDS 1020

/* This is the part of the message area that is written by host. */
struct mve_comm_area_host {
    volatile uint16_t out_rpos;
    volatile uint16_t in_wpos;
    volatile uint32_t reserved[3];
    /*
     * Queue of messages to MVE, each block of data prefixed with
     * a mve_msg_header
     */
    volatile uint32_t in_data[MVE_COMM_QUEUE_SIZE_IN_WORDS];
};

/* This is the part of the message area that is written by MVE. */
struct mve_comm_area_mve {
    volatile uint16_t out_wpos;
    volatile uint16_t in_rpos;
    volatile uint32_t reserved[3];
    /*
     * Queue of messages to host, each block of data prefixed with
     * a mve_msg_header
     */
    volatile uint32_t out_data[MVE_COMM_QUEUE_SIZE_IN_WORDS];
};

/* Data sent between host and mve is prefixed with a header */
struct mve_msg_header {
    uint16_t code;
    uint16_t size;
};

struct mve_job_command {
    uint16_t cores;
    uint16_t frames;
    uint32_t flags;
};

/* Message codes for messages from host to MVE */
enum MVE_MESSAGE_CODE {
    MVE_MESSAGE_CODE_GO            =  1,
    MVE_MESSAGE_CODE_STOP          =  2,
    MVE_MESSAGE_CODE_INPUT_FLUSH   =  4,
    MVE_MESSAGE_CODE_OUTPUT_FLUSH  =  5,
    MVE_MESSAGE_CODE_SET_PARAMETER =  6,
    MVE_MESSAGE_CODE_GET_PARAMETER =  7,
    MVE_MESSAGE_CODE_SWITCH        =  8,
    MVE_MESSAGE_CODE_PING          =  9,
    MVE_MESSAGE_CODE_RESET         = 10,
    MVE_MESSAGE_CODE_SET_CONFIG    = 11,
    MVE_MESSAGE_CODE_GET_CONFIG    = 12,
    MVE_MESSAGE_CODE_DUMP          = 13,
    MVE_MESSAGE_CODE_JOB           = 14
};

/* Response codes for responses from MVE to host */
enum MVE_RESPONSE_CODE {
    MVE_RESPONSE_CODE_INPUT               =  1,
    MVE_RESPONSE_CODE_OUTPUT              =  2,
    MVE_RESPONSE_CODE_PROCESSED           =  3,
    MVE_RESPONSE_CODE_EVENT               =  4,
    MVE_RESPONSE_CODE_SWITCHED_OUT        =  5,
    MVE_RESPONSE_CODE_SWITCHED_IN         =  6,
    MVE_RESPONSE_CODE_ERROR               =  7,
    MVE_RESPONSE_CODE_PONG                =  8,
    MVE_RESPONSE_CODE_STATE_CHANGE        =  9,
    MVE_RESPONSE_CODE_GET_PARAMETER_REPLY = 10,
    MVE_RESPONSE_CODE_SET_PARAMETER_REPLY = 11,
    MVE_RESPONSE_CODE_GET_CONFIG_REPLY    = 12,
    MVE_RESPONSE_CODE_SET_CONFIG_REPLY    = 13,
    MVE_RESPONSE_CODE_INPUT_FLUSHED       = 14,
    MVE_RESPONSE_CODE_OUTPUT_FLUSHED      = 15,
    MVE_RESPONSE_CODE_DUMP                = 16,
    MVE_RESPONSE_CODE_JOB_DEQUEUED        = 17,
    MVE_RESPONSE_CODE_IDLE                = 18
};

/*
 * #define, not enum because mve_userspace.h already introduced
 * enum with these names.
 */
#define MVE_STATE_STOPPED 0
#define MVE_STATE_RUNNING 2

enum MVE_ERROR_CODE {
    MVE_ERROR_ABORT          = 1,
    MVE_ERROR_OUT_OF_MEMORY  = 2,
    MVE_ERROR_ASSERT         = 3,
    MVE_ERROR_UNSUPPORTED    = 4,
    MVE_ERROR_INVALID_BUFFER = 6,
    MVE_ERROR_CORRUPT_STREAM = 7,
    MVE_ERROR_INVALID_STATE  = 8,
    MVE_ERROR_WATCHDOG       = 9
};

#define MVE_IN_PORT  0
#define MVE_OUT_PORT 1

/* Buffer contains an interlaced frame */
#define MVE_FLAGS_INTERLACE          0x1

/* Bottom field first (interlaced only) */
#define MVE_FLAGS_BOT_FIRST          0x2

/* Buffer contains a frame or top field */
#define MVE_FLAGS_TOP_PRESENT        0x4

/* Buffer contains a bottom field */
#define MVE_FLAGS_BOT_PRESENT        0x8

/* Mask used for field presense */
#define MVE_FLAGS_FIELD_MASK         0xC

/* Frame is rotated 90 degrees */
#define MVE_FLAGS_ROTATION_90        0x10

/* Frame is rotated 180 degrees */
#define MVE_FLAGS_ROTATION_180       0x20

/* Frame is rotated 270 degrees */
#define MVE_FLAGS_ROTATION_270       0x30

/* Mask used for rotations in nMVEFlags */
#define MVE_FLAGS_ROTATION_MASK      0x30

/* Buffer contains a CRC map */
#define MVE_FLAGS_CRC_PRESENT        0x40

/* Enable the low latency encoding */
#define MVE_FLAGS_LOW_LATENCY_ENABLE 0x80

/* Qp specified for this frame */
#define MVE_FLAGS_QP_PRESENT         0x10000

/* Regions specified for this frame */
#define MVE_FLAGS_ROI_PRESENT        0x20000

/* bits 31..24 are number of output buffers that are complete and
 * held by firmware in the DPB for reordering purposes
 */ 
#define MVE_FLAGS_DPB_FRAMES_SHIFT   24
#define MVE_FLAGS_DPB_FRAMES_MASK    0xFF000000

/* The planar buffer stores 3-plane decompressed video content */
struct MVE_BUFFER_PLANAR {
    /* Stride between rows for 0 and 180 deg rotation */
    int16_t stride[3];

    /* Stride between rows for 90 and 270 deg rotation */
    int16_t stride_90[3];

    /* Y,Cb,Cr top field */
    uint32_t plane_top[3];

    /* Y,Cb,Cr bottom field (interlace only) */
    uint32_t plane_bot[3];
};

/*
 * The AFBC buffer stores AFBC compressed content that is also used
 * as the reference frame. Out of loop processing (crop, rotation,
 * range reduction) must be supported by the user of this buffer and
 * the parameters are signaled within the buffer descriptor below.
 */
struct MVE_BUFFER_AFBC {
    uint32_t plane_top; /* Top field (interlace) or frame (progressive) */
    uint32_t plane_bot; /* Bottom field (interlace only) */
    uint16_t cropx;     /* Luma x crop */
    uint16_t cropy;     /* Luma y crop */
    uint8_t  y_offset;  /* Deblocking y offset of picture */
    uint8_t  rangemap;  /* Packed VC-1 Luma and Chroma range map coefs */
};

/*
 * The FRAME buffer stores the common information for PLANAR and AFBC buffers,
 * and a union of PLANAR and AFBC specific information.
 */
struct MVE_BUFFER_FRAME {
    /*
     * Host buffer handle number
     * WARNING: struct mve_core_buffer_header relies on having the same
     * nHandle position as MVE_BUFFER
     */
    uint16_t nHandle;

    /* Ensure alignment (even if forced packed) */
    uint16_t nUnused0;

    /* BufferFlags */
    uint32_t nFlags;

    /* User supplied tracking identifier (keep aligned) */
    uint64_t nUserDataTag;

    /* MVE sideband information */
    uint32_t nMVEFlags;

    /* Picture index in decode order */
    uint32_t pic_index;

    /* Decoded height may be smaller than display height */
    uint16_t decoded_height;

    /* Decoded width may be smaller than display width */
    uint16_t decoded_width;
    union {
        struct MVE_BUFFER_PLANAR planar;
        struct MVE_BUFFER_AFBC afbc;
    } data;

    /* Below fields are valid since Host interface spec v1.1 */

    /* CRC map address top field or frame */
    uint32_t crc_top;

    /* CRC map bottom field */
    uint32_t crc_bot;

    /*
     * Addr of a 32-bit word indicates how many luma pixel rows have been
     * written by host
     */
    uint32_t pRowsCounter;

    /* Base Qp to use for encoding this picture */
    uint8_t  QP;

    uint16_t Padding16[3];
    uint32_t Padding;
};

/* The bitstream buffer stores a number of bitstream bytes */
struct MVE_BUFFER_BITSTREAM {
    /*
     * Host buffer handle number
     * WARNING: struct mve_core_buffer_header relies on having the same
     * nHandle position as MVE_BUFFER
     */
    uint16_t nHandle;

    /* Ensure alignment (even if forced packed) */
    uint16_t nUnused0;

    /* BufferFlags */
    uint32_t nFlags;

    /* User supplied tracking identifier (keep aligned) */
    uint64_t nUserDataTag;

    /* Length of allocated buffer */
    uint32_t nAllocLen;

    /* Byte offset from start to first byte */
    uint32_t nOffset;

    /* Number of bytes in the buffer */
    uint32_t nFilledLen;

    /* Pointer to buffer start */
    uint32_t pBufferData;
};

/*
 * Define a region in 16x16 units
 *
 * The region is macroblock positions (x,y) in the range
 * mbx_left <= x < mbx_right
 * mby_top  <= y < mby_bottom
 */
struct MVE_FRAME_REGION {
    uint16_t mbx_left;   /* macroblock x left edge   (inclusive) */
    uint16_t mbx_right;  /* macroblock x right edge  (exclusive) */
    uint16_t mby_top;    /* macroblock y top edge    (inclusive) */
    uint16_t mby_bottom; /* macroblock y bottom edge (exclusive) */
    int8_t   qp_delta;   /* QP delta value for this region       */
};

/*
 * The MVE_BUFFER_REGION buffer stores the information for FRAME buffers,
 * and the information for regions of interest.
 */
#define MVE_MAX_FRAME_REGIONS 16
struct MVE_BUFFER_REGION {
    uint8_t  nRegions;   /* Number of regions */
    struct MVE_FRAME_REGION region[MVE_MAX_FRAME_REGIONS];
};

union MVE_BUFFER {
    struct MVE_BUFFER_FRAME     frame;
    struct MVE_BUFFER_BITSTREAM bitstream;
    struct MVE_BUFFER_REGION    region;
};

#define MVE_MSG_HEADER_CODE_BUFFER_BITSTREAM  0
#define MVE_MSG_HEADER_CODE_BUFFER_FRAME      1
#define MVE_MSG_HEADER_CODE_BUFFER_REGION     2
#define MVE_MAX_MESSAGE_SIZE_IN_WORDS 192

/* RPC */
enum MVE_RPC_FUNCTION {
    MVE_RPC_FUNCTION_DEBUG_PRINTF = 1,
    MVE_RPC_FUNCTION_MEM_ALLOC    = 2,
    MVE_RPC_FUNCTION_MEM_RESIZE   = 3,
    MVE_RPC_FUNCTION_MEM_FREE     = 4
};

/* Memory region selection */
enum MVE_MEM_REGION {
    MVE_MEM_REGION_PROTECTED = 0,
    MVE_MEM_REGION_OUTBUF    = 1,
    MVE_MEM_REGION_FRAMEBUF  = MVE_MEM_REGION_OUTBUF
};

enum MVE_RPC_STATE {
    MVE_RPC_STATE_FREE   = 0,
    MVE_RPC_STATE_PARAM  = 1,
    MVE_RPC_STATE_RETURN = 2
};

#define MVE_RPC_AREA_SIZE_IN_WORDS 256
#define MVE_RPC_DATA_SIZE_IN_WORDS (MVE_RPC_AREA_SIZE_IN_WORDS - 3)
union rpc_params {
    volatile uint32_t data[MVE_RPC_DATA_SIZE_IN_WORDS];

    struct {
        char string[MVE_RPC_DATA_SIZE_IN_WORDS * 4];
    } debug_print;

    struct {
        uint32_t size;
        uint32_t max_size;
        uint8_t region;
    } mem_alloc;

    struct {
        uint32_t ve_pointer;
        uint32_t new_size;
    } mem_resize;

    struct {
        uint32_t ve_pointer;
    } mem_free;

    struct {
        uint32_t ve_pointer;
        uint32_t n_pages;
        uint32_t bitmap_size;
        uint32_t bitmap[MVE_RPC_DATA_SIZE_IN_WORDS - 3];
   } mem_map;
};

struct mve_rpc_comunication_area {
    volatile uint32_t state;
    volatile uint32_t call_id;
    volatile uint32_t size;
    union rpc_params params;
};

#ifdef __cplusplus
}
#endif

#endif /* __FW_INCLUDE__MVE_PROTOCOL_KERNEL_H__ */
