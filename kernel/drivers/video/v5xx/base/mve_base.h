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

#ifndef MVE_BASE_H
#define MVE_BASE_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define IOCTL_MAGIC 251
#define MVE_COMMAND _IOWR(IOCTL_MAGIC, 0, struct mve_command_header)

#define MVE_FLAGS_DMABUF_DISABLE_CACHE_MAINTENANCE 0x80000000

/**
 * Commands from user- to kernel space.
 */
enum mve_command_type
{
    MVE_CREATE_SESSION,
    MVE_DESTROY_SESSION,
    MVE_ACTIVATE_SESSION,

    MVE_ENQUEUE_FLUSH_BUFFERS,
    MVE_ENQUEUE_STATE_CHANGE,

    MVE_GET_EVENT,
    MVE_SET_PARAMETER,
    MVE_GET_PARAMETER,
    MVE_SET_CONFIG,
    MVE_GET_CONFIG,

    MVE_REGISTER_BUFFER,
    MVE_UNREGISTER_BUFFER,
    MVE_FILL_THIS_BUFFER,
    MVE_EMPTY_THIS_BUFFER,

    MVE_NOTIFY_REF_FRAME_RELEASE,

    MVE_REQUEST_MAX_FREQUENCY,

    MVE_READ_HW_INFO,

    MVE_RPC_MEM_ALLOC,
    MVE_RPC_MEM_RESIZE,

    MVE_DEBUG_READ_REGISTER,
    MVE_DEBUG_WRITE_REGISTER,
    MVE_DEBUG_INTERRUPT_COUNT,
    MVE_DEBUG_SEND_COMMAND,
    MVE_DEBUG_FIRMWARE_HUNG_SIMULATION,
};

/**
 * External buffer identifier. The data stored by this type is specific for each
 * buffer API implementation. The mve_buffer_valloc implementation uses this
 * type to store addresses to user space allocated memory (virtual addresses). A buffer
 * API implementation for Android will use this type to store gralloc handles. */
typedef uint64_t mve_buffer_handle_t;

/**
 * @brief Represents a command sent to the driver for processing.
 */
struct mve_command_header
{
    uint32_t cmd;       /**< Which command to execute */
    uint32_t size;      /**< Size of the data section (excluding header size) */
    uint8_t data[0];    /**< First byte of the data section. */
};

/**
 * @brief Region of interest structure.
 *
 * The region is macroblock positions (x,y) in the range
 * mbx_left <= x < mbx_right
 * mby_top  <= y < mby_bottom
 */
struct roi_region
{
    uint16_t mbx_left;   /**< X coordinate of left macro block */
    uint16_t mbx_right;  /**< X coordinate of right macro block */
    uint16_t mby_top;    /**< Y coordinate of top macro block */
    uint16_t mby_bottom; /**< Y coordinate of bottom macro block */
    int16_t qp_delta;    /**< Delta relative the default QP value */
};

#define ROI_REGIONS_MAX 16

/**
 * @brief Instances of this structure are sent along with fill/empty this buffer messages.
 */
struct mve_buffer_details
{
    mve_buffer_handle_t buffer_id;              /**< Buffer unique ID */
    mve_buffer_handle_t handle;                 /**< Handle to buffer */
    uint32_t filled_len;                        /**< Size of the contents in the buffer in bytes */
    uint32_t flags;                             /**< OMX buffer flags */
    uint32_t mve_flags;                         /**< MVE buffer flags */
    uint32_t crc_offset;                        /**< Offset of the CRC data in the buffer. Only valid
                                                 *   for CRC buffers using the attachment allocator. */
    uint64_t timestamp;                         /**< Buffer timestamp */
    uint8_t nRegions;                           /**< Number of ROI regions */
    uint8_t reserved[3];                        /**< Unused but required for alignment reasons */
    struct roi_region regions[ROI_REGIONS_MAX]; /**< ROI data */
};

/**
 * Events from kernel- to user space.
 */
enum mve_event_code
{
    MVE_EVENT_RPC_PRINT          = 0,  /**< Data contains a NULL terminated string */
    MVE_EVENT_SWITCHED_IN        = 1,
    MVE_EVENT_SWITCHED_OUT       = 2,
    MVE_EVENT_PONG               = 3,
    MVE_EVENT_STATE_CHANGED      = 4,
    MVE_EVENT_ERROR              = 5,
    MVE_EVENT_GENERIC            = 6,
    MVE_EVENT_PROCESSED          = 7,
    MVE_EVENT_INPUT              = 8,
    MVE_EVENT_OUTPUT             = 9,
    MVE_EVENT_GET_PARAMCONFIG    = 10,
    MVE_EVENT_SET_PARAMCONFIG    = 11,
    MVE_EVENT_INPUT_FLUSHED      = 12,
    MVE_EVENT_OUTPUT_FLUSHED     = 13,
    MVE_EVENT_CODE_DUMP          = 14,
    MVE_EVENT_SESSION_HUNG       = 15,
    MVE_EVENT_ALLOC_PARAMS       = 16,
    MVE_EVENT_SEQUENCE_PARAMS    = 17,
    MVE_EVENT_BUFFER_PARAM       = 18,
    MVE_EVENT_REF_FRAME_RELEASED = 19,
    MVE_EVENT_RPC_MEM_ALLOC      = 20,
    MVE_EVENT_RPC_MEM_RESIZE     = 21,

    /* These messages are not forwarded to userspace and must therefore
     * be places last in this enum. */
    MVE_EVENT_JOB_DEQUEUED     = 22,
    MVE_EVENT_IDLE             = 23,
    MVE_EVENT_FW_TRACE_BUFFERS = 24,
};

enum mve_event_buffer_param_type
{
    MVE_EVENT_BUFFER_PARAM_TYPE_DPB_HELD_FRAMES = 19,
};

/**
 * The event header of an event. Contains the event code and
 * size of the data attached to the event.
 */
struct mve_event_header
{
    uint16_t code;   /**< Event code */
    uint16_t size;   /**< Size of the data attached to the event in bytes */
    uint8_t data[0]; /**< First byte of the data attached to the event */
};

/**
 * Error codes for MVE responses.
 */
typedef enum
{
    MVE_ERROR_NONE,
    MVE_ERROR_UNDEFINED,
    MVE_ERROR_BAD_PARAMETER,
    MVE_ERROR_BAD_PORT_INDEX,
    MVE_ERROR_FIRMWARE,
    MVE_ERROR_HARDWARE,
    MVE_ERROR_INSUFFICIENT_RESOURCES,
    MVE_ERROR_NOT_IMPLEMENTED,
    MVE_ERROR_NOT_READY,
    MVE_ERROR_TIMEOUT,
    MVE_ERROR_VERSION_MISMATCH
} mve_error;

/**
 * @brief Represents the result of an executed command.
 */
struct mve_response_header
{
    uint32_t error;             /**< MVE error code */
    uint32_t firmware_error;    /**< Firmware error code */
    uint32_t size;              /**< Size of the data section (excluding header size) */
    uint8_t data[0];            /**< First byte of the data section */
};

/**
 * This enum lists the different formats of supplied buffers.
 */
enum mve_buffer_format
{
    MVE_BUFFER_FORMAT_YUV420_PLANAR,     /**< Planar YUV buffer (3 planes) */
    MVE_BUFFER_FORMAT_YUV420_SEMIPLANAR, /**< Semiplanar YUV (2 planes) */
    MVE_BUFFER_FORMAT_YUYYVY_10B,        /**< ARM 10-bit YUV 420 format */
    MVE_BUFFER_FORMAT_YUV420_AFBC,       /**< YUV buffer compressed with AFBC */
    MVE_BUFFER_FORMAT_YUV420_AFBC_10B,   /**< 10-bit YUV buffer compressed with AFBC */
    MVE_BUFFER_FORMAT_YUV422_1P,         /**< YUV buffer (1 plane) */
    MVE_BUFFER_FORMAT_BITSTREAM,         /**< Compressed bitstream data */
    MVE_BUFFER_FORMAT_CRC,               /**< CRC buffer */
    MVE_BUFFER_FORMAT_YV12,              /**< Planar YV12 buffer (3 planes) */
    MVE_BUFFER_FORMAT_YVU420_SEMIPLANAR, /**< Semilanar YVU (2 planes) */
    MVE_BUFFER_FORMAT_RGBA_8888,         /**< RGB format with 32 bit as Red 31:24, Green 23:16, Blue 15:8, Alpha 7:0 */
    MVE_BUFFER_FORMAT_BGRA_8888,         /**< RGB format with 32 bit as Blue 31:24, Green 23:16, Red 15:8, Alpha 7:0 */
    MVE_BUFFER_FORMAT_ARGB_8888,         /**< RGB format with 32 bit as Alpha 31:24, Red 23:16, Green 15:8, Blue 7:0 */
    MVE_BUFFER_FORMAT_ABGR_8888,         /**< RGB format with 32 bit as Alpha 31:24, Blue 23:16, Green 15:8, Red 7:0 */
};

/**
 * Allocator used to allocate a user-space allocated buffer.
 */
enum mve_buffer_allocator
{
    MVE_BUFFER_ALLOCATOR_VMALLOC,    /**< Memory allocated by valloc or malloc. */
    MVE_BUFFER_ALLOCATOR_ATTACHMENT, /**< Represents a buffer that is part of another buffer. */
    MVE_BUFFER_ALLOCATOR_DMABUF,     /**< Memory wrapped by dma_buf. */
    MVE_BUFFER_ALLOCATOR_ASHMEM,     /**< Memory wrapped by ashmem. */
};

/**
 * This structure is used to transfer buffer information between users- and kernel space.
 */
struct mve_buffer_userspace
{
    uint64_t timestamp;                       /**< Buffer timestamp. */
    mve_buffer_handle_t buffer_id;            /**< Buffer unique ID. */
    mve_buffer_handle_t handle;               /**< Handle to the external buffer. */
    mve_buffer_handle_t crc_handle;           /**< Handle to the external CRC buffer. */

    enum mve_buffer_allocator allocator;      /**< Specifies which allocator was used to allocate
                                               *   the buffer. */
    uint32_t size;                            /**< Size of the external buffer in bytes. */
    uint32_t width;                           /**< Width of the buffer (only for pixel formats). */
    uint32_t height;                          /**< Height of the buffer (only for pixel formats). */
    uint32_t stride;                          /**< Stride of the buffer (only for pixel formats). */
    uint32_t stride_alignment;                /**< Aligntment of the stride in bytes (only for pixel formats). */
    enum mve_buffer_format format;            /**< Format of the buffer. */

    uint32_t decoded_width;                   /**< Width of the decoded frame. Only valid for a returned frame buffer */
    uint32_t decoded_height;                  /**< Height of the decoded frame. Only valid for a returned frame buffer */

    uint32_t afbc_width_in_superblocks;       /**< Width of the AFBC buffer in superblocks (only for AFBC formats) */
    uint32_t afbc_alloc_bytes;                /**< Size of the AFBC frame */

    uint32_t filled_len;                      /**< Number of bytes worth of data in the buffer. */
    uint32_t offset;                          /**< Offset from start of buffer to first byte. */
    uint32_t flags;                           /**< Flags for OMX use. */
    uint32_t mve_flags;                       /**< MVE sideband information. */
    uint32_t pic_index;                       /**< Picture index in decode order. Output from FW. */

    uint16_t cropx;                           /**< Luma x crop. */
    uint16_t cropy;                           /**< Luma y crop. */
    uint8_t y_offset;                         /**< Deblocking y offset of picture. */

    enum mve_buffer_allocator crc_allocator;  /**< CRC buffer allocator. */
    uint32_t crc_size;                        /**< Size of the CRC buffer. */
    uint32_t crc_offset;                      /**< Offset of the CRC data in the buffer. */
};

enum mve_hw_state
{
    MVE_HW_STATE_STOPPED = 0,                /**< HW in STOPPED state. */
    MVE_HW_STATE_RUNNING = 2,                /**< HW in RUNNING state. */
    MVE_HW_STATE_PENDING = 4                 /**< Requested for HW State change and waiting for the response. */
};

/**
 * Defines what port(s) to flush.
 */
enum mve_flush
{
    MVE_FLUSH_INPUT_PORT = 1,  /**< Flush the input port */
    MVE_FLUSH_OUTPUT_PORT = 2, /**< Flush the output port */
    MVE_FLUSH_ALL_PORTS = MVE_FLUSH_INPUT_PORT | MVE_FLUSH_OUTPUT_PORT,
    /**< Flush input and output ports */
    MVE_FLUSH_QUICK = 1 << 2,  /**< Perform a quick flush. Quick flush means that
                                *   all flushed buffers will automatically be
                                *   re-enqueued once all buffers have been flushed.
                                *   Userspace will not be notified of the flushed
                                *   buffers or that the flush is complete. */
    MVE_FLUSH_QUICK_SET_INTERLACE = 1 << 3,
    /**< Makes all output buffers added as interlaced
     *   buffers once the quick flush is completed */
};

struct mve_hw_info
{
    uint32_t fuse;                     /**< Hardware fuse register. */
    uint32_t version;                  /**< Hardware version. */
    uint32_t ncores;                   /**< Number of MVE Cores*/
};

struct mve_fw_version
{
    uint8_t major;                    /**< Firmware major version. */
    uint8_t minor;                    /**< Firmware minor version. */
};

struct mve_fw_secure_descriptor
{
    struct mve_fw_version fw_version; /**< FW protocol version */
    uint32_t l2pages;                 /**< Physical address of l2pages created by secure OS */
};

struct mve_fw_frame_alloc_parameters
{
    uint16_t planar_alloc_frame_width;             /**< Width of planar YUV buffer */
    uint16_t planar_alloc_frame_height;            /**< Height of planar YUV buffer */

    uint32_t afbc_alloc_bytes;                     /**< Number of bytes needed for an AFBC buffer */

    uint32_t afbc_alloc_bytes_downscaled;          /**< Number of bytes needed for downscaled AFBC buffer */

    uint16_t afbc_width_in_superblocks;            /**< Width of the AFBC buffer needed by the FW */
    uint16_t afbc_width_in_superblocks_downscaled; /**< Width of the downscaled AFBC buffer needed by the FW */

    uint16_t cropx;                                /**< Hints on how much to adjust the plane addresses to get optimal AXI bursts */
    uint16_t cropy;                                /**< Hints on how much to adjust the plane addresses to get optimal AXI bursts */

    uint32_t mbinfo_alloc_bytes;                   /* Only for debugging */
};

#endif /* MVE_BASE_H */
