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

#ifdef EMULATOR
//#include "emulator_userspace.h"
#else
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/version.h>
#endif

#include "mve_session.h"
#include "mve_com.h"
#include "mve_rsrc_log.h"
#include "mve_rsrc_log_ram.h"

#include <host_interface_v1/mve_protocol_kernel.h>

/**
 * Write data at offset in the buffer pointed out by ptr.
 * The type of the data is specified by the argument type.
 */
#define WRITE_QUEUE(ptr, type, offset, data) \
    *((type *)&((ptr)[(offset) % MVE_COMM_QUEUE_SIZE_IN_WORDS])) = data

/**
 * Read data at offset in the buffer pointed out by src.
 * The type of data is specified by the type argument.
 */
#define READ_QUEUE(src, type, offset, dst) \
    dst = *((type *)&((src)[(offset) % MVE_COMM_QUEUE_SIZE_IN_WORDS]))

/**
 * Round up value. Rounding must be power of 2.
 */
#define ROUNDUP(v, r) (((v) + (r) - 1) & ~(r - 1))

/**
 * Convert from driver buffer structure to FW buffer structure.
 */
static void convert_to_mve_buffer(struct mve_session *session,
                                  mve_com_buffer *src,
                                  union MVE_BUFFER *dst,
                                  enum mve_com_buffer_type type)
{
    int i;

    memset(dst, 0, sizeof(union MVE_BUFFER));

    switch (type)
    {
        case MVE_COM_BUFFER_TYPE_FRAME:
            dst->frame.nHandle            = src->frame.nHandle;
            dst->frame.nFlags             = src->frame.nFlags;
            dst->frame.nUserDataTag       = src->frame.timestamp;
            dst->frame.nMVEFlags          = src->frame.nMVEFlags;
            dst->frame.pic_index          = src->frame.pic_index;
            dst->frame.decoded_height     = src->frame.decoded_height;
            dst->frame.decoded_width      = src->frame.decoded_width;
            memcpy(&dst->frame.data, &src->frame.data, sizeof(dst->frame.data));
            dst->frame.crc_top            = src->frame.crc_top;
            dst->frame.crc_bot            = src->frame.crc_bot;

            if ((dst->frame.nMVEFlags & (MVE_FLAGS_TOP_PRESENT | MVE_FLAGS_BOT_PRESENT)) == 0)
            {
                dst->frame.decoded_height     = 0;
                dst->frame.decoded_width      = 0;
            }
            break;
        case MVE_COM_BUFFER_TYPE_BITSTREAM:
            dst->bitstream.nHandle      = src->bitstream.nHandle;
            dst->bitstream.nFlags       = src->bitstream.nFlags;
            dst->bitstream.nUserDataTag = src->bitstream.timestamp;
            dst->bitstream.nAllocLen    = src->bitstream.nAllocLen;
            dst->bitstream.nOffset      = src->bitstream.nOffset;
            dst->bitstream.nFilledLen   = src->bitstream.nFilledLen;
            dst->bitstream.pBufferData  = src->bitstream.pBufferData;
            break;
        case MVE_COM_BUFFER_TYPE_ROI:
            dst->region.nRegions = src->roi.nRegions;
            for (i = 0; i < src->roi.nRegions; ++i)
            {
                dst->region.region[i].mbx_left = src->roi.regions[i].mbx_left;
                dst->region.region[i].mbx_right = src->roi.regions[i].mbx_right;
                dst->region.region[i].mby_bottom = src->roi.regions[i].mby_bottom;
                dst->region.region[i].mby_top = src->roi.regions[i].mby_top;
                dst->region.region[i].qp_delta = (int8_t)src->roi.regions[i].qp_delta;
            }
            break;
    }
}

/**
 * Convert from FW buffer structure to driver buffer structure.
 */
static void convert_from_mve_buffer(struct mve_session *session,
                                    union MVE_BUFFER *src,
                                    mve_com_buffer *dst,
                                    enum mve_com_buffer_type type)
{
    switch (type)
    {
        case MVE_COM_BUFFER_TYPE_FRAME:
            dst->frame.nHandle            = src->frame.nHandle;
            dst->frame.nFlags             = src->frame.nFlags;
            dst->frame.timestamp          = src->frame.nUserDataTag;
            dst->frame.nMVEFlags          = src->frame.nMVEFlags;
            dst->frame.pic_index          = src->frame.pic_index;
            dst->frame.decoded_height     = src->frame.decoded_height;
            dst->frame.decoded_width      = src->frame.decoded_width;
            memcpy(&dst->frame.data, &src->frame.data, sizeof(src->frame.data));
            dst->frame.crc_top = src->frame.crc_top;
            dst->frame.crc_bot = src->frame.crc_bot;
            break;
        case MVE_COM_BUFFER_TYPE_BITSTREAM:
            dst->bitstream.nHandle      = src->bitstream.nHandle;
            dst->bitstream.nFlags       = src->bitstream.nFlags;
            dst->bitstream.timestamp    = src->bitstream.nUserDataTag;
            dst->bitstream.nAllocLen    = src->bitstream.nAllocLen;
            dst->bitstream.nOffset      = src->bitstream.nOffset;
            dst->bitstream.nFilledLen   = src->bitstream.nFilledLen;
            dst->bitstream.pBufferData  = src->bitstream.pBufferData;
            break;
        default:
            WARN_ON(true); /* Should never end up here */
            break;
    }
}

/**
 * Waits until there is enough space available in the message in-queue to
 * add a message of a given size. If a static timeout expires, this function
 * returns false.
 * @param rpos       Pointer to read position in the queue.
 * @param wpos       Pointer to write position in the queue.
 * @param words      Number of data slot required in the message queue.
 * @param queue_size Size of the queue
 * @return MVE_ERROR_NONE if there is enough space available in the queue to add
 *         a message of the given size. MVE_ERROR_TIMEOUT if the timeout triggered.
 */
static mve_error wait_until_space_available(volatile uint16_t *rpos,
                                            volatile uint16_t *wpos,
                                            uint32_t words,
                                            uint32_t queue_size)
{
#define ITERATION_TIMEOUT 100
    int i = 0;
    mve_error ret = MVE_ERROR_NONE;
    uint32_t free_words;

    if (words > queue_size)
    {
        return MVE_ERROR_TIMEOUT;
    }

    do
    {
        if (*rpos <= *wpos)
        {
            free_words = queue_size - *wpos + *rpos;
        }
        else
        {
            free_words = *rpos - *wpos;
        }
        i++;
        mb();
    }
    while (free_words < (words + 1) && i < ITERATION_TIMEOUT);

    /* Did we timeout? */
    if (i >= ITERATION_TIMEOUT)
    {
        ret = MVE_ERROR_TIMEOUT;
    }

    return ret;
#undef ITERATION_TIMEOUT
}

/**
 * Write data the input queue represented by host_area.
 * @param host_area The host area of the queue.
 * @param mve_area  The MVE area of the queue.
 * @param header    Header of the message to add.
 * @param data      Data section of the message to add.
 * @param num_words Size of the data section in 32-bit words.
 * @return MVE_ERROR_NONE if the message was added to the queue. Error code otherwise.
 */
static mve_error write_to_queue(struct mve_comm_area_host *host_area,
                                struct mve_comm_area_mve *mve_area,
                                struct mve_msg_header *header,
                                uint32_t *data,
                                uint32_t num_words)
{
    mve_error ret;
    uint32_t i;
    uint16_t wpos;

    ret = wait_until_space_available(&mve_area->in_rpos,
                                     &host_area->in_wpos,
                                     1 + num_words,
                                     MVE_COMM_QUEUE_SIZE_IN_WORDS);
    if (MVE_ERROR_NONE == ret)
    {
        wpos = host_area->in_wpos;
        WRITE_QUEUE(host_area->in_data, struct mve_msg_header, wpos, *header);
        wpos = (wpos + 1) % MVE_COMM_QUEUE_SIZE_IN_WORDS;
        for (i = 0; i < num_words; ++i)
        {
            WRITE_QUEUE(host_area->in_data, uint32_t, wpos, data[i]);
            wpos = (wpos + 1) % MVE_COMM_QUEUE_SIZE_IN_WORDS;
        }

        mve_rsrc_mem_flush_write_buffer();
        host_area->in_wpos = wpos;
        mve_rsrc_mem_flush_write_buffer();
    }

    return ret;
}

static void read_from_queue(struct mve_comm_area_host *host_area,
                            struct mve_comm_area_mve *mve_area,
                            struct mve_msg_header *header,
                            uint32_t *dst)
{
    uint16_t rpos;
    uint32_t i, words;

    WARN_ON((unsigned short)0 >= (unsigned short)(mve_area->out_wpos - host_area->out_rpos));
    rpos = host_area->out_rpos;
    READ_QUEUE(mve_area->out_data, struct mve_msg_header, rpos, *header);
    rpos = (rpos + 1) % MVE_COMM_QUEUE_SIZE_IN_WORDS;
    words = (header->size + sizeof(uint32_t) - 1) / sizeof(uint32_t);

    for (i = 0; i < words; ++i)
    {
        READ_QUEUE(mve_area->out_data, uint32_t, rpos, dst[i]);
        rpos = (rpos + 1) % MVE_COMM_QUEUE_SIZE_IN_WORDS;
    }

    mve_rsrc_mem_flush_write_buffer();
    host_area->out_rpos = rpos;
    mve_rsrc_mem_flush_write_buffer();
}

static void log_header(struct iovec *vec,
                       unsigned count,
                       struct mve_session *session,
                       enum mve_log_fwif_channel channel,
                       enum mve_log_fwif_direction direction)
{
    struct mve_log_header header;
    struct mve_log_fwif fwif;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
    struct timespec64 timespec;
#else
    struct timespec timespec;
#endif
    unsigned i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
    ktime_get_real_ts64(&timespec);
#else
    getnstimeofday(&timespec);
#endif
    header.magic = MVE_LOG_MAGIC;
    header.length = 0;
    header.type = MVE_LOG_TYPE_FWIF;
    header.severity = MVE_LOG_INFO;
    header.timestamp.sec = timespec.tv_sec;
    header.timestamp.nsec = timespec.tv_nsec;

    fwif.version_minor = 0;
    fwif.version_major = 1;
    fwif.channel = channel;
    fwif.direction = direction;
    fwif.session = (uintptr_t)session;

    vec[0].iov_base = &header;
    vec[0].iov_len = sizeof(header);

    vec[1].iov_base = &fwif;
    vec[1].iov_len = sizeof(fwif);

    for (i = 1; i < count; ++i)
    {
        header.length += vec[i].iov_len;
    }

    MVE_LOG_DATA(&mve_rsrc_log_fwif, MVE_LOG_INFO, vec, count);
}

static void log_buffer(struct mve_session *session,
                       enum mve_log_fwif_channel channel,
                       enum mve_log_fwif_direction direction,
                       struct mve_msg_header *msg_header,
                       void *data,
                       unsigned queued)
{
    struct
    {
        struct mve_msg_header msg_header;
        struct mve_log_fwif_stat stat;
    }
    stat;
    struct iovec vec[5];

    stat.msg_header.code = MVE_LOG_FWIF_CODE_STAT;
    stat.msg_header.size = sizeof(stat.stat);
    stat.stat.handle = 0;
    stat.stat.queued = queued;

    vec[2].iov_base = msg_header;
    vec[2].iov_len = sizeof(*msg_header);

    vec[3].iov_base = data;
    vec[3].iov_len = ROUNDUP(msg_header->size, 4);

    vec[4].iov_base = &stat;
    vec[4].iov_len = ROUNDUP(sizeof(stat), 4);

    log_header(vec, 5, session, channel, direction);
}

static void log_message(struct mve_session *session,
                        enum mve_log_fwif_direction direction,
                        struct mve_msg_header *msg_header,
                        void *data)
{
    struct iovec vec[4];

    vec[2].iov_base = msg_header;
    vec[2].iov_len = sizeof(*msg_header);

    vec[3].iov_base = data;
    vec[3].iov_len = msg_header->size;

    log_header(vec, 4, session, MVE_LOG_FWIF_CHANNEL_MESSAGE, direction);
}

static mve_error add_message(struct mve_session *session,
                             uint16_t code,
                             uint16_t size,
                             uint32_t *data)
{
    mve_error ret = MVE_ERROR_NONE;

    struct mve_comm_area_host *host_area;
    struct mve_comm_area_mve *mve_area;

    if (0 < size && NULL == data)
    {
        return MVE_ERROR_BAD_PARAMETER;
    }

    host_area = mve_rsrc_dma_mem_map(session->msg_in_queue);
    mve_area = mve_rsrc_dma_mem_map(session->msg_out_queue);

    if (NULL == host_area || NULL == mve_area)
    {
        ret = MVE_ERROR_INSUFFICIENT_RESOURCES;
    }
    else
    {
        uint32_t words;
        struct mve_msg_header header;

        header.code = code;
        header.size = size;

        words = (size + sizeof(uint32_t) - 1) / sizeof(uint32_t);
        ret = write_to_queue(host_area, mve_area, &header, data, words);

        MVE_LOG_EXECUTE(&mve_rsrc_log_fwif, MVE_LOG_INFO,
                        log_message(session, MVE_LOG_FWIF_DIRECTION_HOST_TO_FIRMWARE, &header, data));
    }

    switch (code)
    {
        case MVE_MESSAGE_CODE_GO:
        case MVE_MESSAGE_CODE_STOP:
        case MVE_MESSAGE_CODE_SET_PARAMETER:
        case MVE_MESSAGE_CODE_GET_PARAMETER:
        case MVE_MESSAGE_CODE_SET_CONFIG:
        case MVE_MESSAGE_CODE_GET_CONFIG:
        case MVE_MESSAGE_CODE_INPUT_FLUSH:
        case MVE_MESSAGE_CODE_OUTPUT_FLUSH:
            session->pending_response_count++;
            break;
        default:
            break;
    }
    mve_rsrc_dma_mem_unmap(session->msg_out_queue);
    mve_rsrc_dma_mem_unmap(session->msg_in_queue);

    return ret;
}

static uint32_t *get_message(struct mve_session *session,
                             struct mve_msg_header *header)
{
    uint32_t *ret = NULL;
    uint32_t available_words;

    struct mve_comm_area_host *host_area;
    struct mve_comm_area_mve *mve_area;

    if (NULL == header)
    {
        return NULL;
    }

    host_area = mve_rsrc_dma_mem_map(session->msg_in_queue);
    mve_area = mve_rsrc_dma_mem_map(session->msg_out_queue);

    mve_rsrc_dma_mem_invalidate_cache(session->msg_out_queue);

    if (host_area->out_rpos <= mve_area->out_wpos)
    {
        available_words = mve_area->out_wpos - host_area->out_rpos;
    }
    else
    {
        available_words = MVE_COMM_QUEUE_SIZE_IN_WORDS + mve_area->out_wpos - host_area->out_rpos;
    }

    if (available_words > 0)
    {
        /* There is a message in the queue */
        uint16_t rpos;
        uint32_t words;

        rpos = host_area->out_rpos;

        *header = *((struct mve_msg_header *)&mve_area->out_data[rpos]);
        rpos = (rpos + 1) % MVE_COMM_QUEUE_SIZE_IN_WORDS;

        words = (header->size + sizeof(uint32_t) - 1) / sizeof(uint32_t);

        ret = MVE_RSRC_MEM_CACHE_ALLOC(words * sizeof(uint32_t), GFP_KERNEL);
        if (NULL == ret)
        {
            /* Failed to allocate temporary memory needed to process this
             * message. This message will be dropped! */
            MVE_LOG_PRINT_SESSION(&mve_rsrc_log_session, MVE_LOG_WARNING, session, "Message from MVE dropped due to out of memory conditions.");
            /* Update rpos to skip this message */
            rpos += words;
        }
        else
        {
            int i;

            /* Read the data associated with the message */
            for (i = 0; i < words; ++i)
            {
                ret[i] = mve_area->out_data[rpos];
                rpos = (rpos + 1) % MVE_COMM_QUEUE_SIZE_IN_WORDS;
            }

            mb();

            MVE_LOG_EXECUTE(&mve_rsrc_log_fwif, MVE_LOG_INFO,
                            log_message(session, MVE_LOG_FWIF_DIRECTION_FIRMWARE_TO_HOST, header, ret));
        }

        host_area->out_rpos = rpos;
        mb();

        switch (header->code)
        {
            case MVE_RESPONSE_CODE_STATE_CHANGE:
            case MVE_RESPONSE_CODE_GET_PARAMETER_REPLY:
            case MVE_RESPONSE_CODE_SET_PARAMETER_REPLY:
            case MVE_RESPONSE_CODE_GET_CONFIG_REPLY:
            case MVE_RESPONSE_CODE_SET_CONFIG_REPLY:
            case MVE_RESPONSE_CODE_INPUT_FLUSHED:
            case MVE_RESPONSE_CODE_OUTPUT_FLUSHED:
                session->pending_response_count--;
                break;
            default:
                break;
        }

        mve_rsrc_dma_mem_clean_cache(session->msg_in_queue);
        mve_rsrc_dma_mem_invalidate_cache(session->msg_out_queue);
    }

    mve_rsrc_dma_mem_unmap(session->msg_out_queue);
    mve_rsrc_dma_mem_unmap(session->msg_in_queue);

    return ret;
}

/**
 * Send a buffer message to the FW.
 */
static mve_error send_buffer_msg(struct mve_session *session,
                                 mve_com_buffer *buffer,
                                 enum mve_com_buffer_type type,
                                 struct mve_comm_area_host *host_area,
                                 struct mve_comm_area_mve *mve_area,
                                 union MVE_BUFFER *mve_buffer,
                                 struct mve_msg_header *header)
{
    mve_error ret = MVE_ERROR_NONE;

    switch (type)
    {
        case MVE_COM_BUFFER_TYPE_BITSTREAM:
            header->code = MVE_MSG_HEADER_CODE_BUFFER_BITSTREAM;
            header->size = sizeof(struct MVE_BUFFER_BITSTREAM);
            break;

        case MVE_COM_BUFFER_TYPE_FRAME:
            header->code = MVE_MSG_HEADER_CODE_BUFFER_FRAME;
            header->size = sizeof(struct MVE_BUFFER_FRAME);
            break;

        case MVE_COM_BUFFER_TYPE_ROI:
            header->code = MVE_MSG_HEADER_CODE_BUFFER_REGION;
            header->size = sizeof(struct MVE_BUFFER_REGION);
            break;

        default:
            MVE_LOG_PRINT(&mve_rsrc_log, MVE_LOG_ERROR, "Unknown type (%d).", type);
            ret = MVE_ERROR_BAD_PARAMETER;
    }

    if (MVE_ERROR_NONE == ret)
    {
        uint32_t words;
        uint32_t *data;

        convert_to_mve_buffer(session, buffer, mve_buffer, type);

        words = (header->size + sizeof(uint32_t) - 1) / sizeof(uint32_t);
        data = (uint32_t *)mve_buffer;
        ret = write_to_queue(host_area, mve_area, header, data, words);
    }

    return ret;
}

static mve_error add_input_buffer(struct mve_session *session,
                                  mve_com_buffer *buffer,
                                  enum mve_com_buffer_type type)
{
    mve_error ret = MVE_ERROR_NONE;

    struct mve_comm_area_host *host_area;
    struct mve_comm_area_mve *mve_area;

    host_area = mve_rsrc_dma_mem_map(session->buf_input_in);
    mve_area = mve_rsrc_dma_mem_map(session->buf_input_out);

    if (NULL == host_area || NULL == mve_area || NULL == buffer)
    {
        ret = MVE_ERROR_BAD_PARAMETER;
    }
    else
    {
        union MVE_BUFFER mve_buffer;
        struct mve_msg_header header;

        ret = send_buffer_msg(session, buffer, type, host_area, mve_area, &mve_buffer, &header);

        if (MVE_ERROR_NONE == ret)
        {
            MVE_LOG_EXECUTE(&mve_rsrc_log_fwif, MVE_LOG_INFO,
                            log_buffer(session,
                                       MVE_LOG_FWIF_CHANNEL_INPUT_BUFFER,
                                       MVE_LOG_FWIF_DIRECTION_HOST_TO_FIRMWARE,
                                       &header,
                                       &mve_buffer,
                                       session->input_buffer_count + 1));
        }
    }

    mve_rsrc_dma_mem_unmap(session->buf_input_in);
    mve_rsrc_dma_mem_unmap(session->buf_input_out);

    return ret;
}

static mve_error add_output_buffer(struct mve_session *session,
                                   mve_com_buffer *buffer,
                                   enum mve_com_buffer_type type)
{
    mve_error ret = MVE_ERROR_NONE;

    struct mve_comm_area_host *host_area;
    struct mve_comm_area_mve *mve_area;

    host_area = mve_rsrc_dma_mem_map(session->buf_output_in);
    mve_area = mve_rsrc_dma_mem_map(session->buf_output_out);

    if (NULL == host_area || NULL == mve_area || NULL == buffer)
    {
        ret = MVE_ERROR_BAD_PARAMETER;
    }
    else
    {
        union MVE_BUFFER mve_buffer;
        struct mve_msg_header header;

        ret = send_buffer_msg(session, buffer, type, host_area, mve_area, &mve_buffer, &header);

        if (MVE_ERROR_NONE == ret)
        {
            MVE_LOG_EXECUTE(&mve_rsrc_log_fwif, MVE_LOG_INFO,
                            log_buffer(session,
                                       MVE_LOG_FWIF_CHANNEL_OUTPUT_BUFFER,
                                       MVE_LOG_FWIF_DIRECTION_HOST_TO_FIRMWARE,
                                       &header,
                                       &mve_buffer,
                                       session->output_buffer_count + 1));
        }
    }

    mve_rsrc_dma_mem_unmap(session->buf_output_in);
    mve_rsrc_dma_mem_unmap(session->buf_output_out);

    return ret;
}

static mve_error get_input_buffer(struct mve_session *session,
                                  mve_com_buffer *buffer)
{
    mve_error ret = MVE_ERROR_BAD_PARAMETER;
    struct mve_comm_area_host *host_area;
    struct mve_comm_area_mve *mve_area;

    host_area = mve_rsrc_dma_mem_map(session->buf_input_in);
    mve_area = mve_rsrc_dma_mem_map(session->buf_input_out);
    if (NULL != host_area && NULL != mve_area)
    {
        union MVE_BUFFER mve_buffer;
        struct mve_msg_header header;
        enum mve_com_buffer_type type;

        mve_rsrc_dma_mem_invalidate_cache(session->buf_input_out);
        read_from_queue(host_area, mve_area, &header, (uint32_t *)&mve_buffer);
        mve_rsrc_dma_mem_clean_cache(session->buf_input_in);

        type = (MVE_SESSION_TYPE_DECODER == session->session_type) ?
               MVE_COM_BUFFER_TYPE_BITSTREAM :
               MVE_COM_BUFFER_TYPE_FRAME;
        convert_from_mve_buffer(session, &mve_buffer, buffer, type);
        ret = MVE_ERROR_NONE;

        MVE_LOG_EXECUTE(&mve_rsrc_log_fwif, MVE_LOG_INFO,
                        log_buffer(session, MVE_LOG_FWIF_CHANNEL_INPUT_BUFFER, MVE_LOG_FWIF_DIRECTION_FIRMWARE_TO_HOST, &header, &mve_buffer, session->input_buffer_count - 1));
    }

    mve_rsrc_dma_mem_unmap(session->buf_input_in);
    mve_rsrc_dma_mem_unmap(session->buf_input_out);

    return ret;
}

static mve_error get_output_buffer(struct mve_session *session,
                                   mve_com_buffer *buffer)
{
    mve_error ret = MVE_ERROR_BAD_PARAMETER;
    struct mve_comm_area_host *host_area;
    struct mve_comm_area_mve *mve_area;

    host_area = mve_rsrc_dma_mem_map(session->buf_output_in);
    mve_area = mve_rsrc_dma_mem_map(session->buf_output_out);
    if (NULL != host_area && NULL != mve_area)
    {
        union MVE_BUFFER mve_buffer;
        struct mve_msg_header header;
        enum mve_com_buffer_type type;

        mve_rsrc_dma_mem_invalidate_cache(session->buf_output_out);
        read_from_queue(host_area, mve_area, &header, (uint32_t *)&mve_buffer);
        mve_rsrc_dma_mem_clean_cache(session->buf_output_in);

        type = (MVE_SESSION_TYPE_DECODER == session->session_type) ?
               MVE_COM_BUFFER_TYPE_FRAME :
               MVE_COM_BUFFER_TYPE_BITSTREAM;
        convert_from_mve_buffer(session, &mve_buffer, buffer, type);
        ret = MVE_ERROR_NONE;

        MVE_LOG_EXECUTE(&mve_rsrc_log_fwif, MVE_LOG_INFO,
                        log_buffer(session, MVE_LOG_FWIF_CHANNEL_OUTPUT_BUFFER, MVE_LOG_FWIF_DIRECTION_FIRMWARE_TO_HOST, &header, &mve_buffer, session->output_buffer_count - 1));
    }

    mve_rsrc_dma_mem_unmap(session->buf_output_in);
    mve_rsrc_dma_mem_unmap(session->buf_output_out);

    return ret;
}

static mve_error get_rpc_message(struct mve_session *session,
                                 mve_com_rpc *rpc)
{
    struct mve_rpc_comunication_area *rpc_area;
    mve_error err = MVE_ERROR_NOT_READY;

    rpc_area = mve_rsrc_dma_mem_map(session->rpc_area);
    if (NULL == rpc_area)
    {
        return MVE_ERROR_INSUFFICIENT_RESOURCES;
    }

    mve_rsrc_dma_mem_invalidate_cache(session->rpc_area);

    if (MVE_RPC_STATE_PARAM == rpc_area->state)
    {
        /* Copy RPC details to the client supplied structure */
        rpc->state   = rpc_area->state;
        rpc->call_id = rpc_area->call_id;
        rpc->size    = rpc_area->size;

        switch (rpc_area->call_id)
        {
            case MVE_RPC_FUNCTION_DEBUG_PRINTF:
                memcpy(rpc->params.debug_print.string,
                       rpc_area->params.debug_print.string,
                       MVE_RPC_DATA_SIZE_IN_WORDS * 4);
                break;
            case MVE_RPC_FUNCTION_MEM_ALLOC:
                rpc->params.mem_alloc.size           = rpc_area->params.mem_alloc.size;
                rpc->params.mem_alloc.max_size       = rpc_area->params.mem_alloc.max_size;
                rpc->params.mem_alloc.region         = rpc_area->params.mem_alloc.region;
                rpc->params.mem_alloc.log2_alignment = MVE_MMU_PAGE_SHIFT;
                break;
            case MVE_RPC_FUNCTION_MEM_RESIZE:
                rpc->params.mem_resize.ve_pointer = rpc_area->params.mem_resize.ve_pointer;
                rpc->params.mem_resize.new_size   = rpc_area->params.mem_resize.new_size;
                break;
            case MVE_RPC_FUNCTION_MEM_FREE:
                rpc->params.mem_free.ve_pointer = rpc_area->params.mem_free.ve_pointer;
                break;
        }

        err = MVE_ERROR_NONE;
    }

    mve_rsrc_dma_mem_unmap(session->rpc_area);

    return err;
}

static mve_error put_rpc_message(struct mve_session *session,
                                 mve_com_rpc *rpc)
{
    struct mve_rpc_comunication_area *rpc_area;

    rpc_area = mve_rsrc_dma_mem_map(session->rpc_area);
    if (NULL == rpc_area)
    {
        return MVE_ERROR_INSUFFICIENT_RESOURCES;
    }

    /* Copy RPC details to the client supplied structure */
    switch (rpc->call_id)
    {
        case MVE_RPC_FUNCTION_DEBUG_PRINTF:
            break;
        case MVE_RPC_FUNCTION_MEM_ALLOC:
            rpc_area->params.data[0] = rpc->params.data[0];
            break;
        case MVE_RPC_FUNCTION_MEM_RESIZE:
            rpc_area->params.data[0] = rpc->params.data[0];
            break;
        case MVE_RPC_FUNCTION_MEM_FREE:
            break;
    }

    rpc_area->call_id = rpc->call_id;
    rpc_area->size    = rpc->size;
    wmb();
    rpc_area->state   = rpc->state;

    wmb();
    mve_rsrc_dma_mem_clean_cache(session->rpc_area);
    mve_rsrc_dma_mem_unmap(session->rpc_area);

    return MVE_ERROR_NONE;
}

static void mve_com_host_interface_v1_construct(struct mve_com *com)
{
    memset(com, 0, sizeof(*com));

    com->host_interface.add_message = add_message;
    com->host_interface.get_message = get_message;
    com->host_interface.add_input_buffer = add_input_buffer;
    com->host_interface.add_output_buffer = add_output_buffer;
    com->host_interface.get_input_buffer = get_input_buffer;
    com->host_interface.get_output_buffer = get_output_buffer;
    com->host_interface.get_rpc_message = get_rpc_message;
    com->host_interface.put_rpc_message = put_rpc_message;
}

struct mve_com *mve_com_host_interface_v1_new(void)
{
    struct mve_com *com;

    /* Allocate com object. */
    com = MVE_RSRC_MEM_ZALLOC(sizeof(*com), GFP_KERNEL);
    if (com == NULL)
    {
        MVE_LOG_PRINT(&mve_rsrc_log, MVE_LOG_WARNING, "Failed to allocate com object.");
        return NULL;
    }

    /* Run constructor. */
    mve_com_host_interface_v1_construct(com);

    return com;
}
