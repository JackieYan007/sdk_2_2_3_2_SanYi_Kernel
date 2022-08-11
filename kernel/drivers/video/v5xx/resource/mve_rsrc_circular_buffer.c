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
#include <linux/slab.h>
#endif

#include "mve_rsrc_circular_buffer.h"
#include "mve_rsrc_mem_frontend.h"

struct mver_circular_buffer *mver_circular_buffer_create(uint32_t size)
{
    struct mver_circular_buffer *cb;

    cb = (struct mver_circular_buffer *)
         MVE_RSRC_MEM_ZALLOC(sizeof(struct mver_circular_buffer), GFP_KERNEL);
    if (NULL == cb)
    {
        return NULL;
    }

    cb->data = (void **)MVE_RSRC_MEM_VALLOC(sizeof(void *) * size);
    if (NULL == cb->data)
    {
        MVE_RSRC_MEM_FREE(cb);
        return NULL;
    }

    cb->size = size;

    return cb;
}

void mver_circular_buffer_destroy(struct mver_circular_buffer *cb)
{
    if (NULL != cb)
    {
        MVE_RSRC_MEM_VFREE(cb->data);
        MVE_RSRC_MEM_FREE(cb);
    }
}

bool mver_circular_buffer_add(struct mver_circular_buffer *cb, void *item)
{
    bool ret = false;

    if ((uint32_t)(cb->w_pos - cb->r_pos) < (uint32_t)cb->size)
    {
        cb->data[cb->w_pos++ % cb->size] = item;
        ret = true;
    }

    return ret;
}

bool mver_circular_buffer_remove(struct mver_circular_buffer *cb, void **data)
{
    bool ret = false;

    if ((uint32_t)(cb->w_pos - cb->r_pos) > (uint32_t)0)
    {
        *data = cb->data[cb->r_pos++ % cb->size];
        ret = true;
    }

    return ret;
}

uint32_t mver_circular_buffer_get_num_entries(struct mver_circular_buffer *cb)
{
    return cb->w_pos - cb->r_pos;
}

bool mver_circular_buffer_peek(struct mver_circular_buffer *cb, void **data)
{
    bool ret = false;

    if ((uint32_t)(cb->w_pos - cb->r_pos) > (uint32_t)0)
    {
        *data = cb->data[cb->r_pos % cb->size];
        ret = true;
    }
    else
    {
        *data = NULL;
    }

    return ret;
}

void mver_circular_buffer_remove_all_occurences(struct mver_circular_buffer *cb, void *data)
{
    uint32_t start, end, curr;

    start = cb->r_pos;
    end = cb->w_pos;

    for (curr = start; (uint32_t)(end - curr) > (uint32_t)0; ++curr)
    {
        if (data == cb->data[curr % cb->size])
        {
            uint32_t i;
            for (i = curr; (uint32_t)(i - start) > (uint32_t)0; --i)
            {
                cb->data[i % cb->size] = cb->data[(i - 1) % cb->size];
            }
            start++;
        }
    }

    cb->r_pos = start;
}
