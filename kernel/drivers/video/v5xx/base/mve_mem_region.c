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
#include <asm/bug.h>
#endif

#include "mve_mem_region.h"
#include "mve_fw.h"

static struct mve_mem_virt_region mem_regions_prot_v1[] =
{
    {   /* VIRT_MEM_REGION_FIRMWARE0 */
        0x00000000,
        0x10000000
    },
    {   /* VIRT_MEM_REGION_MSG_IN_QUEUE */
        0x10079000,
        0x1007A000
    },
    {   /* VIRT_MEM_REGION_MSG_OUT_QUEUE */
        0x1007A000,
        0x1007B000
    },
    {   /* VIRT_MEM_REGION_INPUT_BUFFER_IN */
        0x1007B000,
        0x1007C000
    },
    {   /* VIRT_MEM_REGION_INPUT_BUFFER_OUT */
        0x1007C000,
        0x1007D000
    },
    {   /* VIRT_MEM_REGION_OUTPUT_BUFFER_IN */
        0x1007D000,
        0x1007E000
    },
    {   /* VIRT_MEM_REGION_OUTPUT_BUFFER_OUT */
        0x1007E000,
        0x1007F000
    },
    {   /* VIRT_MEM_REGION_RPC_QUEUE */
        0x1007F000,
        0x10080000
    },
    {   /* VIRT_MEM_REGION_PROTECTED */
        0x20000000,
        0x50000000,
    },
    {   /* VIRT_MEM_REGION_OUT_BUF */
        0x50000000,
        0x80000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE1 */
        0x80000000,
        0x90000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE2 */
        0x90000000,
        0xA0000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE3 */
        0xA0000000,
        0xB0000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE4 */
        0xB0000000,
        0xC0000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE5 */
        0xC0000000,
        0xD0000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE6 */
        0xD0000000,
        0xE0000000,
    },
    {   /* VIRT_MEM_REGION_FIRMWARE7 */
        0xE0000000,
        0xF0000000,
    },
    {   /* VIRT_MEM_REGION_REGS */
        0xF0000000,
        0xFFFFFFFF,
    }
};

void mve_mem_virt_region_get(int fw_prot,
                             enum mve_mem_virt_region_type type,
                             struct mve_mem_virt_region *region)
{
    if (NULL != region)
    {
        /* Apparently gcc implemented the enum as an unsigned, and disliked
         * what it considers to be an always-true comparision */
        if (/*VIRT_MEM_REGION_FIRMWARE0 <= type &&*/ VIRT_MEM_REGION_REGS >= type)
        {
            *region = mem_regions_prot_v1[type];
        }
        else
        {
            WARN_ON(true);
            region->start = 0xFFFFFFFF;
            region->end = 0xFFFFFFFF;
        }
    }
}
