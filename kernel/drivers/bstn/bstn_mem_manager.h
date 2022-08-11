/*
 * BSTN: Linux device driver for Blck Sesame Technologies Neural Network IP
 * Last editor: Shichen Lin (shichen.lin@bst.ai)
 * Previous author: Yuchao Wang
 *
 * @file    bstn_mem_manager.h
 * @brief   This file is the header file of the memory manager part of the BSTN
 *          driver. It contains structure definitions, function declarations and
 *          macros related to the memory and address management.
 */

#ifndef BSTN_MEM_MANAGER_H
#define BSTN_MEM_MANAGER_H

/*******************************************************************************
 * Memory Address Related Convertion Macros
 ******************************************************************************/
#define addr_truncate(bstn_addr)            (dsp_ptr)((unsigned long)(bstn_addr) & 0xFFFFFFFF)
#define phys_to_bus(pbstn, paddr)           addr_truncate(((phys_addr_t)(paddr) - ((pbstn)->mem_manager.phys_to_bus_offset)))
#define bus_to_phys(pbstn, baddr)           ((phys_addr_t)(baddr) + ((pbstn)->mem_manager.phys_to_bus_offset))
#define bus_to_kern(pbstn, baddr, blk)      (blk->kern_addr + ((dsp_ptr)(baddr) - phys_to_bus(pbstn, (blk)->phys_addr)))
#define kern_to_bus(pbstn, kaddr, blk)      (phys_to_bus(pbstn, (blk)->phys_addr + (phys_addr_t)((void *)(kaddr) - (blk)->kern_addr)))

struct bstn_memblock {
    struct bstn_device *pbstn;
    phys_addr_t phys_addr;
    void *kern_addr;
    uint32_t size;
};

//the structure to represent a physically continuous buffer allocated for userspace
struct bstn_buffer {
    struct bstn_memblock *block;
    uint64_t user_addr;
    dsp_ptr bus_addr;
    struct hlist_node node;
};

//the structure to manage the buffers allocated for the same process in userspace
struct bstn_mem_ctx {
    struct file *filp;
    DECLARE_HASHTABLE(ht, 12);
    struct list_head link;
};

struct bstn_mem_ops {
    struct bstn_memblock *(*alloc)(struct bstn_device *pbstn, uint32_t size, uint32_t align);
    void (*free)(struct bstn_memblock *memblock);
};

struct bstn_dma_buf_attachment {
    struct dma_buf_attachment *attachment;
    struct sg_table *sgt;
    struct hlist_node node;
};

struct bstn_mem_manager {
    phys_addr_t phys_to_bus_offset;
    phys_addr_t rmem_base;
    phys_addr_t rmem_size;

    const struct bstn_mem_ops *ops;
    /*
        We use mutex to lock any operation on process contexts in order to deal
        with unexpected execution of ioctl syscalls at the same time as close
        syscall.
    */
    struct mutex mm_mutex;
    struct list_head mem_ctx_list;

    struct mutex dma_buf_mutex;
    DECLARE_HASHTABLE(dma_buf_ht, 10);
};

int bsnn_buffer_alloc(struct file *filp, struct bstn_device *pbstn, struct bsnn_buffer *pbuffer);
int bsnn_buffer_free(struct file *filp, struct bstn_device *pbstn, struct bsnn_buffer *pbuffer);
int bstn_mem_ctx_add(struct bstn_device *pbstn, struct file *filp);
int bstn_mem_ctx_remove(struct bstn_device *pbstn, struct file *filp);
int bstn_mem_manager_init(struct bstn_device *pbstn);
void bstn_mem_manager_exit(struct bstn_device *pbstn);
int bstn_dma_buf_import(struct bstn_device *pbstn, struct bstn_dma_buf *buf);
int bstn_dma_buf_return(struct bstn_device *pbstn, struct bstn_dma_buf *buf);

#endif

