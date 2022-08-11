/*!
 * bst_lwnn: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author  Shichen Lin (shichen.lin@bst.ai)
 *
 * @file    bst_lwnn_mem_manager.h
 * @brief   This file is the header file of the memory manager part of the bst_lwnn
 *          driver. It contains structure definitions, function declarations and
 *          macros related to the memory and address management.
 */

#ifndef BST_LWNN_MEM_MANAGER_H
#define BST_LWNN_MEM_MANAGER_H

/*******************************************************************************
 * Memory Address Related Convertion Macros
 ******************************************************************************/
#define addr_truncate(bst_lwnn_addr)            ((dsp_ptr)((unsigned long)(bst_lwnn_addr) & 0xFFFFFFFF))
#define addr_extend(bst_lwnn_addr)              ((phys_addr_t)(bst_lwnn_addr))
#define phys_to_bus(pbst_lwnn, phys_addr)       addr_truncate(((phys_addr_t)(phys_addr)) - (pbst_lwnn)->mem_manager.phys_to_bus_offset)
#define bus_to_phys(pbst_lwnn, bus_addr)        addr_extend(((phys_addr_t)(bus_addr)) + (pbst_lwnn)->mem_manager.phys_to_bus_offset)
#define bus_to_kern(pbst_lwnn, bus_addr, blk)   (blk->kern_addr + ((dsp_ptr)(bus_addr) - phys_to_bus(pbst_lwnn, (blk)->phys_addr)))
#define kern_to_bus(pbst_lwnn, kaddr, blk)      (phys_to_bus(pbst_lwnn, (blk)->phys_addr + (phys_addr_t)((void *)(kaddr) - (blk)->kern_addr)))

struct bst_lwnn_memblock {
    struct bst_lwnn *pbst_lwnn;
    phys_addr_t phys_addr;
    void *kern_addr;
    uint32_t size;
};

//the structure to represent a physically continuous buffer allocated for userspace
struct bst_lwnn_buffer {
    struct bst_lwnn_memblock *block;
    uint64_t user_addr;
    dsp_ptr bus_addr;
    struct hlist_node node;
};

//the structure to manage the buffers allocated for the same process in userspace
struct bst_lwnn_mem_ctx {
    struct file *filp;
    DECLARE_HASHTABLE(ht, 12);
    struct list_head link;
};

struct bst_lwnn_mem_ops {
    struct bst_lwnn_memblock *(*alloc)(struct bst_lwnn *pbst_lwnn, uint32_t size, uint32_t align);
    void (*free)(struct bst_lwnn_memblock *memblock);
};

struct bst_lwnn_dma_buf_attachment {
    struct dma_buf_attachment *attach;
    struct sg_table *sgt;
    struct hlist_node node;
};

struct bst_lwnn_mem_manager {
    phys_addr_t phys_to_bus_offset;

    struct bst_lwnn_mem_ops *ops;

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

int bst_lwnn_user_buffer_alloc(struct file *filp, struct bst_lwnn *pbst_lwnn, struct bst_lwnn_user_buffer *pbuffer);
int bst_lwnn_user_buffer_free(struct file *filp, struct bst_lwnn *pbst_lwnn, struct bst_lwnn_user_buffer *pbuffer);
int bst_lwnn_user_buffer_sync(struct file *filp, struct bst_lwnn *pbst_lwnn, struct bst_lwnn_user_buffer *pbuffer);
int bst_lwnn_mem_ctx_add(struct bst_lwnn* pbst_lwnn, struct file *filp);
int bst_lwnn_mem_ctx_remove(struct bst_lwnn *pbst_lwnn, struct file *filp);
int bst_lwnn_mem_manager_init(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_mem_manager_exit(struct bst_lwnn *pbst_lwnn);
int bst_lwnn_dma_buf_import(struct bst_lwnn *pbst_lwnn, struct bst_lwnn_dma_buf *buf);
int bst_lwnn_dma_buf_return(struct bst_lwnn *pbst_lwnn, struct bst_lwnn_dma_buf *buf);

#endif