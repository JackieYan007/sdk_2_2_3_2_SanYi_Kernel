/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author Wenjian Gou (wenjian.gou@bst.ai)
 *
 * @file    bst_cv_mem_manager.h
 * @brief   This file is the header file of the memory manager part of the bst_cv
 *          driver. It contains structure definitions, function declarations and
 *          macros related to the memory and address management.
 */

#ifndef BST_CV_MEM_MANAGER_H
#define BST_CV_MEM_MANAGER_H

/*******************************************************************************
 * Memory Address Related Convertion Macros
 ******************************************************************************/
#define addr_truncate(bst_cv_addr)            (dsp_ptr)((unsigned long)(bst_cv_addr) & 0xFFFFFFFF)
#define addr_extend(bst_cv_addr)              (phys_addr_t)((unsigned long)(bst_cv_addr))
#define phys_to_bus(pbst_cv, phys_addr)       ((phys_addr_t)(phys_addr) - ((pbst_cv)->mem_manager.phys_to_bus_offset))
#define bus_to_phys(pbst_cv, bus_addr)        ((phys_addr_t)(bus_addr) + ((pbst_cv)->mem_manager.phys_to_bus_offset))
#define kern_to_phys(pbst_cv, kern_addr)      ((phys_addr_t)(kern_addr) - ((pbst_cv)->mem_manager.kern_sub_phys_offset))
#define phys_to_kern(pbst_cv, phys_addr)      ((phys_addr_t)(phys_addr) + ((pbst_cv)->mem_manager.kern_sub_phys_offset))
#define kern_to_bus(pbst_cv, vaddr)           addr_truncate(phys_to_bus(pbst_cv, kern_to_phys(pbst_cv, vaddr)))
#define bus_to_kern(pbst_cv, paddr)           ((void *)(phys_to_kern(pbst_cv, bus_to_phys(pbst_cv, addr_extend(paddr)))))

struct bst_cv_memblock {
    struct bst_cv *pbst_cv;
    phys_addr_t phys_addr;
    void *kern_addr;
    uint32_t size;
};

//the structure to represent a physically continuous buffer allocated for userspace
struct bst_cv_buffer {
    struct bst_cv_memblock *block;
    uint64_t user_addr;
    dsp_ptr bus_addr;
    struct hlist_node node;
};

//the structure to manage the buffers allocated for the same process in userspace
struct bst_cv_mem_ctx {
    struct file *filp;
    DECLARE_HASHTABLE(ht, 12);
    struct list_head link;
};

struct bst_cv_mem_ops {
    struct bst_cv_memblock *(*alloc)(struct bst_cv *pbst_cv, uint32_t size, uint32_t align);
    void (*free)(struct bst_cv_memblock *memblock);
};

struct bst_cv_mem_manager {
    phys_addr_t phys_to_bus_offset;
    phys_addr_t kern_sub_phys_offset;
    //since physical memory is mapped to high-end kernel memory, the offset is always positive

    struct bst_cv_mem_ops *ops;

    /*
        We use mutex to lock any operation on process contexts in order to deal
        with unexpected execution of ioctl syscalls at the same time as close
        syscall.
    */
    struct mutex mm_mutex;
    struct mutex mm_alloc_free_mutex;
    struct list_head mem_ctx_list;
};

int bare_cv_user_buffer_alloc(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc *alloc);
int bare_cv_user_buffer_free(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc *alloc);
int bare_cv_mem_ctx_add(struct bst_cv* pbst_cv, struct file *filp);
int bare_cv_mem_ctx_remove(struct bst_cv *pbst_cv, struct file *filp);
int bare_cv_mem_manager_init(struct bst_cv *pbst_cv);
void bare_cv_mem_manager_exit(struct bst_cv *pbst_cv);

#endif