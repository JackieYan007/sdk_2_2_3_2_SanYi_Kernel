/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * @author Wenjian Gou (wenjian.gou@bst.ai)
 *
 * @file    bst_cv_mem_manager.c
 * @brief   This file is the source code file of the memory manager of the
 *          bst_cv driver. It contains definitions of actual memory allocation
 *          functions as well as the initialization and exit functions of the
 *          memory manager.
 * @note    Because the current buffer and memory structures have not been
 *          finalized yet, further implementation for reliability like garbage
 *          collection is not completed.
 */

#include "bst_cv.h"
#include <linux/mutex.h>
#include <uapi/linux/mman.h>

static struct mutex cvmem_mutex;
struct dma_coherent_mem {
    void        *virt_base;
    dma_addr_t  device_base;
    unsigned long   pfn_base;
    int     size;
    int     flags;
    unsigned long   *bitmap;
    spinlock_t  spinlock;
    bool        use_dev_dma_pfn_offset;
};

static struct bst_cv_memblock *bst_cv_cma_alloc(struct bst_cv *pbst_cv, uint32_t size, uint32_t align);
static void bst_cv_cma_free(struct bst_cv_memblock *block);

static struct bst_cv_mem_ops bst_cv_cma_mem_ops = {
    .alloc = bst_cv_cma_alloc,
    .free = bst_cv_cma_free
};

/*
 * @func    bst_cv_cma_alloc
 * @brief   This function allocates a requested continuous memory block.
 * @params  mem_manager - the pointer to the memory mem_manager
 *          size - the requested size
 *          align - the requested alignment (TBD)
 * @return  the pointer to the memory block - success
 *          NULL - failure
 */
static struct bst_cv_memblock *bst_cv_cma_alloc(struct bst_cv *pbst_cv, uint32_t size, uint32_t align)
{
    void *kern_addr;
    dma_addr_t dma_addr;
    struct bst_cv_memblock *block;

    if (size == 0)
        return NULL;

    size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
    block = devm_kzalloc(&pbst_cv->pdev->dev, sizeof(*block), GFP_KERNEL);
    if (block == NULL)
        return NULL;

    kern_addr = dma_alloc_coherent(&pbst_cv->pdev->dev, size, &dma_addr, GFP_KERNEL);
    if (kern_addr == NULL) {
        devm_kfree(&pbst_cv->pdev->dev, block);
        return NULL;
    }

    block->pbst_cv = pbst_cv;
    block->phys_addr = dma_to_phys(&pbst_cv->pdev->dev, dma_addr);
    BST_CV_STAGE_PRINTK("phys_addr = 0x%lx", block->phys_addr );
    block->size = size;
    block->kern_addr = kern_addr;
    return block;
}

/*
 * @func    bst_cv_cma_free
 * @brief   This function frees the target continuous memory block.
 * @params  block - the pointer to the memory block
 * @return  void
 */
static void bst_cv_cma_free(struct bst_cv_memblock *block)
{
    struct bst_cv *pbst_cv = block->pbst_cv;

    if (block == NULL)
        return;

    dma_free_coherent(&pbst_cv->pdev->dev,
        block->size,
        block->kern_addr,
        phys_to_dma(&pbst_cv->pdev->dev, block->phys_addr));
    devm_kfree(&pbst_cv->pdev->dev, block);
    return;
}

static struct bst_cv_mem_ctx *_find_mem_ctx(struct bst_cv *pbst_cv, struct file *filp)
{
    struct bst_cv_mem_ctx *ctx;
    struct list_head *cur;
    
    list_for_each(cur, &pbst_cv->mem_manager.mem_ctx_list) {
        ctx = container_of(cur, struct bst_cv_mem_ctx, link);
        if (ctx->filp == filp) {
            return ctx;
        }
    }
    return NULL;
}

/*
 * @func    bare_cv_user_buffer_alloc
 * @brief   This function allocates a requested memory chunk to be used as a
 *          user buffer.
 * @params  filp - the misc device file pointer
 *          pbst_cv - the pointer to the bst_cv device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_user_buffer_alloc(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc *alloc)
{
    int ret;
    struct bst_cv_buffer *buffer;
    struct bst_cv_mem_ctx *ctx;

    buffer = devm_kzalloc(&pbst_cv->pdev->dev, sizeof(*buffer), GFP_KERNEL);
    if (buffer == NULL) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "kmalloc failed!");
        ret = -ENOMEM;
        goto fail_before_struct_alloc;
    }

    //allocate the buffer
    mutex_lock(&pbst_cv->mem_manager.mm_alloc_free_mutex );
    buffer->block = pbst_cv->mem_manager.ops->alloc(pbst_cv, alloc->size, alloc->align);
    mutex_unlock(&pbst_cv->mem_manager.mm_alloc_free_mutex );
    if (buffer->block == NULL) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "alloc failed!");
        ret = -ENOMEM;
        goto fail_before_buffer_alloc;
    }

    //map the buffer into userspace
    buffer->user_addr = vm_mmap(filp, 0, buffer->block->size, PROT_READ | PROT_WRITE, MAP_SHARED,
        buffer->block->phys_addr);
    if (!buffer->user_addr || IS_ERR_VALUE(buffer->user_addr)) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "vm_mmap failed! ret: %lld", buffer->user_addr);
        ret = -EFAULT;
        goto fail_after_buffer_alloc;
    }
    buffer->bus_addr = buffer->block->phys_addr; //kern_to_bus(pbst_cv, buffer->block->kern_addr);
    alloc->addr = buffer->bus_addr;
    alloc->kaddr = buffer->block->kern_addr;
    alloc->ptr = buffer->user_addr;
    alloc->align = alloc->align > PAGE_SIZE ? alloc->align : PAGE_SIZE;

    BST_CV_TRACE_PRINTK("addr: %x, size: %x", alloc->addr, alloc->size);
    BST_CV_TRACE_PRINTK("buffer: %px", buffer);

    mutex_lock(&pbst_cv->mem_manager.mm_mutex);
    ctx = _find_mem_ctx(pbst_cv, filp);
    if (ctx == NULL) {
        mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px not found", filp);
        ret = -ENOENT;
        goto fail_after_buffer_alloc;
    }
    //add the buffer into the hash table
    hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
    mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
    return 0;

fail_after_buffer_alloc:
    pbst_cv->mem_manager.ops->free(buffer->block);
fail_before_buffer_alloc:
    devm_kfree(&pbst_cv->pdev->dev, buffer);
fail_before_struct_alloc:
    return ret;
}

/*
 * @func    bare_cv_user_buffer_alloc
 * @brief   This function frees a user buffer.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_user_buffer_free(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc *alloc)
{
    struct bst_cv_buffer *buffer;
    struct bst_cv_mem_ctx *ctx;
    struct hlist_node *tmp;
    int ret = -ENOENT;

    mutex_lock(&pbst_cv->mem_manager.mm_mutex);
    ctx = _find_mem_ctx(pbst_cv, filp);
    if (ctx == NULL) {
        mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px not found", filp);
        return -ENOENT;
    }

    hash_for_each_possible_safe(ctx->ht, buffer, tmp, node, alloc->addr) {
        if (alloc->addr == buffer->bus_addr) {
            hash_del(&buffer->node);
            mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
            BST_CV_TRACE_PRINTK("buffer: %px", buffer);
            //unmap the buffer from the userspace
            ret = vm_munmap((unsigned long)buffer->user_addr, buffer->block->size);
            if (ret < 0) {
                BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "vm_munmap failed! ret: %d", ret);
            }
            //free the buffer
            mutex_lock(&pbst_cv->mem_manager.mm_alloc_free_mutex );
            pbst_cv->mem_manager.ops->free(buffer->block);
            mutex_unlock(&pbst_cv->mem_manager.mm_alloc_free_mutex );
            devm_kfree(&pbst_cv->pdev->dev, buffer);
            return ret;
        }
    }
    mutex_unlock(&pbst_cv->mem_manager.mm_mutex);

    BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no buffer @ addr=0x%x", alloc->addr);
    return ret;
}

int bare_cv_mem_ctx_add(struct bst_cv* pbst_cv, struct file *filp)
{
    struct bst_cv_mem_ctx *ctx, *search;

    ctx = devm_kzalloc(&pbst_cv->pdev->dev, sizeof(*ctx), GFP_KERNEL);
    if (ctx == NULL) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "kmalloc failed");
        return -ENOMEM;
    }

    ctx->filp = filp;
    hash_init(ctx->ht);

    mutex_lock(&pbst_cv->mem_manager.mm_mutex);
    search = _find_mem_ctx(pbst_cv, filp);
    if (search != NULL) {
        mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
        devm_kfree(&pbst_cv->pdev->dev, ctx);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px already exits", filp);
        return -EINVAL;
    }
    list_add(&(ctx->link), &pbst_cv->mem_manager.mem_ctx_list);
    mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
    BST_CV_TRACE_PRINTK("ctx 0x%px added for filp 0x%px", ctx, filp);
    return 0;
}

static void _destroy_mem_ctx(struct bst_cv *pbst_cv, struct bst_cv_mem_ctx *ctx) {
    int i;
    struct bst_cv_buffer *buffer;
    struct hlist_node *tmp;

    hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
        hash_del(&buffer->node);
        BST_CV_TRACE_PRINTK("buffer %px", buffer);
        pbst_cv->mem_manager.ops->free(buffer->block);
        devm_kfree(&pbst_cv->pdev->dev, buffer);   
    }
    devm_kfree(&pbst_cv->pdev->dev, ctx);
    return;
}

int bare_cv_mem_ctx_remove(struct bst_cv *pbst_cv, struct file *filp)
{
    struct bst_cv_mem_ctx *ctx;

    mutex_lock(&pbst_cv->mem_manager.mm_mutex);
    ctx = _find_mem_ctx(pbst_cv, filp);
    if (ctx == NULL) {
        mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "fatal error: memory context with filp=0x%px not found", filp);
        return -EFAULT;
    }

    list_del(&ctx->link);

    /*
        As bare_cv_mem_ctx_remove is called in the close syscall, no
        synchronization protection against hashtable operations in ioctl
        syscalls is needed.
    */
    _destroy_mem_ctx(pbst_cv, ctx);
    mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
    BST_CV_TRACE_PRINTK("ctx 0x%px removed for filp 0x%px", ctx, filp);
    return 0;
}

/*
 * @func    bare_cv_mem_manager_init
 * @brief   This is the initialization function of the memory manager. It sets
 *          up the reseved memory and DMA configs of the device.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bare_cv_mem_manager_init(struct bst_cv *pbst_cv)
{
    int ret;

    //read properties from the device tree
    ret = device_property_read_u64_array(&pbst_cv->pdev->dev, "bus-offset",
        &pbst_cv->mem_manager.phys_to_bus_offset, 1);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no bus-offset property, ret %d", ret);
        return -ENOENT;
    }
    BST_CV_STAGE_PRINTK("phys_to_bus_offset: 0x%llx", pbst_cv->mem_manager.phys_to_bus_offset);

    //set dma coherent mask
    ret = dma_set_coherent_mask(&pbst_cv->pdev->dev, DMA_BIT_MASK(36));
    if (ret) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "dma_set_coherent_mask fail, ret %d", ret);
    }

    //init reserved memory
    ret = of_reserved_mem_device_init(&pbst_cv->pdev->dev);
    if (ret < 0) {
        BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "of_reserved_mem_device_init fail, ret: %d", ret);
        return -ENODEV;
    }

    pbst_cv->mem_manager.ops = &bst_cv_cma_mem_ops;
    INIT_LIST_HEAD(&pbst_cv->mem_manager.mem_ctx_list);
    mutex_init(&pbst_cv->mem_manager.mm_mutex);
    mutex_init(&pbst_cv->mem_manager.mm_alloc_free_mutex);
    return 0;
}


/*
 * @func    bare_cv_mem_manager_exit
 * @brief   This is the cleanup function of the memory manager. It frees all
 *          allocated memory blocks including the assigned memory of the DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bare_cv_mem_manager_exit(struct bst_cv *pbst_cv)
{
    struct bst_cv_mem_ctx *ctx;
    struct list_head *tmp, *cur;

    list_for_each_safe(cur, tmp, &pbst_cv->mem_manager.mem_ctx_list) {
        list_del(cur);
        ctx = container_of(cur, struct bst_cv_mem_ctx, link);
        _destroy_mem_ctx(pbst_cv, ctx);
    }
    of_reserved_mem_device_release(&pbst_cv->pdev->dev);
    return;
}

