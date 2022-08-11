/*
 * bst_cv: Linux device driver for Blck Sesame Technologies Computer Vision IP
 * author: Jason Yin (jason.yin@bst.ai)
 *
 * @file	bst_cv_mem_manager.c
 * @brief   This file is the source code file of the memory manager of the
 *	  bst_cv driver. It contains definitions of actual memory allocation
 *	  functions as well as the initialization and exit functions of the
 *	  memory manager.
 * @note	Because the current buffer and memory structures have not been
 *	  finalized yet, further implementation for reliability like garbage
 *	  collection is not completed.
 */

#include <linux/mman.h>
#include "bst_cv.h"

struct dma_coherent_mem {
	void *virt_base;
	dma_addr_t device_base;
	unsigned long pfn_base;
	int size;
	int flags;
	unsigned long *bitmap;
	spinlock_t spinlock;
	bool use_dev_dma_pfn_offset;
};

static struct bst_cv_memblock *bst_cv_cma_alloc(struct bst_cv *pbst_cv, uint32_t size, uint32_t align);
static void bst_cv_cma_free(struct bst_cv_memblock *block);

static struct bst_cv_mem_ops bst_cv_cma_mem_ops = {
	.alloc = bst_cv_cma_alloc,
	.free = bst_cv_cma_free};

/*
 * @func	bst_cv_cma_alloc
 * @brief   This function allocates a requested continuous memory block.
 * @params  mem_manager - the pointer to the memory mem_manager
 *	  size - the requested size
 *	  align - the requested alignment (TBD)
 * @return  the pointer to the memory block - success
 *	  NULL - failure
 */
static struct bst_cv_memblock *bst_cv_cma_alloc(struct bst_cv *pbst_cv, uint32_t size, uint32_t align)
{
	void *kern_addr;
	dma_addr_t dma_addr;
	struct bst_cv_memblock *block;

	BST_CV_GS_TRACE_PRINTK("size is %d, align is %d", size, align);
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);

	if (size == 0) {
		BST_CV_DEV_ERR(pbst_cv->dev, "size is 0");
		return NULL;
	}

	block = devm_kzalloc(&pbst_cv->pdev->dev, sizeof(*block), GFP_KERNEL);
	if (block == NULL) {
		BST_CV_DEV_ERR(pbst_cv->dev, "block devm_kzalloc failed");
		return NULL;
	}

	kern_addr = dma_alloc_coherent(&pbst_cv->pdev->dev, size, &dma_addr, GFP_KERNEL);
	if (kern_addr == NULL) {
		devm_kfree(&pbst_cv->pdev->dev, block);
		BST_CV_DEV_ERR(pbst_cv->dev, "dma_alloc_coherent failed, size : %d", size);
		return NULL;
	}

	block->pbst_cv = pbst_cv;
	block->phys_addr = dma_to_phys(&pbst_cv->pdev->dev, dma_addr);
	block->size = size;
	block->kern_addr = kern_addr;
	return block;
}

/*
 * @func	bst_cv_cma_free
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
 * @func	bst_cv_user_buffer_alloc
 * @brief   This function allocates a requested memory chunk to be used as a
 *	  user buffer.
 * @params  filp - the misc device file pointer
 *	  pbst_cv - the pointer to the bst_cv device
 *	  alloc - the allocation information
 * @return  0 - success
 *	  error code - failure
 */
int bst_cv_gs_user_buffer_alloc(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc *alloc)
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
	buffer->block = pbst_cv->mem_manager.ops->alloc(pbst_cv, alloc->size, alloc->align);
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
	buffer->bus_addr = buffer->block->phys_addr;
	alloc->addr = buffer->bus_addr;
	alloc->ptr = buffer->user_addr;
	alloc->size = buffer->block->size;
	alloc->align = alloc->align > PAGE_SIZE ? alloc->align : PAGE_SIZE;

	BST_CV_GS_TRACE_PRINTK("addr: %x, size: %d", alloc->addr, buffer->block->size);

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
 * @func	bst_cv_user_buffer_alloc
 * @brief   This function frees a user buffer.
 * @params  pbst_cv - the pointer to the bst_cv device
 *	  alloc - the allocation information
 * @return  0 - success
 *	  error code - failure
 */
int bst_cv_gs_user_buffer_free(struct file *filp, struct bst_cv *pbst_cv, struct xrp_ioctl_alloc *alloc)
{
	struct bst_cv_buffer *buffer;
	struct bst_cv_mem_ctx *ctx;
	struct hlist_node *tmp;
	int ret = -ENOENT;
	int i = 0;

	mutex_lock(&pbst_cv->mem_manager.mm_mutex);
	ctx = _find_mem_ctx(pbst_cv, filp);
	if (ctx == NULL) {
		mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px not found", filp);
		return -ENOENT;
	}

	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		BST_CV_GS_TRACE_PRINTK("0x%llx : 0x%x", buffer->bus_addr, alloc->addr);
		if (alloc->addr == buffer->bus_addr) {
			hash_del(&buffer->node);
			mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
			//unmap the buffer from the userspace
			ret = vm_munmap((unsigned long)buffer->user_addr, buffer->block->size);
			if (ret < 0)
				BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "vm_munmap failed! ret: %d", ret);
			//free the buffer
			pbst_cv->mem_manager.ops->free(buffer->block);
			devm_kfree(&pbst_cv->pdev->dev, buffer);
			return ret;
		}
	}

	mutex_unlock(&pbst_cv->mem_manager.mm_mutex);

	BST_CV_DEV_INFO(&pbst_cv->pdev->dev, "no buffer @ addr=0x%x", alloc->addr);
	return ret;
}

int bst_cv_gs_mem_ctx_add(struct bst_cv *pbst_cv, struct file *filp)
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
	BST_CV_GS_TRACE_PRINTK("ctx 0x%px added for filp 0x%px", ctx, filp);
	return 0;
}

static void _destroy_mem_ctx(struct bst_cv *pbst_cv, struct bst_cv_mem_ctx *ctx)
{
	int i;
	struct bst_cv_buffer *buffer;
	struct hlist_node *tmp;

	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		hash_del(&buffer->node);
		BST_CV_GS_TRACE_PRINTK("buffer %px", buffer);
		pbst_cv->mem_manager.ops->free(buffer->block);
		devm_kfree(&pbst_cv->pdev->dev, buffer);
	}
	devm_kfree(&pbst_cv->pdev->dev, ctx);
	return;
}

int bst_cv_gs_mem_ctx_remove(struct bst_cv *pbst_cv, struct file *filp)
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
	As bst_cv_gs_mem_ctx_remove is called in the close syscall, no
	synchronization protection against hashtable operations in ioctl
	syscalls is needed.
	*/
	_destroy_mem_ctx(pbst_cv, ctx);
	mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
	BST_CV_GS_TRACE_PRINTK("ctx 0x%px removed for filp 0x%px", ctx, filp);
	return 0;
}

/*
 * @func	bst_cv_gs_mem_manager_init
 * @brief   This is the initialization function of the memory manager. It sets
 *	  up the reseved memory and DMA configs of the device.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *	  error code - failure
 */
int bst_cv_gs_mem_manager_init(struct bst_cv *pbst_cv)
{
	int ret;

	//read properties from the device tree
	ret = device_property_read_u64_array(&pbst_cv->pdev->dev, "bus-offset",
					 &pbst_cv->mem_manager.phys_to_bus_offset, 1);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no bus-offset property, ret %d", ret);
		return -ENOENT;
	}
	BST_CV_GS_TRACE_PRINTK("phys_to_bus_offset: 0x%llx", pbst_cv->mem_manager.phys_to_bus_offset);

	//set dma coherent mask
	ret = dma_set_coherent_mask(&pbst_cv->pdev->dev, DMA_BIT_MASK(36));
	if (ret) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "dma_set_coherent_mask fail, ret %d", ret);
	}
	BST_CV_GS_TRACE_PRINTK("dma_set_coherent_mask OK.");

	ret = of_reserved_mem_device_init(&pbst_cv->pdev->dev);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "of_reserved_mem_device_init fail, ret: %d", ret);
		return -ENODEV;
	}

	// BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "jason debug2");

	pbst_cv->mem_manager.ops = &bst_cv_cma_mem_ops;
	INIT_LIST_HEAD(&pbst_cv->mem_manager.mem_ctx_list);
	INIT_LIST_HEAD(&pbst_cv->mem_manager.buffer_busy);

	// BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "jason debug3");

	mutex_init(&pbst_cv->mem_manager.mm_mutex);

	// BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "jason debug4");
	// if(pbst_cv->pdev->dev.dma_mem->device_base == NULL) {
	// 	BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "device_base is NULL");
	// 	return -1;
	// }

	// pbst_cv->mem_manager.reserved_phy_addr = (phys_addr_t)pbst_cv->pdev->dev.dma_mem->device_base;
	// pbst_cv->mem_manager.kern_sub_phys_offset =
	// (phys_addr_t)pbst_cv->pdev->dev.dma_mem->virt_base - pbst_cv->pdev->dev.dma_mem->device_base;
	// pbst_cv->mem_manager.reserved_phy_addr_len = pbst_cv->pdev->dev.dma_mem->size;

	// BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "jason debug5");
	// BST_CV_GS_TRACE_PRINTK("reserve phy addr 0x%llx, size 0x%x, kern_sub_phys 0x%llx",
	// 		pbst_cv->mem_manager.reserved_phy_addr,
	// 		pbst_cv->mem_manager.reserved_phy_addr_len,
	// 		pbst_cv->mem_manager.kern_sub_phys_offset);
	return 0;
}

/*
 * @func	bst_cv_sg_mem_manager_exit
 * @brief   This is the cleanup function of the memory manager. It frees all
 *	  allocated memory blocks including the assigned memory of the DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_gs_mem_manager_exit(struct bst_cv *pbst_cv)
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

int bst_cv_get_phy_addr_by_exported(struct bst_cv *pbst_cv, int fd)
{
	struct list_head *list_t = &pbst_cv->mem_manager.buffer_busy;
	/*export buffer fd, cv output buffer*/
	bst_cv_dma_ctx_t *ctx_t;

	mutex_lock(&pbst_cv->mem_manager.dma_buf_mutex);
	list_for_each_entry(ctx_t, list_t, dma_node) {
		if (ctx_t->buffer.fd == fd) {
			mutex_unlock(&pbst_cv->mem_manager.dma_buf_mutex);
			return ctx_t->buffer.bus_addr;
		}
	}
	return 0;
}

/*!
 * @brief	   This function imports a dma-buf
 * @param[in]   pbst_cv The pointer to the bst_cv_device
 * @param[in]   buf The returned dma-buf information
 * @return	  0 - success
 *	  Error code - failure
 */
int bst_cv_gs_dma_buf_return(struct bst_cv *pbst_cv, struct bst_cv_dma_buf *buf)
{
	struct dma_buf *dmabuf;
	struct bst_cv_dma_buf_attachment *attach;
	struct hlist_node *tmp;
	bst_cv_dma_ctx_t *ctx_t;
	struct list_head *list_t = &pbst_cv->mem_manager.buffer_busy;
	int i;

	dmabuf = dma_buf_get(buf->fd);
	if (IS_ERR(dmabuf)) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to get dma_buf(fd=%d)", buf->fd);
		return PTR_ERR(dmabuf);
	}

	mutex_lock(&pbst_cv->mem_manager.dma_buf_mutex);

	/*import buffer...*/
	hash_for_each_safe(pbst_cv->mem_manager.dma_buf_ht, i, tmp, attach, node) {
		if (attach->attach->dmabuf == dmabuf) {
			//pr_err("===== find it, (attach->attach->dmabuf == dmabuf)\n");
			hash_del(&attach->node);
			mutex_unlock(&pbst_cv->mem_manager.dma_buf_mutex);
			dma_buf_unmap_attachment(attach->attach, attach->sgt, DMA_FROM_DEVICE);
			dma_buf_detach(dmabuf, attach->attach);
			dma_buf_put(dmabuf);
			dma_buf_put(dmabuf);
			return 0;
		}
	}
	/*export buffer fd, cv output buffer*/
	list_for_each_entry(ctx_t, list_t, dma_node) {
		if (ctx_t->buffer.fd == buf->fd) {
			list_del(&ctx_t->dma_node);
			devm_kfree(&pbst_cv->pdev->dev, ctx_t);
			mutex_unlock(&pbst_cv->mem_manager.dma_buf_mutex);
			dma_buf_put(dmabuf);
			dma_buf_put(dmabuf);
			return 0;
		}
	}

	mutex_unlock(&pbst_cv->mem_manager.dma_buf_mutex);
	dma_buf_put(dmabuf);
	BST_CV_DEV_INFO(&pbst_cv->pdev->dev, "attachment for dma_buf(fd=%d) not found", buf->fd);
	return -ENOENT;
}

/*!
 * @brief	   This function imports a dma-buf
 * @param[in]	   pbst_cv The pointer to the bst_cv_device
 * @param[in,out]   buf The imported dma-buf information
 * @return	  0 - success
 *	  Error code - failure
 */
static int bst_cv_dma_buf_import(struct bst_cv *pbst_cv, struct bst_cv_dma_buf *buf)
{
	struct dma_buf *dmabuf;
	struct bst_cv_dma_buf_attachment *attach;
	struct scatterlist *s;
	dma_addr_t expected;
	uint32_t size = 0, prev_size = 0;
	int i, ret;

	// if (buf->fd)
	attach = devm_kmalloc(&pbst_cv->pdev->dev, sizeof(attach), GFP_KERNEL);
	if (attach == NULL) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to allocate dma buf attachment meta\n");
		ret = -ENOMEM;
		goto dma_buf_import_fail_at_allocate_attach;
	}

	dmabuf = dma_buf_get(buf->fd);
	if (IS_ERR(dmabuf)) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to get dma_buf(fd=%d)\n", buf->fd);
		ret = PTR_ERR(dmabuf);
		goto dma_buf_import_fail_at_dma_buf_get;
	}

	//add a real reference to the dma-buf
	get_dma_buf(dmabuf);

	attach->attach = dma_buf_attach(dmabuf, &pbst_cv->pdev->dev);
	if (IS_ERR(attach->attach)) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to attach dma_buf(fd=%d)\n", buf->fd);
		ret = PTR_ERR(attach->attach);
		goto dma_buf_import_fail_at_dma_buf_attach;
	}

	/* get the associated scatterlist for this buffer */
	attach->sgt = dma_buf_map_attachment(attach->attach, DMA_FROM_DEVICE);
	if (IS_ERR(attach->sgt)) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "failed to get the scatterlist for dma_buf(fd=%d)\n", buf->fd);
		ret = PTR_ERR(attach->sgt);
		goto dma_buf_import_fail_at_dma_buf_map_attachment;
	}

	 expected = sg_dma_address(attach->sgt->sgl);
	// int expected_size = sg_dma_len(attach->sgt->sgl);
	// BST_CV_DEV_INFO(pbst_cv->dev, "expected :buf->bus_addr = %x\n", expected);
	// BST_CV_DEV_INFO(pbst_cv->dev, "expected :size = %x\n", expected_size);

	for_each_sg(attach->sgt->sgl, s, attach->sgt->nents, i) {
		if (sg_dma_address(s) != expected) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "scatterlist not contiguous for dma_buf(fd=%d)\n", buf->fd);
			ret = -EFAULT;
			goto dma_buf_import_fail_at_sgt_check;
		}
		expected = sg_dma_address(s) + sg_dma_len(s);
		size += sg_dma_len(s);
		//check for size overflow
		if (size < prev_size) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "scatterlist overflow for dma_buf(fd=%d)\n", buf->fd);
			ret = -EFAULT;
			goto dma_buf_import_fail_at_sgt_check;
		}
		prev_size = size;
	}

	buf->bus_addr = sg_dma_address(attach->sgt->sgl);
	//BST_CV_DEV_INFO(pbst_cv->dev, "buf->bus_addr = %x\n", buf->bus_addr);

	mutex_lock(&pbst_cv->mem_manager.dma_buf_mutex);
	hash_add(pbst_cv->mem_manager.dma_buf_ht, &attach->node, (size_t)dmabuf);
	mutex_unlock(&pbst_cv->mem_manager.dma_buf_mutex);

	dma_buf_put(dmabuf);

	return 0;

dma_buf_import_fail_at_sgt_check:
	dma_buf_unmap_attachment(attach->attach, attach->sgt, DMA_FROM_DEVICE);
dma_buf_import_fail_at_dma_buf_map_attachment:
	dma_buf_detach(dmabuf, attach->attach);
dma_buf_import_fail_at_dma_buf_attach:
	dma_buf_put(dmabuf);
	dma_buf_put(dmabuf);
dma_buf_import_fail_at_dma_buf_get:
	devm_kfree(&pbst_cv->pdev->dev, attach);
dma_buf_import_fail_at_allocate_attach:
	return ret;
}

int bst_cv_ioctl_dma_buf_import(struct bst_cv *pbst_cv,
				buf_req_t __user *ubuf_req)
{
	buf_req_t req;
	int ret;
	struct bst_cv_dma_buf buffer;

	ret = copy_from_user(&req, ubuf_req, sizeof(req));
	if (ret != 0)  {
		pr_err("copy_from_user failed\n");
		return ret;
	}
	if (req.buf_fd == 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "Invalid param!");
		return -EINVAL;
	}

	BST_CV_GS_TRACE_PRINTK("fd = %d\n", req.buf_fd);
	buffer.fd = req.buf_fd;
	bst_cv_dma_buf_import(pbst_cv, &buffer);
	req.phy_addr = buffer.bus_addr;

	if (req.phy_addr != 0) {
	ret = copy_to_user(ubuf_req, &req, sizeof(req));
		if (ret != 0) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_to_user failed!");
			bst_cv_gs_dma_buf_return(pbst_cv, &buffer);
			return ret;
		}
	}

	return 0;
}

int bst_cv_ioctl_dma_buf_return(struct bst_cv *pbst_cv, buf_req_t __user *ubuf_req)
{
	buf_req_t req;
	int ret;
	struct bst_cv_dma_buf buffer;

	ret = copy_from_user(&req, ubuf_req, sizeof(req));
	if (ret != 0) {
		BST_CV_DEV_ERR(pbst_cv->dev, "copy_from_user failed\n");
		return ret;
	}
	if (req.buf_fd == 0) {
		BST_CV_DEV_ERR(pbst_cv->dev, "Invalid param!");
		return -EINVAL;
	}

	buffer.fd = req.buf_fd;
	bst_cv_gs_dma_buf_return(pbst_cv, &buffer);
	return 0;
}

/*!
 * @brief	   The callback function for when the dma-buf is got attached
 *
 * @param[in]   dmabuf	  The dma-buf that got attached
 * @param[in]   attachment  The information of the attaching object
 *
 * @return	  0 - success
 *	  Error code - failure
 */
static int bstcv_dma_buf_export_attach(struct dma_buf *dmabuf, struct dma_buf_attachment *attachment)
{
	BST_CV_GS_TRACE_PRINTK("attach %s", dev_name(attachment->dev));
	return 0;
}

/*!
 * @brief	   The callback function for when the dma-buf is got detached
 * @param[in]   dmabuf	  The dma-buf that got attached
 * @param[in]   attachment  The information of the attaching object
 * @return	  None
 */
static void bstcv_dma_buf_export_detach(struct dma_buf *dmabuf, struct dma_buf_attachment *attachment)
{
	BST_CV_GS_TRACE_PRINTK("detach %s", dev_name(attachment->dev));
}

/*!
 * @brief	   The callback function for when the dma-buf is got mapped to
 *	  allocated buffer
 * @param[in]   attachment  The information of the attaching object
 * @param[in]   dir	 The mapping direction
 *
 * @return	  table	   The sg table that mapps the allocated buffer
 */
static struct sg_table *bstcv_dma_buf_export_map_dma_buf(struct dma_buf_attachment *attachment,
							 enum dma_data_direction dir)
{
	void *daddr = attachment->dmabuf->priv;
	uint32_t size = attachment->dmabuf->size;
	struct sg_table *table;

	BST_CV_GS_TRACE_PRINTK("map %s", dev_name(attachment->dev));
	table = kmalloc(sizeof(*table), GFP_KERNEL);

	// for dma-buf import testing, single entry per table
	// export dma addr, no mapping to virtual addr
	sg_alloc_table(table, 1, GFP_KERNEL);
	sg_dma_len(table->sgl) = size;
	sg_dma_address(table->sgl) = (dma_addr_t)daddr;
	BST_CV_GS_TRACE_PRINTK("map dma size 0x%x, addr 0x%llx", sg_dma_len(table->sgl), sg_dma_address(table->sgl));

	return table;
}

/*!
 * @brief	   The callback function for when the dma-buf is got unmapped
 *
 * @param[in]   attachment  The information of the attaching object
 * @param[in]   table	   The sg table that mapps the allocated buffer
 * @param[in]   dir	 The mapping direction
 *
 * @return	  None
 */
static void bstcv_dma_buf_export_unmap_dma_buf(struct dma_buf_attachment *attachment,
						   struct sg_table *table,
						   enum dma_data_direction dir)
{
	BST_CV_GS_TRACE_PRINTK("export unmap");
	sg_free_table(table);
	kfree(table);
}

/*!
 * @brief	   The callback function for when the dma-buf is released
 *
 * @param[in]   dmabuf	  The dma-buf
 *
 * @return	  None
 */
static void bstcv_dma_buf_export_release(struct dma_buf *dmabuf)
{
	BST_CV_GS_TRACE_PRINTK("export release");
}

/*!
 * @brief	   The callback function to setup vma mapping to user space
 *
 * @param[in]   dmabuf	  The dma-buf that should back the vma
 * @param[in]   vma	 vma for the mmap
 *
 * @return	  0 - success
 *	  Error code - failure
 */
static int bstcv_dma_buf_export_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	BST_CV_GS_TRACE_PRINTK("export mmap");
	return 0;
}

/*!
 * dma-buf exporter callback functions
 */
static const struct dma_buf_ops bstcv_dma_buf_export_ops = {
	.attach = bstcv_dma_buf_export_attach,
	.detach = bstcv_dma_buf_export_detach,
	.map_dma_buf = bstcv_dma_buf_export_map_dma_buf,
	.unmap_dma_buf = bstcv_dma_buf_export_unmap_dma_buf,
	.release = bstcv_dma_buf_export_release,
	.mmap = bstcv_dma_buf_export_mmap,
};

/*!
 * @brief	   This function imports a dma-buf.
 * @param[in]	   pbstn	  The bstn driver
 * @param[in,out]   buf	The user pointer to the imported dma-buf information
 * @return	  0 - success
 *	  Error code - failure
 */
int bst_cv_ioctl_dma_buf_export(struct file *filp, struct bst_cv *pbst_cv, struct bst_cv_dma_buf __user *buf)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct bst_cv_dma_buf buffer;
	bst_cv_dma_ctx_t *cv_dma_ctx;
	struct dma_buf *dma_buf;
	int ret;

	BST_CV_GS_TRACE_PRINTK("enter export");

	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}
	exp_info.size = buffer.size;
	exp_info.priv = (void *)(uintptr_t)buffer.bus_addr;
	exp_info.ops = &bstcv_dma_buf_export_ops;
	exp_info.flags = O_CLOEXEC;

	BST_CV_GS_TRACE_PRINTK("exp addr: 0x%px, size 0x%lx", exp_info.priv, exp_info.size);

	dma_buf = dma_buf_export(&exp_info);
	buffer.fd = dma_buf_fd(dma_buf, O_CLOEXEC);
	buffer.bus_addr = buffer.bus_addr;
	BST_CV_GS_TRACE_PRINTK("exp fd: %d", buffer.fd);

	cv_dma_ctx = devm_kzalloc(&pbst_cv->pdev->dev, sizeof(*cv_dma_ctx), GFP_KERNEL);
	if (cv_dma_ctx == NULL) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "kmalloc failed");
		return -ENOMEM;
	}
	cv_dma_ctx->buffer = buffer;
	list_add(&(cv_dma_ctx->dma_node), &pbst_cv->mem_manager.buffer_busy);

	ret = copy_to_user(buf, &buffer, sizeof(buffer));
	if (ret != 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "copy_to_user failed!");
		return -EFAULT;
	}

	BST_CV_GS_TRACE_PRINTK("exit export");

	return ret;
}

int bst_mem_check_mem_invalid(struct file *filp, struct bst_cv *pbst_cv, __u32 phy_addr)
{
	int i, is_exist = 0;
	struct bst_cv_buffer *buffer;
	struct bst_cv_mem_ctx *ctx;
	struct hlist_node *tmp;

	mutex_lock(&pbst_cv->mem_manager.mm_mutex);
	ctx = _find_mem_ctx(pbst_cv, filp);
	if (ctx == NULL) {
		mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px not found", filp);
		return -ENOENT;
	}

	// BST_CV_GS_TRACE_PRINTK("phy_addr : 0x%x", request->g_param.mem_info.gwarp_dst_phy_addr[j]);
	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		BST_CV_GS_TRACE_PRINTK("0x%llx", buffer->bus_addr);
		if (phy_addr == buffer->bus_addr) {
			is_exist = 1;
		}
	}

	if (!is_exist) {
		mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
		BST_CV_GS_TRACE_PRINTK("phy_addr [0x%x] never alloc from cv driver", phy_addr);
		return 1;
	}
	mutex_unlock(&pbst_cv->mem_manager.mm_mutex);
	return 0;
}

int bst_get_user_addr_by_bus(struct file *filp, struct bst_cv *pbst_cv,
				 phys_addr_t phy_addr, __u64 *user_addr)
{
	int i;
	struct bst_cv_mem_ctx *ctx;
	struct hlist_node *tmp;
	struct bst_cv_buffer *buffer;
	struct bst_cv_mem_manager *mem_manager = &pbst_cv->mem_manager;

	ctx = _find_mem_ctx(pbst_cv, filp);
	if (ctx == NULL) {
		mutex_unlock(&mem_manager->mm_mutex);
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "memory context with filp=0x%px not found", filp);
		return -ENOENT;
	}

	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		if (buffer->bus_addr == phy_addr) {
			*user_addr = buffer->user_addr;
		}
	}

	if (*user_addr == 0)
		return -ENOENT;

	return 0;
}
