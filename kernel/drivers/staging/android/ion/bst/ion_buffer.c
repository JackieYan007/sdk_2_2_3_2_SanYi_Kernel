// SPDX-License-Identifier: GPL-2.0
/*
 * ION Memory Allocator - buffer interface
 *
 * Copyright (c) 2019, Google, Inc.
 */

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-map-ops.h>
#include <linux/debugfs.h>

#define CREATE_TRACE_POINTS
#include "ion_trace.h"
#include "ion_private.h"

static atomic_long_t total_heap_bytes;

static int ion_buffer_debug_show(struct seq_file *s, void *unused)
{
	struct ion_buffer *buffer = s->private;

	seq_printf(s, "pid : %d\n", buffer->client_pid);
	seq_printf(s, "name : %s\n", buffer->client_name);
	seq_printf(s, "size : %ld\n", buffer->size);

	seq_puts(s, "\n");

	return 0;
}

static int ion_buffer_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_buffer_debug_show, inode->i_private);
}


static const struct file_operations ion_buffer_debug_fops = {
	.open           = ion_buffer_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void track_buffer_created(struct ion_buffer *buffer)
{
	long total = atomic_long_add_return(buffer->size, &total_heap_bytes);

	trace_ion_stat(buffer->sg_table, buffer->size, total);
}

static void track_buffer_destroyed(struct ion_buffer *buffer)
{
	long total = atomic_long_sub_return(buffer->size, &total_heap_bytes);

	trace_ion_stat(buffer->sg_table, -buffer->size, total);
}

/* this function should only be called while dev->lock is held */
static struct ion_buffer *ion_buffer_create(struct ion_heap *heap,
					    struct ion_device *dev,
					    unsigned long len,
					    unsigned long flags)
{
	struct ion_buffer *buffer;
	int ret;
	int i = 0;

	struct task_struct *task;
	struct buffer_id *temp_id;
	struct buffer_id *current_id;
	struct buffer_id *pos_id;
	struct buffer_id *prev_id;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	buffer->heap = heap;
	buffer->flags = flags;
	buffer->size = len;

	ret = heap->ops->allocate(heap, buffer, len, flags);

	if (ret) {
		if (!(heap->flags & ION_HEAP_FLAG_DEFER_FREE))
			goto err2;

		ion_heap_freelist_drain(heap, 0);
		ret = heap->ops->allocate(heap, buffer, len, flags);
		if (ret)
			goto err2;
	}

	if (!buffer->sg_table) {
		WARN_ONCE(1, "This heap needs to set the sgtable");
		ret = -EINVAL;
		goto err1;
	}

	spin_lock(&heap->stat_lock);
	heap->num_of_buffers++;
	heap->num_of_alloc_bytes += len;
	if (heap->num_of_alloc_bytes > heap->alloc_bytes_wm)
		heap->alloc_bytes_wm = heap->num_of_alloc_bytes;
	if (heap->num_of_buffers == 1) {
		/* This module reference lasts as long as at least one
		 * buffer is allocated from the heap. We are protected
		 * against ion_device_remove_heap() with dev->lock, so we can
		 * safely assume the module reference is going to* succeed.
		 */
		__module_get(heap->owner);
	}
	spin_unlock(&heap->stat_lock);

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	
	track_buffer_created(buffer);

	current_id = kzalloc(sizeof(*current_id), GFP_KERNEL);
	if (list_empty_careful(&heap->buf_id.list)) {
		current_id->id = i;
		buffer->id = current_id->id;
		list_add(&current_id->list, &heap->buf_id.list);
	} else {
			list_for_each_entry_safe(pos_id, temp_id, &heap->buf_id.list, list){
				if (i == pos_id->id && !(list_is_last(&pos_id->list, &heap->buf_id.list))) {
					i++;
					continue;

				/* The first node is not buf0, special treatment*/
				} else if (i == 0 && pos_id->id > 0) {
					current_id->id = 0;
					buffer->id = current_id->id;
					list_add(&current_id->list, &heap->buf_id.list);
					break;

				/* Insert directly when the end of the linked list is reached */
				} else if (list_is_last(&pos_id->list, &heap->buf_id.list)) {
					i++;
					current_id->id = i;
					buffer->id = current_id->id;
					list_add_tail(&current_id->list, &heap->buf_id.list);
					break;

				} else if (i < pos_id->id){
					current_id->id = i;
					buffer->id = current_id->id;
					prev_id = list_prev_entry(pos_id, list);
					__list_add(&current_id->list, &prev_id->list, &pos_id->list);
					break;
				}
			}
	}
	sprintf(buffer->name, "buffer_%ld", buffer->id);
	buffer->debug_file = debugfs_create_file(buffer->name, 0644,
	                                 heap->debug_heap, buffer,
	                                 &ion_buffer_debug_fops);
	if (!buffer->debug_file) {
        char buf[256], *path;

        path = dentry_path(heap->debug_heap, buf, 256);
        pr_err("Failed to create buffer debugfs at %s/%s\n",
               path, buffer->name);
    }
	
	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	buffer->client_pid = task_pid_nr(current->group_leader);
	/* don't bother to store task struct for kernel threads,
	   they can't be killed anyway */
	if (current->group_leader->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);

	strcpy(buffer->client_name, current->comm);

	return buffer;

err1:
	heap->ops->free(buffer);
err2:
	kfree(buffer);
	return ERR_PTR(ret);
}

static int ion_clear_pages(struct page **pages, int num, pgprot_t pgprot)
{
	void *addr = vmap(pages, num, VM_MAP, pgprot);

	if (!addr)
		return -ENOMEM;
	memset(addr, 0, PAGE_SIZE * num);
	vunmap(addr);

	return 0;
}

static int ion_sglist_zero(struct scatterlist *sgl, unsigned int nents,
			   pgprot_t pgprot)
{
	int p = 0;
	int ret = 0;
	struct sg_page_iter piter;
	struct page *pages[32];

	for_each_sg_page(sgl, &piter, nents, 0) {
		pages[p++] = sg_page_iter_page(&piter);
		if (p == ARRAY_SIZE(pages)) {
			ret = ion_clear_pages(pages, p, pgprot);
			if (ret)
				return ret;
			p = 0;
		}
	}
	if (p)
		ret = ion_clear_pages(pages, p, pgprot);

	return ret;
}

struct ion_buffer *ion_buffer_alloc(struct ion_device *dev, size_t len,
				    unsigned int heap_id_mask,
				    unsigned int flags)
{
	struct ion_buffer *buffer = NULL;
	struct ion_heap *heap;

	if (!dev || !len) {
		return ERR_PTR(-EINVAL);
	}

	/*
	 * traverse the list of heaps available in this system in priority
	 * order.  If the heap type is supported by the client, and matches the
	 * request of the caller allocate from it.  Repeat until allocate has
	 * succeeded or all heaps have been tried
	 */
	len = PAGE_ALIGN(len);
	if (!len)
		return ERR_PTR(-EINVAL);

	down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		/* if the caller didn't specify this heap id */
		if (!((1 << heap->id) & heap_id_mask))
			continue;
		buffer = ion_buffer_create(heap, dev, len, flags);
		if (!IS_ERR(buffer))
			break;
	}
	up_read(&dev->lock);

	if (!buffer)
		return ERR_PTR(-ENODEV);

	if (IS_ERR(buffer))
		return ERR_CAST(buffer);

	return buffer;
}

int ion_buffer_zero(struct ion_buffer *buffer)
{
	struct sg_table *table;
	pgprot_t pgprot;

	if (!buffer)
		return -EINVAL;

	table = buffer->sg_table;
	if (buffer->flags & ION_FLAG_CACHED)
		pgprot = PAGE_KERNEL;
	else
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	return ion_sglist_zero(table->sgl, table->nents, pgprot);
}
EXPORT_SYMBOL_GPL(ion_buffer_zero);

void ion_buffer_prep_noncached(struct ion_buffer *buffer)
{
	struct scatterlist *sg;
	struct sg_table *table;
	int i;

	if (WARN_ONCE(!buffer || !buffer->sg_table,
		      "%s needs a buffer and a sg_table", __func__) ||
	    buffer->flags & ION_FLAG_CACHED)
		return;

	table = buffer->sg_table;

	for_each_sg(table->sgl, sg, table->orig_nents, i)
		arch_dma_prep_coherent(sg_page(sg), sg->length);
}
EXPORT_SYMBOL_GPL(ion_buffer_prep_noncached);

void ion_buffer_release(struct ion_buffer *buffer)
{
	if (buffer->kmap_cnt > 0) {
		pr_warn_once("%s: buffer still mapped in the kernel\n",
			     __func__);
		ion_heap_unmap_kernel(buffer->heap, buffer);
	}
	buffer->heap->ops->free(buffer);
	spin_lock(&buffer->heap->stat_lock);
	buffer->heap->num_of_buffers--;
	buffer->heap->num_of_alloc_bytes -= buffer->size;
	if (buffer->heap->num_of_buffers == 0)
		module_put(buffer->heap->owner);
	spin_unlock(&buffer->heap->stat_lock);
	/* drop reference to the heap module */

	kfree(buffer);
}

int ion_buffer_destroy(struct ion_device *dev, struct ion_buffer *buffer)
{
	struct ion_heap *heap;
	struct buffer_id *temp_id;
	struct buffer_id *pos_id;

	if (!dev || !buffer) {
		pr_warn("%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	heap = buffer->heap;
	track_buffer_destroyed(buffer);

	list_for_each_entry_safe(pos_id, temp_id, &heap->buf_id.list, list){
		if (pos_id->id == buffer->id){
			list_del(&pos_id->list);
			kfree(pos_id);
			break;
		}
	}
	debugfs_remove(buffer->debug_file);

	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_freelist_add(heap, buffer);
	else
		ion_buffer_release(buffer);

	return 0;
}

void *ion_buffer_kmap_get(struct ion_buffer *buffer)
{
	void *vaddr;

	if (buffer->kmap_cnt) {
		buffer->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = ion_heap_map_kernel(buffer->heap, buffer);
	if (WARN_ONCE(!vaddr,
		      "heap->ops->map_kernel should return ERR_PTR on error"))
		return ERR_PTR(-EINVAL);
	if (IS_ERR(vaddr))
		return vaddr;
	buffer->vaddr = vaddr;
	buffer->kmap_cnt++;
	return vaddr;
}

void ion_buffer_kmap_put(struct ion_buffer *buffer)
{
	buffer->kmap_cnt--;
	if (!buffer->kmap_cnt) {
		ion_heap_unmap_kernel(buffer->heap, buffer);
		buffer->vaddr = NULL;
	}
}

u64 ion_get_total_heap_bytes(void)
{
	return atomic_long_read(&total_heap_bytes);
}
