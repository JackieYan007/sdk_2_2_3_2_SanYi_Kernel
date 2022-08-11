#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/mman.h>
#include <uapi/asm-generic/mman-common.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)
#include <linux/dma-mapping.h>
#else
#include <linux/dma-direct.h>
#endif

#include "user_head.h"
#include "ipc.h"
#include "ipc_mempool.h"
#include "ipc_communication_manager.h"
#include "ipc_host_server.h"


#define IPC_DRIVER_NAME "ipc_mempool"

static DEFINE_MUTEX(memblock_mutex);

extern struct platform_device *g_ipc_platform_dev;

static int32_t ipc_cma_alloc(struct ipc_mempool *mempool, u32 size, u32 align,
	struct ipc_memblock** memblock)
{
	struct ipc_memblock *block;
	dma_addr_t dma_addr;
	void *k_addr;

	size = ALIGN(size, PAGE_SIZE);

	block = devm_kzalloc(&g_ipc_platform_dev->dev, sizeof(struct ipc_memblock), GFP_KERNEL);
	if (!block)
		return -ENOMEM;

	k_addr = dma_alloc_coherent(mempool->dev,
						size,
						&dma_addr,
						GFP_KERNEL);
	if (!k_addr) {
		devm_kfree(&g_ipc_platform_dev->dev,  block);
		return -ENOMEM;
	}

	block->pool = mempool;
	block->phy_addr = dma_to_phys(mempool->dev, dma_addr);
	block->size = size;
	block->k_addr = k_addr;
	block->u_addr = 0;
	memset(block->k_addr, 0, size);
	*memblock = block;

	mutex_lock(&memblock_mutex);
	list_add_tail(&(block->memblock_list), &(mempool->memblock_list_head));
	mutex_unlock(&memblock_mutex);
	IPC_LOG_INFO("alloc phy_addr : 0x%px", block->phy_addr);

	return 0;
}

static void ipc_cma_free(struct ipc_memblock *memblock)
{
	struct ipc_mempool *pool = memblock->pool;

	dma_free_coherent(pool->dev,
		  memblock->size,
		  memblock->k_addr,
		  phys_to_dma(pool->dev, memblock->phy_addr));

	struct ipc_memblock *pos;
	mutex_lock(&memblock_mutex);
	list_for_each_entry(pos,&(pool->memblock_list_head),memblock_list)
	{
		if(pos && pos == memblock)
		{
			list_del(&memblock->memblock_list)	;														 //用list_for_each_entry的话，pos可以写你定义的结构体类型（struct student）
			IPC_TRACE_PRINTK("remove memblock : 0x%px", memblock);     // 而不是struct list_head ，而用list_for_each的话，pos必须是 list_head类型的
			break;
		}
	}
	mutex_unlock(&memblock_mutex);
	devm_kfree(&g_ipc_platform_dev->dev, memblock);
}

const struct ipc_mempool_ops ipc_cma_mempool_ops = {
	.alloc = ipc_cma_alloc,
	.free = ipc_cma_free,
	.phyaddr = ipc_memblock_phyaddr,
	.kaddr = ipc_memblock_kaddr
};

int32_t ipc_init_cma_mempool(struct ipc_mempool **ppool, struct device *dev)
{
	struct ipc_mempool *cma_pool = kmalloc(sizeof(struct ipc_mempool), GFP_KERNEL);

	if (!cma_pool)
		return -ENOMEM;

	cma_pool->dev = dev;
	cma_pool->ops = &ipc_cma_mempool_ops;
	INIT_LIST_HEAD(&(cma_pool->memblock_list_head));
	*ppool = cma_pool;
	return 0;
}


void ipc_destroy_cma_mempool(struct ipc_mempool *pool)
{
	if(pool)
		devm_kfree(&g_ipc_platform_dev->dev, pool);
}


int32_t bst_alloc(struct ipc_buffer * ipc_buffer, bool from_user)
{
	uint64_t uaddr = 0;
	int32_t ret = 0;
	struct ipc_memblock * memblock = NULL;
	struct bstipc* bstipc = platform_get_drvdata(g_ipc_platform_dev);

	// spin_lock_irqsave(&(bstipc->pool_lock), flags);
	ret = bstipc->pool->ops->alloc(bstipc->pool,
			ipc_buffer->size, ipc_buffer->align,
			&memblock);
	if (ret) {
		IPC_DEV_ERR(bstipc->dev, "alloc fail!\n");
		// spin_unlock_irqrestore(&(bstipc->pool_lock), flags);
		return ret;
	}
	// spin_unlock_irqrestore(&(bstipc->pool_lock), flags);

	if(from_user)
	{
		uaddr = vm_mmap((struct file *)bstipc->private_data, 0, ipc_memblock_size(memblock),
			PROT_READ | PROT_WRITE, MAP_SHARED,
			ipc_memblock_phyaddr(memblock));
		if (!uaddr || IS_ERR_VALUE(uaddr)) {
			bstipc->pool->ops->free(memblock);
			IPC_DEV_ERR(bstipc->dev, "vm_mmap fail! ret: %ld\n", uaddr);
			return -EFAULT;
		}
	}
	memblock->u_addr = uaddr;
	ipc_buffer->uaddr = uaddr;
	ipc_buffer->handle = (uint64_t)memblock;
	ipc_buffer->phy_addr.ptr_64 = memblock->phy_addr;

	IPC_LOG_INFO("uaddr: 0x%lx, handle: 0x%lx, phy_addr: 0x%lx",
		 ipc_buffer->uaddr, ipc_buffer->handle, ipc_buffer->phy_addr.ptr_64);

	return ret;
}

int32_t bst_free(phys_addr_t phy_addr, bool from_user)
{
	struct ipc_memblock *memblock = NULL;
	struct bstipc* bstipc = NULL;
	int32_t ret = 0;

	IPC_LOG_DEBUG( "enter");

	bstipc = platform_get_drvdata(g_ipc_platform_dev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}

	struct ipc_mempool *pool = bstipc->pool;
	struct ipc_memblock *pos;
	mutex_lock(&memblock_mutex);
	list_for_each_entry(pos,&(pool->memblock_list_head),memblock_list)
	{
		if(pos && pos->phy_addr == phy_addr)
		{
			IPC_LOG_INFO("found mesg at 0x%lx", phy_addr);
			memblock = pos;
			break;
		}
	}
	mutex_unlock(&memblock_mutex);

	if(memblock == NULL)
	{
		IPC_LOG_ERR("can not find mesg at 0x%lx", phy_addr);
		return -1;
	}

	if(memblock->u_addr)
	{
		IPC_LOG_INFO("vm_munmap vaddr:0x%lx", memblock->u_addr);
		ret = vm_munmap(memblock->u_addr , memblock->size);
		if (ret < 0) {
			IPC_DEV_ERR(bstipc->dev, "vm_munmap fail! ret: %d\n", ret);
			return ret;
		}
	}

	IPC_LOG_INFO("memblock size: 0x%x, phy_addr: 0x%lx, k_addr: 0x%px",
		memblock->size, memblock->phy_addr, memblock->k_addr);

	bstipc->pool->ops->free(memblock);

	IPC_LOG_DEBUG("exit");

	return ret;
}

void* get_kaddr_from_phy(phys_addr_t phy)
{
	struct bstipc* bstipc = platform_get_drvdata(g_ipc_platform_dev);
	struct ipc_mempool *pool = bstipc->pool;
	struct ipc_memblock *pos;
	mutex_lock(&memblock_mutex);
	list_for_each_entry(pos,&(pool->memblock_list_head),memblock_list)
	{
		if(pos && pos->phy_addr == phy)
		{
			IPC_TRACE_PRINTK("found mesg at 0x%lx", phy);
			mutex_unlock(&memblock_mutex);
			return pos->k_addr;
		}
	}

	mutex_unlock(&memblock_mutex);
	return NULL;
}
