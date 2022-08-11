/*
 * IPC: Linux device driver for Blck Sesame Technologies inter-processor communication
 *
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/of_reserved_mem.h>
#include <linux/mailbox_client.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/delay.h> // msleep
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>

#include "ipc_mempool.h"
#include "ipc_communication_manager.h"

#include "ipc_common.h"
#include "ipc_mailbox_controller.h"
#include "ipc_regs.h"
#include "ipc_msg_manager.h"
#include "user_head.h"
#include "ipc_session.h"

/********************* macros *******************/
#define IPC_DRIVER_NAME  "ipc-controller"
#define CPU_NR 8
#define SEND_RETRY_TIMES 10000;

/********************* extern global variables *******************/
extern enum ipc_core_e ipc_channel[IPC_CORE_MAX];

/********************* local variables ***************************/
static struct ipc_client_info* a55_client_info[CPU_NR];
static char s_core_of_irq[400];
//init status variable
bool ipc_init_status = false;

/********************* global variables ***************************/
struct platform_device * g_ipc_platform_dev;
//addr variable
static uint64_t g_ipc_all_cores_register_addr_phy = 0;
struct ipc_memblock *g_ipc_memblock = NULL;
struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr = NULL;
struct ipc_all_cores_register_addr* g_ipc_all_cores_register_addr_uaddr = NULL;

/********************* global functions ***************************/
bool get_ipc_init_status(void){return ipc_init_status;};

//ipc message send function, 
int32_t ipc_send_data(struct ipc_client_info *client_info, enum ipc_core_e src, void *data)
{
	IPC_LOG_INFO("send data from %d to %d", src, client_info->core_id);
	struct ipc_mbox* ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);
	
	// write dst source register to enable IPC hw to trigger interrupt
	struct ipc_fill_register_msg* fill_msg = (struct ipc_fill_register_msg*)data ;
	
	uint32_t source_reg_value = 0;
	uint64_t write_reg = client_info->tx_reg;
	uint32_t dst = client_info->core_id - IPC_CORE_ARM0;
	
	uint64_t read_reg = IRQ_CHECK_ADDR(ipc_mbox->event_base, dst);

	client_info->channel_status = CHANNEL_SENDING;
	source_reg_value |= (1ull<<(src));
	IPC_LOG_INFO("write to addr : 0x%x, value is : 0x%x, src = %d", __virt_to_phys(write_reg), source_reg_value, src);

	// while(readl_relaxed(read_reg) != 0);
	uint32_t max_read_cnt = SEND_RETRY_TIMES;
	int flag = 0;
	while (max_read_cnt--)
	{
		//note: only check trigger reg's value
		if ((readl_relaxed(read_reg) & source_reg_value) == 0)
		{
			flag = 1;
			break;
		}
	}
	if (!flag)
	{
		IPC_LOG_WARNING("The dst keeps interrupt! dst = %d", dst);
		return -1;
	}

	memcpy(&(g_ipc_all_cores_register_addr->addr[src].msg), data, sizeof(struct ipc_fill_register_msg));

	ipc_dsb(); //force to write msg to ddr
	writel_relaxed(source_reg_value, write_reg);

	//check irq status 
	uint32_t send_count = SEND_RETRY_TIMES;
	bool send_result = false;

	// IPC_LOG_INFO("read to addr : 0x%px", __virt_to_phys(write_reg));
	while(send_count--)
	{
		if ((readl_relaxed(read_reg) & source_reg_value) == 0)
		{
			client_info->channel_status = CHANNEL_SEND_SUCCESS;
			send_result = true;
			break;
		}
		schedule();
	}
	
	//clear sending box
	memset(&g_ipc_all_cores_register_addr->addr[src].msg, 0, sizeof(struct ipc_fill_register_msg));

	if (!send_result)
	{
		client_info->channel_status = CHANNEL_SEND_FAIL;
		IPC_LOG_WARNING("send to %d fail", dst);
		return -1;
	}
	
	return 0;
}

/**
 *  When the semaphore of which CPU has requested releaseed the lock,
 *  that CPU will receive this interrupt(this handler to be execute)
 *
 */
static irqreturn_t ipc_sem_interrupt_handler(int32_t irq, void *p)
{
	// check which bank
	struct ipc_mbox * ipc_mbox = p;
	int32_t bank_id = 0;

	IPC_LOG_INFO("triggerd irq number %d, bank id %d", irq, bank_id);

	// which semaphore's interrupt
	uint64_t sem_reg_addr = ipc_mbox->sem_base +
					SEM_BANK0_OFFSET_BASE+bank_id*SEM_BANK_OFFSET_STEP+
					SEM0_OFFSET_IN_BANK+SEM_IN_BANK_STEP*bank_id;
	uint32_t value = readl_relaxed(sem_reg_addr);
	int32_t intr_src = find_1_bit(value, 32, 0);

	value |= (1ull<<intr_src);  // set bit to 1
	writel_relaxed(value, sem_reg_addr);// W1C

	return IRQ_HANDLED;
}

// TODO: softirq may need
// ipc interrupt processing function
static irqreturn_t ipc_event_interrupt_handler(int32_t irq, void *p)
{
	struct ipc_mbox * ipc_mbox = p;
	int32_t cpu_id = s_core_of_irq[irq];
	
	uint64_t this_cpu_src_reg_addr = READ_SRC_AND_CLEAR_IRQ_ADDR(ipc_mbox->event_base, cpu_id);

	uint32_t value = readl_relaxed(this_cpu_src_reg_addr);
	// IPC_LOG_INFO("read addr 0x%px with value 0x%x", this_cpu_src_reg_addr, value);
	if(unlikely(value == 0))
	{
		IPC_LOG_ERR("src reg value is wrong 0x%x", value);
		return IRQ_NONE;
	}

	int32_t intr_src = get_msb_bit1_index(value);
	// IPC_LOG_INFO("intr_src = %d", intr_src);
	if(unlikely(intr_src < 0))
	{
		IPC_LOG_ERR("intr_src is wrong %d", intr_src);
		return IRQ_NONE;
	}

	if (unlikely(g_ipc_all_cores_register_addr == NULL))
	{
		value = (1ull << intr_src);  // set bit to 0
		writel_relaxed(value, this_cpu_src_reg_addr);// W1C
		IPC_LOG_ERR("msg addr is not ready %d", intr_src);
		return IRQ_NONE;
	}

	//msg process
	struct ipc_fill_register_msg intr_msg;
	ipc_dsb(); //force to write msg to ddr
	// intr_msg = g_ipc_all_cores_register_addr->addr[intr_src].msg;
	memcpy(&intr_msg, &(g_ipc_all_cores_register_addr->addr[intr_src].msg), sizeof(struct ipc_fill_register_msg));

	if (likely(intr_msg.type > 0))
	{
		ipc_drv_recv(intr_src, cpu_id, (void*)&intr_msg, sizeof(intr_msg));
	}
	else
	{
		IPC_LOG_ERR("intr_msg type wrong, intr_msg.type = %d, src = %d, cmd = %d", intr_msg.type, intr_src, intr_msg.cmd);
	}

	value = (1ull<<intr_src);  
	writel_relaxed(value, this_cpu_src_reg_addr);// W1C

	return IRQ_HANDLED;
}

//semaphore interrupt init
static int32_t of_sem_irqs(struct platform_device *pdev, struct ipc_mbox * ipc_mbox)
{
	int32_t ret = 0;
	int32_t bank_id ;
	const int32_t of_bank_begin_id = 8;
	const int32_t of_bank_end_id = 4+of_bank_begin_id;
	int32_t irq_num;
	for(bank_id=of_bank_begin_id;bank_id<of_bank_end_id; bank_id++)
	{
		irq_num = irq_of_parse_and_map(pdev->dev.of_node, bank_id);
		if(irq_num<0)
		{
			IPC_LOG_WARNING("NO irq is configured for cpu %d", bank_id);
			return ret;
		}

		IPC_LOG_INFO("bank %d, irq %d", bank_id, irq_num);

		ret = devm_request_irq(&pdev->dev, irq_num, ipc_sem_interrupt_handler,
								IRQF_ONESHOT|IRQF_SHARED,
								dev_name(&pdev->dev),
								ipc_mbox);
		if (ret) {
			IPC_DEV_ERR(&pdev->dev, "failed to request irq %d", irq_num);
			return ret;
		}
	}
	return ret;
}

//interrupt init
static int32_t of_event_irqs(struct platform_device *pdev, struct ipc_mbox * ipc_mbox)
{
	int32_t ret = 0;
	int32_t cpu_id = 0;
	int32_t irq_num;
	struct cpumask mask;

	for(cpu_id=0;cpu_id<num_online_cpus(); cpu_id++)
	{
		irq_num = platform_get_irq(pdev, cpu_id); 
		if(irq_num<0)
		{
			IPC_LOG_WARNING("NO irq is platform_get_irq for cpu %d", cpu_id);
			//return irq_num;
		}
		irq_num = irq_of_parse_and_map(pdev->dev.of_node, cpu_id);
		if(irq_num<0)
		{
			IPC_LOG_WARNING("NO irq is configured for cpu %d", cpu_id);
			return irq_num;
		}
		s_core_of_irq[irq_num] = cpu_id;
		IPC_LOG_INFO("cpu %d, irq %d", cpu_id, irq_num);

		a55_client_info[cpu_id] =
			devm_kzalloc(&pdev->dev, sizeof(*a55_client_info[cpu_id]), GFP_KERNEL);
		if (!a55_client_info[cpu_id])
		{
			IPC_DEV_ERR(&pdev->dev, "no enough memory!\n");
			return -ENOMEM;
		}

		a55_client_info[cpu_id]->dev = &pdev->dev;

		char *irq_desc;
		irq_desc = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%d]", "bst_ipc_apu", cpu_id);
		if (!irq_desc)
		{
			IPC_DEV_ERR(&pdev->dev, "devm_kasprintf no enough memory!\n");
			return -ENOMEM;
		}

		ret = devm_request_irq(&pdev->dev, irq_num, ipc_event_interrupt_handler,
								IRQF_ONESHOT|IRQF_SHARED,
								irq_desc,
								ipc_mbox);
		if (ret) {
			IPC_DEV_ERR(&pdev->dev, "failed to request irq %d, ret = %d", irq_num, ret);
			continue;
			// return ret;
		}

		cpumask_clear(&mask);
		cpumask_set_cpu(cpu_id, &mask);
		ret = irq_set_affinity_hint(irq_num, &mask);
		if(unlikely(ret < 0))
		{
			devm_free_irq(&pdev->dev, irq_num, ipc_mbox);
			IPC_DEV_ERR(&pdev->dev, "irq_set_affinity_hint32_t failed: ret = ",ret);
			return ret;
		}
	}
	return ret;
}

#ifdef ON_FPGA
static bool reset_vu440(struct ipc_mbox *ipc_mbox,struct platform_device *pdev)
{
	uint32_t repeat = 20;
	uint32_t status = 0;
	uint32_t value = 0xffffffff;
	struct resource *iomem;
	int32_t ret;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (unlikely(IS_ERR_OR_NULL(iomem))) {
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_DEV_ERR(
			&pdev->dev,
			"platform_get_resource IORESOURCE_MEM 2 return %d",
			ret);
		return ret;
	}
	IPC_LOG_INFO("fpga_reset start: 0x%llx, end: 0x%llx", iomem->start, iomem->end);
	ipc_mbox->fpga_reset = devm_ioremap_resource(&pdev->dev, iomem);
	if (unlikely(IS_ERR_OR_NULL(ipc_mbox->fpga_reset))) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox->fpga_reset);
		IPC_DEV_ERR(&pdev->dev,
					"Failed to remap fpga_reset regs: %d\n",
					ret);
		return ret;
	}

	// fpga status
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (unlikely(IS_ERR_OR_NULL(iomem))) {
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_DEV_ERR(
			&pdev->dev,
			"platform_get_resource IORESOURCE_MEM 3 return %d",
			ret);
		return ret;
	}
	IPC_LOG_INFO("fpga_status start: 0x%llx, end: 0x%llx", iomem->start, iomem->end);
	ipc_mbox->fpga_status = devm_ioremap_resource(&pdev->dev, iomem);
	if (unlikely(IS_ERR_OR_NULL(ipc_mbox->fpga_status))) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox->fpga_status);
		IPC_DEV_ERR(&pdev->dev,
					"Failed to remap fpga_status regs: %d",
					ret);
		return ret;
	}

	writel(0, ipc_mbox->fpga_reset);
	value = readl(ipc_mbox->fpga_reset);
	IPC_LOG_INFO("write fpga_reset value = %d", value);

	writel(3, ipc_mbox->fpga_reset);
	value = readl(ipc_mbox->fpga_reset);
	IPC_LOG_INFO("write fpga_reset value = %d", value);

	while(repeat-- && status != 0x1C) // waiting for reset complete
	{
		msleep(2000);
		status = readl(ipc_mbox->fpga_status);
		IPC_LOG_INFO("read fpga_status = 0x%x", status);
	}
	return status == 0x1c ? true : false;
}
#endif

//ipc message interrupt init
static int32_t set_en_irq_mask(struct ipc_mbox *ipc_mbox)
{
	int32_t cpu_id;
	int32_t i;
	uint64_t cpu_en_reg_addr;
	uint32_t en_mask;

	IPC_LOG_INFO("enter");
	for(cpu_id=0; cpu_id<num_online_cpus(); cpu_id++)
	{
		en_mask = 0;
		//set enable irq mask
		for(i=0;i<num_online_cpus(); i++)
		{
#ifndef ENABLE_SRC_DST_SAME
			if(cpu_id != i)
#endif
			{
				en_mask |= (1 << i);
			}
		}
		en_mask |= (1 << EVENT_BIT_R_SEC);
		en_mask |= (1 << EVENT_BIT_RISC_NET);
		en_mask |= (1 << EVENT_BIT_RISC_VSP);
		en_mask |= (1 << EVENT_BIT_RISC_ISP);
		en_mask |= (1 << EVENT_BIT_DSP_0);
		en_mask |= (1 << EVENT_BIT_DSP_1);
		en_mask |= (1 << EVENT_BIT_DSP_2);
		en_mask |= (1 << EVENT_BIT_DSP_3);
		en_mask |= (1 << EVENT_BIT_RPU0);
		en_mask |= (1 << EVENT_BIT_RPU1);
		en_mask |= (1 << 30);
		IPC_LOG_INFO("cpu %d en_mask = 0x%lx",cpu_id,  en_mask);
		// cpu_en_reg_addr = ipc_mbox->event_base + EVENT_MST_ID_A_CPU0 + cpu_id*EVENT_MST_A_CPU_STEP
		// 	+ EVENT_OFFSET_A_CPU0_IPC_INT_EN + cpu_id*EVENT_ENABLE_ACPU_STEP;

		cpu_en_reg_addr = CPU_EN_REG_ADDR(ipc_mbox->event_base, cpu_id);
		writel_relaxed(en_mask, cpu_en_reg_addr);
		en_mask = readl_relaxed(cpu_en_reg_addr);
		IPC_LOG_INFO("cpu %d after write enable reg, en_mask = 0x%x", cpu_id, en_mask);
	}

	return 0;
}

//ipc message share buffer init
int32_t alloc_mem(struct ipc_buffer * ipc_buffer)
{
	uint64_t uaddr = 0;
	int32_t ret = 0;
	struct ipc_memblock * memblock = NULL;
	struct ipc_mbox* ipc_mbox = platform_get_drvdata(g_ipc_platform_dev);

	ret = ipc_mbox->pool->ops->alloc(ipc_mbox->pool,
			ipc_buffer->size, ipc_buffer->align,
			&memblock);
	if (ret) {
		IPC_LOG_ERR("alloc fail!");
		return ret;
	}

	memblock->u_addr = uaddr;
	ipc_buffer->uaddr = uaddr;
	ipc_buffer->handle = (uint64_t)memblock;
	ipc_buffer->phy_addr.ptr_64 = memblock->phy_addr;

	IPC_LOG_INFO("uaddr: 0x%lx, handle: 0x%lx, phy_addr: 0x%lx",
		 ipc_buffer->uaddr, ipc_buffer->handle, ipc_buffer->phy_addr.ptr_64);

	return ret;
}

int32_t alloc_ipc_msg_share_buffer(bool user)
{
	//this function only can be called once.
	static atomic_t init_done = ATOMIC_INIT(0);
	if (1 == atomic_cmpxchg(&init_done, 0, 1))
	{
		return 0;
	}

	//message share buffer size
	struct ipc_buffer ipc_buffer = {
		.size = 4096,
		.align = 64,
	};

	// struct ipc_memblock *memblock = NULL;
	if (alloc_mem(&ipc_buffer) < 0)
	{
		IPC_LOG_ERR("ipc message share buffer create fail ");
		return -1;
	}

	//TODO:this memblock do not free, even if there is no case need to free it
	//TODO:this share buffer addr can not be leaked out, this addr need to manage in ipc_mbox
	g_ipc_memblock = (struct ipc_memblock *)ipc_buffer.handle;
	g_ipc_all_cores_register_addr = g_ipc_memblock->k_addr;
	g_ipc_all_cores_register_addr_phy = ipc_buffer.phy_addr.low;
	g_ipc_all_cores_register_addr_uaddr = g_ipc_memblock->u_addr;

	IPC_LOG_INFO("g_ipc_all_cores_register_addr = 0x%px", g_ipc_all_cores_register_addr);

	return 0;
}

static int32_t ipc_mempool_init(struct ipc_mbox *ipc_mbox)
{
	int32_t ret = 0;
	ret = of_reserved_mem_device_init(ipc_mbox->dev);
	if (ret < 0) {
		IPC_DEV_ERR(ipc_mbox->dev, "of_reserved_mem_device_init fail, ret: %d\n", ret);
		return -ENODEV;
	}
	return ipc_init_cma_mempool(&ipc_mbox->pool, ipc_mbox->dev);
}

static int32_t ipc_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipc_mbox *ipc_mbox;
	struct resource *iomem;
	int32_t ret = 0;

	//TODO: need to be optimized
	g_ipc_platform_dev = pdev;

	IPC_LOG_DEBUG("enter");

	ipc_mbox = devm_kzalloc(dev, sizeof(*ipc_mbox), GFP_KERNEL);
	if (unlikely(IS_ERR_OR_NULL(ipc_mbox))) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox);
		IPC_DEV_ERR(&pdev->dev, "devm_kzalloc return %d", ret);
		return ret;
	}

	ipc_mbox->dev = &pdev->dev;

	//init share mempool
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(36));
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "dma_set_coherent_mask fail, ret %d\n", ret);
	}
	ret = ipc_mempool_init(ipc_mbox);
	IPC_LOG_INFO("ipc mempool init result: %d\n", ret);
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "ipc_mempool_init fail, ret %d\n", ret);
		return ret;
	}

	//map ipc registers
	// event
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(unlikely(IS_ERR_OR_NULL(iomem)))
	{
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_DEV_ERR(&pdev->dev, "platform_get_resource IORESOURCE_MEM 0 return %d", ret);
		return ret;
	}
	IPC_LOG_INFO("ipc_mbox event start: 0x%llx, end: 0x%llx", iomem->start, iomem->end);
	ipc_mbox->event_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (unlikely(IS_ERR_OR_NULL(ipc_mbox->event_base))) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox->event_base);
		IPC_DEV_ERR(&pdev->dev, "Failed to remap ipc_mbox regs: %d\n", ret);
		return ret;
	}

	// semaphore
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if(unlikely(IS_ERR_OR_NULL(iomem)))
	{
		ret = PTR_ERR_OR_ZERO(iomem);
		IPC_DEV_ERR(&pdev->dev, "platform_get_resource IORESOURCE_MEM 1 return %d", ret);
		return ret;
	}
	IPC_LOG_INFO("ipc_mbox semaphore start: 0x%llx, end: 0x%llx", iomem->start, iomem->end);
	ipc_mbox->sem_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (unlikely(IS_ERR_OR_NULL(ipc_mbox->sem_base))) {
		ret = PTR_ERR_OR_ZERO(ipc_mbox->sem_base);
		IPC_DEV_ERR(&pdev->dev, "Failed to remap ipc_mbox regs: %d\n", ret);
		return ret;
	}

	ret = of_event_irqs(pdev, ipc_mbox);

#ifdef ON_FPGA
	// fpga reset
	if (!reset_vu440(ipc_mbox, pdev)) {
		IPC_DEV_ERR(&pdev->dev, "reset_vu440 failed");
		// return -1;
	}
#endif // ON_FPGA

	set_en_irq_mask(ipc_mbox);
	platform_set_drvdata(pdev, ipc_mbox);

	//ipc communication layer create
	ret = ipc_communication_create();
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "ipc_communication_create fail, ret %d\n", ret);
		return ret;
	}
	//ipc message share buffer create
	ret = alloc_ipc_msg_share_buffer(false);
	if (ret < 0) {
		IPC_DEV_ERR(&pdev->dev, "ipc_communication_create fail, ret %d\n", ret);
		return ret;
	}

	ipc_init_status = true;
	IPC_LOG_INFO("mailbox enabled");
	IPC_LOG_DEBUG("exit");

	return ret;
}

static int32_t ipc_mbox_remove(struct platform_device *pdev)
{
	int32_t cpu_id = 0;
	struct ipc_mbox *mbox = platform_get_drvdata(pdev);

	IPC_LOG_DEBUG("enter");

	IPC_LOG_DEBUG("exit");
	return 0;
}

static const struct of_device_id ipc_mbox_of_match[] = {
	{ .compatible = "bstn,bstn-mbox", },  //this name need change
	{},
};
MODULE_DEVICE_TABLE(of, ipc_mbox_of_match);
static struct platform_driver ipc_mbox_driver = {
	.driver = {
		.name = "bstn-mbox",    //this name need change
		.of_match_table = ipc_mbox_of_match,
	},
	.probe		= ipc_mbox_probe,
	.remove		= ipc_mbox_remove,
};

static int32_t __init ipc_mbox_init(void)
{
	return platform_driver_register(&ipc_mbox_driver);
}
module_platform_driver(ipc_mbox_driver);


