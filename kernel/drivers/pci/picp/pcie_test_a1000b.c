/* pcie baremetal driver */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/bst_boardconfig.h>

#include <linux/picp.h>
#include "pcie_test.h"
#include "picp_private.h"

#define PCIE_TEST 1
#define PCIE_DMA_USEIRQ 0

/* 用于A1000硬件检测中PCIE RC和EP的测试，A1000中PCIE有两个控制器：
   1.DM控制器（既可作RC又可做EP，可x4也可x2）
   2.EP控制器（只能做EP，且只有x2） 
 */
/* 用于动态加载，可传递模块参数hwtest_mode控制驱动的执行（分4种方式）
   1.用DM做RC，x4。计划与方式2对接进行测试
   2.用DM做EP，x4
   3.用EP控制器做EP，x2，计划与方式4对接进行测试，需要配置x2 flip功能
   4.用DM控制器做RC，x2
   需要配置dts为pcie留0x80000000-0x80100000这1M内存供pcie rc和ep测试
 */
struct pci_reg_base pci_base;
struct resource *res[5] = {0,0,0,0,0};
struct dm_outbound outbound0_cfg;
struct dm_outbound outbound1_mem;
struct dm_outbound outbound2_dma;
struct dm_outbound dm_ep_outbound1_mem;
struct dm_outbound ep_outbound1_mem;

struct task_struct *rc_dscan;
struct task_struct *ep_detect;
static u32 hwtest_mode = 1;
static u32 link_status = 0;
spinlock_t dma_rlock, dma_wlock;
void __iomem *debug;
//static int dmaflag = 0;
wait_queue_head_t read_wq;
wait_queue_head_t write_wq;
static u32 read_dma_reg;
static u32 write_dma_reg;

struct task_struct write_task;
struct task_struct read_task;


module_param(hwtest_mode, uint, 0644);
MODULE_PARM_DESC(hwtest_mode, "pcie hwtest mode selection(1:dm-rcx4-sideA, 2:dm-epx4-sideB, 3:ep-epx2-sideA, 4:dm-rcx2-sideB)");
static void ep_test_case(void);

/*******************************************************ioremap********************************************************/
#if 0
/* phy寄存器映射 */
static void ioremap_phy_reg(void)
{
	/* 不测眼图不用操作phy */
	return;
}
#endif

/* apb寄存器映射 */
static void ioremap_apb_reg(void)
{
	void __iomem *addr;
	
	addr = ioremap(PCI_APB_BASE, 0x150);
	if (addr) {	
		pci_base.apb_base = addr;
		pcie_dbg( "ioremap_apb_reg 0x%x --> 0x%llx\n", PCI_APB_BASE, (u64)addr);
	} else {
		pcie_dbg( "ioremap_apb_reg fail\n");
	}
	
	return;
}

/* sw rst寄存器映射 */
static void ioremap_sw_rst_reg(void)
{
	void __iomem *addr;
		
	addr = ioremap(PCI_SW_RST, 0x4);
	if (addr) { 
		pci_base.sw_rst = addr;
		pcie_dbg( "ioremap_sw_rst_reg 0x%x --> 0x%llx\n", PCI_SW_RST, (u64)addr);
	} else {
		pcie_dbg( "ioremap_sw_rst_reg fail\n");
	}
	
	return;
}

/* sw rst寄存器映射 */
static void ioremap_gic_reg(void)
{
	void __iomem *addr;
		
	addr = ioremap(A55_GIC_BASE_ADDR + A55_GIC_DIST_OFFSET, 0x200);
	if (addr) { 
		pci_base.gic_dist = addr;
		pcie_dbg( "ioremap_gic_reg 0x%x --> 0x%llx\n", A55_GIC_BASE_ADDR + A55_GIC_DIST_OFFSET, (u64)addr);
	} else {
		pcie_dbg( "ioremap_gic_reg fail\n");
	}
	
	return;
}

/* ioremap映射dm控制器寄存器物理地址为虚拟地址 */
static void ioremap_dm_reg(void)
{
	void __iomem *addr;
	
	addr = ioremap(PCI_DM_BASE, 0x1000);
	if (addr) {	
		pci_base.dm_base = addr;
		pcie_dbg( "ioremap_dm_reg 0x%x --> 0x%llx\n", PCI_DM_BASE, (u64)addr);
	} else {
		pcie_dbg( "ioremap_dm_reg fail\n");
	}
	addr = ioremap(PCI_DM_ATU, 0x1000); /* 8*0x200 */
	if (addr) {	
		pci_base.dm_atu = addr;
		pcie_dbg( "ioremap_dm_atu 0x%x --> 0x%llx\n", PCI_DM_ATU, (u64)addr);
	} else {
		pcie_dbg( "ioremap_dm_atu fail\n");
	}
	addr = ioremap(PCI_DM_DMA, 0x1200); /* 0x200 + 8*0x200 */
	if (addr) {	
		pci_base.dm_dma = addr;
		pcie_dbg( "ioremap_dm_dma 0x%x --> 0x%llx\n", PCI_DM_DMA, (u64)addr);
	} else {
		pcie_dbg( "ioremap_dm_dma fail\n");
	}
	addr = ioremap(PCI_DM_AXISLAVE_REGION_CFG, PCI_DM_AXISLAVE_REGION_CFG_SIZE); /* 4M */
	if (addr) {	
		pci_base.dm_axislv_cfg = addr;
		pcie_dbg( "ioremap_dm_axislv_cfg 0x%x --> 0x%llx, size=0x%x\n", PCI_DM_AXISLAVE_REGION_CFG, (u64)addr, PCI_DM_AXISLAVE_REGION_CFG_SIZE);
	} else {
		pcie_dbg( "ioremap_dm_axislv_cfg fail\n");
	}
	addr = ioremap(PCI_DM_AXISLAVE_REGION_MEM, PCI_DM_AXISLAVE_REGION_MEM_SIZE); /* 1M */
	if (addr) {	
		pci_base.dm_axislv_mem = addr;
		pcie_dbg( "ioremap_dm_axislv_mem 0x%x --> 0x%llx, size=0x%x\n", PCI_DM_AXISLAVE_REGION_MEM, (u64)addr, PCI_DM_AXISLAVE_REGION_MEM_SIZE);
	} else {
		pcie_dbg( "ioremap_dm_axislv_mem fail\n");
	}
	addr = ioremap(PCI_DM_AXISLAVE_REGION_DMA, PCI_DM_AXISLAVE_REGION_DMA_SIZE); /* 1M */
	if (addr) {	
		pci_base.dm_axislv_dma = addr;
		pcie_dbg( "ioremap_dm_axislv_dma 0x%x --> 0x%llx, size=0x%x\n", PCI_DM_AXISLAVE_REGION_DMA, (u64)addr,PCI_DM_AXISLAVE_REGION_DMA_SIZE);
	} else {
		pcie_dbg( "ioremap_dm_axislv_dma fail\n");
	}
#if 0
	addr = ioremap(PCI_TEST_MEM, PCI_TEST_SIZE); /* 1M */
	if (addr) {	
		pci_base.dm_mem = addr;
		pcie_dbg( "ioremap_dm_mem 0x%x --> 0x%llx\n", PCI_TEST_MEM, (u64)addr);
	} else {
		pcie_dbg( "ioremap_dm_mem fail\n");
	}
#endif	
	return;
}

/* ioremap映射ep控制器寄存器物理地址为虚拟地址 */
static void ioremap_ep_reg(void)
{
	void __iomem *addr;
	
	addr = ioremap(PCI_EP_BASE, 0x1000);
	if (addr) {	
		pci_base.ep_base = addr;
		pcie_dbg( "ioremap_ep_reg 0x%x --> 0x%llx\n", PCI_EP_BASE, (u64)addr);
	} else {
		pcie_dbg( "ioremap_ep_reg fail\n");
	}
	addr = ioremap(PCI_EP_ATU, 0x1000); /* 8*0x200 */
	if (addr) {	
		pci_base.ep_atu = addr;
		pcie_dbg( "ioremap_ep_atu 0x%x --> 0x%llx\n", PCI_EP_ATU, (u64)addr);
	} else {
		pcie_dbg( "ioremap_ep_atu fail\n");
	}
	addr = ioremap(PCI_EP_DMA, 0x1200); /* 0x200 + 8*0x200 */
	if (addr) {	
		pci_base.ep_dma = addr;
		pcie_dbg( "ioremap_ep_dma 0x%x --> 0x%llx\n", PCI_EP_DMA, (u64)addr);
	} else {
		pcie_dbg( "ioremap_ep_dma fail\n");
	}
	addr = ioremap(PCI_EP_AXISLAVE_REGION_MEM, PCI_EP_AXISLAVE_REGION_MEM_SIZE); /* 16K */
	if (addr) {	
		pci_base.ep_axislv_mem = addr;
		pcie_dbg( "ioremap_ep_axislv_mem 0x%x --> 0x%llx\n", PCI_EP_AXISLAVE_REGION_MEM, (u64)addr);
	} else {
		pcie_dbg( "ioremap_ep_axislv_mem fail\n");
	}
#if 0
	addr = ioremap(PCI_TEST_MEM, PCI_TEST_SIZE); /* 1M */
	if (addr) {	
		pci_base.ep_mem = addr;
		pcie_dbg( "ioremap_ep_mem 0x%x --> 0x%llx\n", PCI_TEST_MEM, (u64)addr);
	} else {
		pcie_dbg( "ioremap_ep_mem fail\n");
	}
#endif	
	return;
}

static void ioremap_rcx4(void)
{
	ioremap_apb_reg();
	ioremap_sw_rst_reg();
	ioremap_gic_reg();
	ioremap_dm_reg();
	
	return;
}

static void ioremap_epx4(void)
{
	ioremap_apb_reg();
	ioremap_sw_rst_reg();
	ioremap_dm_reg(); //使用dm控制器配置为ep
	
	return;
}

static void ioremap_epx2(void)
{
	ioremap_apb_reg();
	ioremap_sw_rst_reg();
	ioremap_ep_reg(); //使用ep控制器配置为ep
	
	return;
}

static void ioremap_rcx2(void)
{
	ioremap_apb_reg();
	ioremap_sw_rst_reg();
	ioremap_dm_reg();
	
	return;
}

/*******************************************************reg rw api*****************************************************/
/* phy寄存器读写 */
static inline u32 read_phy(u32 offset)
{
	return 0;
}
static inline void write_phy(u32 offset, u32 val)
{
    return;
}

/* apb寄存器读写 */
static inline u32 read_apb(u32 offset)
{
	return readl(pci_base.apb_base + offset);
}
static inline void write_apb(u32 offset, u32 val)
{
	writel(val, pci_base.apb_base + offset);
	return;
}

/* sw rst寄存器读写 */
static inline u32 read_sw_rst(void)
{
	return readl(pci_base.sw_rst);
}
static inline void write_sw_rst(u32 val)
{
	writel(val, pci_base.sw_rst);
	return;
}

/* dm控制器寄存器读写 */
static inline u32 read_dm_reg(u32 offset)
{
	return readl(pci_base.dm_base + offset);
}
static inline void write_dm_reg(u32 offset, u32 val)
{
	writel(val, pci_base.dm_base + offset);
	return;
}

/* dm控制器ATU寄存器读写 */
static inline u32 read_dm_atu_reg(u32 offset)
{
	return readl(pci_base.dm_atu + offset);
}
static inline void write_dm_atu_reg(u32 offset, u32 val)
{
	writel(val, pci_base.dm_atu + offset);
	return;
}

/* dm控制器DMA寄存器读写 */
static inline u32 read_dm_dma_reg(u32 offset)
{
	return readl(pci_base.dm_dma + offset);
}
static inline void write_dm_dma_reg(u32 offset, u32 val)
{
	writel(val, pci_base.dm_dma + offset);
	return;
}

/* dm侧pcie测试用到的内存读写 */
static inline u32 read_dm_mem(u32 offset)
{
	return readl(pci_base.dm_mem + offset);
}
static inline void write_dm_mem(u32 offset, u32 val)
{
	writel(val, pci_base.dm_mem + offset);
	return;
}

/* ep控制器寄存器读写 */
static inline u32 read_ep_reg(u32 offset)
{
	return readl(pci_base.ep_base + offset);
}
static inline void write_ep_reg(u32 offset, u32 val)
{
	writel(val, pci_base.ep_base + offset);
	return;
}

/* ep控制器ATU寄存器读写 */
static inline u32 read_ep_atu_reg(u32 offset)
{
	return readl(pci_base.ep_atu + offset);
}
static inline void write_ep_atu_reg(u32 offset, u32 val)
{
	writel(val, pci_base.ep_atu + offset);
	return;
}

/* ep控制器DMA寄存器读写 */
static inline u32 read_ep_dma_reg(u32 offset)
{
	return readl(pci_base.ep_dma + offset);
}
static inline void write_ep_dma_reg(u32 offset, u32 val)
{
	writel(val, pci_base.ep_dma + offset);
	return;
}

/* ep侧pcie测试用到的内存读写 */
static inline u32 read_ep_mem(u32 offset)
{
	return readl(pci_base.ep_mem + offset);
}
static inline void write_ep_mem(u32 offset, u32 val)
{
	writel(val, pci_base.ep_mem + offset);
	return;
}

/*******************************************************cfg/mem rw api*****************************************************/

static u32 dm_pcie_outband_cfg_rd(u32 cfg_rd_address) 
{
	u32 iatu_trans_address;
	
	if(cfg_rd_address > outbound0_cfg.size) {
		pcie_dbg( "dm_pcie_outband_cfg_rd para err!\n");
		return 0;
	}
	iatu_trans_address = cfg_rd_address;//outbound0_cfg.base + cfg_rd_address - outbound0_cfg.target;
	return readl(pci_base.dm_axislv_cfg + iatu_trans_address);
}

static void dm_pcie_outband_cfg_wr(u32 cfg_wr_address, u32 cfg_wr_data) 
{
	u32 iatu_trans_address;
	
	if(cfg_wr_address > outbound0_cfg.size) {
	  pcie_dbg( "dm_pcie_outband_cfg_wr para err!\n");
	  return;
	}
	iatu_trans_address = cfg_wr_address;//outbound0_cfg.base + cfg_wr_address - outbound0_cfg.target;
	writel(cfg_wr_data, pci_base.dm_axislv_cfg + iatu_trans_address);

	return;
}

static u32 dm_pcie_outband_mem_rd(u32 mem_rd_address) 
{
	u32 iatu_trans_address;
	
	if(mem_rd_address > outbound1_mem.size) {
		pcie_dbg( "dm_pcie_outband_mem_rd para err!\n");
		return 0;
	}
	iatu_trans_address = mem_rd_address;//outbound1_mem.base + mem_rd_address - outbound1_mem.target;
	return readl(pci_base.dm_axislv_mem + iatu_trans_address);
}

static void dm_pcie_outband_mem_wr(u32 mem_wr_address, u32 mem_wr_data) 
{
	u32 iatu_trans_address;
	
	if(mem_wr_address > outbound1_mem.size) {
	  pcie_dbg( "dm_pcie_outband_mem_wr para err!\n");
	  return;
	}
	iatu_trans_address = mem_wr_address;//outbound1_mem.base + mem_wr_address - outbound1_mem.target;
	writel(mem_wr_data, pci_base.dm_axislv_mem + iatu_trans_address);

	return;
}

static u32 dm_pcie_outband_dma_rd(u32 dma_rd_address) 
{
	if(dma_rd_address > outbound2_dma.size) {
		pcie_dbg( "dm_pcie_outband_dma_rd para err!\n");
		return 0;
	}
	return readl(pci_base.dm_axislv_dma + 0x1000 + dma_rd_address);
	//return readl(0x1000 + dma_rd_address);
}

static void dm_pcie_outband_dma_wr(u32 dma_wr_address, u32 dma_wr_data) 
{
	if(dma_wr_address > outbound2_dma.size) {
	  pcie_dbg( "dm_pcie_outband_dma_wr para err!\n");
	  return;
	}
	writel(dma_wr_data, pci_base.dm_axislv_dma + 0x1000 + dma_wr_address);
	//writel(dma_wr_data, 0x1000 + dma_wr_address);
	return;
}

/* dma_ch_num范围0~3
   return 0表示成功
   return 1表示失败
 */
u32 dm_pcie_dma_read(u32 dma_ch_num, u32 dma_trans_size, u64 dma_read_sa_addr, u64 dma_read_da_addr) 
{
	u32 temp;
	u32 dma_trans_ok = 0;
	int i = 0;

	if (dma_ch_num >= 4) {
		return 1;
	}
	
	//u32 dma_ch_num =0x0;
	//u32 dma_trans_size = 0x10;
	//u32 dma_read_sa_addr = 0x80000240; //ep
	//u32 dma_read_da_addr = 0x80000140; //rc

	//pcie_dbg( "dm_pcie_dma_read:: DMA Read transfer begin, dma_ch_num=0x%0x, dma_trans_size=0x%0x, dma_read_sa_addr=0x%0llx, dma_read_da_addr=0x%0llx\n",
		//dma_ch_num, dma_trans_size, dma_read_sa_addr, dma_read_da_addr);  

	/* 1. cfg dma_enable */
	temp = read_dm_dma_reg(0x2c); //DMA_READ_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_dm_dma_reg(0x2c, temp);   
	/* 2. cfg dma interupt enable */
	temp = read_dm_dma_reg(0xa8); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffefffe;
	temp = temp | 0x00010001;
	//write_dm_dma_reg(0xa8, temp);
	/* 3. cfg DMA channel control register */
	write_dm_dma_reg(0x300 + dma_ch_num*0x200, 0x04000018); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_dm_dma_reg(0x308 + dma_ch_num*0x200, dma_trans_size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_dm_dma_reg(0x30C + dma_ch_num*0x200, dma_read_sa_addr&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_dm_dma_reg(0x310 + dma_ch_num*0x200, dma_read_sa_addr>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_dm_dma_reg(0x314 + dma_ch_num*0x200, dma_read_da_addr&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_dm_dma_reg(0x318 + dma_ch_num*0x200, dma_read_da_addr>>32); //DMA_DAR_High_OFF_RDCH_%0d
	temp = read_dm_dma_reg(0x38);
	//pcie_dbg( "DMA_READ_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",temp);
	/* 7. cfg Doorbell for start DMA transfer */
	temp = read_dm_dma_reg(0x30); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffffff8;
	temp = temp | (dma_ch_num&0x7);
	write_dm_dma_reg(0x30, temp); //DMA_DAR_High_OFF_RDCH_%0d

	temp = read_dm_dma_reg(0xa0);
	//pcie_dbg( "dm_pcie_dma_read---------------------------------DMA_READ_INT_STATUS_OFF: 0x%x\n",temp);
	while (temp != (0x1 << dma_ch_num)) {
		temp = read_dm_dma_reg(0xa0);
		//pcie_dbg( "dm_pcie_dma_read---------------------------------DMA_READ_INT_STATUS_OFF: 0x%x\n",temp);
		if (temp == (0x10000 << dma_ch_num)) {
			pcie_dbg( "****dm_pcie_dma_read about 0xa0:%x\n", temp);
			write_dm_dma_reg(0xac, temp);
		}
		i++;
		if (i > 100000) {
			pcie_dbg( "rc_pcie_fail when dm_pcie_dma_read 0xa0:%x\n", temp);
			//write_dm_dma_reg(0xac, (0x1 << dma_ch_num));
			write_dm_dma_reg(0xac, (0x10001 << dma_ch_num));
			return -1;
		}
	}
	//write_dm_dma_reg(0xac, (0x1 << dma_ch_num));
	write_dm_dma_reg(0xac, (0x10001 << dma_ch_num));
	
	return dma_trans_ok;
}

/* dma_ch_num范围0~3
   return 0表示成功
   return 1表示失败
 */
u32 dm_pcie_dma_write(u32 dma_ch_num, u32 dma_trans_size, u64 dma_write_sa_addr, u64 dma_write_da_addr) 
{
	u32 temp;
	u32 dma_trans_ok = 0;
	int i = 0;

	if (dma_ch_num >= 4) {
		return 1;
	}
	
	//u32 dma_ch_num =0x0;
	//u32 dma_trans_size = 0x10;
	//u32 dma_write_sa_addr = 0x80000140; //rc
	//u32 dma_write_da_addr = 0x80000240; //ep

	//pcie_dbg( "dm_pcie_dma_write:: DMA Write transfer begin, dma_ch_num=0x%0x, dma_trans_size=0x%0x, dma_write_sa_addr=0x%0llx, dma_write_da_addr=0x%0llx\n",
		//dma_ch_num, dma_trans_size, dma_write_sa_addr, dma_write_da_addr);  

	/* 1. cfg dma_enable */
	temp = read_dm_dma_reg(0xc); //dma_write_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_dm_dma_reg(0xc, temp);   
	/* 2.  cfg dma interupt enable */
	temp = read_dm_dma_reg(0x54); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffefffe;
	temp = temp | 0x00010001;
	//write_dm_dma_reg(0x54, temp);
	/* 3. cfg DMA channel control register */
	write_dm_dma_reg(0x200 + dma_ch_num*0x200, 0x040000018); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_dm_dma_reg(0x208 + dma_ch_num*0x200, dma_trans_size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_dm_dma_reg(0x20C + dma_ch_num*0x200, dma_write_sa_addr&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_dm_dma_reg(0x210 + dma_ch_num*0x200, dma_write_sa_addr>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_dm_dma_reg(0x214 + dma_ch_num*0x200, dma_write_da_addr&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_dm_dma_reg(0x218 + dma_ch_num*0x200, dma_write_da_addr>>32); //DMA_DAR_High_OFF_RDCH_%0d
	temp = read_dm_dma_reg(0x18);
	//pcie_dbg( "DMA_WRITE_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",temp);
	/* 7. cfg Doorbell for start DMA transfer */
	temp = read_dm_dma_reg(0x10); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffffff8;
	temp = temp | (dma_ch_num&0x7);
	write_dm_dma_reg(0x10, temp); //DMA_DAR_High_OFF_RDCH_%0d

	temp = read_dm_dma_reg(0x4c);
	//pcie_dbg( "dm_pcie_dma_write---------------------------------DMA_WRITE_INT_STATUS_OFF: 0x%x\n",temp);
	while (temp != (0x1 << dma_ch_num)) {
		temp = read_dm_dma_reg(0x4c);
		//pcie_dbg( "dm_pcie_dma_write---------------------------------DMA_WRITE_INT_STATUS_OFF: 0x%x\n",temp);
		if (temp == (0x10000 << dma_ch_num)) {
			pcie_dbg( "****dm_pcie_dma_write about 0xa0:%x\n", temp);
			write_dm_dma_reg(0x58, temp);
		}
		i++;
		if (i > 100000) {
			pcie_dbg( "rc_pcie_fail when dm_pcie_dma_write\n");
			write_dm_dma_reg(0x58, (0x10001 << dma_ch_num));
			//write_dm_dma_reg(0x58, (0x1 << dma_ch_num));
			return -1;
		}
	}
	write_dm_dma_reg(0x58, (0x10001 << dma_ch_num));
	//write_dm_dma_reg(0x58, (0x1 << dma_ch_num));

	return dma_trans_ok;
}

/* rc远程控制ep进行dma读，从rc 0x80000640到ep 0x80000540 */
/* dma_ch_num范围0~3
   return 0表示成功
   return 1表示失败
 */
static u32 dm_ctrl_ep_dma_read(u32 dma_ch_num, u32 dma_trans_size, u64 dma_read_sa_addr, u64 dma_read_da_addr)
{
	u32 temp;
	u32 dma_trans_ok = 0;
	int i = 0;

	if (dma_ch_num >= 4) {
		return 1;
	}
	
	//u32 dma_ch_num =0x0;
	//u32 dma_trans_size = 0x10;
	//u32 dma_read_sa_addr = 0x80000640; //rc
	//u32 dma_read_da_addr = 0x80000540; //ep

	pcie_dbg( "dm_ctrl_ep_dma_read:: DMA Read transfer begin, dma_ch_num=0x%0x, dma_trans_size=0x%0x, dma_read_sa_addr=0x%0llx, dma_read_da_addr=0x%0llx\n",
		dma_ch_num, dma_trans_size, dma_read_sa_addr, dma_read_da_addr);  

	/* 1. cfg dma_enable */
	temp = dm_pcie_outband_dma_rd(0x2c); //DMA_READ_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	dm_pcie_outband_dma_wr(0x2c, temp);   
#if 0
	/* 2. cfg dma interupt enable */
	temp = dm_pcie_outband_cfg_rd(DMA_OFF + 0xa8); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffefffe;
	temp = temp | 0x00010001;
	dm_pcie_outband_cfg_wr(DMA_OFF + 0xa8, temp);
#endif
	/* 3. cfg DMA channel control register */
	dm_pcie_outband_dma_wr(0x300 + dma_ch_num*0x200, 0x04000018); //DMA_CH_CONTROL1_OFF_RDCH_%0d，RIE
	/* 4. cfg DMA transfer size */
	dm_pcie_outband_dma_wr(0x308 + dma_ch_num*0x200, dma_trans_size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	dm_pcie_outband_dma_wr(0x30C + dma_ch_num*0x200, dma_read_sa_addr&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	dm_pcie_outband_dma_wr(0x310 + dma_ch_num*0x200, dma_read_sa_addr>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	dm_pcie_outband_dma_wr(0x314 + dma_ch_num*0x200, dma_read_da_addr&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	dm_pcie_outband_dma_wr(0x318 + dma_ch_num*0x200, dma_read_da_addr>>32); //DMA_DAR_High_OFF_RDCH_%0d

	dm_pcie_outband_dma_wr(0xcc, PCIE_TEST_MSI_ADDRESS);//与msi配置一致
	temp = dm_pcie_outband_dma_rd(0xcc);
	pcie_dbg( "dm_ctrl_ep_dma_read---------------------------------0xcc: 0x%x\n",temp);
	dm_pcie_outband_dma_wr(0xd0, 0x0);
	dm_pcie_outband_dma_wr(0xd4, PCIE_TEST_MSI_ADDRESS);//与msi配置一致   
	dm_pcie_outband_dma_wr(0xd8, 0x0);
	temp = dm_pcie_outband_dma_rd(0xdc);
	dm_pcie_outband_dma_wr(0xdc, temp & 0xffff0000);
	
	temp = dm_pcie_outband_dma_rd(0x38);
	pcie_dbg( "DMA_READ_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",temp);
	/* 7. cfg Doorbell for start DMA transfer */
	temp = dm_pcie_outband_dma_rd(0x30); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffffff8;
	temp = temp | (dma_ch_num&0x7);
	dm_pcie_outband_dma_wr(0x30, temp); //DMA_DAR_High_OFF_RDCH_%0d

	temp = dm_pcie_outband_dma_rd(0xa8);
	pcie_dbg( "dm_ctrl_ep_dma_read---------------------------------DMA_READ_INT_MASK_OFF: 0x%x\n",temp);

	temp = dm_pcie_outband_dma_rd(0xa0);
	pcie_dbg( "dm_ctrl_ep_dma_read---------------------------------DMA_READ_INT_STATUS_OFF: 0x%x\n",temp);
	while (temp != (0x1 << dma_ch_num)) {
		temp = dm_pcie_outband_dma_rd(0xa0);
		pcie_dbg( "dm_ctrl_ep_dma_read---------------------------------DMA_READ_INT_STATUS_OFF: 0x%x\n",temp);
		if (temp == (0x10000 << dma_ch_num)) {
			dm_pcie_outband_dma_wr(0xac, temp);
		}
		i++;
		if (i > 100) {
			pcie_dbg( "rc_pcie_fail when dm_ctrl_ep_dma_read\n");
			return 1;
		}
	}
	
	temp = read_dm_reg(RC_MSI_CTRL_INT_EP0_STATUS_OFF); 
	pcie_dbg( "dm_ctrl_ep_dma_read---------------------------------RC_MSI_CTRL_INT_EP0_STATUS_OFF: 0x%x\n",temp);
	if (temp != 0x1) {
		pcie_dbg( "rc_pcie_fail when dm_ctrl_ep_dma_read(msi)\n");
	}
	//write_dm_reg(RC_MSI_CTRL_INT_EP0_STATUS_OFF, 0xffffffff);
	
	return dma_trans_ok;
}

/* rc远程控制ep进行dma写，从ep 0x80000540到rc 0x80000640 */
/* dma_ch_num范围0~3
   return 0表示成功
   return 1表示失败
 */
static u32 dm_ctrl_ep_dma_write(u32 dma_ch_num, u32 dma_trans_size, u64 dma_write_sa_addr, u64 dma_write_da_addr) 
{
	u32 temp;
	u32 dma_trans_ok = 0;
	int i = 0;

	if (dma_ch_num >= 4) {
		return 1;
	}
	
	//u32 dma_ch_num =0x0;
	//u32 dma_trans_size = 0x10;
	//u32 dma_write_sa_addr = 0x80000540; //ep
	//u32 dma_write_da_addr = 0x80000640; //rc

	pcie_dbg( "dm_ctrl_ep_dma_write:: DMA Write transfer begin, dma_ch_num=0x%0x, dma_trans_size=0x%0x, dma_write_sa_addr=0x%0llx, dma_write_da_addr=0x%0llx\n",
		dma_ch_num, dma_trans_size, dma_write_sa_addr, dma_write_da_addr);  

	/* 1. cfg dma_enable */
	temp = dm_pcie_outband_dma_rd(0xc); //dma_write_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	dm_pcie_outband_dma_wr(0xc, temp);
#if 0
	/* 2.  cfg dma interupt enable */
	temp = dm_pcie_outband_cfg_rd(DMA_OFF + 0x54); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffefffe;
	temp = temp | 0x00010001;
	dm_pcie_outband_cfg_wr(DMA_OFF + 0x54, temp);
#endif
	/* 3. cfg DMA channel control register */
	dm_pcie_outband_dma_wr(0x200 + dma_ch_num*0x200, 0x04000018); //DMA_CH_CONTROL1_OFF_RDCH_%0d，RIE
	/* 4. cfg DMA transfer size */
	dm_pcie_outband_dma_wr(0x208 + dma_ch_num*0x200, dma_trans_size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	dm_pcie_outband_dma_wr(0x20C + dma_ch_num*0x200, dma_write_sa_addr&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	dm_pcie_outband_dma_wr(0x210 + dma_ch_num*0x200, dma_write_sa_addr>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	dm_pcie_outband_dma_wr(0x214 + dma_ch_num*0x200, dma_write_da_addr&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	dm_pcie_outband_dma_wr(0x218 + dma_ch_num*0x200, dma_write_da_addr>>32); //DMA_DAR_High_OFF_RDCH_%0d
	
	dm_pcie_outband_dma_wr(0x60, PCIE_TEST_MSI_ADDRESS);//与msi配置一致   
	dm_pcie_outband_dma_wr(0x64, 0x0);
	dm_pcie_outband_dma_wr(0x68, PCIE_TEST_MSI_ADDRESS);//与msi配置一致   
	dm_pcie_outband_dma_wr(0x6c, 0x0);
	temp = dm_pcie_outband_dma_rd(0x70);
	dm_pcie_outband_dma_wr(0x70, temp & 0xffff0000);
	
	temp = dm_pcie_outband_dma_rd(0x18);
	pcie_dbg( "DMA_WRITE_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",temp);
	/* 7. cfg Doorbell for start DMA transfer */
	temp = dm_pcie_outband_dma_rd(0x10); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffffff8;
	temp = temp | (dma_ch_num&0x7);
	dm_pcie_outband_dma_wr(0x10, temp); //DMA_DAR_High_OFF_RDCH_%0d
	
	temp = dm_pcie_outband_dma_rd(0x54);
	pcie_dbg( "dm_ctrl_ep_dma_write---------------------------------DMA_WRITE_INT_MASK_OFF: 0x%x\n",temp);

	temp = dm_pcie_outband_dma_rd(0x4c);
	pcie_dbg( "dm_ctrl_ep_dma_write---------------------------------DMA_WRITE_INT_STATUS_OFF: 0x%x\n",temp);
	while (temp != (0x1 << dma_ch_num)) {
		temp = dm_pcie_outband_dma_rd(0x4c);
		pcie_dbg( "dm_ctrl_ep_dma_write---------------------------------DMA_WRITE_INT_STATUS_OFF: 0x%x\n",temp);
		if (temp == (0x10000 << dma_ch_num)) {
			dm_pcie_outband_dma_wr(0x58, temp);
		}
		i++;
		if (i > 100) {
			pcie_dbg( "rc_pcie_fail when dm_ctrl_ep_dma_write\n");
			return 1;
		}
	}
	
	temp = read_dm_reg(RC_MSI_CTRL_INT_EP0_STATUS_OFF); 
	pcie_dbg( "dm_ctrl_ep_dma_write---------------------------------RC_MSI_CTRL_INT_EP0_STATUS_OFF: 0x%x\n",temp);
	if (temp != 0x1) {
		pcie_dbg( "rc_pcie_fail when dm_ctrl_ep_dma_write(msi)\n");
	}
	//write_dm_reg(RC_MSI_CTRL_INT_EP0_STATUS_OFF, 0xffffffff);
	
	return dma_trans_ok;
}


static void a55_gic_poke_irq(int irqid, u32 offset)
{
	//unsigned long dist_base = A55_GIC_BASE_ADDR + A55_GIC_DIST_OFFSET;
	u32 mask = 1 << (irqid % 32);
	writel_relaxed(mask, pci_base.gic_dist + offset + (irqid / 32) * 4);
}

void a55_gic_unmask_irq(int irqid)
{
	a55_gic_poke_irq(irqid, A55_GIC_DIST_ENABLE_SET);
}
void a55_gic_mask_irq(int irqid)
{
	a55_gic_poke_irq(irqid, A55_GIC_DIST_ENABLE_CLEAR);
}

#define A55_IRQ_ENABLE(__irq) 	a55_gic_unmask_irq(__irq)
#define A55_IRQ_DISABLE(__irq) 	a55_gic_mask_irq(__irq)	

void isr_a55_pcie_x4_msi_ctrl_int(int irq_num)
{
    pcie_dbg( "isr_a55_pcie_x4_msi_ctrl_int: irq_num is %d\n",irq_num);
	A55_IRQ_DISABLE(A55_IRQ_PCIE_X4_MSI_CTRL_INT);
}

typedef struct a55_gic_irq_struct {
	void (*irq_handler)(int irqid);
	int irqid;
	unsigned int irqflags;
	char *irqname;
	unsigned long para;
} a55_gic_irq_t;

#if 0
#define IRQ_DEFINE(__IrqId, __IrqHandle, __para, __IrqName, __IrqFlags)        \
	__attribute__((section(".section_irq_table"))) struct a55_gic_irq_struct   \
		_section_item_##__IrqId##_tlb = {                              \
			.irq_handler = __IrqHandle,                            \
			.irqid = __IrqId,                                      \
			.irqflags = __IrqFlags,                                \
			.irqname = __IrqName,                                  \
			.para = __para,                                        \
		}

IRQ_DEFINE(A55_IRQ_PCIE_X4_MSI_CTRL_INT, isr_a55_pcie_x4_msi_ctrl_int,    A55_IRQ_PCIE_X4_MSI_CTRL_INT, "a55 pcie x4_msi_ctrl_int", A55_ISR_ATTR_A55 | A55_ISR_ATTR_LEVEL);
#endif

static void setup_msi(void)
{
	u32 temp;
	/**
	 * 在ep上通过cfgwr配msi_cap(addr/upper_addr/data),  
	 * 在rc上通过AXI DBI配msi_ctrl_addr_off/msi_ctrl_upper_addr_off/msi_ctrl_int_i_en(mask)_off 
	 * 在rc上通过AXI DBI配msi_caphdr中的MSI_ENABLE位
	 */

	temp = read_dm_reg(PCIE_MISC_CONTROL_1_OFF); 
	temp |= PCIE_DBI_RO_WR_EN;      //置1 bit[0]
	temp &= ~PORT_LOGIC_WR_DISABLE; //清0 bit[22]
	write_dm_reg(PCIE_MISC_CONTROL_1_OFF, temp);
	temp = read_dm_reg(PCIE_MISC_CONTROL_1_OFF); 
	pcie_dbg( "PCIE_MISC_CONTROL_1_OFF: 0x%x\n",temp);
	
	temp = read_dm_reg(PCI_MSI_CAP_OFF); 
	temp &= ~PCI_MSI_64_BIT_ADDR_CAP; //清0 bit[23]
	write_dm_reg(PCI_MSI_CAP_OFF, temp | (0x1 << 16));
	temp = read_dm_reg(PCI_MSI_CAP_OFF); 
	pcie_dbg( "PCI_MSI_CAP_OFF bit16 msi enable: 0x%x\n",temp);
#if 0
	temp = read_dm_reg(PCIE_MISC_CONTROL_1_OFF); 
	temp &= ~PCIE_DBI_RO_WR_EN;    //清0 bit[0]
	temp |= PORT_LOGIC_WR_DISABLE; //置1 bit[22]
	write_dm_reg(PCIE_MISC_CONTROL_1_OFF, temp);
	temp = read_dm_reg(PCIE_MISC_CONTROL_1_OFF); 
	pcie_dbg( "PCIE_MISC_CONTROL_1_OFF: 0x%x\n",temp);
#endif		
	dm_pcie_outband_cfg_wr(PCI_MSI_CAP_OFF + PCI_MSI_ADDRESS_LO, PCIE_TEST_MSI_ADDRESS);
	temp = dm_pcie_outband_cfg_rd(PCI_MSI_CAP_OFF + PCI_MSI_ADDRESS_LO);
	pcie_dbg( "PCI_MSI_ADDRESS_LO: 0x%x\n", temp);

	dm_pcie_outband_cfg_wr(PCI_MSI_CAP_OFF + PCI_MSI_ADDRESS_HI, 0x0);
	temp = dm_pcie_outband_cfg_rd(PCI_MSI_CAP_OFF + PCI_MSI_ADDRESS_HI);
	pcie_dbg( "PCI_MSI_ADDRESS_HI: 0x%x\n", temp);

	temp = dm_pcie_outband_cfg_rd(PCI_MSI_CAP_OFF + PCI_MSI_DATA_64);
	dm_pcie_outband_cfg_wr(PCI_MSI_CAP_OFF + PCI_MSI_DATA_64, temp & 0xffff0000);
	temp = dm_pcie_outband_cfg_rd(PCI_MSI_CAP_OFF + PCI_MSI_DATA_64);
	pcie_dbg( "PCI_MSI_DATA_64: 0x%x\n", temp);
	
	//需要与ep上配置的一致
	write_dm_reg(RC_MSI_CTRL_ADDR_OFF, PCIE_TEST_MSI_ADDRESS);
	temp = read_dm_reg(RC_MSI_CTRL_ADDR_OFF); 
	pcie_dbg( "RC_MSI_CTRL_ADDR_OFF: 0x%x\n",temp);
	//需要与ep上配置的一致
	write_dm_reg(RC_MSI_CTRL_UPPER_ADDR_OFF, 0x0);
	temp = read_dm_reg(RC_MSI_CTRL_UPPER_ADDR_OFF); 
	pcie_dbg( "RC_MSI_CTRL_UPPER_ADDR_OFF: 0x%x\n",temp);
	
	write_dm_reg(RC_MSI_CTRL_INT_EP0_MASK_OFF, 0x0);
	temp = read_dm_reg(RC_MSI_CTRL_INT_EP0_MASK_OFF); 
	pcie_dbg( "RC_MSI_CTRL_INT_EP0_MASK_OFF: 0x%x\n",temp);

	write_dm_reg(RC_MSI_CTRL_INT_EP0_EN_OFF, 0xffffffff);
	temp = read_dm_reg(RC_MSI_CTRL_INT_EP0_EN_OFF); 
	pcie_dbg( "RC_MSI_CTRL_INT_EP0_EN_OFF: 0x%x\n",temp);


	//write_dm_reg(RC_MSI_CTRL_INT_EP0_STATUS_OFF, 0xffffffff);
	temp = read_dm_reg(RC_MSI_CTRL_INT_EP0_STATUS_OFF); 
	pcie_dbg( "*********************************RC_MSI_CTRL_INT_EP0_STATUS_OFF: 0x%x\n",temp);
		
	A55_IRQ_ENABLE(A55_IRQ_PCIE_X4_MSI_CTRL_INT);
	
	/* 注意：时序上需要等这边配置完成后，ep那边再发送msi消息过来，否则不会触发中断。 */
	
	return;
}

/*******************************************************RC test********************************************************/

static void dm_pcie_outband_iatu_intitial(void) 
{
	u32 temp;

	outbound0_cfg.base = PCI_DM_AXISLAVE_REGION_CFG;
	outbound0_cfg.size = PCI_DM_AXISLAVE_REGION_CFG_SIZE - 1;
	outbound0_cfg.target = 0x0;

	outbound1_mem.base = PCI_DM_AXISLAVE_REGION_MEM;
	outbound1_mem.size = PCI_DM_AXISLAVE_REGION_MEM_SIZE - 1;
	outbound1_mem.target = PCI_TEST_MEM; //需要修改dts给pcie留0x80000000-0x80100000这1M内存

	outbound2_dma.base = PCI_DM_AXISLAVE_REGION_DMA;
	outbound2_dma.size = PCI_DM_AXISLAVE_REGION_DMA_SIZE - 1;
	outbound2_dma.target = 0x0;//PC_REMOTE_CFG_EP_DMA; //对端RC通过mem访问EP DMA的基地址，硬件固定死的

    /* STEP A, cfg iATU0 as cfg outband */
	/* 1. 配置 iatu-type is cfg-type bit [4:0] */
    temp = read_dm_atu_reg(0x0);
    temp = temp & 0xffffffe0;
    temp = temp | 0x4;
	write_dm_atu_reg(0x0, temp);
    /* 2. 配置 iatu outband_base_address is 'h0 */
    write_dm_atu_reg(0x8,  outbound0_cfg.base); //low 32bit 
    write_dm_atu_reg(0xc,  0x0); //high 32bit 
    /* 3. 配置 iatu limit */
    write_dm_atu_reg(0x10, outbound0_cfg.base + outbound0_cfg.size); 
    /* 4. 配置 target address */
    write_dm_atu_reg(0x14, outbound0_cfg.target); //low 32bit 
    write_dm_atu_reg(0x18, 0x0); //high 32bit    
    /* 5. 配置 iatu region enable, bit 31 */
    temp = read_dm_atu_reg(0x4);
    temp = temp & 0x7fffffff;
    temp = temp | 0x80000000;
    write_dm_atu_reg(0x4, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step A success\n");
	pcie_dbg( "outbound0_cfg.base=0x%x, outbound0_cfg.size=0x%x, outbound0_cfg.target=0x%x, en=0x%x, type=0x%x\n", 
		outbound0_cfg.base, outbound0_cfg.size, outbound0_cfg.target, read_dm_atu_reg(0x4), read_dm_atu_reg(0x0));
#if 1	
    /* STEP B, cfg iATU1 as memory access */
 	/* 1. 配置 iatu-type is cfg-type bit [4:0] */
    temp = read_dm_atu_reg(0x200);
    temp = temp & 0xffffffe0;
    temp = temp | 0x0;
    write_dm_atu_reg(0x200, temp);
    /* 2. 配置 iatu outband_base_address is 'h0 */
    write_dm_atu_reg(0x208, outbound1_mem.base); //low 32bit 
    write_dm_atu_reg(0x20c, 0x0); //high 32bit 
    /* 3. 配置 iatu limit */
    write_dm_atu_reg(0x210, outbound1_mem.base + outbound1_mem.size); 
    /* 4. 配置 target address */
    write_dm_atu_reg(0x214, outbound1_mem.target); //low 32bit 
    write_dm_atu_reg(0x218, 0x0); //high 32bit    
    /* 5. 配置 iatu region enable, bit 31 */
    temp = read_dm_atu_reg(0x204);
    temp = temp & 0x7fffffff;
    temp = temp | 0x80000000;
	write_dm_atu_reg(0x204, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step B success\n");
	pcie_dbg( "outbound1_mem.base=0x%x, outbound1_mem.size=0x%x, outbound1_mem.target=0x%x, en=0x%x, type=0x%x\n", 
		outbound1_mem.base, outbound1_mem.size, outbound1_mem.target, read_dm_atu_reg(0x204), read_dm_atu_reg(0x200));
#endif
	/* STEP C, cfg iATU2 as memory access */
	/* 1. 配置 iatu-type is cfg-type bit [4:0] */
	temp = read_dm_atu_reg(0x400);
    temp = temp & 0xffffffe0;
    temp = temp | 0x0;
	write_dm_atu_reg(0x400, temp);
    /* 2. 配置 iatu outband_base_address is 'h0 */
    write_dm_atu_reg(0x408,  outbound2_dma.base); //low 32bit 
    write_dm_atu_reg(0x40c,  0x0); //high 32bit 
    /* 3. 配置 iatu limit */
    write_dm_atu_reg(0x410, outbound2_dma.base + outbound2_dma.size); 
    /* 4. 配置 target address */
    write_dm_atu_reg(0x414, outbound2_dma.target); //low 32bit 
    write_dm_atu_reg(0x418, 0x0); //high 32bit    
    /* 5. 配置 iatu region enable, bit 31 */
    temp = read_dm_atu_reg(0x404);
    temp = temp & 0x7fffffff;
    temp = temp | 0x80000000;
    write_dm_atu_reg(0x404, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step C success\n");
	pcie_dbg( "outbound2_dma.base=0x%x, outbound2_dma.size=0x%x, outbound2_dma.target=0x%x, en=0x%x, type=0x%x\n", 
		outbound2_dma.base, outbound2_dma.size, outbound2_dma.target, read_dm_atu_reg(0x404), read_dm_atu_reg(0x400));
	
    /* Step D, CFG MAX-PAYLOAD-SIZE */
    temp = read_dm_reg(0x70 + 0x8);
    temp = temp & 0xffffff1f;
    temp = temp | 0x40; //512byte
    write_dm_reg(0x70 + 0x8, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step D success\n");
	pcie_dbg( "MAX-PAYLOAD-SIZE 0x78:%#x\n", read_dm_reg(0x70 + 0x8));
	
    /* Step E, CFG IO.MSE.enable */
    temp = read_dm_reg(0x4);
    temp = temp & 0xfffffff8;
    temp = temp | 0x7;
    write_dm_reg(0x4, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step E success\n");
	
	return;
}

static void dm_ep_pcie_outband_iatu_intitial(void) 
{
	u32 temp;

	dm_ep_outbound1_mem.base = PCI_DM_AXISLAVE_REGION_MEM;
	dm_ep_outbound1_mem.size = PCI_DM_AXISLAVE_REGION_MEM_SIZE - 1;
	dm_ep_outbound1_mem.target = PCI_TEST_MEM; //需要修改dts给pcie留0x80000000-0x80100000这1M内存

    /* STEP B, cfg iATU1 as memory access */
 	/* 1. 配置 iatu-type is cfg-type bit [4:0] */
    temp = read_dm_atu_reg(0x200);
    temp = temp & 0xffffffe0;
    temp = temp | 0x0;
    write_dm_atu_reg(0x200, temp);
    /* 2. 配置 iatu outband_base_address is 'h0 */
    write_dm_atu_reg(0x208, dm_ep_outbound1_mem.base); //low 32bit 
    write_dm_atu_reg(0x20c, 0x0); //high 32bit 
    /* 3. 配置 iatu limit */
    write_dm_atu_reg(0x210, dm_ep_outbound1_mem.base + dm_ep_outbound1_mem.size); 
    /* 4. 配置 target address */
    write_dm_atu_reg(0x214, dm_ep_outbound1_mem.target); //low 32bit 
    write_dm_atu_reg(0x218, 0x0); //high 32bit    
    /* 5. 配置 iatu region enable, bit 31 */
    temp = read_dm_atu_reg(0x104);
    temp = temp & 0x7fffffff;
    temp = temp | 0x80000000;
	write_dm_atu_reg(0x204, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step B success\n");
	
    /* Step c, CFG MAX-PAYLOAD-SIZE */
    temp = read_dm_reg(0x70 + 0x8);
    temp = temp & 0xffffff1f;
    temp = temp | 0x40; //512byte
    write_dm_reg(0x70 + 0x8, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step C success\n");
	pcie_dbg( "MAX-PAYLOAD-SIZE 0x78:%#x\n", read_dm_reg(0x70 + 0x8));
	
    /* Step D, CFG IO.MSE.enable */
    temp = read_dm_reg(0x4);
    temp = temp & 0xfffffff8;
    temp = temp | 0x7;
    write_dm_reg(0x4, temp);
	pcie_dbg( "dm_pcie_outband_iatu_intitial step D success\n");
	
	return;
}

__attribute__((unused))
static void dm_ep_pcie_outband_mem_wr(u32 mem_wr_address, u32 mem_wr_data) 
{
	u32 iatu_trans_address;
	
	if(mem_wr_address > dm_ep_outbound1_mem.size) {
	  pcie_dbg( "dm_ep_pcie_outband_mem_wr para err!\n");
	  return;
	}
	iatu_trans_address = mem_wr_address;//dm_ep_outbound1_mem.base + mem_wr_address - dm_ep_outbound1_mem.target;
	writel(mem_wr_data, pci_base.dm_axislv_mem + iatu_trans_address);

	return;
}

static char *get_pcie_cap_char(u32 extend_en, u32 cap_id_or_ecap_id) 
{
    if(extend_en == 0x0) {
		switch (cap_id_or_ecap_id) {
		case 0x1:
			return "PCI_PM_CAP_ID";
		case 0x03:
			return "PCI_VPD_ID";
		case 0x04:
			return "PCI_SLOT_ID";
		case 0x05:
			return "PCI_MSI_CAP_ID";
		case 0x10:
			return "PCIe_CAP_ID";
		case 0x11:
			return "PCI_MSIX_CAP_ID";
		case 0x12:
			return "PCI_SATA_ID";
		case 0x13:
 			return "PCI_TYPE0_HDR";           
 		case 0x14:
 			return "PCI_TYPE1_HDR";           
		default:
			pcie_dbg( "get_pcie_cap_char::input error PCIe-CAP-ID\n");
            return "ERROR" ;
        }               
    } else {
		switch (cap_id_or_ecap_id) {
		case 0x0028:
			return "PCIE_PL32G_CAP_ID";
		case 0x0027:
			return "PCIE_MARGIN_CAP_ID";
		case 0x0026:
			return "PCIE_PL16G_CAP_ID";
		case 0x0025:
 			return "DLINK_EXT_CAP_ID";           
 		case 0x0024:
 			return "PCIE_VF_RESIZABLE_BAR_CAP_ID";           
        case 0x0023:
			return "PCIE_DVSEC_CAP_ID";
		case 0x0022:
			return "RTR_EXT_CAP_ID";
		case 0x0021:
			return "FRSQ_EXT_CAP_ID";
		case 0x0020:
 			return "PCIE_MPCIE_CAP_ID";           
 		case 0x001F:
 			return "PTM_EXT_CAP_ID";           
  		case 0x001E:
 			return "PCIE_L1SUB_CAP_ID";           
  		case 0x001D:
 			return "PCIE_DPC_CAP_ID";           
  		case 0x001C:
 			return "PCIE_LNR_CAP_ID";           
  		case 0x001B:
 			return "PCIE_PASID_CAP_ID";           
  		case 0x001A:
 			return "PCIE_PMUX_CAP_ID";           
  		case 0x0019:
 			return "PCIE_SPCIE_CAP_ID";           
  		case 0x0017:
 			return "PCIE_TPH_CAP_ID";           
  		case 0x0018:
 			return "PCIE_LTR_CAP_ID";           
  		case 0x0016:
 			return "PCIE_DPA_CAP_ID";           
  		case 0x0015:
 			return "PCIE_RESIZABLE_BAR_CAP_ID";           
  		case 0x0013:
 			return "PRS_EXT_CAP_ID";           
            break;    	    
  		case 0x0012:
 			return "PCIE_MULTICAST_CAP_ID";           
  		case 0x0011:
 			return "PCIE_MRIOV_CAP_ID";           
  		case 0x0010:
 			return "PCIE_SRIOV_CAP_ID";           
  		case 0x000F:
 			return "PCIE_ATS_CAP_ID";           
   		case 0x000E:
 			return "PCIE_ARI_CAP_ID";           
  		case 0x000D:
 			return "PCIE_ACS_CAP_ID";           
  		case 0x000B:
 			return "PCIE_VENDOR_SPECIFIC_CAP_ID";           
   		case 0x000A:
 			return "PCIE_RCRB_HDR_CAP_ID";           
  		case 0x0008:
 			return "PCIE_MFVC_CAP_ID";           
   		case 0x0007:
 			return "PCIE_RC_EVENT_COLLECTOR_EP_ASSOCIATION_CAP_ID";           
  		case 0x0006:
 			return "PCIE_RC_INTERNAL_INT_LINK_CAP_ID";           
   		case 0x0005:
 			return "PCIE_RC_INTERNAL_LINK_CAP_ID";           
        case 0x0001:
			return "PCIE_AER_CAP_ID";
		case 0x0002:
			return "PCIE_VC_CAP_ID";
		case 0x0003:
			return "PCIE_DSN_CAP_ID";
		case 0x0004:
			return "PCIE_POWER_BUDGET_CAP_ID";
		default:
			pcie_dbg( "get_pcie_cap_char::input error PCIe-ECAP-ID\n");
            return "ERROR";
        }                
    }

	return "ERROR";
}

static void dm_pcie_rc_discovers_capapilites(void) 
{
    u32  in_extended; // Set to 1 when discovering extended capabilities
    u32  base_addr; // Holds the base addr for CFG requests
    u32  data;// Holds the data returned from CFG reads
    u32  offset;// Holds the offset to the next capability
    u32  offset_d; // Holds the offset to the next capability
    u32  func_num = 0;// Used to identify the function
    pcie_t_cap_id    cap_id;            // Used to identify PCI capability types in messages
    pcie_t_ecap_id   ecap_id;           // Used to identify PCIe capability types in messages
    u32  pcie_max_mtu;
    char *ecap_char;
    char *cap_char;

    // The base address of PF0
    base_addr = 0x00000000;
    in_extended = 0x0;

    //------------------------------------------------------------------------------
    // Remote CfgRd0 0x034 in PF0
    // Read Standard Capability Pointer of PF0. Routing ID is 0100h.
    //------------------------------------------------------------------------------
    // Determine the first linked list pointer; Capabilities Pointer

    data = dm_pcie_outband_cfg_rd(base_addr + 0x034);
   	pcie_dbg( "pcie_rc_discovers_capabilities:: Begin Read Standard Capability Pointer of PF0 %0x\n", 
		func_num);  
    offset = data & 0xff ;
    if(offset == 0x0) {
		pcie_dbg( "pcie_rc_discovers_capabilities:: No Standard Capabilities for PF%0d found; unable to continue.\n", 
			func_num);  
		return;
    }
    while (offset != 0x0) {
		//------------------------------------------------------------------------------
		// Read Next Capability Pointer.
		//------------------------------------------------------------------------------
		// Access the next capability and read its header
		data = dm_pcie_outband_cfg_rd(base_addr + (offset & 0xfff));
		if (offset != 0x034) {
			pcie_dbg( "pcie_rc_discovers_capabilities:: Read Next Capability Pointer: addr = 0x%x data = 0x%0x\n",
				base_addr + (offset & 0xfff), data);  
		}
		// 7.9.1. Extended Capabilities in Configuration Space
		// Extended Capabilities in Configuration Space always begin at offset 100h with a PCI Express
		// Extended Capability header (Section 7.9.3). Absence of any Extended Capabilities is required to be
		// indicated by an Extended Capability header with a Capability ID of 0000h, a Capability Version of
		// 0h, and a Next Capability Offset of 000h.
		if(in_extended && ((offset & 0xfff) == 0x100) && (data == 0x0)) {
			pcie_dbg( "pcie_rc_discovers_capabilities:: No Extended Capabilities found; unable to continue.\n");  
			break;
		}
		// We now know the offset of the capability and its name (extracted from the header), so store them in the data structure, and calculate the next pointer
		if(in_extended) {
	        ecap_id = data & 0xffff;//bst_pcie30_subenv_dec::pcie_t_ecap_id'(data[15:0]);
	        ecap_char = get_pcie_cap_char(0x1, ecap_id);
	        pcie_dbg( "pcie_rc_discovers_capabilities:: Discovered EXtend-Capability of PF0:%s\n", ecap_char);  
	        // Assign next offset from data read   
	        offset = (data & 0xfff00000) >> 20; //offset = {4'h0,data[31:20]};
		} else {   
	        cap_id = data & 0xff; //cap_id = bst_pcie30_subenv_dec::pcie_t_cap_id'(data[7:0]);
	        cap_char = get_pcie_cap_char(0x0, cap_id);
	        pcie_dbg( "pcie_rc_discovers_capabilities:: Discovered Capability of PF0:%s\n",cap_char);  
	        // Assign next offset from data read
	        offset_d = offset;    
	        offset = (data & 0xf0) >> 8; //offset = {8'h0,data[15:8]};
	        if(cap_id == 0x10) {
	            //this.bst_remote_pcie_send_cfg_read_trans(base_addr + offset_d[11:0] + 'h4, data, cpl_st);
	            data = dm_pcie_outband_cfg_rd(base_addr + (offset_d & 0xfff) + 0x4);
	            pcie_max_mtu = data & 0x7;
	            pcie_dbg( "pcie_rc_discovers_capabilities:: Discovered Capability of PF0: MAX_MTU=%d\n",pcie_max_mtu);  
	            data = dm_pcie_outband_cfg_rd(base_addr + (offset_d & 0xfff) + 0x8);           
	            data = data & 0xffffff1f;
	            data = data | 0x40; // cfg max_payload_size == 3'b010 data[7:5] = pcie_cap_max_payload_size;
	            dm_pcie_outband_cfg_wr(base_addr + (offset_d & 0xfff) + 0x8, data);           
	            data = dm_pcie_outband_cfg_rd(base_addr + (offset_d & 0xfff) + 0x8);           
	            pcie_dbg( "pcie_rc_discovers_capabilities:: Discovered Capability of PF0: PCIe_CAP_MAX_PAYLOAD_SIZE=%d\n",
					data & 0xe0);           
	        }
	    }
		//------------------------------------------------------------------------------
		// Remote CfgRd0 PF0 PCIE Capability Pointer
		// Discover PCIE Capability of PF0.
		//------------------------------------------------------------------------------
		// This is the last capability in the PCI-compatible register space,
		// now move to discovery of the extended config space
		if ((offset == 0x0) && !in_extended) {
			offset = 0x100;
			in_extended = 0x1;
		}
    } 

	return;
}

static void dm_pcie_rc_configures_bars(void) 
{
    u32 base_addr;      // Holds the base addr for CFG requests
    u32 data;           // Holds the data returned by CFG requests
    u32 size;           // Holds the BAR size from data returned by CFG requests
    u32 bar_16b_addr;   // bit [15:0] Holds the base address value to write to the next IO BAR
    u32 bar_32b_addr;   // Holds the base address value to write to the next 32-bit BAR
    u64 bar_64b_addr;   // bit [63:0]Holds the base address value to write to the next 64-bit BAR
    u32 bar_16b_size;   // bit [15:0]Holds the base address increment for the IO BAR 
    u32 bar_32b_size;   // Holds the base address increment for the 32-bit BARs (largest 32-bit BAR in the PF)
    u64 bar_64b_size;   // Holds the base address increment for the 64-bit BARs (largest 64-bit BAR in the PF

    int pf_mem_bars[]={-1,-1,-1,-1,-1,-1};
    int pf_bar_type[]={-1,-1,-1,-1,-1,-1};
    u64 pf_bar_size[]={-1,-1,-1,-1,-1,-1};
    u64 pf0_bar_addr[6] = {0};

    u32 bar_start_16b = 0x00008000;
    u32 bar_start_32b = 0x80000000;  // Starting address for 32-bit BAR mapped memory space
    u64 bar_start_64b = 0x00000000;  // Starting address for 64-bit BAR mapped memory space

    u32 bar_num;
    
    bar_16b_addr = bar_start_16b;
    bar_32b_addr = bar_start_32b;
    bar_64b_addr = bar_start_64b;
    bar_16b_size = 0;
    bar_32b_size = 0;
    bar_64b_size = 0;

	// Class properties used to hold the BAR number used for memory access
	//------------------------------------------------------------------------------ 
	// Discover all PF and VF BARs
	// - Find the size of each PF BAR
	//------------------------------------------------------------------------------ 
	// The base address for PF0 BARs
	base_addr = 0x00000000 + 0x010;
	pcie_dbg( "dm_pcie_rc_configures_bars::-Writing all 1's to each PF BAR..\n");  
	for (bar_num=0; bar_num <= 5; bar_num++) {
		dm_pcie_outband_cfg_wr(base_addr + (bar_num * 0x004), 0xffffffff);           
		data = dm_pcie_outband_cfg_rd(base_addr + (bar_num * 0x004));                
		pcie_dbg( "dm_pcie_rc_configures_bars::-Data read back from PF BAR%0d: 0x%x..\n", 
			bar_num, data);  
		// Handle the case of even and odd BARs separately 
		if ((bar_num % 0x2) == 0x0) {
			//------------------------------------------------------------------------------
			// Case of an even BAR
			// - Returned data of 0 indicates disabled BAR: size = 0
			// - Returned data of all 1 indicates disabled BAR: size = 0
			// - For valid data: size = Mask off lower 4 bits, invert, then add 1
			//------------------------------------------------------------------------------
			if (data == 0x0) {
				pf_bar_size[bar_num] = 0x0;
				pf_bar_type[bar_num] = 0x0;
			} else if ((data & 0x1) == 0x1) { //if data[0] == 1'b1
				// Case of an I/O BAR
				size = ~(data & 0xfffffff0); // Mask off lower 4 bits, invert 
				pf_bar_size[bar_num] = size + 0x1; // Then add 1 
				pf_bar_type[bar_num] = 0x10;
				bar_16b_size = (pf_bar_size[bar_num] > bar_16b_size) ? pf_bar_size[bar_num] : bar_16b_size; 
			} else {
				size = ~(data & 0xfffffff0); // Mask off lower 4 bits, invert
				pf_bar_size[bar_num] = size + 0x1; // Then add 1
				pf_bar_type[bar_num] = ((data & 0x7) == 0x4) ? 0x40 : 0x20;
				if (pf_bar_type[bar_num] == 0x20) {
					pf_mem_bars[bar_num] = bar_num;
				}
			}
		} else {
			//------------------------------------------------------------------------------
			// Case of an odd BAR
			// - Handled differently depending on role (64-bit vs. 32-bit BAR)
			//------------------------------------------------------------------------------
			if (pf_bar_type[bar_num-1] == 0x40) {
			//------------------------------------------------------------------------------
			// Role as upper half of a 64-bit pair
			// - If bigger than 32 bits: Invert, add 1, shift left by 32
			// - Size stored in lower element of the size array, upper element set to 0
			//------------------------------------------------------------------------------
				if (pf_bar_size[bar_num-1] > 0xffffffff) {
					size = ~data;  // Invert
					pf_bar_size[bar_num-1] = ((long long)(size + 0x1) << 32); // Add 1, shift left by 32
				}
				pf_bar_size[bar_num] = 0x0;
				pf_bar_type[bar_num] = 0x0;
				pcie_dbg( "dm_pcie_rc_configures_bars::-Revised discovered PF BAR%0d size to 0x%llx.\n",
					bar_num-1, pf_bar_size[bar_num-1]);  
				bar_64b_size = (pf_bar_size[bar_num-1] > bar_64b_size) ? pf_bar_size[bar_num-1] : bar_64b_size;
				pf_mem_bars[bar_num-1] = bar_num - 1;
		    } else {
				//------------------------------------------------------------------------------
				// Role as separate 32-bit BAR
				// - Returned data of 0 indicates disabled BAR: size = 0
				// - Returned data of all 1 indicates disabled BAR: size = 0
				// - For valid data: size = Mask off lower 4 bits, invert, then add 1
				//------------------------------------------------------------------------------
				if (data == 0x0) {
				pf_bar_size[bar_num] = 0x0;
				pf_bar_type[bar_num] = 0x0;
				} else if ((data & 0x1)== 0x1) {
					// Case of an I/O BAR
					size = ~(data & 0xfffffff0); // Mask off lower 4 bits, invert 
					pf_bar_size[bar_num] = size + 1; // Then add 1 
					pf_bar_type[bar_num] = 0x10;
					bar_16b_size = (pf_bar_size[bar_num] > bar_16b_size) ? pf_bar_size[bar_num] : bar_16b_size; 
		        } else {
					size = ~(data & 0xfffffff0); // Mask off lower 4 bits, invert
					pf_bar_size[bar_num] = size + 1; // Then add 1
					pf_bar_type[bar_num] = 0x20;
					pf_mem_bars[bar_num] = bar_num;
					bar_32b_size = (pf_bar_size[bar_num-1] > bar_32b_size) ? pf_bar_size[bar_num-1] : bar_32b_size; 
					bar_32b_size = (pf_bar_size[bar_num]   > bar_32b_size) ? pf_bar_size[bar_num]   : bar_32b_size;          
				}
			}
		}
		pcie_dbg( "dm_pcie_rc_configures_bars::-Initial discovered PF BAR%0d size of 0x%llx..\n",
			bar_num, pf_bar_size[bar_num]);  
	}

	// Display discovered BARs
	for (bar_num=0; bar_num <= 5; bar_num++) {
		if (pf_bar_type[bar_num] == 0x40) { 
			pcie_dbg( "dm_pcie_rc_configures_bars::-Discovered 64-bit PF BAR%0d with size: 0x%0llx.\n",
				bar_num, pf_bar_size[bar_num]);         
		} else if (pf_bar_size[bar_num] > 0x0) {
			if(pf_bar_type[bar_num] == 0x20) {
				pcie_dbg( "dm_pcie_rc_configures_bars::-Discovered 32-bit PF BAR%0d with size: 0x%0llx\n",
					bar_num, pf_bar_size[bar_num]);                  
			} else {
				pcie_dbg( "dm_pcie_rc_configures_bars::-Discovered IO PF BAR%0d with size: 0x%0llx\n",
					bar_num, pf_bar_size[bar_num]);
			}
		}
	}

	//------------------------------------------------------------------------------ 
	// Configure all PF0 BARs
	// - Configure all enabled BARs; BAR base addresses based on BAR size
	//------------------------------------------------------------------------------ 
	// The base address for PF0 BARs
	base_addr = 0x0000010;
	pcie_dbg( "dm_pcie_rc_configures_bars::-Configure every enabled PF0 BAR.\n");
	for (bar_num=0; bar_num <= 5; bar_num++) {
		if (pf_bar_type[bar_num] == 0x40) {
			// Assign BAR base address and compute next BAR base address
			pf0_bar_addr[bar_num] = bar_64b_addr;
			bar_64b_addr += bar_64b_size;
			// Configure the BAR
			dm_pcie_outband_cfg_wr(base_addr + ((bar_num+0)*0x4), pf0_bar_addr[bar_num]&0xffffffff);           
			dm_pcie_outband_cfg_wr(base_addr + ((bar_num+1)*0x4), pf0_bar_addr[bar_num]>>32);           
			pcie_dbg( "dm_pcie_rc_configures_bars::-Set 64bit PF BAR%0d base address as: 0x%0llx\n",
				bar_num, pf0_bar_addr[bar_num]);   
			data = dm_pcie_outband_cfg_rd(base_addr + (bar_num * 0x4));                
			pcie_dbg( "dm_pcie_rc_configures_bars::CPU read-back BAR%0d cfg address: 0x%0x\n",
				bar_num, data);   
			data = dm_pcie_outband_cfg_rd(base_addr + ((bar_num+1) * 0x4));                
			pcie_dbg( "dm_pcie_rc_configures_bars::CPU read-back BAR%0d cfg address: 0x%0x\n",
				bar_num+1, data);                                         
		} else if (pf_bar_size[bar_num] > 0x0) {
		    // Assign BAR base address and compute next BAR base address
		    if(pf_bar_type[bar_num] == 0x20) {
				pf0_bar_addr[bar_num] = bar_32b_addr;
				bar_32b_addr += bar_32b_size;
		    }
		    else {
				pf0_bar_addr[bar_num] = bar_16b_addr;
				bar_16b_addr += bar_16b_size;
		    }
		    // Configure the BAR
		    dm_pcie_outband_cfg_wr(base_addr + ((bar_num)*0x4), pf0_bar_addr[bar_num]&0xffffffff);           
		    //pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR%0d base address as: 0x%0llx\n",
			//	bar_num, pf0_bar_addr[bar_num]); 
		}
	}
	
	dm_pcie_outband_cfg_wr(base_addr + ((0)*0x4), 0x0&0xffffffff);  
    dm_pcie_outband_cfg_wr(base_addr + ((1)*0x4), PCI_TEST_MEM&0xffffffff);  
    dm_pcie_outband_cfg_wr(base_addr + ((3)*0x4), 0x10&0xffffffff);  
    dm_pcie_outband_cfg_wr(base_addr + ((5)*0x4), 0x11&0xffffffff); 
	pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR0 base address as: 0x%x\n", dm_pcie_outband_cfg_rd(base_addr + ((0)*0x4))); 
	pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR1 base address as: 0x%x\n", dm_pcie_outband_cfg_rd(base_addr + ((1)*0x4))); 
	pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR2 base address as: 0x%x\n", dm_pcie_outband_cfg_rd(base_addr + ((2)*0x4))); 
	pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR3 base address as: 0x%x\n", dm_pcie_outband_cfg_rd(base_addr + ((3)*0x4))); 
	pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR4 base address as: 0x%x\n", dm_pcie_outband_cfg_rd(base_addr + ((4)*0x4))); 
	pcie_dbg( "dm_pcie_rc_configures_bars::-Set 32bit PF BAR5 base address as: 0x%x\n", dm_pcie_outband_cfg_rd(base_addr + ((5)*0x4))); 
	return;
}

static void dm_pcie_rc_enumeration(u32 kkk) 
{    
    u32 pf0_routing_id = 0x00000000;  /* Base address used for routing PF0 CFG accesses */
    u32 cfg_rd_data;
    u32 device_id;
    u32 vender_id;
	
	pcie_dbg( "A1000 RC PCIe do enumeration with Remote PCIe !!!!\n");
    //----------------------------------------------------------------------------------
    // Configuration transaction from Root
    //
    // NOTE :: As svt_pcie_tl_configuration::enable_register_tracking is set the config
    // transaction on bus will update monitor's configuration space.
    //----------------------------------------------------------------------------------
	pcie_dbg( "A1000 RC PCIe discovers Device_ID and Vendor_ID\n");
    cfg_rd_data = dm_pcie_outband_cfg_rd(pf0_routing_id + 0x0);
    vender_id = cfg_rd_data & 0xffff;
    device_id = (cfg_rd_data & 0xffff0000)>>16;
	pcie_dbg( "Vendor_ID is %0x\n",vender_id);
   	pcie_dbg( "Device_ID is %0x\n",device_id);   
    cfg_rd_data = dm_pcie_outband_cfg_rd(pf0_routing_id + 0xc);
  	pcie_dbg( "Multiple Functions Supported  is %0x\n",cfg_rd_data);
    /* Do enumeration */
    /* 1. discovers_capapility */
    dm_pcie_rc_discovers_capapilites();
#if 1
    /* 2. configures_bars */
    dm_pcie_rc_configures_bars();
    /* 3. Memory Space Enable', and 'IO Space Enable' of PF0. */
    cfg_rd_data = dm_pcie_outband_cfg_rd(pf0_routing_id + 0x4); 
	cfg_rd_data &= 0xfffffff8;
	cfg_rd_data |= 0x7;
    dm_pcie_outband_cfg_wr(pf0_routing_id + 0x4, cfg_rd_data); 
    cfg_rd_data = dm_pcie_outband_cfg_rd(pf0_routing_id + 0x4); 
  	pcie_dbg( "Memory Space Enable is %0x\n",cfg_rd_data);
#endif
	return;
}

static void setup_dm_rc_x4(void)
{
	u32 temp;
	u32 loop;
	u32 pcie_genx;
	u32 pcie_link_width;
	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");	   
		
	/* 2.PCIe PHY 复位 */
	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	write_apb(PCIE_CLK_CTRL, 0xc0); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置bifurcation_en=0,RC MODE=1 */
	write_apb(PCIE_MODE, 0x2); //rc x4
	pcie_dbg( "Configure PCIe As RC mode\n");
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	write_apb(PCIE_PHY_CTRL, 0x6);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	write_apb(PCIE_LOCAL_RST, 0x4); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x3);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */ 
	write_apb(PCIE_LOCAL_RST, 0x5); //解除phy reset, x4 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	write_apb(PCIE_PHY_CTRL , 0x0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 11.a.link-capility 8G select */
	temp = read_dm_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_dm_reg(0xa0 , temp);	

	//11.b.link-capility x4, rc x4配0
	temp = read_dm_reg(0x8c0) & 0xffffff80;
	temp = temp | 0x44 ; // 1->x1 2->x2 4->x4 8->x8
	write_dm_reg(0x8c0 , temp);

	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));

	/* clear write Abort and Done Interrupt mask*/
	write_dm_dma_reg(0x54, 0);
	/* clear read  Abort and Done Interrupt mask*/
	write_dm_dma_reg(0xa8, 0);

	/* 12.配置ltssm_en = 1 */
	write_apb(PCIE_LTSSM_EN , 0x1);
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg(  "Now, do PCIe link training....\n");
	pcie_dbg(  "I am RC x4!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");

	/* cfg iatu as outband cfg and mem access */
	dm_pcie_outband_iatu_intitial();
#if 0
	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xd4);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));	

	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while (((read_dm_reg(0x80) & 0x20000000) == 0)); //TODO
	}
	pcie_dbg( "PCIe DLL is active\n");

	temp = read_dm_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x3){
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	}
	else if(pcie_genx == 0x2){
		pcie_dbg( "PCIe Link Speed is GEN2\n");	
	}
	else {
		pcie_dbg( "Erro PCIe link speed\n"); 		 
	}
	if(pcie_link_width==0x4){
		pcie_dbg( "PCIe Link Width is x4\n");   
	}
	else {
		pcie_dbg( "Erro PCIe link Width\n"); 		 
	}   

	/* cfg iatu as outband cfg and mem access */
	dm_pcie_outband_iatu_intitial();

	mdelay(5000);
	
	dm_pcie_rc_enumeration(1);
#endif
	return;
}

static void setup_dm_ep_x4(void)
{
	u32 temp;
	u32 loop;
	u32 pcie_genx;
	u32 pcie_link_width;
	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");	   
		
	/* 2.PCIe PHY 复位 */
	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	write_apb(PCIE_CLK_CTRL, 0xc0); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置EP MODE */
	write_apb(PCIE_MODE, 0x0); //ep x4
	pcie_dbg( "Configure PCIe As EP mode\n");
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	write_apb(PCIE_PHY_CTRL, 0x6);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	write_apb(PCIE_LOCAL_RST, 0x4); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x3);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */ 
	write_apb(PCIE_LOCAL_RST, 0x5); //解除phy reset, x4 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	write_apb(PCIE_PHY_CTRL , 0x0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 11.a.link-capility 8G select */
	temp = read_dm_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_dm_reg(0xa0 , temp);	

	//11.b.link-capility x4, ep x4配0
	temp = read_dm_reg(0x8c0) & 0xffffff80;
	temp = temp | 0x44 ; // 1->x1 2->x2 4->x4 8->x8
	write_dm_reg(0x8c0 , temp);

	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));

	/* clear write Abort and Done Interrupt mask*/
	write_dm_dma_reg(0x54, 0);
	/* clear read  Abort and Done Interrupt mask*/
	write_dm_dma_reg(0xa8, 0);

	/* 12.配置ltssm_en = 1 */
	write_apb(PCIE_LTSSM_EN , 0x1);   
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg( "Now, do PCIe link training....\n");
	pcie_dbg( "I am EP x4!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");

	dm_ep_pcie_outband_iatu_intitial();

#if 0
	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xd4);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));	

	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
#if 0
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while (((read_dm_reg(0x80) & 0x20000000) == 0)); //TODO
	}
	pcie_dbg( "PCIe DLL is active\n");
#endif
	temp = read_dm_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x3){
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	}
	else if(pcie_genx == 0x2){
		pcie_dbg( "PCIe Link Speed is GEN2\n");	
	}
	else {
		pcie_dbg( "Erro PCIe link speed\n"); 		 
	}
	if(pcie_link_width==0x4){
		pcie_dbg( "PCIe Link Width is x4\n");   
	}
	else {
		pcie_dbg( "Erro PCIe link Width\n"); 		 
	}   
#endif	
	return;
}

/* rc测试包括：
   1.读写本端配置空间，检查
   2.读写对端配置空间，检查
   3.读写对端BAR mem空间，检查
   4.操作本端dma与对端传输数据，且数据正确
 */
static void rc_test_case(void)
{
	u32 temp;
	u32 temp1;
	//int i;
	
	/* 1.读写本端配置空间，检查 */
	temp = read_dm_reg(0xc); //DSP:TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG
	pcie_dbg( "read_dm_reg(0xc): 0x%x\n", temp);
	write_dm_reg(0xc, temp | 0x10);
	pcie_dbg( "write_dm_reg(0xc): 0x%x\n", temp | 0x10);
	temp1 = read_dm_reg(0xc);
	pcie_dbg( "read_dm_reg(0xc): 0x%x\n", temp1);
	if (temp1 != (temp | 0x10)) {
		pcie_dbg( "rc_test_case rc_pcie_fail at step1: local cfg rw!\n");
	}
	
	/* 2.读写对端配置空间，检查 */
	temp = dm_pcie_outband_cfg_rd(0xc); //USP:BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG
	pcie_dbg( "dm_pcie_outband_cfg_rd(0xc): 0x%x\n", temp);
	dm_pcie_outband_cfg_wr(0xc, temp | 0x10);
	pcie_dbg( "dm_pcie_outband_cfg_wr(0xc): 0x%x\n", temp | 0x10);
	temp1 = dm_pcie_outband_cfg_rd(0xc);
	pcie_dbg( "dm_pcie_outband_cfg_rd(0xc): 0x%x\n", temp1);
	if (temp1 != (temp | 0x10)) {
		pcie_dbg( "rc_test_case rc_pcie_fail at step2: remote cfg rw!\n");
	}

	/* 3.读写对端BAR mem空间，检查 */
	temp = dm_pcie_outband_mem_rd(0x20);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x20): 0x%x\n", temp);
	dm_pcie_outband_mem_wr(0x20, 0x87654321);
	pcie_dbg( "dm_pcie_outband_mem_wr(0x20): 0x%x\n", 0x87654321);
	temp1 = dm_pcie_outband_mem_rd(0x20);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x20): 0x%x\n", temp1);
	if (temp1 != 0x87654321) {
		pcie_dbg( "rc_test_case rc_pcie_fail at step3: mem rw!\n");
	}
	dm_pcie_outband_mem_wr(0x20, temp); //恢复原值
	temp = dm_pcie_outband_mem_rd(0x20);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x20): 0x%x\n", temp);

#if 1
	dm_pcie_outband_mem_wr(0x20, 0x87654321);
	dm_pcie_outband_mem_wr(0x24, 0x12345678);
	dm_pcie_dma_read(0, 0x8, 0x80000020, 0x80000140); //ep->>rc
	dm_pcie_dma_write(0, 0x8, 0x80000140, 0x80000040); //rc-->ep
	temp = dm_pcie_outband_mem_rd(0x40);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x40): 0x%x\n", temp);
	if (temp != 0x87654321) {
		pcie_dbg( "!!!!!rc_test_case rc_pcie_fail at step4.1: dma rw!\n");
	}
	temp = dm_pcie_outband_mem_rd(0x44);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x44): 0x%x\n", temp);
	if (temp != 0x12345678) {
		pcie_dbg( "!!!!!rc_test_case rc_pcie_fail at step4.2: dma rw!\n");
	}
#endif	
#if 1
	pcie_dbg( "rc local dma rw test\n");
	/* 4.操作本端dma与对端传输数据，且数据正确(rc 0x140-0x180<--->ep 0x240-0x280)
	     a.先读rc本端mem
	     b.从ep搬数据到rc mem(dma read)
	     c.读rc本端mem
	     d.写rc本端mem为0,1,2,3,4,5....
	     e.从rc搬数据到ep mem(dma write)
	     f.再从ep搬数据到rc mem(dma read again)
	     g.再读rc本端mem,比较是否为0,1,2,3,4,5....
	     h.写rc本端mem为全0，便于多次测试
	 */
#if 0
	pcie_dbg( "a.before dma read, read_dm_mem(0x140-0x180):\n"); //before dma read, rc mem
	for (i = 0; i < 0x40; i += 4) {
		//temp = read_dm_mem(0x140 + i);
		temp = readl(phys_to_virt(0x80000140 + i));
		pcie_dbg( "0x%x:0x%x ", phys_to_virt(0x80000140 + i), temp);
	}
	pcie_dbg( "\n");
	for (i = 0; i < 0x40; i += 4) {
		writel(0xf1, phys_to_virt(0x80000140 + i));
	}
	pcie_dbg( "write to 0xf1\n");
#endif	
	dm_pcie_dma_read(0, 0x40, 0x80000240, 0x80000140); //ep->>rc
	pcie_dbg( "b.dma read from ep\n");
#if 0	
	pcie_dbg( "c.after dma read, read_dm_mem(0x140-0x180):\n"); //after dma read, rc mem
	for (i = 0; i < 0x40; i += 4) {
		//temp = read_dm_mem(0x140 + i);
		temp = readl(phys_to_virt(0x80000140 + i));
		pcie_dbg( "0x%x:0x%x ", phys_to_virt(0x80000140 + i), temp);
	}
	pcie_dbg( "\n");
	
	pcie_dbg( "d.write rc mem\n");
	for (i = 0; i < 0x40; i += 4) {
		//write_dm_mem(0x140 + i, i);
		writel(i, phys_to_virt(0x80000140 + i));
	}
	pcie_dbg( "write to 0,1,2,3\n");
#endif	
	dm_pcie_dma_write(0, 0x40, 0x80000140, 0x80000240); //rc-->ep
	pcie_dbg( "e.dma write to ep\n");
#if 0
	for (i = 0; i < 0x40; i += 4) {
		writel(0x3b, phys_to_virt(0x80000140 + i));
	}
	pcie_dbg( "write to 0x3b\n");
#endif	
	dm_pcie_dma_read(0, 0x40, 0x80000240, 0x80000140); //ep->>rc
	pcie_dbg( "f.dma read from ep again\n");
#if 0	
	pcie_dbg( "g.after dma read, read_dm_mem(0x140-0x180):\n"); //after dma read, rc mem
	for (i = 0; i < 0x40; i += 4) {
		//temp = read_dm_mem(0x140 + i);
		temp = readl(phys_to_virt(0x80000140 + i));
		pcie_dbg( "0x%x:0x%x ", phys_to_virt(0x80000140 + i), temp);
		if (temp != i) {
			pcie_dbg( "!!!!!rc_test_case fail at step4: dma rw!\n");
			break;
		}
	}
	pcie_dbg( "\n");
	
	pcie_dbg( "h.write rc mem to all 0\n");
	for (i = 0; i < 0x40; i += 4) {
		//write_dm_mem(0x140 + i, 0);
		writel(0, phys_to_virt(0x80000140 + i));
	}
#endif
#endif

	pcie_dbg( "outbound0_cfg.base=0x%x, outbound0_cfg.limit=0x%x, outbound0_cfg.target=0x%x, en=0x%x, type=0x%x\n", 
		read_dm_atu_reg(0x8), read_dm_atu_reg(0x10), read_dm_atu_reg(0x14), read_dm_atu_reg(0x4), read_dm_atu_reg(0x0));
	pcie_dbg( "outbound1_mem.base=0x%x, outbound1_mem.limit=0x%x, outbound1_mem.target=0x%x, en=0x%x, type=0x%x\n", 
		read_dm_atu_reg(0x208), read_dm_atu_reg(0x210), read_dm_atu_reg(0x214), read_dm_atu_reg(0x204), read_dm_atu_reg(0x200));
	pcie_dbg( "outbound2_dma.base=0x%x, outbound2_dma.limit=0x%x, outbound2_dma.target=0x%x, en=0x%x, type=0x%x\n", 
		read_dm_atu_reg(0x408), read_dm_atu_reg(0x410), read_dm_atu_reg(0x414), read_dm_atu_reg(0x404), read_dm_atu_reg(0x400));
#if 1
	pcie_dbg( "rc ctrl remote ep dma rw test\n");
	/* 5.操作对端dma与本端传输数据，且数据正确(rc 0x640-0x680<--->ep 0x540-0x580)
		 a.先读ep端mem
		 b.从rc搬数据到ep mem(ep dma read)
		 c.读ep端mem
		 d.写ep端mem为0,1,2,3,4,5....
		 e.从ep搬数据到rc mem(ep dma write)
		 f.再从rc搬数据到ep mem(ep dma read again)
		 g.再读ep端mem,比较是否为0,1,2,3,4,5....
		 h.写ep端mem为全0，便于多次测试
	 */
#if 0
	pcie_dbg( "a.before dma read, read_ep_mem(0x540-0x580):\n"); //before dma read, ep mem
	for (i = 0; i < 0x40; i += 4) {
		temp = dm_pcie_outband_mem_rd(0x540 + i);
		pcie_dbg( "0x%x ", temp);
	}
	pcie_dbg( "\n");
	for (i = 0; i < 0x40; i += 4) {
		dm_pcie_outband_mem_wr(0x540 + i, 0xf2);
	}
#endif	
	dm_ctrl_ep_dma_read(0, 0x40, 0x80000640, 0x80000540); //rc->>ep
	pcie_dbg( "b.dma read from rc\n");
#if 0	
	pcie_dbg( "c.after dma read, read_ep_mem(0x540-0x580):\n"); //after dma read, ep mem
	for (i = 0; i < 0x40; i += 4) {
		temp = dm_pcie_outband_mem_rd(0x540 + i);
		pcie_dbg( "0x%x ", temp);
	}
	pcie_dbg( "\n");
	
	pcie_dbg( "d.write ep mem\n");
	for (i = 0; i < 0x40; i += 4) {
		dm_pcie_outband_mem_wr(0x540 + i, i);
	}
#endif	
	dm_ctrl_ep_dma_write(0, 0x40, 0x80000540, 0x80000640); //ep-->rc
	pcie_dbg( "e.dma write to rc\n");
#if 0	
	for (i = 0; i < 0x40; i += 4) {
		dm_pcie_outband_mem_wr(0x540 + i, 0x3c);
	}
#endif
	dm_ctrl_ep_dma_read(0, 0x40, 0x80000640, 0x80000540); //rc->>ep
	pcie_dbg( "f.dma read from rc again\n");
#if 0	
	pcie_dbg( "g.after dma read, read_ep_mem(0x540-0x580):\n"); //after dma read, ep mem
	for (i = 0; i < 0x40; i += 4) {
		temp = dm_pcie_outband_mem_rd(0x540 + i);
		pcie_dbg( "0x%x ", temp);
		if (temp != i) {
			pcie_dbg( "!!!!!rc_test_case fail at step5: rc ctrl ep dma rw!\n");
			break;
		}
	}
	pcie_dbg( "\n");
	
	pcie_dbg( "h.write ep mem to all 0\n");
	for (i = 0; i < 0x40; i += 4) {
		dm_pcie_outband_mem_wr(0x540 + i, 0);
	}
#endif
#endif

	pcie_dbg( "rc_test_case success!\n");

	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	return;
}

__attribute__((unused))
static void dm_rc_x4_sideA(void)
{
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "dm_rc_x4_sideA: step 1\n");
	ioremap_rcx4();

  	/* 2.配置dm为rc x4，及其它初始化，包括phy */
	pcie_dbg( "dm_rc_x4_sideA: step 2\n");
	setup_dm_rc_x4();

	setup_msi();

	/* 3.rc测试 */
	pcie_dbg( "dm_rc_x4_sideA: step 3--rc test\n");
	rc_test_case();
	
	return;
}

__attribute__((unused))
static void dm_ep_x4_sideB(void)
{
	//u32 loop = 0;
	//int i;
		
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "dm_ep_x4_sideB: step 1\n");
	ioremap_epx4();
	
	/* 2.配置dm为ep x4，及其它初始化，包括phy */
	pcie_dbg( "dm_ep_x4_sideB: step 2\n");
	setup_dm_ep_x4();

	dm_ep_pcie_outband_iatu_intitial();
	pcie_dbg( "dm_ep_pcie_outband_iatu_intitial\n"); 
#if 0	
	while(1) {
		if (loop > 0xf0000000) {
			break;
		}
		loop++;
	}
	for(i=5000; i>0; i--) {
		dm_ep_pcie_outband_mem_wr(PCIE_TEST_MSI_ADDRESS, 0x1);
		pcie_dbg( "dm_ep_pcie_outband_mem_wr(PCIE_TEST_MSI_ADDRESS, 0x1)\n"); 
		dm_ep_pcie_outband_mem_wr(PCIE_TEST_MSI_ADDRESS, 0x0);
		pcie_dbg( "dm_ep_pcie_outband_mem_wr(PCIE_TEST_MSI_ADDRESS, 0x0)\n");
	}
#endif
	return; 
}

/*******************************************************EP test********************************************************/

static u32 ep_pcie_outband_mem_rd(u32 mem_rd_address) 
{
	u32 iatu_trans_address;
	
	if(mem_rd_address > ep_outbound1_mem.size) {
		pcie_dbg( "ep_pcie_outband_mem_rd para err!\n");
		return 0;
	}
	iatu_trans_address = mem_rd_address;//ep_outbound1_mem.base + mem_rd_address - ep_outbound1_mem.target;
	return readl(pci_base.ep_axislv_mem + iatu_trans_address);
}

static void ep_pcie_outband_mem_wr(u32 mem_wr_address, u32 mem_wr_data) 
{
	u32 iatu_trans_address;
	
	if(mem_wr_address > ep_outbound1_mem.size) {
	  pcie_dbg( "ep_pcie_outband_mem_wr para err!\n");
	  return;
	}
	iatu_trans_address = mem_wr_address;//ep_outbound1_mem.base + mem_wr_address - ep_outbound1_mem.target;
	writel(mem_wr_data, pci_base.ep_axislv_mem + iatu_trans_address);

	return;
}

void print_reg(void)
{
	//int i = 0;
	//u32 off = 0;
	pcie_dbg( "dma: 0xa0:%x, 0xac:%x, 0xb4:%x, 0xb8:%x\n", 
		read_ep_dma_reg(0xa0), read_ep_dma_reg(0xac), read_ep_dma_reg(0xb4), read_ep_dma_reg(0xb8));

#if 0
	for (i = 0; i < 9; i++) {
		pcie_dbg( "ARE 0x%x:%x\n", 0x104+off, read_ep_reg(0x104+off));
		off += 4;
	}	
	pcie_dbg( "apb: 0x158:%x, 0X12C:%x, 0X124:%x, 0x104:%x\n", 
		read_apb(0x158), read_apb(0X12C), read_apb(0X124), read_apb(0x104)) ;
	pcie_dbg( "apb: link 0xec:%x\n", read_apb(0xec));

	off = 0;
	for (i = 0; i < 17; i++) {
		pcie_dbg( "PCIE_DEBUG 0x%x:%x\n", 0xc0+off, read_apb(0xc0+off));
		off += 4;
	}
#endif
}

/* dma_ch_num范围0~3
   return 0表示成功
   return 1表示失败
 */
u32 ep_pcie_dma_read(u32 dma_ch_num, u32 dma_trans_size, u64 dma_read_sa_addr, u64 dma_read_da_addr) 
{
	u32 temp;
	u32 dma_trans_ok = 0;
	int i = 0;

	if (!link_status || dma_ch_num >= 4 || !dma_trans_size || !dma_read_sa_addr || !dma_read_da_addr) {
		pcie_dbg( "ep_dma_read error: link:%d ch=0x%0x, size=0x%0x, sa_addr=0x%0llx, da_addr=0x%0llx\n",
			link_status, dma_ch_num, dma_trans_size, dma_read_sa_addr, dma_read_da_addr);
		return 1;
	}

	//u32 dma_ch_num =0x0;
	//u32 dma_trans_size = 0x10;
	//u32 dma_read_sa_addr = 0x80000440; //rc
	//u32 dma_read_da_addr = 0x80000340; //ep

	//pcie_dbg( "ep_pcie_dma_read:: DMA Read transfer begin, dma_ch_num=0x%0x, dma_trans_size=0x%0x, dma_read_sa_addr=0x%0llx, dma_read_da_addr=0x%0llx\n",
		//dma_ch_num, dma_trans_size, dma_read_sa_addr, dma_read_da_addr);  
#if 0
	/* 1. cfg dma_enable */
	temp = read_ep_dma_reg(0x2c); //DMA_READ_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_ep_dma_reg(0x2c, temp);  
	/* 2. cfg dma interupt enable */
	temp = read_ep_dma_reg(0xa8); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffefffe;
	temp = temp | 0x00010001;
	//write_ep_dma_reg(0xa8, temp);
#endif
	spin_lock(&dma_rlock);
	/* 3. cfg DMA channel control register */
	write_ep_dma_reg(0x300 + dma_ch_num*0x200, 0x04000018); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_ep_dma_reg(0x308 + dma_ch_num*0x200, dma_trans_size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_ep_dma_reg(0x30C + dma_ch_num*0x200, dma_read_sa_addr&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_ep_dma_reg(0x310 + dma_ch_num*0x200, dma_read_sa_addr>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_ep_dma_reg(0x314 + dma_ch_num*0x200, dma_read_da_addr&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_ep_dma_reg(0x318 + dma_ch_num*0x200, dma_read_da_addr>>32); //DMA_DAR_High_OFF_RDCH_%0d
	temp = read_ep_dma_reg(0x38);
	//pcie_dbg( "DMA_READ_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",temp);
	/* 7. cfg Doorbell for start DMA transfer */
	//spin_lock(&dma_rlock);
	temp = read_ep_dma_reg(0x30); //DMA_READ_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffffff8;
	temp = temp | (dma_ch_num&0x7);
	write_ep_dma_reg(0x30, temp); //DMA_DAR_High_OFF_RDCH_%0d

#if PCIE_DMA_USEIRQ
	wait_event_interruptible(read_wq, (read_dma_reg == (0x1 << dma_ch_num)));
	if (read_dma_reg & (0x10000 << dma_ch_num))
		pcie_dbg("***** DMA about int a0:%x ch:%d\n", read_dma_reg, dma_ch_num);
	read_dma_reg = read_dma_reg & ~(0x10001 << dma_ch_num);
		
#else
	temp = read_ep_dma_reg(0xa0);
	while (temp != (0x1 << dma_ch_num)) {
		temp = read_ep_dma_reg(0xa0);
		if (temp == (0x10000 << dma_ch_num)) {	
			pcie_dbg("***** DMA about int a0:%x ch:%d\n", temp, dma_ch_num);
			write_ep_dma_reg(0xac, temp);
		}

		if (i++ > 100000) {
			pcie_dbg("***** DMA timeout ch%d \n", dma_ch_num);
			spin_unlock(&dma_rlock);
			write_ep_dma_reg(0xac, (0x10001 << dma_ch_num));
			return 1; //fail
		}
	}	
	write_ep_dma_reg(0xac, (0x10001 << dma_ch_num));
#endif
	spin_unlock(&dma_rlock);
	
	return dma_trans_ok;
}

/* dma_ch_num范围0~3
   return 0表示成功
   return 1表示失败
 */
u32 ep_pcie_dma_write(u32 dma_ch_num, u32 dma_trans_size, u64 dma_write_sa_addr, u64 dma_write_da_addr) 
{
	u32 temp;
	u32 dma_trans_ok = 0;
	int i = 0;

	if (!link_status || dma_ch_num >= 4 || !dma_trans_size || !dma_write_sa_addr || !dma_write_da_addr) {
		pcie_dbg( "ep_dma_write error: link:%d ch=0x%0x, size=0x%0x, sa_addr=0x%0llx, da_addr=0x%0llx\n",
			link_status, dma_ch_num, dma_trans_size, dma_write_sa_addr, dma_write_da_addr);
		return 1;
	}

	//u32 dma_ch_num =0x0;
	//u32 dma_trans_size = 0x10;
	//u32 dma_write_sa_addr = 0x80000340; //ep
	//u32 dma_write_da_addr = 0x80000440; //rc

	//pcie_dbg( "ep_pcie_dma_write:: DMA Write transfer begin, dma_ch_num=0x%0x, dma_trans_size=0x%0x, dma_write_sa_addr=0x%0llx, dma_write_da_addr=0x%0llx\n",
		//dma_ch_num, dma_trans_size, dma_write_sa_addr, dma_write_da_addr);  
#if 0
	/* 1. cfg dma_enable */
	temp = read_ep_dma_reg(0xc); //dma_write_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_ep_dma_reg(0xc, temp);  
	/* 2.  cfg dma interupt enable */
	temp = read_ep_dma_reg(0x54); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffefffe;
	temp = temp | 0x00010001;
	//write_ep_dma_reg(0x54, temp);
#endif
	spin_lock(&dma_wlock);
	/* 3. cfg DMA channel control register */
	write_ep_dma_reg(0x200 + dma_ch_num*0x200, 0x04000018); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 4. cfg DMA transfer size */
	write_ep_dma_reg(0x208 + dma_ch_num*0x200, dma_trans_size); //DMA_CH_CONTROL1_OFF_RDCH_%0d
	/* 5. cfg DMA Source Addr */
	write_ep_dma_reg(0x20C + dma_ch_num*0x200, dma_write_sa_addr&0xffffffff); // DMA_SAR_LOW_OFF_RDCH_%0d
	write_ep_dma_reg(0x210 + dma_ch_num*0x200, dma_write_sa_addr>>32); //DMA_SAR_High_OFF_RDCH_%0d
	/* 6. cfg DMA Direct Addr */
	write_ep_dma_reg(0x214 + dma_ch_num*0x200, dma_write_da_addr&0xffffffff); // DMA_DAR_LOW_OFF_RDCH_%0d
	write_ep_dma_reg(0x218 + dma_ch_num*0x200, dma_write_da_addr>>32); //DMA_DAR_High_OFF_RDCH_%0d
	temp = read_ep_dma_reg(0x18);
	//pcie_dbg( "DMA_WRITE_CHANNEL_ARB_WEIGHT_LOW_OFF: 0x%x\n",temp);
	/* 7. cfg Doorbell for start DMA transfer */	
	//spin_lock(&dma_wlock);
	temp = read_ep_dma_reg(0x10); //dma_write_INT_MASK_OFF bit0 and bit 16->1
	temp = temp & 0xfffffff8;
	temp = temp | (dma_ch_num&0x7);
	write_ep_dma_reg(0x10, temp); //DMA_DAR_High_OFF_RDCH_%0d

#if PCIE_DMA_USEIRQ
	wait_event_interruptible(write_wq, (write_dma_reg == (0x1 << dma_ch_num)));
	if (write_dma_reg & (0x10000 << dma_ch_num))
		pcie_dbg("***** DMA about int ac:%x ch:%d\n", write_dma_reg, dma_ch_num);
	write_dma_reg = write_dma_reg & ~(0x10001 << dma_ch_num);
	
#else
	temp = read_ep_dma_reg(0x4c);
	while (temp != (0x1 << dma_ch_num)) {
		temp = read_ep_dma_reg(0x4c);
		if (temp == (0x10000 << dma_ch_num)) {
			pcie_dbg( "****** write about 4c:%x ch:%d\n", temp, dma_ch_num);
			write_ep_dma_reg(0x58, temp);
		}

		if (i++ > 100000) {
			pcie_dbg( "****** timeout ch%d \n", dma_ch_num);
			spin_unlock(&dma_wlock);
			write_ep_dma_reg(0x58, (0x10001 << dma_ch_num));
			return -1; //fail
		}
	}
	write_ep_dma_reg(0x58, (0x10001 << dma_ch_num));
#endif
	spin_unlock(&dma_wlock);
	
	return dma_trans_ok;
}

static void ep_pcie_outband_iatu_intitial(void) 
{
	u32 temp;

	ep_outbound1_mem.base = PCI_EP_AXISLAVE_REGION_MEM;
	ep_outbound1_mem.size = PCI_EP_AXISLAVE_REGION_MEM_SIZE - 1;
	ep_outbound1_mem.target = PCI_TEST_MEM; //需要修改dts给pcie留0x80000000-0x80100000这1M内存
	pcie_dbg( "ep_pcie_outband_iatu_intitial step A success\n");
	
    /* STEP B, cfg iATU1 as memory access */
 	/* 1. 配置 iatu-type is cfg-type bit [4:0] */
    temp = read_ep_atu_reg(0x200);
    temp = temp & 0xffffffe0;
    temp = temp | 0x0;
    write_ep_atu_reg(0x200, temp);
    /* 2. 配置 iatu outband_base_address is 'h0 */
    write_ep_atu_reg(0x208, ep_outbound1_mem.base); //low 32bit 
    write_ep_atu_reg(0x20c, 0x0); //high 32bit 
    /* 3. 配置 iatu limit */
    write_ep_atu_reg(0x210, ep_outbound1_mem.base + ep_outbound1_mem.size); 
    /* 4. 配置 target address */
    write_ep_atu_reg(0x214, ep_outbound1_mem.target); //low 32bit 
    write_ep_atu_reg(0x218, 0x0); //high 32bit    
    /* 5. 配置 iatu region enable, bit 31 */
    temp = read_ep_atu_reg(0x104);
    temp = temp & 0x7fffffff;
    temp = temp | 0x80000000;
    write_ep_atu_reg(0x204, temp);
	pcie_dbg( "ep_pcie_outband_iatu_intitial step B success\n");
   
    /* Step c, CFG MAX-PAYLOAD-SIZE */
	temp = read_ep_reg(0x70 + 0x8);
    temp = temp & 0xffffff1f;
    temp = temp | 0x40; //512byte
    write_ep_reg(0x70 + 0x8, temp);
	pcie_dbg( "ep_pcie_outband_iatu_intitial step C success\n");
	
    /* Step D, CFG IO.MSE.enable */
    temp = read_ep_reg(0x4);
    temp = temp & 0xfffffff8;
    temp = temp | 0x7;
    write_ep_reg(0x4, temp);
	pcie_dbg( "ep_pcie_outband_iatu_intitial step D success\n");
	
	return;
}

__attribute__((unused))
static void setup_ep_ep_x2_flip(void)
{
	u32 temp;
	u32 loop;
	u32 pcie_genx;
	u32 pcie_link_width;
	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");	   
		
	/* 2.PCIe PHY 复位 */
	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	write_apb(PCIE_CLK_CTRL, 0xc0); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置EP MODE */
	write_apb(PCIE_MODE, 0x1); //ep x2
	pcie_dbg( "Configure PCIe As EP mode\n");

	write_apb(PCIE_LANEX_LINK_NUM , 0x1100); //lane2和3
	write_apb(PCIE_PHY_SRC_SEL , 0x50);
	write_apb(PCIE_LANE_FLIP_EN , 0xf);
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	write_apb(PCIE_PHY_CTRL, 0x6);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	write_apb(PCIE_LOCAL_RST, 0x4); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x3);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */ 
	write_apb(PCIE_LOCAL_RST, 0x6); //解除phy reset, x2 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	write_apb(PCIE_PHY_CTRL , 0x0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 11.a.link-capility 8G select */
	temp = read_ep_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_ep_reg(0xa0 , temp);	
#if 0
	//11.b.link-capility x2, ep x2配0
	temp = read_ep_reg(0x8c0) & 0xffffff80;
	write_ep_reg(0x8c0 , temp);		
#endif
	/* 12.配置ltssm_en = 1 */
	write_apb(PCIE_LTSSM_EN , 0x3);   
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg( "Now, do PCIe link training....\n");
	pcie_dbg(  "I am ep x2!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");

	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xec); //ep控制器用0xec
			//pcie_dbg( "0xec=0x%x", temp);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	write_ep_reg(0x80C, read_ep_reg(0x80C) | (0x1 << 17));	

	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
#if 0
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while (((read_ep_reg(0x80) & 0x20000000) == 0)); //TODO
	}
	pcie_dbg( "PCIe DLL is active\n");
#endif
	temp = read_ep_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x3){
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	}
	else {
		pcie_dbg( "Erro PCIe link speed\n"); 
		while(1) {
			temp = read_ep_reg(0x80);
			pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
			if(pcie_genx == 0x3){
				break;
			}
		}
	}
	if(pcie_link_width==0x2){
		pcie_dbg( "PCIe Link Width is x2\n");   
	}
	else {
		pcie_dbg( "Erro PCIe link Width\n"); 
		while(1) {
			temp = read_ep_reg(0x80);
			pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]
			if(pcie_link_width == 0x2){
				break;
			}
		}
	}
	
	ep_pcie_outband_iatu_intitial();
	pcie_dbg( "ep_pcie_outband_iatu_intitial\n"); 

	return;
}

__attribute__((unused))
static void setup_dm_rc_x2(void)
{
	u32 temp;
	u32 loop;
	u32 pcie_genx;
	u32 pcie_link_width;
	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");	   
		
	/* 2.PCIe PHY 复位 */
	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	write_apb(PCIE_CLK_CTRL, 0xc0); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置bifurcation_en=0,RC MODE=1 */
	write_apb(PCIE_MODE, 0x2); //rc x2
	pcie_dbg( "Configure PCIe As RC mode\n");
	
	//write_apb(PCIE_LANEX_LINK_NUM , 0x1100);
	//write_apb(PCIE_PHY_SRC_SEL , 0x50);
	//write_apb(PCIE_LANE_FLIP_EN , 0xf);
	write_apb(PCIE_LANE_FLIP_EN , 0xf);
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	write_apb(PCIE_PHY_CTRL, 0x6);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	write_apb(PCIE_LOCAL_RST, 0x4); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x3);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */ 
	write_apb(PCIE_LOCAL_RST, 0x5); //解除phy reset, x2 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	write_apb(PCIE_PHY_CTRL , 0x0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 11.a.link-capility 8G select */
	temp = read_dm_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_dm_reg(0xa0 , temp);	
	pcie_dbg( "Configure gen3\n");

	//11.b.link-capility x2, rc x2配0x42
	temp = read_dm_reg(0x8c0) & 0xffffff80;
	temp = temp | 0x42 ; // 1->x1 2->x2 4->x4 8->x8
	write_dm_reg(0x8c0 , temp);		
	pcie_dbg( "Configure 0x42\n");
	
	/* 12.配置ltssm_en = 1 */
	write_apb(PCIE_LTSSM_EN , 0x3);   
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg( "Now, do PCIe link training....\n");
	pcie_dbg(  "I am RC x2!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");

	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xd4);
			//pcie_dbg( "0xd4=0x%x", temp);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));	

	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while (((read_dm_reg(0x80) & 0x20000000) == 0)); //TODO
	}
	pcie_dbg( "PCIe DLL is active\n");

	temp = read_dm_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16;      // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x3){
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	} else {
		pcie_dbg( "Erro PCIe link speed\n");
		while(1) {
			temp = read_dm_reg(0x80);
			pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
			if(pcie_genx == 0x3){
				break;
			}
		}
	}
	if(pcie_link_width==0x2){
		pcie_dbg( "PCIe Link Width is x2\n");   
	} else {
		pcie_dbg( "Erro PCIe link Width\n");
		while(1) {
			temp = read_dm_reg(0x80);
			pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]
			if(pcie_link_width == 0x2){
				break;
			}
		}
	}

	/* cfg iatu as outband cfg and mem access */
	dm_pcie_outband_iatu_intitial();

	mdelay(5000);
	
	dm_pcie_rc_enumeration(1);
	
	return;
}

static void setup_dm_ep_x2(void)
{
	u32 temp;
	u32 loop;
	u32 pcie_genx;
	u32 pcie_link_width;
	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");	   
		
	/* 2.PCIe PHY 复位 */
	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	write_apb(PCIE_CLK_CTRL, 0xc0); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置EP MODE */
	write_apb(PCIE_MODE, 0x1); //ep x2
	pcie_dbg( "Configure PCIe As EP mode\n");
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	write_apb(PCIE_PHY_CTRL, 0x6);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	write_apb(PCIE_LOCAL_RST, 0x4); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x3);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */ 
	write_apb(PCIE_LOCAL_RST, 0x5); //解除phy reset, x4 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	write_apb(PCIE_PHY_CTRL , 0x0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 11.a.link-capility 8G select */
	temp = read_dm_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_dm_reg(0xa0 , temp);	
	pcie_dbg( "Configure gen3\n");

	//11.b.link-capility x2, rc x2配0x42
	temp = read_dm_reg(0x8c0) & 0xffffff80;
	temp = temp | 0x42 ; // 1->x1 2->x2 4->x4 8->x8
	write_dm_reg(0x8c0 , temp);		
	pcie_dbg( "Configure 0x42\n");

	/* clear write Abort and Done Interrupt mask*/
	write_dm_dma_reg(0x54, 0);
	/* clear read  Abort and Done Interrupt mask*/
	write_dm_dma_reg(0xa8, 0);
	
	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));
	/* 12.配置ltssm_en = 1 */
	write_apb(PCIE_LTSSM_EN , 0x3);   
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg( "Now, do PCIe link training....\n");
	pcie_dbg(  "I am EP x2!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");

	dm_ep_pcie_outband_iatu_intitial();

#if 0
	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xd4);
			//pcie_dbg( "0xd4=0x%x", temp);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));	
	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
#if 0
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while (((read_dm_reg(0x80) & 0x20000000) == 0)); //TODO
	}
	pcie_dbg( "PCIe DLL is active\n");
#endif
	temp = read_dm_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16;      // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x3){
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	} else {
		pcie_dbg( "Erro PCIe link speed %d\n", pcie_genx);
	}
	if(pcie_link_width==0x2){
		pcie_dbg( "PCIe Link Width is x2\n");   
	} else {
		pcie_dbg( "Erro PCIe link Width %d\n", pcie_link_width);
	}
#endif
	return;
}


static void fad_setup_dm_rc_x2(void)
{
	u32 temp;
	//u32 loop, pcie_genx, pcie_link_width;
	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");	   
		
	/* 2.PCIe PHY 复位 */
	write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	write_apb(PCIE_CLK_CTRL, 0xc0); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置bifurcation_en=0,RC MODE=1 */
	write_apb(PCIE_MODE, 0x3); //rc x2
	pcie_dbg( "Configure PCIe As RC mode\n");
	
	//write_apb(PCIE_LANEX_LINK_NUM , 0x11);
	//write_apb(PCIE_PHY_SRC_SEL , 0x5);
	//write_apb(PCIE_LANE_FLIP_EN , 0xc);
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	write_apb(PCIE_PHY_CTRL, 0x6);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	write_apb(PCIE_LOCAL_RST, 0x4); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	write_apb(PCIE_PHY_SRAM_CTRL, 0x3);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */ 
	write_apb(PCIE_LOCAL_RST, 0x5); //解除phy reset, x2 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	write_apb(PCIE_PHY_CTRL , 0x0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 11.a.link-capility 8G select */
	temp = read_dm_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_dm_reg(0xa0 , temp);	
	pcie_dbg( "Configure gen a0:%x\n", read_dm_reg(0xa0));
#if 1
	//11.b.link-capility x2, rc x2配0x42
	temp = read_dm_reg(0x8c0) & 0xffffff80;
	temp = temp | 0x42 ; // 1->x1 2->x2 4->x4 8->x8
	write_dm_reg(0x8c0 , temp);		
	pcie_dbg( "Configure 0x42\n");
#endif
	temp = read_dm_reg(0x80c) & 0xffffe0ff;
    temp |= 0x2 << 8;
	write_dm_reg(0x80c , temp);	

	//write_dm_reg(0X894, 0X608);
	//write_dm_reg(0X890, 0X3001);
	//write_dm_reg(0X8A8, 0X503FC61);

	//temp = read_dm_reg(0x8e8);
    //temp = (temp & (~0x78780000)) | 0x7878;
	//write_dm_reg(0X8e8, temp);

	/* clear write Abort and Done Interrupt mask*/
	write_dm_dma_reg(0x54, 0);
	/* clear read  Abort and Done Interrupt mask*/
	write_dm_dma_reg(0xa8, 0);
	
	/* 12.配置ltssm_en = 1 */
	write_apb(PCIE_LTSSM_EN , 0x3);   
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg( "Now, do PCIe link training....\n");
	pcie_dbg(  "I am RC x2!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");
	
	/* 14.配置 auto-speed-change = 0 */
	write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));	

	/* cfg iatu as outband cfg and mem access */
	dm_pcie_outband_iatu_intitial();
#if 0
	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xd4);
			//pcie_dbg( "0xd4=0x%x", temp);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	//write_dm_reg(0x80C, read_dm_reg(0x80C) | (0x1 << 17));	

	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while ((read_dm_reg(0x80) & 0x20000000) == 0) //TODO
		{
			pcie_dbg( "** 0x80:%x\n", read_dm_reg(0x80));
		}
	}
	pcie_dbg( "PCIe DLL is active\n");

	temp = read_dm_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16;      // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x1){
		pcie_dbg( "PCIe Link Speed is GEN1\n");	
	} else if (pcie_genx == 0x2) {
		pcie_dbg( "PCIe Link Speed is GEN2\n");	
	} else if (pcie_genx == 0x3) {
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	} else {
		pcie_dbg( "Erro PCIe link speed\n");
		while(1) {
			temp = read_dm_reg(0x80);
			pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
			if(pcie_genx == 0x3){
				break;
			}
		}
	}
	if(pcie_link_width==0x2){
		pcie_dbg( "PCIe Link Width is x2\n");   
	} else {
		pcie_dbg( "Erro PCIe link Width 0x%x\n", pcie_link_width);
		while(1) {
			temp = read_dm_reg(0x80);
			pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]
			if(pcie_link_width == 0x2){
				break;
			}
		}
	}

	/* cfg iatu as outband cfg and mem access */
	dm_pcie_outband_iatu_intitial();

	mdelay(5000);
	
	dm_pcie_rc_enumeration(1);
#endif 	
	return;
}

static void fad_setup_ep_ep_x2_flip(void)
{
	//u32 loop, pcie_genx, pcie_link_width;
	u32 reg_val, temp;
#if PCIE_TEST	
	/* 1.PCIe 软复位 */
	write_sw_rst(read_sw_rst() & (~(0x1 << 8)));
	write_sw_rst(read_sw_rst() | ((0x1<< 8)));
	pcie_dbg( "Release PCIe Soft-reset from top_crm\n");
	
	/* 2.PCIe PHY 复位 */
	reg_val = read_apb(PCIE_LOCAL_RST);
	reg_val = reg_val & (0x0);
	write_apb(PCIE_LOCAL_RST, reg_val);
	pcie_dbg( "Reset PCIe PHY and Controller\n");

	/* 3. 配置PHY clk: phy0_use_pad0 = 1 */
	reg_val = read_apb(PCIE_CLK_CTRL);
	reg_val = reg_val | 0xc0;
	write_apb(PCIE_CLK_CTRL, reg_val); //USE pad
	pcie_dbg( "Configure PCIe-PHY ref clk from PAD on board clk_gen\n");
	
	/* 4. 配置EP MODE */
	reg_val = read_apb(PCIE_MODE);
	pcie_dbg( "PCIE_MODE = %lx\n", reg_val);
	reg_val = reg_val | 0x1;
	write_apb(PCIE_MODE, reg_val); //ep x2
	pcie_dbg( "Configure PCIe As EP mode\n");

	reg_val = read_apb(PCIE_LANEX_LINK_NUM);
	pcie_dbg( "PCIE_LANEX_LINK_NUM = %lx\n", reg_val);
	reg_val = reg_val | 0x1100;
	write_apb(PCIE_LANEX_LINK_NUM , reg_val); //lane2和3

	reg_val = read_apb(PCIE_PHY_SRC_SEL);	
	pcie_dbg( "PCIE_PHY_SRC_SEL = %lx\n", reg_val);
	reg_val = reg_val | 0x50;
	write_apb(PCIE_PHY_SRC_SEL , reg_val);

	//reg_val = read_apb(PCIE_LANE_FLIP_EN);
	//reg_val = reg_val | 0xc;
	//write_apb(PCIE_LANE_FLIP_EN , reg_val);
	
	/* 5.配置PCIE_PHY_SRAM_CTRL sram,_bypass = 0 */
	reg_val = read_apb(PCIE_PHY_SRAM_CTRL);
	reg_val = reg_val | 0x0;
	write_apb(PCIE_PHY_SRAM_CTRL, 0);
	pcie_dbg( "Configure PCIe PHY Sram enable\n");
	
	/* 6.配置app_hold_phy */
	reg_val = read_apb(PCIE_PHY_CTRL);
	reg_val = reg_val | 0x6;
	write_apb(PCIE_PHY_CTRL, reg_val);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");

	/* 7.解除PHY复位 */
	reg_val = read_apb(PCIE_LOCAL_RST);
	reg_val = reg_val | 0x4;
	write_apb(PCIE_LOCAL_RST, reg_val); 
	pcie_dbg( "Release PCIe PHY reset\n");

	/* 8.等待SRAM初始化完成 */
	pcie_dbg( "Waiting PCIe PHY SRAM intitial Done.....\n");
	while((read_apb(PCIE_PHY_SRAM_INIT_STATUS)) != 0x3) {
		;
	}

	/* 9. 完成PHY sram done */
	reg_val = read_apb(PCIE_PHY_SRAM_CTRL);
	reg_val = reg_val | 0x3;
	write_apb(PCIE_PHY_SRAM_CTRL, reg_val);
	pcie_dbg( "PCIe PHY SRAM intitial Done Finish\n");

	/* 10.解控制器复位 */
	reg_val = read_apb(PCIE_LOCAL_RST);
	reg_val = reg_val | 0x6;
	write_apb(PCIE_LOCAL_RST, reg_val); //解除phy reset, x2 controller reset
	pcie_dbg( "Release PCIe Controller reset\n");

	/* 11.配置app_hold_phy = 0 */
	reg_val = read_apb(PCIE_PHY_CTRL);
	reg_val = reg_val | 0x0;
	write_apb(PCIE_PHY_CTRL , 0);   
	pcie_dbg( "Configure PCIe-Controller APP-Hold-Phy\n");
#endif

	/* 11.a.link-capility 8G select */
	temp = read_ep_reg(0xa0) & 0xfffffff0;
	temp = temp | 0x3; // 1->gen1, 2->gen2 3 -> gen3
	write_ep_reg(0xa0 , temp);	
	pcie_dbg( "Configure gen a0:%x\n", read_ep_reg(0xa0));
#if 0
	//11.b.link-capility x2, ep x2配0
	temp = read_ep_reg(0x8c0) & 0xffffff80;
	write_ep_reg(0x8c0 , temp);		
#endif
	//NUM_OF_LANES
	temp = read_ep_reg(0x80c) & 0xffffe0ff;
    temp |= 0x2 << 8;
	write_ep_reg(0x80c , temp);	

	//write_ep_reg(0X894, 0X608);
	//write_ep_reg(0X890, 0X3001);
	//write_ep_reg(0X8A8, 0X503FC61);

	//temp = read_ep_reg(0x8e8);
    //temp = (temp & (~0x78780000)) | 0x7878;
	//write_ep_reg(0X8e8, temp);

	/* 12.配置ltssm_en = 1 */
	reg_val = read_apb(PCIE_LTSSM_EN);
	reg_val = reg_val | 0x3;
	write_apb(PCIE_LTSSM_EN , reg_val);   
	pcie_dbg( "Configure PCIe ltssm enable....\n");
	pcie_dbg( "Now, do PCIe link training....\n");
	pcie_dbg(  "I am ep x2!!\n");
	/* 13 wait LTSSM do finish, pcie link up */ 
	pcie_dbg( "Waiting LTSSM in LO STATE....\n");

	/* 14.配置 auto-speed-change = 0 */
	write_ep_reg(0x80c, read_ep_reg(0x80c) | (0x1 << 17));

	ep_pcie_outband_iatu_intitial();
	pcie_dbg( "ep_pcie_outband_iatu_intitial\n"); 

	/* cfg dma_enable */
	temp = read_ep_dma_reg(0x2c); //DMA_READ_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_ep_dma_reg(0x2c, temp);

	temp = read_ep_dma_reg(0xc); //DMA_READ_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_ep_dma_reg(0xc, temp);
	//write_ep_dma_reg(0xac, 0xFFFFFF);

	/* clear write Abort and Done Interrupt mask*/
	write_ep_dma_reg(0x54, 0);
	/* clear read  Abort and Done Interrupt mask*/
	write_ep_dma_reg(0xa8, 0);

#if 0
	for(loop = 0; loop < 10; loop++) {
		while(1) {
			temp = read_apb(0xec); //ep控制器用0xec
			//pcie_dbg( "0xec=0x%x", temp);
			if((temp & 0x3f) == 0x11) {
				break;
			}
		}
	} 
	pcie_dbg( "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

	/* 14.配置 auto-speed-change = 0 */
	//write_ep_reg(0x80C, read_ep_reg(0x80C) | (0x1 << 17));	

	for(loop = 0; loop < 10000; loop++) {
		temp = read_apb(0xd4);
	}
#if 0
	/* wait for PCIe DLL active */
	for (loop = 0; loop < 0x10; loop++) {	
		while (((read_ep_reg(0x80) & 0x20000000) == 0)); //TODO
	}
	pcie_dbg( "PCIe DLL is active\n");
#endif
	temp = read_ep_reg(0x80);
	pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
	pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

	if(pcie_genx == 0x1){
		pcie_dbg( "PCIe Link Speed is GEN1\n");	
	} else if (pcie_genx == 0x2) {
		pcie_dbg( "PCIe Link Speed is GEN2\n");
	} else if (pcie_genx == 0x3) {
		pcie_dbg( "PCIe Link Speed is GEN3\n");	
	}
	else {
		pcie_dbg( "Erro PCIe link speed\n"); 
		while(1) {
			temp = read_ep_reg(0x80);
			pcie_genx = (temp & 0xf0000) >> 16  ;   // bit[19:16];
			if(pcie_genx == 0x3){
				break;
			}
		}
	}
	if(pcie_link_width==0x2){
		pcie_dbg( "PCIe Link Width is x2\n");   
	}
	else {
		pcie_dbg( "Erro PCIe link Width:0x%x\n", pcie_link_width); 
		while(1) {
			temp = read_ep_reg(0x80);
			pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]
			if(pcie_link_width == 0x2){
				break;
			}
		}
	}
	
	ep_pcie_outband_iatu_intitial();
	pcie_dbg( "ep_pcie_outband_iatu_intitial\n"); 

	/* cfg dma_enable */
	temp = read_ep_dma_reg(0x2c); //DMA_READ_ENGINE_EN_OFF bit0->1
	temp = temp & 0xfffffffe;
	temp = temp | 0x1;
	write_ep_dma_reg(0x2c, temp);   

	return;
#endif
}

/* ep测试包括：
   1.读写对端BAR mem空间，检查
   2.操作本端dma与对端传输数据，且数据正确
 */
__attribute__((unused))
static void ep_test_case(void)
{
	u32 temp;
	u32 temp1;
	//int i;

	mdelay(10000);
	
	/* 1.读写对端BAR mem空间，检查 */
	temp = ep_pcie_outband_mem_rd(0x20);
	pcie_dbg( "ep_pcie_outband_mem_rd(0x20): 0x%x\n", temp);
	ep_pcie_outband_mem_wr(0x20, 0x87654321);
	pcie_dbg( "ep_pcie_outband_mem_wr(0x20): 0x%x\n", 0x87654321);
	temp1 = ep_pcie_outband_mem_rd(0x20);
	pcie_dbg( "ep_pcie_outband_mem_rd(0x20): 0x%x\n", temp1);
	if (temp1 != 0x87654321) {
		pcie_dbg( "ep_test_case ep_pcie_fail at step1: mem rw!\n");
	}
	ep_pcie_outband_mem_wr(0x20, temp); //恢复原值
	temp = ep_pcie_outband_mem_rd(0x20);
	pcie_dbg( "ep_pcie_outband_mem_rd(0x20): 0x%x\n", temp);

#if 1
	ep_pcie_outband_mem_wr(0x20, 0x87654321);
	ep_pcie_outband_mem_wr(0x24, 0x12345678);
	ep_pcie_dma_read(0, 0x8, 0x80000020, 0x80000180); //ep->>rc
	ep_pcie_dma_write(0, 0x8, 0x80000180, 0x80000120); //rc-->ep
	temp = ep_pcie_outband_mem_rd(0x120);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x120): 0x%x\n", temp);
	if (temp != 0x87654321) {
		pcie_dbg( "!!!!!ep_test_case ep_pcie_fail at step2.1: dma rw!\n");
	}
	temp = ep_pcie_outband_mem_rd(0x124);
	pcie_dbg( "dm_pcie_outband_mem_rd(0x124): 0x%x\n", temp);
	if (temp != 0x12345678) {
		pcie_dbg( "!!!!!ep_test_case ep_pcie_fail at step2.2: dma rw!\n");
	}
#endif	
#if 1	
	/* 2.操作本端dma与对端传输数据，且数据正确(ep 0x340-0x380<--->rc 0x440-0x480)
	     a.先读ep本端mem
	     b.从rc搬数据到ep mem(dma read)
	     c.读ep本端mem
	     d.写ep本端mem为0,1,2,3,4,5....
	     e.从ep搬数据到rc mem(dma write)
	     f.再从rc搬数据到ep mem(dma read again)
	     g.再读ep本端mem,比较是否为0,1,2,3,4,5....
	     h.写ep本端mem为全0，便于多次测试
	 */
#if 0
	pcie_dbg( "a.before dma read, read_ep_mem(0x340-0x380):\n"); //before dma read, ep mem
	for (i = 0; i < 0x40; i += 4) {
		//temp = read_ep_mem(0x340 + i);
		temp = readl(phys_to_virt(0x80000340 + i));
		pcie_dbg( "0x%x ", temp);
	}
	pcie_dbg( "\n");
#endif	
	ep_pcie_dma_read(0, 0x40, 0x80000440, 0x80000340); //rc->>ep
	pcie_dbg( "b.dma read from rc\n");
#if 0	
	pcie_dbg( "c.after dma read, read_ep_mem(0x340-0x380):\n"); //after dma read, ep mem
	for (i = 0; i < 0x40; i += 4) {
		//temp = read_ep_mem(0x340 + i);
		temp = readl(phys_to_virt(0x80000340 + i));
		pcie_dbg( "0x%x ", temp);
	}
	pcie_dbg( "\n");
	
	pcie_dbg( "d.write ep mem\n");
	for (i = 0; i < 0x40; i += 4) {
		//write_ep_mem(0x340 + i, i);
		writel(i, phys_to_virt(0x80000340 + i));
	}
#endif	
	ep_pcie_dma_write(0, 0x40, 0x80000340, 0x80000440); //ep-->rc
	pcie_dbg( "e.dma write to rc\n");
	
	ep_pcie_dma_read(0, 0x40, 0x80000440, 0x80000340); //rc->>ep
	pcie_dbg( "f.dma read from rc again\n");
#if 0	
	pcie_dbg( "g.after dma read, read_ep_mem(0x340-0x380):\n"); //after dma read, ep mem
	for (i = 0; i < 0x40; i += 4) {
		//temp = read_ep_mem(0x340 + i);
		temp = readl(phys_to_virt(0x80000340 + i));
		pcie_dbg( "0x%x ", temp);
		if (temp != i) {
			pcie_dbg( "ep_test_case fail at step2: dma rw!\n");
			break;
		}
	}
	pcie_dbg( "\n");
	
	pcie_dbg( "h.write ep mem to all 0\n");
	for (i = 0; i < 0x40; i += 4) {
		//write_ep_mem(0x340 + i, 0);
		writel(i, phys_to_virt(0x80000340 + i));
	}
#endif
#endif
	pcie_dbg( "ep_test_case success!\n");

	//write_apb(PCIE_LOCAL_RST, read_apb(PCIE_LOCAL_RST) & (0x0));
	//pcie_dbg( "Reset PCIe PHY and Controller\n");
	
	return;
}

int get_picelink_status(void)
{
	return link_status;
}
EXPORT_SYMBOL(get_picelink_status);

int get_pcie_mode(void)
{
	return hwtest_mode;
}
EXPORT_SYMBOL(get_pcie_mode);

int pcie_rc_dscan(void *kkk)
{
	u32 pcie_link_width;
	u32 pcie_genx, temp;
	u32 count = 0;
	//msleep(60000);
	setup_dm_rc_x4();

	printk(KERN_ERR "** %s **\n", __func__);
	while(!kthread_should_stop())
	{		
		msleep(1000);
		temp = read_apb(0xd4) & 0x3f;//bit[5:0]
		//printk("LTSSM-RC 0xd4:%x status:%d c:%d\n", temp, link_status, count);

		if (temp >= 0x11) {//已链接状态
			count = 1;
			if (link_status)
				continue;  //循环检测
		} else {
			link_status = 0;
			if (count) {   //重新建链
				printk(KERN_INFO "PCIe Link down!!!! LTSSM:%d\n", temp);
				setup_dm_rc_x4();
				count = 0;
			}
			continue;
		}
		printk(KERN_INFO "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

		/* wait for PCIe DLL active */
		for (count = 0; count < 1000; count++){
			read_dm_reg(0x80);
		}
		if ((read_dm_reg(0x80) & 0x20000000) == 0){
			temp = read_dm_reg(0x80);
			temp = temp | 0x20;
			write_dm_reg(0x80, temp);
			continue;
		}
		pcie_dbg( "PCIe DLL is active\n");

		temp = read_dm_reg(0x80);
		pcie_genx = (temp & 0xf0000) >> 16;      // bit[19:16];
		pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]

		if(pcie_genx == 0x1){
			printk(KERN_INFO "PCIe Link Speed is GEN1\n");	
		} else if (pcie_genx == 0x2) {
			printk(KERN_INFO "PCIe Link Speed is GEN2\n");	
		} else if (pcie_genx == 0x3) {
			printk(KERN_INFO "PCIe Link Speed is GEN3\n");	
		} else {
			printk(KERN_INFO "PCIe link speed 0x%x\n", pcie_genx);
			continue;
		}
		if(pcie_link_width==0x2){
			printk(KERN_INFO "PCIe Link Width is x2\n");   
		} else {
			printk(KERN_INFO "PCIe link Width 0x%x\n", pcie_link_width);
			//continue;
		}

		dm_pcie_rc_enumeration(1);
		//rc_test_case();
		link_status = 1;
	}

	return 0;
}

//pcie 0 controller
int dm_ep_detect(void *kkk)
{
	u32 temp, count = 0;
	u32 pcie_genx;
	u32 pcie_link_width;
	u32 linking = 0;
	//msleep(40000);
	setup_dm_ep_x4();

	printk(KERN_ERR "** %s **\n", __func__);
	while(!kthread_should_stop())
	{		
		msleep(1000);
		temp = read_apb(0xd4) & 0x3f;//bit[5:0]
		//printk("LTSSM-EP 0xec=0x%x status:%d\n",temp, link_status);

		if(temp >= 0x11) { //已链接状态
			count = 1;
			if (link_status)
				continue;  //循环检测
		} else {
			link_status = 0;
			if (count) {   //重新建链
				printk(KERN_ERR "PCIe Link down!!!! LTSSM:%d\n", temp);
				setup_dm_ep_x4();
				count = 0;
			}
			continue;
		}
		printk(KERN_ERR "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

		temp = read_dm_reg(0x80);
		pcie_genx = (temp & 0xf0000) >> 16;	// bit[19:16];
		pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]
	
		if(pcie_genx == 0x1){
			pcie_dbg( "PCIe Link Speed is GEN1\n");
		} else if (pcie_genx == 0x2) {
			pcie_dbg( "PCIe Link Speed is GEN2\n");
		} else if (pcie_genx == 0x3) {
			pcie_dbg( "PCIe Link Speed is GEN3\n");
		} else {
			pcie_dbg( "PCIe link speed 0x%x\n", pcie_genx); 
			continue;
		}
		if(pcie_link_width==0x2){
			pcie_dbg( "PCIe Link Width is x2\n");   
		}
		else {
			pcie_dbg( "PCIe link Width:0x%x\n", pcie_link_width); 
			//continue;
		}
	
		link_status = 1;
	}

	return 0;
}

//pcie 1 controller
int pcie_ep_detect(void *kkk)
{
	u32 temp, count = 0;
	u32 pcie_genx;
	u32 pcie_link_width;
	//msleep(40000);
	fad_setup_ep_ep_x2_flip();

	printk(KERN_ERR "** %s **\n", __func__);
	while(!kthread_should_stop())
	{		
		msleep(1000);
		temp = read_apb(0xec) & 0x3f;//bit[5:0]
		//printk(KERN_ERR "LTSSM-EP 0xec=0x%x status:%d\n",temp, link_status);

		if(temp >= 0x11) { //已链接状态
			count = 1;
			if (link_status)
				continue;  //循环检测
		} else {
			link_status = 0;
			if (count) {   //重新建链
				printk(KERN_INFO "PCIe Link down!!!! LTSSM:%d\n", temp);
				fad_setup_ep_ep_x2_flip();
				count = 0;
			}
			continue;
		}
		printk(KERN_INFO "PCIe LTSSM has into LO STATE, PCIe Link up!!!!\n");

		temp = read_ep_reg(0x80);
		pcie_genx = (temp & 0xf0000) >> 16;	// bit[19:16];
		pcie_link_width = (temp & 0x3f00000)>>20;// bit[25:20]
	
		if(pcie_genx == 0x1){
			printk(KERN_INFO "PCIe Link Speed is GEN1\n");
		} else if (pcie_genx == 0x2) {
			printk(KERN_INFO "PCIe Link Speed is GEN2\n");
		} else if (pcie_genx == 0x3) {
			printk(KERN_INFO "PCIe Link Speed is GEN3\n");
		} else {
			printk(KERN_INFO "PCIe link speed 0x%x\n", pcie_genx); 
			continue;
		}
		if(pcie_link_width==0x2){
			printk(KERN_INFO "PCIe Link Width is x2\n");   
		}
		else {
			printk(KERN_INFO "PCIe link Width:0x%x\n", pcie_link_width); 
			//continue;
		}
	
		link_status = 1;
	}

	return 0;
}

void pcie_dscan_exit(void)
{	
	if (ep_detect) {
		kthread_stop(ep_detect);
		ep_detect = NULL;
	}
	if (rc_dscan) {
		kthread_stop(rc_dscan);
		rc_dscan = NULL;
	}
}
EXPORT_SYMBOL(pcie_dscan_exit);

static int fad_dm_rc_x2_sideA(void)
{
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "fad_dm_rc_x2_sideA: step 1\n");
	ioremap_rcx2();
	
	/* 2.配置dm为rc x2，及其它初始化，包括phy */
	pcie_dbg( "fad_dm_rc_x2_sideA: step 2\n");
	//fad_setup_dm_rc_x2();
	
	rc_dscan = kthread_run(pcie_rc_dscan, NULL, "pcie_rc_dscan");
	if (IS_ERR(rc_dscan))
		return PTR_ERR(rc_dscan);

	return 0;
}

static int dmc_rc_x4_setup(void)
{
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "fad_dm_rc_x2_sideA: step 1\n");
	ioremap_rcx4();
	
	rc_dscan = kthread_run(pcie_rc_dscan, NULL, "pcie_rc_dscan");
	if (IS_ERR(rc_dscan))
		return PTR_ERR(rc_dscan);

	return 0;
}

static int fad_ep_x2_sideB(void)
{
	/* 1.ioremap映射ep控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "fad_ep_x2_sideB: step 1\n");
	ioremap_epx2();
	
	/* 2.配置ep为ep x2，及其它初始化，包括phy，注意配置x2 flip */
	pcie_dbg( "fad_ep_x2_sideB: step 2\n");
	//fad_setup_ep_ep_x2_flip();
	
	ep_detect = kthread_run(pcie_ep_detect, NULL, "pcie_ep_detect");
	if (IS_ERR(ep_detect))
		return PTR_ERR(ep_detect);
	
	/* 3.ep测试 */
	//pcie_dbg( "ep_ep_x2_flip_sideA: step 3--ep test\n");
	//mdelay(6000);
	
	//ep_test_case();

	return 0;
}

__attribute__((unused))
static int faw_dm_ep_x2_sideB(void)
{
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "dm_ep_x2_sideB: step 1\n");
	ioremap_epx4();
	
	/* 2.配置dm为ep x4，及其它初始化，包括phy */
	pcie_dbg( "dm_ep_x2_sideB: step 2\n");
	//setup_dm_ep_x2();

	ep_detect = kthread_run(dm_ep_detect, NULL, "dm_ep_detect");
	if (IS_ERR(ep_detect))
		return PTR_ERR(ep_detect);

	//dm_ep_pcie_outband_iatu_intitial();
	//pcie_dbg( "dm_ep_pcie_outband_iatu_intitial\n"); 

	return 0;
}

static int dmc_ep_x4_setup(void)
{
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "dm_ep_x2_sideB: step 1\n");
	ioremap_epx4();

	ep_detect = kthread_run(dm_ep_detect, NULL, "dm_ep_detect");
	if (IS_ERR(ep_detect))
		return PTR_ERR(ep_detect);

	return 0;
}

__attribute__((unused))
static int faw_dm_rc_x2_sideA(void)
{
	/* 1.ioremap映射dm控制器寄存器物理地址为虚拟地址 */
	pcie_dbg( "faw_dm_rc_x2_sideA: step 1\n");
	ioremap_rcx2();
	
	/* 2.配置dm为rc x2，及其它初始化，包括phy */
	pcie_dbg( "faw_dm_rc_x2_sideA: step 2\n");
	//fad_setup_dm_rc_x2();
	
	rc_dscan = kthread_run(pcie_rc_dscan, NULL, "pcie_rc_dscan");
	if (IS_ERR(rc_dscan))
		return PTR_ERR(rc_dscan);

	return 0;
}

__attribute__((unused))
static irqreturn_t pcie_dma_interrupt(int irq, void *dev_id)
{
	read_dma_reg = read_ep_dma_reg(0xa0);
	write_dma_reg = read_ep_dma_reg(0x4c);
	//printk(KERN_ERR "irq = %d 4c=%#x, a0=%#x\n", irq, write_dma_reg, read_dma_reg);

	if (read_dma_reg) {
		write_ep_dma_reg(0xac, read_dma_reg);
		wake_up_interruptible(&read_wq);
	}

	if (write_dma_reg) {
		write_ep_dma_reg(0x58, write_dma_reg);
		wake_up_interruptible(&write_wq);
	}

	return IRQ_HANDLED;
}

static void mask_highspeed_error(void)
{
	void __iomem *sysctrl_va;
	u32 reg_value;

    sysctrl_va = ioremap(0x33000000, 0x200);
	if (sysctrl_va == NULL) {
        printk(KERN_ERR "ioremap fail 0x33000000");
        return;
    }

	//mask high speed error
    reg_value = readl(sysctrl_va + 0x124);
    reg_value &= (~(1 << 4));
    writel(reg_value, sysctrl_va + 0x124);
    reg_value = readl(sysctrl_va + 0x128);
    reg_value &= (~(1 << 4));
    writel(reg_value, sysctrl_va + 0x128);

    iounmap(sysctrl_va);
}

static int __init hwtest_init(void)
{
	int mode = 1; //get_soc_side();
    if (mode < 0) {
		printk(KERN_ERR "board not support: hwtest_init exit\n");
		return 0;
	} else if (mode == 1) {
		hwtest_mode = 1;
		printk(KERN_ERR "hwtest_mode =%d A1000B as EP\n", hwtest_mode);
	} else {
		hwtest_mode = 0;
		printk(KERN_ERR "hwtest_mode =%d A1000A as RC\n", hwtest_mode);
	}
	mask_highspeed_error();
	
	memset(&pci_base, 0, sizeof(struct pci_reg_base));
	res[0] = request_mem_region(PCI_IOMEM0_START, PCI_IOMEM0_SIZE, "mem0");
	if (!res[0]) {
		printk(KERN_ERR "request_mem0_region fail\n");
		return 0;
	}
	res[1] = request_mem_region(PCI_IOMEM1_START, PCI_IOMEM1_SIZE, "mem1");
	if (!res[1]) {
		printk(KERN_ERR "request_mem1_region fail\n");
		return 0;
	}
	res[2] = request_mem_region(PCI_IOMEM2_START, PCI_IOMEM2_SIZE, "mem2");
	if (!res[2]) {
		printk(KERN_ERR "request_mem2_region fail\n");
		return 0;
	}
#if 0
	res[3] = request_mem_region(PCI_TEST_MEM, PCI_TEST_SIZE, "pci_test");
	if (!res[3]) {
		pcie_dbg( "request_pci_test_region fail\n");
		return 0;
	}
#endif
	spin_lock_init(&dma_rlock);
	spin_lock_init(&dma_wlock);

	if (hwtest_mode)
		dmc_ep_x4_setup(); /* 配置DM控制器做EP，x4 */
	else {
		//hwtest_mode = 1;
		dmc_rc_x4_setup(); /* 配置DM控制器做RC，x4 */
	}
	
	picp_cdev_init();

#if PCIE_DMA_USEIRQ
	//int num 226  dts(226-32) -> 23
	ret = request_irq(23, pcie_dma_interrupt, 0, "x2_dma_irq", NULL);
	if (ret < 0) {
		printk(KERN_ERR "request_irq 226 fail %d\n", ret);
	}
	init_waitqueue_head(&read_wq);
	init_waitqueue_head(&write_wq);
#endif

    return 0;
}

static void __exit hwtest_exit(void)
{
    printk(KERN_ERR "hwtest exit, hwtest_mode=%d\n", hwtest_mode);

	pcie_dscan_exit();
	picp_cdev_exit();

	if (pci_base.apb_base) {
		iounmap(pci_base.apb_base);
	}
	if (pci_base.sw_rst) {
		iounmap(pci_base.sw_rst);
	}
	if (pci_base.gic_dist) {
		iounmap(pci_base.gic_dist);
	}
	if (pci_base.dm_base) {
		iounmap(pci_base.dm_base);
	}
	if (pci_base.dm_atu) {
		iounmap(pci_base.dm_atu);
	}
	if (pci_base.dm_dma) {
		iounmap(pci_base.dm_dma);
	}
	if (pci_base.dm_axislv_cfg) {
		iounmap(pci_base.dm_axislv_cfg);
	}
	if (pci_base.dm_axislv_mem) {
		iounmap(pci_base.dm_axislv_mem);
	}
	if (pci_base.dm_axislv_dma) {
		iounmap(pci_base.dm_axislv_dma);
	}
	if (pci_base.dm_mem) {
		iounmap(pci_base.dm_mem);
	}
	
	if (pci_base.ep_base) {
		iounmap(pci_base.ep_base);
	}
	if (pci_base.ep_atu) {
		iounmap(pci_base.ep_atu);
	}
	if (pci_base.ep_dma) {
		iounmap(pci_base.ep_dma);
	}
	if (pci_base.ep_axislv_mem) {
		iounmap(pci_base.ep_axislv_mem);
	}
	if (pci_base.ep_mem) {
		iounmap(pci_base.ep_mem);
	}
#if 0
	if(res[3]) {
		release_mem_region(PCI_TEST_MEM, PCI_TEST_SIZE);
	}
#endif
	if(res[2]) {
		release_mem_region(PCI_IOMEM2_START, PCI_IOMEM2_SIZE);
	}
	if(res[1]) {
		release_mem_region(PCI_IOMEM1_START, PCI_IOMEM1_SIZE);
	}
	if(res[0]) {
		release_mem_region(PCI_IOMEM0_START, PCI_IOMEM0_SIZE);
	}

	return;
}

module_init(hwtest_init);
module_exit(hwtest_exit);

MODULE_AUTHOR("Xing.liao@bst.ai");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("A1000 hwtest pcie rc and ep");
MODULE_ALIAS("hwtest pcie");

