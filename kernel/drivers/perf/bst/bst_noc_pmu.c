#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/cma.h>

#include "bst_pmu.h"

static unsigned int major_num = 241;
module_param(major_num, uint, S_IRUGO);

static struct bst_noc_cdev *pbst_noc_cdev = NULL;
static NOC_SYS noc_sys;
static unsigned int node_cnt = 0;
NOCBASE_USER nocbase;
NOCCONF_USER nocconf;
struct timer_list noc_timer;

unsigned long long vir2phy_off[14];
static unsigned s_g_open_flag = 0;

static void noc_atb_init(void)
{
	unsigned long buf_phy = 0;
	/* CPUNOC */
	/* etf setting */
	void __iomem *coresight_base_vir = noc_sys.noc_base.coresight_base.vir_addr;
    
	/* set_reg_val : type = 11 */
	writel(0xC5ACCE55, coresight_base_vir + 0xFB0);
	writel(0x2, coresight_base_vir + 0x28);
	writel(0x1, coresight_base_vir + 0x20);
	writel(0x7, coresight_base_vir + 0x308);

	/* CORENOC */
	/* etf setting */
	writel(0xC5ACCE55, coresight_base_vir + 0x6FB0);
	writel(0x2, coresight_base_vir + 0x6028);
	writel(0x1, coresight_base_vir + 0x6020);
	writel(0x7, coresight_base_vir + 0x6308);
	
	/* if you want change ddr range,you must clean two register below */
	writel(0x0, coresight_base_vir + 0x1020);
	writel(0x0, coresight_base_vir + 0x1308);
	/* etr setting */
	writel(0xC5ACCE55, coresight_base_vir + 0x1FB0);
	writel(0x2, coresight_base_vir + 0x1028);
	/* set ddr buffer size */
	writel(noc_sys.noc_base.buf_len / 4, coresight_base_vir + 0x1004);
	buf_phy = (unsigned long)((unsigned long*)noc_sys.noc_base.buf_base.phy_addr);
	/* set ddr range for data saving */
	writel((unsigned int)buf_phy, coresight_base_vir + 0x1118);
	writel((unsigned int)(buf_phy >> 32), coresight_base_vir + 0x111C);
	writel(0x1, coresight_base_vir + 0x1020);
	writel(0x7, coresight_base_vir + 0x1308);
	/* funnel setting */
	writel(0xC5ACCE55, coresight_base_vir + 0x2FB0);
	writel(0xff, coresight_base_vir + 0x2000);
	writel(0xC5ACCE55, coresight_base_vir + 0x3FB0);
	writel(0x7, coresight_base_vir + 0x3000);
}

void noc_buf_clean(void)
{
	volatile unsigned long long *p_buf;

	pr_debug("Start ddr clean...\n");
	if (!noc_sys.noc_base.buf_base.vir_addr) {
		printk(KERN_ERR "ERROR: %s NULL pointer vir_addr", __func__);
		return;
	}
	p_buf = (unsigned long long *)noc_sys.noc_base.buf_base.vir_addr;
	while (p_buf < (unsigned long long *)(noc_sys.noc_base.buf_base.vir_addr + noc_sys.noc_base.buf_len)) {
		*p_buf = 0;
		p_buf++;
	}

	pr_debug("Complete\n");
}
 
static void bst_noc_start(int idmask)
{
	int i;
	PNOC_NODE pnoc_node;
	void __iomem *corenoc_atb_base = noc_sys.noc_base.corenoc_atb_base.vir_addr;
	void __iomem *cpunoc_atb_base = noc_sys.noc_base.cpunoc_atb_base.vir_addr;

	noc_buf_clean();
	
	if(noc_sys.mode != 0){
		/* capture mode */
		for(i = 0;i < NOC_COUNT;i++){
			if(idmask & (1u << i)){
				pnoc_node = &noc_sys.noc_node[i];
				/* set_reg_val : i */
				writel(0x1u, pnoc_node->confbase + NOC_CFG_CTL);
				writel(0xfu, pnoc_node->confbase + NOC_FILTERLUT);
			}
		}
	}
	/* set_reg_val : 13 */
	if(idmask & 0x780){
		/* CORENOC */
		writel(0x1, corenoc_atb_base + 0x8);
		writel(0x1, corenoc_atb_base + 0xc);
	}
	/* set_reg_val : 12 */
	/* CPUNOC */
	writel(0x2, cpunoc_atb_base + 0x8);
	writel(0x1, cpunoc_atb_base + 0xc);

	noc_sys.status = NOC_STATUS_RUNNING;

	pr_debug("Start NOC monitor\n");
}

static void bst_noc_stop(struct timer_list *t)
{
	int i;
	PNOC_NODE pnoc_node;
	void __iomem *corenoc_atb_base = noc_sys.noc_base.corenoc_atb_base.vir_addr;
	void __iomem *cpunoc_atb_base = noc_sys.noc_base.cpunoc_atb_base.vir_addr;

	if(t == NULL){
		/* it's not in timer interrupt so we should remove timer */
		del_timer(&noc_timer);		
	}
	
	if(noc_sys.mode != 0){
		/* capture mode */
		for(i = 0;i < NOC_COUNT;i++){
			if(nocbase.mask & (1u << i)){
				pnoc_node = &noc_sys.noc_node[i];
				/* set_reg_val : i */
				writel(0x0, pnoc_node->confbase + NOC_MAIN_CTRL);
				writel(0x0, pnoc_node->confbase + NOC_CFG_CTL);
			}
		}
	}
	/* set_reg_val : 13 */
	if(nocbase.mask & 0x780){
		/* CORENOC */
		writel(0x1, corenoc_atb_base + 0x8);
		writel(0x0, corenoc_atb_base + 0xc);
	}
	/* set_reg_val : 12 */
	/* CPUNOC */
	writel(0x2, cpunoc_atb_base + 0x8);
	writel(0x0, cpunoc_atb_base + 0xc);

	noc_sys.status = NOC_STATUS_COMPLETION;

	pr_debug("Stop NOC monitor\n");
}

static void set2hw_nocnode(PNOC_NODE pnode , int *mode, unsigned char type)
{
	PNOCNODE_CONF pnocconf = &pnode->nocnode_conf;
	PNOC_SYS pnoc_sys = container_of(mode, NOC_SYS, mode);

	if(*mode == 0){
		/* set_reg_val : type */
		writel(0x8u, pnode->confbase + NOC_MAIN_CTRL);
		writel(0u, pnode->confbase + NOC_PORT_SEL);
		writel(0x8, pnode->confbase + NOC_CNT0_SRC);
		writel(0x10, pnode->confbase + NOC_CNT1_SRC);
		writel(pnoc_sys->window_time, (pnode->confbase + NOC_STAT_PERIOD));
		writel(0x01u, pnode->confbase + NOC_CFG_CTL);
	}else{
		/* set_reg_val : type */
		writel(pnocconf->rid_base, (pnode->confbase + NOC_ROUTEID_BASE));
		writel(pnocconf->rid_mask, (pnode->confbase + NOC_ROUTEID_MASK));
		writel(pnocconf->addr_low, (pnode->confbase + NOC_ADDRBASE_LOW));
		writel(pnocconf->windsize, (pnode->confbase + NOC_WINDOW_SIZE));
		writel(pnocconf->opcpde, (pnode->confbase + NOC_OPCODE));
		writel(0x2u, (pnode->confbase + NOC_MAIN_CTRL));
	}
}

static int init_nocsys(PNOC_SYS ptrsys)
{
	int i;
	PNOC_NODE pnode;

	ptrsys->cpunoc_basephy = (void *)0x33402000;
	ptrsys->corenoc_basephy = (void *)0x33422000;
	ptrsys->mode = 0u;
	ptrsys->window_time = 20u;

	/* cpu d0p0 d0p1 d1p0 d1p1 gdma s2c cv isp net vsp */
	for(i = 0;i < NOC_COUNT;i++){
		pnode = &(ptrsys->noc_node[i]);
		
		if(i < CPUNOC_COUNT){
			pnode->confbase =  ioremap((phys_addr_t)(node_off_tbl[i] + ptrsys->cpunoc_basephy), 0x200);
			if (!pnode->confbase) {
				printk(KERN_ERR "%s: %d line failed to ioremap\n", __func__, __LINE__);
				while(--i)
					iounmap(ptrsys->noc_node[i].confbase);

				return -ENOMEM;
			}
			vir2phy_off[i] = pnode->confbase - (node_off_tbl[i] + ptrsys->cpunoc_basephy);
		}else{
			pnode->confbase = ioremap((phys_addr_t)(node_off_tbl[i] + ptrsys->corenoc_basephy), 0x200);
			if (!pnode->confbase) {
				printk(KERN_ERR "%s: %d line failed to ioremap\n", __func__, __LINE__);
				while(--i)
					iounmap(ptrsys->noc_node[i].confbase);

				return -ENOMEM;
			}
			vir2phy_off[i] = pnode->confbase - (node_off_tbl[i] + ptrsys->corenoc_basephy);
		}
		ptrsys->noc_node[i].nocnode_conf.rid_mask = 0xfe00;
		ptrsys->noc_node[i].nocnode_conf.addr_low = 0x80000000;
		ptrsys->noc_node[i].nocnode_conf.windsize = 0x1d;
		ptrsys->noc_node[i].nocnode_conf.opcpde = 0x3;
	}
	ptrsys->noc_node[0].nocnode_conf.rid_base = 0x2200;
	ptrsys->noc_node[1].nocnode_conf.rid_base = 0x2200;
	ptrsys->noc_node[2].nocnode_conf.rid_base = 0x2400;
	ptrsys->noc_node[3].nocnode_conf.rid_base = 0x8600;
	ptrsys->noc_node[4].nocnode_conf.rid_base = 0x8800;
	ptrsys->noc_node[5].nocnode_conf.rid_base = 0x5200;
	ptrsys->noc_node[6].nocnode_conf.rid_base = 0x8600;

	ptrsys->noc_base.coresight_base.phy_addr = (void *)0x32702000;
	ptrsys->noc_base.coresight_base.vir_addr = ioremap((phys_addr_t)0x32702000, 0x7000);
	if (!ptrsys->noc_base.coresight_base.vir_addr) {
		printk(KERN_ERR "%s: %d line failed to ioremap\n", __func__, __LINE__);
		goto coresight_ioremap_err;
	}
	vir2phy_off[11] = ptrsys->noc_base.coresight_base.vir_addr - ptrsys->noc_base.coresight_base.phy_addr;
	
	ptrsys->noc_base.cpunoc_atb_base.phy_addr = (void *)0x33401100;
	ptrsys->noc_base.cpunoc_atb_base.vir_addr = ioremap((phys_addr_t)0x33401100, 0x10);
	if (!ptrsys->noc_base.cpunoc_atb_base.vir_addr) {
		printk(KERN_ERR "%s: %d line failed to ioremap\n", __func__, __LINE__);
		goto cpunoc_atb_ioremap_err;
	}
	vir2phy_off[12] = ptrsys->noc_base.cpunoc_atb_base.vir_addr - ptrsys->noc_base.cpunoc_atb_base.phy_addr;

	ptrsys->noc_base.corenoc_atb_base.phy_addr = (void *)0x33421480;
	ptrsys->noc_base.corenoc_atb_base.vir_addr = ioremap((phys_addr_t)0x33421480, 0x10);
	if (!ptrsys->noc_base.corenoc_atb_base.vir_addr) {
		printk(KERN_ERR "%s: %d line failed to ioremap\n", __func__, __LINE__);
		goto corenoc_atb_ioremap_err;
	}

	vir2phy_off[13] = ptrsys->noc_base.corenoc_atb_base.vir_addr - ptrsys->noc_base.corenoc_atb_base.phy_addr;

	ptrsys->status = NOC_STATUS_IDLE;

	return 0;

corenoc_atb_ioremap_err:
	iounmap(ptrsys->noc_base.cpunoc_atb_base.vir_addr);
cpunoc_atb_ioremap_err:
	iounmap(ptrsys->noc_base.coresight_base.vir_addr);
coresight_ioremap_err:
	for (i=0; i<NOC_COUNT; i++)
		iounmap(ptrsys->noc_node[i].confbase);

	return -ENOMEM;
}

void set_nocsys(PNOC_SYS ptrsys, PNOCBASE_USER nocbase)
{
	ptrsys->noc_base.buf_base.phy_addr = nocbase->nocbuf;
	/* if we changed nocbuf addr, we need to iounremap old nocbuf vir_addr */
	if(ptrsys->noc_base.buf_base.vir_addr){
		iounmap(ptrsys->noc_base.buf_base.vir_addr);
	}
	ptrsys->noc_base.buf_base.vir_addr = ioremap((phys_addr_t)nocbase->nocbuf, nocbase->buflen);
	if (!ptrsys->noc_base.buf_base.vir_addr) {
		printk(KERN_ERR "%s: %d line failed to ioremap\n", __func__, __LINE__);
		return;
	}
	ptrsys->noc_base.buf_len = nocbase->buflen;
	ptrsys->status = nocbase->status;
}

void get_nocsys(PNOC_SYS ptrsys, PNOCBASE_USER nocbase)
{
	nocbase->buflen = ptrsys->noc_base.buf_len;
	nocbase->mode = ptrsys->mode;
	nocbase->nocbuf = ptrsys->noc_base.buf_base.phy_addr;
	nocbase->window_time = ptrsys->window_time;
	nocbase->status = ptrsys->status;
}

static int bst_noc_open(struct inode *pinode, struct file *pfile)
{
	/* The device can only be used by one application at a time */
	if (s_g_open_flag)
		return -EBUSY;

	s_g_open_flag = 1;
	return 0;
}

static int bst_noc_release(struct inode *inode, struct file *file)
{
	s_g_open_flag = 0;
	return 0;
}

static ssize_t bst_noc_read(struct file *file, char __user *buf, size_t cnt, loff_t *ppos)
{
	ssize_t read_bytes = 0;

	read_bytes = simple_read_from_buffer(buf, cnt, ppos, noc_sys.noc_base.buf_base.vir_addr, noc_sys.noc_base.buf_len);

	return read_bytes;
}

static loff_t bst_noc_llseek(struct file *file, loff_t offset, int whence)
{
    loff_t new_pos = 0;

    switch(whence){
        case SEEK_SET:    
            new_pos = offset;
        	break;
        case SEEK_CUR:    
            new_pos = file->f_pos + offset;
        	break;
        case SEEK_END:    
            new_pos = noc_sys.noc_base.buf_len + offset;
        	break;
        default:
            return -1;
    }
    if(new_pos < 0){
        return -1;
	}
    file->f_pos = new_pos;
    return new_pos;
}

static long bst_noc_iotcl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int node_id = 0;
	int error = 0;
	int cnt = 0;
	
	switch (cmd) {
	case NOC_IOCTL_SET_BASE:
		if(arg != 0){
			if (copy_from_user((void *)&nocbase, (void __user *)arg, sizeof(NOCBASE_USER)))
                                return -EFAULT;
		}else{
			error = -EINVAL;
			break;
		}
		nocbase.status = NOC_STATUS_READY;
		set_nocsys(&noc_sys, &nocbase);
		noc_atb_init();

		break;
	case NOC_IOCTL_GET_BASE:
		if(arg != 0){
			get_nocsys(&noc_sys, &nocbase);
			if (copy_to_user((void __user *)arg, (void *)&nocbase, sizeof(NOCBASE_USER)))
				return -EFAULT;
		}else{
			error = -EINVAL;
		}
		break;
	case NOC_IOCTL_SET_CONF:
		if(arg != 0){
			if (copy_from_user(&nocconf, (void __user *)arg, sizeof(NOCCONF_USER)))
				return -EFAULT;
		}else{
			return -EINVAL;
		}

		/* capture mode */
		node_id = nocconf.id;
		if( node_id >= NOC_COUNT){
			pr_debug("valid id is 0 to 10 !\n");
		}else{
			noc_sys.noc_node[node_id].id = node_id;
			memcpy(&noc_sys.noc_node[node_id].nocnode_conf, &nocconf.noc_node_conf, sizeof(NOCNODE_CONF));
		}

		break;
	case NOC_IOCTL_GET_CONF:
		if(arg != 0){
			if (copy_to_user((void __user *)arg, (void *)&noc_sys.noc_node[node_cnt].nocnode_conf, sizeof(NOCNODE_CONF)))
				return -EFAULT;
			node_cnt++;
			if(node_cnt >= NOC_COUNT){
				node_cnt = 0;
			}
		}else{
			error = -EINVAL;
		}

		break;
	case NOC_IOCTL_SET_START:
			nocbase.mask = arg & 0x7ffu;
			for(cnt = 0; cnt < NOC_COUNT; cnt++){
				if(nocbase.mask & (1u << cnt)){
					set2hw_nocnode(&noc_sys.noc_node[cnt], &noc_sys.mode, cnt);
				}
			}
			bst_noc_start(nocbase.mask);
			/* setup noc_timer to stop automaticly by timer interrupt*/
			timer_setup(&noc_timer, bst_noc_stop, 0);
			noc_timer.expires = jiffies + NOC_MONITOR_TIME * HZ;
			add_timer(&noc_timer);
		break;
	case NOC_IOCTL_SET_STOP:
			/* manually stop */
			bst_noc_stop(NULL);
		break;
	case NOC_IOCTL_SET_WINDOWTIME:
		noc_sys.window_time = arg;
		break;
	case NOC_IOCTL_SET_MODE:
		noc_sys.mode = arg;
		noc_sys.status = NOC_STATUS_IDLE;
		break;
	default:
		pr_debug("ioctl cmd %d error\n",cmd);
		error = -EINVAL;
		break;
	}
	return error;
}

static const struct file_operations bst_noc_fops = {
    .owner = THIS_MODULE,
    .open = bst_noc_open,
    .release = bst_noc_release,
    .read = bst_noc_read,
    .unlocked_ioctl = bst_noc_iotcl,
    .compat_ioctl = bst_noc_iotcl,
	.llseek = bst_noc_llseek,
};

static int __init bst_noc_init(void)
{
	int ret;
	dev_t devno = MKDEV(major_num, 0);
	
	ret = init_nocsys(&noc_sys);
	if (ret<0) {
		printk(KERN_ERR "%s : %d bst noc init_nocsys failed\n", __func__, __LINE__);
		return ret;
	}

	if ( 0 == major_num )
	{
		ret = register_chrdev_region(devno, 1, "nocpmu");
	}
	else
	{
		ret = alloc_chrdev_region(&devno, 0, 1, "nocpmu");
		major_num = devno;
	}
	
	if ( ret < 0 )
	{
		printk(KERN_ERR "bst noc cdev get device num error\n");
		return ret;
	}
	
	pbst_noc_cdev = kzalloc(sizeof(struct bst_noc_cdev), GFP_KERNEL);
	if ( !pbst_noc_cdev )
	{
		printk(KERN_ERR "bst noc cdev get memory failed\n");
		unregister_chrdev_region(devno, 1);
		return -ENOMEM;
	}
	
	cdev_init(&pbst_noc_cdev->cdev, &bst_noc_fops);
	pbst_noc_cdev->cdev.owner = THIS_MODULE;
	
	ret = cdev_add(&pbst_noc_cdev->cdev, devno,  1);
	if ( ret < 0 )
	{
		printk(KERN_ERR "add bst noc cdev failed\n");
		unregister_chrdev_region(devno, 1);
		return ret;
	}
	return 0;
}
late_initcall(bst_noc_init);

static void __exit bst_noc_exit(void)
{
	int i = 0;

    pr_debug( "bst noc cdev exit\n ");
	nocbase.status = NOC_STATUS_FREE;
    if ( NULL != pbst_noc_cdev )
    {
        cdev_del(&pbst_noc_cdev->cdev);
        kfree(pbst_noc_cdev);
    }
    
	iounmap(noc_sys.noc_base.coresight_base.vir_addr);
	iounmap(noc_sys.noc_base.cpunoc_atb_base.vir_addr);
	iounmap(noc_sys.noc_base.corenoc_atb_base.vir_addr);
	if (noc_sys.noc_base.buf_base.vir_addr)
		iounmap(noc_sys.noc_base.buf_base.vir_addr);

	for (i=0; i<NOC_COUNT; i++)
		iounmap(noc_sys.noc_node[i].confbase);

    unregister_chrdev_region(major_num, 1);
}
module_exit(bst_noc_exit);

MODULE_AUTHOR("Yord");
MODULE_LICENSE("GPL v2");