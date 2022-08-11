#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/bst_boardconfig.h>
#include "pcie_save_conf.h"

unsigned char pcie_uboot_args[2][10];
unsigned char pciec_uboot_set;
int pcie0_uboot_type = 0;
int pcie1_uboot_type = 0;
int output_num = 0;

unsigned char *pcie0_addr;
unsigned char *pcie1_addr;
unsigned char *pcie_header;

static struct pcie_type pciecypes[] = {
	{ PCIE_HEAD,	"head no use" },
	{ RC,		    "rc" },
    { EP,		    "ep" },
    { CUSTOM1,		"custom1" },
    { CUSTOM2,		"custom2" },
    { CUSTOM3,		"custom3" },
    { CUSTOM4,		"custom4" },
    { CUSTOM5,		"custom5" },
    { CUSTOM6,		"custom6" },
    { CUSTOM7,		"custom7" },
    { CUSTOM8,		"custom8" },
    { CUSTOM9,		"custom9" },
    { CUSTOM10,		"custom10" },
    { CUSTOM11,		"custom11" },
    { CUSTOM12,		"custom12" },
    { CUSTOM13,		"custom13" },
    { CUSTOM14,		"custom14" },
    { CUSTOM15,		"custom15" },
    { CUSTOM16,		"custom16" },
};

/* qspi Operating collection*/
static char *pciec_qspi_read_head(void)
{
    loff_t addr_conf = PCIECONF_OFFSET;
    return bst_qspi_read_data(BST_QSPI_MTD1_DEV_NO, PCIEC_PER_SIZE, &addr_conf);
}

static int pciec_qspi_write_head(char *buf)
{
    loff_t addr_conf = PCIECONF_OFFSET;
    
    if(buf == NULL) 
        return -1;

    return bst_qspi_write_data(BST_QSPI_MTD1_DEV_NO, buf, &addr_conf, PCIECONF_HEADSIZE);
}
static char *pciec_qspi_read_config(u8 num)   
{
    loff_t addr_conf;
    
    if(num < 0x01 && num > 31)
        return NULL;

    addr_conf = PCIEC_DEFAULT_ADDR + ((num - 1) * PCIEC_PER_SIZE);
    return bst_qspi_read_data(BST_QSPI_MTD1_DEV_NO, PCIEC_PER_SIZE, &addr_conf);
}

static int pciec_qspi_write_config(char *buf,u8 num)   
{
    loff_t addr_conf;

    if(buf == NULL)
        return -1;

    if(num >= 1 && num <= 31)
        addr_conf = PCIEC_DEFAULT_ADDR + ((num - 1) * PCIEC_PER_SIZE);
    else
        return -1;
    
    return bst_qspi_write_data(BST_QSPI_MTD1_DEV_NO, buf, &addr_conf, PCIEC_PER_SIZE);
}

static void pciec_qspi_earse_config(u8 num)
{
    if(num < 0x01 && num > 31)
        return;

    bst_qspi_erase(BST_QSPI_MTD1,PCIEC_DEFAULT_ADDR+((num - 1)* PCIEC_PER_SIZE),PCIECONF_HEADSIZE);
}

static void pciec_qspi_earse_head(void)
{
    bst_qspi_erase(BST_QSPI_MTD1,PCIECONF_OFFSET,PCIECONF_HEADSIZE);
}

static void pciec_qspi_earse_all(void)
{
    bst_qspi_erase(BST_QSPI_MTD1,PCIECONF_OFFSET,PCIECONF_TOTAL_SIZE);
}

static ssize_t sysfs_pcie0conf_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    unsigned int i, cnt=0xff;
    char pcieconf_reg[900] = "";

    for (i = 0; i <= cnt; i++)
    {
        if (! (i & 15))
            sprintf(pcieconf_reg+strlen(pcieconf_reg),"%02x:", i);

        sprintf(pcieconf_reg+strlen(pcieconf_reg)," %02x", pcie0_addr[i]);
        
        if ((i & 15) == 15)
            sprintf(pcieconf_reg+strlen(pcieconf_reg),"\n");
    }
    
    return sprintf(buf, "%s",pcieconf_reg);
}

static ssize_t sysfs_pcie0conf_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int i = 0,ret;
    char *pbuf;
    struct pciec_head *p = (struct pciec_head *)pcie_header;

    pbuf = (char *)vmalloc(strlen(buf)+1);
    if(pbuf == NULL)
        return -ENOMEM;

    memcpy(pbuf, buf, strlen(buf));
    pbuf[strlen(buf)-1] = '\0';

    for(i = 0; i < ARRAY_SIZE(pciecypes); i++)
        {
            if(!strcmp(pbuf,pciecypes[i].name)){
                if(BIT(pciecypes[i].type) & ((p->h_mask << 16) | p->l_mask)){
                    p->userset0 = pciecypes[i].type;
                    pciec_qspi_earse_head();
                    ret = pciec_qspi_write_head(pcie_header);
                    if(ret)
                        pr_err("pciec_qspi_write_head error \r\n");
                    return n; 
                }else{
                    pr_err("input do not set value \r\n");
                }       
            }
        }

    pr_err("input do not match pcie support types\r\n");
    return n;
}

static ssize_t sysfs_pcie1conf_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    unsigned int i, cnt=0xff;
    char pcieconf_reg[900] = "";

    for (i = 0; i <= cnt; i++)
    {
        if (! (i & 15))
            sprintf(pcieconf_reg+strlen(pcieconf_reg),"%02x:", i);

        sprintf(pcieconf_reg+strlen(pcieconf_reg)," %02x", pcie1_addr[i]);
        
        if ((i & 15) == 15)
            sprintf(pcieconf_reg+strlen(pcieconf_reg),"\n");
    }
    
    return sprintf(buf, "%s",pcieconf_reg);
}

static ssize_t sysfs_pcie1conf_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int i = 0,ret;
    char *pbuf;
    struct pciec_head *p = (struct pciec_head *)pcie_header;

    pbuf = (char *)vmalloc(strlen(buf)+1);
    if(pbuf == NULL)
        return -ENOMEM;

    memcpy(pbuf, buf, strlen(buf));
    pbuf[strlen(buf)-1] = '\0';

    for(i = 0; i < ARRAY_SIZE(pciecypes); i++)
        {
            if(!strcmp(pbuf,pciecypes[i].name)){
                if(BIT(pciecypes[i].type) & ((p->h_mask << 16) | p->l_mask)){
                    p->userset1 = pciecypes[i].type;
                    pciec_qspi_earse_head();
                    ret = pciec_qspi_write_head(pcie_header);
                    if(ret)
                        pr_err("pciec_qspi_write_head error \r\n");
                    return n; 
                }else{
                    pr_err("input do not set value \r\n");
                }       
            }
        }

    pr_err("input do not match pcie support types\r\n");
    return n;
}

static ssize_t sysfs_pciesupmode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return 0;
}

static struct kobj_attribute pcie0conf_print_attribute =
    __ATTR(pcie0, S_IWUSR|S_IRUGO, sysfs_pcie0conf_show, sysfs_pcie0conf_store);

static struct kobj_attribute pcie1conf_print_attribute =
    __ATTR(pcie1, S_IWUSR|S_IRUGO, sysfs_pcie1conf_show, sysfs_pcie1conf_store);

static struct kobj_attribute pciesupmode_print_attribute =
    __ATTR(pcie_mode, S_IWUSR|S_IRUGO, sysfs_pciesupmode_show, NULL);
 
static struct attribute *sysfs_pcie_attributes[] = {
    &pcie0conf_print_attribute.attr,
    &pcie1conf_print_attribute.attr,
    &pciesupmode_print_attribute.attr,
    NULL
};

static const struct attribute_group sysfs_pcie_attr_group = {
    .attrs = sysfs_pcie_attributes,
};


static void __init pciec_header_init(void)
{
    struct pciec_head p = {
        .magiccode = 0x0a0b0c0d,
        .version_num = VERSION_NUM,
        .data_type = 0x02,
        .offset = 0x1000,
        .lengths = 0x20000,
        .h_mask = 0x00,
        .l_mask = 0x02, 
        .userset0 = 0x01,
        .userset1 = 0x01,
    };

    memcpy(pcie_header,(char *)&p, sizeof(struct pciec_head));
}

static int __init pcie_first_config(void)
{
    int ret;

    pciec_qspi_earse_all();
    pciec_header_init();

    ret = pciec_qspi_write_head(pcie_header);
    if(ret)
        return -1;
  
    ret = pciec_qspi_write_config(pcie_default_conf[0],1);
    if(ret)
        return -1;

    memcpy(pcie0_addr,(const char *)pcie_default_conf[0],PCIEC_PER_SIZE);
    memcpy(pcie1_addr,(const char *)pcie_default_conf[0],PCIEC_PER_SIZE);

    return 0;
}

static int pciec_parse_bootargs(unsigned char pciec_num)
{
    int i = 0;

    for(i = 0; i < ARRAY_SIZE(pciecypes); i++)
        {
            if(!strcmp(pcie_uboot_args[pciec_num],pciecypes[i].name)){
                return pciecypes[i].type;
            }
        }
    
    return -1;
}

static int pciec_uboot_config(unsigned char pciec_type)
{
    if(pciec_uboot_set & pciec_type)
        return  pciec_parse_bootargs(pciec_type-1);

    return -1;
}

static void pcie_set_conf_f(unsigned char *p,u16 h_mask,u16 l_mask,u8 num)
{
    char *buf;
    u32 mask = (h_mask << 16) | l_mask;

    if(mask & BIT(num)){
        buf = pciec_qspi_read_config(num);
        memcpy(p,buf,PCIEC_PER_SIZE);
    }else{
        buf = pciec_qspi_read_config(0x01);
        pr_err("set %s error,Now go to defalut set\n",(p == pcie0_addr)? "pcie0" : "pcie1");
        memcpy(p,buf,PCIEC_PER_SIZE);
    }

    kfree(buf);
}

static void pcie_get_config(char *head)
{
    struct pciec_head *p = (struct pciec_head *)head;
    
    pcie0_uboot_type = pciec_uboot_config(PCIE0_CONTRO);
    if(pcie0_uboot_type == -1)/* bootargs nomatch */
        pcie_set_conf_f(pcie0_addr,p->h_mask,p->l_mask,p->userset0);
    else
        pcie_set_conf_f(pcie0_addr,p->h_mask,p->l_mask,pcie0_uboot_type);

    pcie1_uboot_type = pciec_uboot_config(PCIE1_CONTRO);
    if(pcie1_uboot_type == -1)/* bootargs nomatch */
        pcie_set_conf_f(pcie1_addr,p->h_mask,p->l_mask,p->userset1);
    else
        pcie_set_conf_f(pcie1_addr,p->h_mask,p->l_mask,pcie1_uboot_type);
}

static int pciec_init(void)
{
    char *buf;
    int ret = 0;

    buf = pciec_qspi_read_head();
    if(!buf)
        return -1;

    memcpy(pcie_header,buf,PCIEC_PER_SIZE);
    kfree(buf);

    if(pcie_header[4] == (VERSION_NUM & 0xff) && pcie_header[5] == ((VERSION_NUM & 0xff00)) >> 8)
        pcie_get_config(pcie_header);
    else
        ret = pcie_first_config();

    return ret;
}

static int pciec_alloc(void)
{
   if(!(pcie0_addr = (unsigned char *)kzalloc(PCIEC_PER_SIZE,GFP_KERNEL))){
        return -ENOMEM;
   }

   if(!(pcie1_addr = (unsigned char *)kzalloc(PCIEC_PER_SIZE,GFP_KERNEL))){
        kfree(pcie0_addr);
        return -ENOMEM;
   }

    if(!(pcie_header = (unsigned char *)kzalloc(PCIEC_PER_SIZE,GFP_KERNEL))){
        kfree(pcie0_addr);
        kfree(pcie1_addr);
        return -ENOMEM;
    }

    return 0;
}

static struct kobject *pciec_kobj = NULL;

static int pciec_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t pciec_read(struct file *filp, 
                    char __user *buf,size_t cnt, loff_t *offt)
{
    int retvalue = 0;
    char *databuf;

    databuf = pciec_qspi_read_config(output_num);
    retvalue = copy_to_user(buf, databuf, cnt);
    if(retvalue != 0)
        pr_err("send data error\r\n");

    kfree(databuf);
    return 0;
}

static ssize_t pciec_write(struct file *file, const char __user *buf,
          size_t size, loff_t *ppos)
{
    int retvalue;
    char *databuf;
    struct pciec_head *p = (struct pciec_head *)pcie_header;

    if(size == 0 || size > 4097)
        return -EFAULT;

    if((databuf = (unsigned char *)kzalloc(size,GFP_KERNEL)) == NULL)
        return -ENOMEM;

    retvalue = copy_from_user(databuf, buf, size);
    if(retvalue < 0) {
        pr_err("kernel write failed!\r\n");
        goto wr_fail;
    }

    if(databuf[4096] < 0x01 && databuf[4096] > 31){
        pr_err("wrong destination to write\r\n");
        goto wr_fail;
    }

    pciec_qspi_earse_config(databuf[4096]);
    retvalue = pciec_qspi_write_config(databuf,databuf[4096]);
    if(retvalue)
        goto wr_fail;

    if(databuf[4096] >= 16 && databuf[4096] <= 31)
        p->h_mask |= BIT(databuf[4096] - 16);
    
    pciec_qspi_earse_head();
    retvalue = pciec_qspi_write_head(pcie_header);
    if(retvalue)
        goto wr_fail;
    
    kfree(databuf);
    return 0;

wr_fail:
    kfree(databuf);
    return -EFAULT;
}

static long pciec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret;

    if (_IOC_TYPE(cmd) != PICEC_IOC_MAGIC) {
        pr_err("[%s] command type [%c] error!\n", 
            __func__, _IOC_TYPE(cmd));
        return -ENOTTY; 
    }

    if (_IOC_NR(cmd) > PICEC_IOC_MAXNR) { 
        pr_err("[%s] command numer [%d] exceeded!\n", 
            __func__, _IOC_NR(cmd));
        return -ENOTTY;
    }    

    switch (cmd) {
	case PICEC_IOC_SNIFFER:
		ret = copy_from_user(&output_num, (const void *)arg, _IOC_SIZE(cmd));
		if (ret)
			return -EFAULT;
		break;

    default:
		return -EEXIST;
    }

    return ret;
}

/* misc dev to import and export configuration*/
static struct file_operations pcieconf_fops = { 
    .owner = THIS_MODULE,
    .open = pciec_open,
    .read = pciec_read,
    .write = pciec_write,
    .unlocked_ioctl = pciec_ioctl,
};

static struct miscdevice pciec_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "pcieconf",
    .fops = &pcieconf_fops,
};

int sysfs_pciec_init(void)
{
    int ret;

    pciec_kobj = kobject_create_and_add("pcieconf", NULL);
    if(!pciec_kobj){
        pr_err("pcieconf sys node create error \n");
        goto kobj_fail;
    }

    ret = sysfs_create_group(pciec_kobj, &sysfs_pcie_attr_group);
    if(ret){
        pr_err("sysfs_create_group failed\n");
        goto group_fail;
    }

    ret = misc_register(&pciec_miscdev);
    if(ret < 0){
        pr_err("misc device register failed!\r\n");
        goto group_fail;
    }

    ret = pciec_alloc();
    if(ret){
        pr_err("pciec_alloc failed\n");
        goto group_fail;
    }

    ret = pciec_init();
    if(ret){
        pr_err("pciec_init failed\n");
        goto init_fail;
    }

    return 0;

init_fail:
    kfree(pcie0_addr);
    kfree(pcie1_addr);
    kfree(pcie_header);

group_fail:
    kobject_put(pciec_kobj);

kobj_fail:
    return -1;
}
late_initcall(sysfs_pciec_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("chengyu.yang@bst.ai");

static int __init pciec_setup(char *str)
{
	if (!strncmp(str, "0=", 2)){
        pciec_uboot_set |= 0x01;
        strlcpy(pcie_uboot_args[0],str+2,10);
    }else if (!strncmp(str, "1=", 2)){
        pciec_uboot_set |= 0x02;
        strlcpy(pcie_uboot_args[1],str+2,10);
    }

    return 1;
}
__setup("pcie", pciec_setup);