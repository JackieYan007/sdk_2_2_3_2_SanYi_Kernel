#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/printk.h>
#include <linux/mtd/mtd.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/moduleparam.h> 
#include <linux/miscdevice.h>
#include <linux/tee_interface.h>
#include <linux/bst_boardconfig.h>

#define TEE_SHOW_CNT			1		  	  /* device num*/
#define TEE_SHOW_NAME		"tee_show"	      /*device name*/ 

/* struct tee_show */
struct tee_show_dev{
	dev_t devid;		
	struct cdev cdev;		
	struct class *class;	
	struct device *device;	
	int major;			
	int minor;				
};

struct tee_read_data {
	char	chipid[100];
	int		board_type;
};

struct tee_show_dev tee_show;


int get_board_type(void)
{
	struct device_node *node;
	const char *str;
	int size;
	
    node = of_find_node_by_path("/");
	str = of_get_property(node, "model", &size);
	//printk(KERN_ERR "of_get_property model:%s\r\n", str);
	
	if (strncmp(str, "BST A1000 Edage Computer", size) == 0) {  //A1000  MEC
		return 0;
	} else if (strncmp(str, "BST A1000 ECU", size) == 0) {
		return 1;
	} else if (strncmp(str, "BST A1000 Edage V3 Computer", size) == 0) {  //A1000  MEC_V3
		return 2;
	} else if (strncmp(str, "BST A1000 EVB", size) == 0) {
		return 3;
	} else if (strncmp(str, "BST A1000 evbl", size) == 0) {
		return 4;
	} else if (strncmp(str, "BST A1000 FADV3-A", size) == 0) {
		return 5;
	} else if (strncmp(str, "BST A1000 FADV3-B", size) == 0) {
		return 6;
	} else if (strncmp(str, "BST A1000 FAD-A", size) == 0) {
		return 7;
	} else if (strncmp(str, "BST A1000 FAD-B", size) == 0) {
		return 8;
	} else if (strncmp(str, "BST A1000 FAD-M", size) == 0) {
		return 9;
	} else if (strncmp(str, "BST A1000 FAD-S", size) == 0) {
		return 10;
	} else if (strncmp(str, "BST A1000 FAW-A", size) == 0) {
		return 11;
	} else if (strncmp(str, "BST A1000 FAW-B", size) == 0) {
		return 12;
	} else if (strncmp(str, "BST A1000 IDDC21", size) == 0) {
		return 13;
	} else if (strncmp(str, "BST A1000 PAT", size) == 0) {
		return 14;
	} else if (strncmp(str, "BST A1000B APA", size) == 0) {
		return 15;
	} else if (strncmp(str, "BST A1000B Edage Computer", size) == 0) {  //A1000B  MEC
		return 16;
	} else if (strncmp(str, "BST A1000B ECU", size) == 0) {
		return 17;
	} else if (strncmp(str, "BST A1000B EVB", size) == 0) {
		return 18;
	} else if (strncmp(str, "BST A1000B ECARX-A", size) == 0) { 
		return 19;
	} else if (strncmp(str, "BST A1000B ECARX-B", size) == 0) { 
		return 20;
	} else if (strncmp(str, "BST A1000B FAD-A", size) == 0) {
		return 21;
	} else if (strncmp(str, "BST A1000B FAD-B", size) == 0) {
		return 22;
	} else if (strncmp(str, "BST A1000B FAD-M", size) == 0) {
		return 23;
	} else if (strncmp(str, "BST A1000B FAD-S", size) == 0) {
		return 24;
	} else if (strncmp(str, "BST A1000B FAW-A", size) == 0) {
		return 25;
	} else if (strncmp(str, "BST A1000B FAW-B", size) == 0) {
		return 26;
	} else if (strncmp(str, "BST A1000B IDDC21", size) == 0) {
		return 27;
	} else if (strncmp(str, "BST A1000B PAT", size) == 0) {
		return 28;			
	} else {
		return -1;
	}
}


static int tee_show_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &tee_show;
	return 0;
}

static ssize_t tee_show_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	int retvalue=0, i=0;
	u64 id[2] = { 0 }, global_chip_id;
	struct tee_read_data  tee_buf;
	memset(tee_buf.chipid, 0, sizeof(tee_buf.chipid));
	tee_buf.board_type = -1;
	printk("start to read tee.	\n");
	tee_get_cpu_id(id);
	id[1] = u64_swap(id[1]);
	if(id[1] == 0xffffffffffffffff){
		printk(KERN_DEBUG "otp random_data == 0xffffffffffffffff \n");
		global_chip_id = 0xffffffffffffffff;
	}else{
		printk(KERN_DEBUG "*********** before_hash: latter64 of OTP= %llx\n",id[1]);
		global_chip_id = hash_id(id[1]);
		printk(KERN_DEBUG "*********** after_hash: global_chip_id = %llx\n", global_chip_id);
	}
	u64tochar(global_chip_id, tee_buf.chipid);
	tee_buf.board_type = get_board_type();    //get board_type
	retvalue = copy_to_user(buf, &tee_buf, cnt);
	if(retvalue == 0){
		printk(KERN_DEBUG "** read tee_show ok! tee_buf.board_type = %d\n", tee_buf.board_type);	
	}else{
		printk(KERN_ERR "read tee_show failed, return! \n");
		return -1;
	}
	return 0;
}

static ssize_t tee_show_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

static int tee_show_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations tee_show_fops = {
	.owner = THIS_MODULE,
	.open = tee_show_open,
	.read = tee_show_read,
	.write = tee_show_write,
	//.release = 	tee_show_release,
};



static int __init tee_show_init(void)
{
	if (tee_show.major) {
		tee_show.devid = MKDEV(tee_show.major, 0);
		register_chrdev_region(tee_show.devid, TEE_SHOW_CNT, TEE_SHOW_NAME);
	} else {
		alloc_chrdev_region(&tee_show.devid, 0, TEE_SHOW_CNT, TEE_SHOW_NAME);
		tee_show.major = MAJOR(tee_show.devid);
		tee_show.minor = MINOR(tee_show.devid);
	}
	printk("tee_show major = %d,  minor = %d\r\n",tee_show.major, tee_show.minor);	
	

	tee_show.cdev.owner = THIS_MODULE;
	cdev_init(&tee_show.cdev, &tee_show_fops);
	
	cdev_add(&tee_show.cdev, tee_show.devid, TEE_SHOW_CNT);

	tee_show.class = class_create(THIS_MODULE, TEE_SHOW_NAME);
	if (IS_ERR(tee_show.class)) {
		return PTR_ERR(tee_show.class);
	}

	tee_show.device = device_create(tee_show.class, NULL, tee_show.devid, NULL, TEE_SHOW_NAME);
	if (IS_ERR(tee_show.device)) {
		return PTR_ERR(tee_show.device);
	}
	
	return 0;
}


static void __exit tee_show_exit(void)
{
	cdev_del(&tee_show.cdev);
	unregister_chrdev_region(tee_show.devid, TEE_SHOW_CNT);

	device_destroy(tee_show.class, tee_show.devid);
	class_destroy(tee_show.class);
}

module_init(tee_show_init);
module_exit(tee_show_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("JingFei");
