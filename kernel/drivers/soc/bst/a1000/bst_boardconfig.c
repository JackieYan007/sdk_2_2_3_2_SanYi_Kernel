#include <linux/fcntl.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/mtd/mtd.h>
#include <linux/bst_boardconfig.h>
#include <linux/tee_interface.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h> 
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/picp.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define EINVAL    22
#define ERR 0xffff

//add by qingpeng.gao
/* 
 * get soc side from device tree 
 * return: 
 *	0:fad-a/faw-a
 *	1:fad-b/faw-b
 * -1:others
 */
int get_soc_side(void)
{
	struct device_node *node;
	const char *str;
	int size;
	
    node = of_find_node_by_path("/");
	str = of_get_property(node, "model", &size);
	printk(KERN_ERR "of_get_property model:%s\r\n", str);
	
	if (strncmp(str, "BST A1000 FAD-A", size) == 0) {
		return 0;
	} else if (strncmp(str, "BST A1000 FAD-B", size) == 0) {
		return 1;
	} else if (strncmp(str, "BST A1000 FAW-A", size) == 0) {
		return 0;
	} else if (strncmp(str, "BST A1000 FAW-B", size) == 0) {
		return 1;
	} else if (strncmp(str, "BST A1000 FADV3-A", size) == 0) {
		return 0;
	} else if (strncmp(str, "BST A1000 FADV3-B", size) == 0) {
		return 1;
	} else {
		return -1;
	}
}
EXPORT_SYMBOL(get_soc_side);
//add by qingpeng.gao end

u64 bst_hash_64(u64 val, unsigned int bits)
{
    u64 hash = val;
 
    u64 n = hash;
    n <<= 18;
    hash -= n;
    n <<= 33;
    hash -= n;
    n <<= 3;
    hash += n;
    n <<= 3;
    hash -= n;
    n <<= 4;
    hash += n;
    n <<= 2;
    hash += n;
 
    return hash >> (64 - bits);
}

u64 hash_id(u64 id)
{
    u64 val;
    val = bst_hash_64(id, 64);
    return val;
}
EXPORT_SYMBOL(hash_id);

int u64tochar(u64 data, char *buf)
{      
	if(!buf)
		return -1;
    buf[0] = data >> 56;
    buf[1] = data >> 48;
    buf[2] = data >> 40;
    buf[3] = data >> 32;
    buf[4] = data >> 24;
    buf[5] = data >> 16;
    buf[6] = data >> 8;
    buf[7] = data;	
	
	return 0;
}

u64 chartou64(char *buf)
{
    u64 tmp ;
	if(!buf)
		return 0;
	tmp =  ((u64)buf[0] << 56) +  ((u64)buf[1] << 48) + ((u64)buf[2] << 40) + \
		   ((u64)buf[3] << 32) +  ((u64)buf[4] << 24) + ((u64)buf[5] << 16) + \
		   ((u64)buf[6] << 8) +  ((u64)buf[7]);
	
    printk("%llx \n",tmp);			
    return tmp;
}

int u32tochar(u32 data, char *buf)
{      
	if(!buf)
		return -1;
    buf[0] = data >> 24;
    buf[1] = data >> 16;
    buf[2] = data >> 8;
    buf[3] = data;	
	return 0;
}

u32 chartou32(char *buf)
{
    u32 tmp ;
	if(!buf)
		return 0;
	tmp =  ((u32)buf[0] << 24) + ((u32)buf[1] << 16) + \
		   ((u32)buf[2] << 8) +  ((u32)buf[3]);
	
    printk("%x \n",tmp);			
    return tmp;
}

u64 u64_swap(u64 data)
{
 u64 val = 0;
 int i=0;

 for(i=0; i<4; i++)
 {
  val |= (u64) (( data & ((u64 )0xff << (i*8))  ) << ((7-2*i)*8 ));
 }

 for(i=4; i<8; i++)
 {
  val |= (u64) (( data & ((u64)0xff << (i*8))  ) >> ((2*i-7)*8 ));
 }

 //printk("U64_swap: val = 0x%llx\n", val);
 return val;
}

u64 concat_hash(u64 data0, u64 data1)
{
    u64 val;
   u64 data;
    data = ((u64)(data0/2) + (u64)(data1/2));
    val = bst_hash_64(data, 64);
    return val;
}

/***********************************************
func:bst_get_other_soc_state
return:
*	-1：other side kernel not online
*	 0：other side kernel starting...
*	 1：other side kernel normal work
*************************************************/
int bst_get_other_soc_state(void)
{
	int value[100] = {0};
	int i = 0;

	for (i=0; i<100; i++) {
		value[i] = gpio_get_value(BST_FAD_INDEX_STAT_NUM_GPIO4);
	}
	/* cmp */
	for (i=0; i<99; i++) {
		if (value[i] != value[i+1]) {
			printk(KERN_WARNING "warning: other side kernel not online\n");
			return -1;
		}
	}
	if (value[0]) {
		printk(KERN_DEBUG "warning: other side kernel starting...\n");
		return 0;
	}
	
	return 1;
}
EXPORT_SYMBOL(bst_get_other_soc_state);


int bst_get_soc_id(void)
{
	int value = 0;
    bst_sysinfo_init();
	value = gpio_get_value(BST_FAD_INDEX_SOC_NUM_GPIO);
	return value;
}

/* set gpio5 OUT gpio4 IN */
int bst_set_heartbeat_infad(int val)
{
    if (val > 1 || val < 0) {
        return -1;
    }


    gpio_set_value(BST_FAD_INDEX_STAT_NUM_GPIO5, val);
    
    return 0;
}
EXPORT_SYMBOL(bst_set_heartbeat_infad);

int bst_get_heartbeat_infad(void)
{
    int val;

    val = gpio_get_value(BST_FAD_INDEX_STAT_NUM_GPIO4);

    return val;
}
EXPORT_SYMBOL(bst_get_heartbeat_infad);

int bst_get_soc_board_type(void)
{
	struct device_node *node;
	const char *str;
	int size;
	
    node = of_find_node_by_path("/");
	str = of_get_property(node, "model", &size);
	
	if (strncmp(str, "BST A1000 FAD-A", size) == 0) {
		return BSTA1000_BOARD_FADA;
	} else if (strncmp(str, "BST A1000 FAD-B", size) == 0) {
		return BSTA1000_BOARD_FADB;
	} else if (strncmp(str, "BST A1000 FAW-A", size) == 0) {
		return BSTA1000_BOARD_FAWA;
	} else if (strncmp(str, "BST A1000 FAW-B", size) == 0) {
		return BSTA1000_BOARD_FAWB;
	} else if (strncmp(str, "BST A1000 EVB", size) == 0) {
		return BSTA1000_BOARD_EVB;
    } else if (strncmp(str, "BST A1000B EVB", size) == 0) {
		return BSTA1000B_BOARD_EVB;
    } else if (strncmp(str, "BST A1000B FAD-A", size) == 0) {
		return BSTA1000B_BOARD_FADA;
	} else if (strncmp(str, "BST A1000B FAD-B", size) == 0) {
		return BSTA1000B_BOARD_FADB;	
	} else {
		return -1;
	}
}
EXPORT_SYMBOL(bst_get_soc_board_type);


void bst_init_heatbeat(void)
{
    /* gpio_4 in; gpio_5 out low level */
	gpio_direction_output(BST_FAD_INDEX_STAT_NUM_GPIO5, LOW_LEVEL);
	gpio_direction_input(BST_FAD_INDEX_STAT_NUM_GPIO4);
}
EXPORT_SYMBOL(bst_init_heatbeat);

int bst_init_gpio4_5(void)
{
	int err = -1; 

	err = gpio_request(BST_FAD_INDEX_STAT_NUM_GPIO5, "debug5");
	if (err)
		return -EBUSY;
		
	err = gpio_request(BST_FAD_INDEX_STAT_NUM_GPIO4, "debug4");
	if (err)
		goto err_req_gpio_debug4;

	/* gpio_4 in; gpio_5 out low level */
	gpio_direction_output(BST_FAD_INDEX_STAT_NUM_GPIO5, LOW_LEVEL);
	gpio_direction_input(BST_FAD_INDEX_STAT_NUM_GPIO4);

	printk("bst_init_gpio4_5 init success\n");

	return 0;

err_req_gpio_debug4:
	gpio_free(BST_FAD_INDEX_STAT_NUM_GPIO5);

	return -EBUSY;
}
EXPORT_SYMBOL(bst_init_gpio4_5);

/****************************
**func:bst_sysinfo_init
****************************/
int bst_sysinfo_init(void)
{
	int err = -1; 
	const char *str;
	int size;
    struct device_node *node;

    node = of_find_node_by_path("/");
    str = of_get_property(node, "model", &size);
    if (strncmp(str, "BST A1000 FADV3-B", size) == 0){
        err = gpio_request(BST_FADV3B_RESET_N_GNSS_GPIO, "gpio_12");
        if (err)
            return -EBUSY;
        /* output */
        gpio_direction_output(BST_FADV3B_RESET_N_GNSS_GPIO,LOW_LEVEL);
    }else if(strncmp(str, "BST A1000B Edage Computer", size) == 0){    //A1000B-MEC gpio config for 5G/PWR/RESET
		//5G pwr and reset:gpio123  gpio124
		err = gpio_request(BST_MEC_INDEX_PWRKEY_5G, "gpio_123");
		if(err)
			return -EBUSY;
		err = gpio_request(BST_MEC_INDEX_RESET_5G, "gpio_124");
		if(err)
			return -EBUSY;
		//fan_ctrl:gpio12
        err = gpio_request(BST_MEC_INDEX_FAN_CTRL, "gpio_12");
        if (err)
            return -EBUSY;
        /* output */
        gpio_direction_output(BST_MEC_INDEX_FAN_CTRL, HIGH_LEVEL);
		gpio_direction_output(BST_MEC_INDEX_PWRKEY_5G, LOW_LEVEL);
		gpio_direction_output(BST_MEC_INDEX_RESET_5G, HIGH_LEVEL);
		msleep(200);
		gpio_direction_output(BST_MEC_INDEX_RESET_5G, LOW_LEVEL);
	}

	err = gpio_request(BST_FAD_INDEX_SOC_NUM_GPIO, "gpio_29");
	if (err)
		return -EBUSY;

	return 0;
}


static int bst_identify_init(void)
{
	printk(KERN_INFO "bst_identify_probe\n");
	bst_sysinfo_init();

    return 0;
}

static void  bst_identify_exit(void) 
{
	
}

late_initcall(bst_identify_init);
module_exit(bst_identify_exit);
