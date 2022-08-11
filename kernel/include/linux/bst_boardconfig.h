#ifndef BST_BOARDCONFIG_H
#define BST_BOARDCONFIG_H

#include <stdbool.h>

#define BST_IDENTIFY_DRV_NAME			"bst_identify"

#define BST_LEN_24	24
#define BST_LEN_8	8
#define BST_LEN_4	4
#define DEF_BOARD_CONFIG_LEN   56
#define USER_BOARD_CONFIG_LEN  48
#define BOARD_CONFIG_LEN_I2C   48


#define BST_BOARD_ID_LEN     32
#define BST_BOARD_ID_LEN_I2C 24

#define BST_BOARD_CONFIG_QSPI_BASE 0x100000     //31M
//#define BST_BOARD_CONFIG_QSPI_BASE 0xf00000     //31M
#define BST_BOARD_CONFIG_QSPI_USER 0x101000     //31M + 4KB
//#define BST_BOARD_CONFIG_QSPI_USER 0xf01000     //31M + 4KB


#define BST_BOARD_ID_QSPI_BASE    0x110000      //31M + 64KB
//#define BST_BOARD_ID_QSPI_BASE    0xf10000      //31M + 64KB
#define BST_BOARDID_VALIDITY_QSPI 0x110020      //31M + 64KB + 32Byte
//#define BST_BOARDID_VALIDITY_QSPI 0xf10020      //31M + 64KB + 32Byte

#define BST_BOARD_QSPI_ERASE_LEN 0x10000

#define DEF_BST_BOARD_CONFIG_EEPROM_BASE_A  0
#define USER_BST_BOARD_CONFIG_EEPROM_BASE_A 48

#define DEF_BST_BOARD_CONFIG_EEPROM_BASE_B  128
#define USER_BST_BOARD_CONFIG_EEPROM_BASE_B 176


#define BST_BOARD_ID_EEPROM_BASE_A 96
#define BST_BOARD_ID_EEPROM_BASE_B 224


/* gpio index soc num */
#define BST_FAD_INDEX_SOC_NUM_GPIO			29
#define BST_FAD_INDEX_STAT_NUM_GPIO4		4
#define BST_FAD_INDEX_STAT_NUM_GPIO5		5
#define BST_MEC_INDEX_PWRKEY_5G				123
#define BST_MEC_INDEX_RESET_5G				124
#define BST_MEC_INDEX_FAN_CTRL				12

/* FADV3 GNSS RESET */
#define BST_FADV3B_RESET_N_GNSS_GPIO        12
#define HIGH_LEVEL							1
#define LOW_LEVEL							0

typedef unsigned int u32;
typedef unsigned long long u64;

extern struct at24_data *bst_at24;

#define BST_DEBUG   (0)

#if BST_DEBUG
#define bst_pr(fmt, ...) \
    printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define bst_pr(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif


enum bst_qspi_mtdpart {
        BST_QSPI_MTD0			= 0,
	    BST_QSPI_MTD1			= 1,
        BST_QSPI_MTD2			= 2,
        BST_QSPI_MTD3			= 3,
	    BST_QSPI_MTD4			= 4,
        BST_QSPI_MTD5			= 5,
}; /* enum */

enum bst_qspi_dev_minor {
        BST_QSPI_MTD0_DEV_NO			= 0,
	    BST_QSPI_MTD0RO_DEV_NO			= 1,
        BST_QSPI_MTD1_DEV_NO			= 2,
        BST_QSPI_MTD1RO_DEV_NO			= 3,
	    BST_QSPI_MTD2_DEV_NO			= 4,
        BST_QSPI_MTD2RO_DEV_NO			= 5,
        BST_QSPI_MTD3_DEV_NO			= 6,
	    BST_QSPI_MTD3RO_DEV_NO			= 7,
        BST_QSPI_MTD4_DEV_NO			= 8,
        BST_QSPI_MTD4RO_DEV_NO			= 9,
	    BST_QSPI_MTD5_DEV_NO			= 10,
        BST_QSPI_MTD5RO_DEV_NO			= 11,
}; /* enum */

enum bst_data_source {
        FROM_OTA			= 0,
	    FROM_QSPI			= 1,
        FROM_EEPROM			= 2,
}; /* enum */

enum bst_data_type {
        BST_MAC0			= 0,
	    BST_MAC1			= 1,
        BST_PCIE0			= 2,
	    BST_PCIE1			= 3,
        BST_USBID0			= 4,
	    BST_USBID1			= 5,
        BST_USBINFO0			= 6,
	    BST_USBINFO1			= 7,
}; /* enum */

enum board_type {
    BSTA1000_BOARD_EVB = 1,
    BSTA1000_BOARD_FADA,
    BSTA1000_BOARD_FADB,
    BSTA1000_BOARD_EC,
    BSTA1000_BOARD_FAWA,
    BSTA1000_BOARD_FAWB,
    BSTA1000_BOARD_ECU,
    BSTA1000_BOARD_EVBL,
    BSTA1000_BOARD_FADM,
    BSTA1000_BOARD_FADS,
    BSTA1000_BOARD_PAT,
    BSTA1000_BOARD_ECV3,
    BSTA1000_BOARD_JAC20,
    BSTA1000_BOARD_JAC21,   
	BSTA1000B_BOARD_EVB,
	BSTA1000B_BOARD_FADA,
	BSTA1000B_BOARD_FADB,
	BSTA1000B_BOARD_JAC21,
	BSTA1000B_BOARD_EC,
  
	BOARD_TYPE_MAX
}; /* enum */

struct def_board_info_config {
	unsigned char           len[4];
	unsigned char           crc[4];
	unsigned char			def_eth_addr0[BST_LEN_8];
	unsigned char			def_eth_addr1[BST_LEN_8];
	unsigned char			def_pcie0[BST_LEN_8];
	unsigned char			def_pcie1[BST_LEN_8];
	unsigned char			def_usb_id0[BST_LEN_4];
	unsigned char			def_usb_id1[BST_LEN_4];
	unsigned char			def_usb_info0[BST_LEN_4];
	unsigned char			def_usb_info1[BST_LEN_4];
};

struct user_board_info_config {
	unsigned char			user_eth_addr0[BST_LEN_8];
	unsigned char			user_eth_addr1[BST_LEN_8];
	unsigned char			user_pcie0[BST_LEN_8];
	unsigned char			user_pcie1[BST_LEN_8];
	unsigned char			user_usb_id0[BST_LEN_4];
	unsigned char			user_usb_id1[BST_LEN_4];
	unsigned char			user_usb_info0[BST_LEN_4];
	unsigned char			user_usb_info1[BST_LEN_4];
};

struct bst_boardid_validity_config {
	unsigned char           validity[BST_LEN_4];
};


struct bst_board_id_config {
	unsigned char           len[BST_LEN_4];
	unsigned char           crc[BST_LEN_4];
	unsigned char			global_chip_id[BST_LEN_8];
	unsigned char			global_board_id[BST_LEN_8];
	unsigned char			local_chip_id[BST_LEN_4];
	unsigned char			local_board_id[BST_LEN_4];
};

struct bst_board_id_config_i2c {
	unsigned char			global_chip_id[BST_LEN_8];
	unsigned char			global_board_id[BST_LEN_8];
	unsigned char			local_chip_id[BST_LEN_4];
	unsigned char			local_board_id[BST_LEN_4];
};



int u64tochar(u64 data, char *buf);
u64 chartou64(char *buf);
int u32tochar(u32 data, char *buf);
u32 chartou32(char *buf);
u64 u64_swap(u64 data);
void reverse(char *p , int size);

void crc32_table_create(void);
unsigned int crc32_calculate(void *buf ,unsigned int size);

int get_bst_board_id_qspi(struct bst_board_id_config* info);
int set_bst_board_id_qspi(struct bst_board_id_config* info);
int get_boardid_validty_qspi(struct	bst_boardid_validity_config* info);
int set_boardid_validity_qspi(struct bst_boardid_validity_config* info);
int set_bst_board_id_i2c(struct  bst_board_id_config_i2c* info);
int get_bst_board_id_i2c(struct	bst_board_id_config_i2c* info);


int get_def_board_info_qspi(struct  def_board_info_config* info);
int get_user_board_info(struct user_board_info_config* info, u32 source);
int set_def_board_info_qspi(struct def_board_info_config* def_info);
int set_user_board_info(struct user_board_info_config* user_info, u32 source);
int set_def_board_info_i2c(struct user_board_info_config* user_info);
int get_def_board_info_i2c(struct user_board_info_config* info);
u64 get_eth_mac(u32 num, u32 source, u32 user);
u32 set_eth_mac(u64 mac_addr, u32 num, u32 destination, u32 user);

unsigned char* get_bst_id_data(struct       bst_board_id_config* info, int type);
int set_bst_id_data(struct       bst_board_id_config* info, int type,char* buf);
u64 get_global_board_id(void);
u64 get_global_chip_id(void);
u32 get_local_chip_id(void);
u32 get_local_board_id(void);
int bst_sysinfo_init(void);
int bst_sys_id_init(struct bst_board_id_config* info);
u64 hash_id(u64 id);
u64 concat_hash(u64 data0, u64 data1);
int bst_get_soc_id(void);
int get_soc_side(void);
int bst_set_heartbeat_infad(int val);
int bst_get_heartbeat_infad(void);
int bst_get_other_soc_state(void);
void bst_init_heatbeat(void);
int bst_init_gpio4_5(void);
#endif
