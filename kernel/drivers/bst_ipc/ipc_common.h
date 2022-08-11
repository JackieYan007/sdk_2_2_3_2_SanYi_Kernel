#ifndef IPC_COMMON_H
#define IPC_COMMON_H

#include <linux/types.h>
#include <asm/io.h>

#include "ipc_interface.h"
#include "ipc_session.h"

// ipc configuration definition
#define IPC_LOG_SUPPORT                     // use to define ipc log format
#define ENABLE_APU_SAMPLE                   // use to apu sample initiate
#define MSG_DUMP                            // use to support ipc message dump for ipcctl

// #define ENABLE_SRC_DST_SAME              // use to support set same resource core and destination core function
// #define MSG_SIZE_EXTENSION               // use to support 64bit message extending to 128bit 
// #define DUMP_MSG_HASH_TABLE              // use to dump ipc message hash map
// #define PACKAGE_MSG_EXTENSION            // use to support ipc package message transferring function
#define IPC_SRC_DEFINITION_SUPPORT       // use to support ipc resource core definition


#define IPC_INFO_MAX_NUM    4096

/*cmd max value*/
#define MSG_CMD_MAX 255
#define REGISTER_MAP_MAX 8
#define SUBSCRIPTION_MAP_MAX 18
#define MSG_SESSION_MAX 64  
#define SIGNAL_START_CMD 8

//ipc cache coherency command define
#define ipc_isb() __asm__ __volatile__ ("isb" : : : "memory")
#define ipc_dsb() __asm__ __volatile__ ("dsb sy" : : : "memory")
#define ipc_dmb() __asm__ __volatile__ ("dmb sy" : : : "memory")

//ipc log define
extern int32_t sysfs_get_log_level(void);
#ifdef IPC_LOG_SUPPORT
#define IPC_TRACE_PRINTK(format,...)	\
    do{ \
        if(sysfs_get_log_level() >= LOG_LEVEL_INFO) \
            printk(KERN_INFO"[D][%s]: %s %d: " format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__);  \
    }while(false)
#else
#define IPC_TRACE_PRINTK(format,...)
#endif

#define IPC_DEV_ERR(dev, format,...) dev_err(dev, "[%s]: ERROR: %s %d: " format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__)

#ifdef IPC_LOG_SUPPORT
#define IPC_LOG(level, format, ...) \
    do{ \
        if(sysfs_get_log_level() >= LOG_LEVEL_INFO) \
            printk("[%s]: %s %d: " format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }while (false)

/* print info */
#define IPC_INFO_PRINT(format, ...) \
    do { \
        printk(KERN_ERR format "\n", ##__VA_ARGS__); \
    }while (false)

/*3*/
#define IPC_LOG_ERR(format, ...) \
    do{ \
        if(sysfs_get_log_level() >= LOG_LEVEL_ERR) \
            printk(KERN_ERR"[E][%s]: %s %d:" format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }while (false)

/*4*/
#define IPC_LOG_WARNING(format, ...) \
    do{ \
        if(sysfs_get_log_level() >= LOG_LEVEL_WARNING) \
            printk(KERN_WARNING"[W][%s]: %s %d:" format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }while (false)

/*"6"*/
#define IPC_LOG_INFO(format, ...) \
    do{ \
        if(sysfs_get_log_level() >= LOG_LEVEL_INFO) \
            printk(KERN_INFO"[I][%s]: %s %d:" format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }while (false)

/*"7"*/
#define IPC_LOG_DEBUG(format, ...) \
    do{ \
        if(sysfs_get_log_level() >= LOG_LEVEL_DEBUG) \
            printk(KERN_DEBUG"[D][%s]: %s %d:" format "\n", IPC_DRIVER_NAME, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    }while (false)

#else
#define IPC_LOG(level, format, ...)
#define IPC_LOG_ERR(format, ...)
#define IPC_LOG_WARNING(format, ...)
#define IPC_LOG_INFO(format, ...)
#define IPC_LOG_DEBUG(format, ...)
#endif // IPC_LOG_SUPPORT


/**
 * 64 bit and less is supported
 * sample usage:  find_bit_zero(0xafffffff0, 64, 0)
 *
 * @return lowest bit of 0 bit
 */
int32_t find_bit_zero(uint64_t number, int32_t bit_num, unsigned bit_offset);

/**
 * 64 bit and less is supported
 * sample usage:  find_1_bit(0xafffffff0, 64, 0)
 *
 *  @return lowest bit of 1 bit
 */
int32_t find_1_bit(uint64_t number, int32_t bit_num, unsigned bit_offset);

static inline int get_leading_zero_num(size_t value)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
	int num;
#pragma GCC diagnostic pop

	__asm__("clz %0, %1": "=r"(num): "r"(value):);

	return num;
}

static inline int get_msb_bit1_index(size_t value)
{
	return ((sizeof(value) << 3) - 1 - get_leading_zero_num(value));
}

static inline int get_1bit_count(uint64_t value)
{
    uint64_t count;
    for (count=0; value; count++)
        value &= value - 1;
    return count;
}

static inline int get_all_1bit(uint64_t map, uint8_t id[]) {
	int cnt = 0;
	while (map > 0) {
		int next_psn = __builtin_ffs(map) - 1;
		map &= (map-1);
		id[cnt++] = next_psn;
	}
	return cnt;
}

//ipc driver diagnostic config
enum log_level_e{
    LOG_LEVEL_ERR,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
};

struct diag_info_send_err {
    int32_t ipc_no_ready;
    int32_t session_invalid;
    int32_t no_ACK;
    int32_t queue_full;
};

struct diag_info_recv_err {
    int32_t ipc_no_ready;
    int32_t session_invalid;
    int32_t queue_empty;
    int32_t timeout;
};

struct diag_info {
    int32_t session_id;
    enum ipc_core_e src;
    enum ipc_core_e dst;
    int32_t num_of_send_msg;
    int32_t num_of_recv_msg;
    struct diag_info_send_err send_err;
    struct diag_info_recv_err recv_err;
};

#endif // IPC_COMMON_H
