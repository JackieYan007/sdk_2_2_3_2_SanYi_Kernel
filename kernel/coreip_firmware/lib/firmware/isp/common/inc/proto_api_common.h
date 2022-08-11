/*********************************************************************************
*This file contains proprietary information that is the sole intellectual
*property of Black Sesame Technologies, Inc. and its affiliates.  No portions
*of this material may be reproduced in any form without the written permission
*of:
*
*Black Sesame Technologies, Inc. and its affiliates
*2255 Martin Ave. Suite D
*Santa Clara, CA  95050
*Copyright @2016: all right reserved.
*/
/**********************************************************************************
* @file         proto_api_common.h
* Version:      @PACKAGE_VERSION@
* @brief        This is a brief description.
*               +----------------------+
*               |  FBUF partition      |
*               +----------------------+
*
*               +----------------------+
*               |  SLAB partition      |
*               +----------------------+
*               |  CMDP partition      |
*               +----------------------+
*               |  INIT partition      |
*        0x0000 +----------------------+
*
*      INIT partition:
*               +----------------------+
*               |  RODA1               |
*               +----------------------+
*               |  RODA0               |
*               +----------------------+
*               |  DATA                |
*               +----------------------+
*               |  EP                  |
*               +----------------------+
*               |  FBUF info           |
*               +----------------------+
*               |  SLAB info           |
*               +----------------------+
*               |  Control             |
*        0x0000 +----------------------+
* */

#ifndef _SONE_PROTO_API_COMMON_H
#define _SONE_PROTO_API_COMMON_H

#include "proto_api_types.h"
/*
#define uint32_t  unsigned int
#define  int32_t    signed int
#define uint16_t  unsigned short
#define  int16_t    signed short
#define uint8_t   unsigned char
#define  int8_t     signed char
*/
#if 0
typedef struct sPROTOCOL_ROOT
{
  void* init;
  void* cmdp;
  void* slab;

}tSone;
#endif

#define API_VERSION   0x0037
#define DOC_VERSION   0x0037
#define SW_VERSION    'A'

#define RET_SUCCESS   0
#define RET_FAILED    -1

#define LOCAL_MEM_ACCESS   0
#define SONE_MIN(X,Y) ((X)>(Y)?(Y):(X))
#define SONE_MAX(X,Y) ((X)>(Y)?(X):(Y))

enum {
	DRV_CH_INDEX = 0,
	FW_CH_INDEX = 1,
	SONE_MAX_CHNUM = 2
};

// list table used by CMDP payload(in/out), SLAB, FBUF
/* ******************************************************
 * circular memory buffer control algorithm:
 *   name[4]  == 0x00000000: can be replace
 *   *header, *tail, *cur as circular element point
 * */

/* struct memory_list, struct memory_section: real system data struct
 * struct gen_memory_list, struct gen_memory_section: for data struct of generation
 * */
struct memory_list {        //for 64bits system
  struct memory_list* prev;
  struct memory_list* next;
  int    len_div32;
  char   name[4];                   //0x00000000: invalid block, can reallocated
  uint32_t resv[2];
  uint32_t first_4bytes;
};

struct memory_section_64b {      //for 64bits system
  char     magic[4];
  uint32_t resv0;
  struct memory_list* start;
  void* end;
  struct memory_list* cur;
  struct memory_list* header;
  struct memory_list* tail;
  unsigned  alloc_num:16;
  unsigned  loop_flag:1;
  unsigned  systype:1;
  unsigned  resvbit:14;
  uint32_t  resv1[3];
  struct memory_list list;  //32bytes align
};

struct generic_link {
  uint32_t length;
  uint32_t addr;
};

// list table used by CMDP payload(in/out), SLAB, FBUF
struct gen_memory_list_32b {      //for 32bits system
  uint32_t prev_addr;
  uint32_t resv_a;
  uint32_t next_addr;
  uint32_t resv_b;
  int      len_div32;
  char     name[4];
  uint32_t resv[2];
  uint32_t first_4bytes;
};


struct gen_memory_section_32b {    //for 32bits system
  char  magic[4];
  uint32_t  resv0;
  uint32_t  start_addr;
  uint32_t  resv_a;
  uint32_t  end_addr;
  uint32_t  resv_b;
  uint32_t  cur_addr;
  uint32_t  resv_c;
  uint32_t  header_addr;
  uint32_t  resv_d;
  uint32_t  tail_addr;
  uint32_t  resv_e;
  unsigned  alloc_num:16;
  unsigned  loop_flag:1;
  unsigned  systype:1;
  unsigned  resvbit:14;
  uint32_t  resv1[3];
  struct gen_memory_list_32b list;  //32bytes align
};

#define SONE_MAX_FBUF_TABNUM    8
//-----------INIT--------------------------
// 32 + 32Bytes
struct parti_init_control {
  char     magic_sone[4];     //"SONE"
  uint16_t api_version;
  uint16_t doc_version;
  uint32_t base_addr;
  uint32_t reserve0[5];

  char magic_init[4];    //"INIT"
  unsigned init_roda0_offset_div32:8;
  unsigned rodata_targ_addr_div4:  16;
  unsigned fbuf_num:               3;
  unsigned base_addr_high:         3;          // for A1000, bus address >32bits, here is bits [33:31]
  unsigned load_mediainfo_disable: 1;          // disable mediainfo loading in INIT
  unsigned trace_print_mode:       1;          //0: print by F/W(image-tool), 1: print by APU
  uint32_t reserve1[3];
  uint32_t slab_base;
  uint32_t cmdp_base[SONE_MAX_CHNUM];
  uint32_t fbuf_table[SONE_MAX_FBUF_TABNUM];
};

#define SONE_MAX_ENTITY_NUM    32
#define SONE_MEDIAINFO_SIZE    4096
#define SONE_MEDIA_RESV      (SONE_MEDIAINFO_SIZE-4-SONE_MAX_ENTITY_NUM*sizeof(struct media_entity))

struct init_object_package {
  uint32_t length;
  char     name[4];
  unsigned type:4;
  unsigned resv:28;
  uint32_t res[5];
  uint32_t content;         //content's first 4 bytes
};

struct init_object_index {
  char     name[4];
  uint32_t address;
};

struct media_v_pad {
  unsigned width:16;
  unsigned height:16;
  unsigned format:8;
  unsigned frame_rate:8;
  unsigned resv:16;
  uint32_t frmbuf_addr;
};

struct media_entity {
  char     name[4];
  unsigned srcpad_num:4;    //max: 16 pad
  unsigned sinkpad_num:4;   //max: 16 pad
  struct media_v_pad srcpad0;
  struct media_v_pad sinkpad0;

  struct generic_link entity_property_addr;    // entity extension reserve: SLAB addr
  struct generic_link srcpad_property_addr;    // souce pad extension reserve: SLAB addr
  struct generic_link sinkpad_property_addr;   // sink pad extension reserve: SLAB addr
  struct generic_link resv_info_addr;          // SLAB addr
};

struct parti_init_mediainfo {
  unsigned media_entity_num:8;
  unsigned resv:24;
  struct   media_entity entity[SONE_MAX_ENTITY_NUM];
  uint8_t  resv2[SONE_MEDIA_RESV];
};

#define PARTI_INIT_SYSCFG_LEN  512
#define CORE_VENDOR_DESC_LEN   64
#define CORE_GLOBAL_CFG_LEN    64
struct core_vendor_desc{
  char corp_name[16];
  char IP_name[16];
  char short_name;
  uint32_t resv[(CORE_VENDOR_DESC_LEN - 16 - 16 - 4) / 4];
};

struct core_global_cfg {
  char platform[4];
  char host_env[4];
  uint32_t trace_mask;
  uint32_t ipc_reg_base;
  uint32_t vsp_reg_base;
  uint32_t isp_reg_base;
  uint32_t resv[(CORE_GLOBAL_CFG_LEN - 4 * 6) / 4];
};

struct parti_init_syscfg {
  struct core_vendor_desc vendor;
  struct core_global_cfg  corecfg;
  uint32_t resv[(PARTI_INIT_SYSCFG_LEN - 128) / 4];
};


//
#define SONE_ENTITY_MAX_NUM  32
#define SONE_INIT_DATA_LEN  512
#define SONE_RODA0_MAXLEN   1024
#define SONE_RODA1_MAXLEN   0x40000
typedef struct sPROTOCOL_INIT
{
  struct parti_init_control    ctrl;
  struct parti_init_mediainfo  mediainfo[SONE_ENTITY_MAX_NUM];

  struct parti_init_syscfg cfg;
  uint32_t rodata_resv[SONE_RODA0_MAXLEN / 4];

}tSoneInit;

//-----------CMDP-------------------------

// payload memory size: 1MB
#define SONE_MEDIA_PAYLOAD_MEMSIZE   0x100000
#define SONE_MEMORY_EMPTYSIZE        0x400
#define SONE_MEDIA_QSIZE             256

enum {
  SONE_CMD_QIN_P0,
  SONE_CMD_QIN_P1,
  SONE_MSG_QOUT_P0,
  SONE_MSG_QOUT_P1,
  SONE_QUEUE_NUM
};

// 512
struct ep_info {
  char     name[4];
  //unsigned thread_id:8;
  unsigned port_id:8;    //for RISCV, it's sync_counter ID; Driver, it's port
  unsigned resvbit:24;
  uint32_t resv[2];
};

#define SONE_EP_MAX_NUM   32


#define SONE_EPINFO_SIZE    1024
#define SONE_EP_RESV      (SONE_EPINFO_SIZE-4  \
        -(SONE_EP_MAX_NUM*sizeof(struct ep_info))   )

struct parti_cmdp_ep {
  char    ch_name;
  uint8_t ch_id;
  uint8_t ep_num;
  uint8_t resv;

  struct ep_info  EP[SONE_EP_MAX_NUM];

  uint32_t ep_resv[SONE_EP_RESV / 4];
};

struct media_oneQ_ctrl {
  unsigned total_num_minus1:9;
  unsigned curpos_in:9;
  unsigned cur_in_num:9;
  unsigned resvbit:5;
  uint32_t resv;
};

typedef struct media_queue_ctrl{
  struct media_oneQ_ctrl p0;
  uint32_t reserve[2];
}tQCtrl;

struct cmdmsg_attachment {
  uint32_t timestamp_send;
  uint32_t timestamp_ack;
  uint32_t resv[6];
};

struct media_hdr_info{
  unsigned cmd_type_main:6;
  unsigned cmd_type_minor:10;
  unsigned usr_flag0:1;
  unsigned usr_flag1:1;
  unsigned usr_flag2:1;
  unsigned sync_mode:1;          //0:  sync mode, 1: async mode
  unsigned multicast_en:1;
  unsigned attachment_en:1;
  unsigned follow_pack_num:4;
  unsigned reserved:6;
};

struct media_magic_data{
  char magic[2];
  unsigned sequ_cnt:4;
  unsigned checksum:3;
  unsigned priority:1;
  unsigned cmd_status:1;   //0: done, 1: unfinished
  unsigned uid:7;          //Unique ID for matching between cmd and msg
};

struct media_cmdmsg_hdr{
  struct media_magic_data magic;
  struct media_hdr_info hdr_info;
  char src[4];
  char dst[4];
};

struct media_command{
  struct media_cmdmsg_hdr cmd_hdr;
  uint32_t user_cmd_data[4];
};

struct media_cmd_queue {
  char   magic[4];    //"CMD0"
  uint32_t resv0[7];
  tQCtrl ctrl_cmd[SONE_MAX_CHNUM];   // for every target core
  struct media_command c0[SONE_MEDIA_QSIZE];

  uint8_t  payload_cmd[SONE_MEDIA_PAYLOAD_MEMSIZE + SONE_MEMORY_EMPTYSIZE];

  struct cmdmsg_attachment ca0[SONE_MEDIA_QSIZE];
};

struct media_cmdmsg_queue {
  struct parti_cmdp_ep      ep;
  struct media_cmd_queue cqueue;
  uint32_t empty0[1024  /4];
};

struct channel_header {
  char magic[4];           //"CMDP"
  unsigned chnum:4;
  unsigned resvbit:28;
  uint32_t resv[32/4-2];
};

typedef volatile struct media_cmdmsg_channels {
  struct channel_header hdr;
  struct media_cmdmsg_queue ch[SONE_MAX_CHNUM];
                                      //A55 driver: ch_id=0, "D"
                                      //FW:  ch_id=1, "F"
}tSoneCmdp;


//-----------SLAB-------------------------
#define SONE_MAX_SLAB_ALLOC_NUM    16
#define SONE_SLAB_BLKSIZE          32

//-----------FBUF-------------------------
#define SONE_MAX_FBUF_ALLOCNUM     32


#define SONE_MEDIA_SLAB_BUFSIZE   0x100000
#define SONE_MEDIA_FBUF_BUFSIZE   0x1000000

#define SONE_MAGIC_SONE       "SONE"
#define SONE_MAGIC_INIT       "INIT"
#define SONE_MAGIC_CMDP       "CMDP"
#define SONE_MAGIC_SLAB       "SLAB"
#define SONE_MAGIC_FBUF       "FBUF"

#define SONE_MAGIC_CMDP_C0       "CMD0"
#define SONE_MAGIC_CMDP_M0       "MSG0"

#define SONE_MAGIC_CMDPAYLOAD_IN  "cmd0"
#define SONE_MAGIC_CMDPAYLOAD_OUT "cmd1"

#define SONE_MAGIC_FBUF0     "FRM0"
#define SONE_MAGIC_FBUF1     "FRM1"
#define SONE_MAGIC_FBUF2     "FRM2"

#define SONE_CORP_NAME       "Black Sesame Inc"

enum {
  CMD_CAM_DEV = 0,
  CMD_ISP_DEV,
  CMD_VOUT_DEV,
  CMD_H264ENC_DEV,
  CMD_GWC_DEV,
  CMD_ISP_TEST,
  CMD_VSP_TEST
};

// cmd_type_minor definition
enum {
  MINOR_INVALID = 0,//rsv
  //to arm
  MINOR_BOOT_DONE,
  MINOR_ABNORMAL,
  MINOR_LOG_PRINT,
  MINOR_SYNC_ISP_VIEW_FRAME_DONE = 0x08,
  MINOR_SYNC_ISP_RAW_FRAME_DONE = 0x09,

  MINOR_VSP_VOUT_UNDERFLOW = 0X10,
  MINOR_VSP_GWARP_FRAME_DONE = 0x14,
  MINOR_VSP_ENCODER_FRAME_DONE = 0x18,

  //from/to arm
  MINOR_GET_VERSION       = 0x20, //realtime
  MINOR_GET_REG,                  //realtime
  MINOR_GET_I2C,                  //realtime
  MINOR_ECHO_TEST,                //realtime
  MINOR_ISP_CAPTURE_RECONF,       //boot up, after MINOR_ISP_BOOTLD_RECONF reply
  MINOR_ISP_GET_GLB_PARAM = 0x28, //realtime
  MINOR_ISP_GET_ALG_PARAM,        //realtime
  MINOR_ISP_I2C_BUS_CTRL,         //realtime
  MINOR_ISP_SET_PWLINFO,          //realtime  use payload,  reply, only for A1000B0
  MINOR_ISP_BOOTLD_RECONF = 0x30, //boot up,  use payload,  reply
  MINOR_ISP_BOOTLD_ALGO_BIN,      //boot up,  use payload,  reply
  MINOR_ISP_BOOTLD_IQ_BIN,        //boot up,  use payload,  reply
  MINOR_ISP_START,                //realtime,               reply
  MINOR_ISP_SET_VIEWINFO,         //realtime, use payload,  reply
  MINOR_ISP_SET_IQINFO,           //realtime, use payload,  reply
  MINOR_ISP_GET_IQINFO,           //realtime,               reply
  MINOR_ISP_SET_CTRLINFO,         //realtime,               reply
  MINOR_ISP_GET_CTRLINFO,         //realtime,               reply
  MINOR_ISP_SAFETY,               //realtime,               reply
  MINOR_ISP_DSP_SYNC,             //realtime,               reply
  MINOR_ISP_DSP_IPC,              //realtime,               reply

  //from arm
  MINOR_SET_REG           = 0x40, //realtime
  MINOR_SET_I2C,                  //realtime
  MINOR_ISP_SET_GLB_PARAM = 0x50, //realtime
  MINOR_ISP_SET_ALG_PARAM,        //realtime
  MINOR_ISP_NEW_VIEW_FRAME_BUF,   //realtime
  MINOR_ISP_CAM_OPEN,             //realtime
  MINOR_ISP_CAM_CLOSE,            //realtime
  MINOR_ISP_FILE2FILE_START,
  MINOR_ISP_IPC_CFG,

  MINOR_VSP_VOUT_OPEN     = 0x60,
  MINOR_VSP_VOUT_START,
  MINOR_VSP_VOUT_CLOSE,
  MINOR_VSP_VOUT_RESULOTION,
  MINOR_VSP_VOUT_FORMAT,    //420, 422
  MINOR_VSP_VOUT_INVALIDATE,
  MINOR_VSP_VOUT_OSD_REGION,
  MINOR_VSP_VOUT_OSD_INVALIDATE,
  MINOR_VSP_VOUT_OSD_ON,
  MINOR_VSP_VOUT_OSD_OFF,
  MINOR_VSP_VOUT_TIMING,

  MINOR_VSP_GWARP_OPEN = 0x70,
  MINOR_VSP_GWARP_START,
  MINOR_VSP_GWARP_CLOSE,
  MINOR_VSP_GWARP_FRAME_START,
  MINOR_VSP_GWARP_TABLE_CONFIG,

  MINOR_VSP_ENCODER_OPEN = 0x80,
  MINOR_VSP_ENCODER_START,
  MINOR_VSP_ENCODER_CLOSE,
  MINOR_VSP_ENCODER_FRAME_START,
  MINOR_VSP_ENCODER_FRAME_STOP,
  MINOR_VSP_ENCODER_RES_CFG,
  MINOR_VSP_ENCODER_FRAMERATE_CFG,
  MINOR_VSP_ENCODER_BITRATE_CFG,
  MINOR_VSP_ENCODER_QP_CFG,
  MINOR_VSP_ENCODER_GOP_CFG,

  MINOR_VSP_SELFCHECK_GWARP_FRAME_START = 0x90,
  MINOR_VSP_SELFCHECK_GWARP_FRAME_STOP,
  MINOR_VSP_SELFCHECK_ENCODER_FRAME_START,
  MINOR_VSP_SELFCHECK_ENCODER_FRAME_STOP,
};

//FMT_CLASS: 3bits
enum {
  FMT_CLASS_YUV = 0,
  FMT_CLASS_RAW,
  FMT_CLASS_RGB,
  FMT_CLASS_NUM
};

// max num <= 32
enum {
  FMT_YUV_NV12 = 0,
  FMT_YUV_NV21,
  FMT_YUV_Y,
  FMT_YUV_YUV420P,
  FMT_YUV_NUM
};

// max num <= 32
enum {
  FMT_RAW_RGGB = 0,
  FMT_RAW_RCCB,
  FMT_RAW_NUM
};

// max num <= 32
enum {
  FMT_RGB_RGB24 = 0,
  FMT_RGB_NUM
};

enum {
  TYPE_DATA_BIN = 0,
  TYPE_DATA_TXT,
  TYPE_DATA_NUM
};

enum {
  SYSTYPE_BITS_32 = 0,
  SYSTYPE_BITS_64,
  SYSTYPE_NUM
};

enum {
  CMDP_CH_ISP = 0,
  CMDP_CH_VSP,
  CMDP_CH_APU,
  CMDP_CH_RPU0,
  CMDP_CH_RPU1,
  CMDP_CH_CV,
  CMDP_CH_NUM
};

void setup_fbuf_parti(tSoneInit* pInit, void* pFbuf, uint32_t base, char* name);
void setup_slab_parti(tSoneInit* pInit, void* pSlab, uint32_t base);
void setup_cmdp_parti(tSoneInit* pInit, tSoneCmdp* pCmdp, uint32_t base);
void setup_init_parti(tSoneInit* pInit, uint32_t base, uint32_t base_high,
		char* strPlatform, char* strHostEnv, uint32_t traceMask, char* core_name, char short_name);
int init_parti_rodata_load(tSoneInit* pInit, char* fname);

void add_one_EP(tSoneCmdp* pCmdp, char* name, int port_id);

void build_cmd(tSoneCmdp* pSoneCmd, void* user_data,
		int cmd_type, int prio, char* src, char* dst,
		char core_code, int has_payload);

uint32_t gen_alloc_mem(void* pSection, uint32_t sect_base, int len);
struct gen_memory_list_32b *getGenPrevList(void* pSection, uint32_t sect_base, struct gen_memory_list_32b *pList);
struct gen_memory_list_32b *getGenNextList(void* pSection, uint32_t sect_base, struct gen_memory_list_32b *pList);

struct memory_list* alloc_mem_64b(void* pSection, int len);
int get_total_mem_alloc(void* pSection, int* ret_alloc_num);
#endif
