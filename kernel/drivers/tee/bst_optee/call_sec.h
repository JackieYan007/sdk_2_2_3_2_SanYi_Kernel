/*
* This file contains proprietary information that is the sole intellectual 
* property of Black Sesame Technologies, Inc. and its affiliates. 
* No portions of this material may be reproduced in any 
* form without the written permission of: 
* Black Sesame Technologies, Inc. and its affiliates 
* 2255 Martin Ave. Suite D
* Santa Clara, CA 95050 
* Copyright @2016: all right reserved. 
*/

#define TA_MY_TEST_UUID { 0x9269fadd, 0x99d5, 0x4afb, \
	{0xa1, 0xdc, 0xee, 0x3e, 0x9c, 0x61, 0xb0, 0x4c}}

typedef enum{
    eEVT_START,
    eEVT_APP0 = 1<<0,
    eEVT_APP1 = 1<<1,        
    eEVT_APP2 = 1<<2,        
    eEVT_APP3 = 1<<3,
    eEVT_APP4 = 1<<4,
    eEVT_APP5 = 1<<5,
    eEVT_APP6 = 1<<6,
    eEVT_APP7 = 1<<7,
    eEVT_SAF0 = 1<<8,
    eEVT_SAF1 = 1<<8,
    eEVT_DSP0 = 1<<10,
    eEVT_DSP1 = 1<<11,        
    eEVT_DSP2 = 1<<12,    
    eEVT_DSP3 = 1<<13,
    eEVT_ISP0 = 1<<14,
    eEVT_VSP0 = 1<<15,
    eEVT_NET = 1<<16,
    eEVT_SEC = 1<<17,
    eEVT_END,
}E_IPC_EVT_INFO;


typedef enum{
    eCRYPTO_START,
    eDEVID_GET,
    eHMAC,
    eHASH,
    eAES,
    eSM3,
    eSM4,
    eRSA,
    eECC,
    eTRNG,
    eSIGN,
    eSIGN_CHECK,
    eCRYPTO_END,
    eOTP_START,//use otp save secure info
    eOTP_END,
}E_REQUEST_TYPE ;

typedef struct IN_BUFF {
    u32 buff_paddr;
    u32 buff_size;
}T_IN_BUFF;

typedef struct REQ_OBJ {
    E_REQUEST_TYPE            req_type;
    u32                       idx;
    u32                       para1;
    u32                       para2;
    u32                       para3;
    u32                       para4;
    u32                       para5;
    u32                       para6;
}T_REQ_OBJ;

typedef struct ACK_OBJ{
    E_REQUEST_TYPE            req_type;
    u32                       idx;
    u32                       ack[64];    
}T_ACK_OBJ;

typedef struct RING_OBJ{
    u32                       is_secure_idle;//secure idle,it can handle other requeset(secure write 1,other cpu write 0, only read 1 can request).)
    u32                       req_stage;//secure set stage and result. 0 = unhandle(other), 1 = handleing(secure), 2 = handle done(secure) ,0xff= handle err(err)
    u32                       cpu_idx;//check cpu idx(set by other,check by secure)
    T_REQ_OBJ            req;
    T_ACK_OBJ            ack;    
}T_RING_OBJ;


typedef struct SECURE_REQ{   
    T_RING_OBJ safety;
    T_RING_OBJ app;
}T_SECURE_REQ;

#define SECURE_REQ_SIZE (sizeof(T_SECURE_REQ))
#define SECURE_REQ_OBJ_SIZE (sizeof(T_RING_OBJ))

int optee_get_chipid(size_t len, void *output);


