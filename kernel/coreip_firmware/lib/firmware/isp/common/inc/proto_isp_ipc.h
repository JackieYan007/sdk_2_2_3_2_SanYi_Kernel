#ifndef _PROTO_ISP_IPC_H
#define _PROTO_ISP_IPC_H
/*** offset for embed line after yuv view ***/
#define COMMON_SW_EMBED_SIZE   32
#define TUNE_2A_INFO_SIZE   64
#define YUV_ROI_STAT_SIZE   280
#define COMMON_SW_EMBED_DDR_OFFSET   0
#define TUNE_2A_INFO_DDR_OFFSET   0
#define YUV_ROI_STAT_DDR_OFFSET   (TUNE_2A_INFO_DDR_OFFSET + TUNE_2A_INFO_SIZE)

typedef enum _mipiSensorIndex_e{//mipi input sensor index, [7:2]mipi index(0/1/2) [1:0] port index(0/1/2/3)
  MipiS00 = 0,
  MipiS01,
  MipiS02,
  MipiS03,
  MipiS04,
  MipiS05,
  MipiS06,
  MipiS07,
  MipiS08,
  MipiS09,
  MipiS10,
  MipiS11,
  MipiS12,
  MipiS13,
  MipiS14,
  MipiS15
}mipiSensorIndex_e;

typedef enum _sensorIndex_e{//[7:2]isp core index(0/1/2)  [1:0] sensor index(0/1/2/3) per core
  S00 = 0,
  S01,
  S02,
  S03,
  S04,
  S05,
  S06,
  S07,
  S08,
  S09,
  S10,
  S11
}sensorIndex_e;

typedef enum _viewId_e{
  noView = 0,
  S00View0,
  S00View1,
  S00View2,
  S00ViewPdns,
  S01View0,
  S01View1,
  S01View2,
  S01ViewPdns,
  S02View0,
  S02View1,
  S02View2,
  S02ViewPdns,
  S03View0,
  S03View1,
  S03View2,
  S03ViewPdns,
  S04View0,
  S04View1,
  S04View2,
  S04ViewPdns,
  S05View0,
  S05View1,
  S05View2,
  S05ViewPdns,
  S06View0,
  S06View1,
  S06View2,
  S06ViewPdns,
  S07View0,
  S07View1,
  S07View2,
  S07ViewPdns,
  S08View0,
  S08View1,
  S08View2,
  S08ViewPdns,
  S09View0,
  S09View1,
  S09View2,
  S09ViewPdns,
  S10View0,
  S10View1,
  S10View2,
  S10ViewPdns,
  S11View0,
  S11View1,
  S11View2,
  S11ViewPdns,
  S00RawId,
  S01RawId,
  S02RawId,
  S03RawId,
  S04RawId,
  S05RawId,
  S06RawId,
  S07RawId,
  S08RawId,
  S09RawId,
  S10RawId,
  S11RawId,
}viewId_e;

typedef enum _srcSel_e{
  SRC_MIPI,
  SRC_HDMI,
  SRC_FILE2FILE_HDR,
  SRC_FILE2FILE_SINGLE
}srcSel_e;

typedef enum _sensorType_e{
  ST_OV10640RAW = 1,
  ST_AR0231RAW  = 2,
  ST_ASX340     = 3,
  ST_AR0144RAW  = 5,
  ST_OV2311RAW  = 6,
  ST_RESERVED   = 7,
  ST_IMX390RAW  = 8,
  ST_AR0233RAW  = 9,
  ST_YUV422     = 15,

  //sony
  ST_IMX424RAW  = 0x10,

  //onsemi
  ST_AR0820RAW  = 0x40,

  //ov
  ST_OV10652RAW = 0x80,
  ST_OX03C10RAW = 0x88,
  ST_OX08b40RAW = 0x89,
}sensorType_e;

typedef enum _dataType_e{
  DT_UYVY   = 0x1e,
  DT_YUYV   = 0x5e,
  DT_RAW10  = 0x2b,
  DT_RAW12  = 0x2c,
  DT_RAW14  = 0x2d,
  DT_RAW16  = 0x2e,
  DT_RAW20To10 = 0xab,
  DT_RAW20To12 = 0xac,
}dataType_e;

typedef enum _sensorRdWrMode_e{
  Addr16bit_Data16bit = 0,
  Addr16bit_Data8bit,
  Addr8bit_Data8bit
}sensorRdWrMode_e;

typedef enum _paramSize_e{
  PARAM_SIZE_08 = 8,
  PARAM_SIZE_16 = 16,
  PARAM_SIZE_32 = 32,
}paramSize_e;

typedef enum _status_e{
  STATUS_INVALID,     //0 as invalid
  STATUS_CMD_SUCESS,
  STATUS_USERDATA_ERROR,
  STATUS_PAYLOAD_ERROR,
  STATUS_CMD_FAILED,
  STATUS_DDR_OVERFLOW,
}status_e;

typedef enum _view0Fmt_e{
  View0_YUVSep_Fmt,
  View0_NV12_Fmt,
  View0_NV21_Fmt,
  View0_YUV422_Fmt,
  View0_RGB888_Fmt,
  View0_Dis
}view0Fmt_e;
typedef enum _view1Fmt_e{
  View1_YUVSep_Fmt,
  View1_NV12_Fmt,
  View1_NV21_Fmt,
  View1_YUV422_Fmt,
  View1_Raw_Fmt,
  View1_Dis
}view1Fmt_e;
typedef enum _view2Fmt_e{
  View2_Yonly_Fmt,
  View2_Dis = 5
}view2Fmt_e;

typedef enum _abnormalItem_e{
  ABN_INVALID,
  ABN_HW_QUEUE_IRQ,                    //DDR NOC queue abnormal interrupt for low queue(offline mode) or high queue(online mode)
  ABN_HW_RAW2DDR_WR_OVERFLOW,          //RAW to DDR write overflow
  ABN_HW_ISP2DDR_PDNS2DDR_WR_OVERFLOW, //ISP/PDNS to DDR write overflow

  ABN_SW_RAWRDYLST_OVERFLOW  = 0x80,
  ABN_SW_NO_SYNC_FRAME_DATA,
  ABN_SW_GRP_RAWRDYLST_OVERFLOW,
  ABN_SW_PDNS_P0RDYLST_OVERFLOW,
  ABN_SW_PDNS_P1RDYLST_OVERFLOW,
  ABN_SW_DVP_FRAMEBUF_OVERFLOW,
  ABN_SW_SRAM_ALLOC_OVERFLOW,
  ABN_SW_RAW_BADFRAME,
}abnormalItem_e;

typedef enum _iqItem_e{
  IQ_INVALID,
  IQ_AECAGC,

  IQ_SDE,
  IQ_BRIGHTNESS,
  IQ_CONTRAST,
  IQ_SATURATION,
  IQ_HUE,

  IQ_YDNS,
  IQ_UVDNS,
  IQ_SHARPEN,
  IQ_DYNAMIC_RANGE,

  IQ_AWB,
  IQ_GAMMA,
  IQ_YUVROI,
}iqItem_e;

typedef enum _ctrlItem_e{
  CTRL_INVALID,
  CTRL_VSYNC2FDONE_TIME,
  CTRL_CUR_TIME,
}ctrlItem_e;

typedef enum _pwlInFmt_e{
  //I: input, O: output, n:1/2/3 video path, b:bit, P:PWL, D:need Divide, L:LSB
  InOn_ByPass                     = 0,  //0  linear 12 bit N.A.            1                           Bypass
  //InOn_10bPD_12bD_nx12b,              //1  linear 10 bit PWL:10b -> 12 b 1                           PWL(L)10->12
  //D2O3_16bD12bV_12b,                  //2  16b DCG + 12b VS              16b DCG -> 2x12b(L, S), 12b VS    -> 12b VS 3 DCG(L,V)  LTM doesn't support.
  I2O3_12bPD12b_16bD12b_3x12b     = 3,  //3  12b DCG + 12b VS  12b DCG-> 16b -> 2x12b(L,S) 12b VS -> 12b VS  3 PWL(L)12->16 DCG(L,V)
  //InOn_ByPass,                        //4  3x12b DCG N.A.  3 Bypass
  InOn_10bPD_12bD_nx12b           = 5,  //5  3x10b DCG PWL: 3x10b -> 3x12b   3 PWL(L,S,V)10->12, can be 1/2/3x10b
  I1O2_16bD_2x12b                 = 6,  //6  16b DCG   16b DCG -> 2x12b(L, S)  2 DCG(L,V)  LTM doesn't support in A1000A
  I1O2_12bPD_16bD_2x12b           = 7,  //7  12b DCG 12b DCG-> 16b -> 2x12b(L,S) 2 PWL(L)12->16 DCG(L,V),  exposure format to select (L,S) or (L,V)
  //IxOx_ByPass,                        //8  2x12b DCG N.A.  2 Bypass
  I2O3_12bL11bD12bL11b_3x12b      = 9,  //9  2x11b raw12 DT, LSB 11b)  2x11b -> 3x12b (L, S, VS) 3 TRANS Inverse module doesn't support in A1000A
  //InOn_ByPass,                        //10  3x12b   N.A.  3 Bypass
  //InOn_ByPass,                        //11  YUV422-12bit  N.A.  N.A.  Bypass
  //I2O3_12bPD12b_16bD12b_3x12b,        //12  12b PWL+12b VS  12b PWL -> 16b -> 2x12b(L,S) 12b VS    -> 12b VS 3 PWL(L)12->16 16b -> 2x12b(L,S) 12b VS    -> 12b VS
  //I1O2_16bD_2x12b,                    //13  16b (2expo) 16b -> 2x12b  2 16b -> 2x12b 12b VS    -> 12b VS Seperate 2 exposures
  I1O3_16bPD_20bD_3x12b           = 14, //14  16b (3expo) 16b -> 20b -> 3x12b (L,S,VS)  3 PWL(L)16->20 20b -> 3x12b (L,S,VS) 33 Knee point PWL: Input 12~16 bit Output: 20 bit
  I1O3_14bPD_20bD_3x12b           = 15, //15  14b (3expo) 14b -> 20b -> 3x12b (L,S,VS)  3 PWL(L)14->20 20b -> 3x12b (L,S,VS)
  I1O3_12bPD_20bD_3x12b           = 16, //16  12b (3expo) 12b -> 20b -> 3x12b (L,S,VS)  3 PWL(L)12->20 20b -> 3x12b (L,S,VS)
                                        //17  log domain: 16 bit  16 log2linear -> 20bit  3 PWL(L)16->20 20b -> 3x12b (L,S,VS)
  I1O3_20bD_3x12b                 = 18, //18  20b (3expo) 20b -> 3x12b(L, S, VS)  3 20b -> 3x12b (L,S,VS)
  InOn_nx12bL10bP_nx12b           = 19, //19  3x12b pwl PWL:  LSB10b -> 12 b  3 PWL(L,S,V)10->12  input 12 bits, but only LSB 10 bits used
  I1O3_20bPD_24bD_3x12b           = 20, //20  20b (3expo) 20b -> 24b -> 3x12b (L,S,VS)  3 PWL(L)16->20 24b -> 3x12b (L,S,VS) 33 Knee point PWL: Input 12~20 bit Output: 24 bit
  I1O3_16bPD_24bD_3x12b           = 21, //21  16b (3expo) 16b -> 24b -> 3x12b (L,S,VS)  3 PWL(L)16->20 24b -> 3x12b (L,S,VS)
  I1O3_14bPD_24bD_3x12b           = 22, //22  14b (3expo) 14b -> 24b -> 3x12b (L,S,VS)  3 PWL(L)16->20 24b -> 3x12b (L,S,VS)
  I1O3_12bPD_24bD_3x12b           = 23, //23  12b (3expo) 12b -> 24b -> 3x12b (L,S,VS)  3 PWL(L)16->20 24b -> 3x12b (L,S,VS)
  I1O3_24bD_3x12b                 = 24, //24  24b (3expo) 24b -> 3x12b  3 24b -> 3x12b (L,S,VS)
}pwlInFmt_e;

typedef volatile struct _common_sw_embedded_data {
  uint32_t vsyncNum;
  uint16_t topCropBefore;
  uint16_t botCropBefore;
  uint16_t lefCropBefore;
  uint16_t rigCropBefore;
  uint16_t width;
  uint16_t height;
  uint16_t topCropAfter;
  uint16_t botCropAfter;
  uint16_t lefCropAfter;
  uint16_t rigCropAfter;
  uint8_t rsv[8];
}common_sw_embedded_data_t;

//to arm
typedef volatile struct _MINOR_BOOT_DONE{
  uint32_t    rsv[4];
}boot_done_t;
typedef volatile struct _MINOR_ABNORMAL{
  uint8_t     abnormalId;     //abnormalItem_e
  uint8_t     abnormalType;
  uint8_t     rsv0[2];
  uint32_t    rsv[3];
}abnormal_t;

typedef volatile struct _MINOR_SYNC_ISP_VIEW_FRAME_DONE{
  union{
    struct _viewCtrl{
      uint8_t     viewId[3];      //from arm viewId_e
      uint8_t     rsv;
    }viewCtrl;
    struct _viewReply{
      uint32_t    vsyncCnt  : 28;
      //enable dynamic resolution for view1 when reqDyncId larger than 0
      //reply viewBuf[1] will be view1 ddr
      uint32_t    reqDynId  : 4;  
    }viewReply;
  };
  uint32_t    viewBuf[3];     //DDR Addr
}new_frame_done_t;

typedef volatile struct _MINOR_SYNC_ISP_RAW_FRAME_DONE{
  uint32_t    vsyncCnt;
  uint32_t    rawBuf;     //DDR Addr
  uint32_t    rsv[2];
}new_rawframe_done_t;

//from/to arm
typedef volatile struct _MINOR_GET_VERSION{
  union{
    uint32_t  rsv0;           //from arm
    uint32_t  version;        //to arm, reply val
  };
  uint32_t    rsv[3];
}get_version_t;
typedef volatile struct _MINOR_GET_REG{
  uint32_t    rsv0;
  uint32_t    regAddr;
  uint32_t    size;           //from arm
  union{
    uint32_t  regVal;         //to arm, reply val
    uint32_t  regValDstBase;  //from arm
  };
}get_reg_t;
typedef volatile struct _MINOR_GET_I2C{
  uint32_t    regBase;
  uint8_t     devId;
  uint8_t     rsv1;
  uint8_t     regBitWid;
  uint8_t     datBitWid;
  uint16_t    regAddr;
  uint16_t    regOffset;      //0: read one register value to regVal, > 0 saved value to DDR
  union{
    uint32_t  regValDdrBase;  //from arm
    uint32_t  regVal;         //to arm, reply val
  };
}get_i2c_t;
typedef volatile struct _MINOR_ECHO_TEST{
  uint32_t    rsv[4];
}echo_test_t;

typedef volatile struct _MINOR_ISP_CAPTURE_RECONF{
  uint8_t     sensorIndex;    //sensorIndex_e
  union{
    struct _capModeBits{
      uint8_t   yuvCapEn  : 1;
      uint8_t   rawCapEn  : 1;
      uint8_t   capRsv    : 4;
      uint8_t   rawAsView2: 1;
      uint8_t   drvCapEn  : 1;  //driver capture mode
    }capModeBits;
    uint8_t     captureMode;
  };
  uint8_t     rsv0[2];
  union{
    uint32_t    payloadAddr;    //0 if no ipc_capture_reconf_t, DDR Addr for ipc_capture_reconf_t
    uint32_t    totalRawSize;        //reply to driver, capture raw size
  };
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_capture_reconf_t;

typedef volatile struct _IPC_CAP_RECONF{	
  uint32_t captureMode; //bit0 yuv,bit1 raw
  uint32_t viewFmt;
  uint32_t width;
  uint32_t height;
  uint32_t topCropBefore;
  uint32_t botCropBefore;
  uint32_t lefCropBefore;
  uint32_t rigCropBefore;
  uint32_t topCropAfter;
  uint32_t botCropAfter;
  uint32_t lefCropAfter;
  uint32_t rigCropAfter;
  uint32_t view1FbufBase;
  uint32_t view1FbufOffset;
  uint32_t rawCount;
  uint32_t rawBase[3];
  uint32_t rawOffset;
}ipc_cap_reconf_t;

typedef volatile struct _MINOR_ISP_GET_GLB_PARAM{
  uint16_t    paramId;        //offset from GlbCfg_Param.txt
  uint16_t    rsv0;
  uint8_t     paramSize;      //paramSize_e
  uint8_t     rsv1[3];
  union{
    uint32_t  rsv2;
    uint32_t  paramVal;       //to arm, reply val
  };
  uint32_t rsv3;
}isp_get_glb_param_t;
typedef volatile struct _MINOR_ISP_GET_ALG_PARAM{
  uint16_t    paramId;        //offset from SXX_AlgoParam.txt
  uint16_t    rsv0;
  uint8_t     paramSize;      //paramSize_e
  uint8_t     rsv1[3];
  union{
    uint32_t  rsv2;
    uint32_t  paramVal;       //to arm, reply val
  };
  uint32_t    rsv3;
}isp_get_alg_param_t;

typedef enum _txMsgMode_e{
  DropMsg_TimeOut   = 0,
  AttachMsg_TimeOut = 1
}txMsgMode_e;
typedef volatile struct _MINOR_ISP_START{
  uint32_t    reconfDDRBase;
  uint32_t    reconfDDRSize;
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  uint8_t     txMsgMode;        //0: drop msg when timeout, 1:will attach this msg to new one when timeout
  uint8_t     rsv2;
  uint16_t    txTimeOut;        //0.5us*txTimeOut, 0: use default timeout value (10*2000), 10ms
}isp_start_t;

typedef volatile struct _MINOR_ISP_IPC_CFG{
  uint32_t    rsv1[3];
  uint8_t     rsv2[2];
  uint16_t    txTimeOut;        //0.5us*txTimeOut, 0: use default timeout value (10*2000), 10ms
}isp_ipc_cfg_t;
//from/to arm,  set payload bit
typedef volatile struct _MINOR_ISP_BOOTLD_RECONF{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     rsv0[3];
  uint32_t    payloadAddr;    //DDR Addr for ipc_reconf_t
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_ld_reconf_t;
typedef volatile struct _MINOR_ISP_I2C_BUS_CTRL{
  uint8_t     i2cBusRelease;     //0:isp control i2c bus 1: isp release i2c bus 
  uint8_t     rsv0;
  uint16_t    sensorBits;    //
  uint32_t    i2cBusBase;    //i2c bus reg base
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  uint32_t    rsv2;
}isp_i2c_bus_ctrl_t;
typedef volatile struct _MINOR_ISP_SET_PWLINFO{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     rsv0[3];
  uint32_t    payloadAddr;    //DDR Addr for PWLInfo_t bin file
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_set_pwl_info_t;
typedef volatile struct _MINOR_ISP_BOOTLD_ALGO_BIN{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     rsv0[3];
  uint32_t    payloadAddr;    //DDR Addr for algo bin file
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_ld_algo_bin_t;
typedef volatile struct _MINOR_ISP_BOOTLD_IQ_BIN{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     rsv0[3];
  uint32_t    payloadAddr;    //DDR Addr for iq bin file
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_ld_iq_bin_t;
typedef volatile struct _MINOR_ISP_SET_VIEWINFO{
  uint8_t     viewid;         //viewId_e
  uint8_t     rsv0[3];
  uint32_t    payloadAddr;    //DDR Addr for view_cfg_t
  union{
    uint32_t  rsv1;
    uint32_t  status;         //to arm, reply status
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_set_viewinfo_t;
typedef volatile struct _MINOR_ISP_SET_IQINFO{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     iqItem;
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     rsv2;
  uint32_t    payloadAddr;    //DDR Addr for ipc_iqinfo_t
  uint32_t    iqVal;          //use iqVal when payloadAddr=0
  union{
    uint32_t    rsv3;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_set_iqinfo_t;
typedef volatile struct _MINOR_ISP_GET_IQINFO{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     iqItem;
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     rsv2;
  uint32_t    iqVal;          //reply
  uint32_t    iqValMin;
  uint32_t    iqValMax;
}isp_get_iqinfo_t;
typedef volatile struct _MINOR_ISP_SET_CTRLINFO{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     ctrlItem;
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     rsv2;
  uint32_t    payloadAddr;    //DDR Addr for ipc_ctrlinfo_t
  uint32_t    ctrlVal;        //use ctrlVal when payloadAddr=0
  union{
    uint32_t    rsv3;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}isp_set_ctrlinfo_t;
typedef volatile struct _MINOR_ISP_GET_CTRLINFO{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     ctrlItem;
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     rsv2;
  uint32_t    ctrlVal;        //reply
  uint32_t    rsv3[2];
}isp_get_ctrlinfo_t;

//from arm
typedef volatile struct _MINOR_SET_REG{
  uint8_t     payloadMode;    //0-use regAddr/1-use payloadAddr
  uint8_t     rsv0[3];
  union{
    uint32_t    payloadAddr;
    uint32_t    regAddr;
  };
  union{
    uint32_t    status;       //to arm, reply status for payload mode
    uint32_t    regVal;
  };
  union{
    uint32_t    rsv2;
    uint32_t    payloadSize;  //0:auto, xxx
  };
}set_reg_t;
typedef volatile struct _MINOR_SET_I2C{
  uint32_t    regBase;
  uint8_t     devId;
  uint8_t     payloadMode;    //0-use regAddr/1-use payloadAddr
  uint8_t     regBitWid;
  uint8_t     datBitWid;
  union{
    uint32_t    payloadAddr;
    uint32_t    regAddr;
  };
  union{
    uint32_t    status;       //to arm, reply status for payload mode
    uint32_t    regVal;
    uint32_t    payloadSize;  //0:auto end when got 0x000000000, xxx:read the real size
  };
}set_i2c_t;
typedef volatile struct _MINOR_ISP_SET_GLB_PARAM{
  uint16_t    paramId;        //offset from GlbCfg_Param.txt
  uint16_t    rsv0;
  uint8_t     paramSize;      //paramSize_e
  uint8_t     rsv1[3];
  uint32_t    paramVal;
  uint32_t    rsv3;
}isp_set_glb_param_t;
typedef volatile struct _MINOR_ISP_SET_ALG_PARAM{
  uint16_t    paramId;        //offset from SXX_AlgoParam.txt
  uint16_t    rsv0;
  uint8_t     paramSize;      //paramSize_e
  uint8_t     rsv1[3];
  uint32_t    paramVal;
  uint32_t    rsv3;
}isp_set_alg_param_t;

typedef enum _dynView_e{
  INVALID_DYNVIEW     = 0,
  //DYNVIEW_VIEW0       = 1,
  DYNVIEW_VIEW1       = 2,//use this
  //DYNVIEW_VIEW2       = 3,
}dynView_e;
typedef volatile struct _MINOR_ISP_NEW_VIEW_FRAME_BUF{
  uint8_t     viewId[3];      //from arm viewId_e
  //enable dynamic resolution for view1 when reqDyncId larger than 0
  //viewBuf[1] is the pointer for view_cfg_t
  //view1DyncFbuf in view_cfg_t will be used as new view1 ddr
  uint8_t     dynViewEn : 4;  // dynView_e
  uint8_t     reqDynId  : 4;  // enabled when larger than 0, 
  uint32_t    viewbuf[3];
}isp_new_frame_buf_t;
typedef volatile struct _MINOR_ISP_CAM_OPEN{
  uint8_t     viewId[3];      //viewId_e
  uint8_t     rsv0;
  uint32_t    rsv[3];
}isp_cam_open_t;
typedef volatile struct _MINOR_ISP_CAM_CLOSE{
  uint8_t     viewId[3];      //viewId_e
  uint8_t     rsv0;
  uint32_t    rsv[3];
}isp_cam_close_t;
typedef volatile struct _MINOR_ISP_FILE2FILE_START{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     rsv0[3];
  uint32_t    rawBase;
  uint32_t    runNum;
  uint32_t    rsv1;
}isp_file2file_start_t;

//safty relation
typedef enum _safetyitem_e{
  INVALID_SAFETYITEM  = 0,
  BIST_DATAMEM        = 1,
  BIST_DDR,
  BIST_REGS,
  BIST_DMA,

  ABN_ECC_PTY_CFG     = 0x10,
  ABN_ECC_PTY_CLR_CFG,
  ABN_ECC_PTY_INJECT_ERR_CFG,  //inject parity or ECC error, not for ISP to SAFETY core interrupt
  ABN_WATCH_DOG_CFG,
  ABN_WATCH_DOG_INJECT_ERR_CFG,
  ABN_TEST_IMAGE_INJECT_ERR_CFG,

  DBG_TEST_PATTERN_CFG = 0x20
}safetyitem_e;
typedef volatile struct _BIST_DATAMEM{
  uint32_t    base;
  uint32_t    size;
  uint32_t    value;
}bist_datamem_t;
typedef volatile struct _BIST_DDR{
  uint32_t    base;
  uint32_t    size;
  uint32_t    value;
}bist_ddr_t;
typedef volatile struct _BIST_REGS{
  uint32_t    i2cRegsBase;
  uint32_t    ipcRegsBase;
  uint32_t    mipiPhyRegsBase;//not used for A1000A0
}bist_regs_t;
typedef volatile struct _BIST_DMA{
  uint32_t    src;
  uint32_t    dst;
  uint32_t    size;
}bist_dma_t;

typedef volatile struct _ABN_ECC_PTY_CFG{
  union{
    //0x5203002c
    struct _cfg2c_bits{
      uint32_t RISCV_ECC_S_INTR_EN      : 1;
      uint32_t RISCV_ECC_D_INTR_EN      : 1;
      uint32_t PRAM_ECC_S_INTR_EN       : 1;
      uint32_t PRAM_ECC_D_INTR_EN       : 1;
      uint32_t OFNR_ECC_S_INTR_EN       : 1;
      uint32_t OFNR_ECC_D_INTR_EN       : 1;
      uint32_t SCALE_ECC_S_INTR_EN      : 1;
      uint32_t SCALE_ECC_D_INTR_EN      : 1;
      uint32_t LTM_ECC_S_INTR_EN        : 1;
      uint32_t LTM_ECC_D_INTR_EN        : 1;
      uint32_t INTR_ECC_S_INTR_EN       : 1;
      uint32_t INTR_ECC_D_INTR_EN       : 1;
      uint32_t ICORE2_ECC_S_INTR_EN     : 1;
      uint32_t ICORE2_ECC_D_INTR_EN     : 1;
      uint32_t ICORE1_ECC_S_INTR_EN     : 1;
      uint32_t ICORE1_ECC_D_INTR_EN     : 1;
      uint32_t ICORE0_ECC_S_INTR_EN     : 1;
      uint32_t ICORE0_ECC_D_INTR_EN     : 1;
      uint32_t NSTS0_ECC_S_INTR_EN      : 1;
      uint32_t NSTS0_ECC_D_INTR_EN      : 1;
      uint32_t NSTS1_ECC_S_INTR_EN      : 1;
      uint32_t NSTS1_ECC_D_INTR_EN      : 1;
      uint32_t NSTS2_ECC_S_INTR_EN      : 1;
      uint32_t NSTS2_ECC_D_INTR_EN      : 1;
      uint32_t rsv2c_bit24_bit28        : 5;
      uint32_t R_NSTS0_PTY_EN           : 1;
      uint32_t R_NSTS1_PTY_EN           : 1;
      uint32_t R_NSTS2_PTY_EN           : 1;
    }cfg2c_bits;
    uint32_t cfg2c;
  };
  union{
    //0x52030030
    struct _cfg30_bits{
      uint32_t R_ICORE0_ECC_EN        : 1;
      uint32_t R_ICORE1_ECC_EN        : 1;
      uint32_t R_ICORE2_ECC_EN        : 1;
      uint32_t R_OFNR_ECC_EN          : 1;
      uint32_t R_LTM_ECC_EN           : 1;
      uint32_t R_INTR_ECC_EN          : 1;
      uint32_t R_PRAM_ECC_EN          : 1;
      uint32_t R_RISCV_ECC_EN         : 1;
      uint32_t rsv30_bit8_bit15       : 8;
      uint32_t R_ICORE0_PTY_EN        : 1;
      uint32_t R_ICORE1_PTY_EN        : 1;
      uint32_t R_ICORE2_PTY_EN        : 1;
      uint32_t R_OFNR_PTY_EN          : 1;
      uint32_t R_EMBED0_PTY_EN        : 1;
      uint32_t R_EMBED1_PTY_EN        : 1;
      uint32_t R_EMBED2_PTY_EN        : 1;
      uint32_t R_RISCV_PTY_EN         : 1;
      uint32_t AHBS_PTY_ENC_EN        : 1;
      uint32_t AHBM_PTY_ENC_EN        : 1;
      uint32_t AXI_PTY_ENC_EN         : 1;
      uint32_t AHBS_PTY_DEC_EN        : 1;
      uint32_t AHBM_PTY_DEC_EN        : 1;
      uint32_t AXI_PTY_DEC_EN         : 1;
      uint32_t LTM_PARITY_CHK_SEND_EN : 1;
      uint32_t LTM_PARITY_CHK_REC_EN  : 1;
    }cfg30_bits;
    uint32_t cfg30;
  };
  uint32_t    rsv0;
}abn_ecc_pty_cfg_t;
typedef volatile struct _ABN_ECC_PTY_CLR_CFG{
  union{
    //0x52030034
    struct _cfg34_ABNbits{
      uint32_t R_ICORE0_ECC_CLEAN         : 1;
      uint32_t R_ICORE1_ECC_CLEAN         : 1;
      uint32_t R_ICORE2_ECC_CLEAN         : 1;
      uint32_t R_OFNR_ECC_CLEAN           : 1;
      uint32_t R_LTM_ECC_CLEAN            : 1;
      uint32_t R_INTR_ECC_CLEAN           : 1;
      uint32_t R_PRAM_ECC_CLEAN           : 1;
      uint32_t R_RISCV_ECC_CLEAN          : 1;
      uint32_t RISCV_TIMEOUT_INTR_CLEAN   : 1;
      uint32_t cfg34_rsv_bit9_bit15       : 7;
      uint32_t R_ICORE0_PTY_CLEAN         : 1;
      uint32_t R_ICORE1_PTY_CLEAN         : 1;
      uint32_t R_ICORE2_PTY_CLEAN         : 1;
      uint32_t R_OFNR_PTY_CLEAN           : 1;
      uint32_t R_EMBED0_PTY_CLEAN         : 1;
      uint32_t R_EMBED1_PTY_CLEAN         : 1;
      uint32_t R_EMBED2_PTY_CLEAN         : 1;
      uint32_t R_RISCV_PTY_CLEAN          : 1;
      uint32_t AHBS_PTY_INTR_CLEAN        : 1;
      uint32_t AHBM_PTY_INTR_CLEAN        : 1;
      uint32_t AXI_PTY_INTR_CLEAN         : 1;
      uint32_t LTM_PTY_CLEAN              : 1;
      uint32_t ICORE0_TEST_CLEAN          : 1;
      uint32_t ICORE1_TEST_CLEAN          : 1;
      uint32_t ICORE2_TEST_CLEAN          : 1;
      uint32_t OFNR_TEST_CLEAN            : 1;
    }cfg34_ABNbits;
    uint32_t cfg34;
  };
  uint32_t    rsv0[2];
}abn_ecc_pty_clr_cfg_t;
typedef volatile struct _ABN_ECC_PTY_INJECT_ERR_CFG{
  union{
    //0x52030034
    struct _cfg34_Inject_bits{
      uint32_t cfg34_rsv_bit0_bit12     : 13;
      uint32_t AHBS_PTY_ERR_INJECT      : 1;
      uint32_t AHBM_PTY_ERR_INJECT      : 1;
      uint32_t AXI_PTY_ERR_INJECT       : 1;
      uint32_t cfg34_rsv_bit16_bit31    : 16;
    }cfg34_INJbits;
    uint32_t cfg34;
  };
  uint32_t    rsv0[2];
}abn_ecc_pty_inject_err_cfg_t;
typedef volatile struct _ABN_WATCH_DOG_CFG{
  uint8_t     timeOutEn;       //0/1
  uint8_t     rsv8[3];
  uint32_t    timeOutMaxCycles;//based on 400MHz
  uint32_t    rsv0;
}abn_watch_dog_cfg_t;
typedef volatile struct _ABN_WATCH_DOG_INJECT_ERR_CFG{
  uint32_t    rsv0[3];
}abn_watch_dog_inject_err_cfg_t;

typedef enum _testimagemode_e{
  TestImageMode_ISPPipe = 1,
  TestImageMode_PDNS    = 2,
}testimagemode_e;
typedef volatile struct _ABN_TEST_IMAGE_INJECT_ERR_CFG{
  uint8_t     testImageInjectErrEn; //0: disables/1: enabled
  uint8_t     testImageMode;        //1: isp pipe/2: pdns
  uint8_t     rsv8[2];
  uint32_t    injectErrCRCData;
  uint32_t    rsv0;
}abn_test_image_inject_err_cfg_t;

typedef volatile struct _DGB_TEST_PATTERN_CFG{
  uint8_t     tpgEn;
  uint8_t     frameRate;
  uint8_t     rsv0[2];
  uint16_t    width;
  uint16_t    height;
  uint32_t    rsv1;
}dbg_test_pattern_cfg_t;
typedef volatile struct _MINOR_ISP_SAFTYINFO{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     safetyItem;
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     rsv2;
  union{
    uint32_t    safetyCfg[3];
    bist_datamem_t                  bistDatamem;
    bist_ddr_t                      bistDdr;
    bist_regs_t                     bistRegs;
    bist_dma_t                      bistDma;
    abn_ecc_pty_cfg_t               abnEccPtyCfg;
    abn_ecc_pty_clr_cfg_t           abnEccPtyClrCfg;
    abn_ecc_pty_inject_err_cfg_t    abnEccPtyInjectErrCfg;
    abn_watch_dog_cfg_t             abnWatchDogCfg;
    abn_watch_dog_inject_err_cfg_t  abnWatchDogInjectErrCfg;
    abn_test_image_inject_err_cfg_t abnTestImageInjectErrCfg;
    dbg_test_pattern_cfg_t          dbgTestPatternCfg;
  };
}isp_safetyinfo_t;

typedef enum _dspsync_e{
  DSP_UNSYNC      = 0,
  DSP_SYNC_STEP1  = 1,
  DSP_SYNC_STEP2  = 2,
  DSP_SYNC_STEP3  = 3,
  DSP_SYNC_STEP4  = 4,
  DSP_SYNC_STEP5  = 5,
  DSP_SYNC_DONE   = 6,
  DSP_SYNC_LUT_CREATE  = 0x10,
  DSP_SYNC_PWL_RUN     = 0x11,
  DSP_SYNC_LUT_RELEASE = 0x12,

  DSP_SYNC_PWL_F2F_RUN = 0x20,
  DSP_SYNC_Comb_F2F_RUN = 0x21,
}dspsync_e;

typedef volatile struct _MINOR_ISP_DSP_SYNC{
  uint8_t     dspCoreId;      //dsp core 0/1/2/3
  uint8_t     dspSyncStatus;  //to arm, reply dspsync_e
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     logLevel;
  uint32_t    logDdrBase;
  uint32_t    logDdrSize;
  uint32_t    dspSyncDdrBase;
}isp_dsp_sync_t;

typedef enum _dspipcitem_e{
  INVALID_DSPIPCITEM  = 0,
  DSPIPC_LUT_CREATE   = 1,
  DSPIPC_PWL          = 2,
  DSPIPC_LUT_RELEASE  = 3,
  DSPIPC_Comb          = 4,
}dspipcitem_e;

typedef volatile struct _PWLInfo{//used for DSPIPC_LUT_CREATE and MINOR_ISP_SET_PWLINFO
  uint32_t rawBitWidth;
  uint32_t maxKP;
	uint32_t nPWLXList[33];
  uint32_t nPWLYList[33];
}PWLInfo_t;
typedef volatile struct _MINOR_ISP_DSP_IPC{
  uint8_t     sensorIndex;    //sensorIndex_e
  uint8_t     dspIpcItem;     //dspipcitem_e
  union{
    uint8_t   rsv1;
    uint8_t   status;         //to arm, reply status
  };
  uint8_t     rsv2;
  uint32_t    infoBase;
  uint32_t    dspIpcCfg;
  union{
    uint32_t    rsv3;
    uint32_t    infoSize;     //0:auto, xxx
  };
}isp_dsp_ipc_t;

typedef volatile struct _ispusrdata{
  union {
    //to arm
    abnormal_t          abnormal;
    new_frame_done_t    newFrameDone;
    new_rawframe_done_t newRawFrameDone;

    //from/to arm
    get_version_t       getVersion;
    get_reg_t           getReg;
    get_i2c_t           getI2c;
    echo_test_t         echoTest;
    isp_capture_reconf_t ispCapReconf;
    isp_get_glb_param_t getGlbParam;
    isp_get_alg_param_t getAlgParam;
    isp_start_t         ispStart;
    isp_i2c_bus_ctrl_t  i2cBusCtrl;
    //from/to arm,  set payload bit
    isp_set_pwl_info_t  ispPwlInfo;
    isp_ld_reconf_t     ispLdReconf;
    isp_ld_algo_bin_t   ispLdAlgoBin;
    isp_ld_iq_bin_t     ispLdIqBin;
    isp_set_viewinfo_t  ispSetViewinfo;
    isp_set_iqinfo_t    ispSetIqinfo;

    //from arm
    set_reg_t           setReg;
    set_i2c_t           setI2c;
    isp_set_glb_param_t setGlbParam;
    isp_set_alg_param_t setAlgParam;
    isp_new_frame_buf_t newFrameBuf;
    isp_cam_open_t      camOpen;
    isp_cam_close_t     camClose;
    isp_file2file_start_t f2fStart;
    isp_ipc_cfg_t       ispIpcCfg;

    isp_dsp_sync_t      dspSync;
    isp_dsp_ipc_t       dspIpc;

    isp_safetyinfo_t    safetyInfo;

    uint32_t            data[4];
  };
} ispusrdata;


#define IspBufDDRBase 0xA2000000
#define IspBufDDRSize 0x12000000
//payload info
typedef volatile struct _raw_cfg_t {
  uint8_t   srcSel;             //srcSel_e
  uint8_t   dataType;           //sensor input data type, dataType_e
  union{
    uint8_t   hdrMode;            //[1:0] 0-HDR WxN/1-HDR Stagger, [7] 1-force offlineMode [6] VS Mode [5] DSP IPC
    struct _hdrModeBits{
      uint8_t hdrFmt            : 1;  //0-HDR WxN/1-HDR Stagger
      uint8_t rsv               : 3;
      uint8_t hdriLe2DdrDis     : 1;  //hdri le to ddr disabled
      uint8_t dspIpcEn          : 1;  //enable dsp ipc for pwl or other features
      uint8_t vsMode            : 1;  //do not use
      uint8_t forceOfflineMode  : 1;  //1-force Offline mode
    }hdrModeBits;
  };
  uint8_t   expNum;             //1-single/2-2exp/3-3exp
  uint8_t   ispPwlInFormat;     //pwlInFmt_e
  uint8_t   ispRawOutFormat;    //debug use
  uint8_t   dvpDataType;        //sensor data to ddr data type, 0: same as dataType/dataType_e
  uint8_t   vinDataType;        //ddr to isp pipe data type,    0: same as dataType/dataType_e
  uint16_t  width;              //mipi/hdmi input width
  uint16_t  height;             //mipi/hdmi input height
  uint16_t  dvpDummyVal;        //[15:x]=dummydata [x:0]=0, depend on sensor dummy data config
  //offline use dvpDummyLines for special sensors
  //[15]    fixed dummy lines mode
  //[13:0]  rawDummyMinLine if bit15=0
  //[13:0]  fixed rawDummyLexpLinesNum if bit15=1
  uint16_t  dvpDummyLines;
}raw_cfg_t;

typedef enum _scalerRemap_e{
  NO_SCALER_REMAP         = 0,
  SCALER_REMAP_TO_CORE0   = 1,
  SCALER_REMAP_TO_CORE1   = 2,
  SCALER_REMAP_TO_CORE2   = 3,
}scalerRemap_e;
typedef volatile struct _view_cfg_t {
  //view0: view0Fmt_e
  //view1: view1Fmt_e
  //view2: view2Fmt_e
  uint8_t   viewFmt;            //view output format
  uint8_t   scalerRemap;        //scalerRemap_e
  union{
    struct _embedCfgBits{
      uint8_t comSwEmbedEn  : 1;
      uint8_t embedBitsRsv  : 7;
    }embedCfgBits;
    uint8_t embedCfg;
  };
  uint8_t   rsv0;

  //crop before scaler [topCropBefore, botCropBefore) [lefCropBefore, rigCropBefore), disabled if all 0
  uint16_t  topCropBefore;      //crop pixel top position     (include current position)
  uint16_t  botCropBefore;      //crop pixel bottom position  (exclude current position)
  uint16_t  lefCropBefore;      //crop pixel left position    (include current position)
  uint16_t  rigCropBefore;      //crop pixel right position   (exclude current position)

  //scaler output size, sclaer disabled if following size is same as raw size
  uint16_t  width;              //view output width
  uint16_t  height;             //view output height

  //crop after scaler [topCropAfter, botCropAfter) [lefCropAfter, rigCropAfter), disabled if all 0
  uint16_t  topCropAfter;       //crop pixel top position     (include current position)
  uint16_t  botCropAfter;       //crop pixel bottom position  (exclude current position)
  uint16_t  lefCropAfter;       //crop pixel left position    (include current position)
  uint16_t  rigCropAfter;       //crop pixel right position   (exclude current position)
  uint32_t  viewDyncFbuf;        //used for viewBuf[1] when reqDyncId larger than 0
}view_cfg_t;
typedef volatile struct _pdns_cfg_t {
  uint8_t pdnsMode;             //0-disable/1-enable pdns
  uint8_t pdnsViewSel;          //0/1 select which view as pdns
  uint8_t rsv[2];
}pdns_cfg_t;

typedef volatile struct _ipc_reconf_t{
  uint32_t    i2cRegBase;       //i2c IP register base
  uint8_t     mipiSensorIndex;  //mipiSensorIndex_e
  uint8_t     sensorIndex;      //sensorIndex_e
  uint8_t     sensorDevID;      //sensor i2c addr
  uint8_t     sensorRdWrMode;   //sensorRdWrMode_e
  uint16_t    sensorType;       //sensorType_e
  //raw data crop for ltm, crop embed lines instead of video data.
  uint8_t     ltmVinTopCrop;
  uint8_t     ltmVinBotCrop;
  raw_cfg_t   rawinfo;
  view_cfg_t  viewinfo[3];
  pdns_cfg_t  pdnsinfo;
  uint16_t    hblank;             //0-use default/xxxx-adjust it to avoid ddr overflow
  uint8_t     linesAddDetBad;     //add lines num for bad frame detect
  uint8_t     rsv;
  //raw data crop before isp pipe and after ltm
  uint16_t    ispInTopCrop;//crop top line num
  uint16_t    ispInBotCrop;//crop bottom line num
  uint16_t    ispInLefCrop;//crop left column num
  uint16_t    ispInRigCrop;//crop right column num
}ipc_reconf_t;

typedef volatile struct _aecaut_t{
  uint8_t   targetY[3];   //auto use
  uint8_t   rsv1[13];
}aecaut_t;
typedef volatile struct _aecman_t{
  uint16_t  aecManualExp[3];
  uint16_t  rsv2;
  uint16_t  aecManualGain[3];
  uint16_t  rsv3;
}aecman_t;
typedef volatile struct _iq_aecagc_t{
  uint8_t   bManualAECEnable;
  uint8_t   rsv0[3];

  union{
    aecaut_t aut_t;
    aecman_t man_t;
  };

  uint32_t  rsv[3];
}iq_aecagc_t;
typedef volatile struct _iq_sde_t{
  uint8_t   sdeEnable;
  uint8_t   contrastBrightnessAdjustEn;
  uint8_t   statuationAdjustEn;
  uint8_t   hueAdjustEn;
  uint32_t  brightness;
  uint32_t  contrast;
  uint32_t  saturation;
  uint32_t  hue;
  uint32_t  rsv[3];
}iq_sde_t;

typedef volatile struct _iq_rgbgamma_t
{
  uint16_t nRGBCurve[33];
  uint16_t rsv;
}iq_rgbgamma_t;

typedef volatile struct _awbaut_t{
  uint16_t  rsv1[9];

  uint16_t  nAWBRegionLLBrTh[4];
  uint16_t  rsv2;
}awbaut_t;
typedef volatile struct _awbman_t{
  uint16_t  manualAWBGain[3][3];  //[expNum 0:L/1:M/2:S][color 0:B/1:G/2:R]

  uint16_t  nAWBRegionLLBrTh[4];
  uint16_t  rsv2;
}awbman_t;
typedef volatile struct _iq_awb_t{
  uint8_t   bManualWB;
  uint8_t   rsv0[3];
  union{
    awbaut_t aut_t;
    awbman_t man_t;
  };
}iq_awb_t;

typedef volatile struct _iq_yuv_roi_ctrl_t // 36 byte, 0x811444 -> 0xa02144
{
  uint8_t nROIIndex;
  uint8_t nSampleRate;
  uint8_t nROIPipeSel;
  uint8_t nAttachViewSel;     //same attach view for one sensor
  uint16_t nROITop;
  uint16_t nROILeft;
  uint16_t nROIWidth;
  uint16_t nROIHeight;
  uint32_t rsv1[5];
}iq_yuv_roi_ctrl_t;

typedef volatile struct _iq_yuv_roi_mean_stat_t
{
  uint16_t  nStatMeanY;
  uint16_t  nStatMeanU;
  uint16_t  nStatMeanV;
}iq_yuv_roi_mean_stat_t;
typedef volatile struct _iq_yuv_roi_hist_stat_t
{
  uint32_t  nYHist[16];
}iq_yuv_roi_hist_stat_t;
typedef volatile struct _iq_yuv_roi_stat_t // 36 byte, 0x811444 -> 0xa02144
{
  iq_yuv_roi_mean_stat_t  yuvroi_mean_stat[4];
  iq_yuv_roi_hist_stat_t  yuvroi_hist_stat[4];
}iq_yuv_roi_stat_t;

typedef volatile struct _ipc_iqinfo_t{
  union{
    iq_aecagc_t           aecagcInfo;
    iq_sde_t              sdeInfo;
    iq_awb_t              awbInfo;
    iq_yuv_roi_ctrl_t     yuvroiCtrl;
  };
}ipc_iqinfo_t;

typedef volatile struct _ipc_ctrlinfo_t{
  union{
    uint32_t    rsv[8];
  };
}ipc_ctrlinfo_t;
#endif
