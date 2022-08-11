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

#include <linux/printk.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/tee_drv.h>
#include <linux/types.h>
#include <linux/delay.h>
#include "../optee/optee_private.h"
#include "tee_client_api.h"
#include "call_sec.h"

#define CALL_MAX_RETRY 100
#define CALL_SEC_ERR -1

extern struct optee *optee_svc;

static int g_TaskInitFlag = -1;    /* Flag if the task done initialize operation */
TEEC_UUID svc_id = TA_MY_TEST_UUID;
TEEC_Context g_TaskContext;

int g_optee_used = 0;
int optee_get_chipid(size_t len, void *output)
{
    TEEC_Session   l_session;    /* Define the session of TA&CA */
    TEEC_Operation l_operation;  /* Define the operation for communicating between TA&CA */
    int l_RetVal = TEEC_SUCCESS;       /* Define the return value of function */
    uint32_t origin;
    TEEC_Result result;

    if (g_optee_used) {
        u32 send_cnt = CALL_MAX_RETRY;
        while(send_cnt--) {
            if (!g_optee_used) {
                break;
            }
            msleep(1);
        }
    }
    if (g_optee_used) {
        pr_err("call secure fail");
        return CALL_SEC_ERR;
    }
    g_optee_used = 1;

    /**1) Initialize this task */
    if(-1 == g_TaskInitFlag) {
        result = TEEC_InitializeContext(NULL, &g_TaskContext);
        if(result != TEEC_SUCCESS) {
            pr_err("InitializeContext failed, ReturnCode=0x%x\n", result);
            l_RetVal= CALL_SEC_ERR;
            goto cleanup_1;
        }
        g_TaskInitFlag = 0;
        // pr_info("InitializeContext success\n");
    }
    
    /**2) Open session */
    result = TEEC_OpenSession_ChipId(&g_TaskContext, &l_session, &svc_id, 
                                TEEC_LOGIN_PUBLIC, NULL, &origin);
    if(result != TEEC_SUCCESS) {
        pr_err("OpenSession failed, ReturnCode=0x%x, ReturnOrigin=0x%x\n", result, origin);
        l_RetVal = CALL_SEC_ERR;
        goto cleanup_2;
    }
    // pr_err("OpenSession success\n");
    
    /**3) Set the communication context between CA&TA */
    memset(&l_operation, 0x0, sizeof(TEEC_Operation));
    l_operation.started = 1;
    l_operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_OUTPUT,TEEC_NONE, 
                                              TEEC_NONE, TEEC_NONE);
    l_operation.params[0].tmpref.size = len;
    l_operation.params[0].tmpref.buffer = output;

    /**4) Send command to TA */
    result = TEEC_InvokeCommand_ChipId(&g_TaskContext, &l_operation, &l_session, &origin);
    if (result != TEEC_SUCCESS) {
        pr_err("InvokeCommand failed, ReturnCode=0x%x, ReturnOrigin=0x%x\n", result, origin);
        l_RetVal = CALL_SEC_ERR;
        goto cleanup_3;
    }
    // pr_err("InvokeCommand success\n");
    
    /**5) The clean up operation */
    cleanup_3:
        TEEC_CloseSession(&l_session);
    cleanup_2:
        TEEC_FinalizeContext(&g_TaskContext);
        g_TaskInitFlag = -1;
    cleanup_1:
        g_optee_used = 0;
        return l_RetVal;
}


