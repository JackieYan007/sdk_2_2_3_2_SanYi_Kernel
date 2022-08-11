/*
* © Copyright Black Sesame Technologies (Shanghai)Ltd. Co., 2020. All rights reserved.
*
* This file contains proprietary information that is the sole intellectual property of
* Black Sesame Technologies (Shanghai)Ltd. Co.
*
* No part of this material or its documentation may be reproduced, distributed,
* transmitted, displayed or published in any manner without the written permission
* of Black Sesame Technologies (Shanghai)Ltd. Co.
*/

#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/types.h>

#include "../optee/optee_private.h"
#include "call_sec.h"

#define CHIPID_LEN 16
int tee_get_cpu_id(uint64_t *id)
{
    int rc = 0;
    static char *locId = NULL;
    if (locId == NULL) {
        locId = kzalloc(CHIPID_LEN+1, GFP_KERNEL);
    }
    if (!locId)
	    return -1;
    // rc = optee_get_chipid(16, (void *)id);
    // id[0] = 0xffffffff;
    // id[1] = 0xffffffff;
    rc = optee_get_chipid(CHIPID_LEN, (void *)locId);
    memcpy(id, locId, CHIPID_LEN);
    pr_err("locId:%s", locId);
	pr_err("rc:%d, addr:%lx, id0:%lx, id1:%lx", rc, id, id[0], id[1]);
    return rc;
}
EXPORT_SYMBOL(tee_get_cpu_id);

