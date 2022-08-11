/*
 * Copyright (c) 2015-2016, Linaro Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/errno.h>
#include <linux/fcntl.h>
// #include <limits.h>
#include <linux/limits.h>
// #include <pthread.h>
#include <linux/kthread.h>
// #include <stdio.h>
// #include <stdlib.h>
#include <linux/string.h>
// #include <linux/mman.h> // <sys/mman.h>
// #include <sys/mman.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <tee_client_api.h>
#include "../tee_private.h"
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/tee.h>
#include <uapi/linux/tee.h>
#include <linux/fs.h>
#include "call_sec.h"

/*#ifndef __aligned
#define __aligned(x) __attribute__((__aligned__(x)))
#endif*/


// #include "teec_benchmark.h"

/* How many device sequence numbers will be tried before giving up */
#define TEEC_MAX_DEV_SEQ	10

/* Helpers to access memref parts of a struct tee_param */
#define MEMREF_SHM_PTR(p)	((p)->u.memref.shm)
#define MEMREF_SHM_OFFS(p)	((p)->u.memref.shm_offs)
#define MEMREF_SIZE(p)		((p)->u.memref.size)

/*
 * Internal flags of TEEC_SharedMemory::internal.flags
 */
#define SHM_FLAG_BUFFER_ALLOCED		(1u << 0)
#define SHM_FLAG_SHADOW_BUFFER_ALLOCED	(1u << 1)

/* static pthread_mutex_t teec_mutex = PTHREAD_MUTEX_INITIALIZER;

static void teec_mutex_lock(pthread_mutex_t *mu)
{
	pthread_mutex_lock(mu);
}

static void teec_mutex_unlock(pthread_mutex_t *mu)
{
	pthread_mutex_unlock(mu);
}
*/

static struct file *bsttee_open_dev(const char *devname, const char *capabilities,
			 uint32_t *gen_caps)
{
	struct file *filp = NULL;
	struct tee_context *ctx_tee = NULL;
	struct tee_ioctl_version_data vers;

	memset(&vers, 0, sizeof(vers));

	filp = filp_open(devname, O_RDWR, 0);
	if (!filp || IS_ERR(filp)) {
		pr_err("devname:%s", devname);
		return NULL;
	}

	ctx_tee = filp->private_data;
	ctx_tee->teedev->desc->ops->get_version(ctx_tee->teedev, &vers);

	if (ctx_tee->teedev->desc->flags & TEE_DESC_PRIVILEGED)
		vers.gen_caps |= TEE_GEN_CAP_PRIVILEGED;

	/* pr_err("teec_open_dev: impl_id:%d, impl_caps:%d, gen_caps:%d",
		vers.impl_id, vers.impl_caps, vers.gen_caps); */
		
	/* We can only handle GP TEEs */
	if (!(vers.gen_caps & TEE_GEN_CAP_GP)) {
		filp_close(filp, NULL);
		return NULL;
	}
		
	if (capabilities) {
		if (strcmp(capabilities, "optee-tz") == 0) {
			if (vers.impl_id != TEE_IMPL_ID_OPTEE) {
				filp_close(filp, NULL);
			}
				return NULL;
			if (!(vers.impl_caps & TEE_OPTEE_CAP_TZ)){
				filp_close(filp, NULL);
				return NULL;
			}
		} else {
			/* Unrecognized capability requested */
			filp_close(filp, NULL);
			return NULL;
		}
	}

	*gen_caps = vers.gen_caps;
	return filp;
}

TEEC_Result TEEC_InitializeContext(const char *name, TEEC_Context *ctx)
{
	char devname[PATH_MAX] = { 0 };
	size_t n = 0;
	struct file *filp = NULL;
	
	if (!ctx)
		return TEEC_ERROR_BAD_PARAMETERS;
	
	for (n = 0; n < TEEC_MAX_DEV_SEQ; n++) {
		uint32_t gen_caps = 0;

		snprintf(devname, sizeof(devname), "/dev/tee%zu", n);
		filp = bsttee_open_dev(devname, name, &gen_caps);
		if (filp && !IS_ERR(filp))  {
			ctx->file = filp;
			ctx->reg_mem = gen_caps & TEE_GEN_CAP_REG_MEM;
			ctx->memref_null = gen_caps & TEE_GEN_CAP_MEMREF_NULL;
			pr_err("TEEC_InitializeContext:ctx->reg_mem:%d, ctx->memref_null:%d", ctx->reg_mem, ctx->memref_null);
			return TEEC_SUCCESS;
		}
	}

	return TEEC_ERROR_ITEM_NOT_FOUND;
}

void TEEC_FinalizeContext(TEEC_Context *ctx)
{
	if (ctx)
		filp_close(ctx->file, NULL);
}

static TEEC_Result teec_pre_process_operation_chipid(
			TEEC_Context *ctx, struct tee_context *ctx_tee,
			TEEC_TempMemoryReference *tmpref,
			struct tee_param *param, TEEC_SharedMemory *shm)
{
	TEEC_Result res = TEEC_ERROR_GENERIC;

	param->attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	shm->flags = TEEC_MEM_OUTPUT;
	shm->size = tmpref->size;

	if (!tmpref->buffer) {
		if (tmpref->size)
			return TEEC_ERROR_BAD_PARAMETERS;

		if (ctx->memref_null) {
			/* Null pointer, indicate no shared memory attached */
			MEMREF_SHM_PTR(param) = NULL; //TEE_MEMREF_NULL;
			shm->id = -1;
		} else {
			res = TEEC_AllocateSharedMemory(ctx, ctx_tee, shm);
			if (res != TEEC_SUCCESS)
				return res;
			MEMREF_SHM_PTR(param) = tee_shm_get_from_id(ctx_tee, shm->id);
			// pr_err("teec_pre_process_operation_chipid ");
		}
	} else {
		shm->buffer = tmpref->buffer;
		res = TEEC_RegisterSharedMemory(ctx, ctx_tee, shm); ///////////////
		if (res != TEEC_SUCCESS)
			return res;
		
		if (shm->shadow_buffer) {
			memcpy(shm->shadow_buffer, tmpref->buffer,
			       tmpref->size);
			// pr_err("shadow_buffer is using");
		}
		/* pr_err("tmpref->buffer:%lx, shm->buffer:%lx, shm->shadow_buffer:%lx",
			(uint64_t)tmpref->buffer, (uint64_t)shm->buffer, (uint64_t)shm->shadow_buffer); */
		MEMREF_SHM_PTR(param) = tee_shm_get_from_id(ctx_tee, shm->id); //改成shadow buffer------
	}
	MEMREF_SIZE(param) = tmpref->size;
	MEMREF_SHM_OFFS(param) = 0;
	
	return TEEC_SUCCESS;
}

static TEEC_Result teec_pre_process_operation(TEEC_Context *ctx,
			struct tee_context *ctx_tee,
			TEEC_Operation *operation,
			struct tee_param *params,
			TEEC_SharedMemory *shms)
{
	TEEC_Result res = TEEC_ERROR_GENERIC;
	size_t n = 0;

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++)
		shms[n].id = -1;

	if (!operation) {
		memset(params, 0, sizeof(struct tee_param) *
				  TEEC_CONFIG_PAYLOAD_REF_COUNT);
		return TEEC_SUCCESS;
	}

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		uint32_t param_type = 0;

		param_type = TEEC_PARAM_TYPE_GET(operation->paramTypes, n);
		switch (param_type) {
		case TEEC_NONE:
			params[n].attr = param_type;
			break;
		case TEEC_VALUE_INPUT:
		case TEEC_VALUE_OUTPUT:
		case TEEC_VALUE_INOUT:
			params[n].attr = param_type;
			/*params[n].a = operation->params[n].value.a;
			params[n].b = operation->params[n].value.b;*/
			break;
		case TEEC_MEMREF_TEMP_INPUT:
		case TEEC_MEMREF_TEMP_OUTPUT:
		case TEEC_MEMREF_TEMP_INOUT:
			res = teec_pre_process_operation_chipid(ctx, ctx_tee,
				&operation->params[n].tmpref, params + n,
				shms + n);
			if (res != TEEC_SUCCESS)
				return res;
			break;
		case TEEC_MEMREF_WHOLE:
			/*res = teec_pre_process_whole(
					&operation->params[n].memref,
					params + n);
			if (res != TEEC_SUCCESS)
				return res;*/
			break;
		case TEEC_MEMREF_PARTIAL_INPUT:
		case TEEC_MEMREF_PARTIAL_OUTPUT:
		case TEEC_MEMREF_PARTIAL_INOUT:
			/*res = teec_pre_process_partial(param_type,
				&operation->params[n].memref, params + n);
			if (res != TEEC_SUCCESS)
				return res;*/
			break;
		default:
			return TEEC_ERROR_BAD_PARAMETERS;
		}
	}

	return TEEC_SUCCESS;
}

static void teec_post_process_tmpref_chipid(TEEC_TempMemoryReference *tmpref,
			struct tee_param *param, TEEC_SharedMemory *shm)
{
	struct tee_shm *shm_tee = MEMREF_SHM_PTR(param);
	char *p;
	phys_addr_t pa;
	if (MEMREF_SIZE(param) <= tmpref->size && tmpref->buffer &&
		shm->shadow_buffer) {
		memcpy(tmpref->buffer, shm->shadow_buffer, MEMREF_SIZE(param));
		/* pr_err("teec_post_process_tmpref_chipid: shm->buffer:%x, shm->shadow_buffer:%x",
			*(char *)shm->buffer, *(char *)shm->shadow_buffer); */
	}
	
	/* pr_err("teec_post_process_tmpref_chipid: tmpref->size:%ld, MEMREF_SIZE(param):%ld, tmpref->buffer:%llx,"
		"shm_tee addr:%llx, shm->shadow_buffer:%llx",
		tmpref->size, MEMREF_SIZE(param), (uint64_t)tmpref->buffer, (uint64_t)shm_tee, (uint64_t)shm->shadow_buffer); */

	tmpref->size = MEMREF_SIZE(param);
}

static void teec_free_temp_refs_chipid(TEEC_SharedMemory *shms, struct tee_param *params)
{
	size_t n = 0;

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		TEEC_ReleaseSharedMemory(shms + n, params + n);
	}
}

static void uuid_to_octets(uint8_t d[TEE_IOCTL_UUID_LEN], const TEEC_UUID *s)
{
	d[0] = s->timeLow >> 24;
	d[1] = s->timeLow >> 16;
	d[2] = s->timeLow >> 8;
	d[3] = s->timeLow;
	d[4] = s->timeMid >> 8;
	d[5] = s->timeMid;
	d[6] = s->timeHiAndVersion >> 8;
	d[7] = s->timeHiAndVersion;
	memcpy(d + 8, s->clockSeqAndNode, sizeof(s->clockSeqAndNode));
}

static void setup_client_data(struct tee_ioctl_open_session_arg *arg,
			      uint32_t connection_method,
			      const void *connection_data)
{
	arg->clnt_login = connection_method;

	switch (connection_method) {
	case TEE_IOCTL_LOGIN_PUBLIC:
		/* No connection data to pass */
		break;
	case TEE_IOCTL_LOGIN_USER:
		/* Kernel auto-fills UID and forms client UUID */
		break;
	case TEE_IOCTL_LOGIN_GROUP:
		/*
		 * Connection data for group login is uint32_t and rest of
		 * clnt_uuid is set as zero.
		 *
		 * Kernel verifies group membership and then forms client UUID.
		 */
		memcpy(arg->clnt_uuid, connection_data, sizeof(gid_t));
		break;
	case TEE_IOCTL_LOGIN_APPLICATION:
		/*
		 * Kernel auto-fills application identifier and forms client
		 * UUID.
		 */
		break;
	case TEE_IOCTL_LOGIN_USER_APPLICATION:
		/*
		 * Kernel auto-fills application identifier, UID and forms
		 * client UUID.
		 */
		break;
	case TEE_IOCTL_LOGIN_GROUP_APPLICATION:
		/*
		 * Connection data for group login is uint32_t rest of
		 * clnt_uuid is set as zero.
		 *
		 * Kernel verifies group membership, auto-fills application
		 * identifier and then forms client UUID.
		 */
		memcpy(arg->clnt_uuid, connection_data, sizeof(gid_t));
		break;
	default:
		/*
		 * Unknown login method, don't pass any connection data as we
		 * don't know size.
		 */
		break;
	}
}
// extern struct optee *optee_svc;
TEEC_Result TEEC_OpenSession_ChipId(TEEC_Context *ctx, TEEC_Session *session,
			const TEEC_UUID *destination,
			uint32_t connection_method, const void *connection_data,
			uint32_t *ret_origin)
{
	struct tee_ioctl_open_session_arg *arg = NULL;
	struct tee_param *params = NULL;
	TEEC_Result res = TEEC_ERROR_GENERIC;
	uint32_t eorig = 0;
	int rc = 0;
	const size_t arg_size = sizeof(struct tee_ioctl_open_session_arg) +
				TEEC_CONFIG_PAYLOAD_REF_COUNT *	sizeof(struct tee_param);
	union {
		struct tee_ioctl_open_session_arg arg;
		uint8_t data[arg_size];
	} buf;
	struct file *filp = ctx->file;
	struct tee_context *ctx_tee = filp->private_data;

	memset(&buf, 0, sizeof(buf));

	if (!ctx || !session) {
		eorig = TEEC_ORIGIN_API;
		res = TEEC_ERROR_BAD_PARAMETERS;
		goto out;
	}

	arg = &buf.arg;
	arg->num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;
	params = (struct tee_param *)(arg + 1);

	uuid_to_octets(arg->uuid, destination);

	setup_client_data(arg, connection_method, connection_data);

	// rc = tee_ioctl_open_session(ctx_tee, uarg);
	rc = ctx_tee->teedev->desc->ops->open_session(ctx_tee, arg, params);
	if (rc) {
		pr_err("TEE_IOC_OPEN_SESSION failed");
		eorig = TEEC_ORIGIN_COMMS;
		res = TEEC_ERROR_GENERIC;
		goto out_free_temp_refs;
	}
	res = arg->ret;
	eorig = arg->ret_origin;
	if (res == TEEC_SUCCESS) {
		session->ctx = ctx;
		session->session_id = arg->session;
	}

out_free_temp_refs:
	
out:
	if (ret_origin)
		*ret_origin = eorig;
	return res;
}

void TEEC_CloseSession(TEEC_Session *session)
{
	struct file *filp = session->ctx->file;
	struct tee_context *ctx = filp->private_data;

	if (!session)
		return;
	
	if (!ctx->teedev->desc->ops->close_session)
		return;
	
	ctx->teedev->desc->ops->close_session(ctx, session->session_id);
	// pr_err("Failed to close session 0x%x", session->session_id);
}

TEEC_Result TEEC_InvokeCommand_ChipId(TEEC_Context *ctx, 
			TEEC_Operation *operation,
			TEEC_Session *session, uint32_t *error_origin)
{
	struct tee_ioctl_invoke_arg *arg = NULL;
	struct tee_param *params = NULL;
	TEEC_Result res = TEEC_ERROR_GENERIC;
	uint32_t eorig = 0;
	int rc = 0;
	const size_t arg_size = sizeof(struct tee_ioctl_invoke_arg) +
				TEEC_CONFIG_PAYLOAD_REF_COUNT *
					sizeof(struct tee_param);
	union {
		struct tee_ioctl_invoke_arg arg;
		uint8_t data[arg_size];
	} buf;
	TEEC_SharedMemory shm[TEEC_CONFIG_PAYLOAD_REF_COUNT];
	struct file *filp = session->ctx->file;
	struct tee_context *ctx_tee = filp->private_data;
	size_t n = 0;

	memset(&buf, 0, sizeof(buf));
	memset(&shm, 0, sizeof(shm));

	if (!session) {
		eorig = TEEC_ORIGIN_API;
		res = TEEC_ERROR_BAD_PARAMETERS;
		goto out;
	}

	// bm_timestamp(); //----------/////////////////////

	arg = &buf.arg;
	arg->num_params = TEEC_CONFIG_PAYLOAD_REF_COUNT;
	params = (struct tee_param *)(arg + 1);

	arg->session = session->session_id;
	arg->func = eDEVID_GET;
	// arg->func = eTRNG;
	
	res = teec_pre_process_operation(ctx, ctx_tee, operation, params, shm);
	if (res != TEEC_SUCCESS) {
		eorig = TEEC_ORIGIN_API;
		goto out_free_temp_refs;
	}

	// rc = tee_ioctl(session->ctx->fd, TEE_IOC_INVOKE, &buf_data);
	rc = ctx_tee->teedev->desc->ops->invoke_func(ctx_tee, arg, params);
	if (rc) {
		pr_err("TEE_IOC_INVOKE failed");
		eorig = TEEC_ORIGIN_COMMS;
		res = TEEC_ERROR_GENERIC;
		goto out_free_temp_refs;
	}

	res = arg->ret;
	eorig = arg->ret_origin;
	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		teec_post_process_tmpref_chipid(&operation->params[n].tmpref, params + n, shm + n);
	}
	// bm_timestamp(); //////////////////////////--------

out_free_temp_refs:
	teec_free_temp_refs_chipid(shm, params);
out:
	if (error_origin)
		*error_origin = eorig;
	return res;
}

/*void TEEC_RequestCancellation(TEEC_Operation *operation)
{
	TEEC_Session *session = NULL;
	struct tee_ioctl_cancel_arg arg;

	memset(&arg, 0, sizeof(arg));

	if (!operation)
		return;

	// teec_mutex_lock(&teec_mutex);
	session = operation->session;
	// teec_mutex_unlock(&teec_mutex);

	if (!session)
		return;

	arg.session = session->session_id;
	arg.cancel_id = 0;

	if (tee_ioctl(session->ctx->fd, TEE_IOC_CANCEL, &arg))
		pr_err("TEE_IOC_CANCEL: %s", strerror(errno));
}*/

TEEC_Result TEEC_RegisterSharedMemory(TEEC_Context *ctx, struct tee_context *ctx_tee,
									 TEEC_SharedMemory *shm)
{
	TEEC_Result res = TEEC_SUCCESS;
	size_t s = 0;
	struct tee_shm *shm_tee;
	int shm_fd = 0;

	if (!ctx || !shm)
		return TEEC_ERROR_BAD_PARAMETERS;

	if (!shm->flags || (shm->flags & ~(TEEC_MEM_INPUT | TEEC_MEM_OUTPUT)))
		return TEEC_ERROR_BAD_PARAMETERS;

	if (!shm->buffer)
		return TEEC_ERROR_BAD_PARAMETERS;

	s = shm->size;
	if (!s)
		s = 8;
	if (ctx->reg_mem) {
		shm_tee = tee_shm_register(ctx_tee, (uintptr_t)shm->buffer, s,
			TEE_SHM_DMA_BUF | TEE_SHM_MAPPED);  //TEE_SHM_USER_MAPPED
		shm->id = shm_tee->id;
		if (!IS_ERR(shm_tee)) {
			shm->registered_shm = shm_tee;
			shm->shadow_buffer = NULL;
			shm->internal.flags = 0;
			pr_err("shm->buffer(x%llx) is valid. shm->size:%ld", (uint64_t)shm->buffer, s);
			goto out;
		}
		pr_err("not supposed 11!!!");
		/*
		 * If we're here TEE_IOC_SHM_REGISTER failed, probably
		 * because some read-only memory was supplied and the Linux
		 * kernel doesn't like that at the moment.
		 *
		 * The error could also have some other origin. In any case
		 * we're not making matters worse by trying to allocate and
		 * register a shadow buffer before giving up.
		 */
		shm->shadow_buffer = kmalloc(s, GFP_KERNEL);
		if (!shm->shadow_buffer)
			return TEEC_ERROR_OUT_OF_MEMORY;
		shm_tee = tee_shm_register(ctx_tee, (uintptr_t)shm->shadow_buffer, s,
			       TEE_SHM_DMA_BUF | TEE_SHM_MAPPED); //TEE_SHM_USER_MAPPED
		shm->id = shm_tee->id;
		if (!IS_ERR(shm_tee)) {
			shm->registered_shm = shm_tee;
			shm->internal.flags = SHM_FLAG_SHADOW_BUFFER_ALLOCED;
			pr_err("shm->shadow_buffer(x%llx) is valid. shm->size:%ld", (uint64_t)shm->shadow_buffer, s);
			goto out;
		}

		res = TEEC_ERROR_GENERIC;
		kfree(shm->shadow_buffer);
		shm->shadow_buffer = NULL;
		return res;
	} else {
		shm_tee = tee_shm_alloc(ctx_tee, s, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
		if (IS_ERR(shm))
			return TEEC_ERROR_OUT_OF_MEMORY;
		shm->id = shm_tee->id;
		shm_fd = tee_shm_get_fd(shm_tee);  //这段删掉---- version3
		if (shm_fd < 0) {
			pr_err("tee_shm_get_fd fail");
			return TEEC_ERROR_OUT_OF_MEMORY;
		}
		tee_shm_put(shm_tee);

		/* 原代码 */
		/* shm->shadow_buffer = mmap(NULL, s, PROT_READ | PROT_WRITE,
					  MAP_SHARED, shm_fd, 0);
		close(fd);
		if (shm->shadow_buffer == (void *)MAP_FAILED) { */

		/* 方案1：ok*/
		shm->shadow_buffer = tee_shm_get_va(shm_tee, 0);
		if (shm->shadow_buffer == ERR_PTR(-EINVAL)) {
		/* 方案2：参数不合法 ret = tee_shm_get_pa(shm_tee, 0, shm->shadow_buffer);   InvokeCommand failed, ReturnCode=0xffff0006, ReturnOrigin=0x4
		if (ret == ERR_PTR(-EINVAL)) { */
			pr_err("tee_shm_get_va fail");
			shm->id = -1;
			return TEEC_ERROR_OUT_OF_MEMORY;
		}
		// 方案3：shm->shadow_buffer = NULL; 打印乱码
		// 方案5：1)内存访问异常：Unable to handle kernel paging request at virtual address ffff8000e7885000
		// 2)注释shadow_buffer和temp-buffer的互相拷贝后，打印乱码
		// shm->shadow_buffer = shm->buffer;

		/* 方案4：会导致optee os打印乱码
		shm->shadow_buffer = kzalloc(s, GFP_KERNEL); //同map的区别是不会读取fd指向的内容
		if (!shm->shadow_buffer) {
			shm->id = -1;
			return TEEC_ERROR_OUT_OF_MEMORY;
		}*/
		shm->registered_shm = NULL;
		shm->internal.flags = 0;
        // pr_err("tee_shm_alloc is success. shadow_buffer(x%llx), shm->size:%ld", (uint64_t)shm->shadow_buffer, s);
	}

out:
	shm->alloced_size = s;
	return TEEC_SUCCESS;
}

/*TEEC_Result TEEC_RegisterSharedMemoryFileDescriptor(TEEC_Context *ctx,
						    TEEC_SharedMemory *shm,
						    int fd)
{
	int rfd = 0;
	struct tee_ioctl_shm_register_fd_data data;

	memset(&data, 0, sizeof(data));

	if (!ctx || !shm || fd < 0)
		return TEEC_ERROR_BAD_PARAMETERS;

	if (!shm->flags || (shm->flags & ~(TEEC_MEM_INPUT | TEEC_MEM_OUTPUT)))
		return TEEC_ERROR_BAD_PARAMETERS;

	data.fd = fd;
	rfd = tee_ioctl(ctx->fd, TEE_IOC_SHM_REGISTER_FD, &data);
	if (rfd < 0)
		return TEEC_ERROR_BAD_PARAMETERS;

	shm->buffer = NULL;
	shm->shadow_buffer = NULL;
	shm->registered_fd = rfd;
	shm->id = data.id;
	shm->size = data.size;
	return TEEC_SUCCESS;
}*/

TEEC_Result TEEC_AllocateSharedMemory(TEEC_Context *ctx, struct tee_context *ctx_tee, TEEC_SharedMemory *shm)
{
	size_t s = 0;
	struct tee_shm *shm_tee;

	if (!ctx || !shm)
		return TEEC_ERROR_BAD_PARAMETERS;

	if (!shm->flags || (shm->flags & ~(TEEC_MEM_INPUT | TEEC_MEM_OUTPUT)))
		return TEEC_ERROR_BAD_PARAMETERS;

	s = shm->size;
	if (!s)
		s = 8;

	if (ctx->reg_mem) {
		shm->buffer = kmalloc(s, GFP_KERNEL);
		if (!shm->buffer)
			return TEEC_ERROR_OUT_OF_MEMORY;

		shm_tee = tee_shm_register(ctx_tee, (uintptr_t)shm->buffer, s,
			       TEE_SHM_DMA_BUF | TEE_SHM_USER_MAPPED);
		if (IS_ERR(shm_tee)) {
			kfree(shm->buffer);
			shm->buffer = NULL;
			return TEEC_ERROR_OUT_OF_MEMORY;
		}
		shm->registered_shm = shm_tee;
	} else {
		shm_tee = tee_shm_alloc(ctx_tee, s, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
		if (IS_ERR(shm))
			return -1;
		shm->id = shm_tee->id;
		
		shm->buffer = tee_shm_get_va(shm_tee, 0); //////////////是否应该用物理地址tee_shm_get_pa  原文是mmap
		if (shm->buffer == ERR_PTR(-EINVAL)) {
			shm->id = -1;
			return TEEC_ERROR_OUT_OF_MEMORY;
		}
		shm->registered_shm = NULL;
	}

	shm->shadow_buffer = NULL;
	shm->alloced_size = s;
	shm->internal.flags = SHM_FLAG_BUFFER_ALLOCED;
	return TEEC_SUCCESS;
}

void TEEC_ReleaseSharedMemory(TEEC_SharedMemory *shm, struct tee_param *param)
{
	struct tee_shm *shm_tee = NULL;

	if (!shm || shm->id == -1 || !param)
		return;

	if (shm->shadow_buffer) {
		if (shm->registered_shm != NULL) {
			if (shm->internal.flags &
			    SHM_FLAG_SHADOW_BUFFER_ALLOCED)
				kfree(shm->shadow_buffer);
			// ksys_close(shm->registered_fd);
			tee_shm_free(shm->registered_shm);
		} else {
			// munmap(shm->shadow_buffer, shm->alloced_size);
			shm_tee = MEMREF_SHM_PTR(param);
			if (!shm_tee) 
				pr_err("shm_tee is null");
			else
				tee_shm_free(shm_tee);
		}
	} else if (shm->buffer) {
		if (shm->registered_shm != NULL) {
			if (shm->internal.flags & SHM_FLAG_BUFFER_ALLOCED)
				kfree(shm->buffer);
			// ksys_close(shm->registered_fd);
			tee_shm_free(shm->registered_shm);
		} else {
			// munmap(shm->buffer, shm->alloced_size);
			shm_tee = MEMREF_SHM_PTR(param);
			if (!shm_tee) 
				pr_err("shm_tee is null");
			else
				tee_shm_free(shm_tee);
		}
	} else if (shm->registered_shm != NULL) {
		// ksys_close(shm->registered_fd);
		tee_shm_free(shm->registered_shm);
	}

	shm->id = -1;
	shm->shadow_buffer = NULL;
	shm->buffer = NULL;
	shm->registered_shm = NULL;
	shm->internal.flags = 0;
}
