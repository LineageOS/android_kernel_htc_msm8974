/* Copyright (c) 2002,2007-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/log2.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"

#include "adreno.h"
#include "adreno_pm4types.h"
#include "adreno_ringbuffer.h"

#include "a2xx_reg.h"
#include "a3xx_reg.h"

#define GSL_RB_NOP_SIZEDWORDS				2

void adreno_ringbuffer_submit(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	BUG_ON(rb->wptr == 0);

	kgsl_pwrscale_busy(rb->device);

	mb();

	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_WPTR, rb->wptr);
}

static int
adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb,
				struct adreno_context *context,
				unsigned int numcmds, int wptr_ahead)
{
	int nopcount;
	unsigned int freecmds;
	unsigned int *cmds;
	uint cmds_gpu;
	unsigned long wait_time;
	unsigned long wait_timeout = msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);
	unsigned long wait_time_part;
	unsigned int rptr;

	
	if (wptr_ahead) {
		
		nopcount = rb->sizedwords - rb->wptr - 1;

		cmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		cmds_gpu = rb->buffer_desc.gpuaddr + sizeof(uint)*rb->wptr;

		GSL_RB_WRITE(rb->device, cmds, cmds_gpu,
				cp_nop_packet(nopcount));

		do {
			rptr = adreno_get_rptr(rb);
		} while (!rptr);

		rb->wptr = 0;
	}

	wait_time = jiffies + wait_timeout;
	wait_time_part = jiffies + msecs_to_jiffies(KGSL_TIMEOUT_PART);
	
	while (1) {
		rptr = adreno_get_rptr(rb);

		freecmds = rptr - rb->wptr;

		if (freecmds == 0 || freecmds > numcmds)
			break;

		if (time_after(jiffies, wait_time)) {
			KGSL_DRV_ERR(rb->device,
			"Timed out while waiting for freespace in ringbuffer "
			"rptr: 0x%x, wptr: 0x%x\n", rptr, rb->wptr);
			return -ETIMEDOUT;
		}

	}
	return 0;
}

unsigned int *adreno_ringbuffer_allocspace(struct adreno_ringbuffer *rb,
					struct adreno_context *context,
					unsigned int numcmds)
{
	unsigned int *ptr = NULL;
	int ret = 0;
	unsigned int rptr;
	BUG_ON(numcmds >= rb->sizedwords);

	rptr = adreno_get_rptr(rb);
	
	if (rb->wptr >= rptr) {
		
		
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			ret = adreno_ringbuffer_waitspace(rb, context,
							numcmds, 1);
	} else {
		
		if ((rb->wptr + numcmds) >= rptr)
			ret = adreno_ringbuffer_waitspace(rb, context,
							numcmds, 0);
		
		
		if (!ret && (rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			ret = adreno_ringbuffer_waitspace(rb, context,
							numcmds, 1);
	}

	if (!ret) {
		ptr = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		rb->wptr += numcmds;
	} else
		ptr = ERR_PTR(ret);

	return ptr;
}

static int _load_firmware(struct kgsl_device *device, const char *fwfile,
			  void **data, int *len)
{
	const struct firmware *fw = NULL;
	int ret;

	ret = request_firmware(&fw, fwfile, device->dev);

	if (ret) {
		KGSL_DRV_ERR(device, "request_firmware(%s) failed: %d\n",
			     fwfile, ret);
		return ret;
	}

	*data = kmalloc(fw->size, GFP_KERNEL);

	if (*data) {
		memcpy(*data, fw->data, fw->size);
		*len = fw->size;
	} else
		KGSL_MEM_ERR(device, "kmalloc(%d) failed\n", fw->size);

	release_firmware(fw);
	return (*data != NULL) ? 0 : -ENOMEM;
}

int adreno_ringbuffer_read_pm4_ucode(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret = 0;

	if (adreno_dev->pm4_fw == NULL) {
		int len;
		void *ptr;

		ret = _load_firmware(device, adreno_dev->pm4_fwfile,
			&ptr, &len);

		if (ret)
			goto err;

		
		if (len % ((sizeof(uint32_t) * 3)) != sizeof(uint32_t)) {
			KGSL_DRV_ERR(device, "Bad firmware size: %d\n", len);
			ret = -EINVAL;
			kfree(ptr);
			goto err;
		}

		adreno_dev->pm4_fw_size = len / sizeof(uint32_t);
		adreno_dev->pm4_fw = ptr;
		adreno_dev->pm4_fw_version = adreno_dev->pm4_fw[1];
	}

err:
	return ret;
}

inline int adreno_ringbuffer_load_pm4_ucode(struct kgsl_device *device,
			unsigned int start, unsigned int end, unsigned int addr)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_RAM_WADDR, addr);
	for (i = start; i < end; i++)
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_RAM_DATA,
					adreno_dev->pm4_fw[i]);

	return 0;
}

int adreno_ringbuffer_read_pfp_ucode(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret = 0;

	if (adreno_dev->pfp_fw == NULL) {
		int len;
		void *ptr;

		ret = _load_firmware(device, adreno_dev->pfp_fwfile,
			&ptr, &len);
		if (ret)
			goto err;

		
		if (len % sizeof(uint32_t) != 0) {
			KGSL_DRV_ERR(device, "Bad firmware size: %d\n", len);
			ret = -EINVAL;
			kfree(ptr);
			goto err;
		}

		adreno_dev->pfp_fw_size = len / sizeof(uint32_t);
		adreno_dev->pfp_fw = ptr;
		adreno_dev->pfp_fw_version = adreno_dev->pfp_fw[5];
	}

err:
	return ret;
}

inline int adreno_ringbuffer_load_pfp_ucode(struct kgsl_device *device,
			unsigned int start, unsigned int end, unsigned int addr)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	if (!adreno_dev)
		return -EINVAL;

	if (adreno_dev->pfp_fw == NULL) {
		int ret = adreno_ringbuffer_read_pfp_ucode(device);
		if (ret)
			return ret;
	}

	KGSL_DRV_INFO(device, "loading pfp ucode version: %d\n",
			adreno_dev->pfp_fw_version);

	adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_ADDR, addr);
	for (i = start; i < end; i++)
		adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_DATA,
						adreno_dev->pfp_fw[i]);

	return 0;
}

static int _ringbuffer_bootstrap_ucode(struct adreno_ringbuffer *rb,
					unsigned int load_jt)
{
	unsigned int *cmds, cmds_gpu, bootstrap_size;
	int i = 0;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int pm4_size, pm4_idx, pm4_addr, pfp_size, pfp_idx, pfp_addr;

	
	if (load_jt) {
		pm4_idx = adreno_dev->pm4_jt_idx;
		pm4_addr = adreno_dev->pm4_jt_addr;
		pfp_idx = adreno_dev->pfp_jt_idx;
		pfp_addr = adreno_dev->pfp_jt_addr;
	} else {
		
		pm4_idx = 1;
		pm4_addr = 0;
		pfp_idx = 1;
		pfp_addr = 0;
	}

	pm4_size = (adreno_dev->pm4_fw_size - pm4_idx);
	pfp_size = (adreno_dev->pfp_fw_size - pfp_idx);

	adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_ADDR, 0x200);
	adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_DATA, 0x6f0005);

	
	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, 0);

	bootstrap_size = (pm4_size + pfp_size + 5);

	cmds = adreno_ringbuffer_allocspace(rb, NULL, bootstrap_size);
	if (cmds == NULL)
			return -ENOMEM;

	cmds_gpu = rb->buffer_desc.gpuaddr +
			sizeof(uint) * (rb->wptr - bootstrap_size);
	
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu,
			cp_type3_packet(CP_BOOTSTRAP_UCODE,
			(bootstrap_size - 1)));
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, pfp_size);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, pfp_addr);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, pm4_size);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, pm4_addr);
	for (i = pfp_idx; i < adreno_dev->pfp_fw_size; i++)
		GSL_RB_WRITE(rb->device, cmds, cmds_gpu, adreno_dev->pfp_fw[i]);
	for (i = pm4_idx; i < adreno_dev->pm4_fw_size; i++)
		GSL_RB_WRITE(rb->device, cmds, cmds_gpu, adreno_dev->pm4_fw[i]);

	adreno_ringbuffer_submit(rb);
	
	return adreno_idle(device);
}

void _ringbuffer_setup_common(struct adreno_ringbuffer *rb)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	kgsl_sharedmem_set(rb->device, &rb->buffer_desc, 0, 0xAA,
			   (rb->sizedwords << 2));


	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_CNTL,
		(ilog2(rb->sizedwords >> 1) & 0x3F) |
		(1 << 27));

	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_BASE,
					rb->buffer_desc.gpuaddr);

	if (adreno_is_a2xx(adreno_dev)) {
		
		kgsl_regwrite(device, REG_CP_INT_ACK, 0xFFFFFFFF);
	}

	
	adreno_writereg(adreno_dev, ADRENO_REG_SCRATCH_ADDR,
				device->memstore.gpuaddr +
				KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
					soptimestamp));

	adreno_writereg(adreno_dev, ADRENO_REG_SCRATCH_UMSK,
			     GSL_RB_MEMPTRS_SCRATCH_MASK);

	
	if (adreno_is_a305(adreno_dev) || adreno_is_a305c(adreno_dev) ||
		adreno_is_a320(adreno_dev))
		kgsl_regwrite(device, REG_CP_QUEUE_THRESHOLDS, 0x000E0602);
	else if (adreno_is_a330(adreno_dev) || adreno_is_a305b(adreno_dev))
		kgsl_regwrite(device, REG_CP_QUEUE_THRESHOLDS, 0x003E2008);

	rb->wptr = 0;
}

int _ringbuffer_start_common(struct adreno_ringbuffer *rb)
{
	int status;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	
	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, 0);

	
	status = adreno_dev->gpudev->rb_init(adreno_dev, rb);
	if (status)
		return status;

	
	status = adreno_idle(device);

	if (status == 0)
		rb->flags |= KGSL_FLAGS_STARTED;

	return status;
}

int adreno_ringbuffer_warm_start(struct adreno_ringbuffer *rb)
{
	int status;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (rb->flags & KGSL_FLAGS_STARTED)
		return 0;

	_ringbuffer_setup_common(rb);

	
	if (adreno_bootstrap_ucode(adreno_dev)) {
		status = _ringbuffer_bootstrap_ucode(rb, 1);
		if (status != 0)
			return status;

	} else {
		
		status = adreno_ringbuffer_load_pm4_ucode(device,
			adreno_dev->pm4_jt_idx, adreno_dev->pm4_fw_size,
			adreno_dev->pm4_jt_addr);
		if (status != 0)
			return status;

		
		status = adreno_ringbuffer_load_pfp_ucode(device,
			adreno_dev->pfp_jt_idx, adreno_dev->pfp_fw_size,
			adreno_dev->pfp_jt_addr);
		if (status != 0)
			return status;
	}

	status = _ringbuffer_start_common(rb);

	return status;
}

int adreno_ringbuffer_cold_start(struct adreno_ringbuffer *rb)
{
	int status;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (rb->flags & KGSL_FLAGS_STARTED)
		return 0;

	_ringbuffer_setup_common(rb);

	
	if (adreno_bootstrap_ucode(adreno_dev)) {


		status = adreno_ringbuffer_load_pm4_ucode(rb->device, 1,
					adreno_dev->pm4_bstrp_size+1, 0);
		if (status != 0)
			return status;

		status = adreno_ringbuffer_load_pfp_ucode(rb->device, 1,
					adreno_dev->pfp_bstrp_size+1, 0);
		if (status != 0)
			return status;

		
		status = _ringbuffer_bootstrap_ucode(rb, 0);
		if (status != 0)
			return status;

	} else {
		
		status = adreno_ringbuffer_load_pm4_ucode(rb->device, 1,
					adreno_dev->pm4_fw_size, 0);
		if (status != 0)
			return status;

		
		status = adreno_ringbuffer_load_pfp_ucode(rb->device, 1,
					adreno_dev->pfp_fw_size, 0);
		if (status != 0)
			return status;
	}

	status = _ringbuffer_start_common(rb);

	return status;
}

void adreno_ringbuffer_stop(struct adreno_ringbuffer *rb)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (rb->flags & KGSL_FLAGS_STARTED) {
		if (adreno_is_a200(adreno_dev))
			kgsl_regwrite(rb->device, REG_CP_ME_CNTL, 0x10000000);

		rb->flags &= ~KGSL_FLAGS_STARTED;
	}
}

int adreno_ringbuffer_init(struct kgsl_device *device)
{
	int status;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;

	rb->device = device;
	rb->sizedwords = KGSL_RB_SIZE >> 2;

	rb->buffer_desc.flags = KGSL_MEMFLAGS_GPUREADONLY;
	
	status = kgsl_allocate_contiguous(&rb->buffer_desc,
		(rb->sizedwords << 2));

	if (status != 0) {
		adreno_ringbuffer_close(rb);
		return status;
	}

	rb->global_ts = 0;

	return 0;
}

void adreno_ringbuffer_close(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);

	kgsl_sharedmem_free(&rb->buffer_desc);

	kfree(adreno_dev->pfp_fw);
	kfree(adreno_dev->pm4_fw);

	adreno_dev->pfp_fw = NULL;
	adreno_dev->pm4_fw = NULL;

	memset(rb, 0, sizeof(struct adreno_ringbuffer));
}

static int
adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
				struct adreno_context *drawctxt,
				unsigned int flags, unsigned int *cmds,
				int sizedwords, uint32_t timestamp)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	unsigned int *ringcmds;
	unsigned int total_sizedwords = sizedwords;
	unsigned int i;
	unsigned int rcmd_gpu;
	unsigned int context_id;
	unsigned int gpuaddr = rb->device->memstore.gpuaddr;
	bool profile_ready;

	if (drawctxt != NULL && kgsl_context_detached(&drawctxt->base))
		return -EINVAL;

	rb->global_ts++;

	
	if (!drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		timestamp = rb->global_ts;
		context_id = KGSL_MEMSTORE_GLOBAL;
	} else {
		context_id = drawctxt->base.id;
	}

	if (drawctxt)
		drawctxt->internal_timestamp = rb->global_ts;

	profile_ready = !adreno_is_a2xx(adreno_dev) && drawctxt &&
		adreno_profile_assignments_ready(&adreno_dev->profile) &&
		!(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE);

	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
	
	total_sizedwords += 2;
	
	total_sizedwords += (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE) ? 2 : 0;

	
	total_sizedwords +=
		(drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) ?  2 : 0;

	if (adreno_is_a3xx(adreno_dev))
		total_sizedwords += 7;

	if (adreno_is_a2xx(adreno_dev))
		total_sizedwords += 2; 

	total_sizedwords += 3; 
	total_sizedwords += 4; 

	if (adreno_is_a20x(adreno_dev))
		total_sizedwords += 2; 

	if (drawctxt) {
		total_sizedwords += 3; 
	}

	if (adreno_is_a20x(adreno_dev))
		total_sizedwords += 2; 

	if (flags & KGSL_CMD_FLAGS_WFI)
		total_sizedwords += 2; 

	if (profile_ready)
		total_sizedwords += 6;   

	
	if (flags & KGSL_CMD_FLAGS_PWRON_FIXUP)
		total_sizedwords += 9;

	ringcmds = adreno_ringbuffer_allocspace(rb, drawctxt, total_sizedwords);

	if (IS_ERR(ringcmds))
		return PTR_ERR(ringcmds);
	if (ringcmds == NULL)
		return -ENOSPC;

	rcmd_gpu = rb->buffer_desc.gpuaddr
		+ sizeof(uint)*(rb->wptr-total_sizedwords);

	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, cp_nop_packet(1));
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, KGSL_CMD_IDENTIFIER);

	if (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, cp_nop_packet(1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
				KGSL_CMD_INTERNAL_IDENTIFIER);
	}

	if (flags & KGSL_CMD_FLAGS_PWRON_FIXUP) {
		
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 0);

		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, cp_nop_packet(1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
				KGSL_PWRON_FIXUP_IDENTIFIER);
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			CP_HDR_INDIRECT_BUFFER_PFD);
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			adreno_dev->pwron_fixup.gpuaddr);
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			adreno_dev->pwron_fixup_dwords);

		
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 1);
	}

	
	if (profile_ready)
		adreno_profile_preib_processing(rb->device, drawctxt->base.id,
				&flags, &ringcmds, &rcmd_gpu);

	
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_MEM_WRITE, 2));
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, (gpuaddr +
		KGSL_MEMSTORE_OFFSET(context_id, soptimestamp)));
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, timestamp);

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 0);
	}

	for (i = 0; i < sizedwords; i++) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, *cmds);
		cmds++;
	}

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 1);
	}

	if (adreno_is_a2xx(adreno_dev)) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_WAIT_FOR_IDLE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 0x00);
	}

	if (adreno_is_a3xx(adreno_dev)) {

		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_EVENT_WRITE, 1));
		GSL_RB_WRITE(rb->device, ringcmds,
			rcmd_gpu, 0x07); 
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_WAIT_FOR_IDLE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 0x00);
	}

	if (profile_ready)
		adreno_profile_postib_processing(rb->device, &flags,
						 &ringcmds, &rcmd_gpu);

	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_EVENT_WRITE, 3));
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, CACHE_FLUSH_TS);
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, (gpuaddr +
		KGSL_MEMSTORE_OFFSET(context_id, eoptimestamp)));
	GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, timestamp);

	if (drawctxt) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_MEM_WRITE, 2));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, (gpuaddr +
			KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
				eoptimestamp)));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			rb->global_ts);
	}

	if (adreno_is_a20x(adreno_dev)) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_EVENT_WRITE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, CACHE_FLUSH);
	}

	if (drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_INTERRUPT, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
				CP_INT_CNTL__RB_INT_MASK);
	}

	if (adreno_is_a3xx(adreno_dev)) {
		
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_SET_CONSTANT, 2));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			(0x4<<16)|(A3XX_HLSQ_CL_KERNEL_GROUP_X_REG - 0x2000));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 0);
	}

	if (flags & KGSL_CMD_FLAGS_WFI) {
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu,
			cp_type3_packet(CP_WAIT_FOR_IDLE, 1));
		GSL_RB_WRITE(rb->device, ringcmds, rcmd_gpu, 0x00000000);
	}

	adreno_ringbuffer_submit(rb);

	return 0;
}

unsigned int
adreno_ringbuffer_issuecmds(struct kgsl_device *device,
						struct adreno_context *drawctxt,
						unsigned int flags,
						unsigned int *cmds,
						int sizedwords)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;

	flags |= KGSL_CMD_FLAGS_INTERNAL_ISSUE;

	return adreno_ringbuffer_addcmds(rb, drawctxt, flags, cmds,
		sizedwords, 0);
}

static bool _parse_ibs(struct kgsl_device_private *dev_priv, uint gpuaddr,
			   int sizedwords);

static bool
_handle_type3(struct kgsl_device_private *dev_priv, uint *hostaddr)
{
	unsigned int opcode = cp_type3_opcode(*hostaddr);
	switch (opcode) {
	case CP_INDIRECT_BUFFER_PFD:
	case CP_INDIRECT_BUFFER_PFE:
	case CP_COND_INDIRECT_BUFFER_PFE:
	case CP_COND_INDIRECT_BUFFER_PFD:
		return _parse_ibs(dev_priv, hostaddr[1], hostaddr[2]);
	case CP_NOP:
	case CP_WAIT_FOR_IDLE:
	case CP_WAIT_REG_MEM:
	case CP_WAIT_REG_EQ:
	case CP_WAT_REG_GTE:
	case CP_WAIT_UNTIL_READ:
	case CP_WAIT_IB_PFD_COMPLETE:
	case CP_REG_RMW:
	case CP_REG_TO_MEM:
	case CP_MEM_WRITE:
	case CP_MEM_WRITE_CNTR:
	case CP_COND_EXEC:
	case CP_COND_WRITE:
	case CP_EVENT_WRITE:
	case CP_EVENT_WRITE_SHD:
	case CP_EVENT_WRITE_CFL:
	case CP_EVENT_WRITE_ZPD:
	case CP_DRAW_INDX:
	case CP_DRAW_INDX_2:
	case CP_DRAW_INDX_BIN:
	case CP_DRAW_INDX_2_BIN:
	case CP_VIZ_QUERY:
	case CP_SET_STATE:
	case CP_SET_CONSTANT:
	case CP_IM_LOAD:
	case CP_IM_LOAD_IMMEDIATE:
	case CP_LOAD_CONSTANT_CONTEXT:
	case CP_INVALIDATE_STATE:
	case CP_SET_SHADER_BASES:
	case CP_SET_BIN_MASK:
	case CP_SET_BIN_SELECT:
	case CP_SET_BIN_BASE_OFFSET:
	case CP_SET_BIN_DATA:
	case CP_CONTEXT_UPDATE:
	case CP_INTERRUPT:
	case CP_IM_STORE:
	case CP_LOAD_STATE:
		break;
	
	case CP_ME_INIT:
	case CP_SET_PROTECTED_MODE:
	default:
		KGSL_CMD_ERR(dev_priv->device, "bad CP opcode %0x\n", opcode);
		return false;
		break;
	}

	return true;
}

static bool
_handle_type0(struct kgsl_device_private *dev_priv, uint *hostaddr)
{
	unsigned int reg = type0_pkt_offset(*hostaddr);
	unsigned int cnt = type0_pkt_size(*hostaddr);
	if (reg < 0x0192 || (reg + cnt) >= 0x8000) {
		KGSL_CMD_ERR(dev_priv->device, "bad type0 reg: 0x%0x cnt: %d\n",
			     reg, cnt);
		return false;
	}
	return true;
}

static bool _parse_ibs(struct kgsl_device_private *dev_priv,
			   uint gpuaddr, int sizedwords)
{
	static uint level; 
	bool ret = false;
	uint *hostaddr, *hoststart;
	int dwords_left = sizedwords; 
	struct kgsl_mem_entry *entry;

	entry = kgsl_sharedmem_find_region(dev_priv->process_priv,
					   gpuaddr, sizedwords * sizeof(uint));
	if (entry == NULL) {
		KGSL_CMD_ERR(dev_priv->device,
			     "no mapping for gpuaddr: 0x%08x\n", gpuaddr);
		return false;
	}

	hostaddr = (uint *)kgsl_gpuaddr_to_vaddr(&entry->memdesc, gpuaddr);
	if (hostaddr == NULL) {
		KGSL_CMD_ERR(dev_priv->device,
			     "no mapping for gpuaddr: 0x%08x\n", gpuaddr);
		return false;
	}

	hoststart = hostaddr;

	level++;

	KGSL_CMD_INFO(dev_priv->device, "ib: gpuaddr:0x%08x, wc:%d, hptr:%p\n",
		gpuaddr, sizedwords, hostaddr);

	mb();
	while (dwords_left > 0) {
		bool cur_ret = true;
		int count = 0; 

		switch (*hostaddr >> 30) {
		case 0x0: 
			count = (*hostaddr >> 16)+2;
			cur_ret = _handle_type0(dev_priv, hostaddr);
			break;
		case 0x1: 
			count = 2;
			break;
		case 0x3: 
			count = ((*hostaddr >> 16) & 0x3fff) + 2;
			cur_ret = _handle_type3(dev_priv, hostaddr);
			break;
		default:
			KGSL_CMD_ERR(dev_priv->device, "unexpected type: "
				"type:%d, word:0x%08x @ 0x%p, gpu:0x%08x\n",
				*hostaddr >> 30, *hostaddr, hostaddr,
				gpuaddr+4*(sizedwords-dwords_left));
			cur_ret = false;
			count = dwords_left;
			break;
		}

		if (!cur_ret) {
			KGSL_CMD_ERR(dev_priv->device,
				"bad sub-type: #:%d/%d, v:0x%08x"
				" @ 0x%p[gb:0x%08x], level:%d\n",
				sizedwords-dwords_left, sizedwords, *hostaddr,
				hostaddr, gpuaddr+4*(sizedwords-dwords_left),
				level);

			if (ADRENO_DEVICE(dev_priv->device)->ib_check_level
				>= 2)
				print_hex_dump(KERN_ERR,
					level == 1 ? "IB1:" : "IB2:",
					DUMP_PREFIX_OFFSET, 32, 4, hoststart,
					sizedwords*4, 0);
			goto done;
		}

		
		dwords_left -= count;
		hostaddr += count;
		if (dwords_left < 0) {
			KGSL_CMD_ERR(dev_priv->device,
				"bad count: c:%d, #:%d/%d, "
				"v:0x%08x @ 0x%p[gb:0x%08x], level:%d\n",
				count, sizedwords-(dwords_left+count),
				sizedwords, *(hostaddr-count), hostaddr-count,
				gpuaddr+4*(sizedwords-(dwords_left+count)),
				level);
			if (ADRENO_DEVICE(dev_priv->device)->ib_check_level
				>= 2)
				print_hex_dump(KERN_ERR,
					level == 1 ? "IB1:" : "IB2:",
					DUMP_PREFIX_OFFSET, 32, 4, hoststart,
					sizedwords*4, 0);
			goto done;
		}
	}

	ret = true;
done:
	if (!ret)
		KGSL_DRV_ERR(dev_priv->device,
			"parsing failed: gpuaddr:0x%08x, "
			"host:0x%p, wc:%d\n", gpuaddr, hoststart, sizedwords);

	level--;

	return ret;
}

static inline bool _ringbuffer_verify_ib(struct kgsl_device_private *dev_priv,
		struct kgsl_ibdesc *ibdesc)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	
	if (ibdesc->sizedwords == 0 || ibdesc->sizedwords > 0xFFFFF) {
		KGSL_DRV_ERR(device, "Invalid IB size 0x%X\n",
				ibdesc->sizedwords);
		return false;
	}

	if (unlikely(adreno_dev->ib_check_level >= 1) &&
		!_parse_ibs(dev_priv, ibdesc->gpuaddr, ibdesc->sizedwords)) {
		KGSL_DRV_ERR(device, "Could not verify the IBs\n");
		return false;
	}

	return true;
}

int
adreno_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
				struct kgsl_context *context,
				struct kgsl_cmdbatch *cmdbatch,
				uint32_t *timestamp)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	int i, ret;

	if (drawctxt->state == ADRENO_CONTEXT_STATE_INVALID)
		return -EDEADLK;

	

	for (i = 0; i < cmdbatch->ibcount; i++) {
		if (!_ringbuffer_verify_ib(dev_priv, &cmdbatch->ibdesc[i]))
			return -EINVAL;
	}

	
	wait_for_completion(&device->cmdbatch_gate);


	device->flags &= ~KGSL_FLAG_WAKE_ON_TOUCH;

	
	ret = adreno_dispatcher_queue_cmd(adreno_dev, drawctxt, cmdbatch,
		timestamp);

	if (ret)
		KGSL_DRV_ERR(device,
			"adreno_dispatcher_queue_cmd returned %d\n", ret);

	if (!ret && test_and_clear_bit(ADRENO_CONTEXT_FAULT, &drawctxt->priv))
		ret = -EPROTO;

	return ret;
}

unsigned int adreno_ringbuffer_get_constraint(struct kgsl_device *device,
				struct kgsl_context *context)
{
	unsigned int pwrlevel = device->pwrctrl.active_pwrlevel;

	switch (context->pwr_constraint.type) {
	case KGSL_CONSTRAINT_PWRLEVEL: {
		switch (context->pwr_constraint.sub_type) {
		case KGSL_CONSTRAINT_PWR_MAX:
			pwrlevel = device->pwrctrl.max_pwrlevel;
			break;
		case KGSL_CONSTRAINT_PWR_MIN:
			pwrlevel = device->pwrctrl.min_pwrlevel;
			break;
		default:
			break;
		}
	}
	break;

	}

	return pwrlevel;
}

void adreno_ringbuffer_set_constraint(struct kgsl_device *device,
			struct kgsl_cmdbatch *cmdbatch)
{
	unsigned int constraint;
	struct kgsl_context *context = cmdbatch->context;
	if (context->pwr_constraint.type &&
		((context->flags & KGSL_CONTEXT_PWR_CONSTRAINT) ||
			(cmdbatch->flags & KGSL_CONTEXT_PWR_CONSTRAINT))) {

		constraint = adreno_ringbuffer_get_constraint(device, context);

		if ((device->pwrctrl.constraint.type ==
			KGSL_CONSTRAINT_NONE) || (constraint <
			device->pwrctrl.constraint.hint.pwrlevel.level)) {

			kgsl_pwrctrl_pwrlevel_change(device, constraint);
			device->pwrctrl.constraint.type =
					context->pwr_constraint.type;
			device->pwrctrl.constraint.hint.
					pwrlevel.level = constraint;
		}

		device->pwrctrl.constraint.expires = jiffies +
			device->pwrctrl.interval_timeout;
	}

}

int adreno_ringbuffer_submitcmd(struct adreno_device *adreno_dev,
		struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct kgsl_ibdesc *ibdesc;
	unsigned int numibs;
	unsigned int *link;
	unsigned int *cmds;
	unsigned int i;
	struct kgsl_context *context;
	struct adreno_context *drawctxt;
	unsigned int start_index = 0;
	int flags = KGSL_CMD_FLAGS_NONE;
	int ret;

	context = cmdbatch->context;
	drawctxt = ADRENO_CONTEXT(context);

	ibdesc = cmdbatch->ibdesc;
	numibs = cmdbatch->ibcount;

	
	adreno_profile_process_results(device);

	if (test_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->priv) &&
		(!test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv))) {

		set_bit(KGSL_FT_SKIPCMD, &cmdbatch->fault_recovery);
		cmdbatch->fault_policy = drawctxt->fault_policy;
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);

		
		adreno_fault_skipcmd_detached(device, drawctxt, cmdbatch);

		
		clear_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->priv);
		drawctxt->fault_policy = 0;
	}


	if ((drawctxt->base.flags & KGSL_CONTEXT_PREAMBLE) &&
		!test_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv) &&
		(adreno_dev->drawctxt_active == drawctxt))
		start_index = 1;


	if (test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv)) {
		start_index = 0;
		numibs = 0;
	}

	cmds = link = kzalloc(sizeof(unsigned int) * (numibs * 3 + 4),
				GFP_KERNEL);
	if (!link) {
		ret = -ENOMEM;
		goto done;
	}

	if (!start_index) {
		*cmds++ = cp_nop_packet(1);
		*cmds++ = KGSL_START_OF_IB_IDENTIFIER;
	} else {
		*cmds++ = cp_nop_packet(4);
		*cmds++ = KGSL_START_OF_IB_IDENTIFIER;
		*cmds++ = CP_HDR_INDIRECT_BUFFER_PFD;
		*cmds++ = ibdesc[0].gpuaddr;
		*cmds++ = ibdesc[0].sizedwords;
	}
	for (i = start_index; i < numibs; i++) {


		if (ibdesc[i].sizedwords == 0)
			*cmds++ = cp_nop_packet(2);
		else
			*cmds++ = CP_HDR_INDIRECT_BUFFER_PFD;

		*cmds++ = ibdesc[i].gpuaddr;
		*cmds++ = ibdesc[i].sizedwords;
	}

	*cmds++ = cp_nop_packet(1);
	*cmds++ = KGSL_END_OF_IB_IDENTIFIER;

	ret = kgsl_setstate(&device->mmu, context->id,
		      kgsl_mmu_pt_get_flags(device->mmu.hwpagetable,
					device->id));

	if (ret)
		goto done;

	ret = adreno_drawctxt_switch(adreno_dev, drawctxt, cmdbatch->flags);

	if (ret)
		goto done;

	if (test_bit(CMDBATCH_FLAG_WFI, &cmdbatch->priv))
		flags = KGSL_CMD_FLAGS_WFI;


	if (test_and_clear_bit(ADRENO_DEVICE_PWRON, &adreno_dev->priv) &&
		test_bit(ADRENO_DEVICE_PWRON_FIXUP, &adreno_dev->priv))
		flags |= KGSL_CMD_FLAGS_PWRON_FIXUP;

	
	adreno_ringbuffer_set_constraint(device, cmdbatch);

	ret = adreno_ringbuffer_addcmds(&adreno_dev->ringbuffer,
					drawctxt,
					flags,
					&link[0], (cmds - link),
					cmdbatch->timestamp);

#ifdef CONFIG_MSM_KGSL_CFF_DUMP
	if (ret)
		goto done;
	ret = adreno_idle(device);
#endif

done:
	device->pwrctrl.irq_last = 0;
	kgsl_trace_issueibcmds(device, context->id, cmdbatch,
		cmdbatch->timestamp, cmdbatch->flags, ret,
		drawctxt->type);

	kfree(link);
	return ret;
}
