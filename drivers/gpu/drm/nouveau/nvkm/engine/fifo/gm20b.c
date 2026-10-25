/*
 * Copyright (C) 2026 SylveonDeko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include "priv.h"
#include "cgrp.h"
#include "chan.h"
#include "runl.h"

#include <nvif/class.h>

#define GM20B_FIFO_CTXSW_PERIOD_US 100000
#define GM20B_FIFO_CTXSW_TIMEOUT_MS 3000
#define GM20B_FIFO_PTIMER_REF_HZ 31250000
#define GM20B_FIFO_USERD_GP_GET 0x88

static u32
gm20b_fifo_ctxsw_period(struct nvkm_device *device)
{
	const u32 scale10x = (GM20B_FIFO_PTIMER_REF_HZ * 10ULL) /
			     (device->crystal * 1000ULL);
	const u32 timeout = GM20B_FIFO_CTXSW_PERIOD_US * 10;

	if ((timeout % scale10x) >= scale10x / 2)
		return timeout / scale10x + 1;
	return timeout / scale10x;
}

static void
gm20b_fifo_init(struct nvkm_fifo *fifo)
{
	struct nvkm_device *device = fifo->engine.subdev.device;

	if (!fifo->userd.map)
		fifo->userd.map = nvkm_kmap(fifo->userd.mem);

	gk104_fifo_init(fifo);

	nvkm_wr32(device, 0x002a0c, 0x80000000 | gm20b_fifo_ctxsw_period(device));
}

static bool
gm20b_fifo_ctxsw_progress(struct nvkm_fifo *fifo, struct nvkm_chan *chan)
{
	u32 get = ioread32_native(fifo->userd.map + chan->userd.base +
				  GM20B_FIFO_USERD_GP_GET);
	bool progress = get != chan->ctxsw.gp_get;

	chan->ctxsw.gp_get = get;
	return progress;
}

static bool
gm20b_fifo_ctxsw_expired(struct nvkm_chan *chan, bool progress)
{
	const u32 period_ms = GM20B_FIFO_CTXSW_PERIOD_US / 1000;

	chan->ctxsw.ms = progress ? period_ms : chan->ctxsw.ms + period_ms;
	return chan->ctxsw.ms > GM20B_FIFO_CTXSW_TIMEOUT_MS;
}

static bool
gm20b_fifo_ctxsw_timed_out(struct nvkm_fifo *fifo, struct nvkm_runl *runl,
			   int id, bool cgid, u32 *ms)
{
	struct nvkm_chan *chan;
	unsigned long flags;
	bool progress = false, expired = false;

	if (cgid) {
		struct nvkm_cgrp *cgrp = nvkm_runl_cgrp_get_cgid(runl, id, &flags);

		if (!cgrp)
			return false;

		nvkm_cgrp_foreach_chan(chan, cgrp)
			progress |= gm20b_fifo_ctxsw_progress(fifo, chan);
		nvkm_cgrp_foreach_chan(chan, cgrp) {
			expired |= gm20b_fifo_ctxsw_expired(chan, progress);
			*ms = max(*ms, chan->ctxsw.ms);
		}

		nvkm_cgrp_put(&cgrp, flags);
		return expired;
	}

	chan = nvkm_runl_chan_get_chid(runl, id, &flags);
	if (!chan)
		return false;

	progress = gm20b_fifo_ctxsw_progress(fifo, chan);
	expired = gm20b_fifo_ctxsw_expired(chan, progress);
	*ms = chan->ctxsw.ms;

	nvkm_chan_put(&chan, flags);
	return expired;
}

static void
gm20b_fifo_intr_ctxsw_timeout(struct nvkm_fifo *fifo, u32 engm)
{
	struct nvkm_subdev *subdev = &fifo->engine.subdev;
	struct nvkm_runl *runl;
	struct nvkm_engn *engn;
	u32 recover = 0;

	nvkm_runl_foreach(runl, fifo) {
		nvkm_runl_foreach_engn_cond(engn, runl, engm & BIT(engn->id)) {
			bool cgid;
			u32 ms = 0;
			int id = engn->func->cxid(engn, &cgid);

			if (id < 0)
				continue;

			if (gm20b_fifo_ctxsw_timed_out(fifo, runl, id, cgid, &ms)) {
				nvkm_error(subdev, "ctxsw timeout: engine %d %s %d, %u ms\n",
					   engn->id, cgid ? "cgrp" : "chan", id, ms);
				recover |= BIT(engn->id);
			} else {
				nvkm_debug(subdev, "waiting for ctxsw: engine %d %s %d, %u ms\n",
					   engn->id, cgid ? "cgrp" : "chan", id, ms);
			}
		}
	}

	if (recover)
		gf100_fifo_intr_ctxsw_timeout(fifo, recover);
}

static void
gm20b_fifo_intr_sched(struct nvkm_fifo *fifo)
{
	struct nvkm_device *device = fifo->engine.subdev.device;
	u32 code = nvkm_rd32(device, 0x00254c) & 0x000000ff;

	if (code != 0x0a) {
		gf100_fifo_intr_sched(fifo);
		return;
	}

	gf100_fifo_intr_sched_ctxsw(fifo);
}

static const struct nvkm_fifo_func
gm20b_fifo = {
	.chid_nr = gm200_fifo_chid_nr,
	.chid_ctor = gk110_fifo_chid_ctor,
	.runq_nr = gm200_fifo_runq_nr,
	.runl_ctor = gk104_fifo_runl_ctor,
	.init = gm20b_fifo_init,
	.init_pbdmas = gk104_fifo_init_pbdmas,
	.intr = gk104_fifo_intr,
	.intr_sched = gm20b_fifo_intr_sched,
	.intr_mmu_fault_unit = gm107_fifo_intr_mmu_fault_unit,
	.intr_ctxsw_timeout = gm20b_fifo_intr_ctxsw_timeout,
	.mmu_fault = &gm107_fifo_mmu_fault,
	.nonstall = &gf100_fifo_nonstall,
	.runl = &gm107_runl,
	.runq = &gk208_runq,
	.engn = &gk104_engn,
	.engn_ce = &gk104_engn_ce,
	.cgrp = {{ 0, 0,  KEPLER_CHANNEL_GROUP_A  }, &gk110_cgrp },
	.chan = {{ 0, 0, MAXWELL_CHANNEL_GPFIFO_A }, &gm107_chan },
};

int
gm20b_fifo_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	       struct nvkm_fifo **pfifo)
{
	return nvkm_fifo_new_(&gm20b_fifo, device, type, inst, pfifo);
}
