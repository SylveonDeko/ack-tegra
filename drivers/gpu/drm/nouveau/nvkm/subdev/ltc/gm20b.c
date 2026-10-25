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

#include <core/memory.h>
#include <subdev/fb.h>
#include <subdev/timer.h>

#include <linux/mm.h>

static int
gm20b_ltc_oneinit(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;
	struct nvkm_fb *fb = device->fb;
	const u32 param = nvkm_rd32(device, 0x17e280);
	const u32 tags_per_line = param & 0x0000ffff;
	const u32 line_size = 512 << ((param & 0x0f000000) >> 24);
	const u32 active = nvkm_rd32(device, 0x17e27c);
	u32 tags, size, stride;
	u64 base;
	int ret;

	ltc->ltc_nr = nvkm_rd32(device, 0x12006c);
	ltc->lts_nr = param >> 28;
	ltc->num_tags = 0;

	if (!tags_per_line || !ltc->ltc_nr || !ltc->lts_nr || !active || active > 2)
		goto mm_init;

	stride = line_size * ltc->lts_nr * max(active, ltc->ltc_nr);
	tags = min_t(u64, totalram_pages() >> (17 - PAGE_SHIFT), 0x1ffff);
	size = DIV_ROUND_UP(tags, tags_per_line) * stride;
	size += max(active, ltc->ltc_nr) << 11;
	size = ALIGN(size, 0x10000);
	tags = min_t(u64, (u64)size * tags_per_line / stride, 0x1ffff);

	ret = nvkm_memory_new(device, NVKM_MEM_TARGET_INST, size, 0x1000, true,
			      &ltc->tag_ram);
	if (ret) {
		nvkm_warn(&ltc->subdev, "no memory for compression tags: %d\n", ret);
		goto mm_init;
	}

	base = DIV_ROUND_UP_ULL(nvkm_memory_addr(ltc->tag_ram), (u64)active << 11);
	if (active == 2)
		base *= 2;

	ltc->tag_base = base;
	ltc->num_tags = tags;
	nvkm_debug(&ltc->subdev, "%u compression tags, store %08x bytes at %010llx\n",
		   tags, size, nvkm_memory_addr(ltc->tag_ram));

mm_init:
	nvkm_mm_fini(&fb->tags.mm);
	if (!ltc->num_tags)
		return nvkm_mm_init(&fb->tags.mm, 0, 0, 0, 1);
	return nvkm_mm_init(&fb->tags.mm, 0, 1, ltc->num_tags - 1, 1);
}

static void
gm20b_ltc_cbc_invalidate(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;
	int c, s;

	nvkm_mask(device, 0x17e26c, 0x00000000, 0x00000002);
	for (c = 0; c < ltc->ltc_nr; c++) {
		for (s = 0; s < ltc->lts_nr; s++) {
			const u32 addr = 0x14046c + (c * 0x2000) + (s * 0x200);
			nvkm_wait_msec(device, 2000, addr, 0x00000002, 0x00000000);
		}
	}
}

static void
gm20b_ltc_init(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;

	nvkm_wr32(device, 0x17e318, nvkm_rd32(device, 0x140518) | 0x00008000);
	nvkm_mask(device, 0x17e20c, 0x40300000, 0x00000000);
	nvkm_wr32(device, 0x17e278, ltc->tag_base);

	if (ltc->num_tags)
		gm20b_ltc_cbc_invalidate(ltc);
}

static const struct nvkm_ltc_func
gm20b_ltc = {
	.oneinit = gm20b_ltc_oneinit,
	.init = gm20b_ltc_init,
	.intr = gm107_ltc_intr,
	.cbc_clear = gm107_ltc_cbc_clear,
	.cbc_wait = gm107_ltc_cbc_wait,
	.zbc_color = 16,
	.zbc_depth = 16,
	.zbc_clear_color = gm107_ltc_zbc_clear_color,
	.zbc_clear_depth = gm107_ltc_zbc_clear_depth,
	.invalidate = gf100_ltc_invalidate,
	.flush = gf100_ltc_flush,
};

int
gm20b_ltc_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_ltc **pltc)
{
	return nvkm_ltc_new_(&gm20b_ltc, device, type, inst, pltc);
}
