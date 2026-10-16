// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/dma-mapping.h>
#include <linux/host1x.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "drm.h"
#include "falcon.h"
#include "vic.h"

#define NVENC_TFBIF_TRANSCFG			0x1844

struct nvenc_config {
	const char *firmware;
	unsigned int version;
	bool supports_sid;
};

struct nvenc {
	struct falcon falcon;

	void __iomem *regs;
	struct tegra_drm_client client;
	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct devfreq *devfreq;

	/* Platform configuration */
	const struct nvenc_config *config;
};

static inline struct nvenc *to_nvenc(struct tegra_drm_client *client)
{
	return container_of(client, struct nvenc, client);
}

static inline void nvenc_writel(struct nvenc *nvenc, u32 value,
				unsigned int offset)
{
	writel(value, nvenc->regs + offset);
}

static int nvenc_devfreq_target(struct device *dev, unsigned long *freq,
				u32 flags)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	err = clk_set_rate(nvenc->clk, *freq);
	if (err < 0)
		return err;

	*freq = clk_get_rate(nvenc->clk);

	return 0;
}

static int nvenc_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);

	*freq = clk_get_rate(nvenc->clk);

	return 0;
}

static struct devfreq_dev_profile nvenc_devfreq_profile = {
	.polling_ms = 50,
	.target = nvenc_devfreq_target,
	.get_cur_freq = nvenc_devfreq_get_cur_freq,
};

static int nvenc_devfreq_init(struct nvenc *nvenc)
{
	unsigned long max_rate = clk_round_rate(nvenc->clk, ULONG_MAX);
	unsigned long min_rate = clk_round_rate(nvenc->clk, 0);
	unsigned long margin = clk_round_rate(nvenc->clk, min_rate + 1) - min_rate;
	unsigned long rate = min_rate;
	unsigned int hw_rates = ((max_rate - min_rate) / margin) + 1;
	struct devfreq *devfreq;

	if (min_rate == max_rate) {
		dev_info(nvenc->dev, "Only one supported clock rate, disabling devfreq\n");
		return 0;
	}

	// Only allow up to 20 rates
	if (hw_rates > 20)
		margin *= DIV_ROUND_UP(hw_rates, 20);

	while (rate < max_rate) {
		dev_pm_opp_add(nvenc->dev, rate, 0);
		rate += margin;
	}

	// The margin step may not fall on max rate exactly, so add max rate explicitly
	dev_pm_opp_add(nvenc->dev, max_rate, 0);

	devfreq = devm_devfreq_add_device(nvenc->dev,
					  &nvenc_devfreq_profile,
					  DEVFREQ_GOV_USERSPACE,
					  NULL);
	if (IS_ERR(devfreq))
		return PTR_ERR(devfreq);

	nvenc->devfreq = devfreq;

	return 0;
}

static int nvenc_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvenc *nvenc = to_nvenc(drm);
	int err;

	err = host1x_client_iommu_attach(client);
	if (err < 0 && err != -ENODEV) {
		dev_err(nvenc->dev, "failed to attach to domain: %d\n", err);
		return err;
	}

	nvenc->channel = host1x_channel_request(client);
	if (!nvenc->channel) {
		err = -ENOMEM;
		goto detach;
	}

	client->syncpts[0] = host1x_syncpt_request(client, 0);
	if (!client->syncpts[0]) {
		err = -ENOMEM;
		goto free_channel;
	}

	err = tegra_drm_register_client(tegra, drm);
	if (err < 0)
		goto free_syncpt;

	/*
	 * Inherit the DMA parameters (such as maximum segment size) from the
	 * parent host1x device.
	 */
	client->dev->dma_parms = client->host->dma_parms;

	return 0;

free_syncpt:
	host1x_syncpt_put(client->syncpts[0]);
free_channel:
	host1x_channel_put(nvenc->channel);
detach:
	host1x_client_iommu_detach(client);

	return err;
}

static int nvenc_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvenc *nvenc = to_nvenc(drm);
	int err;

	/* avoid a dangling pointer just in case this disappears */
	client->dev->dma_parms = NULL;

	err = tegra_drm_unregister_client(tegra, drm);
	if (err < 0)
		return err;

	pm_runtime_dont_use_autosuspend(client->dev);
	pm_runtime_force_suspend(client->dev);

	host1x_syncpt_put(client->syncpts[0]);
	host1x_channel_put(nvenc->channel);
	host1x_client_iommu_detach(client);

	nvenc->channel = NULL;

	if (client->group) {
		dma_unmap_single(nvenc->dev, nvenc->falcon.firmware.phys,
				 nvenc->falcon.firmware.size, DMA_TO_DEVICE);
		tegra_drm_free(tegra, nvenc->falcon.firmware.size,
			       nvenc->falcon.firmware.virt,
			       nvenc->falcon.firmware.iova);
	} else {
		dma_free_coherent(nvenc->dev, nvenc->falcon.firmware.size,
				  nvenc->falcon.firmware.virt,
				  nvenc->falcon.firmware.iova);
	}

	return 0;
}

static const struct host1x_client_ops nvenc_client_ops = {
	.init = nvenc_init,
	.exit = nvenc_exit,
};

static int nvenc_load_falcon_firmware(struct nvenc *nvenc)
{
	struct host1x_client *client = &nvenc->client.base;
	struct tegra_drm *tegra = nvenc->client.drm;
	dma_addr_t iova;
	size_t size;
	void *virt;
	int err;

	if (nvenc->falcon.firmware.virt)
		return 0;

	err = falcon_read_firmware(&nvenc->falcon, nvenc->config->firmware);
	if (err < 0)
		return err;

	size = nvenc->falcon.firmware.size;

	if (!client->group) {
		virt = dma_alloc_coherent(nvenc->dev, size, &iova, GFP_KERNEL);
		if (!virt)
			return -ENOMEM;
	} else {
		virt = tegra_drm_alloc(tegra, size, &iova);
		if (IS_ERR(virt))
			return PTR_ERR(virt);
	}

	nvenc->falcon.firmware.virt = virt;
	nvenc->falcon.firmware.iova = iova;

	err = falcon_load_firmware(&nvenc->falcon);
	if (err < 0)
		goto cleanup;

	/*
	 * In this case we have received an IOVA from the shared domain, so we
	 * need to make sure to get the physical address so that the DMA API
	 * knows what memory pages to flush the cache for.
	 */
	if (client->group) {
		dma_addr_t phys;

		phys = dma_map_single(nvenc->dev, virt, size, DMA_TO_DEVICE);

		err = dma_mapping_error(nvenc->dev, phys);
		if (err < 0)
			goto cleanup;

		nvenc->falcon.firmware.phys = phys;
	}

	return 0;

cleanup:
	if (!client->group)
		dma_free_coherent(nvenc->dev, size, virt, iova);
	else
		tegra_drm_free(tegra, size, virt, iova);

	return err;
}

static int nvenc_boot_falcon(struct nvenc *nvenc)
{
	u32 stream_id;
	int err;

	if (nvenc->config->supports_sid && tegra_dev_iommu_get_stream_id(nvenc->dev, &stream_id)) {
		u32 value;

		value = TRANSCFG_ATT(1, TRANSCFG_SID_FALCON) | TRANSCFG_ATT(0, TRANSCFG_SID_HW);
		nvenc_writel(nvenc, value, NVENC_TFBIF_TRANSCFG);

		nvenc_writel(nvenc, stream_id, VIC_THI_STREAMID0);
		nvenc_writel(nvenc, stream_id, VIC_THI_STREAMID1);
	}

	err = falcon_boot(&nvenc->falcon);
	if (err < 0)
		return err;

	err = falcon_wait_idle(&nvenc->falcon);
	if (err < 0) {
		dev_err(nvenc->dev, "falcon boot timed out\n");
		return err;
	}

	return 0;
}

static __maybe_unused int nvenc_runtime_resume(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	err = clk_prepare_enable(nvenc->clk);
	if (err < 0)
		return err;

	usleep_range(20, 30);

	err = nvenc_load_falcon_firmware(nvenc);
	if (err < 0)
		goto disable_clk;

	err = nvenc_boot_falcon(nvenc);
	if (err < 0)
		goto disable_clk;

	if (nvenc->devfreq) {
		err = devfreq_resume_device(nvenc->devfreq);
		if (err < 0)
			goto disable_clk;
	}

	return 0;

disable_clk:
	clk_disable_unprepare(nvenc->clk);
	return err;
}

static __maybe_unused int nvenc_runtime_suspend(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	if (nvenc->devfreq) {
		err = devfreq_suspend_device(nvenc->devfreq);
		if (err < 0)
			return err;
	}

	host1x_channel_stop(nvenc->channel);

	clk_disable_unprepare(nvenc->clk);

	return 0;
}

static int nvenc_open_channel(struct tegra_drm_client *client,
			    struct tegra_drm_context *context)
{
	struct nvenc *nvenc = to_nvenc(client);

	context->channel = host1x_channel_get(nvenc->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void nvenc_close_channel(struct tegra_drm_context *context)
{
	host1x_channel_put(context->channel);
}

static int nvenc_can_use_memory_ctx(struct tegra_drm_client *client, bool *supported)
{
	*supported = true;

	return 0;
}

static const struct tegra_drm_client_ops nvenc_ops = {
	.open_channel = nvenc_open_channel,
	.close_channel = nvenc_close_channel,
	.submit = tegra_drm_submit,
	.get_streamid_offset = tegra_drm_get_streamid_offset_thi,
	.can_use_memory_ctx = nvenc_can_use_memory_ctx,
};

#define NVIDIA_TEGRA_210_NVENC_FIRMWARE "nvidia/tegra210/nvenc.bin"

static const struct nvenc_config nvenc_t210_config = {
	.firmware = NVIDIA_TEGRA_210_NVENC_FIRMWARE,
	.version = 0x21,
	.supports_sid = false,
};

#define NVIDIA_TEGRA_186_NVENC_FIRMWARE "nvidia/tegra186/nvenc.bin"

static const struct nvenc_config nvenc_t186_config = {
	.firmware = NVIDIA_TEGRA_186_NVENC_FIRMWARE,
	.version = 0x18,
	.supports_sid = true,
};

#define NVIDIA_TEGRA_194_NVENC_FIRMWARE "nvidia/tegra194/nvenc.bin"

static const struct nvenc_config nvenc_t194_config = {
	.firmware = NVIDIA_TEGRA_194_NVENC_FIRMWARE,
	.version = 0x19,
	.supports_sid = true,
};

#define NVIDIA_TEGRA_234_NVENC_FIRMWARE "nvidia/tegra234/nvenc.bin"

static const struct nvenc_config nvenc_t234_config = {
	.firmware = NVIDIA_TEGRA_234_NVENC_FIRMWARE,
	.version = 0x23,
	.supports_sid = true,
};

static const struct of_device_id tegra_nvenc_of_match[] = {
	{ .compatible = "nvidia,tegra210-nvenc", .data = &nvenc_t210_config },
	{ .compatible = "nvidia,tegra186-nvenc", .data = &nvenc_t186_config },
	{ .compatible = "nvidia,tegra194-nvenc", .data = &nvenc_t194_config },
	{ .compatible = "nvidia,tegra234-nvenc", .data = &nvenc_t234_config },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_nvenc_of_match);

static int nvenc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct nvenc *nvenc;
	u32 host_class;
	int err;

	/* inherit DMA mask from host1x parent */
	err = dma_coerce_mask_and_coherent(dev, *dev->parent->dma_mask);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set DMA mask: %d\n", err);
		return err;
	}

	nvenc = devm_kzalloc(dev, sizeof(*nvenc), GFP_KERNEL);
	if (!nvenc)
		return -ENOMEM;

	nvenc->config = of_device_get_match_data(dev);

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	nvenc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(nvenc->regs))
		return PTR_ERR(nvenc->regs);

	nvenc->clk = devm_clk_get(dev, "nvenc");
	if (IS_ERR(nvenc->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(nvenc->clk);
	}

	err = clk_set_rate(nvenc->clk, ULONG_MAX);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set clock rate\n");
		return err;
	}

	nvenc->falcon.dev = dev;
	nvenc->falcon.regs = nvenc->regs;

	err = falcon_init(&nvenc->falcon);
	if (err < 0)
		return err;

	err = of_property_read_u32(dev->of_node, "nvidia,host1x-class", &host_class);
	if (err < 0)
		host_class = HOST1X_CLASS_NVENC;

	platform_set_drvdata(pdev, nvenc);

	INIT_LIST_HEAD(&nvenc->client.base.list);
	nvenc->client.base.ops = &nvenc_client_ops;
	nvenc->client.base.dev = dev;
	nvenc->client.base.class = host_class;
	nvenc->client.base.syncpts = syncpts;
	nvenc->client.base.num_syncpts = 1;
	nvenc->dev = dev;

	INIT_LIST_HEAD(&nvenc->client.list);
	nvenc->client.version = nvenc->config->version;
	nvenc->client.ops = &nvenc_ops;

	err = host1x_client_register(&nvenc->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto exit_falcon;
	}

	err = nvenc_devfreq_init(nvenc);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init devfreq: %d\n", err);
		goto unregister_client;
	}

	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 500);
	devm_pm_runtime_enable(dev);

	return 0;

unregister_client:
	host1x_client_unregister(&nvenc->client.base);
exit_falcon:
	falcon_exit(&nvenc->falcon);

	return err;
}

static void nvenc_remove(struct platform_device *pdev)
{
	struct nvenc *nvenc = platform_get_drvdata(pdev);

	host1x_client_unregister(&nvenc->client.base);
	falcon_exit(&nvenc->falcon);
}

static const struct dev_pm_ops nvenc_pm_ops = {
	RUNTIME_PM_OPS(nvenc_runtime_suspend, nvenc_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

struct platform_driver tegra_nvenc_driver = {
	.driver = {
		.name = "tegra-nvenc",
		.of_match_table = tegra_nvenc_of_match,
		.pm = &nvenc_pm_ops
	},
	.probe = nvenc_probe,
	.remove = nvenc_remove,
};

#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_210_NVENC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_186_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_186_NVENC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_194_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_194_NVENC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_234_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_234_NVENC_FIRMWARE);
#endif
