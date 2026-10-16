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

#define NVJPG_TFBIF_TRANSCFG			0x1444

struct nvjpg_config {
	const char *firmware;
	unsigned int version;
	bool supports_sid;
};

struct nvjpg {
	struct falcon falcon;

	void __iomem *regs;
	struct tegra_drm_client client;
	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct devfreq *devfreq;

	/* Platform configuration */
	const struct nvjpg_config *config;
};

static inline struct nvjpg *to_nvjpg(struct tegra_drm_client *client)
{
	return container_of(client, struct nvjpg, client);
}

static inline void nvjpg_writel(struct nvjpg *nvjpg, u32 value,
				unsigned int offset)
{
	writel(value, nvjpg->regs + offset);
}

static int nvjpg_devfreq_target(struct device *dev, unsigned long *freq,
				u32 flags)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);
	int err;

	err = clk_set_rate(nvjpg->clk, *freq);
	if (err < 0)
		return err;

	*freq = clk_get_rate(nvjpg->clk);

	return 0;
}

static int nvjpg_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);

	*freq = clk_get_rate(nvjpg->clk);

	return 0;
}

static struct devfreq_dev_profile nvjpg_devfreq_profile = {
	.polling_ms = 50,
	.target = nvjpg_devfreq_target,
	.get_cur_freq = nvjpg_devfreq_get_cur_freq,
};

static int nvjpg_devfreq_init(struct nvjpg *nvjpg)
{
	unsigned long max_rate = clk_round_rate(nvjpg->clk, ULONG_MAX);
	unsigned long min_rate = clk_round_rate(nvjpg->clk, 0);
	unsigned long margin = clk_round_rate(nvjpg->clk, min_rate + 1) - min_rate;
	unsigned long rate = min_rate;
	unsigned int hw_rates = ((max_rate - min_rate) / margin) + 1;
	struct devfreq *devfreq;

	if (min_rate == max_rate) {
		dev_info(nvjpg->dev, "Only one supported clock rate, disabling devfreq\n");
		return 0;
	}

	// Only allow up to 20 rates
	if (hw_rates > 20)
		margin *= DIV_ROUND_UP(hw_rates, 20);

	while (rate < max_rate) {
		dev_pm_opp_add(nvjpg->dev, rate, 0);
		rate += margin;
	}

	// The margin step may not fall on max rate exactly, so add max rate explicitly
	dev_pm_opp_add(nvjpg->dev, max_rate, 0);

	devfreq = devm_devfreq_add_device(nvjpg->dev,
					  &nvjpg_devfreq_profile,
					  DEVFREQ_GOV_USERSPACE,
					  NULL);
	if (IS_ERR(devfreq))
		return PTR_ERR(devfreq);

	nvjpg->devfreq = devfreq;

	return 0;
}

static int nvjpg_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvjpg *nvjpg = to_nvjpg(drm);
	int err;

	err = host1x_client_iommu_attach(client);
	if (err < 0 && err != -ENODEV) {
		dev_err(nvjpg->dev, "failed to attach to domain: %d\n", err);
		return err;
	}

	nvjpg->channel = host1x_channel_request(client);
	if (!nvjpg->channel) {
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
	host1x_channel_put(nvjpg->channel);
detach:
	host1x_client_iommu_detach(client);

	return err;
}

static int nvjpg_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvjpg *nvjpg = to_nvjpg(drm);
	int err;

	/* avoid a dangling pointer just in case this disappears */
	client->dev->dma_parms = NULL;

	err = tegra_drm_unregister_client(tegra, drm);
	if (err < 0)
		return err;

	pm_runtime_dont_use_autosuspend(client->dev);
	pm_runtime_force_suspend(client->dev);

	host1x_syncpt_put(client->syncpts[0]);
	host1x_channel_put(nvjpg->channel);
	host1x_client_iommu_detach(client);

	nvjpg->channel = NULL;

	if (client->group) {
		dma_unmap_single(nvjpg->dev, nvjpg->falcon.firmware.phys,
				 nvjpg->falcon.firmware.size, DMA_TO_DEVICE);
		tegra_drm_free(tegra, nvjpg->falcon.firmware.size,
			       nvjpg->falcon.firmware.virt,
			       nvjpg->falcon.firmware.iova);
	} else {
		dma_free_coherent(nvjpg->dev, nvjpg->falcon.firmware.size,
				  nvjpg->falcon.firmware.virt,
				  nvjpg->falcon.firmware.iova);
	}

	return 0;
}

static const struct host1x_client_ops nvjpg_client_ops = {
	.init = nvjpg_init,
	.exit = nvjpg_exit,
};

static int nvjpg_load_falcon_firmware(struct nvjpg *nvjpg)
{
	struct host1x_client *client = &nvjpg->client.base;
	struct tegra_drm *tegra = nvjpg->client.drm;
	dma_addr_t iova;
	size_t size;
	void *virt;
	int err;

	if (nvjpg->falcon.firmware.virt)
		return 0;

	err = falcon_read_firmware(&nvjpg->falcon, nvjpg->config->firmware);
	if (err < 0)
		return err;

	size = nvjpg->falcon.firmware.size;

	if (!client->group) {
		virt = dma_alloc_coherent(nvjpg->dev, size, &iova, GFP_KERNEL);
		if (!virt)
			return -ENOMEM;
	} else {
		virt = tegra_drm_alloc(tegra, size, &iova);
		if (IS_ERR(virt))
			return PTR_ERR(virt);
	}

	nvjpg->falcon.firmware.virt = virt;
	nvjpg->falcon.firmware.iova = iova;

	err = falcon_load_firmware(&nvjpg->falcon);
	if (err < 0)
		goto cleanup;

	/*
	 * In this case we have received an IOVA from the shared domain, so we
	 * need to make sure to get the physical address so that the DMA API
	 * knows what memory pages to flush the cache for.
	 */
	if (client->group) {
		dma_addr_t phys;

		phys = dma_map_single(nvjpg->dev, virt, size, DMA_TO_DEVICE);

		err = dma_mapping_error(nvjpg->dev, phys);
		if (err < 0)
			goto cleanup;

		nvjpg->falcon.firmware.phys = phys;
	}

	return 0;

cleanup:
	if (!client->group)
		dma_free_coherent(nvjpg->dev, size, virt, iova);
	else
		tegra_drm_free(tegra, size, virt, iova);

	return err;
}

static int nvjpg_boot_falcon(struct nvjpg *nvjpg)
{
	u32 stream_id;
	int err;

	if (nvjpg->config->supports_sid && tegra_dev_iommu_get_stream_id(nvjpg->dev, &stream_id)) {
		u32 value;

		value = TRANSCFG_ATT(1, TRANSCFG_SID_FALCON) | TRANSCFG_ATT(0, TRANSCFG_SID_HW);
		nvjpg_writel(nvjpg, value, NVJPG_TFBIF_TRANSCFG);

		nvjpg_writel(nvjpg, stream_id, VIC_THI_STREAMID0);
		nvjpg_writel(nvjpg, stream_id, VIC_THI_STREAMID1);
	}

	err = falcon_boot(&nvjpg->falcon);
	if (err < 0)
		return err;

	err = falcon_wait_idle(&nvjpg->falcon);
	if (err < 0) {
		dev_err(nvjpg->dev, "falcon boot timed out\n");
		return err;
	}

	return 0;
}

static __maybe_unused int nvjpg_runtime_resume(struct device *dev)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);
	int err;

	err = clk_prepare_enable(nvjpg->clk);
	if (err < 0)
		return err;

	usleep_range(20, 30);

	err = nvjpg_load_falcon_firmware(nvjpg);
	if (err < 0)
		goto disable_clk;

	err = nvjpg_boot_falcon(nvjpg);
	if (err < 0)
		goto disable_clk;

	if (nvjpg->devfreq) {
		err = devfreq_resume_device(nvjpg->devfreq);
		if (err < 0)
			goto disable_clk;
	}

	return 0;

disable_clk:
	clk_disable_unprepare(nvjpg->clk);
	return err;
}

static __maybe_unused int nvjpg_runtime_suspend(struct device *dev)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);
	int err;

	if (nvjpg->devfreq) {
		err = devfreq_suspend_device(nvjpg->devfreq);
		if (err < 0)
			return err;
	}

	host1x_channel_stop(nvjpg->channel);

	clk_disable_unprepare(nvjpg->clk);

	return 0;
}

static int nvjpg_open_channel(struct tegra_drm_client *client,
			    struct tegra_drm_context *context)
{
	struct nvjpg *nvjpg = to_nvjpg(client);

	context->channel = host1x_channel_get(nvjpg->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void nvjpg_close_channel(struct tegra_drm_context *context)
{
	host1x_channel_put(context->channel);
}

static int nvjpg_can_use_memory_ctx(struct tegra_drm_client *client, bool *supported)
{
	*supported = true;

	return 0;
}

static const struct tegra_drm_client_ops nvjpg_ops = {
	.open_channel = nvjpg_open_channel,
	.close_channel = nvjpg_close_channel,
	.submit = tegra_drm_submit,
	.get_streamid_offset = tegra_drm_get_streamid_offset_thi,
	.can_use_memory_ctx = nvjpg_can_use_memory_ctx,
};

#define NVIDIA_TEGRA_210_NVJPG_FIRMWARE "nvidia/tegra210/nvjpg.bin"

static const struct nvjpg_config tegra210_nvjpg_config = {
	.firmware = NVIDIA_TEGRA_210_NVJPG_FIRMWARE,
	.version = 0x21,
	.supports_sid = false,
};

#define NVIDIA_TEGRA_186_NVJPG_FIRMWARE "nvidia/tegra186/nvjpg.bin"

static const struct nvjpg_config tegra186_nvjpg_config = {
	.firmware = NVIDIA_TEGRA_186_NVJPG_FIRMWARE,
	.version = 0x18,
	.supports_sid = true,
};

#define NVIDIA_TEGRA_194_NVJPG_FIRMWARE "nvidia/tegra194/nvjpg.bin"

static const struct nvjpg_config tegra194_nvjpg_config = {
	.firmware = NVIDIA_TEGRA_194_NVJPG_FIRMWARE,
	.version = 0x19,
	.supports_sid = true,
};

#define NVIDIA_TEGRA_234_NVJPG_FIRMWARE "nvidia/tegra234/nvjpg.bin"

static const struct nvjpg_config tegra234_nvjpg_config = {
	.firmware = NVIDIA_TEGRA_234_NVJPG_FIRMWARE,
	.version = 0x23,
	.supports_sid = true,
};

static const struct of_device_id tegra_nvjpg_of_match[] = {
	{ .compatible = "nvidia,tegra210-nvjpg", .data = &tegra210_nvjpg_config },
	{ .compatible = "nvidia,tegra186-nvjpg", .data = &tegra186_nvjpg_config },
	{ .compatible = "nvidia,tegra194-nvjpg", .data = &tegra194_nvjpg_config },
	{ .compatible = "nvidia,tegra234-nvjpg", .data = &tegra234_nvjpg_config },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_nvjpg_of_match);

static int nvjpg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct nvjpg *nvjpg;
	u32 host_class;
	int err;

	/* inherit DMA mask from host1x parent */
	err = dma_coerce_mask_and_coherent(dev, *dev->parent->dma_mask);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set DMA mask: %d\n", err);
		return err;
	}

	nvjpg = devm_kzalloc(dev, sizeof(*nvjpg), GFP_KERNEL);
	if (!nvjpg)
		return -ENOMEM;

	nvjpg->config = of_device_get_match_data(dev);

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	nvjpg->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(nvjpg->regs))
		return PTR_ERR(nvjpg->regs);

	nvjpg->clk = devm_clk_get(dev, "nvjpg");
	if (IS_ERR(nvjpg->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(nvjpg->clk);
	}

	err = clk_set_rate(nvjpg->clk, ULONG_MAX);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set clock rate\n");
		return err;
	}

	nvjpg->falcon.dev = dev;
	nvjpg->falcon.regs = nvjpg->regs;

	err = falcon_init(&nvjpg->falcon);
	if (err < 0)
		return err;

	err = of_property_read_u32(dev->of_node, "nvidia,host1x-class", &host_class);
	if (err < 0)
		host_class = HOST1X_CLASS_NVJPG;

	platform_set_drvdata(pdev, nvjpg);

	INIT_LIST_HEAD(&nvjpg->client.base.list);
	nvjpg->client.base.ops = &nvjpg_client_ops;
	nvjpg->client.base.dev = dev;
	nvjpg->client.base.class = host_class;
	nvjpg->client.base.syncpts = syncpts;
	nvjpg->client.base.num_syncpts = 1;
	nvjpg->dev = dev;

	INIT_LIST_HEAD(&nvjpg->client.list);
	nvjpg->client.version = nvjpg->config->version;
	nvjpg->client.ops = &nvjpg_ops;

	err = host1x_client_register(&nvjpg->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto exit_falcon;
	}

	err = nvjpg_devfreq_init(nvjpg);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init devfreq: %d\n", err);
		goto unregister_client;
	}

	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 500);
	devm_pm_runtime_enable(dev);

	return 0;

unregister_client:
	host1x_client_unregister(&nvjpg->client.base);
exit_falcon:
	falcon_exit(&nvjpg->falcon);

	return err;
}

static void nvjpg_remove(struct platform_device *pdev)
{
	struct nvjpg *nvjpg = platform_get_drvdata(pdev);

	host1x_client_unregister(&nvjpg->client.base);
	falcon_exit(&nvjpg->falcon);
}

static const struct dev_pm_ops nvjpg_pm_ops = {
	RUNTIME_PM_OPS(nvjpg_runtime_suspend, nvjpg_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

struct platform_driver tegra_nvjpg_driver = {
	.driver = {
		.name = "tegra-nvjpg",
		.of_match_table = tegra_nvjpg_of_match,
		.pm = &nvjpg_pm_ops
	},
	.probe = nvjpg_probe,
	.remove = nvjpg_remove,
};

#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_210_NVJPG_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_186_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_186_NVJPG_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_194_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_194_NVJPG_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_234_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_234_NVJPG_FIRMWARE);
#endif
