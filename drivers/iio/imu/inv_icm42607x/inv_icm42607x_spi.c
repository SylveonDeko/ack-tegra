// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2026 SylveonDeko
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include "inv_icm42607x.h"

static int inv_icm42607x_spi_bus_setup(struct inv_icm42607x_state *st)
{
	int ret;

	ret = regmap_set_bits(st->map, INV_ICM42607X_REG_DEVICE_CONFIG,
			      INV_ICM42607X_DEVICE_CONFIG_SPI_AP_4WIRE);
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG1,
				INV_ICM42607X_INTF_CONFIG1_I3C_DDR_EN |
				INV_ICM42607X_INTF_CONFIG1_I3C_SDR_EN);
	if (ret)
		return ret;

	/* set slew rate for SPI */
	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_DRIVE_CONFIG3,
				 INV_ICM42607X_DRIVE_CONFIG3_SPI_MASK,
				 INV_ICM42607X_DRIVE_CONFIG3_SPI(INV_ICM42607X_SLEW_RATE_INF_2NS));
	if (ret)
		return ret;

	/* disable i2c bus */
	return regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG0,
				  INV_ICM42607X_INTF_CONFIG0_UI_SIFS_CFG_MASK,
				  INV_ICM42607X_INTF_CONFIG0_UI_SIFS_CFG_I2C_DIS);
}

static int inv_icm42607x_probe(struct spi_device *spi)
{
	const void *match;
	enum inv_icm42607x_chip chip;
	struct regmap *regmap;

	match = device_get_match_data(&spi->dev);
	if (!match)
		return -EINVAL;
	chip = (uintptr_t)match;

	regmap = devm_regmap_init_spi(spi, &inv_icm42607x_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return inv_icm42607x_core_probe(regmap, chip, inv_icm42607x_spi_bus_setup);
}

/*
 * device id table is used to identify what device can be
 * supported by this driver
 */
static const struct spi_device_id inv_icm42607x_id[] = {
	{ "icm42607p", INV_CHIP_ICM42607P },
	{ "icm42670", INV_CHIP_ICM42670 },
	{ }
};
MODULE_DEVICE_TABLE(spi, inv_icm42607x_id);

static const struct of_device_id inv_icm42607x_of_matches[] = {
	{
		.compatible = "invensense,icm42607p",
		.data = (void *)INV_CHIP_ICM42607P,
	}, {
		.compatible = "invensense,icm42670",
		.data = (void *)INV_CHIP_ICM42670,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, inv_icm42607x_of_matches);

static struct spi_driver inv_icm42607x_driver = {
	.driver = {
		.name = "inv-icm42607x-spi",
		.of_match_table = inv_icm42607x_of_matches,
		.pm = pm_ptr(&inv_icm42607x_pm_ops),
	},
	.id_table = inv_icm42607x_id,
	.probe = inv_icm42607x_probe,
};
module_spi_driver(inv_icm42607x_driver);

MODULE_AUTHOR("SylveonDeko");
MODULE_DESCRIPTION("InvenSense ICM-42607x SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_ICM42607X");
