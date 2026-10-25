// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2026 SylveonDeko
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>

#include "inv_icm42607x.h"
#include "inv_icm42607x_buffer.h"

static const struct regmap_range inv_icm42607x_regmap_rd_noinc_no_ranges[] = {
	regmap_reg_range(0x0000, INV_ICM42607X_REG_FIFO_DATA - 1),
	regmap_reg_range(INV_ICM42607X_REG_FIFO_DATA + 1, 0x00FF),
};

static const struct regmap_access_table inv_icm42607x_regmap_rd_noinc_accesses = {
	.no_ranges = inv_icm42607x_regmap_rd_noinc_no_ranges,
	.n_no_ranges = ARRAY_SIZE(inv_icm42607x_regmap_rd_noinc_no_ranges),
};

const struct regmap_config inv_icm42607x_regmap_config = {
	.name = "inv_icm42607x",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x00FF,
	.rd_noinc_table = &inv_icm42607x_regmap_rd_noinc_accesses,
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_regmap_config, "IIO_ICM42607X");

struct inv_icm42607x_hw {
	u8 whoami;
	const char *name;
	const struct inv_icm42607x_conf *conf;
};

/* chip initial default configuration */
static const struct inv_icm42607x_conf inv_icm42607x_default_conf = {
	.gyro = {
		.mode = INV_ICM42607X_SENSOR_MODE_OFF,
		.fs = INV_ICM42607X_GYRO_FS_1000DPS,
		.odr = INV_ICM42607X_ODR_100HZ,
		.filter = INV_ICM42607X_FILTER_BW_25HZ,
	},
	.accel = {
		.mode = INV_ICM42607X_SENSOR_MODE_OFF,
		.fs = INV_ICM42607X_ACCEL_FS_4G,
		.odr = INV_ICM42607X_ODR_100HZ,
		.filter = INV_ICM42607X_FILTER_BW_25HZ,
	},
	.temp_en = false,
};

static const struct inv_icm42607x_hw inv_icm42607x_hw[INV_CHIP_NB] = {
	[INV_CHIP_ICM42607P] = {
		.whoami = INV_ICM42607X_WHOAMI_ICM42607P,
		.name = "icm42607p",
		.conf = &inv_icm42607x_default_conf,
	},
	[INV_CHIP_ICM42670] = {
		.whoami = INV_ICM42607X_WHOAMI_ICM42670,
		.name = "icm42670",
		.conf = &inv_icm42607x_default_conf,
	},
};

const struct iio_mount_matrix *
inv_icm42607x_get_mount_matrix(const struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan)
{
	const struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);

	return &st->orientation;
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_get_mount_matrix, "IIO_ICM42607X");

u32 inv_icm42607x_odr_to_period(enum inv_icm42607x_odr odr)
{
	static const u32 odr_periods[INV_ICM42607X_ODR_NB] = {
		[INV_ICM42607X_ODR_1600HZ] = 625000,
		[INV_ICM42607X_ODR_800HZ] = 1250000,
		[INV_ICM42607X_ODR_400HZ] = 2500000,
		[INV_ICM42607X_ODR_200HZ] = 5000000,
		[INV_ICM42607X_ODR_100HZ] = 10000000,
		[INV_ICM42607X_ODR_50HZ] = 20000000,
		[INV_ICM42607X_ODR_25HZ] = 40000000,
		[INV_ICM42607X_ODR_12_5HZ] = 80000000,
		[INV_ICM42607X_ODR_6_25HZ_LP] = 160000000,
		[INV_ICM42607X_ODR_3_125HZ_LP] = 320000000,
		[INV_ICM42607X_ODR_1_5625HZ_LP] = 640000000,
	};

	return odr_periods[odr];
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_odr_to_period, "IIO_ICM42607X");

static int inv_icm42607x_set_pwr_mgmt0(struct inv_icm42607x_state *st,
				       enum inv_icm42607x_sensor_mode gyro,
				       enum inv_icm42607x_sensor_mode accel,
				       bool temp, unsigned int *sleep_ms)
{
	enum inv_icm42607x_sensor_mode oldgyro = st->conf.gyro.mode;
	enum inv_icm42607x_sensor_mode oldaccel = st->conf.accel.mode;
	bool oldtemp = st->conf.temp_en;
	unsigned int sleepval;
	unsigned int val;
	int ret;

	/* if nothing changed, exit */
	if (gyro == oldgyro && accel == oldaccel && temp == oldtemp)
		return 0;

	val = INV_ICM42607X_PWR_MGMT0_GYRO(gyro) |
	      INV_ICM42607X_PWR_MGMT0_ACCEL(accel);
	if (!temp)
		val |= INV_ICM42607X_PWR_MGMT0_ACCEL_LP_CLK_SEL;
	ret = regmap_write(st->map, INV_ICM42607X_REG_PWR_MGMT0, val);
	if (ret)
		return ret;

	st->conf.gyro.mode = gyro;
	st->conf.accel.mode = accel;
	st->conf.temp_en = temp;

	/* compute required wait time for sensors to stabilize */
	sleepval = 0;
	/* temperature stabilization time */
	if (temp && !oldtemp) {
		if (sleepval < INV_ICM42607X_TEMP_STARTUP_TIME_MS)
			sleepval = INV_ICM42607X_TEMP_STARTUP_TIME_MS;
	}
	/* accel startup time */
	if (accel != oldaccel && oldaccel == INV_ICM42607X_SENSOR_MODE_OFF) {
		/* block any register write for at least 200 µs */
		usleep_range(200, 300);
		if (sleepval < INV_ICM42607X_ACCEL_STARTUP_TIME_MS)
			sleepval = INV_ICM42607X_ACCEL_STARTUP_TIME_MS;
	}
	if (gyro != oldgyro) {
		/* gyro startup time */
		if (oldgyro == INV_ICM42607X_SENSOR_MODE_OFF) {
			/* block any register write for at least 200 µs */
			usleep_range(200, 300);
			if (sleepval < INV_ICM42607X_GYRO_STARTUP_TIME_MS)
				sleepval = INV_ICM42607X_GYRO_STARTUP_TIME_MS;
		/* gyro stop time */
		} else if (gyro == INV_ICM42607X_SENSOR_MODE_OFF) {
			if (sleepval < INV_ICM42607X_GYRO_STOP_TIME_MS)
				sleepval = INV_ICM42607X_GYRO_STOP_TIME_MS;
		}
	}

	/* deferred sleep value if sleep pointer is provided or direct sleep */
	if (sleep_ms)
		*sleep_ms = sleepval;
	else if (sleepval)
		msleep(sleepval);

	return 0;
}

int inv_icm42607x_set_accel_conf(struct inv_icm42607x_state *st,
				 struct inv_icm42607x_sensor_conf *conf,
				 unsigned int *sleep_ms)
{
	struct inv_icm42607x_sensor_conf *oldconf = &st->conf.accel;
	unsigned int val;
	int ret;

	/* Sanitize missing values with current values */
	if (conf->mode < 0)
		conf->mode = oldconf->mode;
	if (conf->fs < 0)
		conf->fs = oldconf->fs;
	if (conf->odr < 0)
		conf->odr = oldconf->odr;
	if (conf->filter < 0)
		conf->filter = oldconf->filter;

	/* force power mode against ODR when sensor is on */
	switch (conf->mode) {
	case INV_ICM42607X_SENSOR_MODE_LOW_POWER:
	case INV_ICM42607X_SENSOR_MODE_LOW_NOISE:
		if (conf->odr >= INV_ICM42607X_ODR_6_25HZ_LP) {
			conf->mode = INV_ICM42607X_SENSOR_MODE_LOW_POWER;
			conf->filter = INV_ICM42607X_FILTER_AVG_16X;
		} else {
			conf->mode = INV_ICM42607X_SENSOR_MODE_LOW_NOISE;
			conf->filter = INV_ICM42607X_FILTER_BW_25HZ;
		}
		break;
	default:
		break;
	}

	/* set ACCEL_CONFIG0 register (accel fullscale & odr) */
	if (conf->fs != oldconf->fs || conf->odr != oldconf->odr) {
		val = INV_ICM42607X_ACCEL_CONFIG0_FS_SEL(conf->fs) |
		      INV_ICM42607X_ACCEL_CONFIG0_ODR(conf->odr);
		ret = regmap_write(st->map, INV_ICM42607X_REG_ACCEL_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->fs = conf->fs;
		oldconf->odr = conf->odr;
	}

	/* set ACCEL_CONFIG1 register (accel filter or averaging) */
	if (conf->filter != oldconf->filter) {
		if (conf->mode == INV_ICM42607X_SENSOR_MODE_LOW_POWER)
			ret = regmap_update_bits(st->map,
						 INV_ICM42607X_REG_ACCEL_CONFIG1,
						 INV_ICM42607X_ACCEL_CONFIG1_AVG_MASK,
						 INV_ICM42607X_ACCEL_CONFIG1_AVG(conf->filter));
		else
			ret = regmap_update_bits(st->map,
						 INV_ICM42607X_REG_ACCEL_CONFIG1,
						 INV_ICM42607X_ACCEL_CONFIG1_FILTER_MASK,
						 INV_ICM42607X_ACCEL_CONFIG1_FILTER(conf->filter));
		if (ret)
			return ret;
		oldconf->filter = conf->filter;
	}

	/* set PWR_MGMT0 register (accel sensor mode) */
	return inv_icm42607x_set_pwr_mgmt0(st, st->conf.gyro.mode, conf->mode,
					   st->conf.temp_en, sleep_ms);
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_set_accel_conf, "IIO_ICM42607X");

int inv_icm42607x_set_gyro_conf(struct inv_icm42607x_state *st,
				struct inv_icm42607x_sensor_conf *conf,
				unsigned int *sleep_ms)
{
	struct inv_icm42607x_sensor_conf *oldconf = &st->conf.gyro;
	unsigned int val;
	int ret;

	/* sanitize missing values with current values */
	if (conf->mode < 0)
		conf->mode = oldconf->mode;
	if (conf->fs < 0)
		conf->fs = oldconf->fs;
	if (conf->odr < 0)
		conf->odr = oldconf->odr;
	if (conf->filter < 0)
		conf->filter = oldconf->filter;

	/* set GYRO_CONFIG0 register (gyro fullscale & odr) */
	if (conf->fs != oldconf->fs || conf->odr != oldconf->odr) {
		val = INV_ICM42607X_GYRO_CONFIG0_FS_SEL(conf->fs) |
		      INV_ICM42607X_GYRO_CONFIG0_ODR(conf->odr);
		ret = regmap_write(st->map, INV_ICM42607X_REG_GYRO_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->fs = conf->fs;
		oldconf->odr = conf->odr;
	}

	/* set GYRO_CONFIG1 register (gyro filter) */
	if (conf->filter != oldconf->filter) {
		val = INV_ICM42607X_GYRO_CONFIG1_FILTER(conf->filter);
		ret = regmap_update_bits(st->map, INV_ICM42607X_REG_GYRO_CONFIG1,
					 INV_ICM42607X_GYRO_CONFIG1_FILTER_MASK, val);
		if (ret)
			return ret;
		oldconf->filter = conf->filter;
	}

	/* set PWR_MGMT0 register (gyro sensor mode) */
	return inv_icm42607x_set_pwr_mgmt0(st, conf->mode, st->conf.accel.mode,
					   st->conf.temp_en, sleep_ms);
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_set_gyro_conf, "IIO_ICM42607X");

int inv_icm42607x_set_temp_conf(struct inv_icm42607x_state *st, bool enable,
				unsigned int *sleep_ms)
{
	int ret;

	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_TEMP_CONFIG0,
				 INV_ICM42607X_TEMP_CONFIG0_FILTER_MASK,
				 INV_ICM42607X_TEMP_CONFIG0_FILTER(INV_ICM42607X_FILTER_BW_34HZ));
	if (ret)
		return ret;

	return inv_icm42607x_set_pwr_mgmt0(st, st->conf.gyro.mode,
					   st->conf.accel.mode, enable,
					   sleep_ms);
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_set_temp_conf, "IIO_ICM42607X");

int inv_icm42607x_debugfs_reg(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);

	guard(mutex)(&st->lock);

	if (readval)
		return regmap_read(st->map, reg, readval);

	return regmap_write(st->map, reg, writeval);
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_debugfs_reg, "IIO_ICM42607X");

static int inv_icm42607x_set_conf(struct inv_icm42607x_state *st,
				  const struct inv_icm42607x_conf *conf)
{
	unsigned int val;
	int ret;

	/* set PWR_MGMT0 register (gyro & accel sensor mode, temp enabled) */
	val = INV_ICM42607X_PWR_MGMT0_GYRO(conf->gyro.mode) |
	      INV_ICM42607X_PWR_MGMT0_ACCEL(conf->accel.mode);
	if (!conf->temp_en)
		val |= INV_ICM42607X_PWR_MGMT0_ACCEL_LP_CLK_SEL;
	ret = regmap_write(st->map, INV_ICM42607X_REG_PWR_MGMT0, val);
	if (ret)
		return ret;

	/* set GYRO_CONFIG0 register (gyro fullscale & odr) */
	val = INV_ICM42607X_GYRO_CONFIG0_FS_SEL(conf->gyro.fs) |
	      INV_ICM42607X_GYRO_CONFIG0_ODR(conf->gyro.odr);
	ret = regmap_write(st->map, INV_ICM42607X_REG_GYRO_CONFIG0, val);
	if (ret)
		return ret;

	/* set ACCEL_CONFIG0 register (accel fullscale & odr) */
	val = INV_ICM42607X_ACCEL_CONFIG0_FS_SEL(conf->accel.fs) |
	      INV_ICM42607X_ACCEL_CONFIG0_ODR(conf->accel.odr);
	ret = regmap_write(st->map, INV_ICM42607X_REG_ACCEL_CONFIG0, val);
	if (ret)
		return ret;

	/* set GYRO_CONFIG1 register (gyro filter) */
	val = INV_ICM42607X_GYRO_CONFIG1_FILTER(conf->gyro.filter);
	ret = regmap_write(st->map, INV_ICM42607X_REG_GYRO_CONFIG1, val);
	if (ret)
		return ret;

	/* set ACCEL_CONFIG1 register (accel filter) */
	val = INV_ICM42607X_ACCEL_CONFIG1_FILTER(conf->accel.filter);
	ret = regmap_write(st->map, INV_ICM42607X_REG_ACCEL_CONFIG1, val);
	if (ret)
		return ret;

	/* update internal conf */
	st->conf = *conf;

	return 0;
}

/**
 *  inv_icm42607x_setup() - check and setup chip
 *  @st:	driver internal state
 *  @bus_setup:	callback for setting up bus specific registers
 *
 *  Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm42607x_setup(struct inv_icm42607x_state *st,
			       inv_icm42607x_bus_setup bus_setup)
{
	const struct inv_icm42607x_hw *hw;
	const struct device *dev = regmap_get_device(st->map);
	unsigned int val;
	int chip, ret;

	/* check chip self-identification value */
	ret = regmap_read(st->map, INV_ICM42607X_REG_WHOAMI, &val);
	if (ret)
		return ret;

	for (chip = INV_CHIP_INVALID + 1; chip < INV_CHIP_NB; chip++) {
		if (inv_icm42607x_hw[chip].whoami == val)
			break;
	}
	if (chip == INV_CHIP_NB) {
		dev_dbg(dev, "unsupported whoami %#02x\n", val);
		return -ENODEV;
	}
	st->chip = chip;
	hw = &inv_icm42607x_hw[chip];
	st->name = hw->name;

	/* reset to make sure previous state are not there */
	ret = regmap_write(st->map, INV_ICM42607X_REG_SIGNAL_PATH_RESET,
			   INV_ICM42607X_SIGNAL_PATH_RESET_SOFT_RESET);
	if (ret)
		return ret;
	msleep(INV_ICM42607X_RESET_TIME_MS);

	ret = regmap_read(st->map, INV_ICM42607X_REG_INT_STATUS, &val);
	if (ret)
		return ret;
	if (!(val & INV_ICM42607X_INT_STATUS_RESET_DONE)) {
		dev_err(dev, "reset error, reset done bit not set\n");
		return -ENODEV;
	}

	/* set chip bus configuration */
	ret = bus_setup(st);
	if (ret)
		return ret;

	/* sensor data in big-endian (default) */
	ret = regmap_set_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG0,
			      INV_ICM42607X_INTF_CONFIG0_SENSOR_DATA_ENDIAN);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG1,
				 INV_ICM42607X_INTF_CONFIG1_CLKSEL_MASK,
				 INV_ICM42607X_INTF_CONFIG1_CLKSEL_PLL);
	if (ret)
		return ret;

	return inv_icm42607x_set_conf(st, hw->conf);
}

static irqreturn_t inv_icm42607x_irq_timestamp(int irq, void *_data)
{
	struct inv_icm42607x_state *st = _data;

	st->timestamp.gyro = iio_get_time_ns(st->indio_gyro);
	st->timestamp.accel = iio_get_time_ns(st->indio_accel);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t inv_icm42607x_irq_handler(int irq, void *_data)
{
	struct inv_icm42607x_state *st = _data;
	struct device *dev = regmap_get_device(st->map);
	unsigned int status;
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_read(st->map, INV_ICM42607X_REG_INT_STATUS, &status);
	if (ret)
		goto out_unlock;

	/* FIFO full */
	if (status & INV_ICM42607X_INT_STATUS_FIFO_FULL)
		dev_warn(dev, "FIFO full data lost!\n");

	/* FIFO threshold reached */
	if (status & INV_ICM42607X_INT_STATUS_FIFO_THS) {
		ret = inv_icm42607x_buffer_fifo_read(st, 0);
		if (ret) {
			dev_err(dev, "FIFO read error %d\n", ret);
			goto out_unlock;
		}
		ret = inv_icm42607x_buffer_fifo_parse(st);
		if (ret)
			dev_err(dev, "FIFO parsing error %d\n", ret);
	}

out_unlock:
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}

/**
 * inv_icm42607x_irq_init() - initialize int pin and interrupt handler
 * @st:		driver internal state
 * @irq:	irq number
 * @irq_type:	irq trigger type
 * @open_drain:	true if irq is open drain, false for push-pull
 *
 * Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm42607x_irq_init(struct inv_icm42607x_state *st, int irq,
				  int irq_type, bool open_drain)
{
	struct device *dev = regmap_get_device(st->map);
	unsigned int val;
	int ret;

	/* configure INT1 interrupt: default is active low on edge */
	switch (irq_type) {
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_HIGH:
		val = INV_ICM42607X_INT_CONFIG_INT1_ACTIVE_HIGH;
		break;
	default:
		val = INV_ICM42607X_INT_CONFIG_INT1_ACTIVE_LOW;
		break;
	}

	switch (irq_type) {
	case IRQF_TRIGGER_LOW:
	case IRQF_TRIGGER_HIGH:
		val |= INV_ICM42607X_INT_CONFIG_INT1_LATCHED;
		break;
	default:
		break;
	}

	if (!open_drain)
		val |= INV_ICM42607X_INT_CONFIG_INT1_PUSH_PULL;

	ret = regmap_write(st->map, INV_ICM42607X_REG_INT_CONFIG, val);
	if (ret)
		return ret;

	irq_type |= IRQF_ONESHOT;
	return devm_request_threaded_irq(dev, irq, inv_icm42607x_irq_timestamp,
					 inv_icm42607x_irq_handler, irq_type,
					 "inv_icm42607x", st);
}

static int inv_icm42607x_enable_regulator_vddio(struct inv_icm42607x_state *st)
{
	int ret;

	ret = regulator_enable(st->vddio_supply);
	if (ret)
		return ret;

	/* wait a little for supply ramp */
	usleep_range(3000, 4000);

	return 0;
}

static void inv_icm42607x_disable_vddio_reg(void *_data)
{
	struct inv_icm42607x_state *st = _data;
	struct device *dev = regmap_get_device(st->map);

	if (pm_runtime_enabled(dev) && pm_runtime_status_suspended(dev))
		return;

	regulator_disable(st->vddio_supply);
}

int inv_icm42607x_core_probe(struct regmap *regmap, int chip,
			     inv_icm42607x_bus_setup bus_setup)
{
	struct device *dev = regmap_get_device(regmap);
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct inv_icm42607x_state *st;
	int irq, irq_type;
	bool open_drain;
	int ret;

	if (chip <= INV_CHIP_INVALID || chip >= INV_CHIP_NB) {
		dev_err(dev, "invalid chip = %d\n", chip);
		return -ENODEV;
	}

	/* get INT1 only supported interrupt or fallback to first interrupt */
	irq = fwnode_irq_get_byname(fwnode, "INT1");
	if (irq < 0 && irq != -EPROBE_DEFER) {
		dev_info(dev, "no INT1 interrupt defined, fallback to first interrupt\n");
		irq = fwnode_irq_get(fwnode, 0);
	}
	if (irq < 0)
		return dev_err_probe(dev, irq, "error missing INT1 interrupt\n");

	irq_type = irq_get_trigger_type(irq);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_FALLING;

	open_drain = device_property_read_bool(dev, "drive-open-drain");

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	dev_set_drvdata(dev, st);
	mutex_init(&st->lock);
	st->chip = chip;
	st->map = regmap;

	ret = iio_read_mount_matrix(dev, &st->orientation);
	if (ret) {
		dev_err(dev, "failed to retrieve mounting matrix %d\n", ret);
		return ret;
	}

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get vdd regulator\n");

	msleep(INV_ICM42607X_POWER_UP_TIME_MS);

	st->vddio_supply = devm_regulator_get(dev, "vddio");
	if (IS_ERR(st->vddio_supply))
		return PTR_ERR(st->vddio_supply);

	ret = inv_icm42607x_enable_regulator_vddio(st);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, inv_icm42607x_disable_vddio_reg, st);
	if (ret)
		return ret;

	/* setup chip registers */
	ret = inv_icm42607x_setup(st, bus_setup);
	if (ret)
		return ret;

	ret = inv_icm42607x_buffer_init(st);
	if (ret)
		return ret;

	st->indio_gyro = inv_icm42607x_gyro_init(st);
	if (IS_ERR(st->indio_gyro))
		return PTR_ERR(st->indio_gyro);

	st->indio_accel = inv_icm42607x_accel_init(st);
	if (IS_ERR(st->indio_accel))
		return PTR_ERR(st->indio_accel);

	ret = inv_icm42607x_irq_init(st, irq, irq_type, open_drain);
	if (ret)
		return ret;

	/* setup runtime power management */
	ret = devm_pm_runtime_set_active_enabled(dev);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, INV_ICM42607X_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	return ret;
}
EXPORT_SYMBOL_NS_GPL(inv_icm42607x_core_probe, "IIO_ICM42607X");

/*
 * Suspend saves sensors state and turns everything off.
 * Check first if runtime suspend has not already done the job.
 */
static int inv_icm42607x_suspend(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	int ret;

	guard(mutex)(&st->lock);

	st->suspended.gyro = st->conf.gyro.mode;
	st->suspended.accel = st->conf.accel.mode;
	st->suspended.temp = st->conf.temp_en;
	if (pm_runtime_suspended(dev))
		return 0;

	/* disable FIFO data streaming */
	if (st->fifo.on) {
		ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
				   INV_ICM42607X_FIFO_CONFIG1_BYPASS);
		if (ret)
			return ret;
	}

	ret = inv_icm42607x_set_pwr_mgmt0(st, INV_ICM42607X_SENSOR_MODE_OFF,
					  INV_ICM42607X_SENSOR_MODE_OFF, false,
					  NULL);
	if (ret)
		return ret;

	regulator_disable(st->vddio_supply);

	return 0;
}

/*
 * System resume gets the system back on and restores the sensors state.
 * Manually put runtime power management in system active state.
 */
static int inv_icm42607x_resume(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	struct inv_icm42607x_sensor_state *gyro_st = iio_priv(st->indio_gyro);
	struct inv_icm42607x_sensor_state *accel_st = iio_priv(st->indio_accel);
	int ret;

	guard(mutex)(&st->lock);

	if (pm_runtime_suspended(dev))
		return 0;

	ret = inv_icm42607x_enable_regulator_vddio(st);
	if (ret)
		return ret;

	/* restore sensors state */
	ret = inv_icm42607x_set_pwr_mgmt0(st, st->suspended.gyro,
					  st->suspended.accel,
					  st->suspended.temp, NULL);
	if (ret)
		return ret;

	/* restore FIFO data streaming */
	if (st->fifo.on) {
		inv_sensors_timestamp_reset(&gyro_st->ts);
		inv_sensors_timestamp_reset(&accel_st->ts);
		ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
				   INV_ICM42607X_FIFO_CONFIG1_MODE);
	}

	return ret;
}

/* Runtime suspend will turn off sensors that are enabled by iio devices. */
static int inv_icm42607x_runtime_suspend(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	int ret;

	guard(mutex)(&st->lock);

	/* disable all sensors */
	ret = inv_icm42607x_set_pwr_mgmt0(st, INV_ICM42607X_SENSOR_MODE_OFF,
					  INV_ICM42607X_SENSOR_MODE_OFF, false,
					  NULL);
	if (ret)
		return ret;

	regulator_disable(st->vddio_supply);

	return 0;
}

/* Sensors are enabled by iio devices, no need to turn them back on here. */
static int inv_icm42607x_runtime_resume(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);

	guard(mutex)(&st->lock);

	return inv_icm42607x_enable_regulator_vddio(st);
}

EXPORT_NS_GPL_DEV_PM_OPS(inv_icm42607x_pm_ops, IIO_ICM42607X) = {
	SYSTEM_SLEEP_PM_OPS(inv_icm42607x_suspend, inv_icm42607x_resume)
	RUNTIME_PM_OPS(inv_icm42607x_runtime_suspend,
		       inv_icm42607x_runtime_resume, NULL)
};

MODULE_AUTHOR("SylveonDeko");
MODULE_DESCRIPTION("InvenSense ICM-42607x core driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_INV_SENSORS_TIMESTAMP");
