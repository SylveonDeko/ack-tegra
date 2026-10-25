// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2026 SylveonDeko
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include <linux/iio/buffer.h>
#include <linux/iio/common/inv_sensors_timestamp.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#include "inv_icm42607x.h"
#include "inv_icm42607x_temp.h"
#include "inv_icm42607x_buffer.h"

#define INV_ICM42607X_ACCEL_CHAN(_modifier, _index, _ext_info)		\
	{								\
		.type = IIO_ACCEL,					\
		.modified = 1,						\
		.channel2 = _modifier,					\
		.info_mask_separate =					\
			BIT(IIO_CHAN_INFO_RAW),				\
		.info_mask_shared_by_type =				\
			BIT(IIO_CHAN_INFO_SCALE),			\
		.info_mask_shared_by_type_available =			\
			BIT(IIO_CHAN_INFO_SCALE),			\
		.info_mask_shared_by_all =				\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.info_mask_shared_by_all_available =			\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.scan_index = _index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.endianness = IIO_BE,				\
		},							\
		.ext_info = _ext_info,					\
	}

enum inv_icm42607x_accel_scan {
	INV_ICM42607X_ACCEL_SCAN_X,
	INV_ICM42607X_ACCEL_SCAN_Y,
	INV_ICM42607X_ACCEL_SCAN_Z,
	INV_ICM42607X_ACCEL_SCAN_TEMP,
	INV_ICM42607X_ACCEL_SCAN_TIMESTAMP,
};

static const struct iio_chan_spec_ext_info inv_icm42607x_accel_ext_infos[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_ALL, inv_icm42607x_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec inv_icm42607x_accel_channels[] = {
	INV_ICM42607X_ACCEL_CHAN(IIO_MOD_X, INV_ICM42607X_ACCEL_SCAN_X,
				 inv_icm42607x_accel_ext_infos),
	INV_ICM42607X_ACCEL_CHAN(IIO_MOD_Y, INV_ICM42607X_ACCEL_SCAN_Y,
				 inv_icm42607x_accel_ext_infos),
	INV_ICM42607X_ACCEL_CHAN(IIO_MOD_Z, INV_ICM42607X_ACCEL_SCAN_Z,
				 inv_icm42607x_accel_ext_infos),
	INV_ICM42607X_TEMP_CHAN(INV_ICM42607X_ACCEL_SCAN_TEMP),
	IIO_CHAN_SOFT_TIMESTAMP(INV_ICM42607X_ACCEL_SCAN_TIMESTAMP),
};

/*
 * IIO buffer data: size must be a power of 2 and timestamp aligned
 * 16 bytes: 6 bytes acceleration, 2 bytes temperature, 8 bytes timestamp
 */
struct inv_icm42607x_accel_buffer {
	struct inv_icm42607x_fifo_sensor_data accel;
	s16 temp;
	aligned_s64 timestamp;
};

#define INV_ICM42607X_SCAN_MASK_ACCEL_3AXIS				\
	(BIT(INV_ICM42607X_ACCEL_SCAN_X) |				\
	BIT(INV_ICM42607X_ACCEL_SCAN_Y) |				\
	BIT(INV_ICM42607X_ACCEL_SCAN_Z))

#define INV_ICM42607X_SCAN_MASK_TEMP	BIT(INV_ICM42607X_ACCEL_SCAN_TEMP)

static const unsigned long inv_icm42607x_accel_scan_masks[] = {
	/* 3-axis accel + temperature */
	INV_ICM42607X_SCAN_MASK_ACCEL_3AXIS | INV_ICM42607X_SCAN_MASK_TEMP,
	0,
};

/* enable accelerometer sensor and FIFO write */
static int inv_icm42607x_accel_update_scan_mode(struct iio_dev *indio_dev,
						const unsigned long *scan_mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	unsigned int fifo_en = 0;
	unsigned int sleep_temp = 0;
	unsigned int sleep_accel = 0;
	unsigned int sleep;
	int ret;

	mutex_lock(&st->lock);

	if (*scan_mask & INV_ICM42607X_SCAN_MASK_TEMP) {
		/* enable temp sensor */
		ret = inv_icm42607x_set_temp_conf(st, true, &sleep_temp);
		if (ret)
			goto out_unlock;
		fifo_en |= INV_ICM42607X_SENSOR_TEMP;
	}

	if (*scan_mask & INV_ICM42607X_SCAN_MASK_ACCEL_3AXIS) {
		/* enable accel sensor */
		conf.mode = INV_ICM42607X_SENSOR_MODE_LOW_NOISE;
		ret = inv_icm42607x_set_accel_conf(st, &conf, &sleep_accel);
		if (ret)
			goto out_unlock;
		fifo_en |= INV_ICM42607X_SENSOR_ACCEL;
	}

	/* update data FIFO write */
	inv_icm42607x_buffer_set_fifo_en(st, fifo_en | st->fifo.en);

out_unlock:
	mutex_unlock(&st->lock);
	/* sleep maximum required time */
	sleep = max(sleep_accel, sleep_temp);
	if (sleep)
		msleep(sleep);
	return ret;
}

static int inv_icm42607x_accel_read_sensor(struct inv_icm42607x_state *st,
					   struct iio_chan_spec const *chan,
					   s16 *val)
{
	struct device *dev = regmap_get_device(st->map);
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	unsigned int reg;
	__be16 *data;
	int ret;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (chan->channel2) {
	case IIO_MOD_X:
		reg = INV_ICM42607X_REG_ACCEL_DATA_X;
		break;
	case IIO_MOD_Y:
		reg = INV_ICM42607X_REG_ACCEL_DATA_Y;
		break;
	case IIO_MOD_Z:
		reg = INV_ICM42607X_REG_ACCEL_DATA_Z;
		break;
	default:
		return -EINVAL;
	}

	pm_runtime_get_sync(dev);
	mutex_lock(&st->lock);

	/* enable accel sensor */
	conf.mode = INV_ICM42607X_SENSOR_MODE_LOW_NOISE;
	ret = inv_icm42607x_set_accel_conf(st, &conf, NULL);
	if (ret)
		goto exit;

	/* read accel register data */
	data = (__be16 *)&st->buffer[0];
	ret = regmap_bulk_read(st->map, reg, data, sizeof(*data));
	if (ret)
		goto exit;

	*val = (s16)be16_to_cpup(data);
	if (*val == INV_ICM42607X_DATA_INVALID)
		ret = -EINVAL;
exit:
	mutex_unlock(&st->lock);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

/* IIO format int + nano */
static const int inv_icm42607x_accel_scale[] = {
	/* +/- 16G => 0.004788403 m/s-2 */
	[2 * INV_ICM42607X_ACCEL_FS_16G] = 0,
	[2 * INV_ICM42607X_ACCEL_FS_16G + 1] = 4788403,
	/* +/- 8G => 0.002394202 m/s-2 */
	[2 * INV_ICM42607X_ACCEL_FS_8G] = 0,
	[2 * INV_ICM42607X_ACCEL_FS_8G + 1] = 2394202,
	/* +/- 4G => 0.001197101 m/s-2 */
	[2 * INV_ICM42607X_ACCEL_FS_4G] = 0,
	[2 * INV_ICM42607X_ACCEL_FS_4G + 1] = 1197101,
	/* +/- 2G => 0.000598550 m/s-2 */
	[2 * INV_ICM42607X_ACCEL_FS_2G] = 0,
	[2 * INV_ICM42607X_ACCEL_FS_2G + 1] = 598550,
};

static int inv_icm42607x_accel_read_scale(struct inv_icm42607x_state *st,
					  int *val, int *val2)
{
	unsigned int idx;

	idx = st->conf.accel.fs;

	*val = inv_icm42607x_accel_scale[2 * idx];
	*val2 = inv_icm42607x_accel_scale[2 * idx + 1];
	return IIO_VAL_INT_PLUS_NANO;
}

static int inv_icm42607x_accel_write_scale(struct inv_icm42607x_state *st,
					   int val, int val2)
{
	struct device *dev = regmap_get_device(st->map);
	unsigned int idx;
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	int ret;

	for (idx = 0; idx < ARRAY_SIZE(inv_icm42607x_accel_scale); idx += 2) {
		if (val == inv_icm42607x_accel_scale[idx] &&
		    val2 == inv_icm42607x_accel_scale[idx + 1])
			break;
	}
	if (idx >= ARRAY_SIZE(inv_icm42607x_accel_scale))
		return -EINVAL;

	conf.fs = idx / 2;

	pm_runtime_get_sync(dev);

	scoped_guard(mutex, &st->lock)
		ret = inv_icm42607x_set_accel_conf(st, &conf, NULL);

	pm_runtime_put_autosuspend(dev);

	return ret;
}

/* IIO format int + micro */
static const int inv_icm42607x_accel_odr[] = {
	/* 1.5625Hz */
	1, 562500,
	/* 3.125Hz */
	3, 125000,
	/* 6.25Hz */
	6, 250000,
	/* 12.5Hz */
	12, 500000,
	/* 25Hz */
	25, 0,
	/* 50Hz */
	50, 0,
	/* 100Hz */
	100, 0,
	/* 200Hz */
	200, 0,
	/* 400Hz */
	400, 0,
	/* 800Hz */
	800, 0,
	/* 1600Hz */
	1600, 0,
};

static const int inv_icm42607x_accel_odr_conv[] = {
	INV_ICM42607X_ODR_1_5625HZ_LP,
	INV_ICM42607X_ODR_3_125HZ_LP,
	INV_ICM42607X_ODR_6_25HZ_LP,
	INV_ICM42607X_ODR_12_5HZ,
	INV_ICM42607X_ODR_25HZ,
	INV_ICM42607X_ODR_50HZ,
	INV_ICM42607X_ODR_100HZ,
	INV_ICM42607X_ODR_200HZ,
	INV_ICM42607X_ODR_400HZ,
	INV_ICM42607X_ODR_800HZ,
	INV_ICM42607X_ODR_1600HZ,
};

static int inv_icm42607x_accel_read_odr(struct inv_icm42607x_state *st,
					int *val, int *val2)
{
	unsigned int odr;
	unsigned int i;

	odr = st->conf.accel.odr;

	for (i = 0; i < ARRAY_SIZE(inv_icm42607x_accel_odr_conv); ++i) {
		if (inv_icm42607x_accel_odr_conv[i] == odr)
			break;
	}
	if (i >= ARRAY_SIZE(inv_icm42607x_accel_odr_conv))
		return -EINVAL;

	*val = inv_icm42607x_accel_odr[2 * i];
	*val2 = inv_icm42607x_accel_odr[2 * i + 1];

	return IIO_VAL_INT_PLUS_MICRO;
}

static int inv_icm42607x_accel_write_odr(struct iio_dev *indio_dev,
					 int val, int val2)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct inv_icm42607x_sensor_state *accel_st = iio_priv(indio_dev);
	struct inv_sensors_timestamp *ts = &accel_st->ts;
	struct device *dev = regmap_get_device(st->map);
	unsigned int idx;
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	int ret;

	for (idx = 0; idx < ARRAY_SIZE(inv_icm42607x_accel_odr); idx += 2) {
		if (val == inv_icm42607x_accel_odr[idx] &&
		    val2 == inv_icm42607x_accel_odr[idx + 1])
			break;
	}
	if (idx >= ARRAY_SIZE(inv_icm42607x_accel_odr))
		return -EINVAL;

	conf.odr = inv_icm42607x_accel_odr_conv[idx / 2];
	if (conf.odr == st->conf.accel.odr)
		return 0;

	pm_runtime_get_sync(dev);
	mutex_lock(&st->lock);

	ret = inv_sensors_timestamp_update_odr(ts, inv_icm42607x_odr_to_period(conf.odr),
					       iio_buffer_enabled(indio_dev));
	if (ret)
		goto out_unlock;

	ret = inv_icm42607x_set_accel_conf(st, &conf, NULL);
	if (ret)
		goto out_unlock;
	inv_icm42607x_buffer_update_fifo_period(st);
	inv_icm42607x_buffer_update_watermark(st);

out_unlock:
	mutex_unlock(&st->lock);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int inv_icm42607x_accel_read_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int *val, int *val2, long mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	s16 data;
	int ret;

	switch (chan->type) {
	case IIO_ACCEL:
		break;
	case IIO_TEMP:
		return inv_icm42607x_temp_read_raw(indio_dev, chan, val, val2, mask);
	default:
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = inv_icm42607x_accel_read_sensor(st, chan, &data);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		*val = data;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return inv_icm42607x_accel_read_scale(st, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return inv_icm42607x_accel_read_odr(st, val, val2);
	default:
		return -EINVAL;
	}
}

static int inv_icm42607x_accel_read_avail(struct iio_dev *indio_dev,
					  struct iio_chan_spec const *chan,
					  const int **vals,
					  int *type, int *length, long mask)
{
	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = inv_icm42607x_accel_scale;
		*type = IIO_VAL_INT_PLUS_NANO;
		*length = ARRAY_SIZE(inv_icm42607x_accel_scale);
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = inv_icm42607x_accel_odr;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(inv_icm42607x_accel_odr);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int inv_icm42607x_accel_write_raw(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 int val, int val2, long mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = inv_icm42607x_accel_write_scale(st, val, val2);
		iio_device_release_direct(indio_dev);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return inv_icm42607x_accel_write_odr(indio_dev, val, val2);
	default:
		return -EINVAL;
	}
}

static int inv_icm42607x_accel_write_raw_get_fmt(struct iio_dev *indio_dev,
						 struct iio_chan_spec const *chan,
						 long mask)
{
	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int inv_icm42607x_accel_hwfifo_set_watermark(struct iio_dev *indio_dev,
						    unsigned int val)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);

	guard(mutex)(&st->lock);

	st->fifo.watermark.accel = val;
	return inv_icm42607x_buffer_update_watermark(st);
}

static int inv_icm42607x_accel_hwfifo_flush(struct iio_dev *indio_dev,
					    unsigned int count)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	if (count == 0)
		return 0;

	guard(mutex)(&st->lock);

	ret = inv_icm42607x_buffer_hwfifo_flush(st, count);
	if (ret)
		return ret;

	return st->fifo.nb.accel;
}

static const struct iio_info inv_icm42607x_accel_info = {
	.read_raw = inv_icm42607x_accel_read_raw,
	.read_avail = inv_icm42607x_accel_read_avail,
	.write_raw = inv_icm42607x_accel_write_raw,
	.write_raw_get_fmt = inv_icm42607x_accel_write_raw_get_fmt,
	.debugfs_reg_access = inv_icm42607x_debugfs_reg,
	.update_scan_mode = inv_icm42607x_accel_update_scan_mode,
	.hwfifo_set_watermark = inv_icm42607x_accel_hwfifo_set_watermark,
	.hwfifo_flush_to_buffer = inv_icm42607x_accel_hwfifo_flush,
};

struct iio_dev *inv_icm42607x_accel_init(struct inv_icm42607x_state *st)
{
	struct device *dev = regmap_get_device(st->map);
	const char *name;
	struct inv_icm42607x_sensor_state *accel_st;
	struct inv_sensors_timestamp_chip ts_chip;
	struct iio_dev *indio_dev;
	int ret;

	name = devm_kasprintf(dev, GFP_KERNEL, "%s-accel", st->name);
	if (!name)
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*accel_st));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);
	accel_st = iio_priv(indio_dev);

	/*
	 * clock period is 32kHz (31250ns)
	 * jitter is +/- 2% (20 per mille)
	 */
	ts_chip.clock_period = 31250;
	ts_chip.jitter = 20;
	ts_chip.init_period = inv_icm42607x_odr_to_period(st->conf.accel.odr);
	inv_sensors_timestamp_init(&accel_st->ts, &ts_chip);

	iio_device_set_drvdata(indio_dev, st);
	indio_dev->name = name;
	indio_dev->info = &inv_icm42607x_accel_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm42607x_accel_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm42607x_accel_channels);
	indio_dev->available_scan_masks = inv_icm42607x_accel_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  &inv_icm42607x_buffer_ops);
	if (ret)
		return ERR_PTR(ret);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ERR_PTR(ret);

	return indio_dev;
}

int inv_icm42607x_accel_parse_fifo(struct iio_dev *indio_dev)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct inv_icm42607x_sensor_state *accel_st = iio_priv(indio_dev);
	struct inv_sensors_timestamp *ts = &accel_st->ts;
	ssize_t i, size;
	unsigned int no;
	const void *accel, *gyro, *timestamp;
	const s8 *temp;
	unsigned int odr;
	s64 ts_val;
	/* buffer is copied to userspace, zeroing it to avoid any data leak */
	struct inv_icm42607x_accel_buffer buffer = { };

	/* parse all fifo packets */
	for (i = 0, no = 0; i < st->fifo.count; i += size, ++no) {
		size = inv_icm42607x_fifo_decode_packet(&st->fifo.data[i],
							&accel, &gyro, &temp, &timestamp, &odr);
		/* quit if error or FIFO is empty */
		if (size <= 0)
			return size;

		/* skip packet if no accel data or data is invalid */
		if (!accel || !inv_icm42607x_fifo_is_data_valid(accel))
			continue;

		/* update odr */
		if (odr & INV_ICM42607X_SENSOR_ACCEL)
			inv_sensors_timestamp_apply_odr(ts, st->fifo.period,
							st->fifo.nb.total, no);

		memcpy(&buffer.accel, accel, sizeof(buffer.accel));
		/* convert 8 bits FIFO temperature in high resolution format */
		buffer.temp = temp ? (*temp * 64) : 0;
		ts_val = inv_sensors_timestamp_pop(ts);
		iio_push_to_buffers_with_timestamp(indio_dev, &buffer, ts_val);
	}

	return 0;
}
