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
#include <linux/iio/iio.h>

#include "inv_icm42607x.h"
#include "inv_icm42607x_temp.h"

static int inv_icm42607x_temp_read(struct inv_icm42607x_state *st, s16 *temp)
{
	struct device *dev = regmap_get_device(st->map);
	__be16 *raw;
	int ret;

	pm_runtime_get_sync(dev);
	mutex_lock(&st->lock);

	ret = inv_icm42607x_set_temp_conf(st, true, NULL);
	if (ret)
		goto exit;

	raw = (__be16 *)&st->buffer[0];
	ret = regmap_bulk_read(st->map, INV_ICM42607X_REG_TEMP_DATA, raw, sizeof(*raw));
	if (ret)
		goto exit;

	*temp = (s16)be16_to_cpup(raw);
	/*
	 * Temperature data is invalid if both accel and gyro are off.
	 * Return -EBUSY in this case.
	 */
	if (*temp == INV_ICM42607X_DATA_INVALID)
		ret = -EBUSY;

exit:
	mutex_unlock(&st->lock);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

int inv_icm42607x_temp_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	s16 temp;
	int ret;

	if (chan->type != IIO_TEMP)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = inv_icm42607x_temp_read(st, &temp);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		*val = temp;
		return IIO_VAL_INT;
	/*
	 * T°C = (temp / 128) + 25
	 * Tm°C = 1000 * ((temp / 128) + 25)
	 * Tm°C = 7.8125 * temp + 25000
	 * Tm°C = (temp + 3200) * 7.8125
	 * scale: 100000 / 12800 = 7.8125
	 * offset: 3200
	 */
	case IIO_CHAN_INFO_SCALE:
		*val = 7;
		*val2 = 812500;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		*val = 3200;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}
