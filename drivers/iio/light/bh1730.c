// SPDX-License-Identifier: GPL-2.0
/*
 * ROHM BH1730 ambient light sensor driver
 *
 * Copyright (c) 2021 CTCaer <ctcaer@gmail.com>
 *
 * Based on previous iio BH1730FVC drivers from:
 * Copyright (c) 2018 Google, Inc.
 * Author: Pierre Bourdon <delroth@google.com>
 *
 * Copyright (C) 2012 Samsung Electronics. All rights reserved.
 * Author: Won Huh <won.huh@samsung.com>
 *
 * Data sheets:
 *  http://www.rohm.com/web/global/datasheet/BH1730FVC/bh1730fvc-e
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define BH1730_NAME	"bh1730fvc"

#define BH1730_CMD_BIT BIT(7)

#define BH1730_REG_CONTROL	0x00
#define BH1730_REG_TIMING	0x01
#define BH1730_REG_GAIN		0x07
#define BH1730_REG_DATA0LOW	0x14
#define BH1730_REG_DATA1LOW	0x16

#define BH1730_CONTROL_POWER		BIT(0)
#define BH1730_CONTROL_ADC_EN		BIT(1)

#define BH1730_ADC_CALC_DELAY_US	2000 /* BH1730_INTERNAL_CLOCK_MS * 714 */
#define BH1730_ITIME_TO_US		2700 /* BH1730_INTERNAL_CLOCK_MS * 964 */

#define BH1730_DEFAULT_INTEG_CYCLE	38
#define BH1730_DEFAULT_ITIME_MS		100

#define BH1730_POWER_ON_DELAY_US	10000

#define BH1730_MAX_MEASURED_LUX		100000

enum bh1730_gain {
	BH1730_GAIN_1X = 0,
	BH1730_GAIN_2X,
	BH1730_GAIN_64X,
	BH1730_GAIN_128X,
};

struct gain_coeff_t {
	u32 cl;
	u32 fl;
};

struct opt_win_coeff_t {
	u32 rc;
	u32 cv;
	u32 ci;
};

struct lux_cal_data_t {
	const struct gain_coeff_t *gain_coeff;
	const struct opt_win_coeff_t *opt_win_coeff;
	u32 opt_win_coeff_count;
	u32 itime_cycle;
	u32 mul;
};

/*
 * No optical window or optical window that has flat transmittance
 * from visible light to infrared light.
 */
static const struct opt_win_coeff_t def_lux_coeff[] = {
	{  260, 1290, 2733 },
	{  550,  795,  859 },
	{ 1090,  510,  345 },
	{ 2130,  276,  130 }
};

static const struct gain_coeff_t def_gain_coeff[] = {
	{ 3000,    -1 }, /* 1x */
	{ 2000,  9800 }, /* 2x */
	{   15, 60000 }, /* 64x */
	{    0,  1300 }  /* 128x */
};

static const struct lux_cal_data_t def_lux_data = {
	.gain_coeff = def_gain_coeff,
	.opt_win_coeff = def_lux_coeff,
	.opt_win_coeff_count = ARRAY_SIZE(def_lux_coeff),
	.itime_cycle = BH1730_DEFAULT_INTEG_CYCLE,
	.mul = 1000
};

struct bh1730_data {
	struct i2c_client *client;
	struct lux_cal_data_t cal;
	struct regulator *reg_vdd;
	struct regulator *reg_vid;
	enum bh1730_gain gain;
	bool powered;
	u32 itime_us;
	u32 tmt_us;
};

static int bh1730_read_word(struct bh1730_data *bh1730, u8 reg)
{
	int ret = i2c_smbus_read_word_data(bh1730->client,
					   BH1730_CMD_BIT | reg);
	if (ret < 0)
		dev_err(&bh1730->client->dev,
			"i2c read failed error %d, register %01x\n",
			ret, reg);

	return ret;
}

static int bh1730_write(struct bh1730_data *bh1730, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(bh1730->client,
					    BH1730_CMD_BIT | reg,
					    val);
	if (ret < 0)
		dev_err(&bh1730->client->dev,
			"i2c write failed error %d, register %01x\n",
			ret, reg);

	return ret;
}

static int bh1730_gain_multiplier(struct bh1730_data *bh1730)
{
	switch (bh1730->gain) {
	case BH1730_GAIN_1X:
		return 1;
	case BH1730_GAIN_2X:
		return 2;
	case BH1730_GAIN_64X:
		return 64;
	case BH1730_GAIN_128X:
		return 128;
	}

	dev_warn(&bh1730->client->dev,
		 "invalid gain multiplier settings: %d\n", bh1730->gain);
	bh1730->gain = BH1730_GAIN_1X;

	return 1;
}

static int bh1730_set_gain(struct bh1730_data *bh1730, enum bh1730_gain gain)
{
	int ret = bh1730_write(bh1730, BH1730_REG_GAIN, gain);

	if (ret < 0)
		return ret;

	bh1730->gain = gain;

	return 0;
}

static int bh1730_set_integration_time_cycle(struct bh1730_data *bh1730,
					     u32 cycle)
{
	int ret, itime;

	itime = 256 - cycle;

	/* ITIME == 0 is reserved for manual integration mode. */
	if (itime <= 0 || itime > 255) {
		dev_warn(&bh1730->client->dev,
			 "integration time out of range: %d cycles\n", cycle);

		return -ERANGE;
	}

	ret = bh1730_write(bh1730, BH1730_REG_TIMING, itime);
	if (ret < 0)
		return ret;

	bh1730->itime_us = BH1730_ITIME_TO_US * cycle;
	bh1730->tmt_us = bh1730->itime_us + BH1730_ADC_CALC_DELAY_US;

	return 0;
}

static int bh1730_adjust_gain(struct bh1730_data *bh1730)
{
	int visible, ir, highest, gain, ret;

	visible = bh1730_read_word(bh1730, BH1730_REG_DATA0LOW);
	if (visible < 0)
		return visible;

	ir = bh1730_read_word(bh1730, BH1730_REG_DATA1LOW);
	if (ir < 0)
		return ir;

	highest = max(visible, ir);

	/* Adjust gain based on sensitivity calibration */
	gain = bh1730->gain;
	if (highest > bh1730->cal.gain_coeff[gain].fl &&
	    gain != BH1730_GAIN_1X) {
		gain--; /* Decrease sensitivity */
	} else if (highest < bh1730->cal.gain_coeff[gain].cl &&
		   gain != BH1730_GAIN_128X) {
		gain++; /* Increase sensitivity */
	}

	if (gain != bh1730->gain) {
		ret = bh1730_set_gain(bh1730, gain);
		if (ret < 0)
			return ret;

		fsleep(bh1730->tmt_us);
	}

	return 0;
}

static int bh1730_get_lux(struct bh1730_data *bh1730)
{
	const struct opt_win_coeff_t *opt_win_coeff;
	int i, visible, ir;
	u64 lux = 0;

	visible = bh1730_read_word(bh1730, BH1730_REG_DATA0LOW);

	/* If visible is 0, skip calculations */
	if (visible <= 0)
		return visible;

	ir = bh1730_read_word(bh1730, BH1730_REG_DATA1LOW);
	if (ir < 0)
		return ir;

	/* Calibrate based on optical window */
	for (i = 0; i < bh1730->cal.opt_win_coeff_count; i++) {
		opt_win_coeff = &bh1730->cal.opt_win_coeff[i];
		if (1000 * ir / visible < opt_win_coeff->rc) {
			lux = ((u64)opt_win_coeff->cv * visible) -
			      (opt_win_coeff->ci * ir);
			break;
		}
	}

	/* Calculate lux */
	lux *= BH1730_DEFAULT_ITIME_MS;
	lux = div_u64(lux, bh1730_gain_multiplier(bh1730) * bh1730->itime_us);
	lux = div_u64(lux * bh1730->cal.mul, 1000);

	if (lux > BH1730_MAX_MEASURED_LUX)
		lux = BH1730_MAX_MEASURED_LUX;

	return (int)lux;
}

static int bh1730_power_ctrl(struct bh1730_data *bh1730, bool enable)
{
	struct device *dev = &bh1730->client->dev;
	int ret;

	if (bh1730->powered == enable)
		return 0;

	if (enable) {
		ret = regulator_enable(bh1730->reg_vdd);
		if (ret) {
			dev_err(dev, "failed to enable vdd: %d\n", ret);
			return ret;
		}

		ret = regulator_enable(bh1730->reg_vid);
		if (ret) {
			dev_err(dev, "failed to enable vid: %d\n", ret);
			regulator_disable(bh1730->reg_vdd);
			return ret;
		}

		fsleep(BH1730_POWER_ON_DELAY_US);
	} else {
		regulator_disable(bh1730->reg_vid);
		regulator_disable(bh1730->reg_vdd);
	}

	bh1730->powered = enable;

	return 0;
}

static int bh1730_power_on(struct bh1730_data *bh1730)
{
	int ret = bh1730_power_ctrl(bh1730, true);

	if (ret < 0)
		return ret;

	return bh1730_write(bh1730, BH1730_REG_CONTROL,
			    BH1730_CONTROL_POWER | BH1730_CONTROL_ADC_EN);
}

static int bh1730_init_config(struct bh1730_data *bh1730)
{
	int ret;

	ret = bh1730_set_gain(bh1730, BH1730_GAIN_64X);
	if (ret < 0)
		return ret;

	return bh1730_set_integration_time_cycle(bh1730,
						 bh1730->cal.itime_cycle);
}

static int bh1730_power_off(struct bh1730_data *bh1730)
{
	int ret = bh1730_write(bh1730, BH1730_REG_CONTROL, 0);

	if (ret < 0)
		return ret;

	return bh1730_power_ctrl(bh1730, false);
}

static int bh1730_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct bh1730_data *bh1730 = iio_priv(indio_dev);
	int data_reg, ret;

	ret = bh1730_adjust_gain(bh1730);
	if (ret < 0)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = bh1730_get_lux(bh1730);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		switch (chan->channel2) {
		case IIO_MOD_LIGHT_CLEAR:
			data_reg = BH1730_REG_DATA0LOW;
			break;
		case IIO_MOD_LIGHT_IR:
			data_reg = BH1730_REG_DATA1LOW;
			break;
		default:
			return -EINVAL;
		}
		ret = bh1730_read_word(bh1730, data_reg);
		if (ret < 0)
			return ret;
		ret = ret * 1000 / bh1730_gain_multiplier(bh1730);
		*val = ret / 1000;
		*val2 = (ret % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		*val2 = bh1730->itime_us;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int bh1730_parse_dt(struct bh1730_data *bh1730)
{
	struct device *dev = &bh1730->client->dev;
	struct device_node *dn = dev->of_node;
	u32 *opt_win_coeff = NULL;
	u32 *gain_coeff = NULL;
	int opt_win_coeff_count, gain_coeff_count;
	u32 cycle = 0, mul = 0;

	bh1730->cal = def_lux_data;

	bh1730->reg_vdd = devm_regulator_get(dev, "als-vdd");
	if (IS_ERR(bh1730->reg_vdd))
		return dev_err_probe(dev, PTR_ERR(bh1730->reg_vdd),
				     "failed to get als-vdd\n");

	bh1730->reg_vid = devm_regulator_get(dev, "als-vid");
	if (IS_ERR(bh1730->reg_vid))
		return dev_err_probe(dev, PTR_ERR(bh1730->reg_vid),
				     "failed to get als-vid\n");

	if (!dn)
		goto out;

	/* Get calibration */
	if (of_property_read_u32(dn, "rohm,integration-cycle", &cycle) ||
	    of_property_read_u32(dn, "rohm,lux-multiplier", &mul) ||
	    !cycle || !mul)
		goto out;

	/* Get optical window coefficients */
	opt_win_coeff_count = of_property_count_u32_elems(dn, "rohm,opt-win-coeff");
	if (opt_win_coeff_count > 0 &&
	    !(opt_win_coeff_count % (sizeof(struct opt_win_coeff_t) / sizeof(u32)))) {
		opt_win_coeff = devm_kcalloc(dev, opt_win_coeff_count,
					     sizeof(u32), GFP_KERNEL);
		if (!opt_win_coeff)
			return -ENOMEM;

		if (of_property_read_u32_array(dn, "rohm,opt-win-coeff",
					       opt_win_coeff, opt_win_coeff_count))
			goto out;
	}

	/* Get gain sensitivity coefficients, 2 for each gain supported */
	gain_coeff_count = of_property_count_u32_elems(dn, "rohm,gain-coeff");
	if (gain_coeff_count == 8) {
		gain_coeff = devm_kcalloc(dev, gain_coeff_count,
					  sizeof(u32), GFP_KERNEL);
		if (!gain_coeff)
			return -ENOMEM;

		if (of_property_read_u32_array(dn, "rohm,gain-coeff",
					       gain_coeff, gain_coeff_count))
			goto out;
	}

	if (opt_win_coeff) {
		bh1730->cal.opt_win_coeff =
			(const struct opt_win_coeff_t *)opt_win_coeff;
		bh1730->cal.opt_win_coeff_count = opt_win_coeff_count /
			(sizeof(struct opt_win_coeff_t) / sizeof(u32));
	}

	if (gain_coeff)
		bh1730->cal.gain_coeff = (const struct gain_coeff_t *)gain_coeff;

	bh1730->cal.itime_cycle = cycle;
	bh1730->cal.mul = mul;

	return 0;

out:
	dev_info(dev, "using default calibration\n");

	return 0;
}

static const struct iio_info bh1730_info = {
	.read_raw = bh1730_read_raw,
};

static const struct iio_chan_spec bh1730_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_INT_TIME),
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_CLEAR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static void bh1730_power_off_action(void *data)
{
	bh1730_power_off(data);
}

static int bh1730_probe(struct i2c_client *client)
{
	struct bh1730_data *bh1730;
	struct iio_dev *indio_dev;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*bh1730));
	if (!indio_dev)
		return -ENOMEM;

	bh1730 = iio_priv(indio_dev);
	bh1730->client = client;
	i2c_set_clientdata(client, indio_dev);

	ret = bh1730_parse_dt(bh1730);
	if (ret < 0)
		return ret;

	ret = bh1730_power_on(bh1730);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&client->dev, bh1730_power_off_action,
				       bh1730);
	if (ret)
		return ret;

	ret = bh1730_init_config(bh1730);
	if (ret < 0)
		return ret;

	indio_dev->info = &bh1730_info;
	indio_dev->name = "bh1730";
	indio_dev->channels = bh1730_channels;
	indio_dev->num_channels = ARRAY_SIZE(bh1730_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int bh1730_suspend(struct device *dev)
{
	struct bh1730_data *bh1730 = iio_priv(dev_get_drvdata(dev));

	return bh1730_power_off(bh1730);
}

static int bh1730_resume(struct device *dev)
{
	struct bh1730_data *bh1730 = iio_priv(dev_get_drvdata(dev));
	int ret;

	ret = bh1730_power_on(bh1730);
	if (ret < 0)
		return ret;

	return bh1730_init_config(bh1730);
}

static DEFINE_SIMPLE_DEV_PM_OPS(bh1730_pm_ops, bh1730_suspend, bh1730_resume);

static const struct i2c_device_id bh1730_i2c_device_id[] = {
	{ BH1730_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bh1730_i2c_device_id);

static const struct of_device_id of_bh1730_match[] = {
	{ .compatible = "rohm,bh1730fvc" },
	{ }
};
MODULE_DEVICE_TABLE(of, of_bh1730_match);

static struct i2c_driver bh1730_driver = {
	.probe = bh1730_probe,
	.driver = {
		.name = BH1730_NAME,
		.of_match_table = of_bh1730_match,
		.pm = pm_sleep_ptr(&bh1730_pm_ops),
	},
	.id_table = bh1730_i2c_device_id,
};
module_i2c_driver(bh1730_driver);

MODULE_AUTHOR("CTCaer <ctcaer@gmail.com>");
MODULE_DESCRIPTION("ROHM BH1730FVC driver");
MODULE_LICENSE("GPL");
