// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the SX937x
 *
 * Copyright Leica Geosystems AG
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/types.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include "sx9370.h"

struct sx9370_data {
	struct mutex mutex;
	struct i2c_client *client;
	struct iio_trigger *trig;
	struct regmap *regmap;
	int prox_stat[SX9370_NUM_CHANNELS];
	bool trigger_enabled;
	u32 *buffer;
	struct completion completion;
	int channel_users[SX9370_NUM_CHANNELS];
};

static const struct iio_event_spec sx9370_events[] = { {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_EITHER,
	.mask_separate = BIT(IIO_EV_INFO_VALUE),
} };

#define SX9370_CHANNEL(idx)                                                \
	{                                                                  \
		.type = IIO_PROXIMITY,                                     \
		.info_mask_separate = BIT(IIO_CHAN_INFO_DEBOUNCE_COUNT) |  \
				      BIT(IIO_CHAN_INFO_HYSTERESIS) |      \
				      BIT(IIO_CHAN_INFO_SAMP_FREQ) |       \
				      BIT(IIO_CHAN_INFO_PROCESSED) |       \
				      BIT(IIO_CHAN_INFO_SCALE) |           \
				      BIT(IIO_CHAN_INFO_RAW),              \
		.info_mask_shared_by_all_available =                       \
			BIT(IIO_CHAN_INFO_DEBOUNCE_COUNT) |                \
			BIT(IIO_CHAN_INFO_HYSTERESIS) |                    \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                     \
			BIT(IIO_CHAN_INFO_SCALE),                          \
		.indexed = 1, .channel = idx, .event_spec = sx9370_events, \
		.channel2 = idx, .datasheet_name = "proximity",            \
		.num_event_specs = ARRAY_SIZE(sx9370_events),              \
		.scan_index = idx,                                         \
		.scan_type = {                                             \
			.sign = 'u',                                       \
			.realbits = 32,                                    \
			.storagebits = 32,                                 \
			.shift = 0,                                        \
		},                                                         \
	}

static const struct iio_chan_spec sx9370_channels[] = {
	SX9370_CHANNEL(0), SX9370_CHANNEL(1), SX9370_CHANNEL(2),
	SX9370_CHANNEL(3), SX9370_CHANNEL(4), SX9370_CHANNEL(5),
	SX9370_CHANNEL(6), SX9370_CHANNEL(7), IIO_CHAN_SOFT_TIMESTAMP(8),
};

static const int sx9370_samp_freq_table[] = {
	250000, 219000, 194000, 175000, 159000, 146000, 135000, 125000,
	117000, 109000, 97200,	87500,	79600,	72900,	67300,	62500,
	58300,	54700,	48600,	43800,	39800,	36500,	33700,	31300,
	29200,	27300,	21900,	18200,	15600,	7800,	4600,	3400
};

static const int sx9370_hyst_table[][2] = {
	{ 0, 0 },
	{ 0, 60000 },
	{ 0, 120000 },
	{ 0, 250000 },
};

static const int sx9370_scale_factor[] = { 1, 2, 4, 8, 16, 32, 64 };
static const u32 sx9370_debounce_table[] = { 0, 2, 4, 8 };

static const struct regmap_config sx9370_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.max_register = 0x8298,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int sx9370_inc_chan_users(struct sx9370_data *data, int chan)
{
	data->channel_users[chan]++;
	return data->channel_users[chan];
}

static int sx9370_dec_chan_users(struct sx9370_data *data, int chan)
{
	data->channel_users[chan]--;
	return data->channel_users[chan];
}

static int sx9370_inc_users(struct sx9370_data *data, int chan)
{
	if (sx9370_inc_chan_users(data, chan) == 1) {
		return regmap_update_bits(data->regmap, SX9370_COMMAND,
					  BIT(chan), BIT(chan));
	}
	return 0;
}

static int sx9370_dec_users(struct sx9370_data *data, int chan)
{
	if (sx9370_dec_chan_users(data, chan) == 0) {
		return regmap_update_bits(data->regmap, SX9370_COMMAND,
					  BIT(chan), 0);
	}
	return 0;
}

static int sx9370_read_reg_field(struct sx9370_data *data, int *val, int reg,
				 u32 mask)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(data->regmap, reg, &regval);
	if (ret < 0)
		return ret;

	*val = (regval & mask) >> __ffs(mask);

	return 0;
}

static int sx9370_read_samp_freq(struct sx9370_data *data,
				 const struct iio_chan_spec *chan, int *val,
				 int *val2)
{
	int regfield;
	int ret;

	ret = sx9370_read_reg_field(data, &regfield,
				    SX9370_AFE_PARAMETERS_PH(chan->channel),
				    SX9370_SAMPLE_FREQ_MASK);
	if (ret < 0)
		return ret;

	*val = sx9370_samp_freq_table[regfield];
	*val2 = 0;

	return IIO_VAL_INT;
}

static int sx9370_read_hysteresis(struct sx9370_data *data,
				  const struct iio_chan_spec *chan, int *val,
				  int *val2)
{
	int regfield;
	int ret;

	ret = sx9370_read_reg_field(data, &regfield,
				    SX9370_FILTER_SETUP_A_PH(chan->channel),
				    HYSTERESIS_MASK);
	if (ret < 0)
		return ret;

	*val = sx9370_hyst_table[regfield][0];
	*val2 = sx9370_hyst_table[regfield][1];

	return IIO_VAL_INT;
}

static int sx9370_read_debounce_count(struct sx9370_data *data,
				      const struct iio_chan_spec *chan,
				      int *val, int *val2)
{
	int regfield;
	int ret;

	ret = sx9370_read_reg_field(data, &regfield,
				    SX9370_FILTER_SETUP_A_PH(chan->channel),
				    DEBOUNCER_RELEASE_MASK);
	if (ret < 0)
		return ret;

	*val = sx9370_debounce_table[regfield];
	*val2 = 0;
	return IIO_VAL_INT;
}

static int sx9370_read_scale(struct sx9370_data *data,
			     const struct iio_chan_spec *chan, int *val,
			     int *val2)
{
	int regfield;
	int ret;

	ret = sx9370_read_reg_field(data, &regfield,
				    SX9370_FILTER_SETUP_A_PH(chan->channel),
				    SX9370_SCALE_MASK);
	if (ret < 0)
		return ret;

	*val = sx9370_scale_factor[regfield];
	*val2 = 0;

	return IIO_VAL_INT;
}

static int sx9370_set_samp_freq(struct sx9370_data *data,
				const struct iio_chan_spec *chan, int val,
				int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sx9370_samp_freq_table); i++)
		if (val == sx9370_samp_freq_table[i] && val2 == 0)
			return regmap_update_bits(
				data->regmap,
				SX9370_AFE_PARAMETERS_PH(chan->channel),
				SX9370_SAMPLE_FREQ_MASK,
				i << SX9370_SAMPLE_FREQ_SHIFT);
	return -EINVAL;
}

static int sx9370_set_hysteresis(struct sx9370_data *data,
				 const struct iio_chan_spec *chan, int val,
				 int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sx9370_hyst_table); i++)
		if (val == sx9370_hyst_table[i][0] &&
		    val2 == sx9370_hyst_table[i][1])
			return regmap_update_bits(
				data->regmap,
				SX9370_FILTER_SETUP_A_PH(chan->channel),
				HYSTERESIS_MASK, i << HYSTERESIS_SHIFT);
	return -EINVAL;
}

static int sx9370_set_debounce_count(struct sx9370_data *data,
				     const struct iio_chan_spec *chan, int val,
				     int val2)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(sx9370_debounce_table); i++) {
		if (val == sx9370_debounce_table[i] && val2 == 0) {
			ret = regmap_update_bits(
				data->regmap,
				SX9370_FILTER_SETUP_A_PH(chan->channel),
				DEBOUNCER_RELEASE_MASK,
				i << DEBOUNCER_RELEASE_SHIFT);
			if (ret < 0)
				return ret;

			ret = regmap_update_bits(
				data->regmap,
				SX9370_FILTER_SETUP_A_PH(chan->channel),
				DEBOUNCER_DETECTED_MASK,
				i << DEBOUNCER_DETECTED_SHIFT);

			return ret;
		}
	}
	return -EINVAL;
}

static int sx9370_set_diff_scale(struct sx9370_data *data,
				 const struct iio_chan_spec *chan, int val,
				 int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sx9370_scale_factor); i++) {
		if (val == sx9370_scale_factor[i] && val2 == 0) {
			return regmap_update_bits(
				data->regmap,
				SX9370_FILTER_SETUP_A_PH(chan->channel),
				SX9370_SCALE_MASK, i << SX9370_SCALE_SHIFT);
		}
	}
	return -EINVAL;
}

static int sx9370_write_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan, int val, int val2,
			    long mask)
{
	struct sx9370_data *data = iio_priv(indio_dev);

	switch (chan->type) {
	case IIO_PROXIMITY:
		switch (mask) {
		case IIO_CHAN_INFO_SAMP_FREQ:
			return sx9370_set_samp_freq(data, chan, val, val2);
		case IIO_CHAN_INFO_HYSTERESIS:
			return sx9370_set_hysteresis(data, chan, val, val2);
		case IIO_CHAN_INFO_DEBOUNCE_COUNT:
			return sx9370_set_debounce_count(data, chan, val, val2);
		case IIO_CHAN_INFO_SCALE:
			return sx9370_set_diff_scale(data, chan, val, val2);
		default:
			dev_err(&data->client->dev,
				"unknown channel type %d for write_raw\n",
				chan->type);
			return -EINVAL;
		}
	default:
		dev_err(&data->client->dev,
			"unknown channel type %d for write_raw\n", chan->type);
		return -EINVAL;
	}
}

static int sx9370_enable_conversion_irq(struct sx9370_data *data, bool enable)
{
	int ret;

	if (enable) {
		ret = regmap_update_bits(data->regmap, SX9370_IRQ_MASK_A,
					 SX9370_IRQ_CONVERSION_DONE,
					 SX9370_IRQ_CONVERSION_DONE);
		if (ret < 0)
			return ret;

		ret = regmap_update_bits(data->regmap, SX9370_IRQ_SETUP,
					 SX9370_IRQ_PAUSE_ON_INTERRUPT,
					 SX9370_IRQ_PAUSE_ON_INTERRUPT);
	} else {
		ret = regmap_update_bits(data->regmap, SX9370_IRQ_MASK_A,
					 SX9370_IRQ_CONVERSION_DONE, 0);
		if (ret < 0)
			return ret;

		ret = regmap_update_bits(data->regmap, SX9370_IRQ_SETUP,
					 SX9370_IRQ_PAUSE_ON_INTERRUPT, 0);
	}

	return ret;
}

static int sx9370_read_conversion_sync(struct sx9370_data *data,
				       struct iio_chan_spec const *chan,
				       int max_len, int *vals, int *val_len,
				       long mask)
{
	int ret;

	ret = wait_for_completion_timeout(&data->completion,
					  msecs_to_jiffies(200));
	if (!ret) {
		dev_err(&data->client->dev,
			"Timeout waiting for conversion done\n");
		ret = -ETIMEDOUT;
		goto out;
	}
	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		/*
		 * 4 values: offset, useful, usefilter, average
		 */
		if (max_len < 4) {
			ret = -EINVAL;
			goto out;
		}

		ret = sx9370_read_reg_field(data, &vals[0],
					    SX9370_OFFSET_PH(chan->channel),
					    SX9370_OFFSET_MASK);
		if (ret < 0)
			goto out;
		ret = sx9370_read_reg_field(data, &vals[0],
					    SX9370_DIFF_PH(chan->channel),
					    SX9370_DIFF_MASK);
		if (ret < 0)
			goto out;
		ret = sx9370_read_reg_field(data, &vals[1],
					    SX9370_USEFUL_PH(chan->channel),
					    SX9370_USEFUL_MASK);
		if (ret < 0)
			goto out;
		ret = sx9370_read_reg_field(data, &vals[2],
					    SX9370_USEFILTER_PH(chan->channel),
					    SX9370_USEFILTER_MASK);
		if (ret < 0)
			goto out;
		ret = sx9370_read_reg_field(data, &vals[3],
					    SX9370_AVERAGE_PH(chan->channel),
					    SX9370_AVERAGE_MASK);
		if (ret < 0)
			goto out;
		*val_len = 4;
		sx9370_enable_conversion_irq(data, false);
		return IIO_VAL_INT_MULTIPLE;
	case IIO_CHAN_INFO_RAW:
		ret = sx9370_read_reg_field(data, &vals[0],
					    SX9370_DIFF_PH(chan->channel),
					    SX9370_DIFF_MASK);

		if (ret < 0)
			goto out;
		sx9370_enable_conversion_irq(data, false);
		return IIO_VAL_INT;

	default:
		ret = -EINVAL;
		break;
	}

out:
	sx9370_enable_conversion_irq(data, false);
	return ret;
}

static int sx9370_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct sx9370_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	kfree(data->buffer);
	data->buffer = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
	mutex_unlock(&data->mutex);

	if (data->buffer == NULL)
		return -ENOMEM;

	return 0;
}

static int sx9370_read_raw_multi(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int max_len,
				 int *vals, int *val_len, long mask)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	int ret;

	if (chan->type != IIO_PROXIMITY) {
		dev_err(&data->client->dev,
			"unknown channel type %d for read_raw\n", chan->type);
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return sx9370_read_samp_freq(data, chan, &vals[0], &vals[1]);
	case IIO_CHAN_INFO_HYSTERESIS:
		return sx9370_read_hysteresis(data, chan, &vals[0], &vals[1]);
	case IIO_CHAN_INFO_DEBOUNCE_COUNT:
		return sx9370_read_debounce_count(data, chan, &vals[0],
						  &vals[1]);
	case IIO_CHAN_INFO_SCALE:
		return sx9370_read_scale(data, chan, &vals[0], &vals[1]);

	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		ret = sx9370_enable_conversion_irq(data, true);
		if (ret < 0)
			return ret;
		ret = sx9370_read_conversion_sync(data, chan, max_len, vals,
						  val_len, mask);
		sx9370_enable_conversion_irq(data, false);
		return ret;
	default:
		dev_err(&data->client->dev,
			"unknown mask %ld for read_raw_multi\n", mask);
		return -EINVAL;
	}
}

static int sx9370_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = (const int *)sx9370_samp_freq_table;
		*length = ARRAY_SIZE(sx9370_samp_freq_table);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_HYSTERESIS:
		*vals = (const int *)sx9370_hyst_table;
		*length = ARRAY_SIZE(sx9370_hyst_table) * 2;
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_DEBOUNCE_COUNT:
		*vals = (const int *)sx9370_debounce_table;
		*length = ARRAY_SIZE(sx9370_debounce_table);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SCALE:
		*vals = (const int *)sx9370_scale_factor;
		*length = ARRAY_SIZE(sx9370_scale_factor);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;

	default:
		dev_err(&indio_dev->dev, "unknown mask %ld for read_avail\n",
			mask);
		return -EINVAL;
	}
}

static irqreturn_t sx9370_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct sx9370_data *data = iio_priv(indio_dev);

	if (data->trigger_enabled)
		iio_trigger_poll(data->trig);

	/*
	 * Even if no event is enabled, we need to wake the thread to
	 * clear the interrupt state by reading SX9370_IRQ_SOURCE.  It
	 * is not possible to do that here because regmap_read takes a
	 * mutex.
	 */
	return IRQ_WAKE_THREAD;
}

static void sx9370_handle_proximity_events(struct iio_dev *indio_dev)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	unsigned int status_a, status_b, status_c;
	enum iio_event_direction dir;
	int ph, prox;

	regmap_read(data->regmap, SX9370_DEVICE_STATUS_A, &status_a);
	regmap_read(data->regmap, SX9370_DEVICE_STATUS_B, &status_b);
	regmap_read(data->regmap, SX9370_DEVICE_STATUS_C, &status_c);

	/*
	 * PROX4: D7  D6  D5  D4  D3  D2  D1  D0
	 * PROX3: D15 D14 D13 D12 D11 D10 D9  D8
	 * PROX2: D23 D22 D21 D20 D19 D18 D17 D16
	 * PROX1: D31 D30 D29 D28 D27 D26 D25 D24
	 * PHASE: PH7 PH6 PH5 PH4 PH3 PH2 PH1 PH0
	 */

	for (ph = 0; ph < SX9370_NUM_CHANNELS; ph++) {
		for (prox = 0; prox < 4; prox++) {
			int bit_pos = ph + ((3 - prox) * SX9370_NUM_CHANNELS);
			bool new_state = !!(status_a & BIT(bit_pos));
			bool old_state = !!(data->prox_stat[ph] & BIT(prox));

			if (new_state != old_state) {
				enum iio_event_type event_type;

				dev_info(&data->client->dev,
					 "PH%d THRESH%d %s event detected\n",
					 ph, prox + 1,
					 new_state ? "CLOSE" : "FAR");

				switch (prox) {
				case 0:
					event_type = IIO_EV_TYPE_THRESH;
					break;
				case 1:
					event_type = IIO_EV_TYPE_THRESH2;
					break;
				case 2:
					event_type = IIO_EV_TYPE_THRESH3;
					break;
				case 3:
					event_type = IIO_EV_TYPE_THRESH4;
					break;
				default:
					dev_err(&data->client->dev,
						"Invalid proximity threshold %d\n",
						prox);
					return;
				}

				if (new_state) {
					dir = IIO_EV_DIR_RISING;
					data->prox_stat[ph] |= BIT(prox);
				} else {
					dir = IIO_EV_DIR_FALLING;
					data->prox_stat[ph] &= ~BIT(prox);
				}

				iio_push_event(
					indio_dev,
					IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, ph,
							     event_type, dir),
					iio_get_time_ns(indio_dev));
			}
		}
	}
}

static irqreturn_t sx9370_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct sx9370_data *data = iio_priv(indio_dev);
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, SX9370_IRQ_SOURCE, &val);
	if (ret < 0) {
		dev_err(&data->client->dev, "i2c transfer error in irq\n");
		goto out;
	}

	if (val & SX9370_IRQ_SMART_SENSOR)
		sx9370_handle_proximity_events(indio_dev);
	if (val & SX9370_IRQ_CONVERSION_DONE)
		complete(&data->completion);

	if (val & SX9370_IRQ_RELEASE)
		sx9370_handle_proximity_events(indio_dev);
	if (val & SX9370_IRQ_DETECTED)
		sx9370_handle_proximity_events(indio_dev);
out:
	return IRQ_HANDLED;
}

static int sx9370_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val, int val2)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	int scale_factor;
	int thresh_value;
	int mask;
	int ret;

	if (chan->channel < 0 || chan->channel >= SX9370_NUM_CHANNELS)
		return -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		mask = GENMASK(7, 0);
		break;
	case IIO_EV_TYPE_THRESH2:
		mask = GENMASK(15, 8);
		break;
	case IIO_EV_TYPE_THRESH3:
		mask = GENMASK(23, 16);
		break;
	case IIO_EV_TYPE_THRESH4:
		mask = GENMASK(31, 24);
		break;
	default:
		return -EINVAL;
	}

	switch (info) {
	case IIO_EV_INFO_VALUE:
		ret = sx9370_read_reg_field(
			data, &scale_factor,
			SX9370_FILTER_SETUP_A_PH(chan->channel),
			SX9370_SCALE_MASK);
		if (ret < 0)
			return ret;
		val = int_sqrt((val << 1) / (1 << scale_factor));

		return regmap_update_bits(data->regmap,
					  SX9370_PROX_THRESH_PH(chan->channel),
					  mask, val << __ffs(mask));
	default:
		dev_err(&data->client->dev, "unknown info %d\n", info);
		return -EINVAL;
	}
}

static int sx9370_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	unsigned int mask;
	int scale_factor;
	int thresh_value;
	int ret;

	if (chan->channel < 0 || chan->channel >= SX9370_NUM_CHANNELS)
		return -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		mask = GENMASK(7, 0);
		break;
	case IIO_EV_TYPE_THRESH2:
		mask = GENMASK(15, 8);
		break;
	case IIO_EV_TYPE_THRESH3:
		mask = GENMASK(23, 16);
		break;
	case IIO_EV_TYPE_THRESH4:
		mask = GENMASK(31, 24);
		break;
	default:
		dev_err(&data->client->dev, "unknown type %d\n", type);
		return -EINVAL;
	}

	switch (info) {
	case IIO_EV_INFO_VALUE:
		ret = sx9370_read_reg_field(
			data, &thresh_value,
			SX9370_PROX_THRESH_PH(chan->channel), mask);
		if (ret < 0)
			return ret;

		ret = sx9370_read_reg_field(
			data, &scale_factor,
			SX9370_FILTER_SETUP_A_PH(chan->channel),
			SX9370_SCALE_MASK);
		if (ret < 0)
			return ret;

		if (scale_factor == 0x07)
			return -EINVAL;

		scale_factor = 1 << scale_factor;

		*val = ((thresh_value * thresh_value) >> 1) * scale_factor;
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		dev_err(&data->client->dev, "unknown info %d\n", info);
		return -EINVAL;
	}
}

static const struct iio_info sx9370_info = {
	.read_avail = &sx9370_read_avail,
	.read_event_value = &sx9370_read_event_value,
	.read_raw_multi = &sx9370_read_raw_multi,
	.write_event_value = &sx9370_write_event_value,
	.write_raw = &sx9370_write_raw,
	.update_scan_mode = &sx9370_update_scan_mode,
};

static int sx9370_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct sx9370_data *data = iio_priv(indio_dev);
	int ret;

	ret = sx9370_enable_conversion_irq(data, state);
	if (ret < 0)
		return ret;

	data->trigger_enabled = state;
	return 0;
}

static const struct iio_trigger_ops sx9370_trigger_ops = {
	.set_trigger_state = sx9370_set_trigger_state,
};

static irqreturn_t sx9370_trigger_handler(int irq, void *private)
{
	struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct sx9370_data *data = iio_priv(indio_dev);
	int val;
	int bit;
	int ret;
	int i = 0;

	ret = wait_for_completion_timeout(&data->completion,
					  msecs_to_jiffies(200));
	if (!ret) {
		dev_err(&data->client->dev,
			"Timeout waiting for conversion done\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	mutex_lock(&data->mutex);

	iio_for_each_active_channel(indio_dev, bit) {
		ret = sx9370_read_reg_field(
			data, &val,
			SX9370_DIFF_PH(indio_dev->channels[bit].channel),
			SX9370_DIFF_MASK);
		if (ret < 0)
			goto out;

		data->buffer[i++] = val;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data->buffer,
					   iio_get_time_ns(indio_dev));

out:
	mutex_unlock(&data->mutex);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int sx9370_buffer_postenable(struct iio_dev *indio_dev)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	int ret = 0, i;

	mutex_lock(&data->mutex);
	for (i = 0; i < SX9370_NUM_CHANNELS; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			ret = sx9370_inc_users(data, i);
			if (ret < 0)
				break;
		}
	}

	if (ret < 0) {
		for (i = i - 1; i >= 0; i--) {
			if (test_bit(i, indio_dev->active_scan_mask))
				sx9370_dec_users(data, i);
		}
		mutex_unlock(&data->mutex);
		return ret;
	}

	mutex_unlock(&data->mutex);

	return sx9370_enable_conversion_irq(data, true);
}

static int sx9370_buffer_predisable(struct iio_dev *indio_dev)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	int ret, i;

	ret = sx9370_enable_conversion_irq(data, false);
	if (ret < 0)
		return ret;

	mutex_lock(&data->mutex);

	for (i = 0; i < SX9370_NUM_CHANNELS; i++) {
		if (test_bit(i, indio_dev->active_scan_mask))
			sx9370_dec_users(data, i);
	}

	mutex_unlock(&data->mutex);

	return 0;
}

static const struct iio_buffer_setup_ops sx9370_buffer_setup_ops = {
	.postenable = sx9370_buffer_postenable,
	.predisable = sx9370_buffer_predisable,
};

static int sx9370_apply_dt_reg_init(struct i2c_client *client,
				    struct sx9370_data *data)
{
	struct device_node *np = client->dev.of_node;
	int count, i, ret;
	u32 *reg_init_data;
	u32 reg, val;

	if (!np)
		return 0;

	count = of_property_count_u32_elems(np, "Semtech,reg-init");
	if (count <= 0)
		return 0;

	if (count % 2 != 0) {
		dev_err(&client->dev, "Invalid reg-init length: %d\n", count);
		return -EINVAL;
	}

	reg_init_data =
		devm_kcalloc(&client->dev, count, sizeof(u32), GFP_KERNEL);
	if (!reg_init_data)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "Semtech,reg-init", reg_init_data,
					 count);
	if (ret) {
		dev_err(&client->dev, "Failed to read reg-init: %d\n", ret);
		return ret;
	}

	for (i = 0; i < count; i += 2) {
		reg = reg_init_data[i];
		val = reg_init_data[i + 1];

		dev_warn(&client->dev, "Writing DT reg 0x%04x = 0x%08x\n", reg,
			 val);

		ret = regmap_write(data->regmap, reg, val);
		if (ret < 0) {
			dev_err(&client->dev,
				"Failed to write DT reg 0x%04x = 0x%08x: %d\n",
				reg, val, ret);
			return ret;
		}
	}

	dev_info(&client->dev, "Successfully applied %d DT configurations\n",
		 count / 2);
	return 0;
}

static int sx9370_init_device(struct iio_dev *indio_dev)
{
	struct sx9370_data *data = iio_priv(indio_dev);
	unsigned int val;
	int whoami;
	int ret;
	int i;

	ret = regmap_write(data->regmap, SX9370_DEVICE_RESET,
			   SX9370_SOFT_RESET);
	if (ret < 0)
		return ret;

	do {
		regmap_read(data->regmap, SX9370_IRQ_SOURCE, &val);

	} while (val != 0);

	ret = regmap_read(data->regmap, SX9370_DEVICE_INFO, &val);
	if (ret < 0)
		return ret;

	whoami = FIELD_GET(SX9370_WHOAMI_MASK, val);
	switch (whoami) {
	case CHIP_SX9370:
		indio_dev->name = "sx9370";
		break;
	case CHIP_SX9373:
		indio_dev->name = "sx9373";
		break;
	case CHIP_SX9374:
		indio_dev->name = "sx9374";
		break;
	case CHIP_SX9376:
		indio_dev->name = "sx9376";
		break;
	default:
		dev_err(&data->client->dev, "Unknown device ID: 0x%04x\n",
			whoami);
		return -ENODEV;
	}
	dev_info(&data->client->dev, "Device ID: %s\n", indio_dev->name);

	for (i = 0; i < ARRAY_SIZE(sx9370_default_regs); i++) {
		ret = regmap_write(data->regmap, sx9370_default_regs[i].reg,
				   sx9370_default_regs[i].def);
		if (ret < 0)
			return ret;
	}

	ret = sx9370_apply_dt_reg_init(data->client, data);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Failed to apply DT register configurations: %d\n",
			ret);
		return ret;
	}

	return ret;
}

static void sx9370_gpio_probe(struct i2c_client *client,
			      struct sx9370_data *data)
{
	struct gpio_desc *gpiod_int;
	struct device *dev;
	int ret;

	if (!client)
		return;

	dev = &client->dev;

	if (client->irq <= 0) {
		gpiod_int = devm_gpiod_get(dev, "interrupt", GPIOD_IN);
		if (IS_ERR(gpiod_int))
			dev_err(dev, "gpio get irq failed\n");
		else
			client->irq = gpiod_to_irq(gpiod_int);
	}
}

static void sx9370_cleanup_resources(void *data)
{
	struct iio_dev *indio_dev = data;
	struct sx9370_data *priv = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);

	if (priv->client->irq > 0)
		iio_trigger_unregister(priv->trig);

	kfree(priv->buffer);
	priv->buffer = NULL;
}

static int sx9370_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct sx9370_data *data;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->mutex);
	init_completion(&data->completion);
	data->trigger_enabled = false;

	data->regmap = devm_regmap_init_i2c(client, &sx9370_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	indio_dev->channels = sx9370_channels;
	indio_dev->num_channels = ARRAY_SIZE(sx9370_channels);
	indio_dev->info = &sx9370_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	i2c_set_clientdata(client, indio_dev);

	sx9370_gpio_probe(client, data);

	ret = sx9370_init_device(indio_dev);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&client->dev, sx9370_cleanup_resources,
				       indio_dev);
	if (ret) {
		dev_err(&client->dev, "Failed to add cleanup action\n");
		return ret;
	}

	if (client->irq <= 0)
		dev_err(&client->dev, "no valid irq found\n");
	else {
		ret = devm_request_threaded_irq(
			&client->dev, client->irq, sx9370_irq_handler,
			sx9370_irq_thread_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, indio_dev->name,
			indio_dev);
		if (ret < 0)
			return ret;

		data->trig = devm_iio_trigger_alloc(&client->dev, "%s-dev%d",
						    indio_dev->name,
						    iio_device_id(indio_dev));
		if (!data->trig)
			return -ENOMEM;

		data->trig->ops = &sx9370_trigger_ops;
		iio_trigger_set_drvdata(data->trig, indio_dev);

		ret = iio_trigger_register(data->trig);
		if (ret)
			return ret;
	}

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
					 sx9370_trigger_handler,
					 &sx9370_buffer_setup_ops);
	if (ret < 0)
		goto out_trigger_unregister;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_buffer_cleanup;

	return 0;

out_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
out_trigger_unregister:
	if (client->irq > 0)
		iio_trigger_unregister(data->trig);

	return ret;
}

static int sx9370_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sx9370_data *data = iio_priv(indio_dev);

	disable_irq(data->client->irq);
	return 0;
}

static int sx9370_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct sx9370_data *data = iio_priv(indio_dev);

	enable_irq(data->client->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sx9370_pm_ops, sx9370_suspend, sx9370_resume);

static const struct of_device_id sx9370_of_match[] = {
	{
		.compatible = "semtech,sx9370",
	},
	{
		.compatible = "semtech,sx9373",
	},
	{
		.compatible = "semtech,sx9374",
	},
	{
		.compatible = "semtech,sx9376",
	},
	{}
};
MODULE_DEVICE_TABLE(of, sx9370_of_match);

static const struct i2c_device_id sx9370_id[] = { { "sx9370" }, {} };
MODULE_DEVICE_TABLE(i2c, sx9370_id);

static struct i2c_driver sx9370_driver = {
	 .driver = {
		 .name	= "sx9370",
		 .of_match_table = sx9370_of_match,
		 .pm = pm_sleep_ptr(&sx9370_pm_ops),
	},
	.probe		= sx9370_probe,
	.id_table	= sx9370_id,
};

module_i2c_driver(sx9370_driver);
MODULE_AUTHOR("Leica Geosystems AG");
MODULE_DESCRIPTION("SX9370 SAR Sensor Driver");
MODULE_LICENSE("GPL v2");
