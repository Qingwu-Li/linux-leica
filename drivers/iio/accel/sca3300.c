// SPDX-License-Identifier: GPL-2.0-only
/*
 * Murata SCA3300 3-axis industrial accelerometer
 *
 * Copyright (c) 2021 Vaisala Oyj. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define SCA3300_ALIAS "sca3300"

#define SCA3300_CRC8_POLYNOMIAL 0x1d

/* Device mode register */
#define SCA3300_REG_MODE	0xd
#define SCA3300_MODE_SW_RESET	0x20

/* Last register in map */
#define SCA3300_REG_SELBANK	0x1f

/* Device status and mask */
#define SCA3300_REG_STATUS	0x6
#define SCA3300_STATUS_MASK	GENMASK(8, 0)

/* Device ID */
#define SCA3300_REG_WHOAMI	0x10

/* Device return status and mask */
#define SCA3300_VALUE_RS_ERROR	0x3
#define SCA3300_MASK_RS_STATUS	GENMASK(1, 0)

enum sca3300_scan_indexes {
	SCA3300_ACC_X = 0,
	SCA3300_ACC_Y,
	SCA3300_ACC_Z,
	SCA3300_TEMP,
	SCA3300_TIMESTAMP,
};

#define SCA3300_ACCEL_CHANNEL(index, reg, axis) {			\
	.type = IIO_ACCEL,						\
	.address = reg,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type =					\
	BIT(IIO_CHAN_INFO_SCALE) |					\
	BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),		\
	.info_mask_shared_by_type_available =				\
	BIT(IIO_CHAN_INFO_SCALE) |					\
	BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),		\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_CPU,					\
	},								\
}

#define SCA3300_TEMP_CHANNEL(index, reg) {				\
		.type = IIO_TEMP,					\
		.address = reg,						\
		.scan_index = index,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.endianness = IIO_CPU,				\
		},							\
}

static const struct iio_chan_spec sca3300_channels[] = {
	SCA3300_ACCEL_CHANNEL(SCA3300_ACC_X, 0x1, X),
	SCA3300_ACCEL_CHANNEL(SCA3300_ACC_Y, 0x2, Y),
	SCA3300_ACCEL_CHANNEL(SCA3300_ACC_Z, 0x3, Z),
	SCA3300_TEMP_CHANNEL(SCA3300_TEMP, 0x05),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const int sca3300_freq_tbl[] = {70, 10};
static const int scl3300_freq_tbl[] = {40, 70, 10};
static const int sca3300_freq_modes_map[] = {0, 0, 0, 1};
static const int scl3300_freq_modes_map[] = {0, 1, 2};

static const int sca3300_accel_scale_tbl[][2] = {{0, 370}, {0, 741}, {0, 185}};
static const int scl3300_accel_scale_tbl[][2] = {{0, 167}, {0, 333}, {0, 83}};
static const int sca3300_accel_scale_modes_map[] = {0, 1, 2, 2};
static const int scl3300_accel_scale_modes_map[] = {0, 1, 2};

static const int sca3300_avail_modes_map[] = {0, 1, 2, 3};
static const int scl3300_avail_modes_map[] = {0, 1, 3};
static const unsigned long sca3300_scan_masks[] = {
	BIT(SCA3300_ACC_X) | BIT(SCA3300_ACC_Y) | BIT(SCA3300_ACC_Z) |
	BIT(SCA3300_TEMP),
	0
};

struct sca3300_chip_info {
	const struct iio_chan_spec *channels;
	const int (*accel_scale_table)[2];
	const int *accel_scale_modes_map;
	const unsigned long *scan_masks;
	const int *avail_modes_table;
	const int *freq_modes_map;
	const int *freq_table;
	const u8 num_accel_scales;
	const u8 num_avail_modes;
	const u8 num_channels;
	const u8 num_freqs;
	const u8 chip_id;
	const char *name;
};

/**
 * struct sca3300_data - device data
 * @spi: SPI device structure
 * @lock: Data buffer lock
 * @scan: Triggered buffer. Four channel 16-bit data + 64-bit timestamp
 * @txbuf: Transmit buffer
 * @rxbuf: Receive buffer
 */
struct sca3300_data {
	struct spi_device *spi;
	struct mutex lock;
	struct {
		s16 channels[4];
		s64 ts __aligned(sizeof(s64));
	} scan;
	const struct sca3300_chip_info *chip;
	u8 txbuf[4] ____cacheline_aligned;
	u8 rxbuf[4];
};

static const struct sca3300_chip_info sca3300_chip_tbl[] = {
	{	.num_accel_scales = ARRAY_SIZE(sca3300_accel_scale_tbl)*2,
		.accel_scale_modes_map = sca3300_accel_scale_modes_map,
		.accel_scale_table = sca3300_accel_scale_tbl,
		.num_channels = ARRAY_SIZE(sca3300_channels),
		.freq_modes_map = sca3300_freq_modes_map,
		.avail_modes_table = sca3300_avail_modes_map,
		.freq_table = sca3300_freq_tbl,
		.scan_masks = sca3300_scan_masks,
		.channels = sca3300_channels,
		.num_avail_modes = 4,
		.name = "sca3300",
		.chip_id = 0x51,
		.num_freqs = 2,
	},
	{	.num_accel_scales = ARRAY_SIZE(scl3300_accel_scale_tbl)*2,
		.accel_scale_modes_map = scl3300_accel_scale_modes_map,
		.accel_scale_table = scl3300_accel_scale_tbl,
		.num_channels = ARRAY_SIZE(sca3300_channels),
		.avail_modes_table = scl3300_avail_modes_map,
		.freq_modes_map = scl3300_freq_modes_map,
		.scan_masks = sca3300_scan_masks,
		.freq_table = scl3300_freq_tbl,
		.channels = sca3300_channels,
		.num_avail_modes = 3,
		.name = "scl3300",
		.chip_id = 0xC1,
		.num_freqs = 3,
	},
};

DECLARE_CRC8_TABLE(sca3300_crc_table);

static int sca3300_transfer(struct sca3300_data *sca_data, int *val)
{
	/* Consecutive requests min. 10 us delay (Datasheet section 5.1.2) */
	struct spi_delay delay = { .value = 10, .unit = SPI_DELAY_UNIT_USECS };
	int32_t ret;
	int rs;
	u8 crc;
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = sca_data->txbuf,
			.len = ARRAY_SIZE(sca_data->txbuf),
			.delay = delay,
			.cs_change = 1,
		},
		{
			.rx_buf = sca_data->rxbuf,
			.len = ARRAY_SIZE(sca_data->rxbuf),
			.delay = delay,
		}
	};

	/* inverted crc value as described in device data sheet */
	crc = ~crc8(sca3300_crc_table, &sca_data->txbuf[0], 3, CRC8_INIT_VALUE);
	sca_data->txbuf[3] = crc;

	ret = spi_sync_transfer(sca_data->spi, xfers, ARRAY_SIZE(xfers));
	if (ret) {
		dev_err(&sca_data->spi->dev,
			"transfer error, error: %d\n", ret);
		return -EIO;
	}

	crc = ~crc8(sca3300_crc_table, &sca_data->rxbuf[0], 3, CRC8_INIT_VALUE);
	if (sca_data->rxbuf[3] != crc) {
		dev_err(&sca_data->spi->dev, "CRC checksum mismatch");
		return -EIO;
	}

	/* get return status */
	rs = sca_data->rxbuf[0] & SCA3300_MASK_RS_STATUS;
	if (rs == SCA3300_VALUE_RS_ERROR)
		ret = -EINVAL;

	*val = sign_extend32(get_unaligned_be16(&sca_data->rxbuf[1]), 15);

	return ret;
}

static int sca3300_error_handler(struct sca3300_data *sca_data)
{
	int ret;
	int val;

	mutex_lock(&sca_data->lock);
	sca_data->txbuf[0] = SCA3300_REG_STATUS << 2;
	ret = sca3300_transfer(sca_data, &val);
	mutex_unlock(&sca_data->lock);
	/*
	 * Return status error is cleared after reading status register once,
	 * expect EINVAL here.
	 */
	if (ret != -EINVAL) {
		dev_err(&sca_data->spi->dev,
			"error reading device status: %d\n", ret);
		return ret;
	}

	dev_err(&sca_data->spi->dev, "device status: 0x%lx\n",
		val & SCA3300_STATUS_MASK);

	return 0;
}

static int sca3300_read_reg(struct sca3300_data *sca_data, u8 reg, int *val)
{
	int ret;

	mutex_lock(&sca_data->lock);
	sca_data->txbuf[0] = reg << 2;
	ret = sca3300_transfer(sca_data, val);
	mutex_unlock(&sca_data->lock);
	if (ret != -EINVAL)
		return ret;

	return sca3300_error_handler(sca_data);
}

static int sca3300_write_reg(struct sca3300_data *sca_data, u8 reg, int val)
{
	int reg_val = 0;
	int ret;

	mutex_lock(&sca_data->lock);
	/* BIT(7) for write operation */
	sca_data->txbuf[0] = BIT(7) | (reg << 2);
	put_unaligned_be16(val, &sca_data->txbuf[1]);
	ret = sca3300_transfer(sca_data, &reg_val);
	mutex_unlock(&sca_data->lock);
	if (ret != -EINVAL)
		return ret;

	return sca3300_error_handler(sca_data);
}

static int sca3300_set_op_mode(struct sca3300_data *sca_data, int index)
{
	int mode;

	if ((index < 0) || (index >= sca_data->chip->num_avail_modes))
		return -EINVAL;

	mode = sca_data->chip->avail_modes_table[index];

	return sca3300_write_reg(sca_data, SCA3300_REG_MODE, mode);
}

static int sca3300_get_op_mode(struct sca3300_data *sca_data, int *index)
{
	int reg_val;
	int ret;
	int i;

	ret = sca3300_read_reg(sca_data, SCA3300_REG_MODE, &reg_val);
	if (ret)
		return ret;

	for (i = 0; i < sca_data->chip->num_avail_modes; i++) {
		if (sca_data->chip->avail_modes_table[i] == reg_val) {
			*index = i;
			break;
		}
	}

	return ret;
}

static int sca3300_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct sca3300_data *data = iio_priv(indio_dev);
	int mode = -1;
	int reg_val;
	int index;
	int ret;
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		for (i = 0; i < data->chip->num_avail_modes; i++) {
			index = data->chip->accel_scale_modes_map[i];
			if ((val == data->chip->accel_scale_table[index][0]) &&
			(val2 == data->chip->accel_scale_table[index][1])) {
				mode = i;
				break;
			}
		}
		return sca3300_set_op_mode(data, mode);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = sca3300_get_op_mode(data, &reg_val);
		if (ret)
			return ret;
		index = data->chip->accel_scale_modes_map[reg_val];
		for (i = 0; i < data->chip->num_avail_modes; i++) {
			int index_new = data->chip->accel_scale_modes_map[i];
			int index_freq = data->chip->freq_modes_map[i];

			if (val == data->chip->freq_table[index_freq]) {
				if (data->chip->accel_scale_table[index] ==
				data->chip->accel_scale_table[index_new]) {
					mode = i;
					break;
				}
			}
		}
		return sca3300_set_op_mode(data, mode);
	default:
		return -EINVAL;
	}
}

static int sca3300_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct sca3300_data *data = iio_priv(indio_dev);
	int reg_val;
	int index;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = sca3300_read_reg(data, chan->address, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = sca3300_get_op_mode(data, &reg_val);
		if (ret)
			return ret;
		switch (chan->type) {
		case IIO_ACCEL:
			index = data->chip->accel_scale_modes_map[reg_val];
			*val = data->chip->accel_scale_table[index][0];
			*val2 = data->chip->accel_scale_table[index][1];
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:

		ret = sca3300_get_op_mode(data, &reg_val);
		if (ret)
			return ret;
		index = data->chip->freq_modes_map[reg_val];
		*val = data->chip->freq_table[index];
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static irqreturn_t sca3300_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct sca3300_data *data = iio_priv(indio_dev);
	int bit, ret, val, i = 0;

	for_each_set_bit(bit, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = sca3300_read_reg(data, sca3300_channels[bit].address,
				       &val);
		if (ret) {
			dev_err_ratelimited(&data->spi->dev,
				"failed to read register, error: %d\n", ret);
			/* handled, but bailing out due to errors */
			goto out;
		}
		data->scan.channels[i++] = val;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
					   iio_get_time_ns(indio_dev));
out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

/*
 * sca3300_init - Device init sequence. See datasheet rev 2 section
 * 4.2 Start-Up Sequence for details.
 */
static int sca3300_init(struct sca3300_data *sca_data,
			struct iio_dev *indio_dev)
{
	int value = 0;
	int ret;
	int i;

	ret = sca3300_write_reg(sca_data, SCA3300_REG_MODE,
				SCA3300_MODE_SW_RESET);
	if (ret)
		return ret;

	/*
	 * Wait 1ms after SW-reset command.
	 * Wait 15ms for settling of signal paths.
	 */
	usleep_range(16e3, 50e3);

	ret = sca3300_read_reg(sca_data, SCA3300_REG_WHOAMI, &value);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(sca3300_chip_tbl); i++) {
		if (sca3300_chip_tbl[i].chip_id == value)
			break;
	}
	if (i == ARRAY_SIZE(sca3300_chip_tbl)) {
		dev_err(&sca_data->spi->dev, "Invalid chip %x\n", value);
		return -ENODEV;
	}

	indio_dev->available_scan_masks = sca3300_chip_tbl[i].scan_masks;
	indio_dev->num_channels = sca3300_chip_tbl[i].num_channels;
	indio_dev->channels = sca3300_chip_tbl[i].channels;
	sca_data->chip = &sca3300_chip_tbl[i];
	indio_dev->name = sca3300_chip_tbl[i].name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return 0;
}

static int sca3300_debugfs_reg_access(struct iio_dev *indio_dev,
				      unsigned int reg, unsigned int writeval,
				      unsigned int *readval)
{
	struct sca3300_data *data = iio_priv(indio_dev);
	int value;
	int ret;

	if (reg > SCA3300_REG_SELBANK)
		return -EINVAL;

	if (!readval)
		return sca3300_write_reg(data, reg, writeval);

	ret = sca3300_read_reg(data, reg, &value);
	if (ret)
		return ret;

	*readval = value;

	return 0;
}

static int sca3300_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	struct sca3300_data *data = iio_priv(indio_dev);
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			*vals = (const int *)data->chip->accel_scale_table;
			*length = data->chip->num_accel_scales;
			*type = IIO_VAL_INT_PLUS_MICRO;
			return IIO_AVAIL_LIST;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*vals = (const int *)data->chip->freq_table;
		*length = data->chip->num_freqs;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_info sca3300_info = {
	.read_raw = sca3300_read_raw,
	.write_raw = sca3300_write_raw,
	.debugfs_reg_access = &sca3300_debugfs_reg_access,
	.read_avail = sca3300_read_avail,
};

static int sca3300_probe(struct spi_device *spi)
{
	struct sca3300_data *sca_data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*sca_data));
	if (!indio_dev)
		return -ENOMEM;

	sca_data = iio_priv(indio_dev);
	mutex_init(&sca_data->lock);
	sca_data->spi = spi;

	crc8_populate_msb(sca3300_crc_table, SCA3300_CRC8_POLYNOMIAL);

	indio_dev->info = &sca3300_info;

	ret = sca3300_init(sca_data, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "failed to init device, error: %d\n", ret);
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      iio_pollfunc_store_time,
					      sca3300_trigger_handler, NULL);
	if (ret) {
		dev_err(&spi->dev,
			"iio triggered buffer setup failed, error: %d\n", ret);
		return ret;
	}

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "iio device register failed, error: %d\n",
			ret);
	}

	return ret;
}

static const struct of_device_id sca3300_dt_ids[] = {
	{ .compatible = "murata,sca3300"},
	{ .compatible = "murata,scl3300"},
	{}
};
MODULE_DEVICE_TABLE(of, sca3300_dt_ids);

static struct spi_driver sca3300_driver = {
	.driver = {
		.name		= SCA3300_ALIAS,
		.of_match_table = sca3300_dt_ids,
	},
	.probe	= sca3300_probe,
};
module_spi_driver(sca3300_driver);

MODULE_AUTHOR("Tomas Melin <tomas.melin@vaisala.com>");
MODULE_DESCRIPTION("Murata SCA3300 SPI Accelerometer");
MODULE_LICENSE("GPL v2");
