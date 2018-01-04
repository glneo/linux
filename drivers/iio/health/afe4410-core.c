// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4410 Heart Rate Monitors and Low-Cost Pulse Oximeters
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

#include "afe440x.h"

#define AFE4410_DRIVER_NAME		"afe4410"

/* AFE4410 registers */
#define AFE4410_TIA_GAIN_SEP23		0x1f
#define AFE4410_TIA_GAIN_SEP		0x20
#define AFE4410_TIA_GAIN		0x21

#define AFE4410_LEDCNTRL2		0x24
#define AFE4410_DESIGN_ID		0x28
#define AFE4410_PROG_INT2_STC		0x34
#define AFE4410_PROG_INT2_ENDC		0x35
#define AFE4410_LED3LEDSTC		0x36
#define AFE4410_LED3LEDENDC		0x37
#define AFE4410_CLKDIV_PRF		0x39
#define AFE4410_OFFDAC			0x3a
#define AFE4410_THR_DET_LOW_CODE	0x3b
#define AFE4410_THR_DET_HIGH_CODE	0x3c
#define AFE4410_DEC			0x3d
#define AFE4410_OFFDAC_LMSB		0x3e
#define AFE4410_AVG_LED2_ALED2VAL	0x3f
#define AFE4410_AVG_LED1_ALED1VAL	0x40
#define AFE4410_FIFO			0x42
#define AFE4410_LED4LEDSTC		0x43
#define AFE4410_LED4LEDENDC		0x44
#define AFE4410_TG_PD1STC		0x45
#define AFE4410_TG_PD1ENDC		0x46
#define AFE4410_TG_PD2STC		0x47
#define AFE4410_TG_PD2ENDC		0x48
#define AFE4410_DATA_RDY_STC		0x52
#define AFE4410_DATA_RDY_ENDC		0x53
#define AFE4410_PROG_INT1_STC		0x57
#define AFE4410_PROG_INT1_ENDC		0x58
#define AFE4410_DYN_TIA_STC		0x64
#define AFE4410_DYN_TIA_ENDC		0x65
#define AFE4410_DYN_ADC_STC		0x66
#define AFE4410_DYN_ADC_ENDC		0x67
#define AFE4410_DYN_CLK_STC		0x68
#define AFE4410_DYN_CLK_ENDC		0x69
#define AFE4410_DEEP_SLEEP_STC		0x6a
#define AFE4410_DEEP_SLEEP_ENDC		0x6b

/* AFE4410 CONTROL0 register fields */
#define AFE440X_CONTROL0_RW_CONT	BIT(4)
#define AFE440X_CONTROL0_ENABLE_ULP	BIT(5)
#define AFE440X_CONTROL0_FIFO_EN	BIT(6)

/* AFE4410 CONTROL2 register fields */
#define AFE440X_CONTROL2_DYN_ADC	BIT(3)
#define AFE440X_CONTROL2_DYN_TIA	BIT(4)
#define AFE440X_CONTROL2_OSC_ENABLE	BIT(9)
#define AFE440X_CONTROL2_DYN_BIAS	BIT(14)
#define AFE440X_CONTROL2_ENSEPGAIN4	BIT(15)
#define AFE440X_CONTROL2_DYN_TX0	BIT(20)

enum afe4410_fields {
	/* Gains */
	F_TIA_GAIN_SEP2_LSB = 1, F_TIA_CF_SEP2, F_TIA_GAIN_SEP2_MSB,
	F_TIA_GAIN_SEP3_LSB, F_TIA_CF_SEP3, F_TIA_GAIN_SEP3_MSB,
	F_TIA_GAIN_SEP_LSB, F_TIA_CF_SEP, F_TIA_GAIN_SEP_MSB,
	F_TIA_GAIN_LSB, F_TIA_CF, F_TIA_GAIN_MSB,

	/* LED Current */
	F_ILED1_MSB, F_ILED2_MSB, F_ILED3_MSB, F_ILED4_MSB,
	F_ILED1_LSB, F_ILED2_LSB, F_ILED3_LSB, F_ILED4_LSB,

	/* Offset DAC */
	F_I_OFFDAC_LED3_MID, F_POL_OFFDAC_LED3, F_I_OFFDAC_LED1_MID,
	F_POL_OFFDAC_LED1, F_I_OFFDAC_AMB1_MID, F_POL_OFFDAC_AMB1,
	F_I_OFFDAC_LED2_MID, F_POL_OFFDAC_LED2, F_I_OFFDAC_LED3_LSB,
	F_I_OFFDAC_LED3_MSB, F_I_OFFDAC_LED1_LSB, F_I_OFFDAC_LED1_MSB,
	F_I_OFFDAC_AMB1_LSB, F_I_OFFDAC_AMB1_MSB, F_I_OFFDAC_LED2_LSB,
	F_I_OFFDAC_LED2_MSB, F_I_OFFDAC_LED3_LSB_EXT, F_I_OFFDAC_LED1_LSB_EXT,
	F_I_OFFDAC_AMB1_LSB_EXT, F_I_OFFDAC_LED2_LSB_EXT,

	/* FIFO and INT Mux */
	F_FIFO_PARTITION, F_INT_MUX1, F_REG_FIFO_PERIOD, F_FIFO_EARLY,
	F_INT_MUX2, F_INT_MUX3,

	/* sentinel */
	F_MAX_FIELDS
};

static const struct reg_field afe4410_reg_fields[] = {
	/* Gains */
	[F_TIA_GAIN_SEP2_LSB]	= REG_FIELD(AFE4410_TIA_GAIN_SEP23, 0, 2),
	[F_TIA_CF_SEP2]		= REG_FIELD(AFE4410_TIA_GAIN_SEP23, 3, 5),
	[F_TIA_GAIN_SEP2_MSB]	= REG_FIELD(AFE4410_TIA_GAIN_SEP23, 6, 6),
	[F_TIA_GAIN_SEP3_LSB]	= REG_FIELD(AFE4410_TIA_GAIN_SEP23, 8, 10),
	[F_TIA_CF_SEP3]		= REG_FIELD(AFE4410_TIA_GAIN_SEP23, 11, 13),
	[F_TIA_GAIN_SEP3_MSB]	= REG_FIELD(AFE4410_TIA_GAIN_SEP23, 14, 14),
	[F_TIA_GAIN_SEP_LSB]	= REG_FIELD(AFE4410_TIA_GAIN_SEP, 0, 2),
	[F_TIA_CF_SEP]		= REG_FIELD(AFE4410_TIA_GAIN_SEP, 3, 5),
	[F_TIA_GAIN_SEP_MSB]	= REG_FIELD(AFE4410_TIA_GAIN_SEP, 6, 6),
	[F_TIA_GAIN_LSB]	= REG_FIELD(AFE4410_TIA_GAIN, 0, 2),
	[F_TIA_CF]		= REG_FIELD(AFE4410_TIA_GAIN, 3, 5),
	[F_TIA_GAIN_MSB]	= REG_FIELD(AFE4410_TIA_GAIN, 6, 6),
	/* LED Current */
	[F_ILED1_MSB]		= REG_FIELD(AFE440X_LEDCNTRL, 0, 5),
	[F_ILED2_MSB]		= REG_FIELD(AFE440X_LEDCNTRL, 6, 11),
	[F_ILED3_MSB]		= REG_FIELD(AFE440X_LEDCNTRL, 12, 17),
	[F_ILED4_MSB]		= REG_FIELD(AFE4410_LEDCNTRL2, 11, 16),
	[F_ILED1_LSB]		= REG_FIELD(AFE440X_LEDCNTRL, 18, 19),
	[F_ILED2_LSB]		= REG_FIELD(AFE440X_LEDCNTRL, 20, 21),
	[F_ILED3_LSB]		= REG_FIELD(AFE440X_LEDCNTRL, 22, 23),
	[F_ILED4_LSB]		= REG_FIELD(AFE4410_LEDCNTRL2, 9, 10),
	/* Offset DAC */
	[F_I_OFFDAC_LED3_MID]	= REG_FIELD(AFE4410_OFFDAC, 0, 3),
	[F_POL_OFFDAC_LED3]	= REG_FIELD(AFE4410_OFFDAC, 4, 4),
	[F_I_OFFDAC_LED1_MID]	= REG_FIELD(AFE4410_OFFDAC, 5, 8),
	[F_POL_OFFDAC_LED1]	= REG_FIELD(AFE4410_OFFDAC, 9, 9),
	[F_I_OFFDAC_AMB1_MID]	= REG_FIELD(AFE4410_OFFDAC, 10, 13),
	[F_POL_OFFDAC_AMB1]	= REG_FIELD(AFE4410_OFFDAC, 14, 14),
	[F_I_OFFDAC_LED2_MID]	= REG_FIELD(AFE4410_OFFDAC, 15, 18),
	[F_POL_OFFDAC_LED2]	= REG_FIELD(AFE4410_OFFDAC, 19, 19),
	[F_I_OFFDAC_LED3_LSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 0, 0),
	[F_I_OFFDAC_LED3_MSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 1, 1),
	[F_I_OFFDAC_LED1_LSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 2, 2),
	[F_I_OFFDAC_LED1_MSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 3, 3),
	[F_I_OFFDAC_AMB1_LSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 4, 4),
	[F_I_OFFDAC_AMB1_MSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 5, 5),
	[F_I_OFFDAC_LED2_LSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 6, 6),
	[F_I_OFFDAC_LED2_MSB]	= REG_FIELD(AFE4410_OFFDAC_LMSB, 7, 7),
	[F_I_OFFDAC_LED3_LSB_EXT] = REG_FIELD(AFE4410_OFFDAC_LMSB, 8, 8),
	[F_I_OFFDAC_LED1_LSB_EXT] = REG_FIELD(AFE4410_OFFDAC_LMSB, 9, 9),
	[F_I_OFFDAC_AMB1_LSB_EXT] = REG_FIELD(AFE4410_OFFDAC_LMSB, 10, 10),
	[F_I_OFFDAC_LED2_LSB_EXT] = REG_FIELD(AFE4410_OFFDAC_LMSB, 11, 11),
	/* FIFO and INT Mux */
	[F_FIFO_PARTITION]	= REG_FIELD(AFE4410_FIFO, 0, 3),
	[F_INT_MUX1]		= REG_FIELD(AFE4410_FIFO, 4, 5),
	[F_REG_FIFO_PERIOD]	= REG_FIELD(AFE4410_FIFO, 6, 13),
	[F_FIFO_EARLY]		= REG_FIELD(AFE4410_FIFO, 14, 18),
	[F_INT_MUX2]		= REG_FIELD(AFE4410_FIFO, 20, 21),
	[F_INT_MUX3]		= REG_FIELD(AFE4410_FIFO, 22, 23),
};

enum afe4410_groups {
	/* Gains */
	G_TIA_GAIN_SEP2, G_TIA_CF_SEP2,
	G_TIA_GAIN_SEP3, G_TIA_CF_SEP3,
	G_TIA_GAIN_SEP, G_TIA_CF_SEP,
	G_TIA_GAIN, G_TIA_CF,

	/* LED Current */
	G_ILED1, G_ILED2, G_ILED3, G_ILED4,

	/* Offset DAC */
	G_OFFDAC_LED2, G_OFFDAC_ALED2,
	G_OFFDAC_LED1, G_OFFDAC_ALED1,

	/* sentinel */
	G_MAX_FIELDS
};

static const unsigned int afe4410_groups[][6] = {
	/* Gains */
	[G_TIA_GAIN_SEP2] = { F_TIA_GAIN_SEP2_LSB, F_TIA_GAIN_SEP2_MSB },
	[G_TIA_CF_SEP2] = { F_TIA_CF_SEP2 },
	[G_TIA_GAIN_SEP3] = { F_TIA_GAIN_SEP3_LSB, F_TIA_GAIN_SEP3_MSB },
	[G_TIA_CF_SEP3] = { F_TIA_CF_SEP3 },
	[G_TIA_GAIN_SEP] = { F_TIA_GAIN_SEP_LSB, F_TIA_GAIN_SEP_MSB },
	[G_TIA_CF_SEP] = { F_TIA_CF_SEP },
	[G_TIA_GAIN] = { F_TIA_GAIN_LSB, F_TIA_GAIN_MSB },
	[G_TIA_CF] = { F_TIA_CF },
	/* LED Current */
	[G_ILED1] = { F_ILED1_LSB, F_ILED1_MSB },
	[G_ILED2] = { F_ILED2_LSB, F_ILED2_MSB },
	[G_ILED3] = { F_ILED3_LSB, F_ILED3_MSB },
	[G_ILED4] = { F_ILED4_LSB, F_ILED4_MSB },
	/* Offset DAC */
	[G_OFFDAC_LED2] = {
		F_I_OFFDAC_LED2_LSB_EXT,
		F_I_OFFDAC_LED2_LSB,
		F_I_OFFDAC_LED2_MID,
		F_I_OFFDAC_LED2_MSB
	},
	[G_OFFDAC_ALED2] = {
		F_I_OFFDAC_LED3_LSB_EXT,
		F_I_OFFDAC_LED3_LSB,
		F_I_OFFDAC_LED3_MID,
		F_I_OFFDAC_LED3_MSB
	},
	[G_OFFDAC_LED1] = {
		F_I_OFFDAC_LED1_LSB_EXT,
		F_I_OFFDAC_LED1_LSB,
		F_I_OFFDAC_LED1_MID,
		F_I_OFFDAC_LED1_MSB
	},
	[G_OFFDAC_ALED1] = {
		F_I_OFFDAC_AMB1_LSB_EXT,
		F_I_OFFDAC_AMB1_LSB,
		F_I_OFFDAC_AMB1_MID,
		F_I_OFFDAC_AMB1_MSB
	},
};

#include "../../base/regmap/internal.h"

int regmap_group_read(struct regmap_field **fields, const unsigned int *group, unsigned int *val)
{
	unsigned int current_bit_shift = 0;
	unsigned int field;
	unsigned int field_val;
	int ret;

	*val = 0;

	while ((field = *group++)) {
		ret = regmap_field_read(fields[field], &field_val);
		if (ret)
			return ret;

		*val |= (field_val << current_bit_shift);
		current_bit_shift += hweight_long(fields[field]->mask);
	}

	return 0;
}

int regmap_group_write(struct regmap_field **fields, const unsigned int *group, unsigned int val)
{
	unsigned int field;
	int ret;

	while ((field = *group++)) {
		ret = regmap_field_write(fields[field], val);
		if (ret)
			return ret;

		val >>= hweight_long(fields[field]->mask);
	}

	return 0;
}

#define FIFO_LEN 10

/**
 * struct afe4410_data - device instance data
 * @dev: Device structure
 * @regmap: Register map of the device
 * @fields: Register fields of the device
 * @regulator: Regulator for the IC
 * @irq: ADC_RDY line interrupt number
 */
struct afe4410_data {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *fields[F_MAX_FIELDS];
	struct regulator *regulator;
	int irq;
	int (*fifo_read)(struct device *dev, s32 *buffer, int len);
	s32 buffer[4 * FIFO_LEN] ____cacheline_aligned; /* (4x24-bit channels + 4x8-bit padding) x FIFO Length */
};

enum afe4410_chan_id {
	LED2 = 0,
	ALED2,
	LED1,
	ALED1,
};

static const unsigned int afe4410_channel_values[] = {
	[LED2] = AFE440X_LED2VAL,
	[ALED2] = AFE440X_ALED2VAL,
	[LED1] = AFE440X_LED1VAL,
	[ALED1] = AFE440X_ALED1VAL,
};

static const unsigned int afe4410_channel_leds[] = {
	[LED2] = G_ILED2,
	[ALED2] = G_ILED3,
	[LED1] = G_ILED1,
	[ALED1] = G_ILED4,
};

static const unsigned int afe4410_channel_offdacs[] = {
	[LED2] = G_OFFDAC_LED2,
	[ALED2] = G_OFFDAC_ALED2,
	[LED1] = G_OFFDAC_LED1,
	[ALED1] = G_OFFDAC_ALED1,
};

static const struct iio_chan_spec afe4410_channels[] = {
	/* ADC values */
	AFE440X_INTENSITY_CHAN(LED2, BIT(IIO_CHAN_INFO_OFFSET)),
	AFE440X_INTENSITY_CHAN(ALED2, BIT(IIO_CHAN_INFO_OFFSET)),
	AFE440X_INTENSITY_CHAN(LED1, BIT(IIO_CHAN_INFO_OFFSET)),
	AFE440X_INTENSITY_CHAN(ALED1, BIT(IIO_CHAN_INFO_OFFSET)),
	/* LED current */
	AFE440X_CURRENT_CHAN(LED2),
	AFE440X_CURRENT_CHAN(ALED2),
	AFE440X_CURRENT_CHAN(LED1),
	AFE440X_CURRENT_CHAN(ALED1),
};

static const struct afe440x_val_table afe4410_res_table[] = {
	{ .integer = 500000, .fract = 0 },
	{ .integer = 250000, .fract = 0 },
	{ .integer = 100000, .fract = 0 },
	{ .integer = 50000, .fract = 0 },
	{ .integer = 25000, .fract = 0 },
	{ .integer = 10000, .fract = 0 },
	{ .integer = 1000000, .fract = 0 },
	{ .integer = 2000000, .fract = 0 },
	{ .integer = 1500000, .fract = 0 },
};
AFE440X_TABLE_ATTR(in_intensity_resistance_available, afe4410_res_table);

static const struct afe440x_val_table afe4410_cap_table[] = {
	{ .integer = 0, .fract = 5000 },
	{ .integer = 0, .fract = 2500 },
	{ .integer = 0, .fract = 10000 },
	{ .integer = 0, .fract = 7500 },
	{ .integer = 0, .fract = 20000 },
	{ .integer = 0, .fract = 17500 },
	{ .integer = 0, .fract = 25000 },
	{ .integer = 0, .fract = 22500 },
};
AFE440X_TABLE_ATTR(in_intensity_capacitance_available, afe4410_cap_table);

static ssize_t afe440x_show_register(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4410_data *afe = iio_priv(indio_dev);
	struct afe440x_attr *afe440x_attr = to_afe440x_attr(attr);
	unsigned int reg_val;
	int vals[2];
	int ret;

	ret = regmap_group_read(afe->fields,
				afe4410_groups[afe440x_attr->field], &reg_val);
	if (ret)
		return ret;

	if (reg_val >= afe440x_attr->table_size)
		return -EINVAL;

	vals[0] = afe440x_attr->val_table[reg_val].integer;
	vals[1] = afe440x_attr->val_table[reg_val].fract;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, 2, vals);
}

static ssize_t afe440x_store_register(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4410_data *afe = iio_priv(indio_dev);
	struct afe440x_attr *afe440x_attr = to_afe440x_attr(attr);
	int val, integer, fract, ret;

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &fract);
	if (ret)
		return ret;

	for (val = 0; val < afe440x_attr->table_size; val++)
		if (afe440x_attr->val_table[val].integer == integer &&
		    afe440x_attr->val_table[val].fract == fract)
			break;
	if (val == afe440x_attr->table_size)
		return -EINVAL;

	ret = regmap_group_write(afe->fields,
				 afe4410_groups[afe440x_attr->field], val);
	if (ret)
		return ret;

	return count;
}

//#define AFE4410_RATE_400	0x0
//#define AFE4410_RATE_200	0x4
//#define AFE4410_RATE_100	0x5
//#define AFE4410_RATE_50	0x6
//#define AFE4410_RATE_25	0x7
//
//static const char * const afe4410_sample_freq_avail[] = {
//	[AFE4410_RATE_400] = "400",
//	[0x1] = "0",
//	[0x2] = "0",
//	[0x3] = "0",
//	[AFE4410_RATE_200] = "200",
//	[AFE4410_RATE_100] = "100",
//	[AFE4410_RATE_50] = "50",
//	[AFE4410_RATE_25] = "25",
//};
//
//static ssize_t afe4410_read_frequency(struct device *dev,
//				      struct device_attribute *attr, char *buf)
//{
//	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
//	struct afe4410_data *afe = iio_priv(indio_dev);
//	unsigned int rate;
//	int ret;
//
//	ret = regmap_read(afe->regmap, AFE4410_CLKDIV_PRF, &rate);
//	if (ret)
//		return ret;
//
//	if (rate > AFE4410_RATE_25)
//		return -EINVAL;
//
//	return sprintf(buf, "%s\n", afe4410_sample_freq_avail[rate]);
//}
//
//static ssize_t afe4410_write_frequency(struct device *dev,
//				       struct device_attribute *attr,
//				       const char *buf, size_t len)
//{
//	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
//	struct afe4410_data *afe = iio_priv(indio_dev);
//	int i, ret;
//
//	/* TODO: Find a better way to search and check for 0, cache this write */
//	for (i = 0; i < ARRAY_SIZE(afe4410_sample_freq_avail); i++)
//		if (sysfs_streq(afe4410_sample_freq_avail[i], buf))
//			break;
//	if (i == ARRAY_SIZE(afe4410_sample_freq_avail))
//		return -EINVAL;
//
//	ret = regmap_write(afe->regmap, AFE4410_CLKDIV_PRF, i);
//	if (ret)
//		return ret;
//
//	return len;
//}

static AFE440X_ATTR(in_intensity0_resistance, G_TIA_GAIN_SEP, afe4410_res_table);
static AFE440X_ATTR(in_intensity0_capacitance, G_TIA_CF_SEP, afe4410_cap_table);

static AFE440X_ATTR(in_intensity1_resistance, G_TIA_GAIN_SEP2, afe4410_res_table);
static AFE440X_ATTR(in_intensity1_capacitance, G_TIA_CF_SEP2, afe4410_cap_table);

static AFE440X_ATTR(in_intensity2_resistance, G_TIA_GAIN, afe4410_res_table);
static AFE440X_ATTR(in_intensity2_capacitance, G_TIA_CF, afe4410_cap_table);

static AFE440X_ATTR(in_intensity3_resistance, G_TIA_GAIN_SEP3, afe4410_res_table);
static AFE440X_ATTR(in_intensity3_capacitance, G_TIA_CF_SEP3, afe4410_cap_table);

//static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
//		afe4410_read_frequency,
//		afe4410_write_frequency);
//
//static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("400 200 100 50 25");

static struct attribute *afe4410_attributes[] = {
	&dev_attr_in_intensity_resistance_available.attr,
	&dev_attr_in_intensity_capacitance_available.attr,
	&afe440x_attr_in_intensity0_resistance.dev_attr.attr,
	&afe440x_attr_in_intensity0_capacitance.dev_attr.attr,
	&afe440x_attr_in_intensity1_resistance.dev_attr.attr,
	&afe440x_attr_in_intensity1_capacitance.dev_attr.attr,
	&afe440x_attr_in_intensity2_resistance.dev_attr.attr,
	&afe440x_attr_in_intensity2_capacitance.dev_attr.attr,
	&afe440x_attr_in_intensity3_resistance.dev_attr.attr,
	&afe440x_attr_in_intensity3_capacitance.dev_attr.attr,
//	&iio_dev_attr_sampling_frequency.dev_attr.attr,
//	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group afe4410_attribute_group = {
	.attrs = afe4410_attributes
};

static int afe4410_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct afe4410_data *afe = iio_priv(indio_dev);
	unsigned int value_reg = afe4410_channel_values[chan->address];
	unsigned int led_group = afe4410_channel_leds[chan->address];
	unsigned int offdac_group = afe4410_channel_offdacs[chan->address];
	int ret;

	switch (chan->type) {
	case IIO_INTENSITY:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			/* TODO: iio_device_claim_direct_mode */
			ret = regmap_read(afe->regmap, value_reg, val);
			if (ret)
				return ret;
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_OFFSET:
			ret = regmap_group_read(afe->fields,
						afe4410_groups[offdac_group], val);
			if (ret)
				return ret;
			return IIO_VAL_INT;
		}
		break;
	case IIO_CURRENT:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = regmap_group_read(afe->fields,
						afe4410_groups[led_group], val);
			if (ret)
				return ret;
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			*val = 0;
			*val2 = 200000;
			return IIO_VAL_INT_PLUS_MICRO;
		}
		break;
	default:
		break;
	}

	return -EINVAL;
}

static int afe4410_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct afe4410_data *afe = iio_priv(indio_dev);
	unsigned int led_group = afe4410_channel_leds[chan->address];
	unsigned int offdac_group = afe4410_channel_offdacs[chan->address];

	switch (chan->type) {
	case IIO_INTENSITY:
		switch (mask) {
		case IIO_CHAN_INFO_OFFSET:
			return regmap_group_write(afe->fields,
						  afe4410_groups[offdac_group], val);
		}
		break;
	case IIO_CURRENT:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			return regmap_group_write(afe->fields,
						  afe4410_groups[led_group], val);
		}
		break;
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info afe4410_iio_info = {
	.attrs = &afe4410_attribute_group,
	.read_raw = afe4410_read_raw,
	.write_raw = afe4410_write_raw,
	.driver_module = THIS_MODULE,
};

static const unsigned long afe4410_scan_masks[] = {
	BIT(LED2) | BIT(ALED2) | BIT(LED1) | BIT(ALED1),
	0
};

int afe4410_buffer_postenable(struct iio_dev *indio_dev)
{
	struct afe4410_data *afe = iio_priv(indio_dev);

	/* Turn on FIFO buffer */
	regmap_update_bits(afe->regmap, AFE440X_CONTROL0,
			   AFE440X_CONTROL0_FIFO_EN,
			   AFE440X_CONTROL0_FIFO_EN);

	/* Start device sequence timer */
	regmap_update_bits(afe->regmap, AFE440X_CONTROL1,
			   AFE440X_CONTROL1_TIMEREN,
			   AFE440X_CONTROL1_TIMEREN);

	return 0;
}

int afe4410_buffer_predisable(struct iio_dev *indio_dev)
{
	struct afe4410_data *afe = iio_priv(indio_dev);

	regmap_update_bits(afe->regmap, AFE440X_CONTROL1,
			   AFE440X_CONTROL1_TIMEREN, 0);

	regmap_update_bits(afe->regmap, AFE440X_CONTROL0,
			   AFE440X_CONTROL0_FIFO_EN, 0);

	return 0;
}

static const struct iio_buffer_setup_ops afe4410_buffer_setup_ops = {
	.postenable = &afe4410_buffer_postenable,
	.predisable = &afe4410_buffer_predisable,
};

static irqreturn_t afe4410_trigger_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct afe4410_data *afe = iio_priv(indio_dev);
	int i, ret;

	ret = afe->fifo_read(afe->dev, afe->buffer, sizeof(afe->buffer));
	if (ret < 0)
		goto err;

	for (i = 0; i < FIFO_LEN; i++)
		iio_push_to_buffers(indio_dev, &afe->buffer[i * 4]);

err:
	return IRQ_HANDLED;
}

/* Default timings from data-sheet */
#define AFE4410_TIMING_PAIRS		\
	{ AFE440X_LED2STC,		10 * 0x01	},	\
	{ AFE440X_LED2ENDC,		10 * 0x03	},	\
	{ AFE440X_LED1LEDSTC,		10 * 0x0a	},	\
	{ AFE440X_LED1LEDENDC,		10 * 0x0d	},	\
	{ AFE440X_ALED2STC,		10 * 0x06	},	\
	{ AFE440X_ALED2ENDC,		10 * 0x08	},	\
	{ AFE440X_LED1STC,		10 * 0x0b	},	\
	{ AFE440X_LED1ENDC,		10 * 0x0d	},	\
	{ AFE440X_LED2LEDSTC,		10 * 0x00	},	\
	{ AFE440X_LED2LEDENDC,		10 * 0x03	},	\
	{ AFE440X_ALED1STC,		10 * 0x10	},	\
	{ AFE440X_ALED1ENDC,		10 * 0x12	},	\
	{ AFE440X_LED2CONVST,		10 * 0x05	},	\
	{ AFE440X_LED2CONVEND,		10 * 0x08	},	\
	{ AFE440X_ALED2CONVST,		10 * 0x0a	},	\
	{ AFE440X_ALED2CONVEND,		10 * 0x0d	},	\
	{ AFE440X_LED1CONVST,		10 * 0x0f	},	\
	{ AFE440X_LED1CONVEND,		10 * 0x12	},	\
	{ AFE440X_ALED1CONVST,		10 * 0x14	},	\
	{ AFE440X_ALED1CONVEND,		10 * 0x17	},	\
	{ AFE440X_PRPCOUNT,		10 * 0x1f	},	\
	{ AFE4410_LED3LEDSTC,		10 * 0x05	},	\
	{ AFE4410_LED3LEDENDC,		10 * 0x08	},	\
	{ AFE4410_LED4LEDSTC,		10 * 0x0f	},	\
	{ AFE4410_LED4LEDENDC,		10 * 0x12	},	\
	{ AFE4410_DATA_RDY_STC, 	10 * 0x1d	},	\
	{ AFE4410_DATA_RDY_ENDC,	10 * 0x1d	},	\
	{ AFE4410_DYN_TIA_STC,		10 * 0x00	},	\
	{ AFE4410_DYN_TIA_ENDC,		10 * 0x20	},	\
	{ AFE4410_DYN_ADC_STC,		10 * 0x00	},	\
	{ AFE4410_DYN_ADC_ENDC,		10 * 0x20	},	\
	{ AFE4410_DYN_CLK_STC,		10 * 0x00	},	\
	{ AFE4410_DYN_CLK_ENDC,		10 * 0x20	},	\
	{ AFE4410_DEEP_SLEEP_STC,	10 * 0x21	},	\
	{ AFE4410_DEEP_SLEEP_ENDC,	10 * 0x18	}

static const struct reg_sequence afe4410_reg_sequences[] = {
	{ AFE440X_CONTROL0, AFE440X_CONTROL0_RW_CONT |
			    AFE440X_CONTROL0_ENABLE_ULP },
	AFE4410_TIMING_PAIRS,
	{ AFE4410_TIA_GAIN_SEP, AFE440X_TIAGAIN_ENSEPGAIN },
	{ AFE440X_CONTROL2, AFE440X_CONTROL2_DYN_ADC |
	                    AFE440X_CONTROL2_DYN_TIA |
	                    AFE440X_CONTROL2_OSC_ENABLE |
	                    AFE440X_CONTROL2_DYN_BIAS |
	                    AFE440X_CONTROL2_ENSEPGAIN4 |
	                    AFE440X_CONTROL2_DYN_TX0 },
	{ AFE4410_FIFO, 0x260 },
};

static const struct regmap_range afe4410_yes_ranges[] = {
	regmap_reg_range(AFE440X_LED2VAL, AFE440X_LED1_ALED1VAL),
	regmap_reg_range(AFE4410_AVG_LED2_ALED2VAL, AFE4410_AVG_LED1_ALED1VAL),
};

static const struct regmap_access_table afe4410_volatile_table = {
	.yes_ranges = afe4410_yes_ranges,
	.n_yes_ranges = ARRAY_SIZE(afe4410_yes_ranges),
};

const struct regmap_config afe4410_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,

	.zero_flag_mask = true,
	.max_register = AFE4410_DEEP_SLEEP_ENDC,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &afe4410_volatile_table,
};
EXPORT_SYMBOL_GPL(afe4410_regmap_config);

const struct of_device_id afe4410_of_match[] = {
	{ .compatible = "ti,afe4410", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, afe4410_of_match);
EXPORT_SYMBOL_GPL(afe4410_of_match);

static int __maybe_unused afe4410_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4410_data *afe = iio_priv(indio_dev);
	int ret;

	ret = regmap_update_bits(afe->regmap, AFE440X_CONTROL2,
				 AFE440X_CONTROL2_PDN_AFE,
				 AFE440X_CONTROL2_PDN_AFE);
	if (ret)
		return ret;

	ret = regulator_disable(afe->regulator);
	if (ret) {
		dev_err(dev, "Unable to disable regulator\n");
		return ret;
	}

	return 0;
}

static int __maybe_unused afe4410_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4410_data *afe = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(afe->regulator);
	if (ret) {
		dev_err(dev, "Unable to enable regulator\n");
		return ret;
	}

	ret = regmap_update_bits(afe->regmap, AFE440X_CONTROL2,
				 AFE440X_CONTROL2_PDN_AFE, 0);
	if (ret)
		return ret;

	return 0;
}

SIMPLE_DEV_PM_OPS(afe4410_pm_ops, afe4410_suspend, afe4410_resume);
EXPORT_SYMBOL_GPL(afe4410_pm_ops);

int afe4410_setup(struct regmap *regmap, int irq,
		int (*fifo_read)(struct device *dev, s32 *buffer, int len))
{
	struct device *dev = regmap_get_device(regmap);
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct afe4410_data *afe;
	int i, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*afe));
	if (!indio_dev)
		return -ENOMEM;

	afe = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	afe->dev = dev;
	afe->regmap = regmap;
	afe->irq = irq;
	afe->fifo_read = fifo_read;

	for (i = 0; i < F_MAX_FIELDS; i++) {
		afe->fields[i] = devm_regmap_field_alloc(afe->dev, afe->regmap, afe4410_reg_fields[i]);
		if (IS_ERR(afe->fields[i])) {
			dev_err(afe->dev, "Unable to allocate regmap fields\n");
			return PTR_ERR(afe->fields[i]);
		}
	}

	afe->regulator = devm_regulator_get(afe->dev, "tx_sup");
	if (IS_ERR(afe->regulator)) {
		dev_err(afe->dev, "Unable to get regulator\n");
		return PTR_ERR(afe->regulator);
	}
	ret = regulator_enable(afe->regulator);
	if (ret) {
		dev_err(afe->dev, "Unable to enable regulator\n");
		return ret;
	}

	ret = regmap_write(afe->regmap, AFE440X_CONTROL0,
			   AFE440X_CONTROL0_SW_RESET);
	if (ret) {
		dev_err(afe->dev, "Unable to reset device\n");
		goto disable_reg;
	}

	ret = regmap_multi_reg_write(afe->regmap, afe4410_reg_sequences,
				     ARRAY_SIZE(afe4410_reg_sequences));
	if (ret) {
		dev_err(afe->dev, "Unable to set register defaults\n");
		goto disable_reg;
	}

	/* TODO: Allow positive offset DAC values */
	regmap_field_write(afe->fields[F_POL_OFFDAC_LED3], 0x1);
	regmap_field_write(afe->fields[F_POL_OFFDAC_LED1], 0x1);
	regmap_field_write(afe->fields[F_POL_OFFDAC_AMB1], 0x1);
	regmap_field_write(afe->fields[F_POL_OFFDAC_LED2], 0x1);

	indio_dev->modes = (INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE);
	indio_dev->dev.parent = afe->dev;
	indio_dev->channels = afe4410_channels;
	indio_dev->num_channels = ARRAY_SIZE(afe4410_channels);
	indio_dev->name = AFE4410_DRIVER_NAME;
	indio_dev->info = &afe4410_iio_info;
	indio_dev->available_scan_masks = afe4410_scan_masks;
	indio_dev->setup_ops = &afe4410_buffer_setup_ops;

	if (afe->irq > 0) {
		buffer = devm_iio_kfifo_allocate(afe->dev);
		if (!buffer) {
			ret = -ENOMEM;
			goto disable_reg;
		}

		iio_device_attach_buffer(indio_dev, buffer);

		ret = devm_request_threaded_irq(afe->dev, afe->irq,
						NULL, afe4410_trigger_handler,
						IRQF_ONESHOT,
						AFE4410_DRIVER_NAME,
						indio_dev);
		if (ret) {
			dev_err(afe->dev, "Unable to request IRQ\n");
			goto disable_reg;
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(afe->dev, "Unable to register IIO device\n");
		goto disable_reg;
	}

	return 0;

disable_reg:
	regulator_disable(afe->regulator);

	return ret;
}
EXPORT_SYMBOL_GPL(afe4410_setup);

int afe4410_teardown(struct regmap *regmap)
{
	struct device *dev = regmap_get_device(regmap);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct afe4410_data *afe = iio_priv(indio_dev);
	int ret;

	iio_device_unregister(indio_dev);

	ret = regulator_disable(afe->regulator);
	if (ret) {
		dev_err(afe->dev, "Unable to disable regulator\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(afe4410_teardown);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI AFE4410 Heart Rate Monitor and Pulse Oximeter AFE");
MODULE_LICENSE("GPL v2");
