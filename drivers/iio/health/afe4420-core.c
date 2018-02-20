// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4420 Optical Heart-Rate Monitor and Bio-Sensor
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
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

#include "afe4420.h"

#define AFE4420_DRIVER_NAME		"afe4420"

/* AFE4420 registers */
#define AFE4420_CONTROL0		0x00
#define AFE4420_PRPCOUNT		0x1d
#define AFE4420_CONTROL1		0x23
#define AFE4420_FIFO			0x42
#define AFE4420_POINTER_DIFF		0x6d
#define AFE4420_PHASE			0x88
#define AFE4420_AACM			0x93
#define AFE4420_PDCNTRL0(x)		(0x98 + (x * 4))
#define AFE4420_PDCNTRL1(x)		(0x99 + (x * 4))
#define AFE4420_PDCNTRL2(x)		(0x9a + (x * 4))
#define AFE4420_LEDCNTRL1		0xac
#define AFE4420_LEDCNTRL2		0xae
#define AFE4420_PHASECNTRL0(x)		(0xb8 + (x * 3))
#define AFE4420_PHASECNTRL1(x)		(0xb9 + (x * 3))
#define AFE4420_PHASECNTRL2(x)		(0xba + (x * 3))

/* AFE4420 CONTROL0 register fields */
#define AFE4420_CONTROL0_REG_READ	BIT(0)
#define AFE4420_CONTROL0_TM_COUNT_RST	BIT(1)
#define AFE4420_CONTROL0_SW_RESET	BIT(3)
#define AFE4420_CONTROL0_RW_CONT	BIT(4)
#define AFE4420_CONTROL0_FIFO_EN	BIT(6)

/* AFE4420 PRPCOUNT register fields */
#define AFE4420_PRPCOUNT_PRPCT		GENMASK(15, 0)
#define AFE4420_PRPCOUNT_TIMEREN	BIT(23)

/* AFE4420 CONTROL1 register fields */
#define AFE4420_CONTROL1_OSC_DISABLE	BIT(9)
#define AFE4420_CONTROL1_IFS_OFFDAC	GENMASK(12, 10)
#define AFE4420_CONTROL1_EN_AACM_GBL	BIT(15)
#define AFE4420_CONTROL1_ILED_2X	BIT(17)
#define AFE4420_CONTROL1_PD_DISCONNECT	BIT(23)

/* AFE4420 FIFO register fields */
#define AFE4420_FIFO_INT_MUX_ADY_RDY	GENMASK(5, 4)
#define AFE4420_FIFO_INT_MUX_DATA_RDY		0x00
#define AFE4420_FIFO_INT_MUX_THR_DET_RDY	0x10
#define AFE4420_FIFO_INT_MUX_FIFO_RDY		0x20
#define AFE4420_FIFO_REG_WM_FIFO	GENMASK(13, 6)
#define AFE4420_FIFO_INT_MUX_GPIO1	GENMASK(21, 20)
#define AFE4420_FIFO_INT_MUX_SDOUT	GENMASK(23, 22)

/* AFE4420 AACM register fields */
#define AFE4420_AACM_IMM_REFRESH	BIT(0)
#define AFE4420_AACM_QUICK_CONV		BIT(1)

/* AFE4420 PHASE register fields */
#define AFE4420_PHASE_REG_NUMPHASE	GENMASK(3, 0)
#define AFE4420_PHASE_FILT1_RESET_ENZ	BIT(16)
#define AFE4420_PHASE_FILT2_RESET_ENZ	BIT(17)
#define AFE4420_PHASE_FILT3_RESET_ENZ	BIT(18)
#define AFE4420_PHASE_FILT4_RESET_ENZ	BIT(19)

/* AFE4420 PHASECNTRL0 register fields */
#define AFE4420_PHASECNTRL0_LED_DRV1_TX1	BIT(0)
#define AFE4420_PHASECNTRL0_LED_DRV1_TX2	BIT(1)
#define AFE4420_PHASECNTRL0_LED_DRV1_TX3	BIT(2)
#define AFE4420_PHASECNTRL0_LED_DRV1_TX4	BIT(3)
#define AFE4420_PHASECNTRL0_LED_DRV2_TX1	BIT(8)
#define AFE4420_PHASECNTRL0_LED_DRV2_TX2	BIT(9)
#define AFE4420_PHASECNTRL0_LED_DRV2_TX3	BIT(10)
#define AFE4420_PHASECNTRL0_LED_DRV2_TX4	BIT(11)
#define AFE4420_PHASECNTRL0_PD_ON1	BIT(16)
#define AFE4420_PHASECNTRL0_PD_ON2	BIT(17)
#define AFE4420_PHASECNTRL0_PD_ON3	BIT(18)
#define AFE4420_PHASECNTRL0_PD_ON4	BIT(19)

/* AFE4420 PHASECNTRL2 register fields */
#define AFE4420_PHASECNTRL2_TWLED	GENMASK(7, 0)
#define AFE4420_PHASECNTRL2_MASK_FACTOR	GENMASK(10, 9)
#define AFE4420_PHASECNTRL2_STAGGER_LED	BIT(12)
#define AFE4420_PHASECNTRL2_THR_SEL_DATA_CTRL	GENMASK(16, 15)
#define AFE4420_PHASECNTRL2_FIFO_DATA_CTRL	GENMASK(18, 17)

#define AFE4420_TOTAL_PHASES 16
#define AFE4420_FIFO_MAX_SAMPLES 128

#define AFE4420_FIFO_LEN 10

#define AFE4420_DEFAULT_PRPCOUNT 0x13ff
#define AFE4420_DEFAULT_TWLED 0x6

enum afe4420_pds {
	PD1 = 0,
	PD2,
	PD3,
	PD4,
};

enum afe4420_phases {
	PHASE1 = 0,
	PHASE2,
	PHASE3,
	PHASE4,
	PHASE5,
	PHASE6,
	PHASE7,
	PHASE8,
	PHASE9,
	PHASE10,
	PHASE11,
	PHASE12,
	PHASE13,
	PHASE14,
	PHASE15,
	PHASE16,
};

#define PD_FIELDS(pd)			\
	/* Setup */			\
	F_EN_AACM_##pd,			\
	F_NUMPHASE_AACM_##pd,		\
	F_FREEZE_AACM_##pd,		\
	F_IOFFDAC_BASE_##pd,		\
	F_POL_OFFDAC_BASE_##pd,		\
	/* Calibration */		\
	F_CALIB_AACM_##pd,		\
	/* Calibration input */		\
	F_IOFFDAC_AACM_READ_##pd,	\
	F_POL_OFFDAC_AACM_READ_##pd

#define PHASE_FIELDS(phase)		\
	/* Averaging */			\
	F_NUMAV_##phase,		\
	/* Gains */			\
	F_TIA_GAIN_RF_##phase,		\
	F_TIA_GAIN_CF_##phase,		\
	/* Offset DAC */		\
	F_I_OFFDAC_##phase,		\
	F_POL_OFFDAC_##phase

/* AFE4420 fields */
enum afe4420_fields {
	F_IFS_OFFDAC,
	F_PD_DISCONNECT,
	/* Watermark level */
	F_WM_FIFO,
	/* Number of active phases */
	F_NUMPHASE,
	/* Offset for AACM calibration */
	F_CHANNEL_OFFSET_AACM,
	/* PD Fields */
	PD_FIELDS(PD1),
	PD_FIELDS(PD2),
	PD_FIELDS(PD3),
	PD_FIELDS(PD4),
	/* LED Current */
	F_ILED_TX1,
	F_ILED_TX2,
	F_ILED_TX3,
	F_ILED_TX4,
	/* Phase Fields */
	PHASE_FIELDS(PHASE1),
	PHASE_FIELDS(PHASE2),
	PHASE_FIELDS(PHASE3),
	PHASE_FIELDS(PHASE4),
	PHASE_FIELDS(PHASE5),
	PHASE_FIELDS(PHASE6),
	PHASE_FIELDS(PHASE7),
	PHASE_FIELDS(PHASE8),
	PHASE_FIELDS(PHASE9),
	PHASE_FIELDS(PHASE10),
	PHASE_FIELDS(PHASE11),
	PHASE_FIELDS(PHASE12),
	PHASE_FIELDS(PHASE13),
	PHASE_FIELDS(PHASE14),
	PHASE_FIELDS(PHASE15),
	PHASE_FIELDS(PHASE16),
	/* sentinel */
	F_MAX_FIELDS
};

#define PD_REG_FIELDS(pd)								\
	/* Setup */									\
	[F_EN_AACM_##pd]		= REG_FIELD(AFE4420_PDCNTRL0(pd), 0, 0),	\
	[F_NUMPHASE_AACM_##pd]		= REG_FIELD(AFE4420_PDCNTRL0(pd), 4, 7),	\
	[F_FREEZE_AACM_##pd]		= REG_FIELD(AFE4420_PDCNTRL0(pd), 10, 10),	\
	[F_IOFFDAC_BASE_##pd]		= REG_FIELD(AFE4420_PDCNTRL0(pd), 16, 22),	\
	[F_POL_OFFDAC_BASE_##pd]	= REG_FIELD(AFE4420_PDCNTRL0(pd), 23, 23),	\
	/* Calibration */								\
	[F_CALIB_AACM_##pd]		= REG_FIELD(AFE4420_PDCNTRL1(pd), 0, 11),	\
	/* Calibration input */								\
	[F_IOFFDAC_AACM_READ_##pd]	= REG_FIELD(AFE4420_PDCNTRL2(pd), 1, 7),	\
	[F_POL_OFFDAC_AACM_READ_##pd]	= REG_FIELD(AFE4420_PDCNTRL2(pd), 8, 8)

#define PHASE_REG_FIELDS(phase)								\
	/* Averaging */									\
	[F_NUMAV_##phase]	= REG_FIELD(AFE4420_PHASECNTRL1(phase), 0, 3),		\
	/* Gains */									\
	[F_TIA_GAIN_RF_##phase]	= REG_FIELD(AFE4420_PHASECNTRL1(phase), 4, 7),		\
	[F_TIA_GAIN_CF_##phase]	= REG_FIELD(AFE4420_PHASECNTRL1(phase), 10, 12),	\
	/* Offset DAC */								\
	[F_I_OFFDAC_##phase]	= REG_FIELD(AFE4420_PHASECNTRL1(phase), 16, 22),	\
	[F_POL_OFFDAC_##phase]	= REG_FIELD(AFE4420_PHASECNTRL1(phase), 23, 23)

static const struct reg_field afe4420_reg_fields[] = {
	[F_IFS_OFFDAC]	= REG_FIELD(AFE4420_CONTROL1, 10, 12),
	[F_PD_DISCONNECT]	= REG_FIELD(AFE4420_CONTROL1, 23, 23),
	/* Watermark level */
	[F_WM_FIFO]	= REG_FIELD(AFE4420_FIFO, 6, 13),
	/* Number of active phases */
	[F_NUMPHASE]	= REG_FIELD(AFE4420_PHASE, 0, 3),
	/* Offset for AACM calibration */
	[F_CHANNEL_OFFSET_AACM] = REG_FIELD(AFE4420_AACM, 8, 20),
	/* PD Fields */
	PD_REG_FIELDS(PD1),
	PD_REG_FIELDS(PD2),
	PD_REG_FIELDS(PD3),
	PD_REG_FIELDS(PD4),
	/* LED Current */
	[F_ILED_TX1]	= REG_FIELD(AFE4420_LEDCNTRL1, 0, 7),
	[F_ILED_TX2]	= REG_FIELD(AFE4420_LEDCNTRL1, 12, 19),
	[F_ILED_TX3]	= REG_FIELD(AFE4420_LEDCNTRL2, 0, 7),
	[F_ILED_TX4]	= REG_FIELD(AFE4420_LEDCNTRL2, 12, 19),
	/* Phase Fields */
	PHASE_REG_FIELDS(PHASE1),
	PHASE_REG_FIELDS(PHASE2),
	PHASE_REG_FIELDS(PHASE3),
	PHASE_REG_FIELDS(PHASE4),
	PHASE_REG_FIELDS(PHASE5),
	PHASE_REG_FIELDS(PHASE6),
	PHASE_REG_FIELDS(PHASE7),
	PHASE_REG_FIELDS(PHASE8),
	PHASE_REG_FIELDS(PHASE9),
	PHASE_REG_FIELDS(PHASE10),
	PHASE_REG_FIELDS(PHASE11),
	PHASE_REG_FIELDS(PHASE12),
	PHASE_REG_FIELDS(PHASE13),
	PHASE_REG_FIELDS(PHASE14),
	PHASE_REG_FIELDS(PHASE15),
	PHASE_REG_FIELDS(PHASE16),
};

/**
 * struct afe4420_data - device instance data
 * @dev: Device structure
 * @regmap: Register map of the device
 * @fields: Register fields of the device
 * @regulator: Regulator for the IC
 * @reset_gpio: Reset GPIO pin for device
 * @irq: ADC_RDY line interrupt number
 * @fifo_read: Callback to interface for FIFO reads
 * @buffer: Buffer to read FIFO into
 */
struct afe4420_data {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *fields[F_MAX_FIELDS];
	struct regulator *regulator;
	struct gpio_desc *reset_gpio;
	int irq;
	unsigned int used_phases;
	int (*fifo_read)(struct device *dev, s32 *buffer, int len);
	/* Maximum samples FIFO can hold x (24-bit channels + 8-bit padding) */
	s8 buffer[AFE4420_FIFO_MAX_SAMPLES * 4] ____cacheline_aligned;
};

enum afe4420_led_chan_id {
	LED1 = 0,
	LED2,
	LED3,
	LED4,
};

static const unsigned int afe4420_channel_leds[] = {
	[LED1] = F_ILED_TX1,
	[LED2] = F_ILED_TX2,
	[LED3] = F_ILED_TX3,
	[LED4] = F_ILED_TX4,
};

static const struct iio_chan_spec afe4420_channels[] = {
	/* ADC values */
	AFE4420_INTENSITY_CHAN(PHASE1),
	AFE4420_INTENSITY_CHAN(PHASE2),
	AFE4420_INTENSITY_CHAN(PHASE3),
	AFE4420_INTENSITY_CHAN(PHASE4),
	AFE4420_INTENSITY_CHAN(PHASE5),
	AFE4420_INTENSITY_CHAN(PHASE6),
	AFE4420_INTENSITY_CHAN(PHASE7),
	AFE4420_INTENSITY_CHAN(PHASE8),
	AFE4420_INTENSITY_CHAN(PHASE9),
	AFE4420_INTENSITY_CHAN(PHASE10),
	AFE4420_INTENSITY_CHAN(PHASE11),
	AFE4420_INTENSITY_CHAN(PHASE12),
	AFE4420_INTENSITY_CHAN(PHASE13),
	AFE4420_INTENSITY_CHAN(PHASE14),
	AFE4420_INTENSITY_CHAN(PHASE15),
	AFE4420_INTENSITY_CHAN(PHASE16),

	/* LED current */
	AFE4420_CURRENT_CHAN(LED1),
	AFE4420_CURRENT_CHAN(LED2),
	AFE4420_CURRENT_CHAN(LED3),
	AFE4420_CURRENT_CHAN(LED4),
};

static IIO_CONST_ATTR(in_intensity_averages_available,
		      "1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16");

static const struct afe4420_val_table afe4420_res_table[] = {
	{ .integer = 10000, .fract = 0 },
	{ .integer = 25000, .fract = 0 },
	{ .integer = 50000, .fract = 0 },
	{ .integer = 100000, .fract = 0 },
	{ .integer = 166000, .fract = 0 },
	{ .integer = 200000, .fract = 0 },
	{ .integer = 250000, .fract = 0 },
	{ .integer = 500000, .fract = 0 },
	{ .integer = 1000000, .fract = 0 },
	{ .integer = 1500000, .fract = 0 },
	{ .integer = 2000000, .fract = 0 },
};
AFE4420_TABLE_ATTR(in_intensity_resistance_available, afe4420_res_table);

static const struct afe4420_val_table afe4420_cap_table[] = {
	{ .integer = 0, .fract = 2500 },
	{ .integer = 0, .fract = 5000 },
	{ .integer = 0, .fract = 7500 },
	{ .integer = 0, .fract = 10000 },
	{ .integer = 0, .fract = 17500 },
	{ .integer = 0, .fract = 20000 },
	{ .integer = 0, .fract = 22500 },
	{ .integer = 0, .fract = 25000 },
};
AFE4420_TABLE_ATTR(in_intensity_capacitance_available, afe4420_cap_table);

static ssize_t afe4420_show_averages(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	unsigned int reg_val, val;
	int ret;

	ret = regmap_field_read(afe->fields[afe4420_attr->field], &reg_val);
	if (ret)
		return ret;

	val = reg_val + 1;

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t afe4420_store_averages(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	int val, reg_val;
	int ret;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (val < 1 || val > 16)
		return -EINVAL;

	reg_val = val - 1;

	ret = regmap_field_write(afe->fields[afe4420_attr->field], reg_val);
	if (ret)
		return ret;

	return count;
}

static ssize_t afe4420_show_resistance(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	unsigned int reg_val;
	int vals[2];
	int ret;

	ret = regmap_field_read(afe->fields[afe4420_attr->field], &reg_val);
	if (ret)
		return ret;

	if (reg_val >= ARRAY_SIZE(afe4420_res_table))
		return -EINVAL;

	vals[0] = afe4420_res_table[reg_val].integer;
	vals[1] = afe4420_res_table[reg_val].fract;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, 2, vals);
}

static ssize_t afe4420_store_resistance(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	int val, integer, fract, ret;

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &fract);
	if (ret)
		return ret;

	for (val = 0; val < ARRAY_SIZE(afe4420_res_table); val++)
		if (afe4420_res_table[val].integer == integer &&
		    afe4420_res_table[val].fract == fract)
			break;
	if (val == ARRAY_SIZE(afe4420_res_table))
		return -EINVAL;

	ret = regmap_field_write(afe->fields[afe4420_attr->field], val);
	if (ret)
		return ret;

	return count;
}

static ssize_t afe4420_show_capacitance(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	unsigned int reg_val;
	int vals[2];
	int ret;

	ret = regmap_field_read(afe->fields[afe4420_attr->field], &reg_val);
	if (ret)
		return ret;

	if (reg_val >= ARRAY_SIZE(afe4420_cap_table))
		return -EINVAL;

	vals[0] = afe4420_cap_table[reg_val].integer;
	vals[1] = afe4420_cap_table[reg_val].fract;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, 2, vals);
}

static ssize_t afe4420_store_capacitance(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	int val, integer, fract, ret;

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &fract);
	if (ret)
		return ret;

	for (val = 0; val < ARRAY_SIZE(afe4420_cap_table); val++)
		if (afe4420_cap_table[val].integer == integer &&
		    afe4420_cap_table[val].fract == fract)
			break;
	if (val == ARRAY_SIZE(afe4420_cap_table))
		return -EINVAL;

	ret = regmap_field_write(afe->fields[afe4420_attr->field], val);
	if (ret)
		return ret;

	return count;
}

static ssize_t afe4420_show_ioffdac(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	unsigned int reg_val;
	int ret;

	ret = regmap_field_read(afe->fields[afe4420_attr->field], &reg_val);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%u\n", reg_val);
}

static ssize_t afe4420_store_ioffdac(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	int val, ret;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	ret = regmap_field_write(afe->fields[afe4420_attr->field], val);
	if (ret)
		return ret;

	return count;
}

static ssize_t afe4420_show_poloffdac(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	unsigned int reg_val;
	int ret;

	ret = regmap_field_read(afe->fields[afe4420_attr->field], &reg_val);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%u\n", reg_val);
}

static ssize_t afe4420_store_poloffdac(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	struct afe4420_attr *afe4420_attr = to_afe4420_attr(attr);
	int val, ret;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	ret = regmap_field_write(afe->fields[afe4420_attr->field], val);
	if (ret)
		return ret;

	return count;
}

static AFE4420_ATTR(pd_disconnect, ioffdac, F_PD_DISCONNECT, 0);
static AFE4420_ATTR(ifs_offdac, ioffdac, F_IFS_OFFDAC, 0);
static AFE4420_ATTR(channel_offset_aacm, ioffdac, F_CHANNEL_OFFSET_AACM, 0);

#define PD_ATTRS(num, pd) \
static AFE4420_ATTR(in_pd##num##_en_aacm, ioffdac, F_EN_AACM_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_numphase_aacm, ioffdac, F_NUMPHASE_AACM_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_freeze_aacm, ioffdac, F_FREEZE_AACM_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_ioffdac_base, ioffdac, F_IOFFDAC_BASE_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_pol_offdac_base, ioffdac, F_POL_OFFDAC_BASE_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_calib_aacm, ioffdac, F_CALIB_AACM_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_ioffdac_aacm_read, ioffdac, F_IOFFDAC_AACM_READ_##pd, pd); \
static AFE4420_ATTR(in_pd##num##_pol_offdac_aacm_read, ioffdac, F_POL_OFFDAC_AACM_READ_##pd, pd)

PD_ATTRS(0, PD1);
PD_ATTRS(1, PD2);
PD_ATTRS(2, PD3);
PD_ATTRS(3, PD4);

#define PHASE_ATTRS(num, phase) \
static AFE4420_ATTR(in_intensity##num##_averages, averages, F_NUMAV_##phase, phase); \
static AFE4420_ATTR(in_intensity##num##_resistance, resistance, F_TIA_GAIN_RF_##phase, phase); \
static AFE4420_ATTR(in_intensity##num##_capacitance, capacitance, F_TIA_GAIN_CF_##phase, phase); \
static AFE4420_ATTR(in_intensity##num##_ioffdac, ioffdac, F_I_OFFDAC_##phase, phase); \
static AFE4420_ATTR(in_intensity##num##_poloffdac, poloffdac, F_POL_OFFDAC_##phase, phase)

PHASE_ATTRS(0, PHASE1);
PHASE_ATTRS(1, PHASE2);
PHASE_ATTRS(2, PHASE3);
PHASE_ATTRS(3, PHASE4);
PHASE_ATTRS(4, PHASE5);
PHASE_ATTRS(5, PHASE6);
PHASE_ATTRS(6, PHASE7);
PHASE_ATTRS(7, PHASE8);
PHASE_ATTRS(8, PHASE9);
PHASE_ATTRS(9, PHASE10);
PHASE_ATTRS(10, PHASE11);
PHASE_ATTRS(11, PHASE12);
PHASE_ATTRS(12, PHASE13);
PHASE_ATTRS(13, PHASE14);
PHASE_ATTRS(14, PHASE15);
PHASE_ATTRS(15, PHASE16);

static IIO_CONST_ATTR(sampling_frequency, "25"); /* 128000 / AFE4420_DEFAULT_PRPCOUNT */

#define PD_ATTRIBS(num) \
	&afe4420_attr_in_pd##num##_en_aacm.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_numphase_aacm.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_freeze_aacm.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_ioffdac_base.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_pol_offdac_base.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_calib_aacm.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_ioffdac_aacm_read.dev_attr.attr, \
	&afe4420_attr_in_pd##num##_pol_offdac_aacm_read.dev_attr.attr

#define PHASE_ATTRIBS(num) \
	&afe4420_attr_in_intensity##num##_averages.dev_attr.attr, \
	&afe4420_attr_in_intensity##num##_resistance.dev_attr.attr, \
	&afe4420_attr_in_intensity##num##_capacitance.dev_attr.attr, \
	&afe4420_attr_in_intensity##num##_ioffdac.dev_attr.attr, \
	&afe4420_attr_in_intensity##num##_poloffdac.dev_attr.attr

static struct attribute *afe4420_attributes[] = {
	&iio_const_attr_in_intensity_averages_available.dev_attr.attr,
	&dev_attr_in_intensity_resistance_available.attr,
	&dev_attr_in_intensity_capacitance_available.attr,
	&iio_const_attr_sampling_frequency.dev_attr.attr,

	&afe4420_attr_pd_disconnect.dev_attr.attr,
	&afe4420_attr_ifs_offdac.dev_attr.attr,
	&afe4420_attr_channel_offset_aacm.dev_attr.attr,

	PD_ATTRIBS(0),
	PD_ATTRIBS(1),
	PD_ATTRIBS(2),
	PD_ATTRIBS(3),

	PHASE_ATTRIBS(0),
	PHASE_ATTRIBS(1),
	PHASE_ATTRIBS(2),
	PHASE_ATTRIBS(3),
	PHASE_ATTRIBS(4),
	PHASE_ATTRIBS(5),
	PHASE_ATTRIBS(6),
	PHASE_ATTRIBS(7),
	PHASE_ATTRIBS(8),
	PHASE_ATTRIBS(9),
	PHASE_ATTRIBS(10),
	PHASE_ATTRIBS(11),
	PHASE_ATTRIBS(12),
	PHASE_ATTRIBS(13),
	PHASE_ATTRIBS(14),
	PHASE_ATTRIBS(15),

	NULL
};

static const struct attribute_group afe4420_attribute_group = {
	.attrs = afe4420_attributes
};

static int afe4420_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct afe4420_data *afe = iio_priv(indio_dev);
	unsigned int led_field = afe4420_channel_leds[chan->address];
	int ret;

	switch (chan->type) {
	case IIO_CURRENT:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = regmap_field_read(afe->fields[led_field], val);
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

static int afe4420_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct afe4420_data *afe = iio_priv(indio_dev);
	unsigned int led_field = afe4420_channel_leds[chan->address];

	switch (chan->type) {
	case IIO_CURRENT:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			return regmap_field_write(afe->fields[led_field], val);
		}
		break;
	default:
		break;
	}

	return -EINVAL;
}

static const unsigned long afe4420_scan_masks[] = {
	GENMASK(PHASE1, PHASE1),
	GENMASK(PHASE2, PHASE1),
	GENMASK(PHASE3, PHASE1),
	GENMASK(PHASE4, PHASE1),
	GENMASK(PHASE5, PHASE1),
	GENMASK(PHASE6, PHASE1),
	GENMASK(PHASE7, PHASE1),
	GENMASK(PHASE8, PHASE1),
	GENMASK(PHASE9, PHASE1),
	GENMASK(PHASE10, PHASE1),
	GENMASK(PHASE11, PHASE1),
	GENMASK(PHASE12, PHASE1),
	GENMASK(PHASE13, PHASE1),
	GENMASK(PHASE14, PHASE1),
	GENMASK(PHASE15, PHASE1),
	GENMASK(PHASE16, PHASE1),
	0
};

static int afe4420_update_scan_mode(struct iio_dev *indio_dev,
				    const unsigned long *scan_mask)
{
	struct afe4420_data *afe = iio_priv(indio_dev);
	unsigned int phases = find_first_zero_bit(scan_mask, indio_dev->masklength);
	unsigned int i;
	int ret;

	/* Enable PD for each enabled phase */
	for (i = PHASE1; i < phases; i++) {
		ret = regmap_update_bits(afe->regmap, AFE4420_PHASECNTRL0(i),
					 AFE4420_PHASECNTRL0_PD_ON1, AFE4420_PHASECNTRL0_PD_ON1);
		if (ret) {
			dev_err(afe->dev, "Unable to write PD enable to phase\n");
			return ret;
		}
	}

	/* Set sample time for each enabled phase */
	for (i = PHASE1; i < phases; i++) {
		ret = regmap_update_bits(afe->regmap, AFE4420_PHASECNTRL2(i),
					 AFE4420_PHASECNTRL2_TWLED, AFE4420_DEFAULT_TWLED);
		if (ret) {
			dev_err(afe->dev, "Unable to write sample time to phase\n");
			return ret;
		}
	}

	/* Set watermark for FIFO_RDY signal */
	ret = regmap_field_write(afe->fields[F_WM_FIFO], (phases * AFE4420_FIFO_LEN) - 1);
	if (ret) {
		dev_err(afe->dev, "Unable to write watermark level\n");
		return ret;
	}

	/* Set number of active signal phases */
	ret = regmap_field_write(afe->fields[F_NUMPHASE], phases - 1);
	if (ret) {
		dev_err(afe->dev, "Unable to write number of active phases\n");
		return ret;
	}

	afe->used_phases = phases;

	return 0;
}

static const struct iio_info afe4420_iio_info = {
	.attrs = &afe4420_attribute_group,
	.read_raw = afe4420_read_raw,
	.write_raw = afe4420_write_raw,
	.update_scan_mode = afe4420_update_scan_mode,
	.driver_module = THIS_MODULE,
};

static int afe4420_buffer_postenable(struct iio_dev *indio_dev)
{
	struct afe4420_data *afe = iio_priv(indio_dev);

	/* Turn on FIFO buffer */
//	regmap_update_bits(afe->regmap, AFE4420_CONTROL0,
//			   AFE4420_CONTROL0_FIFO_EN,
//			   AFE4420_CONTROL0_FIFO_EN);

	/* Start device sequence timer */
//	regmap_update_bits(afe->regmap, AFE4420_PRPCOUNT,
//			   AFE4420_PRPCOUNT_TIMEREN,
//			   AFE4420_PRPCOUNT_TIMEREN);

	/* Release timer from reset and enable FIFO in same write */
	regmap_write(afe->regmap, AFE4420_CONTROL0, AFE4420_CONTROL0_FIFO_EN);

	return 0;
}

static int afe4420_buffer_predisable(struct iio_dev *indio_dev)
{
	struct afe4420_data *afe = iio_priv(indio_dev);

//	regmap_update_bits(afe->regmap, AFE4420_PRPCOUNT,
//			   AFE4420_PRPCOUNT_TIMEREN, 0);
//
//	regmap_update_bits(afe->regmap, AFE4420_CONTROL0,
//			   AFE4420_CONTROL0_FIFO_EN, 0);

	/* Disable FIFO and put timer in reset in same write */
	regmap_write(afe->regmap, AFE4420_CONTROL0, AFE4420_CONTROL0_TM_COUNT_RST);

	return 0;
}

static const struct iio_buffer_setup_ops afe4420_buffer_setup_ops = {
	.postenable = &afe4420_buffer_postenable,
	.predisable = &afe4420_buffer_predisable,
};

static irqreturn_t afe4420_trigger_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct afe4420_data *afe = iio_priv(indio_dev);
	unsigned int reg_val;
	int pointer_diff, cycles, i, ret;

	/* Check data ready in FIFO */
	ret = regmap_read(afe->regmap, AFE4420_POINTER_DIFF, &reg_val);
	if (ret)
		goto err;
	pointer_diff = sign_extend32(reg_val, 8);
	pointer_diff += 1;

	if (pointer_diff % afe->used_phases) {
		dev_err(afe->dev, "Samples in FIFO not an even multiple of used phases\n");
		goto err;
	}
	cycles = pointer_diff / afe->used_phases;

	dev_dbg(afe->dev, "Full PRFs in FIFO: %d\n", cycles);

	if (cycles < AFE4420_FIFO_LEN)
		dev_info(afe->dev, "Early FIFO interrupt\n");
	else if (cycles > AFE4420_FIFO_LEN)
		dev_info(afe->dev, "Late FIFO interrupt..\n");

	ret = afe->fifo_read(afe->dev, (s32 *)afe->buffer, (afe->used_phases * cycles * sizeof(s32)));
	if (ret < 0)
		goto err;

	for (i = 0; i < cycles; i++)
		iio_push_to_buffers(indio_dev, &afe->buffer[i * (afe->used_phases * sizeof(s32))]);

err:
	return IRQ_HANDLED;
}

/* Default timings */
#define AFE4420_TIMING_PAIRS							\
	{ AFE4420_PHASECNTRL2(PHASE2), AFE4420_PHASECNTRL2_STAGGER_LED },	\
										\
	{ AFE4420_PHASECNTRL0(PHASE4), AFE4420_PHASECNTRL0_LED_DRV1_TX1 |	\
	                               AFE4420_PHASECNTRL0_LED_DRV2_TX1},	\
										\
	{ AFE4420_PHASECNTRL0(PHASE5), AFE4420_PHASECNTRL0_LED_DRV1_TX2 |	\
	                               AFE4420_PHASECNTRL0_LED_DRV2_TX2},	\
										\
	{ AFE4420_PHASECNTRL0(PHASE6), AFE4420_PHASECNTRL0_LED_DRV1_TX3 |	\
	                               AFE4420_PHASECNTRL0_LED_DRV2_TX3},	\
										\
	{ AFE4420_PHASECNTRL0(PHASE7), AFE4420_PHASECNTRL0_LED_DRV1_TX4 |	\
	                               AFE4420_PHASECNTRL0_LED_DRV2_TX4},

static const struct reg_sequence afe4420_reg_sequences[] = {
	{ AFE4420_CONTROL0, AFE4420_CONTROL0_TM_COUNT_RST },
	{ AFE4420_PRPCOUNT, AFE4420_PRPCOUNT_TIMEREN | AFE4420_DEFAULT_PRPCOUNT },
	{ AFE4420_CONTROL1, AFE4420_CONTROL1_IFS_OFFDAC |
	                    AFE4420_CONTROL1_EN_AACM_GBL |
	                    AFE4420_CONTROL1_ILED_2X },
	{ AFE4420_FIFO, AFE4420_FIFO_INT_MUX_FIFO_RDY },
	{ AFE4420_PHASE, AFE4420_PHASE_FILT1_RESET_ENZ |
	                 AFE4420_PHASE_FILT2_RESET_ENZ |
	                 AFE4420_PHASE_FILT3_RESET_ENZ |
	                 AFE4420_PHASE_FILT4_RESET_ENZ },
	{ AFE4420_AACM, AFE4420_AACM_IMM_REFRESH |
	                AFE4420_AACM_QUICK_CONV },
	AFE4420_TIMING_PAIRS
};

static bool afe4420_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AFE4420_POINTER_DIFF:
	case AFE4420_PDCNTRL2(PD1):
	case AFE4420_PDCNTRL2(PD2):
	case AFE4420_PDCNTRL2(PD3):
	case AFE4420_PDCNTRL2(PD4):
		return true;
	default:
		return false;
	}
}

const struct regmap_config afe4420_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,

	.zero_flag_mask = true,
	.max_register = AFE4420_PHASECNTRL2(AFE4420_TOTAL_PHASES),
//	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = &afe4420_is_volatile_reg,
};
EXPORT_SYMBOL_GPL(afe4420_regmap_config);

const struct of_device_id afe4420_of_match[] = {
	{ .compatible = "ti,afe4420", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, afe4420_of_match);
EXPORT_SYMBOL_GPL(afe4420_of_match);

static int __maybe_unused afe4420_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	int ret;

	/* Set the reset pin */
	gpiod_set_value(afe->reset_gpio, 1);

	ret = regulator_disable(afe->regulator);
	if (ret) {
		dev_err(dev, "Unable to disable regulator\n");
		return ret;
	}

	return 0;
}

static int __maybe_unused afe4420_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(afe->regulator);
	if (ret) {
		dev_err(dev, "Unable to enable regulator\n");
		return ret;
	}

	/* Release the reset pin */
	gpiod_set_value(afe->reset_gpio, 0);

	// FIXME: Restore cached reg values

	return 0;
}

SIMPLE_DEV_PM_OPS(afe4420_pm_ops, afe4420_suspend, afe4420_resume);
EXPORT_SYMBOL_GPL(afe4420_pm_ops);

int afe4420_setup(struct regmap *regmap, int irq,
		int (*fifo_read)(struct device *dev, s32 *buffer, int len))
{
	struct device *dev = regmap_get_device(regmap);
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct afe4420_data *afe;
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
		afe->fields[i] = devm_regmap_field_alloc(afe->dev, afe->regmap, afe4420_reg_fields[i]);
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

	afe->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(afe->reset_gpio)) {
		dev_err(dev, "error requesting reset_gpio: %ld\n", PTR_ERR(afe->reset_gpio));
		return PTR_ERR(afe->reset_gpio);
	} else if (afe->reset_gpio) {
		/* bring the device out of reset */
		gpiod_set_value(afe->reset_gpio, 0);
	}

	ret = regmap_write(afe->regmap, AFE4420_CONTROL0,
			   AFE4420_CONTROL0_SW_RESET);
	if (ret) {
		dev_err(afe->dev, "Unable to reset device\n");
		goto disable_reg;
	}

	ret = regmap_multi_reg_write(afe->regmap, afe4420_reg_sequences,
				     ARRAY_SIZE(afe4420_reg_sequences));
	if (ret) {
		dev_err(afe->dev, "Unable to set register defaults\n");
		goto disable_reg;
	}

	indio_dev->modes = (INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE);
	indio_dev->dev.parent = afe->dev;
	indio_dev->channels = afe4420_channels;
	indio_dev->num_channels = ARRAY_SIZE(afe4420_channels);
	indio_dev->name = AFE4420_DRIVER_NAME;
	indio_dev->info = &afe4420_iio_info;
	indio_dev->available_scan_masks = afe4420_scan_masks;
	indio_dev->setup_ops = &afe4420_buffer_setup_ops;

	if (afe->irq > 0) {
		buffer = devm_iio_kfifo_allocate(afe->dev);
		if (!buffer) {
			ret = -ENOMEM;
			goto disable_reg;
		}

		iio_device_attach_buffer(indio_dev, buffer);

		ret = devm_request_threaded_irq(afe->dev, afe->irq,
						NULL, afe4420_trigger_handler,
						IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
						AFE4420_DRIVER_NAME,
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
EXPORT_SYMBOL_GPL(afe4420_setup);

int afe4420_teardown(struct regmap *regmap)
{
	struct device *dev = regmap_get_device(regmap);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct afe4420_data *afe = iio_priv(indio_dev);
	int ret;

	iio_device_unregister(indio_dev);

	ret = regulator_disable(afe->regulator);
	if (ret) {
		dev_err(afe->dev, "Unable to disable regulator\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(afe4420_teardown);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI AFE4420 Optical Heart-Rate Monitor and Bio-Sensor");
MODULE_LICENSE("GPL v2");
