// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4420 Optical Heart-Rate Monitor and Bio-Sensor
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#ifndef _AFE4420_H
#define _AFE4420_H

#include <linux/regmap.h>

#define AFE4420_INTENSITY_CHAN(_index)				\
	{							\
		.type = IIO_INTENSITY,				\
		.channel = _index,				\
		.address = _index,				\
		.scan_index = _index,				\
		.scan_type = {					\
				.sign = 's',			\
				.realbits = 24,			\
				.storagebits = 32,		\
				.endianness = IIO_CPU,		\
		},						\
		.indexed = true,				\
	}

#define AFE4420_CURRENT_CHAN(_index)				\
	{							\
		.type = IIO_CURRENT,				\
		.channel = _index,				\
		.address = _index,				\
		.scan_index = -1,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
			BIT(IIO_CHAN_INFO_SCALE),		\
		.indexed = true,				\
		.output = true,					\
	}

struct afe4420_val_table {
	int integer;
	int fract;
};

#define AFE4420_TABLE_ATTR(_name, _table)				\
static ssize_t _name ## _show(struct device *dev,			\
			      struct device_attribute *attr, char *buf)	\
{									\
	ssize_t len = 0;						\
	int i;								\
									\
	for (i = 0; i < ARRAY_SIZE(_table); i++)			\
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%06u ", \
				 _table[i].integer,			\
				 _table[i].fract);			\
									\
	buf[len - 1] = '\n';						\
									\
	return len;							\
}									\
static DEVICE_ATTR_RO(_name)

struct afe4420_attr {
	struct device_attribute dev_attr;
	unsigned int field;
	unsigned int phase;
};

#define to_afe4420_attr(_dev_attr)				\
	container_of(_dev_attr, struct afe4420_attr, dev_attr)

#define AFE4420_ATTR(_name, _type, _field, _phase)		\
	struct afe4420_attr afe4420_attr_##_name = {		\
		.dev_attr = __ATTR(_name, (S_IRUGO | S_IWUSR),	\
				   afe4420_show_##_type,	\
				   afe4420_store_##_type),	\
		.field = _field,				\
		.phase = _phase,				\
	}

extern const struct regmap_config afe4420_regmap_config;
extern const struct of_device_id afe4420_of_match[];
extern const struct dev_pm_ops afe4420_pm_ops;

int afe4420_setup(struct regmap *regmap, int irq,
		int (*fifo_read)(struct device *dev, s32 *buffer, int len));
int afe4420_teardown(struct regmap *regmap);

#endif /* _AFE4420_H */
