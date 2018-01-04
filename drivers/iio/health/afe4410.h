// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4410 Heart Rate Monitors and Low-Cost Pulse Oximeters
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#ifndef _AFE4410_H
#define _AFE4410_H

#include <linux/regmap.h>

extern const struct regmap_config afe4410_regmap_config;
extern const struct of_device_id afe4410_of_match[];
extern const struct dev_pm_ops afe4410_pm_ops;

int afe4410_setup(struct regmap *regmap, int irq);
int afe4410_teardown(struct regmap *regmap);

#endif /* _AFE4410_H */
