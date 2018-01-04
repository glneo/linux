// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4410 I2C Interface Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>

#include "afe4410.h"

#define AFE4410_I2C_DRIVER_NAME	"afe4410-i2c"

static int afe4410_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &afe4410_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Unable to allocate register map\n");
		return PTR_ERR(regmap);
	}

	i2c_set_clientdata(client, regmap);

	return afe4410_setup(regmap, client->irq);
}

static int afe4410_i2c_remove(struct i2c_client *client)
{
	struct regmap *regmap = i2c_get_clientdata(client);

	afe4410_teardown(regmap);

	return 0;
}

static const struct i2c_device_id afe4410_i2c_ids[] = {
	{ "afe4410", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, afe4410_i2c_ids);

static struct i2c_driver afe4410_i2c_driver = {
	.driver = {
		.name = AFE4410_I2C_DRIVER_NAME,
		.of_match_table = afe4410_of_match,
		.pm = &afe4410_pm_ops,
	},
	.probe = afe4410_i2c_probe,
	.remove = afe4410_i2c_remove,
	.id_table = afe4410_i2c_ids,
};
module_i2c_driver(afe4410_i2c_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI AFE4410 I2C Interface Driver");
MODULE_LICENSE("GPL v2");
