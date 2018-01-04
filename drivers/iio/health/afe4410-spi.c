// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4410 SPI Interface Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "afe4410.h"

#define AFE4410_SPI_DRIVER_NAME	"afe4410-spi"

static int afe4410_spi_fifo_read(struct device *dev, s32 *buffer, int len)
{
	struct spi_device *spi = to_spi_device(dev);

	u8 addr[1] ____cacheline_aligned = { 0xff }; /* 8-bit address */
	struct spi_transfer addr_t = {
		.tx_buf = addr,
		.rx_buf	= NULL,
		.len = sizeof(addr),
		.bits_per_word = 8,
	};

	struct spi_transfer buf_t = {
		.tx_buf = NULL,
		.rx_buf	= buffer,
		.len = len,
		.bits_per_word = 24,
	};

	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&addr_t, &m);
	spi_message_add_tail(&buf_t, &m);

	return spi_sync(spi, &m);
}

static int afe4410_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &afe4410_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Unable to allocate register map\n");
		return PTR_ERR(regmap);
	}

	spi_set_drvdata(spi, regmap);

	return afe4410_setup(regmap, spi->irq, afe4410_spi_fifo_read);
}

static int afe4410_spi_remove(struct spi_device *spi)
{
	struct regmap *regmap = spi_get_drvdata(spi);

	afe4410_teardown(regmap);

	return 0;
}

static const struct spi_device_id afe4410_spi_ids[] = {
	{ "afe4410", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, afe4410_spi_ids);

static struct spi_driver afe4410_spi_driver = {
	.driver = {
		.name = AFE4410_SPI_DRIVER_NAME,
		.of_match_table = afe4410_of_match,
		.pm = &afe4410_pm_ops,
	},
	.probe = afe4410_spi_probe,
	.remove = afe4410_spi_remove,
	.id_table = afe4410_spi_ids,
};
module_spi_driver(afe4410_spi_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI AFE4410 SPI Interface Driver");
MODULE_LICENSE("GPL v2");
