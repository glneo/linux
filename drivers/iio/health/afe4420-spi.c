// SPDX-License-Identifier: GPL-2.0
/*
 * AFE4420 SPI Interface Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include "afe4420.h"

#define AFE4420_SPI_DRIVER_NAME	"afe4420-spi"

static int afe4420_spi_fifo_read(struct device *dev, s32 *buffer, int len)
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

static int afe4420_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &afe4420_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Unable to allocate register map\n");
		return PTR_ERR(regmap);
	}

	spi_set_drvdata(spi, regmap);

	return afe4420_setup(regmap, spi->irq, afe4420_spi_fifo_read);
}

static int afe4420_spi_remove(struct spi_device *spi)
{
	struct regmap *regmap = spi_get_drvdata(spi);

	afe4420_teardown(regmap);

	return 0;
}

static const struct spi_device_id afe4420_spi_ids[] = {
	{ "afe4420", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, afe4420_spi_ids);

static struct spi_driver afe4420_spi_driver = {
	.driver = {
		.name = AFE4420_SPI_DRIVER_NAME,
		.of_match_table = afe4420_of_match,
		.pm = &afe4420_pm_ops,
	},
	.probe = afe4420_spi_probe,
	.remove = afe4420_spi_remove,
	.id_table = afe4420_spi_ids,
};
module_spi_driver(afe4420_spi_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TI AFE4420 SPI Interface Driver");
MODULE_LICENSE("GPL v2");
