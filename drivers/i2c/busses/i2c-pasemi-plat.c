// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2006-2007 PA Semi, Inc
 *
 * SMBus host driver for PA Semi PWRficient
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>

#include "i2c-pasemi-core.h"

static int pasemi_i2c_platform_probe(struct platform_device *pdev)
{
	struct pasemi_smbus *smbus;
	struct resource *res;
	int error;

	smbus = kzalloc(sizeof(struct pasemi_smbus), GFP_KERNEL);
	if (!smbus)
		return -ENOMEM;

	smbus->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smbus->ioaddr = devm_ioremap_resource(smbus->dev, res);
	smbus->base = res->start;
	smbus->size = resource_size(res);

	if (IS_ERR(smbus->ioaddr))
		return PTR_ERR(smbus->ioaddr);

	smbus->adapter.dev.of_node = pdev->dev.of_node;
	error = pasemi_i2c_common_probe(smbus);
	if (error)
		return error;

	platform_set_drvdata(pdev, smbus);

	return 0;
}

static int pasemi_i2c_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id pasemi_i2c_of_match[] = {
	{ .compatible = "apple,t8103-i2c", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, pasemi_i2c_of_match);

static struct platform_driver pasemi_i2c_platform_driver = {
	.driver	= {
		.name			= "i2c-pasemi",
		.of_match_table		= pasemi_i2c_of_match,
	},
	.probe	= pasemi_i2c_platform_probe,
	.remove	= pasemi_i2c_platform_remove,
};
module_platform_driver(pasemi_i2c_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("PA Semi PWRficient SMBus platform driver");
