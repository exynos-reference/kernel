/*
 * System Control Driver
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
 * Author: Dong Aisheng <dong.aisheng@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/syscon.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

static DEFINE_SPINLOCK(syscon_list_slock);
static LIST_HEAD(syscon_list);

struct syscon {
	struct regmap *regmap;
	struct device *dev;
	struct list_head list;
};

struct regmap *syscon_node_to_regmap(struct device_node *np)
{
	struct syscon *entry, *syscon = ERR_PTR(-EPROBE_DEFER);

	spin_lock(&syscon_list_slock);

	list_for_each_entry(entry, &syscon_list, list)
		if (entry->dev->of_node == np) {
			syscon = entry;
			break;
		}

	spin_unlock(&syscon_list_slock);

	return syscon->regmap;
}
EXPORT_SYMBOL_GPL(syscon_node_to_regmap);

struct regmap *syscon_regmap_lookup_by_compatible(const char *s)
{
	struct device_node *syscon_np;
	struct regmap *regmap;

	syscon_np = of_find_compatible_node(NULL, NULL, s);
	if (!syscon_np)
		return ERR_PTR(-ENODEV);

	regmap = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);

	return regmap;
}
EXPORT_SYMBOL_GPL(syscon_regmap_lookup_by_compatible);

struct regmap *syscon_regmap_lookup_by_pdevname(const char *s)
{
	struct syscon *entry, *syscon = ERR_PTR(-EPROBE_DEFER);

	spin_lock(&syscon_list_slock);

	list_for_each_entry(entry, &syscon_list, list)
		if (!strcmp(dev_name(entry->dev), s)) {
			syscon = entry;
			break;
		}

	spin_unlock(&syscon_list_slock);

	return syscon->regmap;
}
EXPORT_SYMBOL_GPL(syscon_regmap_lookup_by_pdevname);

struct regmap *syscon_regmap_lookup_by_phandle(struct device_node *np,
					const char *property)
{
	struct device_node *syscon_np;
	struct regmap *regmap;

	if (property)
		syscon_np = of_parse_phandle(np, property, 0);
	else
		syscon_np = np;

	if (!syscon_np)
		return ERR_PTR(-ENODEV);

	regmap = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);

	return regmap;
}
EXPORT_SYMBOL_GPL(syscon_regmap_lookup_by_phandle);

static const struct of_device_id of_syscon_match[] = {
	{ .compatible = "syscon", },
	{ },
};

static struct regmap_config syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static void devm_syscon_unregister(struct device *dev, void *res)
{
	struct syscon *syscon = *(struct syscon **)res;

	spin_lock(&syscon_list_slock);
	list_del(&syscon->list);
	spin_unlock(&syscon_list_slock);
}

int devm_syscon_register(struct device *dev, struct regmap *regmap)
{
	struct syscon **ptr, *syscon;

	syscon = devm_kzalloc(dev, sizeof(*syscon), GFP_KERNEL);
	if (!syscon)
		return -ENOMEM;

	syscon->regmap = regmap;
	syscon->dev = dev;

	ptr = devres_alloc(devm_syscon_unregister, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	spin_lock(&syscon_list_slock);
	list_add_tail(&syscon->list, &syscon_list);
	spin_unlock(&syscon_list_slock);

	*ptr = syscon;
	devres_add(dev, ptr);

	return 0;
}
EXPORT_SYMBOL_GPL(devm_syscon_register);

static int syscon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct syscon_platform_data *pdata = dev_get_platdata(dev);
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	syscon_regmap_config.max_register = res->end - res->start - 3;
	if (pdata)
		syscon_regmap_config.name = pdata->label;
	regmap = devm_regmap_init_mmio(dev, base,
					&syscon_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "regmap init failed\n");
		return PTR_ERR(regmap);
	}

	ret = devm_syscon_register(dev, regmap);
	if (ret)
		return ret;

	dev_dbg(dev, "regmap %pR registered\n", res);

	return 0;
}

static const struct platform_device_id syscon_ids[] = {
	{ "syscon", },
	{ }
};

static struct platform_driver syscon_driver = {
	.driver = {
		.name = "syscon",
		.owner = THIS_MODULE,
		.of_match_table = of_syscon_match,
	},
	.probe		= syscon_probe,
	.id_table	= syscon_ids,
};

static int __init syscon_init(void)
{
	return platform_driver_register(&syscon_driver);
}
postcore_initcall(syscon_init);

static void __exit syscon_exit(void)
{
	platform_driver_unregister(&syscon_driver);
}
module_exit(syscon_exit);

MODULE_AUTHOR("Dong Aisheng <dong.aisheng@linaro.org>");
MODULE_DESCRIPTION("System Control driver");
MODULE_LICENSE("GPL v2");
