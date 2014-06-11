/*
 * panel driver for lvds and eDP panels
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * Ajay Kumar <ajaykumar.rs@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

struct panel_lvds {
	struct drm_panel	base;
	struct regulator	*backlight_fet;
	struct regulator	*lcd_fet;
	struct videomode	vm;
	int			width_mm;
	int			height_mm;
	bool			backlight_fet_enabled;
	bool			lcd_fet_enabled;
	int			led_en_gpio;
	int			lcd_en_gpio;
	int			panel_prepare_delay;
	int			panel_enable_delay;
	int			panel_disable_delay;
	int			panel_unprepare_delay;
};

static inline struct panel_lvds *to_panel(struct drm_panel *panel)
{
	return container_of(panel, struct panel_lvds, base);
}

static int panel_lvds_prepare(struct drm_panel *panel)
{
	struct panel_lvds *lvds_panel = to_panel(panel);

	if (!IS_ERR_OR_NULL(lvds_panel->lcd_fet))
		if (!lvds_panel->lcd_fet_enabled) {
			if (regulator_enable(lvds_panel->lcd_fet))
				DRM_ERROR("failed to enable LCD fet\n");
			lvds_panel->lcd_fet_enabled = true;
		}

	if (gpio_is_valid(lvds_panel->lcd_en_gpio))
		gpio_set_value(lvds_panel->lcd_en_gpio, 1);

	msleep(lvds_panel->panel_prepare_delay);

	return 0;
}

static int panel_lvds_enable(struct drm_panel *panel)
{
	struct panel_lvds *lvds_panel = to_panel(panel);

	if (!IS_ERR_OR_NULL(lvds_panel->backlight_fet))
		if (!lvds_panel->backlight_fet_enabled) {
			if (regulator_enable(lvds_panel->backlight_fet))
				DRM_ERROR("failed to enable LED fet\n");
			lvds_panel->backlight_fet_enabled = true;
		}

	msleep(lvds_panel->panel_enable_delay);

	if (gpio_is_valid(lvds_panel->led_en_gpio))
		gpio_set_value(lvds_panel->led_en_gpio, 1);

	return 0;
}

static int panel_lvds_disable(struct drm_panel *panel)
{
	struct panel_lvds *lvds_panel = to_panel(panel);

	if (gpio_is_valid(lvds_panel->led_en_gpio))
		gpio_set_value(lvds_panel->led_en_gpio, 0);

	if (!IS_ERR_OR_NULL(lvds_panel->backlight_fet))
		if (lvds_panel->backlight_fet_enabled) {
			regulator_disable(lvds_panel->backlight_fet);
			lvds_panel->backlight_fet_enabled = false;
		}

	msleep(lvds_panel->panel_disable_delay);

	return 0;
}

static int panel_lvds_unprepare(struct drm_panel *panel)
{
	struct panel_lvds *lvds_panel = to_panel(panel);

	if (gpio_is_valid(lvds_panel->lcd_en_gpio))
		gpio_set_value(lvds_panel->lcd_en_gpio, 0);

	if (!IS_ERR_OR_NULL(lvds_panel->lcd_fet))
		if (lvds_panel->lcd_fet_enabled) {
			regulator_disable(lvds_panel->lcd_fet);
			lvds_panel->lcd_fet_enabled = false;
		}

	msleep(lvds_panel->panel_unprepare_delay);

	return 0;
}

static int panel_lvds_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct panel_lvds *lvds_panel = to_panel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode.\n");
		return 0;
	}

	drm_display_mode_from_videomode(&lvds_panel->vm, mode);
	mode->width_mm = lvds_panel->width_mm;
	mode->height_mm = lvds_panel->height_mm;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs panel_lvds_funcs = {
	.unprepare = panel_lvds_unprepare,
	.disable = panel_lvds_disable,
	.prepare = panel_lvds_prepare,
	.enable = panel_lvds_enable,
	.get_modes = panel_lvds_get_modes,
};

static int panel_lvds_probe(struct platform_device *pdev)
{
	struct panel_lvds *panel;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int ret;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	ret = of_get_videomode(node, &panel->vm, 0);
	if (ret) {
		DRM_ERROR("failed: of_get_videomode() : %d\n", ret);
		return ret;
	}

	panel->lcd_fet = devm_regulator_get_optional(dev, "lcd_vdd");
	if (IS_ERR(panel->lcd_fet))
		return -EPROBE_DEFER;

	panel->backlight_fet = devm_regulator_get_optional(dev, "vcd_led");
	if (IS_ERR(panel->backlight_fet))
		return -EPROBE_DEFER;

	panel->lcd_en_gpio = of_get_named_gpio(node, "lcd-en-gpio", 0);
	panel->led_en_gpio = of_get_named_gpio(node, "led-en-gpio", 0);

	of_property_read_u32(node, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(node, "panel-height-mm", &panel->height_mm);

	of_property_read_u32(node, "panel-prepare-delay",
					&panel->panel_prepare_delay);
	of_property_read_u32(node, "panel-enable-delay",
					&panel->panel_enable_delay);
	of_property_read_u32(node, "panel-disable-delay",
					&panel->panel_disable_delay);
	of_property_read_u32(node, "panel-unprepare-delay",
					&panel->panel_unprepare_delay);

	if (gpio_is_valid(panel->lcd_en_gpio)) {
		ret = devm_gpio_request_one(dev, panel->lcd_en_gpio,
					GPIOF_OUT_INIT_LOW, "lcd_en_gpio");
		if (ret) {
			DRM_ERROR("failed to get lcd-en gpio [%d]\n", ret);
			return 0;
		}
	} else {
		panel->lcd_en_gpio = -ENODEV;
	}

	if (gpio_is_valid(panel->led_en_gpio)) {
		ret = devm_gpio_request_one(dev, panel->led_en_gpio,
					GPIOF_OUT_INIT_LOW, "led_en_gpio");
		if (ret) {
			DRM_ERROR("failed to get led-en gpio [%d]\n", ret);
			return 0;
		}
	} else {
		panel->led_en_gpio = -ENODEV;
	}

	drm_panel_init(&panel->base);
	panel->base.dev = dev;
	panel->base.funcs = &panel_lvds_funcs;

	ret = drm_panel_add(&panel->base);
	if (ret < 0)
		return ret;

	dev_set_drvdata(dev, panel);

	return 0;
}

static int panel_lvds_remove(struct platform_device *pdev)
{
	struct panel_lvds *panel = dev_get_drvdata(&pdev->dev);

	panel_lvds_disable(&panel->base);
	panel_lvds_unprepare(&panel->base);

	drm_panel_remove(&panel->base);

	return 0;
}

static const struct of_device_id lvds_panel_dt_match[] = {
	{ .compatible = "panel-lvds" },
	{},
};
MODULE_DEVICE_TABLE(of, lvds_panel_dt_match);

struct platform_driver lvds_panel_driver = {
	.driver = {
		.name = "panel_lvds",
		.owner = THIS_MODULE,
		.of_match_table = lvds_panel_dt_match,
	},
	.probe = panel_lvds_probe,
	.remove = panel_lvds_remove,
};
module_platform_driver(lvds_panel_driver);

MODULE_AUTHOR("Ajay Kumar <ajaykumar.rs@samsung.com>");
MODULE_DESCRIPTION("lvds/eDP panel driver");
MODULE_LICENSE("GPL v2");
