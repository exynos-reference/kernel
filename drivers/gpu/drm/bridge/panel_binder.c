/*
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_panel.h>

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include "bridge/panel_binder.h"

struct panel_binder {
	struct drm_connector	connector;
	struct i2c_client	*client;
	struct drm_encoder	*encoder;
	struct drm_bridge	*bridge;
	struct drm_panel	*panel;
};

static void panel_binder_pre_enable(struct drm_bridge *bridge)
{
	struct panel_binder *panel_binder = bridge->driver_private;

	drm_panel_prepare(panel_binder->panel);
}

static void panel_binder_enable(struct drm_bridge *bridge)
{
	struct panel_binder *panel_binder = bridge->driver_private;

	drm_panel_enable(panel_binder->panel);
}

static void panel_binder_disable(struct drm_bridge *bridge)
{
	struct panel_binder *panel_binder = bridge->driver_private;

	drm_panel_disable(panel_binder->panel);
}

static void panel_binder_post_disable(struct drm_bridge *bridge)
{
	struct panel_binder *panel_binder = bridge->driver_private;

	drm_panel_unprepare(panel_binder->panel);
}

void panel_binder_destroy(struct drm_bridge *bridge)
{
	struct panel_binder *panel_binder = bridge->driver_private;

	drm_panel_detach(panel_binder->panel);
	drm_bridge_cleanup(bridge);
}

struct drm_bridge_funcs panel_binder_funcs = {
	.pre_enable = panel_binder_pre_enable,
	.enable = panel_binder_enable,
	.disable = panel_binder_disable,
	.post_disable = panel_binder_post_disable,
	.destroy = panel_binder_destroy,
};

static int panel_binder_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int panel_binder_get_modes(struct drm_connector *connector)
{
	struct panel_binder *panel_binder;

	panel_binder = container_of(connector, struct panel_binder, connector);

	return panel_binder->panel->funcs->get_modes(panel_binder->panel);
}

static struct drm_encoder *panel_binder_best_encoder(struct drm_connector
								*connector)
{
	struct panel_binder *panel_binder;

	panel_binder = container_of(connector, struct panel_binder, connector);

	return panel_binder->encoder;
}

static const struct drm_connector_helper_funcs
					panel_binder_connector_helper_funcs = {
	.get_modes = panel_binder_get_modes,
	.mode_valid = panel_binder_mode_valid,
	.best_encoder = panel_binder_best_encoder,
};

static enum drm_connector_status panel_binder_detect(struct drm_connector
							*connector, bool force)
{
	return connector_status_connected;
}

static void panel_binder_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs panel_binder_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = panel_binder_detect,
	.destroy = panel_binder_connector_destroy,
};

struct drm_bridge *panel_binder_init(struct drm_device *dev,
					struct drm_encoder *encoder,
					struct i2c_client *client,
					struct device_node *node,
					struct drm_panel *panel,
					int connector_type,
					uint8_t polled)
{
	int ret;
	struct drm_bridge *bridge;
	struct panel_binder *panel_binder;

	if (IS_ERR_OR_NULL(panel)) {
		DRM_ERROR("invalid drm_panel pointer\n");
		return NULL;
	}

	bridge = devm_kzalloc(dev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		DRM_ERROR("failed to allocate drm bridge\n");
		return NULL;
	}

	panel_binder = devm_kzalloc(dev->dev, sizeof(*panel_binder),
								GFP_KERNEL);
	if (!panel_binder) {
		DRM_ERROR("failed to allocate bridge panel_binder\n");
		return NULL;
	}

	panel_binder->client = client;
	panel_binder->encoder = encoder;
	panel_binder->bridge = bridge;
	panel_binder->panel = panel;

	ret = drm_bridge_init(dev, bridge, &panel_binder_funcs);
	if (ret) {
		DRM_ERROR("failed to initialize bridge with drm\n");
		goto err;
	}

	bridge->driver_private = panel_binder;

	drm_panel_attach(panel_binder->panel, &panel_binder->connector);

	if (!encoder->bridge)
		/* First entry in the bridge chain */
		encoder->bridge = bridge;

	panel_binder->connector.polled = polled;
	ret = drm_connector_init(dev, &panel_binder->connector,
			&panel_binder_connector_funcs, connector_type);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err;
	}
	drm_connector_helper_add(&panel_binder->connector,
			&panel_binder_connector_helper_funcs);
	drm_sysfs_connector_add(&panel_binder->connector);
	drm_mode_connector_attach_encoder(&panel_binder->connector, encoder);

	return bridge;

err:
	return NULL;
}
EXPORT_SYMBOL(panel_binder_init);
