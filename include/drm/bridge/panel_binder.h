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

#ifndef _DRM_BRIDGE_PANEL_H_
#define _DRM_BRIDGE_PANEL_H_

struct drm_device;
struct drm_encoder;
struct i2c_client;
struct device_node;
struct drm_panel;

#if defined(CONFIG_DRM_PANEL_BINDER)
struct drm_bridge *panel_binder_init(struct drm_device *dev,
					struct drm_encoder *encoder,
					struct i2c_client *client,
					struct device_node *node,
					struct drm_panel *panel,
					int connector_type,
					uint8_t polled);
#else
static inline struct drm_bridge *panel_binder_init(struct drm_device *dev,
						struct drm_encoder *encoder,
						struct i2c_client *client,
						struct device_node *node,
						struct drm_panel *panel,
						int connector_type,
						uint8_t polled)
{
	return 0;
}
#endif

#endif
