/*
 * Copyright (C) 2014 Google, Inc.
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

#ifndef _DRM_BRIDGE_PS8622_H_
#define _DRM_BRIDGE_PS8622_H_

struct drm_device;
struct drm_encoder;
struct i2c_client;
struct device_node;

#ifdef CONFIG_DRM_PS8622

struct drm_bridge *ps8622_init(struct drm_device *dev,
				struct drm_encoder *encoder,
				struct i2c_client *client,
				struct device_node *node);

#else

static inline struct drm_bridge *ps8622_init(struct drm_device *dev,
				struct drm_encoder *encoder,
				struct i2c_client *client,
				struct device_node *node)
{
	return -ENODEV;
}

#endif

#endif
