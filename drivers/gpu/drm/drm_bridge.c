/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/err.h>
#include <linux/module.h>

#include <drm/drm_crtc.h>

static DEFINE_MUTEX(bridge_lock);
static LIST_HEAD(bridge_lookup);

int drm_bridge_add_for_lookup(struct drm_bridge *bridge)
{
	mutex_lock(&bridge_lock);
	list_add_tail(&bridge->head, &bridge_lookup);
	mutex_unlock(&bridge_lock);

	return 0;
}
EXPORT_SYMBOL(drm_bridge_add_for_lookup);

void drm_bridge_remove_from_lookup(struct drm_bridge *bridge)
{
	mutex_lock(&bridge_lock);
	list_del_init(&bridge->head);
	mutex_unlock(&bridge_lock);
}
EXPORT_SYMBOL(drm_bridge_remove_from_lookup);

int drm_bridge_attach_encoder(struct drm_bridge *bridge,
				struct drm_encoder *encoder)
{
	if (!bridge || !encoder)
		return -EINVAL;

	if (bridge->encoder)
		return -EBUSY;

	encoder->bridge = bridge;
	bridge->encoder = encoder;
	bridge->drm_dev = encoder->dev;

	return 0;
}
EXPORT_SYMBOL(drm_bridge_attach_encoder);

#ifdef CONFIG_OF
struct drm_bridge *of_drm_find_bridge(struct device_node *np)
{
	struct drm_bridge *bridge;

	mutex_lock(&bridge_lock);

	list_for_each_entry(bridge, &bridge_lookup, head) {
		if (bridge->dev->of_node == np) {
			mutex_unlock(&bridge_lock);
			return bridge;
		}
	}

	mutex_unlock(&bridge_lock);
	return NULL;
}
EXPORT_SYMBOL(of_drm_find_bridge);
#endif

MODULE_AUTHOR("Ajay Kumar <ajaykumar.rs@samsung.com>");
MODULE_DESCRIPTION("DRM bridge infrastructure");
MODULE_LICENSE("GPL and additional rights");
