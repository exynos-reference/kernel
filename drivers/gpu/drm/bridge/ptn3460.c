/*
 * NXP PTN3460 DP/LVDS bridge driver
 *
 * Copyright (C) 2013 Google, Inc.
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
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "drmP.h"
#include "drm_edid.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

#include "bridge/ptn3460.h"

#define PTN3460_EDID_ADDR			0x0
#define PTN3460_EDID_EMULATION_ADDR		0x84
#define PTN3460_EDID_ENABLE_EMULATION		0
#define PTN3460_EDID_EMULATION_SELECTION	1
#define PTN3460_EDID_SRAM_LOAD_ADDR		0x85

struct ptn3460_bridge {
	struct i2c_client *client;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	int gpio_pd_n;
	int gpio_rst_n;
	u32 edid_emulation;
	bool enabled;
};

static int ptn3460_write_byte(struct ptn3460_bridge *ptn_bridge, char addr,
		char val)
{
	int ret;
	char buf[2];

	buf[0] = addr;
	buf[1] = val;

	ret = i2c_master_send(ptn_bridge->client, buf, ARRAY_SIZE(buf));
	if (ret <= 0) {
		DRM_ERROR("Failed to send i2c command, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ptn3460_select_edid(struct ptn3460_bridge *ptn_bridge)
{
	int ret;
	char val;

	/* Load the selected edid into SRAM (accessed at PTN3460_EDID_ADDR) */
	ret = ptn3460_write_byte(ptn_bridge, PTN3460_EDID_SRAM_LOAD_ADDR,
			ptn_bridge->edid_emulation);
	if (ret) {
		DRM_ERROR("Failed to transfer edid to sram, ret=%d\n", ret);
		return ret;
	}

	/* Enable EDID emulation and select the desired EDID */
	val = 1 << PTN3460_EDID_ENABLE_EMULATION |
		ptn_bridge->edid_emulation << PTN3460_EDID_EMULATION_SELECTION;

	ret = ptn3460_write_byte(ptn_bridge, PTN3460_EDID_EMULATION_ADDR, val);
	if (ret) {
		DRM_ERROR("Failed to write edid value, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static void ptn3460_pre_enable(struct drm_bridge *bridge)
{
	struct ptn3460_bridge *ptn_bridge = bridge->driver_private;
	int ret;

	if (ptn_bridge->enabled)
		return;

	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_set_value(ptn_bridge->gpio_pd_n, 1);

	if (gpio_is_valid(ptn_bridge->gpio_rst_n)) {
		gpio_set_value(ptn_bridge->gpio_rst_n, 0);
		udelay(10);
		gpio_set_value(ptn_bridge->gpio_rst_n, 1);
	}

	drm_next_bridge_pre_enable(bridge);

	/*
	 * There's a bug in the PTN chip where it falsely asserts hotplug before
	 * it is fully functional. We're forced to wait for the maximum start up
	 * time specified in the chip's datasheet to make sure we're really up.
	 */
	msleep(90);

	ret = ptn3460_select_edid(ptn_bridge);
	if (ret)
		DRM_ERROR("Select edid failed ret=%d\n", ret);

	ptn_bridge->enabled = true;
}

static void ptn3460_enable(struct drm_bridge *bridge)
{
	drm_next_bridge_enable(bridge);
}

static void ptn3460_disable(struct drm_bridge *bridge)
{
	struct ptn3460_bridge *ptn_bridge = bridge->driver_private;

	if (!ptn_bridge->enabled)
		return;

	ptn_bridge->enabled = false;

	drm_next_bridge_disable(bridge);

	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_set_value(ptn_bridge->gpio_rst_n, 1);

	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_set_value(ptn_bridge->gpio_pd_n, 0);
}

static void ptn3460_post_disable(struct drm_bridge *bridge)
{
	drm_next_bridge_post_disable(bridge);
}

void ptn3460_bridge_destroy(struct drm_bridge *bridge)
{
	struct ptn3460_bridge *ptn_bridge = bridge->driver_private;

	drm_bridge_cleanup(bridge);
	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_free(ptn_bridge->gpio_pd_n);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_free(ptn_bridge->gpio_rst_n);

	drm_next_bridge_destroy(bridge);

	/* Nothing else to free, we've got devm allocated memory */
}

struct drm_bridge_funcs ptn3460_bridge_funcs = {
	.pre_enable = ptn3460_pre_enable,
	.enable = ptn3460_enable,
	.disable = ptn3460_disable,
	.post_disable = ptn3460_post_disable,
	.destroy = ptn3460_bridge_destroy,
};

struct drm_bridge *ptn3460_init(struct drm_device *dev,
				struct drm_encoder *encoder,
				struct i2c_client *client,
				struct device_node *node)
{
	int ret;
	struct drm_bridge *bridge;
	struct ptn3460_bridge *ptn_bridge;

	bridge = devm_kzalloc(dev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		DRM_ERROR("Failed to allocate drm bridge\n");
		return NULL;
	}

	ptn_bridge = devm_kzalloc(dev->dev, sizeof(*ptn_bridge), GFP_KERNEL);
	if (!ptn_bridge) {
		DRM_ERROR("Failed to allocate ptn bridge\n");
		return NULL;
	}

	ptn_bridge->client = client;
	ptn_bridge->encoder = encoder;
	ptn_bridge->bridge = bridge;
	ptn_bridge->gpio_pd_n = of_get_named_gpio(node, "powerdown-gpio", 0);
	if (gpio_is_valid(ptn_bridge->gpio_pd_n)) {
		ret = gpio_request_one(ptn_bridge->gpio_pd_n,
				GPIOF_OUT_INIT_HIGH, "PTN3460_PD_N");
		if (ret) {
			DRM_ERROR("Request powerdown-gpio failed (%d)\n", ret);
			return NULL;
		}
	}

	ptn_bridge->gpio_rst_n = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n)) {
		/*
		 * Request the reset pin low to avoid the bridge being
		 * initialized prematurely
		 */
		ret = gpio_request_one(ptn_bridge->gpio_rst_n,
				GPIOF_OUT_INIT_LOW, "PTN3460_RST_N");
		if (ret) {
			DRM_ERROR("Request reset-gpio failed (%d)\n", ret);
			gpio_free(ptn_bridge->gpio_pd_n);
			return NULL;
		}
	}

	ret = of_property_read_u32(node, "edid-emulation",
			&ptn_bridge->edid_emulation);
	if (ret) {
		DRM_ERROR("Can't read edid emulation value\n");
		goto err;
	}

	ret = drm_bridge_init(dev, bridge, &ptn3460_bridge_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		goto err;
	}

	bridge->driver_private = ptn_bridge;

	if (!encoder->bridge)
		/* First entry in the bridge chain */
		encoder->bridge = bridge;

	return bridge;

err:
	if (gpio_is_valid(ptn_bridge->gpio_pd_n))
		gpio_free(ptn_bridge->gpio_pd_n);
	if (gpio_is_valid(ptn_bridge->gpio_rst_n))
		gpio_free(ptn_bridge->gpio_rst_n);
	return NULL;
}
EXPORT_SYMBOL(ptn3460_init);
