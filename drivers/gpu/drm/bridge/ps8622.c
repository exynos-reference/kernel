/*
 * Parade PS8622 eDP/LVDS bridge driver
 *
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

#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"

struct ps8622_bridge {
	struct drm_bridge *bridge;
	struct drm_encoder *encoder;
	struct i2c_client *client;
	struct regulator *v12;
	struct backlight_device *bl;
	struct mutex enable_mutex;

	int gpio_slp_n;
	int gpio_rst_n;

	u8 max_lane_count;
	u8 lane_count;

	bool enabled;
};

struct ps8622_device_data {
	u8 max_lane_count;
};

static const struct ps8622_device_data ps8622_data = {
	.max_lane_count = 1,
};

static const struct ps8622_device_data ps8625_data = {
	.max_lane_count = 2,
};

/* Brightness scale on the Parade chip */
#define PS8622_MAX_BRIGHTNESS 0xff

/* Timings taken from the version 1.7 datasheet for the PS8622/PS8625 */
#define PS8622_POWER_RISE_T1_MIN_US 10
#define PS8622_POWER_RISE_T1_MAX_US 10000
#define PS8622_RST_HIGH_T2_MIN_US 3000
#define PS8622_RST_HIGH_T2_MAX_US 30000
#define PS8622_PWMO_END_T12_MS 200
#define PS8622_POWER_FALL_T16_MAX_US 10000
#define PS8622_POWER_OFF_T17_MS 500

#if ((PS8622_RST_HIGH_T2_MIN_US + PS8622_POWER_RISE_T1_MAX_US) > \
	(PS8622_RST_HIGH_T2_MAX_US + PS8622_POWER_RISE_T1_MIN_US))
#error "T2.min + T1.max must be less than T2.max + T1.min"
#endif

static int ps8622_set(struct i2c_client *client, u8 page, u8 reg, u8 val)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 data[] = {reg, val};

	msg.addr = client->addr + page;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1)
		pr_warn("PS8622 I2C write (0x%02x,0x%02x,0x%02x) failed: %d\n",
			client->addr + page, reg, val, ret);
	return !(ret == 1);
}

static int ps8622_send_config(struct ps8622_bridge *ps_bridge)
{
	struct i2c_client *cl = ps_bridge->client;
	int err = 0;

	/* wait 20ms after power ON */
	usleep_range(20000, 30000);

	err |= ps8622_set(cl, 0x02, 0xa1, 0x01); /* HPD low */
	/* SW setting */
	err |= ps8622_set(cl, 0x04, 0x14, 0x01); /* [1:0] SW output 1.2V voltage
						  * is lower to 96% */
	/* RCO SS setting */
	err |= ps8622_set(cl, 0x04, 0xe3, 0x20); /* [5:4] = b01 0.5%, b10 1%,
						  * b11 1.5% */
	err |= ps8622_set(cl, 0x04, 0xe2, 0x80); /* [7] RCO SS enable */
	/* RPHY Setting */
	err |= ps8622_set(cl, 0x04, 0x8a, 0x0c); /* [3:2] CDR tune wait cycle
						  * before measure for fine tune
						  * b00: 1us b01: 0.5us b10:2us
						  * b11: 4us */
	err |= ps8622_set(cl, 0x04, 0x89, 0x08); /* [3] RFD always on */
	err |= ps8622_set(cl, 0x04, 0x71, 0x2d); /* CTN lock in/out:
						  * 20000ppm/80000ppm.
						  * Lock out 2 times. */
	/* 2.7G CDR settings */
	err |= ps8622_set(cl, 0x04, 0x7d, 0x07); /* NOF=40LSB for HBR CDR
						  * setting */
	err |= ps8622_set(cl, 0x04, 0x7b, 0x00); /* [1:0] Fmin=+4bands */
	err |= ps8622_set(cl, 0x04, 0x7a, 0xfd); /* [7:5] DCO_FTRNG=+-40% */
	/* 1.62G CDR settings */
	err |= ps8622_set(cl, 0x04, 0xc0, 0x12); /* [5:2]NOF=64LSB [1:0]DCO
						  * scale is 2/5 */
	err |= ps8622_set(cl, 0x04, 0xc1, 0x92); /* Gitune=-37% */
	err |= ps8622_set(cl, 0x04, 0xc2, 0x1c); /* Fbstep=100% */
	err |= ps8622_set(cl, 0x04, 0x32, 0x80); /* [7] LOS signal disable */
	/* RPIO Setting */
	err |= ps8622_set(cl, 0x04, 0x00, 0xb0); /* [7:4] LVDS driver bias
						  * current : 75% (250mV swing)
						  * */
	err |= ps8622_set(cl, 0x04, 0x15, 0x40); /* [7:6] Right-bar GPIO output
						  * strength is 8mA */
	/* EQ Training State Machine Setting */
	err |= ps8622_set(cl, 0x04, 0x54, 0x10); /* RCO calibration start */
	/* Logic, needs more than 10 I2C command */
	err |= ps8622_set(cl, 0x01, 0x02, 0x80 | ps_bridge->max_lane_count);
						 /* [4:0] MAX_LANE_COUNT set to
						  * max supported lanes */
	err |= ps8622_set(cl, 0x01, 0x21, 0x80 | ps_bridge->lane_count);
						 /* [4:0] LANE_COUNT_SET set to
						  * chosen lane count */
	err |= ps8622_set(cl, 0x00, 0x52, 0x20);
	err |= ps8622_set(cl, 0x00, 0xf1, 0x03); /* HPD CP toggle enable */
	err |= ps8622_set(cl, 0x00, 0x62, 0x41);
	err |= ps8622_set(cl, 0x00, 0xf6, 0x01); /* Counter number, add 1ms
						  * counter delay */
	err |= ps8622_set(cl, 0x00, 0x77, 0x06); /* [6]PWM function control by
						  * DPCD0040f[7], default is PWM
						  * block always works. */
	err |= ps8622_set(cl, 0x00, 0x4c, 0x04); /* 04h Adjust VTotal tolerance
						  * to fix the 30Hz no display
						  * issue */
	err |= ps8622_set(cl, 0x01, 0xc0, 0x00); /* DPCD00400='h00, Parade OUI =
						  * 'h001cf8 */
	err |= ps8622_set(cl, 0x01, 0xc1, 0x1c); /* DPCD00401='h1c */
	err |= ps8622_set(cl, 0x01, 0xc2, 0xf8); /* DPCD00402='hf8 */
	err |= ps8622_set(cl, 0x01, 0xc3, 0x44); /* DPCD403~408 = ASCII code
						  * D2SLV5='h4432534c5635 */
	err |= ps8622_set(cl, 0x01, 0xc4, 0x32); /* DPCD404 */
	err |= ps8622_set(cl, 0x01, 0xc5, 0x53); /* DPCD405 */
	err |= ps8622_set(cl, 0x01, 0xc6, 0x4c); /* DPCD406 */
	err |= ps8622_set(cl, 0x01, 0xc7, 0x56); /* DPCD407 */
	err |= ps8622_set(cl, 0x01, 0xc8, 0x35); /* DPCD408 */
	err |= ps8622_set(cl, 0x01, 0xca, 0x01); /* DPCD40A, Initial Code major
						  * revision '01' */
	err |= ps8622_set(cl, 0x01, 0xcb, 0x05); /* DPCD40B, Initial Code minor
						  * revision '05' */
	if (ps_bridge->bl) {
		err |= ps8622_set(cl, 0x01, 0xa5, 0xa0);
						/* DPCD720, internal PWM */
		err |= ps8622_set(cl, 0x01, 0xa7,
				ps_bridge->bl->props.brightness);
						 /* FFh for 100% brightness,
						  *  0h for 0% brightness */
	} else {
		err |= ps8622_set(cl, 0x01, 0xa5, 0x80);
						/* DPCD720, external PWM */
	}
	err |= ps8622_set(cl, 0x01, 0xcc, 0x13); /* Set LVDS output as 6bit-VESA
						  * mapping, single LVDS channel
						  * */
	err |= ps8622_set(cl, 0x02, 0xb1, 0x20); /* Enable SSC set by register
						  * */
	err |= ps8622_set(cl, 0x04, 0x10, 0x16); /* Set SSC enabled and +/-1%
						  * central spreading */
	/* Logic end */
	err |= ps8622_set(cl, 0x04, 0x59, 0x60); /* MPU Clock source: LC => RCO
						  * */
	err |= ps8622_set(cl, 0x04, 0x54, 0x14); /* LC -> RCO */
	err |= ps8622_set(cl, 0x02, 0xa1, 0x91); /* HPD high */

	return err ? -EIO : 0;
}

static int ps8622_backlight_update(struct backlight_device *bl)
{
	struct ps8622_bridge *ps_bridge = dev_get_drvdata(&bl->dev);
	int ret, brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	mutex_lock(&ps_bridge->enable_mutex);

	if (!ps_bridge->enabled) {
		ret = -EINVAL;
		goto out;
	}

	ret = ps8622_set(ps_bridge->client, 0x01, 0xa7, brightness);

out:
	mutex_unlock(&ps_bridge->enable_mutex);
	return ret;
}

static int ps8622_backlight_get(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops ps8622_backlight_ops = {
	.update_status	= ps8622_backlight_update,
	.get_brightness	= ps8622_backlight_get,
};

static void ps8622_pre_enable(struct drm_bridge *bridge)
{
	struct ps8622_bridge *ps_bridge = bridge->driver_private;
	int ret;

	mutex_lock(&ps_bridge->enable_mutex);
	if (ps_bridge->enabled)
		goto out;

	if (gpio_is_valid(ps_bridge->gpio_rst_n))
		gpio_set_value(ps_bridge->gpio_rst_n, 0);

	if (ps_bridge->v12) {
		ret = regulator_enable(ps_bridge->v12);
		if (ret)
			DRM_ERROR("fails to enable ps_bridge->v12");
	}

	drm_next_bridge_pre_enable(bridge);

	if (gpio_is_valid(ps_bridge->gpio_slp_n))
		gpio_set_value(ps_bridge->gpio_slp_n, 1);

	/*
	 * T1 is the range of time that it takes for the power to rise after we
	 * enable the lcd fet. T2 is the range of time in which the data sheet
	 * specifies we should deassert the reset pin.
	 *
	 * If it takes T1.max for the power to rise, we need to wait atleast
	 * T2.min before deasserting the reset pin. If it takes T1.min for the
	 * power to rise, we need to wait at most T2.max before deasserting the
	 * reset pin.
	 */
	usleep_range(PS8622_RST_HIGH_T2_MIN_US + PS8622_POWER_RISE_T1_MAX_US,
		     PS8622_RST_HIGH_T2_MAX_US + PS8622_POWER_RISE_T1_MIN_US);

	if (gpio_is_valid(ps_bridge->gpio_rst_n))
		gpio_set_value(ps_bridge->gpio_rst_n, 1);

	ret = ps8622_send_config(ps_bridge);
	if (ret)
		DRM_ERROR("Failed to send config to bridge (%d)\n", ret);

	ps_bridge->enabled = true;

out:
	mutex_unlock(&ps_bridge->enable_mutex);
}

static void ps8622_enable(struct drm_bridge *bridge)
{
	struct ps8622_bridge *ps_bridge = bridge->driver_private;

	mutex_lock(&ps_bridge->enable_mutex);
	drm_next_bridge_enable(bridge);
	mutex_unlock(&ps_bridge->enable_mutex);
}

static void ps8622_disable(struct drm_bridge *bridge)
{
	struct ps8622_bridge *ps_bridge = bridge->driver_private;

	mutex_lock(&ps_bridge->enable_mutex);

	if (!ps_bridge->enabled)
		goto out;

	ps_bridge->enabled = false;

	drm_next_bridge_disable(bridge);
	msleep(PS8622_PWMO_END_T12_MS);

	/*
	 * This doesn't matter if the regulators are turned off, but something
	 * else might keep them on. In that case, we want to assert the slp gpio
	 * to lower power.
	 */
	if (gpio_is_valid(ps_bridge->gpio_slp_n))
		gpio_set_value(ps_bridge->gpio_slp_n, 0);

	if (ps_bridge->v12)
		regulator_disable(ps_bridge->v12);

	/*
	 * Sleep for at least the amount of time that it takes the power rail to
	 * fall to prevent asserting the rst gpio from doing anything.
	 */
	usleep_range(PS8622_POWER_FALL_T16_MAX_US,
		     2 * PS8622_POWER_FALL_T16_MAX_US);
	if (gpio_is_valid(ps_bridge->gpio_rst_n))
		gpio_set_value(ps_bridge->gpio_rst_n, 0);

	msleep(PS8622_POWER_OFF_T17_MS);

out:
	mutex_unlock(&ps_bridge->enable_mutex);
}

static void ps8622_post_disable(struct drm_bridge *bridge)
{
	drm_next_bridge_post_disable(bridge);
}

static void ps8622_destroy(struct drm_bridge *bridge)
{
	struct ps8622_bridge *ps_bridge = bridge->driver_private;

	drm_bridge_cleanup(bridge);
	if (ps_bridge->bl)
		backlight_device_unregister(ps_bridge->bl);

	put_device(&ps_bridge->client->dev);
}

static const struct drm_bridge_funcs ps8622_bridge_funcs = {
	.pre_enable = ps8622_pre_enable,
	.enable = ps8622_enable,
	.disable = ps8622_disable,
	.post_disable = ps8622_post_disable,
	.destroy = ps8622_destroy,
};

static const struct of_device_id ps8622_devices[] = {
	{
		.compatible = "parade,ps8622",
		.data	= &ps8622_data,
	}, {
		.compatible = "parade,ps8625",
		.data	= &ps8625_data,
	}, {
		/* end node */
	}
};

struct drm_bridge *ps8622_init(struct drm_device *dev,
				struct drm_encoder *encoder,
				struct i2c_client *client,
				struct device_node *node)
{
	int ret;
	const struct of_device_id *match;
	const struct ps8622_device_data *device_data;
	struct drm_bridge *bridge;
	struct ps8622_bridge *ps_bridge;

	node = of_find_matching_node_and_match(NULL, ps8622_devices, &match);
	if (!node)
		return NULL;

	bridge = devm_kzalloc(dev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		DRM_ERROR("Failed to allocate drm bridge\n");
		return NULL;
	}

	ps_bridge = devm_kzalloc(dev->dev, sizeof(*ps_bridge), GFP_KERNEL);
	if (!ps_bridge) {
		DRM_ERROR("could not allocate ps bridge\n");
		return NULL;
	}

	mutex_init(&ps_bridge->enable_mutex);

	device_data = match->data;
	ps_bridge->client = client;
	ps_bridge->encoder = encoder;
	ps_bridge->bridge = bridge;
	ps_bridge->v12 = devm_regulator_get(&client->dev, "vdd_bridge");
	if (IS_ERR(ps_bridge->v12)) {
		DRM_INFO("no 1.2v regulator found for PS8622\n");
		ps_bridge->v12 = NULL;
	}

	ps_bridge->gpio_slp_n = of_get_named_gpio(node, "sleep-gpio", 0);
	if (gpio_is_valid(ps_bridge->gpio_slp_n)) {
		ret = devm_gpio_request_one(&client->dev, ps_bridge->gpio_slp_n,
					    GPIOF_OUT_INIT_HIGH,
					    "PS8622_SLP_N");
		if (ret)
			goto err_client;
	}

	ps_bridge->gpio_rst_n = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(ps_bridge->gpio_rst_n)) {
		/*
		 * Assert the reset pin high to avoid the bridge being
		 * initialized prematurely
		 */
		ret = devm_gpio_request_one(&client->dev, ps_bridge->gpio_rst_n,
					    GPIOF_OUT_INIT_HIGH,
					    "PS8622_RST_N");
		if (ret)
			goto err_client;
	}

	ps_bridge->max_lane_count = device_data->max_lane_count;

	if (of_property_read_u8(node, "lane-count", &ps_bridge->lane_count))
		ps_bridge->lane_count = ps_bridge->max_lane_count;
	else if (ps_bridge->lane_count > ps_bridge->max_lane_count) {
		DRM_ERROR("lane-count property is too high for DP bridge\n");
		ps_bridge->lane_count = ps_bridge->max_lane_count;
	}

	if (!of_find_property(node, "use-external-pwm", NULL)) {
		ps_bridge->bl = backlight_device_register("ps8622-backlight",
				dev->dev, ps_bridge, &ps8622_backlight_ops,
				NULL);
		if (IS_ERR(ps_bridge->bl)) {
			DRM_ERROR("failed to register backlight\n");
			ret = PTR_ERR(ps_bridge->bl);
			ps_bridge->bl = NULL;
			goto err_client;
		}
		ps_bridge->bl->props.max_brightness = PS8622_MAX_BRIGHTNESS;
		ps_bridge->bl->props.brightness = PS8622_MAX_BRIGHTNESS;
	}

	ret = drm_bridge_init(dev, ps_bridge->bridge, &ps8622_bridge_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		goto err_backlight;
	}

	ps_bridge->bridge->driver_private = ps_bridge;
	ps_bridge->enabled = false;

	if (!encoder->bridge)
		/* First entry in the bridge chain */
		encoder->bridge = bridge;

	return bridge;

err_backlight:
	if (ps_bridge->bl)
		backlight_device_unregister(ps_bridge->bl);
err_client:
	put_device(&client->dev);
	DRM_ERROR("device probe failed : %d\n", ret);
	return NULL;
}
EXPORT_SYMBOL(ps8622_init);
