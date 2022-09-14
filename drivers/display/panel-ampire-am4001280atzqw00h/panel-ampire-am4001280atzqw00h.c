// SPDX-License-Identifier: GPL-2.0-only

/**
 * Ampire AM-4001280ATZQW-00H MIPI-DSI panel driver

 * Author: 
 * Jan Greiner <jan.greiner@mnet-mail.de>
 */

/*
 * Copyright (C) 2022 Jan Greiner
 *
 * This driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License V2 as published by the Free Software Foundation.

 * This driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See the GNU General Public License for more details.

 * You should have received a copy of the GNU General Public
 * License along with this driver; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */


#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

/*
 * There is no description in the Reference Manual about these commands.
 * We received them from vendor, so just use them as is.
 */
static const struct cmd_set_entry mcs_am40001280[] = {
  {0xB0,0x5A}, {0xB1,0x00}, {0x89,0x01}, {0x91,0x07},
  {0x92,0xF9}, {0xB1,0x03}, {0x2C,0x28}, {0x00,0xB7},
  {0x01,0x1B}, {0x02,0x00}, {0x03,0x00}, {0x04,0x00},
  {0x05,0x00}, {0x06,0x00}, {0x07,0x00}, {0x08,0x00},
  {0x09,0x00}, {0x0A,0x01}, {0x0B,0x01}, {0x0C,0x20},
  {0x0D,0x00}, {0x0E,0x24}, {0x0F,0x1C}, {0x10,0xC9},
  {0x11,0x60}, {0x12,0x70}, {0x13,0x01}, {0x14,0xE7},
  {0x15,0xFF}, {0x16,0x3D}, {0x17,0x0E}, {0x18,0x01},
  {0x19,0x00}, {0x1A,0x00}, {0x1B,0xFC}, {0x1C,0x0B},
  {0x1D,0xA0}, {0x1E,0x03}, {0x1F,0x04}, {0x20,0x0C},
  {0x21,0x00}, {0x22,0x04}, {0x23,0x81}, {0x24,0x1F},
  {0x25,0x10}, {0x26,0x9B}, {0x2D,0x01}, {0x2E,0x84},
  {0x2F,0x00}, {0x30,0x02}, {0x31,0x08}, {0x32,0x01},
  {0x33,0x1C}, {0x34,0x40}, {0x35,0xFF}, {0x36,0xFF},
  {0x37,0xFF}, {0x38,0xFF}, {0x39,0xFF}, {0x3A,0x05},
  {0x3B,0x00}, {0x3C,0x00}, {0x3D,0x00}, {0x3E,0xCF},
  {0x3F,0x84}, {0x40,0x28}, {0x41,0xFC}, {0x42,0x01},
  {0x43,0x40}, {0x44,0x05}, {0x45,0xE8}, {0x46,0x16},
  {0x47,0x00}, {0x48,0x00}, {0x49,0x88}, {0x4A,0x08},
  {0x4B,0x05}, {0x4C,0x03}, {0x4D,0xD0}, {0x4E,0x13},
  {0x4F,0xFF}, {0x50,0x0A}, {0x51,0x53}, {0x52,0x26},
  {0x53,0x22}, {0x54,0x09}, {0x55,0x22}, {0x56,0x00},
  {0x57,0x1C}, {0x58,0x03}, {0x59,0x3F}, {0x5A,0x28},
  {0x5B,0x01}, {0x5C,0xCC}, {0x5D,0x21}, {0x5E,0x84},
  {0x5F,0x10}, {0x60,0x42}, {0x61,0x40}, {0x62,0x06},
  {0x63,0x3A}, {0x64,0xA6}, {0x65,0x04}, {0x66,0x09},
  {0x67,0x21}, {0x68,0x84}, {0x69,0x10}, {0x6A,0x42},
  {0x6B,0x08}, {0x6C,0x21}, {0x6D,0x84}, {0x6E,0x74},
  {0x6F,0xE2}, {0x70,0x6B}, {0x71,0x6B}, {0x72,0x94},
  {0x73,0x10}, {0x74,0x42}, {0x75,0x08}, {0x76,0x00},
  {0x77,0x00}, {0x78,0x0F}, {0x79,0xE0}, {0x7A,0x01},
  {0x7B,0xFF}, {0x7C,0xFF}, {0x7D,0x0F}, {0x7E,0x41},
  {0x7F,0xFE}, {0xB1,0x02}, {0x00,0xFF}, {0x01,0x05},
  {0x02,0xC8}, {0x03,0x00}, {0x04,0x14}, {0x05,0x4B},
  {0x06,0x64}, {0x07,0x0A}, {0x08,0xC0}, {0x09,0x00},
  {0x0A,0x00}, {0x0B,0x10}, {0x0C,0xE6}, {0x0D,0x0D},
  {0x0F,0x00}, {0x10,0x3D}, {0x11,0x4C}, {0x12,0xCF},
  {0x13,0xAD}, {0x14,0x4A}, {0x15,0x92}, {0x16,0x24},
  {0x17,0x55}, {0x18,0x73}, {0x19,0xE9}, {0x1A,0x70},
  {0x1B,0x0E}, {0x1C,0xFF}, {0x1D,0xFF}, {0x1E,0xFF},
  {0x1F,0xFF}, {0x20,0xFF}, {0x21,0xFF}, {0x22,0xFF},
  {0x23,0xFF}, {0x24,0xFF}, {0x25,0xFF}, {0x26,0xFF},
  {0x27,0x1F}, {0x28,0xFF}, {0x29,0xFF}, {0x2A,0xFF},
  {0x2B,0xFF}, {0x2C,0xFF}, {0x2D,0x07}, {0x33,0x3F},
  {0x35,0x7F}, {0x36,0x3F}, {0x38,0xFF}, {0x3A,0x80},
  {0x3B,0x01}, {0x3C,0x80}, {0x3D,0x2C}, {0x3E,0x00},
  {0x3F,0x90}, {0x40,0x05}, {0x41,0x00}, {0x42,0xB2},
  {0x43,0x00}, {0x44,0x40}, {0x45,0x06}, {0x46,0x00},
  {0x47,0x00}, {0x48,0x9B}, {0x49,0xD2}, {0x4A,0x21},
  {0x4B,0x43}, {0x4C,0x16}, {0x4D,0xC0}, {0x4E,0x0F},
  {0x4F,0xF1}, {0x50,0x78}, {0x51,0x7A}, {0x52,0x34},
  {0x53,0x99}, {0x54,0xA2}, {0x55,0x02}, {0x56,0x14},
  {0x57,0xB8}, {0x58,0xDC}, {0x59,0xD4}, {0x5A,0xEF},
  {0x5B,0xF7}, {0x5C,0xFB}, {0x5D,0xFD}, {0x5E,0x7E},
  {0x5F,0xBF}, {0x60,0xEF}, {0x61,0xE6}, {0x62,0x76},
  {0x63,0x73}, {0x64,0xBB}, {0x65,0xDD}, {0x66,0x6E},
  {0x67,0x37}, {0x68,0x8C}, {0x69,0x08}, {0x6A,0x31},
  {0x6B,0xB8}, {0x6C,0xB8}, {0x6D,0xB8}, {0x6E,0xB8},
  {0x6F,0xB8}, {0x70,0x5C}, {0x71,0x2E}, {0x72,0x17},
  {0x73,0x00}, {0x74,0x00}, {0x75,0x00}, {0x76,0x00},
  {0x77,0x00}, {0x78,0x00}, {0x79,0x00}, {0x7A,0xDC},
  {0x7B,0xDC}, {0x7C,0xDC}, {0x7D,0xDC}, {0x7E,0xDC},
  {0x7F,0x6E}, {0x0B,0x00}, {0xB1,0x03}, {0x2C,0x2C},
  {0xB1,0x00}, {0x89,0x03}
};

static const u32 ampire_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 ampire_bus_flags = DRM_BUS_FLAG_DE_LOW | DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

struct ampire_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct ampire_platform_data *pdata;
};

struct ampire_platform_data {
	int (*enable)(struct ampire_panel *panel);
};

static const struct drm_display_mode default_mode = {
	.clock = 200000, // 200000 is a good clock rate
	.hdisplay = 400,
	.hsync_start = 400 + 30,
	.hsync_end = 400 + 5 + 40,
	.htotal = 400 + 30 + 5 + 40,
	.vdisplay = 1280,
	.vsync_start = 1280 + 30,
	.vsync_end = 1280 + 20 + 30,
	.vtotal = 1280 + 30 + 20 + 30,
	.width_mm = 190,
	.height_mm = 59,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static inline struct ampire_panel *to_ampire_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ampire_panel, panel);
}

static int ampire_panel_push_cmd_list(struct mipi_dsi_device *dsi,
				   struct cmd_set_entry const *cmd_set,
				   size_t count)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = cmd_set++;
		u8 buffer[2] = { entry->cmd, entry->param };

		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}

	return ret;
};

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return COL_FMT_16BPP;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return COL_FMT_18BPP;
	case MIPI_DSI_FMT_RGB888:
		return COL_FMT_24BPP;
	default:
		return COL_FMT_24BPP; /* for backward compatibility */
	}
};

static int ampire_panel_prepare(struct drm_panel *panel)
{
	struct ampire_panel *amp = to_ampire_panel(panel);
	int ret;

	printk("Panel prepare");

	if (amp->prepared)
		return 0;

	ret = regulator_bulk_enable(amp->num_supplies, amp->supplies);
	if (ret)
		return ret;

	/* At lest 10ms needed between power-on and reset-out as RM specifies */
	usleep_range(10000, 12000);

	if (amp->reset) {
		gpiod_set_value_cansleep(amp->reset, 0);
		/*
		 * 50ms delay after reset-out, as per manufacturer initalization
		 * sequence.
		 */
		msleep(50);
	}

	amp->prepared = true;

	return 0;
}

static int ampire_panel_unprepare(struct drm_panel *panel)
{
	struct ampire_panel *amp = to_ampire_panel(panel);
	int ret;

	if (!amp->prepared)
		return 0;

	/*
	 * Right after asserting the reset, we need to release it, so that the
	 * touch driver can have an active connection with the touch controller
	 * even after the display is turned off.
	 */
	if (amp->reset) {
		gpiod_set_value_cansleep(amp->reset, 1);
		usleep_range(15000, 17000);
		gpiod_set_value_cansleep(amp->reset, 0);
	}

	ret = regulator_bulk_disable(amp->num_supplies, amp->supplies);
	if (ret)
		return ret;

	amp->prepared = false;

	return 0;
}

static int am40001280_enable(struct ampire_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	printk("Panel platform enable");

	if (panel->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		goto fail;
	}

	mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display off (%d)\n", ret);
		goto fail;
	}

	ret = ampire_panel_push_cmd_list(dsi, &mcs_am40001280[0], ARRAY_SIZE(mcs_am40001280));
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}

	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 7000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	backlight_enable(panel->backlight);

	panel->enabled = true;

	return 0;

fail:
	gpiod_set_value_cansleep(panel->reset, 1);

	return ret;
}


static int ampire_panel_enable(struct drm_panel *panel)
{
	struct ampire_panel *amp = to_ampire_panel(panel);

	printk("Panel enable");

	return amp->pdata->enable(amp);
}

static int ampire_panel_disable(struct drm_panel *panel)
{
	struct ampire_panel *amp = to_ampire_panel(panel);
	struct mipi_dsi_device *dsi = amp->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	printk("Panel disable");

	if (!amp->enabled)
		return 0;

	backlight_disable(amp->backlight);

	usleep_range(10000, 12000);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	amp->enabled = false;

	return 0;
}

static int ampire_panel_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_DEV_ERROR(panel->dev, "failed to add mode %ux%ux@%u\n",
			      default_mode.hdisplay, default_mode.vdisplay,
			      default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = ampire_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 ampire_bus_formats,
					 ARRAY_SIZE(ampire_bus_formats));
	return 1;
}

static int ampire_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ampire_panel *amp = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	if (!amp->prepared)
		return 0;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int ampire_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ampire_panel *amp = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!amp->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops ampire_bl_ops = {
	.update_status = ampire_bl_update_status,
	.get_brightness = ampire_bl_get_brightness,
};

static const struct drm_panel_funcs ampire_panel_funcs = {
	.prepare = ampire_panel_prepare,
	.unprepare = ampire_panel_unprepare,
	.enable = ampire_panel_enable,
	.disable = ampire_panel_disable,
	.get_modes = ampire_panel_get_modes,
};

static const char * const ampire_supply_names[] = {
	"v3p3",
};

static int ampire_init_regulators(struct ampire_panel *amp)
{
	struct device *dev = &amp->dsi->dev;
	int i;

	amp->num_supplies = ARRAY_SIZE(ampire_supply_names);
	amp->supplies = devm_kcalloc(dev, amp->num_supplies,
				     sizeof(*amp->supplies), GFP_KERNEL);
	if (!amp->supplies)
		return -ENOMEM;

	for (i = 0; i < amp->num_supplies; i++)
		amp->supplies[i].supply = ampire_supply_names[i];

	return devm_regulator_bulk_get(dev, amp->num_supplies, amp->supplies);
};

static const struct ampire_platform_data ampire_am40001280= {
	.enable = &am40001280_enable,
};

static const struct of_device_id ampire_of_match[] = {
	{ .compatible = "ampire,am40001280", .data = &ampire_am40001280},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ampire_of_match);

static int ampire_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(ampire_of_match, dev);
	struct device_node *np = dev->of_node;
	struct ampire_panel *panel;
	struct backlight_properties bl_props;
	int ret;
	u32 video_mode;

	printk("Panel probe");

	if (!of_id || !of_id->data)
		return -ENODEV;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->pdata = of_id->data;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	panel->reset = devm_gpiod_get_optional(dev, "reset",
					       GPIOD_OUT_LOW |
					       GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(panel->reset)) {
		ret = PTR_ERR(panel->reset);
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}
	gpiod_set_value_cansleep(panel->reset, 1);

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, dsi, &ampire_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	ret = ampire_init_regulators(panel);
	if (ret)
		return ret;

	drm_panel_init(&panel->panel);
	panel->panel.funcs = &ampire_panel_funcs;
	panel->panel.dev = dev;
	dev_set_drvdata(dev, panel);

	ret = drm_panel_add(&panel->panel);
	if (ret)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	return ret;
}

static int ampire_panel_remove(struct mipi_dsi_device *dsi)
{
	struct ampire_panel *amp = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	printk("Panel remove");

	ret = mipi_dsi_detach(dsi);
	if (ret)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			      ret);

	drm_panel_remove(&amp->panel);

	return 0;
}

static void ampire_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct ampire_panel *amp = mipi_dsi_get_drvdata(dsi);

	printk("Panel shutdown");

	ampire_panel_disable(&amp->panel);
	ampire_panel_unprepare(&amp->panel);
}

static struct mipi_dsi_driver ampire_panel_driver = {
	.driver = {
		.name = "panel-ampire-am40001280",
		.of_match_table = ampire_of_match,
	},
	.probe = ampire_panel_probe,
	.remove = ampire_panel_remove,
	.shutdown = ampire_panel_shutdown,
};
module_mipi_dsi_driver(ampire_panel_driver);

MODULE_AUTHOR("Jan Greiner <jan.greiner@mnet-mail.de>");
MODULE_DESCRIPTION("DRM driver for the Ampire AM-4001280ATZQW-00H MIPI DSI panel");
MODULE_LICENSE("GPL v2");