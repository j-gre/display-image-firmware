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
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/media-bus-format.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/** Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

/** Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

/** Amount of volage/current regulators. */
#define DCS_REGULATOR_SUPPLY_NUM 2

/** Manufacturer Command Set pages (CMD2) format */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

/**
 * Command Set Pages received from Ampire.
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

/**
 * Define a custom panel driver format.
 * This is generated via "panel_to_drv_data" from an instance of "drm_panel". 
 */
struct panel_driver_data {
	struct mipi_dsi_device *dsi;
	struct drm_panel panel;
	struct drm_display_mode mode;

	struct mutex lock;

	struct backlight_device *bl_dev;

	struct regulator *supply;
	struct i2c_adapter *ddc;

	unsigned long hw_guard_end;
	unsigned long hw_guard_wait;

	const struct drm_panel_data *panel_data;
	const struct platform_data *pl_data;
	struct gpio_desc *enable_pin;

	struct gpio_desc *reset_pin;

	struct edid *edid;

	struct regulator_bulk_data *supplies;
	int num_supplies;

	/* Runtime variables */
	bool prepared;
	bool enabled;
	bool suspended;

	enum drm_panel_orientation orientation;

	bool intro_printed;
};

/**
 *
 */
struct platform_data {
	int (*enable)(struct panel_driver_data *drv_data);
};

/**
 * Define a custom format for additional panel data.
 */
struct drm_panel_data {
	
	/**
	 * @modes: Pointer to array of fixed modes appropriate for this panel.
	 */
	const struct drm_display_mode *modes;

	/**
	 * @timings: Pointer to array of display timings.
	 
	 * NOTE: cannot be used with "modes" and also these will be used to
	 * validate a device tree override if one is present.
	 */
	const struct display_timing *timings;

	/** @num_timings: Number of elements in timings array. */
	u32 num_timings;

	/** @bpc: Bits per color. */
	u32 bpc;

	/** @size: Structure containing the physical size of this panel. */
	struct {
		/**
		 * @size.width: Width (in mm) of the active display area.
		 */
		u32 width;

		/**
		 * @size.height: Height (in mm) of the active display area.
		 */
		u32 height;
	} size;

	/** @res: Structure containing the resolution of this panel. */
	struct {
		/** @xres: The horizontal display resolution (in dots). */
		u32 x;
		
		/** @yres: The vertical display resolution (in dots). */
		u32 y;
	} res;

	/** @refresh: Refresh rate framerate (in Hz). */
	u32 refresh;

	/** @max_hs_rate: Maximum data transfer rate in highspeed mode. */
	u32 max_hs_rate;
	/** @max_lp_rate: Maximum data transfer rate in lowpseed mode. */
	u32 max_lp_rate;

	/** Support for the tearing effect output signal on the TE signal line */
	bool tearing_effect_support;

	/** @delay: Structure containing various delay values for this panel. */
	struct {
		/**
		 * @delay.prepare: Time for the panel to become ready.
		 *
		 * The time (in milliseconds) that it takes for the panel to
		 * become ready and start receiving video data
		 */
		u32 prepare;

		/**
		 * @delay.enable: Time for the panel to display a valid frame.
		 *
		 * The time (in milliseconds) that it takes for the panel to
		 * display the first valid frame after starting to receive
		 * video data.
		 */
		u32 enable;

		/**
		 * @delay.disable: Time for the panel to turn the display off.
		 *
		 * The time (in milliseconds) that it takes for the panel to
		 * turn the display off (no content is visible).
		 */
		u32 disable;

		/**
		 * @delay.unprepare: Time to power down completely.
		 *
		 * The time (in milliseconds) that it takes for the panel
		 * to power itself down completely.
		 *
		 * This time is used to prevent a future "prepare" from
		 * starting until at least this many milliseconds has passed.
		 * If at prepare time less time has passed since unprepare
		 * finished, the driver waits for the remaining time.
		 */
		u32 unprepare;
	} delay;

	/** @bus_flags: See DRM_BUS_FLAG_... defines. */
	u32 bus_flags;

	/** @connector_type: LVDS, eDP, DSI, DPI, etc. */
	int connector_type;
};

/**
 * Function pointer for getting the driver data from the panel specification
 */
static inline struct panel_driver_data *panel_to_drv_data(struct drm_panel *panel)
{
	return container_of(panel, struct panel_driver_data, panel);
}

/**
 *
 */
static int push_cmd_list(struct mipi_dsi_device *dsi, struct cmd_set_entry const *cmd_set, size_t count)
{
	int ret;
	size_t i;

	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = cmd_set++;
		u8 buffer[2] = { entry->cmd, entry->param };

		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}

	return 0;
};

/**
 *
 */
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
		return COL_FMT_24BPP;
	}
};

/**
 * Instance of drm_display_mode.
 * Referenced by an instance of drm_panel_data.
 */
static const struct drm_display_mode am4001280atzqw00h_mode = {
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
		 DRM_MODE_FLAG_NVSYNC
};

/**
 * Instance of drm_panel_data.
 * Later referenced by DSIC and MIPI functions and within "platform_of_match[]".
 */
static const struct drm_panel_data am4001280atzqw00h_data = {

	/* Reference the display mode(s) initialized earlier. */
	.modes = &am4001280atzqw00h_mode,
	.bpc = 8,
	.size = {
		.width = 59,
		.height = 190,
	},
	.res = {
		.x = 400,
		.y = 1280
	},
	.refresh = 60,
	.tearing_effect_support = false,
	/*
	.delay = {
		.prepare = 1000,
		.enable = 0,
		.disable = 0,
		.unprepare = 0
	},
	*/
	.bus_flags = DRM_BUS_FLAG_DE_LOW | DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE,
	.connector_type = DRM_MODE_CONNECTOR_DSI
};

static const u32 am4001280atzqw00h_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

/**
 * 
 */
static const char * const am4001280atzqw00h_supply_names[] = {
	"v3p3"
};

/**
 * === Panel functions ===
 */

/**
 * == DRM panel functions ==
 */

/**
 * 
 */
static int am4001280atzqw00h_prepare(struct drm_panel *panel)
{
	struct panel_driver_data *drv_data = panel_to_drv_data(panel);
	struct mipi_dsi_device *dsi = drv_data->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if(drv_data->prepared) {
		DRM_DEV_ERROR(dev, "Got call to prepare despite already being prepared (%d)\n", 1);
		return 1;
	}

	/** Enable voltage/current regulator clients */
	ret = regulator_bulk_enable(drv_data->num_supplies, drv_data->supplies);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enable voltage/current regulators while preparing (%d)\n", ret);
		return ret;
	}

	/** At lest 10ms needed between power-on and reset-out */
	usleep_range(10000, 12000);

	if (drv_data->reset_pin) {
		gpiod_set_value_cansleep(drv_data->reset_pin, 0);

		/** 50ms delay after reset-out */
		msleep(50);
	}				

	return 0;
}

/**
 * 
 */
static int am4001280atzqw00h_unprepare(struct drm_panel *panel)
{
	struct panel_driver_data *drv_data = panel_to_drv_data(panel);
	struct mipi_dsi_device *dsi = drv_data->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if(drv_data->prepared) {
		DRM_DEV_ERROR(dev, "Got call to unprepare despite already not being prepared (%d)\n", 1);
		return 1;
	}

	if (drv_data->reset_pin) {
		gpiod_set_value_cansleep(drv_data->reset_pin, 1);
		usleep_range(15000, 17000);
		gpiod_set_value_cansleep(drv_data->reset_pin, 0);
	}

	ret = regulator_bulk_disable(drv_data->num_supplies, drv_data->supplies);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to disable voltage/current regulators while unpreparing (%d)\n", ret);
		return ret;
	}
	drv_data->prepared = false;

	return 0;
}

/**
 *
 */
static int am4001280atzqw00h_suspend(struct device *dev) 
{
	struct panel_driver_data *drv_data = dev_get_drvdata(dev);
	struct mipi_dsi_device *dsi = drv_data->dsi;
	int ret;

	if(drv_data->suspended) {
		DRM_DEV_ERROR(dev, "Got call to suspend despite already being suspended (%d)\n", 1);
		return 1;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	return 0;
}

/**
 *
 */
static int am4001280atzqw00h_resume(struct device *dev)
{
	struct panel_driver_data *drv_data = dev_get_drvdata(dev);
	struct mipi_dsi_device *dsi = drv_data->dsi;
	int ret;

	if(!drv_data->suspended) {
		DRM_DEV_ERROR(dev, "Got call to resume despite not being suspended (%d)\n", 1);
		return 1;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode (%d)\n", ret);
		return ret;
	}

	return 0;
}

/**
 * 
 */
static int am4001280atzqw00h_enable(struct drm_panel *panel)
{
	struct panel_driver_data *drv_data = panel_to_drv_data(panel);

	return drv_data->pl_data->enable(drv_data);
}

/**
 * 
 */
static int am4001280atzqw00h_platform_enable(struct panel_driver_data *drv_data)
{
	struct drm_panel *panel = &drv_data->panel;
	struct mipi_dsi_device *dsi = drv_data->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	if(drv_data->enabled) {
		DRM_DEV_ERROR(dev, "Got call to enable despite already being enabled (%d)\n", 1);
		return 1;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "Interface color format set to 0x%x\n", color_format);

	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to perform software reset (%d)\n", ret);
		goto fail;
	}
	
	/** Raise low power mode flag */
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = am4001280atzqw00h_suspend(dev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode while enabling (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display off while enabling (%d)\n", ret);
		goto fail;
	}

	ret = push_cmd_list(dsi, &mcs_am40001280[0], ARRAY_SIZE(mcs_am40001280));
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS while enabling (%d)\n", ret);
		goto fail;
	}

	ret = am4001280atzqw00h_resume(dev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode while enabling(%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 7000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display to on while enabling (%d)\n", ret);
		goto fail;
	}

	backlight_enable(drv_data->bl_dev);

	drv_data->enabled = true;

	return 0;

fail:
	gpiod_set_value_cansleep(drv_data->reset_pin, 1);

	return ret;

}


/**
 * 
 */
static int am4001280atzqw00h_disable(struct drm_panel *panel)
{
	struct panel_driver_data *drv_data = panel_to_drv_data(panel);
	struct mipi_dsi_device *dsi = drv_data->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if(drv_data->enabled) {
		DRM_DEV_ERROR(dev, "Got call to disable despite not being enabled (%d)\n", 1);
		return 1;
	}

	ret = backlight_disable(drv_data->bl_dev);

	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to disable backlight (%d)\n", ret);
		return ret;
	}

	usleep_range(10000, 12000);

	/** Switch to HP mode to send the command more quicky */
	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);

	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display to OFF while disabling (%d)\n", ret);
		return ret;
	}

	ret = am4001280atzqw00h_suspend(dev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode while disabling (%d)\n", ret);
		goto fail;
	}

	/** Switch back to LP mode*/
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	drv_data->enabled = false;

	return 0;

	fail:
		dsi->mode_flags |= MIPI_DSI_MODE_LPM;
		return ret;
}

/**
 *
 */
static int am4001280atzqw00h_wait(struct drm_panel *panel, ktime_t start_ktime, unsigned int min_ms) 
{
	struct panel_driver_data *drv_data = panel_to_drv_data(panel);
	struct mipi_dsi_device *dsi = drv_data->dsi;
	struct device *dev = &dsi->dev;
	int ret;
	ktime_t now_ktime, min_ktime;

	if (!min_ms) {
		DRM_DEV_ERROR(dev, "Got invalid waiting time (%d)\n", ret);
		return 1;
	}

	min_ktime = ktime_add(start_ktime, ms_to_ktime(min_ms));
	now_ktime = ktime_get();
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_ENTER_IDLE_MODE, NULL, 0);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter idle mode while waiting (%d)\n", ret);
		return ret;
	}

	if (ktime_before(now_ktime, min_ktime))
		msleep(ktime_to_ms(ktime_sub(min_ktime, now_ktime)) + 1);

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_EXIT_IDLE_MODE, NULL, 0);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit idle mode while waiting (%d)\n", ret);
		return ret;
	}
	return 0;
}

/**
 *
 */
static int am4001280atzqw00h_get_modes(struct drm_panel *panel)
{
	struct panel_driver_data *drv_data = panel_to_drv_data(panel);
	struct drm_connector *connector = panel->connector;
	struct mipi_dsi_device *dsi = drv_data->dsi;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &am4001280atzqw00h_mode);

	if (!mode) {
		DRM_DEV_ERROR(panel->dev, "Failed to add mode %ux%ux\n", am4001280atzqw00h_mode.hdisplay, am4001280atzqw00h_mode.vdisplay);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = am4001280atzqw00h_data.bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info, am4001280atzqw00h_bus_formats, ARRAY_SIZE(am4001280atzqw00h_bus_formats));

	return 1;
}

/**
 * == Backlight related functions ==
 */

/**
 *
 */
static int am4001280atzqw00h_backlight_update_status(struct backlight_device *bl_dev) 
{
	struct mipi_dsi_device *dsi = bl_get_data(bl_dev);
	struct device *dev = &dsi->dev;
	struct panel_driver_data *drv_data = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if(!drv_data->prepared) {
		dev_warn(dev, "Tried to update backlight status despite not being prepared.");
		return 0;
	}

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl_dev->props.brightness);
	if (ret < 0) {
		dev_err(dev, "Failed to set backlight brightness while updating the backlight.(%d)\n", ret);
		return ret;
	}

	return 0;
}

/**
 *
 */
static int am4001280atzqw00h_get_backlight_brightness(struct backlight_device *bl_dev) 
{
	struct mipi_dsi_device *dsi = bl_get_data(bl_dev);
	struct device *dev = &dsi->dev;
	struct panel_driver_data *drv_data = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	if(!drv_data->prepared) {
		dev_warn(dev, "Tried to get backlight brightness despite not being prepared.");
		return 0;
	}

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);

	if (ret < 0) {
		dev_err(dev, "Failed to get backlight brightness.(%d)\n", ret);
		return ret;
	}
	bl_dev->props.brightness = brightness;

	return brightness & 0xff;
}

/**
 * Instance of backlight_ops.
 *
 */
static const struct backlight_ops am4001280atzqw00h_backlight_ops = {
	.update_status = am4001280atzqw00h_backlight_update_status,
	.get_brightness = am4001280atzqw00h_get_backlight_brightness
};

/**
 * Instance of drm_panel_funcs.
 *
 * Some of the local functions are exposed to the kernel display controller by binding them 
 * to the drm_panel_funcs object so they be called when desired.
 */
static const struct drm_panel_funcs am4001280atzqw00h_funcs = {
	.prepare = am4001280atzqw00h_prepare,
	.enable = am4001280atzqw00h_enable,
	.disable = am4001280atzqw00h_disable,
	.unprepare = am4001280atzqw00h_unprepare,
	.get_modes = am4001280atzqw00h_get_modes
};

/**
 * 
 */
static const struct platform_data am4001280atzqw00h_platform_data = {
	.enable = am4001280atzqw00h_platform_enable
};

/**
 * List all display devices that should be marked as comatible with this driver.
 */
static const struct of_device_id panel_of_match[] = {
	{ 
		.compatible = "ampire,am40001280",
		/* Reference the drm_panel_data object */ 
		.data = &am4001280atzqw00h_platform_data
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, panel_of_match);


/**
 * == MIPI fucntions ==
 */

/**
 * 
 */
static int am4001280atzqw00h_probe(struct mipi_dsi_device *dsi)
{
	struct panel_driver_data *drv_data;
	struct device *dev = &dsi->dev;
	struct device_node *dev_node = dev->of_node;
	const struct of_device_id *of_id = of_match_device(panel_of_match, dev);
	struct backlight_properties bl_props;
	u32 video_mode;

	int ret;
	int i;

	if (!of_id || !of_id->data) {
		return -ENODEV;
	}

	drv_data = devm_kzalloc(dev, sizeof(*drv_data), GFP_KERNEL);

	if (!drv_data) {
		return -ENOMEM;
	}
	
	mipi_dsi_set_drvdata(dsi, drv_data);

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO;

	drv_data->dsi = dsi;
	drv_data->pl_data = of_id->data;

/** Try to set the correct video mode. */
	ret = of_property_read_u32(dev_node, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/** Burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/** Non-burst mode with sync event */
			break;
		case 2:
			/** Non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			/** Throw an error since defaulting non-continuous clock behavior is not possible. */
			DRM_DEV_ERROR(dev, "Got invalid video mode during probe %d\n", video_mode);
			break;
		}
	}
	ret = of_property_read_u32(dev_node, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to get the number of dsi-lanes during probe(%d)\n", ret);
		return ret;
	}

	drv_data->reset_pin = devm_gpiod_get_optional(dev, "reset",
					       			GPIOD_OUT_LOW |
					       			GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if(IS_ERR(drv_data->reset_pin)) {
		ret = PTR_ERR(drv_data->reset_pin);
		DRM_DEV_ERROR(dev, "Failed get reset pin during probe (%d)\n", ret);
		return ret;
	}
	gpiod_set_value_cansleep(drv_data->reset_pin, 1);

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 200;
	bl_props.max_brightness = 255;
	
	drv_data->bl_dev = devm_backlight_device_register(dev, dev_name(dev), dev, dsi, &am4001280atzqw00h_backlight_ops, &bl_props);
	if(IS_ERR(drv_data->bl_dev)) {
		ret = PTR_ERR(drv_data->bl_dev);
		DRM_DEV_ERROR(dev, "Failed register backlight during probe (%d)\n", ret);
		return ret;
	}

	drv_data->num_supplies = ARRAY_SIZE(am4001280atzqw00h_supply_names);
	drv_data->supplies = devm_kcalloc(dev, drv_data->num_supplies, sizeof(*drv_data->supplies), GFP_KERNEL);
	if (!drv_data->supplies) {
		return -ENOMEM;
	}

	for (i = 0; i < drv_data->num_supplies; i++) {
		drv_data->supplies[i].supply = am4001280atzqw00h_supply_names[i];
	}
	ret = devm_regulator_bulk_get(dev, drv_data->num_supplies, drv_data->supplies);

	drm_panel_init(&drv_data->panel);
	drv_data->panel.funcs = &am4001280atzqw00h_funcs;
	drv_data->panel.dev = dev;
	dev_set_drvdata(dev, drv_data);

	ret = drm_panel_add(&drv_data->panel);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to add panel during probe (%d)\n", ret);
		return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to attach panel during probe (%d)\n", ret);
		drm_panel_remove(&drv_data->panel);
		return ret;
	}

	return 0;
}


/**
 * 
 */
static void am4001280atzqw00h_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_driver_data *drv_data = mipi_dsi_get_drvdata(dsi);
	int err;


	err = am4001280atzqw00h_disable(&drv_data->panel);
	if (err < 0) {
		DRM_DEV_ERROR(&dsi->dev, "Failed to disable panel during shutdown (%d)\n", err);
	}
	err = am4001280atzqw00h_unprepare(&drv_data->panel);
	if (err < 0){
		DRM_DEV_ERROR(&dsi->dev, "Failed to unprepare panel during shutdown (%d)\n", err);
	}
}

/**
 * 
 */
static int am4001280atzqw00h_remove(struct mipi_dsi_device *dsi)
{
	struct panel_driver_data *drv_data = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;

	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach panel from DSI host (%d)\n", ret);
	
	drm_panel_remove(&drv_data->panel);

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);

	return 0;
}

/**
 * Power management options
 */
static const struct dev_pm_ops am4001280atzqw00h_pm_ops = {
	SET_RUNTIME_PM_OPS(am4001280atzqw00h_suspend, am4001280atzqw00h_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(am4001280atzqw00h_suspend, am4001280atzqw00h_resume)
};

/**
 * Instance of the mipi_dsi_driver.
 *
 * All local "_disc_" functions are exposed by binding them to this driver so 
 * the kernel can call them when desired.
 */
static struct mipi_dsi_driver am4001280atzqw00h_driver = {
	.driver = {
		.name="panel-ampire-am40001280",
		.of_match_table = panel_of_match,
		.owner = THIS_MODULE,
	},
	.probe = am4001280atzqw00h_probe,
	.shutdown = am4001280atzqw00h_shutdown,
	.remove = am4001280atzqw00h_remove
};

/** Macro to (un-)register this driver instead of implementing "__init" or "__exit". */
module_mipi_dsi_driver(am4001280atzqw00h_driver);

MODULE_AUTHOR("Jan Greiner <jan.greiner@mnet-mail.de>");
MODULE_DESCRIPTION("DRM driver for the Ampire AM-4001280ATZQW-00H MIPI DSI panel");
MODULE_LICENSE("GPL v2");
