/*
 * arch/arm/mach-tegra/board-endeavor-panel.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/tegra_pwm_bl.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/hardware.h>
#include <mach/panel_id.h>
#include <mach/board_htc.h>
#include <mach/tegra_fb.h>

#include "board.h"
#include "board-endeavor-panel.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"

#include <linux/disp_debug.h>
#include <linux/debugfs.h>

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE
#define DSI_PANEL_RESET 1

#ifdef CONFIG_TEGRA_DC
static struct regulator *endeavor_dsi_reg = NULL;
static struct regulator *v_lcm_3v3 = NULL;
static struct regulator *v_lcmio_1v8 = NULL;
#endif

#define LCM_TE		TEGRA_GPIO_PJ1
#define LCM_PWM		TEGRA_GPIO_PW1
#define LCM_RST		TEGRA_GPIO_PN6

static struct workqueue_struct *bkl_wq;
static struct work_struct bkl_work;
static struct timer_list bkl_timer;

static int is_power_on = 0;

static struct gpio panel_init_gpios[] = {
    {LCM_TE,	GPIOF_IN,		"lcm_te"},
    {LCM_PWM,	GPIOF_OUT_INIT_LOW,	"pm0"},
    {LCM_RST,	GPIOF_OUT_INIT_HIGH,	"lcm reset"},
};

/* Endeavoru has Sharp, Sony or Auo panels */

#define BACKLIGHT_MAX 255

#define ORIG_PWM_MAX 255
#define ORIG_PWM_DEF 142
#define ORIG_PWM_MIN 30

static unsigned char shrink_pwm(int val)
{
	unsigned char shrink_br;
	int min_pwm = 7;
	int def_pwm = 78;
	int max_pwm = 255;

	/* define line segments for Endeavor */
	if (val <= ORIG_PWM_MIN)
		shrink_br = min_pwm;
	else if (val > ORIG_PWM_MIN && val <= ORIG_PWM_DEF)
		shrink_br = min_pwm +
			(val-ORIG_PWM_MIN)*(def_pwm-min_pwm)/(ORIG_PWM_DEF-ORIG_PWM_MIN);
	else
	shrink_br = def_pwm +
	(val-ORIG_PWM_DEF)*(max_pwm-def_pwm)/(ORIG_PWM_MAX-ORIG_PWM_DEF);

	return shrink_br;
}

static int endeavor_backlight_notify(struct device *unused, int brightness)
{
	if (brightness > 0)
		brightness = shrink_pwm(brightness);

	return brightness;
}

static int endeavor_disp1_check_fb(struct device *dev, struct fb_info *info);
/*
 * In case which_pwm is TEGRA_PWM_PM0,
 * gpio_conf_to_sfio should be TEGRA_GPIO_PW0: set LCD_CS1_N pin to SFIO
 * In case which_pwm is TEGRA_PWM_PM1,
 * gpio_conf_to_sfio should be TEGRA_GPIO_PW1: set LCD_M1 pin to SFIO
 */
static struct platform_tegra_pwm_backlight_data endeavor_disp1_backlight_data = {
	.which_dc		= 0,
	.which_pwm		= TEGRA_PWM_PM1,
	.gpio_conf_to_sfio	= TEGRA_GPIO_PW1,
	.switch_to_sfio		= &tegra_gpio_disable,
	.max_brightness		= 255,
	.dft_brightness		= 142,
	.notify			= endeavor_backlight_notify,
	.period			= 0xFF,
	.clk_div		= 20,
	.clk_select		= 0,
	.backlight_mode		= MIPI_BACKLIGHT,	//Set MIPI_BACKLIGHT as default
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb		= endeavor_disp1_check_fb,
	.backlight_status	= BACKLIGHT_ENABLE,
	.cam_launch_bkl_value	= 181,
};

static struct platform_device endeavor_disp1_backlight_device = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &endeavor_disp1_backlight_data,
	},
};

static struct platform_device *endeavor_bl_devices[]  = {
	&endeavor_disp1_backlight_device,
};

static void bkl_do_work(struct work_struct *work)
{
	struct backlight_device *bl = platform_get_drvdata(&endeavor_disp1_backlight_device);
	if (bl) {
		DISP_DEBUG_LN("set backlight after resume");
		bl->props.bkl_on = 1;
		backlight_update_status(bl);
	}
}

static void bkl_update(unsigned long data) {
	queue_work(bkl_wq, &bkl_work);
}

#ifdef CONFIG_TEGRA_DC
static int endeavor_dsi_panel_enable(void)
{
	DISP_INFO_IN();

	if (is_power_on)
		return 0;

	/* delay for DSI hardware stable*/
	hr_msleep(10);

	/* change LCM_TE & LCM_PWM to SFIO */
	tegra_gpio_disable(LCM_TE);

	REGULATOR_GET(endeavor_dsi_reg, "avdd_dsi_csi");
	regulator_enable(endeavor_dsi_reg);

	/* Start panel init */	
	REGULATOR_GET(v_lcm_3v3, "v_lcm_3v3");
	REGULATOR_GET(v_lcmio_1v8, "v_lcmio_1v8");

	/* LCD_RST pull low */
	gpio_set_value(LCM_RST, 0);
	hr_msleep(5);

	/*Turn on LCMIO_1V8_EN*/
	regulator_enable(v_lcmio_1v8);
	hr_msleep(1);

	/*Turn on LCMIO_3V3_EN*/
	regulator_enable(v_lcm_3v3);
	hr_msleep(20);

	/* panel reset */
	gpio_set_value(LCM_RST, 1);
	hr_msleep(2);

	gpio_set_value(LCM_RST, 0);
	hr_msleep(1);

	gpio_set_value(LCM_RST, 1);
	hr_msleep(25);

	is_power_on = 1;

	DISP_INFO_OUT();
failed:
	return 0;
}

static int endeavor_dsi_panel_disable(void)
{
	if (!is_power_on)
		return 0;

	DISP_INFO_IN();

	gpio_set_value(LCM_RST, 0);
	hr_msleep(12);

	REGULATOR_GET(v_lcm_3v3, "v_lcm_3v3");
	regulator_disable(v_lcm_3v3);
	hr_msleep(5);

	REGULATOR_GET(v_lcmio_1v8, "v_lcmio_1v8");
	regulator_disable(v_lcmio_1v8);

	REGULATOR_GET(endeavor_dsi_reg, "avdd_dsi_csi");
	regulator_disable(endeavor_dsi_reg);

	/* change LCM_TE to GPIO */
	tegra_gpio_enable(LCM_TE);

	is_power_on = 0;

	DISP_INFO_OUT();
failed:
	return 0;
}

struct tegra_dsi_out endeavor_dsi = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,

#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	/*
	 * The one-shot frame time must be shorter than the time between TE.
	 * Increasing refresh_rate will result in a decrease in the frame time
	 * for one-shot. rated_refresh_rate is only an approximation of the
	 * TE rate, and is only used to report refresh rate to upper layers.
	 */
	.refresh_rate = 66,
	.rated_refresh_rate = 60,
#else
	.refresh_rate = 60,
#endif

	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_has_frame_buffer = true,
	.dsi_instance = 0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,

	/* PANEL_ID_SHARP_NT_C2_9A */
	.dsi_init_cmd = dsi_init_sharp_nt_c2_9a_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_init_sharp_nt_c2_9a_cmd),

	.osc_off_cmd = osc_off_cmd,
	.n_osc_off_cmd = ARRAY_SIZE(osc_off_cmd),

	.osc_on_cmd = osc_on_cmd,
	.n_osc_on_cmd = ARRAY_SIZE(osc_on_cmd),

	.dsi_cabc_moving_mode = nt_moving_mode_cmd,
	.n_cabc_cmd = ARRAY_SIZE(nt_moving_mode_cmd),

	.dsi_cabc_still_mode = nt_still_mode_cmd,

	.dsi_cabc_dimming_on_cmd = dimming_on_cmd,
	.n_cabc_dimming_on_cmd = ARRAY_SIZE(dimming_on_cmd),
	/* PANEL_ID_SHARP_NT_C2_9A - END*/

	.dsi_suspend_cmd = dsi_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_suspend_cmd),

	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,

	.lp_cmd_mode_freq_khz = 20000,

	/* base on kernel 2.6 setting */
	.phy_timing.t_hsdexit_ns = 108,
	.phy_timing.t_hstrail_ns = 60,
	.phy_timing.t_datzero_ns = 172,
	.phy_timing.t_hsprepare_ns = 68,
	.phy_timing.t_clktrail_ns = 68,
	.phy_timing.t_clkpost_ns = 124,
	.phy_timing.t_clkzero_ns = 220,
	.phy_timing.t_clkprepare_ns = 36,
	.phy_timing.t_taget_ns = 5700,
	.phy_timing.t_tasure_ns = 2900,
	.phy_timing.t_tago_ns = 7100,
	.phy_timing.t_wakeup_ns = 25700,
};

static struct tegra_dc_mode endeavor_dsi_modes[] = {
	{
		.pclk = 20000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 1,
		.h_back_porch = 29,
		.v_back_porch = 1,
		.h_active = 720,
		.v_active = 1280,
		.h_front_porch = 55,
		.v_front_porch = 2,
	},
};

static struct tegra_dc_out endeavor_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.parent_clk	= "pll_d_out0",
	.depth		= 24,

	.flags		= DC_CTRL_MODE,
	.type		= TEGRA_DC_OUT_DSI,

	.modes		= endeavor_dsi_modes,
	.n_modes	= ARRAY_SIZE(endeavor_dsi_modes),

	.dsi		= &endeavor_dsi,

	.enable		= endeavor_dsi_panel_enable,
	.disable	= endeavor_dsi_panel_disable,

	.width		= 53,
	.height		= 95,

	.video_min_bw	= 51000000,
};

static struct tegra_fb_data endeavor_dsi_fb_data = {
	.win		= 0,
	.xres		= 720,
	.yres		= 1280,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data endeavor_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &endeavor_disp1_out,
	.emc_clk_rate	= 204000000,
	.fb		= &endeavor_dsi_fb_data,
};

static struct resource endeavor_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by endeavor_panel_init() */
		.end	= 0,	/* Filled in by endeavor_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct nvhost_device endeavor_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= endeavor_disp1_resources,
	.num_resources	= ARRAY_SIZE(endeavor_disp1_resources),
	.dev = {
		.platform_data = &endeavor_disp1_pdata,
	},
};

static int endeavor_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &endeavor_disp1_device.dev;
}
#endif

#ifdef CONFIG_TEGRA_NVMAP
static struct nvmap_platform_carveout endeavor_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by endeavor_panel_init() */
		.size		= 0,	/* Filled in by endeavor_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data endeavor_nvmap_data = {
	.carveouts	= endeavor_carveouts,
	.nr_carveouts	= ARRAY_SIZE(endeavor_carveouts),
};

static struct platform_device endeavor_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &endeavor_nvmap_data,
	},
};
#endif

static struct platform_device *endeavor_gfx_devices[] __initdata = {
#ifdef CONFIG_TEGRA_NVMAP
	&endeavor_nvmap_device,
#endif
	&tegra_pwfm0_device,
};

int __init endeavor_panel_init(void)
{
	int err;
	int i = 0;
	int pin_count = ARRAY_SIZE(panel_init_gpios);
	struct resource __maybe_unused *res;

	DISP_INFO_IN();

	err = gpio_request_array(panel_init_gpios, ARRAY_SIZE(panel_init_gpios));
	if (err) {
		DISP_ERR("gpio request failed\n");
		goto failed;
	}

	for (i = 0; i < pin_count; i++) {
		tegra_gpio_enable(panel_init_gpios[i].gpio);
	}

#ifdef CONFIG_TEGRA_NVMAP
	endeavor_carveouts[1].base = tegra_carveout_start;
	endeavor_carveouts[1].size = tegra_carveout_size;
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(endeavor_gfx_devices,
				ARRAY_SIZE(endeavor_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&endeavor_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	if (res) {
		res->start = tegra_fb_start;
		res->end = tegra_fb_start + tegra_fb_size - 1;
	}
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&endeavor_disp1_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif

	if (!err)
		err = platform_add_devices(endeavor_bl_devices,
				ARRAY_SIZE(endeavor_bl_devices));

	INIT_WORK(&bkl_work, bkl_do_work);
	bkl_wq = create_workqueue("bkl_wq");
	setup_timer(&bkl_timer, bkl_update, 0);

failed:
	DISP_INFO_OUT();

	return err;
}
