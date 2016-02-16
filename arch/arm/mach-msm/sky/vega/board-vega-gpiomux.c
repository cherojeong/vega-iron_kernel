/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-8064.h"


/* Chip selects for SPI clients */
static struct gpiomux_setting gpio_spi_cs_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_UP,
};

/* Chip selects for EPM SPI clients */
static struct gpiomux_setting gpio_epm_spi_cs_config = {
	.func = GPIOMUX_FUNC_6,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_UP,
};


static struct gpiomux_setting gpio_i2c_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_i2c_config_sus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting mbhc_hs_detect = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cdc_mclk = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};


static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

#if defined(CONFIG_SKY_SERIAL_MSM_GSBI5_SUPPORT)
static struct gpiomux_setting gsbi5_func2_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif

static struct gpiomux_setting gsbi3_suspended_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi3_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_USB_EHCI_MSM_HSIC
static struct gpiomux_setting hsic_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hsic_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting hsic_wakeup_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hsic_wakeup_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config apq8064_hsic_configs[] = {
	/* HSIC_STB */
	{
		.gpio = 88,               /*HSIC_STROBE */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
	/* HSIC_DATA */
	{
		.gpio = 89,               /* HSIC_DATA */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
	/* MDM2AP_WAKEUP */
	{
		.gpio = 47,              /* wake up */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_wakeup_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_wakeup_sus_cfg,
		},
	},
};
#endif

static struct msm_gpiomux_config apq8064_gsbi_configs[] __initdata = {
	/* TOUCH_I2C_SDA */
	{
		.gpio      = 8,			/* GSBI3 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi3_active_cfg,
		},
	},
	/* TOUCH_I2C_SCL */
	{
		.gpio      = 9,			/* GSBI3 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi3_active_cfg,
		},
	},
	{
		.gpio      = 30,		/* FP CS */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_cs_config,
		},
	},
	{
		.gpio      = 32,		/* EPM CS */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_epm_spi_cs_config,
		},
	},
#if defined(CONFIG_SKY_SERIAL_MSM_GSBI5_SUPPORT)
	{
		.gpio = 51,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5_func2_cfg,
		}
	},
	{
		.gpio = 52,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5_func2_cfg,
		}
	},
#endif
	/* NFC_I2C_SDA */
	{
		.gpio      = 21,		/* GSBI1 QUP I2C_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config_sus,
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
		},
	},
	/* NFC_I2C_SCL */
	{
		.gpio      = 20,		/* GSBI1 QUP I2C_DATA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config_sus,
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
		},
	},
};

static struct msm_gpiomux_config apq8064_slimbus_config[] __initdata = {
	{
		.gpio   = 40,           /* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio   = 41,           /* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};

static struct msm_gpiomux_config apq8064_audio_codec_configs[] __initdata = {
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mbhc_hs_detect,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_SUSPENDED] = &cdc_mclk,
		},
	},
};

static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_pblrdy = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};


static struct gpiomux_setting ap2mdm_soft_reset_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_wakeup = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config mdm_configs[] __initdata = {
	/* AP2MDM_STATUS */
	{
		.gpio = 48,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_STATUS */
	{
		.gpio = 49,
		.settings = {
			[GPIOMUX_ACTIVE] = &mdm2ap_status_cfg,
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
		}
	},
	/* MDM2AP_ERRFATAL */
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
		}
	},
	/* AP2MDM_ERRFATAL */
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* AP2MDM_SOFT_RESET, aka AP2MDM_PON_RESET_N */
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_soft_reset_cfg,
		}
	},
	/* AP2MDM_WAKEUP */
	{
		.gpio = 35,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_wakeup,
		}
	},
	/* MDM2AP_PBL_READY*/
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_pblrdy,
		}
	},
};

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

static struct gpiomux_setting mdp_vsync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

/* Synaptics rmi4 Touch Device Driver */
static struct gpiomux_setting gpio_rmi4_reset_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting gpio_rmi4_reset_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting gpio_rmi4_attn_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_rmi4_attn_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_vci_en_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting gpio_vci_en_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config apq8064_vega_gpio_configs[] __initdata = {
	/* TE */
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE] = &mdp_vsync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
		},
	},
	/* LCD_VCI_EN2 */
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_vci_en_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_vci_en_suspend_cfg,
		},
	},	
	/* TS_RESET_N */
	{
		.gpio = 43,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_rmi4_reset_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_rmi4_reset_suspend_cfg,
		},
	},
	/* TOUCH_ATTN */
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_rmi4_attn_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_rmi4_attn_suspend_cfg,
		},
	},
};

static struct gpiomux_setting input_pu_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
};

static struct gpiomux_setting output_pu_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting output_pd_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting output_pn_cfg = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config i2c_gpio09_configs[] = {
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pu_cfg,
			[GPIOMUX_SUSPENDED] = &output_pu_cfg,
		},
	},
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pu_cfg,
			[GPIOMUX_SUSPENDED] = &output_pu_cfg,
		},
	},
};

static struct msm_gpiomux_config i2c_gpio10_configs[] = {
	{
		.gpio = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pu_cfg,
			[GPIOMUX_SUSPENDED] = &output_pu_cfg,
		},
	},
	{
		.gpio = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pu_cfg,
			[GPIOMUX_SUSPENDED] = &output_pu_cfg,
		},
	},	
};

static struct msm_gpiomux_config i2c_gpio11_configs[] = {
	{
		.gpio = 70,
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pn_cfg,
			[GPIOMUX_SUSPENDED] = &output_pn_cfg,
		},
	},
	{
		.gpio = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pn_cfg,
			[GPIOMUX_SUSPENDED] = &output_pn_cfg,
		},
	},	
};

#ifdef CONFIG_CHARGER_SMB347_PT

static struct msm_gpiomux_config smb347_gpio_configs[] = {
	{
		.gpio = 7, /* SC_STAT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &input_pu_cfg,
			[GPIOMUX_SUSPENDED] = &input_pu_cfg,
		},
	},
	{
		.gpio = 12, /*SC_EN*/
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pu_cfg,
			[GPIOMUX_SUSPENDED] = &output_pd_cfg,
		},
	},
	{
		.gpio = 69, /* SC_USB5_1_HC 69 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &output_pu_cfg,
			[GPIOMUX_SUSPENDED] = &output_pd_cfg,
		},
	},
	{
		.gpio = 77, /*SC_INOK*/
		.settings = {
			[GPIOMUX_ACTIVE]    = &input_pu_cfg,
			[GPIOMUX_SUSPENDED] = &input_pu_cfg,
		},
	},
};
#endif

void __init apq8064_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return;
	}

	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));

	msm_gpiomux_install(apq8064_gsbi_configs,
			ARRAY_SIZE(apq8064_gsbi_configs));

	msm_gpiomux_install(apq8064_slimbus_config,
			ARRAY_SIZE(apq8064_slimbus_config));

	msm_gpiomux_install(apq8064_audio_codec_configs,
			ARRAY_SIZE(apq8064_audio_codec_configs));

	pr_debug("%s(): audio-auxpcm: Include GPIO configs"
		" as audio is not the primary user"
		" for these GPIO Pins\n", __func__);

	msm_gpiomux_install(mdm_configs,
			ARRAY_SIZE(mdm_configs));

#ifdef CONFIG_USB_EHCI_MSM_HSIC

	msm_gpiomux_install(apq8064_hsic_configs,
			ARRAY_SIZE(apq8064_hsic_configs));
#endif

	msm_gpiomux_install(apq8064_vega_gpio_configs,
			ARRAY_SIZE(apq8064_vega_gpio_configs));

	msm_gpiomux_install(i2c_gpio09_configs,
			 ARRAY_SIZE(i2c_gpio09_configs));

	msm_gpiomux_install(i2c_gpio10_configs,
			 ARRAY_SIZE(i2c_gpio10_configs));

	msm_gpiomux_install(i2c_gpio11_configs,
			 ARRAY_SIZE(i2c_gpio11_configs));

#ifdef CONFIG_CHARGER_SMB347_PT
	msm_gpiomux_install(smb347_gpio_configs,
			 ARRAY_SIZE(smb347_gpio_configs));
#endif

}
