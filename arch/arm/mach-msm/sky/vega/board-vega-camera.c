/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include <linux/i2c.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include <mach/camera.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>

#include "devices.h"
#include "board-vega.h"

#ifdef CONFIG_MSM_CAMERA

static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 1*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 3*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_5, /*active 4*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_6, /*active 5*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 6*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_3, /*active 7*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*i2c suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

	{
		.func = GPIOMUX_FUNC_9, /*active 9*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_A, /*active 10*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_6, /*active 11*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_4, /*active 12*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

};

static struct msm_gpiomux_config apq8064_cam_common_configs[] = {
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[12],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},

};

static struct msm_gpiomux_config apq8064_cam_2d_configs[] = {
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[9],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[10],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
};

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 604143360,
		.ib  = 2416573440UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 548812800,
		.ib  = 1646438400,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 274423680,
		.ib  = 1097694720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 604143360,
		.ib  = 2416573440UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};

static struct msm_bus_vectors cam_video_ls_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 348192000,
		.ib  = 617103360,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
};

static struct msm_bus_vectors cam_dual_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 348192000,
		.ib  = 1208286720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};


static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_video_ls_vectors),
		cam_video_ls_vectors,
	},
	{
		ARRAY_SIZE(cam_dual_vectors),
		cam_dual_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

static struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.csid_core = 0,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 1,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

static struct camera_vreg_t apq_8064_cam_vreg[] = {
#if 0
	{"8921_s8", 	REG_LDO, 1200000, 1200000, 100000,10}, /* CAMV_ISP_CORE_1P2V VDD_CORE_1P2[CORE] */
	{"8921_lvs5", 	REG_VS, 	   0, 		0, 		0, 0}, /* CAMV_IO_1P8V VREG_LVS5_1P8[HOST] */
	{"8921_l15", 	REG_LDO, 2800000, 2850000,  	0, 1}, /* CAMV_SYS_2P8V VREG_L15_2P85[SYS] */
	{"8921_l12", 	REG_LDO, 1200000, 1200000, 105000, 0}, /* CAMV_A_2P8V VREG_L12_1P2 [DVDD] */
	{"8921_l23", 	REG_LDO, 1800000, 1800000,      0, 1}, /* CAMV_SENSOR_IO_1P8V VREG_L23_1P8 [VDDIO] */
	{"8921_l16", 	REG_LDO, 2800000, 2800000,      0, 0}, /* VREG_L16_2P8 [AF] */
	{"8921_l29", 	REG_LDO, 1800000, 1800000,      0, 0}, /* CAMV_RAM_1P8V VDD_RAM_1P8 [RAM] */
#endif
};

static struct gpio apq8064_back_common_cam_gpio[] = {
	{5, GPIOF_DIR_OUT, "CAMIF_MCLK"},
};

#define CAMIO_R_RST_N PM8921_GPIO_PM_TO_SYS(28)
#define CAMIO_R_STB_N PM8921_GPIO_PM_TO_SYS(31)
#define CAM1_RAM_EN PM8921_GPIO_PM_TO_SYS(17)
#define CAM1_INT_N PM8921_GPIO_PM_TO_SYS(16)

static struct gpio apq8064_back_cam_gpio[] = {
	{CAMIO_R_RST_N, GPIOF_DIR_OUT, "CAM_R_RST_N_PM"},
	{CAMIO_R_STB_N, GPIOF_DIR_OUT, "CAM_R_STB_N_PM"},
	{CAM1_RAM_EN, GPIOF_DIR_OUT, "CAM_RAM_N_PM"},
	{CAM1_INT_N, GPIOF_DIR_IN, "CAM_INT_N_PM"},
	{3, GPIOF_DIR_OUT, "FLASH_CNTL_N"},
	{10, GPIOF_DIR_IN, "CAMIF_I2C_DATA"},
	{11, GPIOF_DIR_IN, "CAMIF_I2C_CLK"},
};

static struct msm_camera_gpio_conf apq8064_back_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = apq8064_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(apq8064_cam_2d_configs),
	.cam_gpio_common_tbl = apq8064_back_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(apq8064_back_common_cam_gpio),
	.cam_gpio_req_tbl = apq8064_back_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(apq8064_back_cam_gpio),
};

#define CAMIO_RST_N PM8921_GPIO_PM_TO_SYS(29)
static struct gpio apq8064_front_common_cam_gpio[] = {
	{2, GPIOF_DIR_OUT, "CAMIF_MCLK"},
};
static struct gpio apq8064_front_cam_gpio[] = {
	{CAMIO_RST_N, GPIOF_DIR_OUT, "CAMIO_RST_N_PM"},	
	{13, GPIOF_DIR_OUT, "CAM_STB_N"},
};

static struct msm_camera_gpio_conf apq8064_front_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = apq8064_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(apq8064_cam_2d_configs),
	.cam_gpio_common_tbl = apq8064_front_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(apq8064_front_common_cam_gpio),	
	.cam_gpio_req_tbl = apq8064_front_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(apq8064_front_cam_gpio),
};

static struct msm_camera_i2c_conf apq8064_back_cam_i2c_conf = {
	.use_i2c_mux = 1,
	.mux_dev = &msm8960_device_i2c_mux_gsbi4,
	.i2c_mux_mode = MODE_L,
};


static struct msm_camera_i2c_conf apq8064_front_cam_i2c_conf = {
	.use_i2c_mux = 1,
	.mux_dev = &msm8960_device_i2c_mux_gsbi4,
	.i2c_mux_mode = MODE_L,
};

#ifdef CONFIG_CE1502
static struct msm_camera_sensor_flash_data flash_ce1502 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
};
static struct msm_camera_csi_lane_params ce1502_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_ce1502 = {
	.mount_angle	= 90,
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_back_cam_gpio_conf,
	.i2c_conf = &apq8064_back_cam_i2c_conf,
	.csi_lane_params = &ce1502_csi_lane_params,
};
static struct msm_camera_sensor_info msm_camera_sensor_ce1502_data = {
	.sensor_name	= "ce1502",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_ce1502,
	.sensor_platform_info = &sensor_board_info_ce1502,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};
#endif

#ifdef CONFIG_AS0260
static struct msm_camera_sensor_flash_data flash_as0260 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params as0260_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_as0260 = {
	.mount_angle	= 270, 
	.cam_vreg = apq_8064_cam_vreg,
	.num_vreg = ARRAY_SIZE(apq_8064_cam_vreg),
	.gpio_conf = &apq8064_front_cam_gpio_conf,
	.i2c_conf = &apq8064_front_cam_i2c_conf,
	.csi_lane_params = &as0260_csi_lane_params,
};
static struct msm_camera_sensor_info msm_camera_sensor_as0260_data = {
	.sensor_name	= "as0260",
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_as0260,

	.sensor_platform_info = &sensor_board_info_as0260,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};
#endif

static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

void __init apq8064_init_cam(void)
{
	msm_gpiomux_install(apq8064_cam_common_configs,
			ARRAY_SIZE(apq8064_cam_common_configs));

	platform_device_register(&msm_camera_server);
	platform_device_register(&msm8960_device_i2c_mux_gsbi4);
	platform_device_register(&msm8960_device_csiphy0);
	platform_device_register(&msm8960_device_csiphy1);
	platform_device_register(&msm8960_device_csid0);
	platform_device_register(&msm8960_device_csid1);
	platform_device_register(&msm8960_device_ispif);
	platform_device_register(&msm8960_device_vfe);
	platform_device_register(&msm8960_device_vpe);
}

#ifdef CONFIG_I2C
static struct i2c_board_info apq8064_camera_i2c_boardinfo[] = {
#ifdef CONFIG_CE1502
	{
		I2C_BOARD_INFO("ce1502", 0x78),
		.platform_data = &msm_camera_sensor_ce1502_data,
	},
#endif
#ifdef CONFIG_AS0260
	{
		I2C_BOARD_INFO("as0260", 0x90),
		.platform_data = &msm_camera_sensor_as0260_data,
		.flags = I2C_CLIENT_TEN,
	},
#endif
};

struct msm_camera_board_info apq8064_camera_board_info = {
	.board_info = apq8064_camera_i2c_boardinfo,
	.num_i2c_board_info = ARRAY_SIZE(apq8064_camera_i2c_boardinfo),
};
#endif
#endif
