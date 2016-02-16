/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2013, Pantech Co.,Ltd. All right reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_sony.h"

static struct msm_panel_info pinfo;

#ifdef CONFIG_MIPI_CLOCK_450MBPS /* defined in mipi_sony.h */
/* MIPI CLOCK 450MBPS */
static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
	/* 800*1280, RGB888, 4 Lane 60 fps 450Mbps video mode */
	/* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20},
	/* timing */
	{0xab, 0x8a, 0x18, 0x00, 0x92, 0x97, 0x1b, 0x8c,
	0x0c, 0x03, 0x04, 0xa0},
	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},
	/* strength */
	{0xff, 0x00, 0x06, 0x00},
	/* pll control */
	{0x0, 0xc1, 0x31, 0xda, 0x00, 0x50, 0x48, 0x63,
	0x31, 0x0f, 0x03,
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};
#else
/* MIPI CLOCK 500MBPS */
static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
	/* 720*1280, RGB888, 4 Lane 60 fps 500Mbps video mode */
	/* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20},
	/* timing */
	{0xab, 0x8a, 0x18, 0x00, 0x92, 0x97, 0x1b, 0x8c,
	0x0c, 0x03, 0x04, 0xa0},
	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},
	/* strength */
	{0xff, 0x00, 0x06, 0x00},
	/* pll control */
	{0x0, 0xf2, 0x31, 0xda, 0x00, 0x50, 0x48, 0x63,
	0x31, 0x0f, 0x03,
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};
#endif

static int __init mipi_sony_cmd_720p_init(void)
{
	int ret;

	printk(KERN_INFO "%s: panel info intializing!\n",__func__);

	pinfo.xres = 720;
	pinfo.yres = 1280;

	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255; /*16;*/
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

#ifdef CONFIG_MIPI_CLOCK_450MBPS
	pinfo.clk_rate = 450000000;
#else
	pinfo.clk_rate = 500000000;
#endif	
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6700; /* adjust refx100 to prevent tearing */

#if 0
	pinfo.lcdc.h_back_porch = 18;
	pinfo.lcdc.h_front_porch = 156;
	pinfo.lcdc.h_pulse_width = 3;
	pinfo.lcdc.v_back_porch = 3;
	pinfo.lcdc.v_front_porch = 5;
	pinfo.lcdc.v_pulse_width = 2;
#else
	pinfo.lcd.v_back_porch = 1;
	pinfo.lcd.v_front_porch =1;
	pinfo.lcd.v_pulse_width = 2;
#endif
	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;	
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;


#ifdef CONFIG_MIPI_CLOCK_450MBPS
	pinfo.mipi.t_clk_post = 0x21;
	pinfo.mipi.t_clk_pre = 0x2f;
#else
	pinfo.mipi.t_clk_post = 0x21;
	pinfo.mipi.t_clk_pre = 0x30;
#endif
	pinfo.mipi.stream = 0;	
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 62;
	pinfo.mipi.te_sel = 1;
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;

	pinfo.mipi.esc_byte_ratio = 6;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;

	ret = mipi_sony_device_register(&pinfo, MIPI_DSI_PRIM,
				MIPI_DSI_PANEL_720P_PT);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

	return ret;
}
module_init(mipi_sony_cmd_720p_init);

