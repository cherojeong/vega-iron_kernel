/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2013, Pantech Co.,Ltd. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/gpio.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_sony.h"
#include "mdp4.h"


static struct msm_panel_common_pdata *mipi_sony_pdata;

static struct dsi_buf sony_tx_buf;
static struct dsi_buf sony_rx_buf;

static struct mutex sony_mutex;

static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* ? 3 or 2 DTYPE_DCS_WRITE */


#ifdef CONFIG_SONY_CMD_720P_CABC
/* Content Adaptive Backlight Control & Mode Table
   -------------------------
   DATA    |     MODE
   -------------------------
   0x00    | Off mode
   0x01    | UI image
   0x02    | Still Picture
   0x03    | Moving Image
   -------------------------
*/

/* ACX455AKM HD720 LCD tentative LEDPWM sequence */
static char led_pwm1[2] = {0x51, 0x00}; /* Display Brightness(Default Brightness) */
static char led_pwm2[2] = {0x53, 0x2C}; /* PWM output on 0x2C / off 0x00 */
static char led_pwm3[2] = {0x55, 0x03}; /* CABC control */

#endif

static struct dsi_cmd_desc sony_display_on_cmds[] = {


	{DTYPE_DCS_WRITE, 1, 0, 0, 100,
		sizeof(exit_sleep), exit_sleep},

#ifdef CONFIG_SONY_CMD_720P_CABC	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(led_pwm1), led_pwm1}, /* Default Brightness */
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(led_pwm2), led_pwm2}, /* PWM output on */
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(led_pwm3), led_pwm3}, /* CABC Control Moving Image */
#endif

	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on}, /* Display On */

};

static struct dsi_cmd_desc sony_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

#ifdef CONFIG_SONY_CMD_720P_CABC

char mipi_sony_backlight_tbl[17] ={
	0,       35,   40,   55,   70,   85,
	100, 115, 130, 145, 160,	175, 
	190, 205, 220, 235, 255};

static char backlight_pwm[2] = {0x51, 0x00}; /* Display Brightness(Default Brightness) */

static struct dsi_cmd_desc backlight_cmd = {
	DTYPE_DCS_LWRITE, 1, 0, 0, 0, 
		sizeof(backlight_pwm), backlight_pwm
};

#endif

static int lcd_cont_splash_done = 0;

static int mipi_sony_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct msm_panel_info *pinfo;
	struct dcs_cmd_req cmdreq;
	

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	pinfo = &mfd->panel_info;
	mipi  = &mfd->panel_info.mipi;

	mutex_lock(&sony_mutex);	

	if (!lcd_cont_splash_done) {
		if (mipi_sony_pdata 
			&& mipi_sony_pdata->cont_splash_enabled) {
			led_pwm1[1] = 0x40; /* default backlight level 10 */
		}
		lcd_cont_splash_done = 1;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));	

	cmdreq.cmds = sony_display_on_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(sony_display_on_cmds);
	cmdreq.flags = CMD_REQ_COMMIT; 
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq);

	mutex_unlock(&sony_mutex);

	printk(KERN_ERR "%s : success.\n", __func__);

	return 0;
}

static int mipi_sony_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct dcs_cmd_req cmdreq;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mutex_lock(&sony_mutex);	

	memset(&cmdreq, 0, sizeof(cmdreq));	
	cmdreq.cmds = sony_display_off_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(sony_display_off_cmds);
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq);

	if (mipi_sony_pdata
		&& mipi_sony_pdata->cont_splash_enabled) {
		led_pwm1[1] = 0x00;
	}

	mutex_unlock(&sony_mutex);

	pr_debug("%s : success.\n", __func__);

	return 0;
}

static void mipi_sony_set_backlight(struct msm_fb_data_type *mfd)
{
	struct dcs_cmd_req cmdreq;
	static int prev_bl_level = 0;

	if (prev_bl_level == mfd->bl_level)
		return;

	mutex_lock(&sony_mutex);
	
	prev_bl_level = mfd->bl_level;

	memset(&cmdreq, 0, sizeof(cmdreq));	

#ifdef CONFIG_SONY_CMD_720P_CABC
	mipi_set_tx_power_mode(0);

	backlight_pwm[1] = mfd->bl_level;//mipi_sony_backlight_tbl[mfd->bl_level];

	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt =1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	mipi_set_tx_power_mode(1);
#endif
	mutex_unlock(&sony_mutex);

	pr_debug("%s : %d\n",__func__,mfd->bl_level);


}
static int __devinit mipi_sony_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;

	if (pdev->id == 0) {
		mipi_sony_pdata = pdev->dev.platform_data;
		return 0;
	}

	mutex_init(&sony_mutex);

	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;

		mipi  = &mfd->panel_info.mipi;
	}

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_sony_lcd_probe,
	.driver = {
		.name   = "mipi_sony",
	},
};

static struct msm_fb_panel_data sony_panel_data = {
	.on	= mipi_sony_lcd_on,
	.off	= mipi_sony_lcd_off,
	.set_backlight = mipi_sony_set_backlight, 
};

static int ch_used[3];

int mipi_sony_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_sony", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sony_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &sony_panel_data,
			sizeof(sony_panel_data));
	if (ret) {
		printk(KERN_ERR
				"%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
				"%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_sony_lcd_init(void)
{
	mipi_dsi_buf_alloc(&sony_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&sony_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_sony_lcd_init);

