/* Copyright (c) 2013, Pantech Co.,Ltd .
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
#include <linux/platform_device.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/regulator/consumer.h>
#include "devices.h"

#include "board-vega.h"

static struct i2c_board_info msm_i2c_audiosubsystem_info[] __initdata ={
#ifdef CONFIG_SND_SOC_YDA165
	{
		I2C_BOARD_INFO("yda165-amp", 0x6C),
	},
#endif
};

static void __init vega_add_i2c_yda165_amp_devices(void)
{
	i2c_register_board_info(APQ_8064_GPIO11_I2C_BUS_ID,
				msm_i2c_audiosubsystem_info, 
				ARRAY_SIZE(msm_i2c_audiosubsystem_info));

	printk(KERN_INFO "%s: register audio subsystem (Yamaha Amp YDA165)\n",__func__);

}


void __init vega_add_sound_devices(void)
{
	vega_add_i2c_yda165_amp_devices();
// 	platform_add_devices(sound_devices, ARRAY_SIZE(sound_devices));
}
