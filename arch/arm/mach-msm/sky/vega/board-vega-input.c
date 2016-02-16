#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <mach/vreg.h>
#include <mach/rpc_server_handset.h>
#include <mach/board.h>

/* keypad */
#include <linux/mfd/pm8xxx/pm8921.h>

/* i2c */
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>

#include <linux/earlysuspend.h>
#include <linux/rmi.h>
#include "devices.h"
#include "board-vega.h"

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_PANTECH

#define THINSTYK_ADDR	0x20
#define THINSTYK_ATTN	55
#define THINSTYK_RESET	43

#define TOUCH_NAME "synaptics_rmi4"

struct syna_gpio_data {
	u16 gpio_number;
	char* gpio_name;
};

static struct syna_gpio_data thinstyk_gpiodata = {
		.gpio_number = THINSTYK_ATTN,	
		.gpio_name = "sdmmc2_clk.gpio_130",
};

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval=0;
	struct syna_gpio_data *data = gpio_data;

	if (configure) {
		retval = gpio_request(data->gpio_number, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		retval = gpio_direction_input(data->gpio_number);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, data->gpio_number);
	}

	return retval;
}



static int synaptics_power_on(int on)
{
	int rc = -EINVAL;
	static struct regulator *vreg_l11 = NULL;
	static struct regulator *vreg_l21 = NULL;
	static int configure = 0;

	/* 3.3V_TOUCH_VDD, VREG_L15: 2.75 ~ 3.3 */
	if (!vreg_l11) {
		vreg_l11 = regulator_get(&msm_mipi_dsi1_device.dev, "dsi1_avdd");
		if (IS_ERR(vreg_l11)) {
			pr_err("%s: regulator get of 8921_l11 failed (%ld)\n",
					__func__,
			       PTR_ERR(vreg_l11));
			rc = PTR_ERR(vreg_l11);
			vreg_l11 = NULL;
			return rc;
		}
	}
	/* 1.8V_TOUCH_IO, VREG_L22: 1.7 ~ 2.85 */
	if (!vreg_l21) {
		vreg_l21 = regulator_get(&msm_mipi_dsi1_device.dev, "dsi_vddi");
		if (IS_ERR(vreg_l21)) {
			pr_err("%s: regulator get of 8921_l21 failed (%ld)\n",
					__func__,
			       PTR_ERR(vreg_l21));
			rc = PTR_ERR(vreg_l21);
			vreg_l21 = NULL;
			return rc;
		}
	}

	rc = regulator_set_voltage(vreg_l11, 3300000, 3300000);
	rc |= regulator_set_voltage(vreg_l21, 1800000, 1800000);
	if (rc < 0) {
		pr_err("%s: cannot control regulator\n",
		       __func__);
		return rc;
	}

	rc = regulator_set_optimum_mode(vreg_l11, 100000);
	if (rc < 0) {
		pr_err("set_optimum_mode l11 failed, rc=%d\n", rc);
			return -EINVAL;
	}

	rc = regulator_set_optimum_mode(vreg_l21, 110000); 
	if (rc < 0) {
		pr_err("set_optimum_mode ext_3p3v failed, rc=%d\n", rc);
		return -EINVAL;
	}

	if (on) {
		regulator_enable(vreg_l11);
		msleep(10); /* tB Wait + tC VDDI_18V rise */
		regulator_enable(vreg_l21);

		if (!configure) {

			rc = gpio_request(THINSTYK_RESET, "rmi4_reset");
			if (rc) {
				pr_err("%s: Failed to get reset gpio %d. Code: %d.",
				__func__, THINSTYK_RESET, rc);
				return rc;
			}
			gpio_direction_output(THINSTYK_RESET, 1);
			configure =1;
		}

		gpio_set_value(THINSTYK_RESET,0);
		msleep(10);
		gpio_set_value(THINSTYK_RESET,1);
		msleep(10);
	} else {

		gpio_set_value(THINSTYK_RESET,0);

		regulator_disable(vreg_l21);
		msleep(1); /* tV + tW */
		regulator_disable(vreg_l11);
	}

	return rc;
}

static int synaptics_post_suspend(const void *pm_data)
{
	int rc = -EINVAL;
	rc = synaptics_power_on(0);
	return rc;
}

static int synaptics_pre_resume(const void *pm_data)
{
	int rc = -EINVAL;
	rc = synaptics_power_on(1);
	return rc;
}

static struct rmi_device_platform_data thinstyk_platformdata = {
	.driver_name = "rmi_generic",
	.attn_gpio = THINSTYK_ATTN,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data = &thinstyk_gpiodata,
	.gpio_config = synaptics_touchpad_gpio_setup,
	.sensor_name = "touch",
#ifdef CONFIG_RMI4_F11_TYPEB	
	.f11_type_b = true,
#endif
	.post_suspend = synaptics_post_suspend,
	.pre_resume = synaptics_pre_resume,
	.power_ctrl = synaptics_power_on,
	.reset_gpio = THINSTYK_RESET,

};

static struct i2c_board_info synaptics_i2c_info[] = {
	[0] = {
		I2C_BOARD_INFO(TOUCH_NAME, THINSTYK_ADDR),
		.platform_data = &thinstyk_platformdata,
	},
};
#endif

void __init apq8064_init_input(void)
{
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_PANTECH
	i2c_register_board_info(APQ_8064_GSBI3_QUP_I2C_BUS_ID,
				&synaptics_i2c_info[0], 1);
#endif

	printk(KERN_INFO "%s: input device registered.\n",__func__);

}


