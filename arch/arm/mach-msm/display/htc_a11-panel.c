#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <asm/mach-types.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/debug_display.h>
#include "../../../../drivers/video/msm/mdss/mdss_dsi.h"

#define PANEL_ID_KIWI_SHARP_HX      0
#define PANEL_ID_A5_JDI_NT35521_C3      1

/* HTC: dsi_power_data overwrite the role of dsi_drv_cm_data
   in mdss_dsi_ctrl_pdata structure */
struct dsi_power_data {
	uint32_t sysrev;         /* system revision info */
	struct regulator *vddio; /* 1.8v */
	struct regulator *vdda;  /* 1.2v */
	struct regulator *vddpll; /* mipi 1.8v */
	struct regulator *vlcm3v; /* 3v */

	int lcm_bl_en;
};

#ifdef MODULE
extern struct module __this_module;
#define THIS_MODULE (&__this_module)
#else
#define THIS_MODULE ((struct module *)0)
#endif

static struct i2c_adapter	*i2c_bus_adapter = NULL;

struct i2c_dev_info {
	uint8_t				dev_addr;
	struct i2c_client	*client;
};

#define I2C_DEV_INFO(addr) \
	{.dev_addr = addr >> 1, .client = NULL}

static struct i2c_dev_info device_addresses[] = {
	I2C_DEV_INFO(0x6C)
};

static inline int platform_write_i2c_block(struct i2c_adapter *i2c_bus
								, u8 page
								, u8 offset
								, u16 count
								, u8 *values
								)
{
	struct i2c_msg msg;
	u8 *buffer;
	int ret;
	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		printk("%s:%d buffer allocation failed\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	msg.flags = 0;
	msg.addr = page >> 1;
	msg.buf = buffer;
	msg.len = count + 1;

	ret = i2c_transfer(i2c_bus, &msg, 1);

	kfree(buffer);

	if (ret != 1) {
		printk("%s:%d I2c write failed 0x%02x:0x%02x\n"
				,__FUNCTION__,__LINE__, page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int tps_65132_add_i2c(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int idx;

	/* "Hotplug" the MHL transmitter device onto the 2nd I2C bus  for BB-xM or 4th for pandaboard*/
	i2c_bus_adapter = adapter;
	if (i2c_bus_adapter == NULL) {
		PR_DISP_ERR("%s() failed to get i2c adapter\n", __func__);
		return ENODEV;
	}

	for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
		if(idx == 0)
			device_addresses[idx].client = client;
		else {
			device_addresses[idx].client = i2c_new_dummy(i2c_bus_adapter,
											device_addresses[idx].dev_addr);
			if (device_addresses[idx].client == NULL){
				return ENODEV;
			}
		}
	}

	return 0;
}


static int __devinit tps_65132_tx_i2c_probe(struct i2c_client *client,
					      const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		PR_DISP_ERR("%s: Failed to i2c_check_functionality \n", __func__);
		return -EIO;
	}


	if (!client->dev.of_node) {
		PR_DISP_ERR("%s: client->dev.of_node = NULL\n", __func__);
		return -ENOMEM;
	}

	ret = tps_65132_add_i2c(client);

	if(ret < 0) {
		PR_DISP_ERR("%s: Failed to tps_65132_add_i2c, ret=%d\n", __func__,ret);
		return ret;
	}

	return 0;
}


static const struct i2c_device_id tps_65132_tx_id[] = {
	{"tps65132", 0}
};

static struct of_device_id TSP_match_table[] = {
	{.compatible = "disp-tps-65132",}
};

static struct i2c_driver tps_65132_tx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tps65132",
		.of_match_table = TSP_match_table,
		},
	.id_table = tps_65132_tx_id,
	.probe = tps_65132_tx_i2c_probe,
	.command = NULL,
};

static int htc_a11_regulator_init(struct platform_device *pdev)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dsi_power_data *pwrdata = NULL;

	PR_DISP_INFO("%s\n", __func__);
	if (!pdev) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = platform_get_drvdata(pdev);
	if (!ctrl_pdata) {
		pr_err("%s: invalid driver data\n", __func__);
		return -EINVAL;
	}

	pwrdata = devm_kzalloc(&pdev->dev,
				sizeof(struct dsi_power_data), GFP_KERNEL);
	if (!pwrdata) {
		pr_err("[DISP] %s: FAILED to alloc pwrdata\n", __func__);
		return -ENOMEM;
	}

	ctrl_pdata->dsi_pwrctrl_data = pwrdata;

	pwrdata->vddio = devm_regulator_get(&pdev->dev, "vddio");
	if (IS_ERR(pwrdata->vddio)) {
		pr_err("%s: could not get vddio reg, rc=%ld\n",
			__func__, PTR_ERR(pwrdata->vddio));
		return PTR_ERR(pwrdata->vddio);
	}

	pwrdata->vlcm3v = devm_regulator_get(&pdev->dev, "vdd");//pm8226_l23
	if (IS_ERR(pwrdata->vlcm3v)) {
		pr_err("%s: could not get vdd reg, rc=%ld\n",
			__func__, PTR_ERR(pwrdata->vlcm3v));
		return PTR_ERR(pwrdata->vlcm3v);
	}

	pwrdata->vddpll = devm_regulator_get(&pdev->dev, "vddpll");
	if (IS_ERR(pwrdata->vddpll)) {
		pr_err("%s: could not get vddpll reg, rc=%ld\n",
			__func__, PTR_ERR(pwrdata->vddpll));
		return PTR_ERR(pwrdata->vddpll);
	}

	/*
		ret = regulator_set_voltage(pwrdata->vddio, 1800000,
				1800000);
		if (ret) {
			pr_err("%s: set voltage failed on vddio vreg, rc=%d\n",
				__func__, ret);
			return ret;
		}
	*/
	pwrdata->vdda = devm_regulator_get(&pdev->dev, "vdda");
	if (IS_ERR(pwrdata->vdda)) {
		pr_err("%s: could not get vdda vreg, rc=%ld\n",
			__func__, PTR_ERR(pwrdata->vdda));
		return PTR_ERR(pwrdata->vdda);
	}

	ret = regulator_set_voltage(pwrdata->vdda, 1200000,
		1200000);
	if (ret) {
		pr_err("%s: set voltage failed on vdda vreg, rc=%d\n",
			__func__, ret);
		return ret;
	}

	ret = regulator_set_voltage(pwrdata->vlcm3v, 3000000,
			3000000);
	if (ret) {
		pr_err("%s: set voltage failed on vlcm3v vreg, rc=%d\n",
			__func__, ret);
		return ret;
	}

	pwrdata->lcm_bl_en = of_get_named_gpio(pdev->dev.of_node,
						"htc,lcm_bl_en-gpio", 0);

	ret = i2c_add_driver(&tps_65132_tx_i2c_driver);
	if (ret < 0) {
		pr_err("[DISP] %s: FAILED to add i2c_add_driver ret=%x\n",
			__func__, ret);
	}
	return 0;
}


static int htc_a11_regulator_deinit(struct platform_device *pdev)
{
	/* devm_regulator() will automatically free regulators
	   while dev detach. */
	/* nothing */
	return 0;
}

int htc_a11_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return -EINVAL;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (enable) {
		if (pdata->panel_info.first_power_on == 1) {
			PR_DISP_INFO("reset already on in first time\n");
			return 0;
		}

		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		usleep_range(2000,2500);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		usleep_range(100,150);
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		msleep(25);
	} else {
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}

	return 0;
}

static void htc_a11_bkl_en(struct mdss_panel_data *pdata, int enable)
{
	static int en = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dsi_power_data *pwrdata = NULL;
	u8 data = 0x00;
	int ret =0;

	if(en == enable)
		return;
	en = enable;

	PR_DISP_INFO("%s:en=%d\n", __func__, enable);

	if (pdata == NULL) {
	        pr_err("%s: Invalid input data\n", __func__);
	        return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
	                        panel_data);
	pwrdata = ctrl_pdata->dsi_pwrctrl_data;

	if(enable) {
		gpio_set_value(pwrdata->lcm_bl_en, 1);
		usleep_range(2000,2500);

		/* Output Configuration */
		data = 0x02;
		ret = platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x50, 1, &data);
		if(ret)
			platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x50, 1, &data);

		usleep_range(500,1000);

		/* Control B Full-Scale Current */
		data=0x09;
		ret = platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x01, 1, &data);
		if(ret)
			platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x01, 1, &data);

		usleep_range(500,1000);

		/* Control B PWM */
		data=0x70;
		ret = platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x02, 1, &data);
		if(ret)
			platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x02, 1, &data);

		usleep_range(500,1000);

		/* Control B Zone Target 4 */
		data=0x14;
		ret = platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x05, 1, &data);
		if(ret)
			platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x05, 1, &data);

		usleep_range(500,1000);

		/* Control Enable */
		data=0x04;
		ret = platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x00, 1, &data);
		if(ret)
			platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x00, 1, &data);

		usleep_range(6000,8000);

		/* Feedback Enable */
		data=0xFF;
		ret = platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x03, 1, &data);
		if(ret)
			platform_write_i2c_block(i2c_bus_adapter, 0x6C, 0x03, 1, &data);
		usleep_range(500,1000);

	} else {
		gpio_set_value(pwrdata->lcm_bl_en, 0);
	}

}

static int htc_a11_panel_power_on(struct mdss_panel_data *pdata, int enable)
{
	int ret;

	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dsi_power_data *pwrdata = NULL;

	PR_DISP_INFO("%s: en=%d\n", __func__, enable);
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	pwrdata = ctrl_pdata->dsi_pwrctrl_data;

	if (!pwrdata) {
		pr_err("%s: pwrdata not initialized\n", __func__);
		return -EINVAL;
	}

	if (enable) {
		ret = regulator_set_optimum_mode(pwrdata->vddio, 100000);
		if (ret < 0) {
			pr_err("%s: vddio set opt mode failed.\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(pwrdata->vddpll, 100000);
		if (ret < 0) {
			pr_err("%s: vddpll set opt mode failed.\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(pwrdata->vdda, 100000);
		if (ret < 0) {
			pr_err("%s: vdda set opt mode failed.\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(pwrdata->vlcm3v, 100000);
		if (ret < 0) {
			pr_err("%s: vlcm3v set opt mode failed.\n",
				__func__);
			return ret;
		}
		usleep_range(12000,12500);

		/*enable 1v8*/
		ret = regulator_enable(pwrdata->vddio);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n",
				__func__);
			return ret;
		}
		usleep_range(1000,1500);

		ret = regulator_enable(pwrdata->vlcm3v);
		if (ret) {
			pr_err("%s: Failed to enable regulator : vlcm3v.\n",
				__func__);
			return ret;
		}
		usleep_range(1000,1500);

		ret = regulator_enable(pwrdata->vddpll);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n",
				__func__);
			return ret;
		}
		usleep_range(1000,1500);

		/*ENABLE 1V2*/
		ret = regulator_enable(pwrdata->vdda);
		if (ret) {
			pr_err("%s: Failed to enable regulator : vdda.\n",
				__func__);
			return ret;
		}

		usleep_range(15000,20000);
	} else {

		usleep_range(2000,2500);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);

		/*disable 1v2*/
		ret = regulator_disable(pwrdata->vdda);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n",
				__func__);
			return ret;
		}

		ret = regulator_disable(pwrdata->vlcm3v);
		if (ret) {
			pr_err("%s: Failed to disable regulator : vlcm3v.\n",
				__func__);
			return ret;
		}

		ret = regulator_disable(pwrdata->vddpll);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n",
				__func__);
			return ret;
		}

		/*disable 1v8*/
		ret = regulator_disable(pwrdata->vddio);
		if (ret) {
			pr_err("%s: Failed to disable regulator : vddio.\n",
				__func__);
			return ret;
		}

		usleep_range(2000,2500);

		ret = regulator_set_optimum_mode(pwrdata->vddio, 100);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set opt mode failed.\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(pwrdata->vlcm3v, 100);
		if (ret < 0) {
			pr_err("%s: vlcm3v_vreg set opt mode failed.\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(pwrdata->vddpll, 100);
		if (ret < 0) {
			pr_err("%s: vddpll_vreg set opt mode failed.\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(pwrdata->vdda, 100);
		if (ret < 0) {
			pr_err("%s: vdda_vreg set opt mode failed.\n",
				__func__);
			return ret;
		}
	}
	PR_DISP_INFO("%s: en=%d done\n", __func__, enable);

	return 0;
}


static struct mdss_dsi_pwrctrl dsi_pwrctrl = {
	.dsi_regulator_init = htc_a11_regulator_init,
	.dsi_regulator_deinit = htc_a11_regulator_deinit,
	.dsi_power_on = htc_a11_panel_power_on,
	.dsi_panel_reset = htc_a11_panel_reset,
	.bkl_config = htc_a11_bkl_en,
};

static struct platform_device dsi_pwrctrl_device = {
	.name          = "mdss_dsi_pwrctrl",
	.id            = -1,
	.dev.platform_data = &dsi_pwrctrl,
};

int __init htc_8226_dsi_panel_power_register(void)
{
	PR_DISP_INFO("%s#%d\n", __func__, __LINE__);
	platform_device_register(&dsi_pwrctrl_device);
	return 0;
}
