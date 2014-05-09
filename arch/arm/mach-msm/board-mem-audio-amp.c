/* arch/arm/mach-msm/htc_acoustic_alsa.c
 *
 * Copyright (C) 2012 HTC Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <mach/htc_acoustic_alsa.h>

#define GPIO_AUD_RT_1V8_EN 120
static bool power_on = false;

void mem_ul_amp_power_enable(bool enable)
{
	if (enable && !power_on)
	{
		pr_debug("%s: %s\n", __func__, enable?"ture":"false");
		gpio_request(GPIO_AUD_RT_1V8_EN, "AUD_RT_1V8_EN");
		gpio_tlmm_config(GPIO_CFG(GPIO_AUD_RT_1V8_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_free(GPIO_AUD_RT_1V8_EN);
		power_on = true;
	} else if(!enable && power_on){
		pr_debug("%s: %s\n", __func__, enable?"ture":"false");
		gpio_request(GPIO_AUD_RT_1V8_EN, "AUD_RT_1V8_EN");
		gpio_tlmm_config(GPIO_CFG(GPIO_AUD_RT_1V8_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_free(GPIO_AUD_RT_1V8_EN);
		power_on = false;
	} else {
		pr_debug("%s: %s but do nothing\n", __func__, enable?"ture":"false");
	}
}

static struct amp_power_ops amp_power = {
	.set_amp_power_enable = mem_ul_amp_power_enable,
};

static int __init amp_power_init(void)
{
	int ret = 0;
	pr_debug("%s", __func__);
	htc_amp_power_register_ops(&amp_power);
	return ret;
}

static void __exit amp_power_exit(void)
{
}

arch_initcall(amp_power_init);
module_exit(amp_power_exit);
