/*
 * Definitions for rt5506 Headphone amp chip.
 */
#ifndef RT5506_H
#define RT5506_H

#include <linux/ioctl.h>
#ifdef __KERNEL__
#include <linux/wakelock.h>
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>
#endif

#define RT5506_I2C_NAME "rt5506"
#define MAX_REG_DATA 15

#define QUERY_IMMED		msecs_to_jiffies(0)
#define QUERY_LATTER		msecs_to_jiffies(200)
#define AMP_SENSE_READY		0x80

enum {
	RT55XX_INIT = 0,
	RT55XX_MUTE,
	RT55XX_MAX_FUNC
};

enum RT55XX_MODE {
	RT55XX_MODE_OFF = RT55XX_MAX_FUNC,
	RT55XX_MODE_PLAYBACK,
	RT55XX_MODE_PLAYBACK8OH,
	RT55XX_MODE_PLAYBACK16OH,
	RT55XX_MODE_PLAYBACK32OH,
	RT55XX_MODE_PLAYBACK64OH,
	RT55XX_MODE_PLAYBACK128OH,
	RT55XX_MODE_PLAYBACK256OH,
	RT55XX_MODE_PLAYBACK500OH,
	RT55XX_MODE_PLAYBACK1KOH,
	RT55XX_MODE_VOICE,
	RT55XX_MODE_TTY,
	RT55XX_MODE_FM,
	RT55XX_MODE_RING,
	RT55XX_MODE_MFG,
	RT55XX_MODE_BEATS_8_64,
	RT55XX_MODE_BEATS_128_500,
	RT55XX_MODE_MONO,
	RT55XX_MODE_MONO_BEATS,
	RT55XX_MAX_MODE
};

enum HEADSET_QUERY_STATUS {
	RT55XX_QUERY_OFF = 0,
	RT55XX_QUERY_HEADSET,
	RT55XX_QUERY_FINISH,
};

enum RT55XX_STATUS {
	RT55XX_OFF = 0,
	RT55XX_PLAYBACK,
	RT55XX_SUSPEND,

};

enum HEADSET_OM {
	HEADSET_8OM = 0,
	HEADSET_16OM,
	HEADSET_32OM,
	HEADSET_64OM,
	HEADSET_128OM,
	HEADSET_256OM,
	HEADSET_500OM,
	HEADSET_1KOM,
	HEADSET_MONO,
	HEADSET_OM_UNDER_DETECT,
};

enum AMP_GPIO_STATUS {
	AMP_GPIO_OFF = 0,
	AMP_GPIO_ON,
	AMP_GPIO_QUERRTY_ON,
};

enum AMP_S4_STATUS {
	AMP_S4_AUTO = 0,
	AMP_S4_PWM,
};

#ifdef __KERNEL__
struct rt55xx_platform_data {
	uint32_t gpio_rt55xx_enable;
	uint32_t gpio_rt55xx_power;
	const char *power_supply;
	struct rpm_regulator *power_reg;
};
#endif

struct rt55xx_reg_data {
	unsigned char addr;
	unsigned char val;
};

struct rt55xx_config {
	unsigned int reg_len;
	struct rt55xx_reg_data reg[MAX_REG_DATA];
};

struct rt55xx_comm_data {
	unsigned int out_mode;
	struct rt55xx_config config;
};

struct rt55xx_config_data {
	unsigned int mode_num;
	struct rt55xx_comm_data cmd_data[RT55XX_MAX_MODE];  /* [mode][mode_kind][reserve][cmds..] */
};

#define RT55XX_IOCTL_MAGIC	'g'
//#define RT55XX_SET_CONFIG	_IOW(RT55XX_IOCTL_MAGIC, 0x01, unsigned)
//#define RT55XX_READ_CONFIG	_IOR(RT55XX_IOCTL_MAGIC, 0x02, unsigned)
#define RT55XX_SET_MODE		_IOW(RT55XX_IOCTL_MAGIC, 0x03, int)
#define RT55XX_SET_PARAM	_IOW(RT55XX_IOCTL_MAGIC, 0x04, struct rt55xx_config_data)
//#define RT55XX_WRITE_REG	_IOW(RT55XX_IOCTL_MAGIC, 0x07, unsigned)
#define RT55XX_QUERY_OM		_IOR(RT55XX_IOCTL_MAGIC, 0x08, int)

#endif
