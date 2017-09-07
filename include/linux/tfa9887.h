/*
 * Definitions for tfa9887 speaker amp chip.
 */
#ifndef TFA9887_H
#define TFA9887_H

#include <linux/ioctl.h>

#define TFA9887_I2C_NAME "tfa9887"
#define TFA9887L_I2C_NAME "tfa9887l"

#define TFA9887_IOCTL_MAGIC 'a'
#define TFA9887_WRITE_CONFIG	_IOW(TFA9887_IOCTL_MAGIC, 0x01, unsigned int)
#define TFA9887_READ_CONFIG	_IOW(TFA9887_IOCTL_MAGIC, 0x02, unsigned int)
#define TFA9887_ENABLE_DSP	_IOW(TFA9887_IOCTL_MAGIC, 0x03, unsigned int)
#define TFA9887_WRITE_L_CONFIG	_IOW(TFA9887_IOCTL_MAGIC, 0x04, unsigned int)
#define TFA9887_READ_L_CONFIG	_IOW(TFA9887_IOCTL_MAGIC, 0x05, unsigned int)
#define TFA9887_KERNEL_LOCK	_IOW(TFA9887_IOCTL_MAGIC, 0x06, unsigned int)
#define TFA9887_KERNEL_INIT	_IOW(TFA9887_IOCTL_MAGIC, 0x07, unsigned int)

#ifdef __KERNEL__
struct tfa9887_platform_data {
	uint32_t gpio_tfa9887_spk_en;
};
#endif

int set_tfa9887_spkamp(int en, int dsp_mode);
int set_tfa9887l_spkamp(int en, int dsp_mode);
int tfa9887_l_write(char *txData, int length);
int tfa9887_l_read(char *rxData, int length);
#endif

