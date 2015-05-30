#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#
obj-$(CONFIG_SENSORS_NFC_PN544) += pn544.o
obj-$(CONFIG_AMP_TFA9887)       += tfa9887.o
obj-$(CONFIG_AMP_TFA9887L)      += tfa9887l.o
obj-$(CONFIG_AMP_RT5506)	+= rt5506.o

obj-$(CONFIG_INPUT_CAPELLA_CM32181)     += cm32181.o
obj-$(CONFIG_INPUT_CWSTM32)    += CwMcuSensor.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

