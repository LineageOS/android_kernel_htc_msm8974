#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_PN544)		+= pn544.o
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
