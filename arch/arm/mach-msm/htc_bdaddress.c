/* arch/arm/mach-msm/htc_bluetooth.c
 *
 * Code to extract Bluetooth bd_address information
 * from ATAG set up by the bootloader.
 *
 * Copyright (C) 2010 HTC Corporation
 * Author:Yomin Lin <yomin_lin@htc.com>
 * Author:Allen Ou <allen_ou@htc.com>
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include <mach/htc_bdaddress.h>
#include <linux/of.h>

#define ATAG_BT_DEBUG

/* configuration tags specific to Bluetooth*/
//#define ATAG_BLUETOOTH 0x43294329
#define MAX_BT_SIZE 0x8U

#define CALIBRATION_DATA_PATH "/calibration_data"
#define BT_FLASH_DATA "bt_flash"

static unsigned char bt_bd_ram[MAX_BT_SIZE];
static char bdaddress[20];

static unsigned char *get_bt_bd_ram(void)
{
	struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	int p_size = 0;
	unsigned char *p_data = NULL;
#ifdef ATAG_BT_DEBUG
	unsigned int i;
#endif

	if (offset) {
		p_data = (unsigned char*) of_get_property(offset, BT_FLASH_DATA, &p_size);
#ifdef ATAG_BT_DEBUG
		if (p_data) {
			printk(KERN_INFO "BT addr:");
			for (i = 0; i < p_size; ++i)
				printk(KERN_CONT "%02x ", p_data[i]);
		}
#endif
	}
	if (p_data != NULL)
		memcpy(bt_bd_ram, p_data, p_size);

	return (bt_bd_ram);
}

void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);

	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
			cTemp[0], cTemp[1], cTemp[2],
			cTemp[3], cTemp[4], cTemp[5]);

	printk(KERN_INFO "fd=%02x, apply=%02x\n", cTemp[2]+1, cTemp[5]+2);
	printk(KERN_INFO "fd=%02x, state=%02x\n", cTemp[4]+2, cTemp[1]+1);
	printk(KERN_INFO "fd=%02x, status=%02x\n", cTemp[0]+1, cTemp[3]+2);
}
module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

