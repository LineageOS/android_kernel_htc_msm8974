/*
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>

#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define PROCNAME "driver/hdf"
#define FLAG_LEN 64
#define HTC_DEBUG_HDF_DEV_NAME "htc,htc_debug_hdf"

static struct of_device_id htc_debug_hdf_match_table[] = {
    { .compatible = HTC_DEBUG_HDF_DEV_NAME },
    {}
};

static char htc_debug_flag[FLAG_LEN+1];
extern int get_partition_num_by_name(char *name);
static int offset=628;
static int first_read=1;
mm_segment_t oldfs;

#if 0
#define SECMSG(s...) pr_info("[SECURITY] "s)

#else
#define SECMSG(s...) do{} while(0)
#endif

#define TAG "[SEC] "
#undef PDEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO TAG "[K] %s(%i, %s): " fmt "\n", \
        __func__, current->pid, current->comm, ## args)

#undef PERR
#define PERR(fmt, args...) printk(KERN_ERR TAG "[E] %s(%i, %s): " fmt "\n", \
        __func__, current->pid, current->comm, ## args)

#undef PINFO
#define PINFO(fmt, args...) printk(KERN_INFO TAG "[I] %s(%i, %s): " fmt "\n", \
        __func__, current->pid, current->comm, ## args)

static ssize_t kernel_write(struct file *file, const char *buf, size_t count, loff_t pos)
{
	mm_segment_t old_fs;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	res = vfs_write(file, (const char __user *)buf, count, &pos);
	set_fs(old_fs);
	return res;
}

int htc_debug_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len=FLAG_LEN+3;
    char R_Buffer[FLAG_LEN+3];
    char RfMisc[FLAG_LEN+3];
    char filename[32] = "";
    struct file *filp = NULL;
    ssize_t nread;
    int pnum;

    if (off > 0) {
        len = 0;
    } else {
        if(first_read){
            pnum = get_partition_num_by_name("misc");

            if (pnum < 0) {
                printk(KERN_ERR"unknown partition number for misc partition\n");
                return 0;
            }

            snprintf(filename, 32, "/dev/block/mmcblk0p%d", pnum);

            filp = filp_open(filename, O_RDWR, 0);
            if (IS_ERR(filp)) {
                printk(KERN_ERR"unable to open file: %s\n", filename);
                return PTR_ERR(filp);
            }

            SECMSG("%s: offset :%d\n", __func__, offset);
            filp->f_pos = offset;

            memset(RfMisc,0,FLAG_LEN+3);
            nread = kernel_read(filp, filp->f_pos, RfMisc, FLAG_LEN+2);

            memset(htc_debug_flag,0,FLAG_LEN+1);
            memcpy(htc_debug_flag,RfMisc+2,FLAG_LEN);

            SECMSG("%s: RfMisc        :%s (%zd)\n", __func__,RfMisc, nread);
            SECMSG("%s: htc_debug_flag:%s \n", __func__, htc_debug_flag);

            if (filp)
                filp_close(filp, NULL);

            first_read = 0;
        }

        memset(R_Buffer,0,FLAG_LEN+3);
        memcpy(R_Buffer,"0x",2);
        memcpy(R_Buffer+2,htc_debug_flag,FLAG_LEN);
        memcpy(page,R_Buffer,FLAG_LEN+3);
    }

    return len;
}

int htc_debug_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char buf[FLAG_LEN+3]={0};
    char filename[32] = "";
    struct file *filp = NULL;
    ssize_t nread;
    int pnum;

    SECMSG("%s called (count:%d)\n", __func__, (int)count);

    if (count != sizeof(buf))
        return -EFAULT;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;

    memset(htc_debug_flag,0,FLAG_LEN+1);
    memcpy(htc_debug_flag,buf+2,FLAG_LEN);

    SECMSG("Receive :%s\n",buf);
    SECMSG("Flag    :%s\n",htc_debug_flag);

    pnum = get_partition_num_by_name("misc");

    if (pnum < 0) {
        printk(KERN_ERR"unknown partition number for misc partition\n");
        return 0;
    }

    snprintf(filename, 32, "/dev/block/mmcblk0p%d", pnum);

    filp = filp_open(filename, O_RDWR, 0);
    if (IS_ERR(filp)) {
        printk(KERN_ERR"unable to open file: %s\n", filename);
        return PTR_ERR(filp);
    }

    SECMSG("%s: offset :%d\n", __func__, offset);
    filp->f_pos = offset;
    nread = kernel_write(filp, buf, FLAG_LEN+2, filp->f_pos);
    SECMSG("%s:wrire: %s (%zd)\n", __func__, buf, nread);

    if (filp)
        filp_close(filp, NULL);

    return count;
}

static int htc_debug_parse_tbl(struct device *dev, char *prop)
{
    int ret, prop_len;
    char *data = NULL;

    if (!of_find_property(dev->of_node, prop, &prop_len))
        return -EINVAL;

    pr_info("%s - length: %d\n", __func__, prop_len);
    if(prop_len < FLAG_LEN)
        return -EINVAL;

    ret = of_property_read_string(dev->of_node, prop, (const char **)&data);
    pr_info("%s - loglevel: %s\n", __func__, data);

    memset(htc_debug_flag, 0, (FLAG_LEN + 1));
    memcpy(htc_debug_flag, data, FLAG_LEN);

    
    first_read = 0;
    return prop_len;
}

static int htc_debug_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    pr_info("%s\n", __func__);

    htc_debug_parse_tbl(dev, "htc,loglevel");
    return 0;
}

static struct platform_driver htc_debug_driver =
{
    .probe = htc_debug_probe,
    .driver = {
        .name = HTC_DEBUG_HDF_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = htc_debug_hdf_match_table,
    },
};

static int __init htc_debug_init(void)
{
    struct proc_dir_entry *entry;

    entry = create_proc_entry(PROCNAME, 0660, NULL);
    if (entry == NULL) {
        PERR("unable to create /proc entry");
        return -ENOMEM;
    }

    entry->read_proc = htc_debug_read;
    entry->write_proc = htc_debug_write;

    platform_driver_register(&htc_debug_driver);

    return 0;
}

static void __exit htc_debug_exit(void)
{
    remove_proc_entry(PROCNAME, NULL);
}

module_init(htc_debug_init);
module_exit(htc_debug_exit);

MODULE_DESCRIPTION("HTC Secure Debug Log Driver");
MODULE_LICENSE("GPL");
MODULE_LICENSE("HTC SSD ANDROID");
