/******************************************************************************
 *
 *  file name       : tuner_drv.c
 *  brief note      : The Control Layer of Tmm Tuner Driver
 *
 *  creation data   : 2011.07.25
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 1720                       $ Revision of Last commit
 *  $Date:: 2013-05-08 22:02:48 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/07/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/

/******************************************************************************
 * include
 ******************************************************************************/
#include <linux/module.h>       
#include <linux/kernel.h>       
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include "tuner_drv.h"
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#include <linux/mutex.h>
#endif

//++ DTV_PCN1000007_HTC_GPIO_DEFINE
/* [HTC]  GPIO define */

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/******************************************************************************
 * define
 ******************************************************************************/
#define XA 0
#define XB 1
#define XC 2
#define XD 3

#define GPIO_NRST_XA 90
#define GPIO_NPDREG_XA 25
#define GPIO_NPDXTAL_XA 26
#define GPIO_FULLSEG_INT_XA 93
#define GPIO_SRIO_1V8_EN_XA 95
#define GPIO_FULLSEG_1V1_EN_XA 96
#define FM_FULLSEG_ANT_SW_XA 24

#define GPIO_NRST_XB 90
#define GPIO_NPDREG_XB 25
#define GPIO_NPDXTAL_XB 26
#define GPIO_FULLSEG_INT_XB 81
#define GPIO_SRIO_1V8_EN_XB 83
#define GPIO_FULLSEG_1V1_EN_XB 84
#define FM_FULLSEG_ANT_SW_XB 24
//-- DTV_PCN1000007_HTC_GPIO_DEFINE


//++ DTV_PCN1000009_HTC_DEVICE_NUMBER
/* [HTC]  Device number */
#define DEV_NAME "TUNER" //設備名稱
int TUNER_CONFIG_DRV_MAJOR = 0; //主設備號
int TUNER_CONFIG_DRV_MINOR = 0; //次設備號
//-- DTV_PCN1000009_HTC_DEVICE_NUMBER

/******************************************************************************
 * data
 ******************************************************************************/
/* Mmap用のアドレス保持用 */
void *mem_p;

/* poll制御用テーブル */
wait_queue_head_t g_tuner_poll_wait_queue;       /* poll待ち用waitキュー      */
spinlock_t        g_tuner_lock;                  /* spin_lock用資源           */
unsigned long     g_tuner_wakeup_flag;           /* poll待ちフラグ            */

/* 割り込み要因テーブル */
unsigned char g_tuner_intcnd_f;                  /* INTCNDD_F レジスタ値      */
unsigned char g_tuner_intcnd_s;                  /* INTCNDD_S レジスタ値      */
unsigned char g_tuner_intst_f;
unsigned char g_tuner_intst_s;

/* kernel_thread */
struct task_struct *g_tuner_kthread_id;          /* カーネルスレッド識別子     */
u32                 g_tuner_kthread_flag;        /* カーネルスレッド起床フラグ */
wait_queue_head_t   g_tuner_kthread_wait_queue;  /* カーネルスレッド起床用waitキュー*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
/* mutex */
struct mutex g_tuner_mutex;                       /* 排他制御                  */
#endif

/******************************************************************************
 * function
 ******************************************************************************/
static ssize_t tuner_module_entry_read( struct file* FIle,
                                        char* Buffer,
                                        size_t Count,
                                        loff_t* OffsetPosition );
static ssize_t tuner_module_entry_write( struct file* FIle,
                                         const char* Buffer,
                                         size_t Count,
                                         loff_t* OffsetPosition );
static unsigned int tuner_module_entry_poll( struct file *file,
                                             struct poll_table_struct *poll_tbl );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int tuner_module_entry_ioctl( struct inode* Inode,
                                     struct file* FIle, 
                                     unsigned int uCommand,
                                     unsigned long uArgument );
#else  /* LINUX_VERSION_CODE */ 
static long tuner_module_entry_ioctl( struct file *file,
                                      unsigned int uCommand,
                                      unsigned long uArgument );
#endif /* LINUX_VERSION_CODE */
static int tuner_module_entry_open( struct inode* Inode,
                                    struct file* FIle );
static int tuner_module_entry_close( struct inode* Inode,
                                     struct file* FIle );
static int __devinit tuner_probe( struct platform_device *pdev );
static int __exit tuner_remove( struct platform_device *pdev );
static int  __init tuner_drv_start( void );
static void __exit tuner_drv_end( void );

/* エントリーポイント */
static struct file_operations TunerFileOperations =
{
   .owner   = THIS_MODULE,
   .read    = tuner_module_entry_read,
   .write   = tuner_module_entry_write,
   .poll    = tuner_module_entry_poll,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
   .ioctl   = tuner_module_entry_ioctl,
#else  /* LINUX_VERSION_CODE */
   .unlocked_ioctl = tuner_module_entry_ioctl,
#endif /* LINUX_VERSION_CODE */
   .open    = tuner_module_entry_open,
   .release = tuner_module_entry_close
};

//static const struct of_device_id mn885521_mttable[] = {
//        { .compatible = "MN885521"},
//        { },
//};

static struct platform_driver mmtuner_driver = {
    .probe  = tuner_probe,
    .remove = __exit_p(tuner_remove),
    .driver = { .name = "mmtuner",
                .owner = THIS_MODULE,
//		.of_match_table = mn885521_mttable,
              }
};

static struct platform_device *mmtuner_device;
static struct class *device_class;
unsigned long open_cnt;                           /* OPENカウンタ             */

//++ DTV_PCN1000007_HTC_GPIO_DEFINE
/* [HTC]  GPIO define */
struct mn885521_platform_data {
        int npd_reg_npd_xtal;
        int nrst;
        int spisel;
        int sadr1;
        int sadr2;
	struct regulator *fm_fullseg_pw_vreg;
};
struct mn885521_platform_data *pdata;

struct mn885521_gpios {
        int npdreg;
	int npdxtal;
        int nrst;
	int interrupt;
	int _1v8_en;
	int _1v1_en;
	int fm_fullseg_ant_sw;
};
struct mn885521_gpios *gpios;
//-- DTV_PCN1000007_HTC_GPIO_DEFINE

#ifndef TUNER_CONFIG_IRQ_PC_LINUX
irqreturn_t tuner_interrupt( int irq, void *dev_id );
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
int tuner_interrupt( void );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
/******************************************************************************
 * code area
 ******************************************************************************/
//++ DTV_PCN1000007_HTC_GPIO_DEFINE
/* [HTC]  GPIO define */
int board_gpio_init(void)
{
	int ret =0;
	int pcbid = -1;

        gpios = kzalloc(sizeof(struct mn885521_gpios), GFP_KERNEL);
        if (gpios == NULL)
        {
                INFO_PRINT("%s: can't allocate memory\n", __func__);
                ret = -ENOMEM;
		return ret;
        }

        pcbid = of_machine_projectid(1);
	if (pcbid == XA)
	{
		gpios->npdreg =  GPIO_NPDREG_XA;
		gpios->npdxtal = GPIO_NPDXTAL_XA;
        	gpios->nrst = GPIO_NRST_XA;
        	gpios->interrupt = GPIO_FULLSEG_INT_XA;
        	gpios->_1v8_en = GPIO_SRIO_1V8_EN_XA;
        	gpios->_1v1_en = GPIO_FULLSEG_1V1_EN_XA;
        	gpios->fm_fullseg_ant_sw = FM_FULLSEG_ANT_SW_XA;
	}else{
                gpios->npdreg =  GPIO_NPDREG_XB;
                gpios->npdxtal = GPIO_NPDXTAL_XB;
                gpios->nrst = GPIO_NRST_XB;
                gpios->interrupt = GPIO_FULLSEG_INT_XB;
                gpios->_1v8_en = GPIO_SRIO_1V8_EN_XB;
                gpios->_1v1_en = GPIO_FULLSEG_1V1_EN_XB;
                gpios->fm_fullseg_ant_sw = FM_FULLSEG_ANT_SW_XB;
	}

	printk("%s, pcbid = %x, npdreg = %d, npdxtal = %d, nrst = %d, interrupt = %d, 1v8_en = %d, 1v1_en = %d, fm_fullseg_ant_sw = %d \n", 
		__func__, pcbid, gpios->npdreg, gpios->npdxtal, gpios->nrst, gpios->interrupt, gpios->_1v8_en, gpios->_1v1_en, gpios->fm_fullseg_ant_sw);
	return ret;
}

int board_gpio_request(void)
{
	int ret =0;

	ret = gpio_request(gpios->_1v8_en, "fullseg_1v8_en");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}

	ret = gpio_request(gpios->npdreg, "fullseg_npd_reg");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request(gpios->npdxtal, "fullseg_npd_xtal");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}

	ret = gpio_request(gpios->nrst, "fullseg_nrst");
	if (ret < 0) {
		pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}


	return ret;
}

void board_gpio_free(void)
{
        gpio_free(gpios->_1v8_en);
        gpio_free(gpios->npdreg);
        gpio_free(gpios->npdxtal);
        gpio_free(gpios->nrst);

        return;
}
//-- DTV_PCN1000007_HTC_GPIO_DEFINE

//++ DTV_PCN1000008_HTC_POWER_CONTROL
/* [HTC]  Power control */
int poweron_tuner_rework(int on)
{
	int ret =0;

	if (on)
	{
		printk("[FULLSEG] %s, on\r\n", __func__); 
	        ret = gpio_request(gpios->npdreg, "fullseg_npd_reg");
	        if (ret < 0) {
        	        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                	return ret;
	        }
        	ret = gpio_direction_output(gpios->npdreg, 1);
	        if (ret < 0) {
        	        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                	gpio_free(gpios->npdreg);
	                return ret;
        	}
		msleep(10); // delay 10ms

                ret = gpio_request(gpios->npdxtal, "fullseg_npd_xtal");
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                        return ret;
                }
                ret = gpio_direction_output(gpios->npdxtal, 1);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->npdxtal);
                        return ret;
                }
                msleep(10); // delay 10ms


                ret = gpio_request(gpios->nrst, "fullseg_nrst");
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                        return ret;
                }
                ret = gpio_direction_output(gpios->nrst, 1);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->nrst);
                        return ret;
                }

	}else{
		printk("[FULLSEG] %s, off\r\n", __func__);
                ret = gpio_request(gpios->nrst, "fullseg_nrst");
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                        return ret;
                }
                ret = gpio_direction_output(gpios->nrst, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->nrst);
                        return ret;
                }
		msleep(10); //delay 10ms

                ret = gpio_request(gpios->npdxtal, "fullseg_npd_xtal");
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                        return ret;
                }
                ret = gpio_direction_output(gpios->npdxtal, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->npdxtal);
                        return ret;
                }
                ret = gpio_request(gpios->npdreg, "fullseg_npd_reg");
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                        return ret;
                }
                ret = gpio_direction_output(gpios->npdreg, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->npdreg);
                        return ret;
                }
                msleep(10); //delay 10ms
	}
	return ret;
}

static struct regulator *reg_8941_l17;

static int fm_fullseg_antenna_sw_power_enable(char *power, unsigned volt, struct regulator **tuner_power)
{
    int rc;

    if (power == NULL)
       return -ENODEV;

    *tuner_power = regulator_get(NULL, power);

    if (IS_ERR(*tuner_power)) {
        printk(KERN_ERR "[FULLSEG] %s: Unable to get %s\n", __func__, power);
        return -ENODEV;
    }

    if (volt == 2850000) {
        rc = regulator_set_voltage(*tuner_power, volt, volt);
        if (rc < 0) {
            printk(KERN_ERR "[FULLSEG] %s: unable to set %s voltage to %d rc:%d\n", __func__, power, volt, rc);
            regulator_put(*tuner_power);
            *tuner_power = NULL;
            return -ENODEV;
        }
    }
    else
    {
        printk(KERN_ERR "[FULLSEG] %s: Volt is not set 2V85, set volt is %d\n", __func__, volt);
    }

    rc = regulator_enable(*tuner_power);
    if (rc < 0) {
        printk(KERN_ERR "[FULLSEG] %s: Enable regulator %s failed\n", __func__, power);
        regulator_put(*tuner_power);
        *tuner_power = NULL;
        return -ENODEV;
    }

    return rc;
}

static int fm_fullseg_antenna_sw_power_disable(struct regulator *tuner_power)
{
    int rc;
    if (tuner_power == NULL)
        return -ENODEV;

    if (IS_ERR(tuner_power)) {
        printk(KERN_ERR "[FULLSEG] %s: Invalid requlator ptr\n", __func__);
        return -ENODEV;
    }

    rc = regulator_disable(tuner_power);
    if (rc < 0)
        printk(KERN_ERR "[FULLSEG] %s: disable regulator failed\n", __func__);

	regulator_put(tuner_power);
	tuner_power = NULL;
	return rc;
}

int fm_ant_power_fullseg(int on)
{
	int ret=0;

	DEBUG_PRINT("fm_ant_power_fullseg: %d", on);
	if(on)
	{
		fm_fullseg_antenna_sw_power_enable("8941_l17", 2850000, &reg_8941_l17);
		ret = gpio_request(gpios->fm_fullseg_ant_sw, "fm_fullseg_ant_sw");
		if (ret < 0) {
			pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
			return ret;
		}
		ret = gpio_direction_output(gpios->fm_fullseg_ant_sw, 0);
		if (ret < 0) {
			pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(gpios->fm_fullseg_ant_sw);
			return ret;
		}
	}
	else
		fm_fullseg_antenna_sw_power_disable(reg_8941_l17);
	return ret;
}

int poweron_tuner(int on)
{
	int ret =0;

	if (on)
	{
		printk("[FULLSEG] %s, on\r\n", __func__); 
                ret = gpio_direction_output(gpios->_1v8_en, 1);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->_1v8_en);
                        return ret;
                }

                gpio_tlmm_config(GPIO_CFG(gpios->_1v1_en, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
                gpio_set_value(gpios->_1v1_en, 1);

        	ret = gpio_direction_output(gpios->npdreg, 1);
	        if (ret < 0) {
        	        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                	gpio_free(gpios->npdreg);
	                return ret;
        	}
		msleep(10); // delay 10ms

                ret = gpio_direction_output(gpios->npdxtal, 1);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->npdxtal);
                        return ret;
                }
                msleep(10); // delay 10ms

                ret = gpio_direction_output(gpios->nrst, 1);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->nrst);
                        return ret;
                }

		//tune on Fullseg antenna SW power (2.85v)
        	fm_fullseg_antenna_sw_power_enable("8941_l17", 2850000, &reg_8941_l17);

		gpio_free(gpios->fm_fullseg_ant_sw);
                ret = gpio_request(gpios->fm_fullseg_ant_sw, "fm_fullseg_ant_sw");
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_request failed %d\n", __func__, ret);
                        return ret;
                }
	
                ret = gpio_direction_output(gpios->fm_fullseg_ant_sw, 1);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->fm_fullseg_ant_sw);
                        return ret;
                }
	}else{
		printk("[FULLSEG] %s, off\r\n", __func__);
                ret = gpio_direction_output(gpios->fm_fullseg_ant_sw, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->fm_fullseg_ant_sw);
                        return ret;
                }
		//tune off antenna sw power (2.85v)
	        fm_fullseg_antenna_sw_power_disable(reg_8941_l17);

                ret = gpio_direction_output(gpios->nrst, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->nrst);
                        return ret;
                }
		msleep(10); //delay 10ms

                ret = gpio_direction_output(gpios->npdxtal, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->npdxtal);
                        return ret;
                }

                ret = gpio_direction_output(gpios->npdreg, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->npdreg);
                        return ret;
                }
                msleep(10); //delay 10ms

                gpio_tlmm_config(GPIO_CFG(gpios->_1v1_en, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
                gpio_set_value(gpios->_1v1_en, 0);

                ret = gpio_direction_output(gpios->_1v8_en, 0);
                if (ret < 0) {
                        pr_err("[FULLSEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                        gpio_free(gpios->_1v8_en);
                        return ret;
                }


	}
	return ret;
}
//-- DTV_PCN1000008_HTC_POWER_CONTROL
 
static int mn885521_parse_dt(struct device *dev, struct mn885521_platform_data *pdata)
{
	int rc = 0;
//        struct property *prop = NULL;
//        struct device_node *dt = dev->of_node;

	INFO_PRINT("%s \n", __func__);

        pdata->fm_fullseg_pw_vreg = devm_regulator_get(dev, "fm_fullseg_pw");
        if (IS_ERR(pdata->fm_fullseg_pw_vreg)) {
		pdata->fm_fullseg_pw_vreg = NULL;
                pr_err("%s: could not get fm_fullseg_pw reg!\n", __func__);
                return -EINVAL;
        }else
		INFO_PRINT("%s: get fm_fullseg_pw reg\n", __func__);

        rc = regulator_set_voltage(pdata->fm_fullseg_pw_vreg, 2850000, 2850000);
        if (rc < 0) {
        	pr_err("%s: set voltage failed\n", __func__);
                return -EINVAL;
        }else
		INFO_PRINT("%s: set  fm_fullseg_pw voltage\n", __func__);

#if 0
        prop = of_find_property(dt, "mn885521,npd_reg_npd_xtal", NULL);
        if (prop) {
                pdata->npd_reg_npd_xtal = of_get_named_gpio(dt, "mn885521,npd_reg_npd_xtal", 0);
		printk("%s: pdata->npd_reg_npd_xtal = %x\n", __func__, pdata->npd_reg_npd_xtal);
        }else
		printk("%s: fail to get npd_reg_npd_xtal\n", __func__);
	

        prop = of_find_property(dt, "mn885521,nrst", NULL);
        if (prop) {
                pdata->nrst = of_get_named_gpio(dt, "mn885521,nrst", 0);
		printk("%s: pdata->nrst = %x\n", __func__, pdata->nrst);
        }

        prop = of_find_property(dt, "mn885521,spisel", NULL);
        if (prop) {
                pdata->spisel = of_get_named_gpio(dt, "mn885521,spisel", 0);
		printk("%s: pdata->spisel = %x\n", __func__, pdata->spisel);
        }

        prop = of_find_property(dt, "mn885521,sadr1", NULL);
        if (prop) {
                pdata->sadr1 = of_get_named_gpio(dt, "mn885521,sadr1", 0);
		printk("%s: pdata->sadr1 = %x\n", __func__, pdata->sadr1);
        }

        prop = of_find_property(dt, "mn885521,sadr2", NULL);
        if (prop) {
                pdata->sadr2 = of_get_named_gpio(dt, "mn885521,sadr2", 0);
		printk("%s: pdata->sadr2 = %x\n", __func__, pdata->sadr2);
        }
#endif
        return 0;
}


int read_gpio_from_dt(struct device *dev)
{
        int ret = 0;

        INFO_PRINT("%s\n", __func__);
        pdata = kzalloc(sizeof(struct mn885521_platform_data), GFP_KERNEL);
        if (pdata == NULL)
	{
		INFO_PRINT("%s: can't allocate memory\n", __func__);
                ret = -ENOMEM;
	}
	if (dev->of_node) {
        	ret = mn885521_parse_dt(dev, pdata);
	}else
		INFO_PRINT("%s: of_node is NULL\n", __func__);
        return ret;
}


#if 0
int set_i2c_spi_mode(void)
{
	int ret;

	ret = gpio_request(GPIO_SPISEL, "fullseg_spisel");
        if (ret < 0) {
        	pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
	ret = gpio_direction_output(GPIO_SPISEL, 0);
        if (ret < 0) {
        	pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(GPIO_SPISEL);
                return ret;
	}

        ret = gpio_request(GPIO_SADR2, "fullseg_sadr2");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(GPIO_SADR2, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(GPIO_SADR2);
                return ret;
        }

	return ret;
}
#endif

/******************************************************************************
 *    function:   tuner_probe
 *    brief   :   probe control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   pdev
 *    output  :   none
 ******************************************************************************/
static int __devinit tuner_probe(struct platform_device *pdev)
{
    INFO_PRINT("mmtuner_probe: Called. -4-\n");
    /* ドライバの登録 */
    if (register_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME, &TunerFileOperations))
    {
        ERROR_PRINT("mmtuner_probe: register_chrdev()\
                     Failed Major:%d.\n", TUNER_CONFIG_DRV_MAJOR);
        return -1;
    }

    /* 外部変数の初期化 */
    init_waitqueue_head( &g_tuner_poll_wait_queue );
    spin_lock_init( &g_tuner_lock );
    g_tuner_wakeup_flag = TUNER_OFF;
    g_tuner_intcnd_f = 0x00;
    g_tuner_intcnd_s = 0x00;
	open_cnt         = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    mutex_init(&g_tuner_mutex);
#endif
//++ DTV_PCN1000007_HTC_GPIO_DEFINE
/* [HTC]  GPIO define */
    board_gpio_init();
    board_gpio_request();
//-- DTV_PCN1000007_HTC_GPIO_DEFINE
//    read_gpio_from_dt(&pdev->dev);
//    set_i2c_spi_mode();

    INFO_PRINT("tuner_probe: END.\n");
    return 0;
}

/******************************************************************************
 *    function:   tuner_remove
 *    brief   :   remove control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   pdev
 *    output  :   none
 ******************************************************************************/
static int __exit tuner_remove(struct platform_device *pdev)
{
    INFO_PRINT("tuner_remove: Called.\n");
    TRACE();

    /* 割込みの解放 */
    tuner_drv_release_interrupt();

    /* ドライバの登録解除 */
    unregister_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME);
    
    INFO_PRINT("tuner_remove: END.\n");

    return 0;
}


/******************************************************************************
 *    function:   tuner_kernel_thread
 *    brief   :   kernel_thread of mmtuner driver
 *    date    :   2011.09.16
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
int tuner_kernel_thread( void * arg )
{
    int ret = 0;
    unsigned long flags;
    unsigned long ktread_flg;
    //TUNER_DATA_RW rw_data[ 4 ];                  /* 設定データ                */
    mm_segment_t    oldfs;
    struct sched_param  param; 

    struct i2c_adapter	*adap;
    struct i2c_msg		msgs[4];
    //unsigned char		adr_intcnd_f;
    //unsigned char		adr_intcnd_s;
    unsigned char		buff[3];
    unsigned char		bufs[3];

    INFO_PRINT("tuner_kernel_thread: START.\n");

    /* 内部変数初期化 */
    ret = 0;
    flags = 0;
    ktread_flg = 0;
    param.sched_priority = TUNER_CONFIG_KTH_PRI;

    daemonize( "tuner_kthread" ); 
    INFO_PRINT("tuner_kernel_thread: TUNER_CONFIG_I2C_BUSNUM = %x\n", TUNER_CONFIG_I2C_BUSNUM);

    oldfs = get_fs();
    set_fs( KERNEL_DS );
    ret = sched_setscheduler( g_tuner_kthread_id, SCHED_FIFO, &param );
    set_fs( oldfs );

    buff[0] = (unsigned char)TUNER_DRV_ADR_INTCND_F;
    bufs[0] = (unsigned char)TUNER_DRV_ADR_INTCND_S;

    while(1)
    {
        DEBUG_PRINT("tuner_kernel_thread waiting... ");
        wait_event_interruptible( g_tuner_kthread_wait_queue, g_tuner_kthread_flag );

        spin_lock_irqsave( &g_tuner_lock, flags );
        ktread_flg = g_tuner_kthread_flag;
        g_tuner_kthread_flag &= ~TUNER_KTH_IRQHANDLER;	// 20121122
        spin_unlock_irqrestore( &g_tuner_lock, flags);

        memset( msgs, 0x00, sizeof(struct i2c_msg) * 4 );
        adap = i2c_get_adapter( TUNER_CONFIG_I2C_BUSNUM );
        if (adap == NULL) {
        	TRACE();
            break;
        }

        /* 割り込み要因読み出し要求 */
        if ( ( ktread_flg & TUNER_KTH_IRQHANDLER ) == TUNER_KTH_IRQHANDLER )
        {
            DEBUG_PRINT("tuner_kernel_thread IRQHANDLER start ");

			buff[1] = buff[2] = 0;
			bufs[1] = bufs[2] = 0;

            /* 読み出しデータ */
            /* INTCND_F */
            msgs[0].addr	= TUNER_SLAVE_ADR_M1;
            msgs[0].flags	= 0;	/* write */
            msgs[0].len		= 1;
            msgs[0].buf		= &buff[0];
            msgs[1].addr	= TUNER_SLAVE_ADR_M1;
            msgs[1].flags	= I2C_M_RD;
            msgs[1].len		= 2;
            msgs[1].buf		= buff+1;
            msgs[2].addr	= TUNER_SLAVE_ADR_M2;
            msgs[2].flags	= 0;	/* write */
            msgs[2].len		= 1;
            msgs[2].buf		= &bufs[0];
            msgs[3].addr	= TUNER_SLAVE_ADR_M2;
            msgs[3].flags	= I2C_M_RD;
            msgs[3].len		= 2;
            msgs[3].buf		= bufs+1;

            ret = i2c_transfer(adap, msgs, 4);
            if (ret < 0) {
            	TRACE();
              	i2c_put_adapter(adap);
               	break;
            }
            DEBUG_PRINT("read        slv:0x%02x adr:0x%02x len:%-4d 0x%02x ... 0x%02x ",
                		msgs[0].addr, *(msgs[0].buf), msgs[1].len, msgs[1].buf[0], msgs[1].buf[1]);
            DEBUG_PRINT("read        slv:0x%02x adr:0x%02x len:%-4d 0x%02x ... 0x%02x ",
						msgs[2].addr, *(msgs[2].buf), msgs[3].len, msgs[3].buf[0], msgs[3].buf[1]);

            g_tuner_intcnd_f |= buff[1];
            g_tuner_intcnd_s |= bufs[1];
            g_tuner_intst_f	= buff[2];
            g_tuner_intst_s	= bufs[2];

//            rw_data[ 0 ].slave_adr = TUNER_SLAVE_ADR_M1;
//            rw_data[ 0 ].adr       = TUNER_DRV_ADR_INTCND_F;
//            rw_data[ 0 ].sbit      = TUNER_DRV_ENAS_ALL;
//            rw_data[ 0 ].ebit      = TUNER_DRV_ENAE_ALL;
//            rw_data[ 0 ].enabit    = TUNER_DRV_ENA_ALL;
//            rw_data[ 0 ].param     = TUNER_DRV_PARAM_RINIT;
//            /* INTCND_S */
//            rw_data[ 1 ].slave_adr = TUNER_SLAVE_ADR_M2;
//            rw_data[ 1 ].adr       = TUNER_DRV_ADR_INTCND_S;
//            rw_data[ 1 ].sbit      = TUNER_DRV_ENAS_ALL;
//            rw_data[ 1 ].ebit      = TUNER_DRV_ENAE_ALL;
//            rw_data[ 1 ].enabit    = TUNER_DRV_ENA_ALL;
//            rw_data[ 1 ].param     = TUNER_DRV_PARAM_RINIT;
//            /* INTST_F */
//            rw_data[ 2 ].slave_adr = TUNER_SLAVE_ADR_M1;
//            rw_data[ 2 ].adr       = TUNER_DRV_ADR_INTST_F;
//            rw_data[ 2 ].sbit      = TUNER_DRV_ENAS_ALL;
//            rw_data[ 2 ].ebit      = TUNER_DRV_ENAE_ALL;
//            rw_data[ 2 ].enabit    = TUNER_DRV_ENA_ALL;
//            rw_data[ 2 ].param     = TUNER_DRV_PARAM_RINIT;
//            /* INTST_S */
//            rw_data[ 3 ].slave_adr = TUNER_SLAVE_ADR_M2;
//            rw_data[ 3 ].adr       = TUNER_DRV_ADR_INTST_S;
//            rw_data[ 3 ].sbit      = TUNER_DRV_ENAS_ALL;
//            rw_data[ 3 ].ebit      = TUNER_DRV_ENAE_ALL;
//            rw_data[ 3 ].enabit    = TUNER_DRV_ENA_ALL;
//            rw_data[ 3 ].param     = TUNER_DRV_PARAM_RINIT;
//
//            /* 割り込み要因リード */
//            ret = tuner_drv_hw_access( TUNER_IOCTL_VALGET,
//                                       rw_data,
//                                       4 );
//            if( ret != 0 )
//            {
//                DEBUG_PRINT( " tuner_kernel_thread i2c_read Err " );
//                /* 要因読み込み結果が異常の場合、ALLBitクリアする */
//                rw_data[ 0 ].param = TUNER_DRV_PARAM_ALL;
//                rw_data[ 1 ].param = TUNER_DRV_PARAM_ALL;
//            }
//            else
//            {
//                /* 要因読み込み結果が正常の場合、割り込み要因を保存する */
//                /* 割り込み要因格納 */
//                g_tuner_intcnd_f |= ( unsigned char )rw_data[ 0 ].param;
//                g_tuner_intcnd_s |= ( unsigned char )rw_data[ 1 ].param;
//                g_tuner_intst_f |= ( unsigned char )rw_data[ 2 ].param;
//                g_tuner_intst_s |= ( unsigned char )rw_data[ 3 ].param;
//
//            }

            DEBUG_PRINT( "// IRQ factor update: INTCND_F:0x%02x INTST_F:0x%02x"
            		,g_tuner_intcnd_f, g_tuner_intst_f );
            DEBUG_PRINT( "// IRQ factor update: INTCND_S:0x%02x INTST_S:0x%02x"
            		,g_tuner_intcnd_s, g_tuner_intst_s );
            
            /* 割り込み要因クリア */
//            ret = tuner_drv_hw_access( TUNER_IOCTL_VALSET,
//                                       rw_data,
//                                       2 );
            memset( msgs, 0x00, sizeof(struct i2c_msg) * 4 );
            msgs[0].addr	= TUNER_SLAVE_ADR_M1;
            msgs[0].flags	= 0;	/* write */
            msgs[0].len		= 2;
            msgs[0].buf		= buff;
            msgs[1].addr	= TUNER_SLAVE_ADR_M2;
            msgs[1].flags	= 0;	/* write */
            msgs[1].len		= 2;
            msgs[1].buf		= bufs;
            ret = i2c_transfer(adap, msgs, 2);
            if (ret < 0) {
            	TRACE();
                i2c_put_adapter(adap);
                break;
            }
            i2c_put_adapter(adap);

//            if( ret != 0 )
//            {
//                break;
//            }

            /* poll待ち解除 */
            g_tuner_wakeup_flag = TUNER_ON;
            wake_up( &g_tuner_poll_wait_queue );

            DEBUG_PRINT("tuner_interrupt end ");

//20121122  spin_lock_irqsave( &g_tuner_lock, flags );
//20121122  g_tuner_kthread_flag &= ~TUNER_KTH_IRQHANDLER;
//20121122  spin_unlock_irqrestore( &g_tuner_lock, flags );

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
            /* レベル割り込みの場合は割り込み有効に戻す */
            tuner_drv_enable_interrupt();
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */
        }

        /* カーネルスレッド終了要求 */
        if ( ( ktread_flg & TUNER_KTH_END ) == TUNER_KTH_END )
        {
            DEBUG_PRINT("tuner_kernel_thread KTH_END start ");
            spin_lock_irqsave( &g_tuner_lock, flags );
            g_tuner_kthread_flag &= ~TUNER_KTH_END;
            spin_unlock_irqrestore( &g_tuner_lock, flags );
            break;
        }
    }

    INFO_PRINT("tuner_kernel_thread: END. ");

    return 0;
}


/******************************************************************************
 *    function:   tuner_drv_start
 *    brief   :   initialization control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
static int __init tuner_drv_start(void)
{
    int ret =0;
    struct device *dev = NULL;

    //++ DTV_PCN1000009_HTC_DEVICE_NUMBER
    /* [HTC]  Device number */
    int count = 1;
    dev_t tuner_dev;

    /*  Dynamic assign major */
    ret = alloc_chrdev_region(&tuner_dev, TUNER_CONFIG_DRV_MAJOR, count, DEV_NAME);
    if (ret < 0) 
       printk("Major number allocation is failed\n");

    // FIX ME
    TUNER_CONFIG_DRV_MAJOR = 255; //MAJOR(tuner_dev);
    TUNER_CONFIG_DRV_MINOR = 0; //MINOR(tuner_dev);	
    //-- DTV_PCN1000009_HTC_DEVICE_NUMBER

    INFO_PRINT("mmtuner_tuner_drv_start: Called\n");

    /* ドライバ登録 */
    ret = platform_driver_register(&mmtuner_driver);

    if( ret != 0 )
    {
        ERROR_PRINT("init_module: Error:\
                     failed in platform_driver_register.\n");
        return ret;
    }
    
    /* 領域確保 */
    mmtuner_device = platform_device_alloc("mmtuner", -1);

    if (!mmtuner_device)
    {
        ERROR_PRINT("init_module: Error: failed in platform_device_alloc.\n");
        platform_driver_unregister(&mmtuner_driver);
        return -ENOMEM;
    }
    
    /* デバイス追加 */
    ret = platform_device_add(mmtuner_device);
    if ( ret )
    {
        ERROR_PRINT("init_module: Error: failed in platform_device_add.\n");
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return ret;
    }
    /* デバイスノード(クラス)作成 */
    device_class = class_create(THIS_MODULE, "mmtuner");
    if (IS_ERR(device_class)) 
    {
        ERROR_PRINT("init_module: Error: failed in class_create.\n");
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return PTR_ERR(device_class);
    }

    /* デバイス作成 */
    dev = device_create (device_class, NULL, MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR), NULL, "mmtuner");

    if(IS_ERR(dev))
    {
        ERROR_PRINT("init_module: Error: failed in device_create.\n");
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return PTR_ERR(dev);
    }
   
    /* カーネルスレッド生成処理 */
    g_tuner_kthread_flag = TUNER_KTH_NONE;

    init_waitqueue_head( &g_tuner_kthread_wait_queue );

    g_tuner_kthread_id = kthread_create( tuner_kernel_thread,
                                         NULL,
                                         "tuner_kthread" );
    if( IS_ERR( g_tuner_kthread_id ) )
    {
        g_tuner_kthread_id = NULL;
        platform_device_put(mmtuner_device);
        platform_driver_unregister(&mmtuner_driver);
        return -EIO;
    }

    wake_up_process( g_tuner_kthread_id );

    INFO_PRINT("mmtuner_tuner_drv_start: END\n");
    return 0;
}

/******************************************************************************
 *    function:   tuner_drv_end
 *    brief   :   exit control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
static void __exit tuner_drv_end(void)
{
    INFO_PRINT("mmtuner_tuner_drv_end: Called\n");

//++ DTV_PCN1000007_HTC_GPIO_DEFINE
/* [HTC]  GPIO define */
    board_gpio_free();
//-- DTV_PCN1000007_HTC_GPIO_DEFINE

    /* カーネルスレッドを終了する */
    g_tuner_kthread_flag |= TUNER_KTH_END;
    if( waitqueue_active( &g_tuner_kthread_wait_queue ))
    {
        wake_up( &g_tuner_kthread_wait_queue );
    }

    /* カーネルスレッドの登録を解除 */
    if( g_tuner_kthread_id )
    {
        kthread_stop( g_tuner_kthread_id );
    }

    /* デバイス削除 */
    device_destroy(device_class, MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR));
    /* クラス削除*/
    class_destroy(device_class);
    /* デバイス登録解除 */
    platform_device_unregister(mmtuner_device);
    /* ドライバ登録解除 */
    platform_driver_unregister(&mmtuner_driver);

    INFO_PRINT("mmtuner_tuner_drv_end: END\n");
}

/******************************************************************************
 *    function:   tuner_module_entry_open
 *    brief   :   open control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   Inode
 *            :   FIle
 *    output  :   none
 ******************************************************************************/
static int tuner_module_entry_open(struct inode* Inode, struct file* FIle)
{

    INFO_PRINT("tuner_module_entry_open: Called\n");

#ifdef  TUNER_CONFIG_DRV_MULTI      /* 多重open可能 */
    open_cnt++;
#else  /* TUNER_CONFIG_DRV_MULTI */ /* 多重open不可 */
	/* 既にopenを実施している場合 */
    if( open_cnt > 0 )
    {
        INFO_PRINT("tuner_module_entry_open: open error\n");
        return -1;
    }
	/* 初めてのopenの場合 */
    else
    {
        INFO_PRINT("tuner_module_entry_open: open_cnt = 1\n");
        open_cnt++;
    }
#endif /* TUNER_CONFIG_DRV_MULTI */

//++ DTV_PCN1000008_HTC_POWER_CONTROL
/* [HTC]  Power control */
    poweron_tuner(1);	
//-- DTV_PCN1000008_HTC_POWER_CONTROL
    return 0;
}

/******************************************************************************
 *    function:   tuner_module_entry_close
 *    brief   :   close control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   Inode
 *            :   FIle
 *    output  :   none
 ******************************************************************************/
static int tuner_module_entry_close(struct inode* Inode, struct file* FIle)
{
    	struct devone_data *dev;

    	INFO_PRINT("tuner_module_entry_close: Called\n");

	//++ DTV_PCN1000008_HTC_POWER_CONTROL
	/* [HTC]  Power control */
	poweron_tuner(0);
	//-- DTV_PCN1000008_HTC_POWER_CONTROL

	/* open_cntが0以下の場合 */
	if( open_cnt <= 0 )
	{
        INFO_PRINT("tuner_module_entry_close: close error\n");
        return -1;
	}
	else
	{
		open_cnt--;
	}

	/* 全てのopenがcloseされた場合 */
	if( open_cnt == 0 )
	{
        /* 割込みの解放 */
	tuner_drv_release_interrupt();

        if( FIle == NULL )
        {
            return -1;
        }

        dev = FIle->private_data;

        if( dev )
        {
            kfree( dev );
        }
	}

    return 0;

}

/******************************************************************************
 *    function:   tuner_module_entry_read
 *    brief   :   read control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   FIle
 *            :   Buffer
 *            :   Count
 *            :   OffsetPosition
 *    output  :   none
 ******************************************************************************/
static ssize_t tuner_module_entry_read(struct file * FIle, char * Buffer,
                                 size_t Count, loff_t * OffsetPosition)
{
    return 0;

}

/******************************************************************************
 *    function:   tuner_module_entry_write
 *    brief   :   write control of a driver
 *    date    :   2011.10.31
 *    author  :   K.Okawa(KXD14)
 *
 *    return  :    0		normal exit
 *            :   -1		error exit
 *    input   :   FIle
 *            :   Buffer	[slave][reg.addr][data-0][data-1]...[data-(n-1)]
 *            :   Count		(>=3) n+2
 *            :   OffsetPosition
 *    output  :   none
 ******************************************************************************/
static ssize_t tuner_module_entry_write(struct file* FIle,
		const char* Buffer, size_t Count, loff_t* OffsetPosition)
{
    int				ret;
    unsigned long	copy_ret;
    //TUNER_DATA_RW *write_arg;			/* pointer to data area */
    unsigned char	*buf;				/* pointer to data area */

    struct i2c_adapter	*adap;
    struct i2c_msg		msgs[1];

    /* argument check */
    if (Count < 3) {
    	TRACE();
    	return -EINVAL;
    }

    /* memory allocation for data area */
    buf = (unsigned char *)vmalloc(Count);
    if (buf == NULL) {
        return -EINVAL;
    }

    copy_ret = copy_from_user(buf, Buffer, Count);
    if (copy_ret != 0) {
        vfree(buf);
        return -EINVAL;
    }

    /* get i2c adapter */
    adap = i2c_get_adapter(TUNER_CONFIG_I2C_BUSNUM);
    if (adap == NULL) {
    	TRACE();
    	vfree(buf);
    	return -EINVAL;
    }

    /* construct i2c message */
    memset(msgs, 0x00, sizeof(struct i2c_msg) * 1);

    msgs[0].addr	= buf[0];
    msgs[0].flags	= 0;		/* write */
    msgs[0].len		= Count - 1;
    msgs[0].buf		= buf + 1;

    ret = i2c_transfer(adap, msgs, 1);
    if (ret < 0) {
    	TRACE();
    	i2c_put_adapter(adap);
    	vfree(buf);
    	return -EINVAL;
    }
    //DEBUG_PRINT("write        slv:0x%02x adr:0x%02x len:%-4d 0x%02x ... 0x%02x ",
    //		buf[0], buf[1], Count-2, buf[2], buf[Count-1]);

    vfree(buf);
    return ret;
}

/******************************************************************************
 *    function:   tuner_module_entry_ioctl
 *    brief   :   ioctl control of a driver
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   Inode
 *            :   FIle
 *            :   uCommand
 *            :   uArgument
 *    output  :   none
 ******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int tuner_module_entry_ioctl(struct inode* Inode, struct file* FIle,
                              unsigned int uCommand, unsigned long uArgument)
#else  /* LINUX_VERSION_CODE */
static long tuner_module_entry_ioctl(struct file *file,
                              unsigned int uCommand, unsigned long uArgument)
#endif /* LINUX_VERSION_CODE */

{
    int                   ret;
    TUNER_DATA_RW         data;
    unsigned long         copy_ret;
    int                   param;
    TUNER_DATA_RW         event_status[ TUNER_EVENT_REGNUM ];

    //INFO_PRINT("tuner_module_entry_ioctl:START\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    /* 排他制御:ロック開始 */
    mutex_lock(&g_tuner_mutex);
#endif

    /* 引数チェック */
    if( uArgument == 0 )
    {
        TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
        /* 排他制御:ロック解除 */
        mutex_unlock(&g_tuner_mutex);
#endif  
        return -EINVAL;
    }
    
    switch( uCommand )
    {
        /* HWからデータを取得 */
        case TUNER_IOCTL_VALGET:
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            ret = tuner_drv_hw_access( uCommand, &data, 1 );

            if( ret == 0 )
            {
                /* 読み出した値を上位へ返却 */
                copy_ret = copy_to_user( &( *(TUNER_DATA_RW *)uArgument ),
                                         &data,
                                         sizeof( TUNER_DATA_RW ));
                if( copy_ret != 0 )
                {
                    TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                    /* 排他制御:ロック解除 */
                    mutex_unlock(&g_tuner_mutex);
#endif  
                    return -EINVAL;
                }
            }

            break;


        /* HWへデータを設定 */
        case TUNER_IOCTL_VALSET:
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            ret = tuner_drv_hw_access( uCommand, &data, 1 );
            break;

        case TUNER_IOCTL_VALGET_EVENT:
            /* INTCND_F */
            copy_ret = copy_to_user( &( *( unsigned char *)uArgument ),
                                     &g_tuner_intcnd_f,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }
            /* INTCND_S */
            copy_ret = copy_to_user( &( *( unsigned char *)( uArgument + 1 )),
                                     &g_tuner_intcnd_s,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;
            }
            /* INTST_F */
            copy_ret = copy_to_user( &( *( unsigned char *)(uArgument + 2)),
                                     &g_tuner_intst_f,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;
            }
            /* INTCND_F */
            copy_ret = copy_to_user( &( *( unsigned char *)(uArgument + 3)),
                                     &g_tuner_intst_s,
                                     sizeof( unsigned char ));
            if (copy_ret != 0) {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            DEBUG_PRINT( "// IRQ factor send: INTCND_F:0x%02x INTST_F:0x%02x"
            		,g_tuner_intcnd_f, g_tuner_intst_f );
            DEBUG_PRINT( "// IRQ factor send: INTCND_S:0x%02x INTST_S:0x%02x"
            		,g_tuner_intcnd_s, g_tuner_intst_s );

            /* 要因初期化 */
            g_tuner_intcnd_f = 0x00;
            g_tuner_intcnd_s = 0x00;
            g_tuner_intst_f = 0x00;
            g_tuner_intst_s = 0x00;

            ret = copy_ret;

            break;
        /* イベント設定 */
        case TUNER_IOCTL_VALSET_EVENT:
        	DEBUG_PRINT("*** VALSET_EVENT ***\n");
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            /* when 1st time of event setting, be enable interrupt */
            event_status[0].slave_adr = TUNER_SLAVE_ADR_M1;  /* slave address */
            event_status[0].adr       = REG_INTDEF1_F;       /* reg. address  */
            event_status[0].sbit      = SIG_ENS_INTDEF1_F;   /* start bit position */
            event_status[0].ebit      = SIG_ENE_INTDEF1_F;   /* end bit position */
            event_status[0].param     = 0x00;                /* clear for read */
            event_status[0].enabit    = SIG_ENA_INTDEF1_F;   /* enable bit mask */
            event_status[1].slave_adr = TUNER_SLAVE_ADR_M1;
            event_status[1].adr       = REG_INTDEF2_F;
            event_status[1].sbit      = SIG_ENS_INTDEF2_F;
            event_status[1].ebit      = SIG_ENE_INTDEF2_F;
            event_status[1].param     = 0x00;
            event_status[1].enabit    = SIG_ENA_INTDEF2_F;
            event_status[2].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[2].adr       = REG_INTDEF1_S;
            event_status[2].sbit      = SIG_ENS_INTDEF1_S;
            event_status[2].ebit      = SIG_ENE_INTDEF1_S;
            event_status[2].param     = 0x00;
            event_status[2].enabit    = SIG_ENA_INTDEF1_S;
            event_status[3].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[3].adr       = REG_INTDEF2_S;
            event_status[3].sbit      = SIG_ENS_INTDEF2_S;
            event_status[3].ebit      = SIG_ENE_INTDEF2_S;
            event_status[3].param     = 0x00;
            event_status[3].enabit    = SIG_ENA_INTDEF2_S;

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALGET, event_status, TUNER_EVENT_REGNUM );
            
            if( ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;  
            }

            if (((event_status[0].param & event_status[0].enabit) == 0x00) &&
            	((event_status[1].param & event_status[1].enabit) == 0x00) &&
                ((event_status[2].param & event_status[2].enabit) == 0x00) &&
                ((event_status[3].param & event_status[3].enabit) == 0x00))
            {
            	DEBUG_PRINT("*** REQUEST IRQ ***");
                ret = tuner_drv_set_interrupt();
                
                if( ret != 0 )
                {
                    TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif
                return -EINVAL;  
                }
            }

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALSET, &data, 1 );

            break;
        /* イベント解除 */
        case TUNER_IOCTL_VALREL_EVENT:
            copy_ret = copy_from_user( &data,
                                       &( *(TUNER_DATA_RW *)uArgument ),
                                       sizeof( TUNER_DATA_RW ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALSET, &data, 1 );

            if( ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }

            /* イベント設定が最終の場合、割り込みを無効にする */
            event_status[0].slave_adr = TUNER_SLAVE_ADR_M1;  /* slave address */
            event_status[0].adr       = REG_INTDEF1_F;       /* reg. address  */
            event_status[0].sbit      = SIG_ENS_INTDEF1_F;   /* start bit position */
            event_status[0].ebit      = SIG_ENE_INTDEF1_F;   /* end bit position */
            event_status[0].param     = 0x00;                /* clear for read */
            event_status[0].enabit    = SIG_ENA_INTDEF1_F;   /* enable bit mask */
            event_status[1].slave_adr = TUNER_SLAVE_ADR_M1;
            event_status[1].adr       = REG_INTDEF2_F;
            event_status[1].sbit      = SIG_ENS_INTDEF2_F;
            event_status[1].ebit      = SIG_ENE_INTDEF2_F;
            event_status[1].param     = 0x00;
            event_status[1].enabit    = SIG_ENA_INTDEF2_F;
            event_status[2].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[2].adr       = REG_INTDEF1_S;
            event_status[2].sbit      = SIG_ENS_INTDEF1_S;
            event_status[2].ebit      = SIG_ENE_INTDEF1_S;
            event_status[2].param     = 0x00;
            event_status[2].enabit    = SIG_ENA_INTDEF1_S;
            event_status[3].slave_adr = TUNER_SLAVE_ADR_M2;
            event_status[3].adr       = REG_INTDEF2_S;
            event_status[3].sbit      = SIG_ENS_INTDEF2_S;
            event_status[3].ebit      = SIG_ENE_INTDEF2_S;
            event_status[3].param     = 0x00;
            event_status[3].enabit    = SIG_ENA_INTDEF2_S;

            ret = tuner_drv_hw_access( TUNER_IOCTL_VALGET, event_status, TUNER_EVENT_REGNUM );

            if (((event_status[0].param & event_status[0].enabit) == 0x00) &&
                ((event_status[1].param & event_status[1].enabit) == 0x00) &&
                ((event_status[2].param & event_status[2].enabit) == 0x00) &&
                ((event_status[3].param & event_status[3].enabit) == 0x00))
            {
            	DEBUG_PRINT("*** release IRQ REQUEST ***");
                tuner_drv_release_interrupt();
            }

            break;
        case TUNER_IOCTL_VALSET_POWER:
            copy_ret = copy_from_user( &param,
                                       &( *( int * )uArgument ),
                                       sizeof( int ));
            if( copy_ret != 0 )
            {
                TRACE();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                /* 排他制御:ロック解除 */
                mutex_unlock(&g_tuner_mutex);
#endif  
                return -EINVAL;
            }
            ret = tuner_drv_ctl_power( param );

            break;
        default:
            TRACE();
            ret = -EINVAL;
            break;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    /* 排他制御:ロック解除 */
    mutex_unlock(&g_tuner_mutex);
#endif    
    //INFO_PRINT("tuner_module_entry_ioctl:END\n");
    return ret;
}

/******************************************************************************
 *    function:   tuner_module_entry_poll
 *    brief   :   poll control of a driver
 *    date    :   2011.08.23
 *    author  :   M.Takahashi(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   file
 *            :   poll_tbl
 *    output  :   none
 ******************************************************************************/
static unsigned int tuner_module_entry_poll(
                        struct file *file,
                        struct poll_table_struct *poll_tbl )
{
    unsigned long tuner_flags;
    unsigned int  tuner_mask;


    /* 初期化 */
    tuner_mask = 0;

    /* wait状態遷移 */
    poll_wait( file, &g_tuner_poll_wait_queue, poll_tbl );

    /* 割り込み禁止 */
    spin_lock_irqsave( &g_tuner_lock, tuner_flags );

    /* wait解除状態の場合 */
    if( g_tuner_wakeup_flag == TUNER_ON )
    {
        tuner_mask = ( POLLIN | POLLRDNORM );
    }
    
    g_tuner_wakeup_flag = TUNER_OFF;

    /* 割り込み許可 */
    spin_unlock_irqrestore( &g_tuner_lock, tuner_flags );

    return tuner_mask;
}

/******************************************************************************
 *    function:   tuner_interrupt
 *    brief   :   interrpu control of a driver
 *    date    :   2011.08.23
 *    author  :   M.Takahashi(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   irq
 *            :   dev_id
 *    output  :   none
 ******************************************************************************/
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
irqreturn_t tuner_interrupt( int irq, void *dev_id )
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
int tuner_interrupt( void )
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
{
    DEBUG_PRINT("tuner_interrupt start ");

    /* 処理はカーネルスレッドで行う */
    g_tuner_kthread_flag |= TUNER_KTH_IRQHANDLER;
    if( waitqueue_active( &g_tuner_kthread_wait_queue ))
    {
#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
        /* レベル割り込みの場合は割り込み無効に設定する */
        tuner_drv_disable_interrupt();
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */
        wake_up( &g_tuner_kthread_wait_queue );
    }
    else
    {
        DEBUG_PRINT("tuner_interrupt waitqueue_active err!!! ");
        /* カーネルスレッドを起床がNGの場合、割り込みを停止する */
        /* 割り込みが発生し続けないためのフェールセーフ処理 */
//20121122        tuner_drv_release_interrupt();
    }

    DEBUG_PRINT("tuner_interrupt end ");

    return IRQ_HANDLED;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Panasonic Co., Ltd.");
MODULE_DESCRIPTION("MM Tuner Driver");

module_init(tuner_drv_start);
module_exit(tuner_drv_end);
/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
