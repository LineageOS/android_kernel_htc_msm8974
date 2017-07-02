/* drivers/i2c/chips/epl88051_KPW.c - light and proxmity sensors driver
 * Copyright (C) 2014 ELAN Corporation.
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

#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/epl88051_KPW.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

#define PS_RAW_8BIT     	0   	
#define ALS_POLLING_MODE	0	
#define PS_POLLING_MODE         0	

#define ALS_LOW_THRESHOLD	1000
#define ALS_HIGH_THRESHOLD	3000

#if PS_RAW_8BIT
#define PS_LOW_THRESHOLD	1
#define PS_HIGH_THRESHOLD	3
#else
#define PS_LOW_THRESHOLD	500
#define PS_HIGH_THRESHOLD	800
#endif
#define LUX_PER_COUNT		700

#define S5PV210 		0
#define ZERADUG 		0
#define QCOM   			1

#define HTC_ATTR 		1
#define HTC_ALS     		1

#define COMMON_DEBUG 		0
#define ALS_DEBUG   		0
#define PS_DEBUG    		0
#define SHOW_DBG    		0

#define ALS_DYN_INTT    	1
#define PS_DYN_K        	1

#define ALS_LEVEL    		16

static int polling_time = 200;

static int als_level[] = { 20, 45, 70, 90, 150,
			   300, 500, 700, 1150, 2250,
			   4500, 8000, 15000, 30000, 50000 };

static int als_value[] = { 10, 30, 60, 80,
			   100, 200, 400, 600,
			   800, 1500, 3000, 6000,
			   10000, 20000, 40000, 60000 };
#if HTC_ALS
static struct mutex als_get_adc_mutex;
#define ALS_INTRE_LEVEL    9
unsigned long als_intr_level[] = {15, 39, 63, 316, 639, 4008, 5748, 10772, 14517, 65535};
static uint32_t adctable[10] = {0};
uint32_t golden_adc = 0xE43; 

#endif

int als_frame_time	= 0;
int ps_frame_time  = 0;

#if ALS_DYN_INTT
int dynamic_intt_idx;
int dynamic_intt_init_idx = 1;	
int c_gain;
int dynamic_intt_lux = 0;

uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 12000;
uint32_t dynamic_intt_min_lux = 0;
uint32_t dynamic_intt_min_unit = 1000;

static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_1024, EPL_ALS_INTT_256};
static int als_dynamic_intt_value[] = {1024, 256};
static int als_dynamic_intt_gain[] = {EPL_GAIN_MID, EPL_GAIN_LOW};
static int als_dynamic_intt_high_thr[] = {60000, 53000};
static int als_dynamic_intt_low_thr[] = {200, 200};
static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_value)/sizeof(int);
#endif

#if ALS_DYN_INTT
typedef enum {
	CMC_BIT_LSRC_NON	= 0x0,
	CMC_BIT_LSRC_SCALE     	= 0x1,
	CMC_BIT_LSRC_SLOPE	= 0x2,
	CMC_BIT_LSRC_BOTH	= 0x3,
} CMC_LSRC_REPORT_TYPE;
#endif

#if PS_DYN_K
static int dynk_polling_delay = 200;
int dynk_min_ps_raw_data;
int dynk_max_ir_data;

u32 dynk_thd_low = 0;
u32 dynk_thd_high = 0;

#define TH_ADD 			200
#define NEAR_DELAY_TIME		((100 * HZ) / 1000)
#endif

#if HTC_ATTR
static uint16_t ps_canc_set;
static uint16_t mfg_thd;
static int kcalibrated;
static int PS_max;
static int p_status = 9;
static int als_kadc = 0;
int f_epl88051_level = -1;
static int phone_status;
static int call_count = 0;
#define ALS_CALIBRATED		0x6DA5
#define PS_CALIBRATED		0X5053
static int lightsensor_cali;
static int psensor_cali;
int current_lightsensor_kadc;
static int delay_ls_adc = 0;
static int debug_flag = 0;

#define CALIBRATION_DATA_PATH	"/calibration_data"
#define LIGHT_SENSOR_FLASH_DATA	"als_flash"
#define PSENSOR_FLASH_DATA	"ps_flash"
#endif

bool polling_flag = true;
bool eint_flag = true;

static const char ps_cal_file[]="/data/data/com.eminent.ps.calibration/ps.dat";

static const char als_cal_file[]="/data/data/com.eminent.ps.calibration/als.dat";

static int PS_h_offset = 2000;
static int PS_l_offset = 1000;
static int PS_MAX_XTALK = 30000;


#define TXBYTES			2
#define RXBYTES			2

#define PACKAGE_SIZE		8
#define I2C_RETRY_COUNT 	10
#define EPL_DEV_NAME		"EPL88051_KPW"
#define DRIVER_VERSION          "1.1.5"

static int PS_DEBUG_FLAG = 0;
static int ALS_DEBUG_FLAG = 0;
static int COMMON_DEBUG_FLAG = 0;
module_param(PS_DEBUG_FLAG, int, 0600);
module_param(ALS_DEBUG_FLAG, int, 0600);
module_param(COMMON_DEBUG_FLAG, int, 0600);

typedef enum {
	CMC_BIT_RAW   		= 0x0,
	CMC_BIT_PRE_COUNT     	= 0x1,
	CMC_BIT_DYN_INT		= 0x2,
	CMC_BIT_DEF_LIGHT	= 0x4,
	CMC_BIT_TABLE		= 0x8,
	CMC_BIT_LEVEL		= 0x16,
} CMC_ALS_REPORT_TYPE;

typedef struct _epl_raw_data {
	u8 raw_bytes[PACKAGE_SIZE];
	u16 renvo;
} epl_raw_data;

struct epl_sensor_priv
{
	struct i2c_client *client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct work_struct resume_work;
	struct delayed_work  eint_work;
	struct delayed_work  polling_work;
	struct workqueue_struct *lp_wq;
	struct wake_lock ps_wake_lock;

#if PS_DYN_K
	struct delayed_work  dynk_thd_polling_work;
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#if HTC_ATTR
	struct class *epl_sensor_class;
	struct device *ls_dev;
	struct device *ps_dev;
	int ps_pocket_mode;
	unsigned long j_start;
	unsigned long j_end;

	uint32_t emmc_als_kadc;
	uint32_t emmc_ps_kadc1;
	uint32_t emmc_ps_kadc2;
	uint32_t irq_gpio_flags;
#endif
#if HTC_ALS
	uint32_t *adc_table;
	uint32_t *adc_table_high_lux;
	uint16_t cali_table[10];
	uint16_t cali_table_high_lux[10];
#endif
	int intr_pin;
	int (*power)(int on);

	int ps_opened;
	int als_opened;

	int als_suspend;
	int ps_suspend;

	int lux_per_count;

	int enable_pflag;
	int enable_lflag;
	int enable_gflag;

	int read_flag;
	int irq;
	spinlock_t lock;
#if PS_DYN_K
	int ps_threshold_diff;
	int ps_th_add;
#endif

#if ALS_DYN_INTT
	uint32_t ratio;
	uint32_t last_ratio;
	int c_gain_h;			
	int c_gain_l; 			
	uint32_t lsource_thd_high;	
	uint32_t lsource_thd_low;	
	u8 als_init_cycle;
	u8 als_cycle;
#endif

#if HTC_ATTR
	uint32_t als_kadc;
	uint32_t als_kadc_high_lux;
	uint32_t als_gadc;
	int current_level;
	uint32_t current_adc;
	int ls_calibrate;
	uint32_t epl88051_slave_address;
	uint32_t ps_thd_set;
	uint32_t ps_thd_no_cal;
	uint32_t dynamical_threshold;
	uint32_t dark_level;
	uint16_t inte_ps_canc;
	uint16_t mfg_thd;
	u8       ps_duty;
	u8       ps_pers;
	u8       ps_it;
	u8       ps_hd;
	u8       ps_led_current;
#endif
	
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[ALS_LEVEL-1];
	u32 als_value[ALS_LEVEL];
	struct pinctrl 	*pinctrl;
	struct pinctrl_state *gpio_state_init;
};

static struct platform_device *sensor_dev;
struct epl_sensor_priv *epl_sensor_obj;
static epl_optical_sensor epl_sensor;
int i2c_max_count=8;

static epl_raw_data	gRawData;

static struct wake_lock ps_lock;
static struct mutex sensor_mutex;
static struct mutex sensor_enable_mutex;


static const char ElanPsensorName[] = "proximity";
static const char ElanALsensorName[] = "lightsensor-level";

#define LOG_TAG			 "[EPL88051_KPW] "
#define LOG_FUN(f)             	 printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define LOG_INFO(fmt, args...) 	 printk(KERN_INFO LOG_TAG fmt, ##args)
#define LOG_ERR(fmt, args...)	 printk(KERN_ERR  LOG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)


void epl_sensor_update_mode(struct i2c_client *client);
int epl_sensor_read_als_status(struct i2c_client *client);
static int epl_sensor_setup_interrupt(struct epl_sensor_priv *epld);
static int epl88051_pinctrl_init(struct epl_sensor_priv *epld);

static void epl_sensor_eint_work(struct work_struct *work);
static DECLARE_WORK(epl_sensor_irq_work, epl_sensor_eint_work);
static void epl_sensor_resume_work(struct work_struct *work);

static void epl_sensor_polling_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, epl_sensor_polling_work);

#if PS_DYN_K
void epl_sensor_dynk_thd_polling_work(struct work_struct *work);
void epl_sensor_restart_dynk_polling(void);
static DECLARE_DELAYED_WORK(dynk_thd_polling_work, epl_sensor_dynk_thd_polling_work);
#endif

#if HTC_ALS
static int lightsensor_update_table(struct epl_sensor_priv *epld);
#endif

int als_thd_add = 50;
static int epl_sensor_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr ;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        LOG_ERR("i2c write error,TXBYTES %d\n",ret);
        mdelay(10);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        LOG_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}

static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = epl_sensor_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
    return 0;
}
static int epl_sensor_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{

    int ret = 0;


    int retry;
    int read_count=0, rx_count=0;

    while(bytecount>0)
    {
        epl_sensor_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);

        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            LOG_ERR("i2c read error,RXBYTES %d\r\n",ret);
            mdelay(10);
        }

        if(retry>=I2C_RETRY_COUNT)
        {
            LOG_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}

static void epl_sensor_restart_polling(void)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	cancel_delayed_work(&epld->polling_work);
	
	if (epld->lp_wq)
		queue_delayed_work(epld->lp_wq, &epld->polling_work, msecs_to_jiffies(50));
}

static void epl_sensor_report_lux(int repott_lux)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    if(ALS_DEBUG_FLAG) {
	LOG_INFO("-------------------  ALS raw = %d, lux = %d, lightsensor_cali=%d \n",
		epl_sensor.als.data.channels[1], repott_lux, lightsensor_cali);
    }

    input_report_abs(epld->als_input_dev, ABS_MISC, repott_lux);
    input_sync(epld->als_input_dev);
}
#if ALS_DYN_INTT
long raw_convert_to_lux(u16 raw_data)
{
    long lux = 0;
    long dyn_intt_raw = 0;
    int gain_value;

    if(epl_sensor.als.gain == EPL_GAIN_MID)
    {
        gain_value = 8;
    }
    else if (epl_sensor.als.gain == EPL_GAIN_LOW)
    {
        gain_value = 1;
    }
    
    dyn_intt_raw = raw_data/(gain_value *als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); 

    if(dyn_intt_raw > 0xffff)
        epl_sensor.als.dyn_intt_raw = 0xffff;
    else
        epl_sensor.als.dyn_intt_raw = dyn_intt_raw;

    
    lux = raw_data * (c_gain / (gain_value * als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1])); 

    if(ALS_DEBUG_FLAG) {
	LOG_INFO("[%s]:dyn_intt_raw=%ld, epl_sensor.als.dyn_intt_raw=%d \r\n",
			 __func__, dyn_intt_raw, epl_sensor.als.dyn_intt_raw);
	LOG_INFO("[%s]:raw_data=%d, dyn_intt_raw=%d, lux=%ld \r\n",
			 __func__, raw_data, epl_sensor.als.dyn_intt_raw, lux);
    }

    if(lux >= (dynamic_intt_max_lux * dynamic_intt_min_unit)){

	if(ALS_DEBUG_FLAG) {
		LOG_INFO("[%s]:raw_convert_to_lux: change max lux\r\n", __func__);
    	}
        lux = dynamic_intt_max_lux * dynamic_intt_min_unit;
    }
    else if(lux <= (dynamic_intt_min_lux*dynamic_intt_min_unit)){

	if(ALS_DEBUG_FLAG) {
		LOG_INFO("[%s]:raw_convert_to_lux: change min lux\r\n", __func__);
	}
        lux = dynamic_intt_min_lux * dynamic_intt_min_unit;
    }

    return lux;
}
#endif
static int epl_sensor_get_als_value(struct epl_sensor_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
#if ALS_DYN_INTT
	long now_lux=0, lux_tmp=0;
	bool change_flag = false;
#endif
#if HTC_ALS
	int level=0, i=0;
	uint32_t als_step;
#endif
	switch(epl_sensor.als.report_type) {
	case CMC_BIT_RAW:
		return als;
	break;

	case CMC_BIT_PRE_COUNT:
		return (als * epl_sensor.als.factory.lux_per_count)/1000;
	break;

	case CMC_BIT_TABLE:
		for(idx = 0; idx < obj->als_level_num; idx++) {
		    if(als < als_level[idx])
		        break;
		}

		if(idx >= obj->als_value_num) {
			LOG_ERR("exceed range\n");
			idx = obj->als_value_num - 1;
		}

		if(!invalid) {
			LOG_INFO("ALS: %05d => %05d\n", als, als_value[idx]);
			return als_value[idx];
		} else {
			LOG_ERR("ALS: %05d => %05d (-1)\n", als, als_value[idx]);
			return als;
		}
	break;
#if ALS_DYN_INTT
	case CMC_BIT_DYN_INT:
		if(epl_sensor.als.lsrc_type != CMC_BIT_LSRC_NON) {
			long luxratio = 0;
			epl_sensor_read_als_status(obj->client);

			if (epl_sensor.als.data.channels[0] == 0) {
				epl_sensor.als.data.channels[0] = 1;
				LOG_ERR("[%s]:read ch0 data is 0 \r\n", __func__);
			}
			
			luxratio = (long)((als*dynamic_intt_min_unit) /
						 epl_sensor.als.data.channels[0]);
			obj->ratio = luxratio;

			if((epl_sensor.als.saturation >> 5) == 0) {
				if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SCALE ||
				   epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH) {
				        if(obj->ratio == 0)
						obj->last_ratio = luxratio;
					else
						obj->last_ratio = (luxratio + obj->last_ratio*9) / 10;

					if (obj->last_ratio >= obj->lsource_thd_high) {
						
						c_gain = obj->c_gain_h;
					} else if (obj->last_ratio <= obj->lsource_thd_low) {
						
						c_gain = obj->c_gain_l;
					} else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH) {
						int a = 0, b = 0, c = 0;
						a = (obj->c_gain_h - obj->c_gain_l) * dynamic_intt_min_unit / (obj->lsource_thd_high - obj->lsource_thd_low);
						b = (obj->c_gain_h) - ((a * obj->lsource_thd_high)/dynamic_intt_min_unit );
						c = ((a * obj->last_ratio)/dynamic_intt_min_unit) + b;
						if(c > obj->c_gain_h)
							c_gain = obj->c_gain_h;
						else if (c < obj->c_gain_l)
							c_gain = obj->c_gain_l;
						else
							c_gain = c;
					}
				} else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SLOPE) {
					if (luxratio >= obj->lsource_thd_high) {
						
						c_gain = obj->c_gain_h;
					} else if (luxratio <= obj->lsource_thd_low) {
						
						c_gain = obj->c_gain_l;
					} else {
						
						int a = 0, b = 0, c = 0;
						a = (obj->c_gain_h - obj->c_gain_l) * dynamic_intt_min_unit / (obj->lsource_thd_high - obj->lsource_thd_low);
						b = (obj->c_gain_h) - ((a * obj->lsource_thd_high)/dynamic_intt_min_unit );
						c = ((a * luxratio)/dynamic_intt_min_unit) + b;
						if(c > obj->c_gain_h)
							c_gain = obj->c_gain_h;
						else if (c < obj->c_gain_l)
							c_gain = obj->c_gain_l;
						else
							c_gain = c;
					}
				}
				LOG_INFO("[%s]:ch0=%d, ch1=%d, c_gain=%d, obj->ratio=%d, obj->last_ratio=%d \r\n\n",
								__func__,epl_sensor.als.data.channels[0], als,
						 		c_gain, obj->ratio, obj->last_ratio);
			} else {
				LOG_INFO("[%s]: ALS saturation(%d) \r\n", __func__, (epl_sensor.als.saturation >> 5));
			}

		}

		if(ALS_DEBUG_FLAG) {
			LOG_INFO("[%s]: dynamic_intt_idx=%d, als_dynamic_intt_value=%d, dynamic_intt_gain=%d, als=%d \r\n",
					__func__, dynamic_intt_idx, als_dynamic_intt_value[dynamic_intt_idx],
					 als_dynamic_intt_gain[dynamic_intt_idx], als);
		}

		if(als > dynamic_intt_high_thr)	{
			if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
				als = dynamic_intt_high_thr;
				lux_tmp = raw_convert_to_lux(als);
				if(ALS_DEBUG_FLAG)
					LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
			} else {
				change_flag = true;
				als  = dynamic_intt_high_thr;
				lux_tmp = raw_convert_to_lux(als);
				dynamic_intt_idx++;
				if(ALS_DEBUG_FLAG) {
					LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n",
								 dynamic_intt_idx, als);
				}
			}
		} else if(als < dynamic_intt_low_thr) {
			if(dynamic_intt_idx == 0) {
				
				lux_tmp = raw_convert_to_lux(als);
				if(ALS_DEBUG_FLAG)
					LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MIN_LUX\r\n");
			} else {
				change_flag = true;
				als  = dynamic_intt_low_thr;
				lux_tmp = raw_convert_to_lux(als);
				dynamic_intt_idx--;
				if(ALS_DEBUG_FLAG) {
					LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n",
								 dynamic_intt_idx, als);
				}
			}
		} else
			lux_tmp = raw_convert_to_lux(als);

		now_lux = lux_tmp;
		dynamic_intt_lux = now_lux/dynamic_intt_min_unit;
#if !HTC_ALS
		epl_sensor_report_lux(dynamic_intt_lux);
#endif
		if(change_flag == true) {
			epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
			epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
			dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
			dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
			epl_sensor_update_mode(obj->client);
			
		}
#if !HTC_ALS
		return dynamic_intt_lux;
#else
        if(dynamic_intt_idx == 1)
        {
            

            if (obj->ls_calibrate)
		        als_step = epl_sensor.als.dyn_intt_raw;
    		else {
#if 0
    			als_step = epl_sensor.als.dyn_intt_raw * obj->als_gadc / obj->als_kadc;
#else
                als_step = epl_sensor.als.dyn_intt_raw * obj->als_gadc / obj->als_kadc_high_lux;
#endif
    			if( (als_step*obj->als_kadc) < (als*obj->als_gadc) ) {
    	                als_step = als_step + 1;
                }

    			if (als_step > 0xFFFF)
    				als_step = 0xFFFF;

			if(ALS_DEBUG_FLAG) {
	    			LOG_INFO("[%s]: als high adc = %d, als_step = %d \r\n",
					 __func__, epl_sensor.als.dyn_intt_raw, als_step);
			}
    		}

            for (i = 0; i < 10; i++) {
        		if (als_step <= (*(obj->adc_table_high_lux + i))) {
        			level = i;
        			if (*(obj->adc_table_high_lux + i))
        				break;
        		}
        		if (i == 9) {			
        			level = i;
        			break;
    		    }
	        }
        }
        else
        {
		if(ALS_DEBUG_FLAG) {
	            LOG_INFO("obj->als_gadc=%d,obj->als_kadc=%d \r\n", obj->als_gadc, obj->als_kadc);
		}
            
            if (obj->ls_calibrate)
			    als_step = epl_sensor.als.dyn_intt_raw;
    		else {
#if 0
    			als_step = epl_sensor.als.dyn_intt_raw * obj->als_gadc / obj->als_kadc;
#else
                als_step = epl_sensor.als.dyn_intt_raw * obj->als_gadc / obj->als_kadc_high_lux;
#endif
    			if( (als_step*obj->als_kadc) < (als*obj->als_gadc) ) {
    	                	als_step = als_step + 1;
                	}

    			if (als_step > 0xFFFF)
    				als_step = 0xFFFF;

			if(ALS_DEBUG_FLAG) {
	    			LOG_INFO("[%s]: als adc = %d, als_step = %d \r\n",
					 __func__, epl_sensor.als.dyn_intt_raw, als_step);
			}
    		}

            for (i = 0; i < 10; i++) {
        		if (als_step <= (*(obj->adc_table + i))) {
        			level = i;
        			if (*(obj->adc_table + i))
        				break;
        		}
        		if (i == 9) {			
        			level = i;
        			break;
    		    }
	        }
        }

	if(ALS_DEBUG_FLAG) {
        	LOG_INFO("[%s]: level=%d \r\n", __func__, level);
	}

        if(change_flag == false){
            obj->current_level = level;
            obj->current_adc = als_step;
        }

        change_flag = false;
        return level;
#endif
	break;
#endif

#if HTC_ALS
	case CMC_BIT_LEVEL:

		if(ALS_DEBUG_FLAG) {
			LOG_INFO("obj->als_gadc=%d, obj->als_kadc=%d \r\n",
						 obj->als_gadc, obj->als_kadc);
		}

		if (obj->ls_calibrate)
			als_step = als;
		else {
#if 0
            als_step = ((uint32_t)als) * obj->als_gadc / obj->als_kadc;
#else
			als_step = ((uint32_t)als) * obj->als_gadc / obj->als_kadc_high_lux;
#endif
			if( (als_step*obj->als_kadc) < (als*obj->als_gadc) ) {
        	                als_step = als_step + 1;
	                }

			if (als_step > 0xFFFF)
				als_step = 0xFFFF;

			if(ALS_DEBUG_FLAG) {
				LOG_INFO("[%s]: als adc = %d, als_step = %d \r\n", __func__, als, als_step);
			}
		}
#if 0
		for (i = 0; i < 10; i++) {
			if (als_step <= (*(obj->adc_table + i))) {
				level = i;
				if (*(obj->adc_table + i))
					break;
			}
			if (i == 9) {			
				level = i;
				break;
			}
		}
#else
        for (i = 0; i < 10; i++) {
			if (als_step <= (*(obj->adc_table_high_lux + i))) {
				level = i;
				if (*(obj->adc_table_high_lux + i))
					break;
			}
			if (i == 9) {			
				level = i;
				break;
			}
		}
#endif
		if(ALS_DEBUG_FLAG) {
			LOG_INFO("[%s]: level=%d \r\n", __func__, level);
		}
		obj->current_level = level;
		obj->current_adc = als_step;
		return level;
	break;
#endif
	}
	return 0;
}

int epl_sensor_read_als(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);

	if(client == NULL)
	{
		LOG_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	epl_sensor_I2C_Read(epld->client, 0x13, 4);

	epl_sensor.als.data.channels[0] = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
	epl_sensor.als.data.channels[1] = (gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2];

	if(ALS_DEBUG_FLAG) {
		LOG_INFO("read als channel 0 = %d\n", epl_sensor.als.data.channels[0]);
		LOG_INFO("read als channel 1 = %d\n", epl_sensor.als.data.channels[1]);
	}

    if(epl_sensor.wait == EPL_WAIT_SINGLE)
	    epl_sensor_I2C_Write(epld->client,0x11,  epl_sensor.power | epl_sensor.reset);
	return 0;
}


static void epl_sensor_report_ps_status(void)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int status;

	status = epl_sensor.ps.compare_low >> 3;

	LOG_INFO("[PS] proximity interrupt: %s, ps_adc=%d, High thd= %d, calibration %d, value= %d\n",
		 status ? "FAR" : "NEAR", epl_sensor.ps.data.data, dynk_thd_high, psensor_cali, epl_sensor.ps.compare_low >> 3);

	if(status == 0 && time_before(epld->j_end, (epld->j_start + NEAR_DELAY_TIME))) {
		epld->ps_pocket_mode = 1;
		LOG_INFO("[PS] Ignore NEAR event\n");
	} else {
		input_report_abs(epld->ps_input_dev, ABS_DISTANCE, status);
		input_sync(epld->ps_input_dev);
	}
}

int epl_sensor_read_ps(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = i2c_get_clientdata(client);
	if(client == NULL) {
		LOG_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	epl_sensor_I2C_Read(epld->client, 0x1C, 4);

	epl_sensor.ps.data.ir_data = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
#if PS_RAW_8BIT
	epl_sensor.ps.data.data = gRawData.raw_bytes[3];
#else
	epl_sensor.ps.data.data = (gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2];
#endif

	if(PS_DEBUG_FLAG) {
		LOG_INFO("[%s] data = %d\n", __FUNCTION__, epl_sensor.ps.data.data);
		LOG_INFO("[%s] ir data = %d\n", __FUNCTION__, epl_sensor.ps.data.ir_data);
	}

	if(epl_sensor.wait == EPL_WAIT_SINGLE)
		epl_sensor_I2C_Write(epld->client, 0x11, epl_sensor.power | epl_sensor.reset);

	return 0;
}

int epl_sensor_read_ps_status(struct i2c_client *client)
{
	u8 buf;

	if(client == NULL)
	{
		LOG_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	epl_sensor_I2C_Read(client, 0x1b, 1);

	buf = gRawData.raw_bytes[0];

	epl_sensor.ps.saturation = (buf & 0x20);
	epl_sensor.ps.compare_high = (buf & 0x10);
	epl_sensor.ps.compare_low = (buf & 0x08);
	epl_sensor.ps.interrupt_flag = (buf & 0x04);
	epl_sensor.ps.compare_reset = (buf & 0x02);
	epl_sensor.ps.lock= (buf & 0x01);
#if HTC_ATTR
	p_status = epl_sensor.ps.compare_low >> 3;
#endif
	if(PS_DEBUG_FLAG) {
		LOG_INFO("ps: ~~~~ PS ~~~~~ \n");
		LOG_INFO("ps: buf = 0x%x\n", buf);
		LOG_INFO("ps: sat = 0x%x\n", epl_sensor.ps.saturation);
		LOG_INFO("ps: cmp h = 0x%x, l = 0x%x\n", epl_sensor.ps.compare_high, epl_sensor.ps.compare_low);
		LOG_INFO("ps: int_flag = 0x%x\n",epl_sensor.ps.interrupt_flag);
		LOG_INFO("ps: cmp_rstn = 0x%x, lock = %x\n", epl_sensor.ps.compare_reset, epl_sensor.ps.lock);
	}
	return 0;
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	struct i2c_client *client = epld->client;
	uint8_t high_msb ,high_lsb, low_msb, low_lsb;


	high_msb   = (uint8_t) (high_thd >> 8);
	high_lsb   = (uint8_t) (high_thd & 0x00ff);
	low_msb    = (uint8_t) (low_thd >> 8);
	low_lsb    = (uint8_t) (low_thd & 0x00ff);


#if PS_RAW_8BIT
	LOG_INFO("%s: PS_RAW_8BIT low_thd = %d, high_thd = %d \n",
				__FUNCTION__, low_thd, high_thd);
	epl_sensor_I2C_Write(client, 0x0C, 0);
	epl_sensor_I2C_Write(client, 0x0D, low_lsb);
	epl_sensor_I2C_Write(client, 0x0E, 0);
	epl_sensor_I2C_Write(client, 0x0F, high_lsb);
#else
	LOG_INFO("%s: low_thd = %d, high_thd = %d \n",
				__FUNCTION__, low_thd, high_thd);
	epl_sensor_I2C_Write(client, 0x0C, low_lsb);
	epl_sensor_I2C_Write(client, 0x0D, low_msb);
	epl_sensor_I2C_Write(client, 0x0E, high_lsb);
	epl_sensor_I2C_Write(client, 0x0F, high_msb);
#endif
	return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	struct i2c_client *client = epld->client;
	uint8_t high_msb, high_lsb, low_msb, low_lsb;

	high_msb = (uint8_t) (high_thd >> 8);
	high_lsb = (uint8_t) (high_thd & 0x00ff);
	low_msb  = (uint8_t) (low_thd >> 8);
	low_lsb  = (uint8_t) (low_thd & 0x00ff);

	epl_sensor_I2C_Write(client, 0x08, low_lsb);
	epl_sensor_I2C_Write(client, 0x09, low_msb);
	epl_sensor_I2C_Write(client, 0x0A, high_lsb);
	epl_sensor_I2C_Write(client, 0x0B, high_msb);

	LOG_INFO("%s: low_thd = %d, high_thd = %d \n",
				__FUNCTION__, low_thd, high_thd);
	return 0;
}

int epl_sensor_read_als_status(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	u8 buf;

	if(client == NULL)
	{
		LOG_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	epl_sensor_I2C_Read(epld->client, 0x12, 1);

	buf = gRawData.raw_bytes[0];

	epl_sensor.als.saturation = (buf & 0x20);
	epl_sensor.als.compare_high = (buf & 0x10);
	epl_sensor.als.compare_low = (buf & 0x08);
	epl_sensor.als.interrupt_flag = (buf & 0x04);
	epl_sensor.als.compare_reset = (buf & 0x02);
	epl_sensor.als.lock= (buf & 0x01);

	if(ALS_DEBUG_FLAG) {
		LOG_INFO("als: ~~~~ ALS ~~~~~ \n");
		LOG_INFO("als: buf = 0x%x\n", buf);
		LOG_INFO("als: sat = 0x%x\n", epl_sensor.als.saturation);
		LOG_INFO("als: cmp h = 0x%x, l = 0x%x\n", epl_sensor.als.compare_high, epl_sensor.als.compare_low);
		LOG_INFO("als: int_flag = 0x%x\n",epl_sensor.als.interrupt_flag);
		LOG_INFO("als: cmp_rstn = 0x%x, lock = 0x%x\n", epl_sensor.als.compare_reset, epl_sensor.als.lock);
	}

	return 0;
}

static int write_factory_calibration(struct epl_sensor_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;

	mm_segment_t fs;
	loff_t pos;

	LOG_FUN();
    pos = 0;

	fp_cal = filp_open(ps_cal_file, O_CREAT|O_RDWR|O_TRUNC, 0755);
	if (IS_ERR(fp_cal))
	{
		LOG_ERR("[ELAN]create file_h error\n");
		return -1;
	}

    fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);

    filp_close(fp_cal, NULL);

	set_fs(fs);

	return 0;
}

static bool read_factory_calibration(void)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	char buffer[100]= {0};
	if(epl_sensor.ps.factory.calibration_enable && !epl_sensor.ps.factory.calibrated)
	{
		fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);

		if (IS_ERR(fp))
		{
			LOG_ERR("NO PS calibration file(%d)\n", (int)IS_ERR(fp));
			epl_sensor.ps.factory.calibration_enable =  false;
		}
		else
		{
		    int ps_cancelation = 0, ps_hthr = 0, ps_lthr = 0;
			pos = 0;
			fs = get_fs();
			set_fs(KERNEL_DS);
			vfs_read(fp, buffer, sizeof(buffer), &pos);
			filp_close(fp, NULL);

			sscanf(buffer, "%d,%d,%d", &ps_cancelation, &ps_hthr, &ps_lthr);
			epl_sensor.ps.factory.cancelation = ps_cancelation;
			epl_sensor.ps.factory.high_threshold = ps_hthr;
			epl_sensor.ps.factory.low_threshold = ps_lthr;
			set_fs(fs);

			epl_sensor.ps.high_threshold = epl_sensor.ps.factory.high_threshold;
			epl_sensor.ps.low_threshold = epl_sensor.ps.factory.low_threshold;
			epl_sensor.ps.cancelation = epl_sensor.ps.factory.cancelation;
		}

		epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
		epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
		set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

		epl_sensor.ps.factory.calibrated = true;
	}

	if(epl_sensor.als.factory.calibration_enable && !epl_sensor.als.factory.calibrated)
	{
		fp = filp_open(als_cal_file, O_RDONLY, S_IRUSR);
		if (IS_ERR(fp))
		{
			LOG_ERR("NO ALS calibration file(%d)\n", (int)IS_ERR(fp));
			epl_sensor.als.factory.calibration_enable =  false;
		}
		else
		{
		    int als_lux_per_count = 0;
			pos = 0;
			fs = get_fs();
			set_fs(KERNEL_DS);
			vfs_read(fp, buffer, sizeof(buffer), &pos);
			filp_close(fp, NULL);

			sscanf(buffer, "%d", &als_lux_per_count);
			epl_sensor.als.factory.lux_per_count = als_lux_per_count;
			set_fs(fs);
		}
		epl_sensor.als.factory.calibrated = true;
	}
	return true;
}

static int epl_run_ps_calibration(struct epl_sensor_priv *epl_data)
{
    struct epl_sensor_priv *epld = epl_data;
    bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5, i;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0, ps_cal_len = 0;
    char ps_calibration[20];


    if(PS_MAX_XTALK < 0)
    {
        LOG_ERR("[%s]:Failed: PS_MAX_XTALK < 0 \r\n", __func__);
        return -EINVAL;
    }

    if(enable_ps == 0)
    {
        epld->enable_pflag = 1;
        epl_sensor_update_mode(epld->client);
    }

    polling_flag = false;

    for(i=0; i<count; i++)
    {
        msleep(50);
    	switch(epl_sensor.mode)
    	{
    		case EPL_MODE_PS:
    		case EPL_MODE_ALS_PS:
                if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
    		 	    epl_sensor_read_ps(epld->client);
    			ch1 = epl_sensor.ps.data.data;
		    break;
    	}

    	ch1_all = ch1_all + ch1;
    	if(epl_sensor.wait == EPL_WAIT_SINGLE)
    		epl_sensor_I2C_Write(epld->client,0x11, epl_sensor.power | epl_sensor.reset);
    }


    ch1 = (u16)(ch1_all/count);

    if(ch1 > PS_MAX_XTALK)
    {
        LOG_ERR("[%s]:Failed: ch1 > max_xtalk(%d) \r\n", __func__, ch1);
        return -EINVAL;
    }
    else if(ch1 <= 0)
    {
        LOG_ERR("[%s]:Failed: ch1 = 0\r\n", __func__);
        return -EINVAL;
    }

    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d", ps_cancelation, ps_hthr, ps_lthr);

    if(write_factory_calibration(epld, ps_calibration, ps_cal_len) < 0)
    {
        LOG_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }

    epl_sensor.ps.low_threshold = ps_lthr;
	epl_sensor.ps.high_threshold = ps_hthr;
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	LOG_INFO("[%s]: ch1 = %d\n", __func__, ch1);

    polling_flag = true;
    epl_sensor_restart_polling();
	return ch1;
}


static void write_global_variable(struct i2c_client *client)
{
	u8 buf;

	
	buf = epl_sensor.reset | epl_sensor.power;
	epl_sensor_I2C_Write(client,0x11, buf);

	
	epl_sensor_I2C_Read(client, 0x20, 2);
	epl_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;

	
	epl_sensor_I2C_Write(client, 0xfd, 0x8e);
	epl_sensor_I2C_Write(client, 0xfe, 0x22);
	epl_sensor_I2C_Write(client, 0xfe, 0x02);
	epl_sensor_I2C_Write(client, 0xfd, 0x00);

	epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);

	
	buf = epl_sensor.ps.integration_time | epl_sensor.ps.gain;
	epl_sensor_I2C_Write(client,0x03, buf);

	buf = epl_sensor.ps.adc | epl_sensor.ps.cycle;
	epl_sensor_I2C_Write(client,0x04, buf);

	buf = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
	epl_sensor_I2C_Write(client,0x05, buf);

	buf = epl_sensor.interrupt_control | epl_sensor.ps.persist | epl_sensor.ps.interrupt_type;
	epl_sensor_I2C_Write(client,0x06, buf);

	buf = epl_sensor.ps.compare_reset | epl_sensor.ps.lock;
	epl_sensor_I2C_Write(client,0x1b, buf);

	epl_sensor_I2C_Write(client,0x22, (u8)(epl_sensor.ps.cancelation & 0xff));
	epl_sensor_I2C_Write(client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff0) >> 8));
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	
	buf = epl_sensor.als.integration_time | epl_sensor.als.gain;
	epl_sensor_I2C_Write(client,0x01, buf);

	buf = epl_sensor.als.adc | epl_sensor.als.cycle;
	epl_sensor_I2C_Write(client,0x02, buf);

	buf = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
	epl_sensor_I2C_Write(client,0x07, buf);

	buf = epl_sensor.als.compare_reset | epl_sensor.als.lock;
	epl_sensor_I2C_Write(client,0x12, buf);

	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);

	
	buf = epl_sensor.wait | epl_sensor.mode;
	epl_sensor_I2C_Write(client,0x00, buf);
}

static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{

    
	switch((ps_polling << 1) | als_polling)
	{
		case 0: 
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 1: 
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 2: 
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;

		case 3: 
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
	}
}

static void initial_global_variable(struct i2c_client *client, struct epl_sensor_priv *obj)
{
	
	epl_sensor.power   = EPL_POWER_ON;
	epl_sensor.reset   = EPL_RESETN_RUN;
	epl_sensor.mode    = EPL_MODE_IDLE;
	epl_sensor.wait	   = EPL_WAIT_20_MS;
	epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

	
	epl_sensor.als.polling_mode 		= ALS_POLLING_MODE;
	epl_sensor.als.integration_time 	= EPL_ALS_INTT_64;
	epl_sensor.als.gain 			= EPL_GAIN_MID;
	epl_sensor.als.adc 			= EPL_PSALS_ADC_12;
	epl_sensor.als.cycle 			= EPL_CYCLE_64;
	obj->als_init_cycle             	= EPL_CYCLE_16;
	obj->als_cycle                  	= epl_sensor.als.cycle;
	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
	epl_sensor.als.persist 			= EPL_PERIST_1;
	epl_sensor.als.compare_reset 		= EPL_CMP_RESET;
	epl_sensor.als.lock 			= EPL_UN_LOCK;
	epl_sensor.als.report_type = CMC_BIT_DYN_INT; 
	epl_sensor.als.high_threshold 		= ALS_HIGH_THRESHOLD;
	epl_sensor.als.low_threshold 		= ALS_LOW_THRESHOLD;
	
	epl_sensor.als.factory.calibration_enable = false;
	epl_sensor.als.factory.calibrated	  = false;
	epl_sensor.als.factory.lux_per_count 	  = LUX_PER_COUNT;

#if ALS_DYN_INTT
	epl_sensor.als.lsrc_type 		= CMC_BIT_LSRC_NON;

	if(epl_sensor.als.report_type == CMC_BIT_DYN_INT) {
		dynamic_intt_idx 		= dynamic_intt_init_idx;
		epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
		epl_sensor.als.gain 		= als_dynamic_intt_gain[dynamic_intt_idx];
		dynamic_intt_high_thr 		= als_dynamic_intt_high_thr[dynamic_intt_idx];
		dynamic_intt_low_thr 		= als_dynamic_intt_low_thr[dynamic_intt_idx];
	}

	c_gain			= 21000; 
	obj->lsource_thd_high	= 15000; 
	obj->lsource_thd_low	= 6500;  
	obj->c_gain_h		= 21000; 
	obj->c_gain_l		= 9000;  

#endif
	
	epl_sensor.ps.polling_mode 	= PS_POLLING_MODE;
#if HTC_ATTR
	epl_sensor.ps.cycle 		= obj->ps_duty;
	epl_sensor.ps.persist 		= obj->ps_pers;
	epl_sensor.ps.integration_time 	= obj->ps_it;
	epl_sensor.ps.ir_drive 		= obj->ps_led_current;
#else
	epl_sensor.ps.cycle 		= EPL_CYCLE_64;
	epl_sensor.ps.persist 		= EPL_PERIST_1;
	epl_sensor.ps.integration_time 	= EPL_PS_INTT_80;
	epl_sensor.ps.ir_drive 		= EPL_IR_DRIVE_100;
#endif
	epl_sensor.ps.gain 		= EPL_GAIN_MID;
	epl_sensor.ps.adc 		= EPL_PSALS_ADC_11;
	epl_sensor.ps.ir_on_control 	= EPL_IR_ON_CTRL_ON;
	epl_sensor.ps.ir_mode 		= EPL_IR_MODE_CURRENT;
	epl_sensor.ps.compare_reset 	= EPL_CMP_RESET;
	epl_sensor.ps.lock 		= EPL_UN_LOCK;
	epl_sensor.ps.high_threshold 	= PS_HIGH_THRESHOLD;
	epl_sensor.ps.low_threshold 	= PS_LOW_THRESHOLD;
	epl_sensor.ps.cancelation       = obj->inte_ps_canc;
	
	epl_sensor.ps.factory.calibration_enable = false;
	epl_sensor.ps.factory.calibrated 	 = false;
	epl_sensor.ps.factory.cancelation	 = 0;
#if PS_DYN_K

#if PS_RAW_8BIT
	dynk_min_ps_raw_data = 0xff;
#else
	dynk_min_ps_raw_data = 0xffff;
#endif
	dynk_max_ir_data = 50000;
	obj->ps_th_add = (obj->ps_th_add) ? obj->ps_th_add : TH_ADD;
	obj->ps_threshold_diff = 1;

#if HTC_ATTR
#if PS_RAW_8BIT
	PS_max = 0xff;
#else
	PS_max = 0xffff;
#endif
#endif

#endif
	set_als_ps_intr_type(client, epl_sensor.ps.polling_mode,
					 epl_sensor.als.polling_mode);
	
	write_global_variable(client);
}


#if PS_DYN_K
void epl_sensor_restart_dynk_polling(void)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;

	cancel_delayed_work(&epld->dynk_thd_polling_work);
	if (epld->lp_wq)
		queue_delayed_work(epld->lp_wq, &epld->dynk_thd_polling_work,
				msecs_to_jiffies(2*dynk_polling_delay));
}

void epl_sensor_dynk_thd_polling_work(struct work_struct *work)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;
	bool enable_ps = obj->enable_pflag==1 && obj->ps_suspend==0;

	if(PS_DEBUG_FLAG) {
		bool enable_als = obj->enable_lflag==1 && obj->als_suspend==0;
		LOG_INFO("[%s]:als / ps enable: %d / %d\n", __func__,enable_als, enable_ps);
	}

	obj->j_end = jiffies;
	if (time_after(obj->j_end, (obj->j_start + 3 * HZ))) {
		obj->ps_pocket_mode = 0;
#if 0
	if(obj->ps_pocket_mode | p_irq_status)
		p_status = 0;
	else
		p_status = 1;
#endif

       }
	if(enable_ps == true) {
		if(polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0) {
			mutex_lock(&sensor_mutex);
			epl_sensor_read_ps_status(obj->client);
			epl_sensor_read_ps(obj->client);
#if 1       
			if(epl_sensor.ps.lock == EPL_LOCK) {
				LOG_INFO("[%s]: PS LOCK \r\n", __func__);
				
				epl_sensor_I2C_Write(obj->client, 0x1b, EPL_CMP_RUN | EPL_UN_LOCK);
				
				epl_sensor_I2C_Write(obj->client, 0x12, EPL_CMP_RUN | EPL_UN_LOCK);
				epl_sensor_report_ps_status();
			}
#endif
			mutex_unlock(&sensor_mutex);
		}

		if( (dynk_min_ps_raw_data > epl_sensor.ps.data.data) &&
		    (epl_sensor.ps.saturation == 0) &&
		    (epl_sensor.ps.data.ir_data < dynk_max_ir_data) ) {
			dynk_min_ps_raw_data = epl_sensor.ps.data.data;
			dynk_thd_low 	     = dynk_min_ps_raw_data + obj->ps_th_add;
			dynk_thd_high 	     = dynk_thd_low + obj->ps_threshold_diff;
#if PS_RAW_8BIT
		        if(dynk_thd_low>255)
				dynk_thd_low = 255;
			if(dynk_thd_high>256)
				dynk_thd_high = 256;
#else
	 		if(dynk_thd_low>65534)
				dynk_thd_low = 65534;
			if(dynk_thd_high>65535)
				dynk_thd_high = 65535;
#endif

			if(PS_DEBUG_FLAG) {
				LOG_INFO("[%s]:dyn ps raw = %d, min = %d, ir_data = %d\n",
					__func__, epl_sensor.ps.data.data,
				 	dynk_min_ps_raw_data, epl_sensor.ps.data.ir_data);
			}

			eint_flag = false;
			mutex_lock(&sensor_mutex);
			set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
			mutex_unlock(&sensor_mutex);
			eint_flag = true;

			if(PS_DEBUG_FLAG) {
				LOG_INFO("[%s]:dyn k thre_l = %ld, thre_h = %ld\n", __func__,
						 (long)dynk_thd_low, (long)dynk_thd_high);
			}
		}

		queue_delayed_work(obj->lp_wq, &obj->dynk_thd_polling_work,
					msecs_to_jiffies(dynk_polling_delay));
	}
}
#endif

static int als_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_adc, als_cycle;

    als_intt = als_intt_value[intt>>2];
    als_adc = adc_value[adc>>3];
    als_cycle = cycle_value[cycle];

    if(COMMON_DEBUG_FLAG) {
	LOG_INFO("ALS: INTT=%d, ADC=%d, Cycle=%d \r\n", als_intt, als_adc, als_cycle);
    }

    sensing_us_time = (als_intt + als_adc*2*2) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;

    if(COMMON_DEBUG_FLAG) {
    	LOG_INFO("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
    }

    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int adc, int cycle)
{
	long sensing_us_time;
	int sensing_ms_time;
	int ps_intt, ps_adc, ps_cycle;

	ps_intt = ps_intt_value[intt>>2];
	ps_adc = adc_value[adc>>3];
	ps_cycle = cycle_value[cycle];

	if(COMMON_DEBUG_FLAG) {
		LOG_INFO("PS: INTT=%d, ADC=%d, Cycle=%d \r\n", ps_intt, ps_adc, ps_cycle);
	}

	sensing_us_time = (ps_intt*3 + ps_adc*2*3) * ps_cycle;
	sensing_ms_time = sensing_us_time / 1000;

	if(COMMON_DEBUG_FLAG) {
		LOG_INFO("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
	}

    return (sensing_ms_time + 5);
}

static int epl_sensor_get_wait_time(int ps_time, int als_time)
{
    int wait_idx = 0;
    int wait_time = 0;

    wait_time = als_time - ps_time;
    if(wait_time < 0){
        wait_time = 0;
    }

    if(COMMON_DEBUG_FLAG) {
    	LOG_INFO("[%s]: wait_len = %d \r\n", __func__, wait_len);
    }

    for(wait_idx = 0; wait_idx < wait_len; wait_idx++) {
        if(wait_time < wait_value[wait_idx]) {
	    break;
	}
    }

    if(wait_idx >= wait_len) {
    	wait_idx = wait_len - 1;
    }

    if(COMMON_DEBUG_FLAG) {
	LOG_INFO("[%s]: wait_idx = %d, wait = %dms \r\n", __func__, wait_idx, wait_value[wait_idx]);
    }

    return (wait_idx << 4);
}

void epl_sensor_update_mode(struct i2c_client *client)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int als_time = 0, ps_time = 0;

    bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
    bool enable_als = epld->enable_lflag==1 && epld->als_suspend==0;

    LOG_INFO("[%s]: ++ mode selection =0x%x\n", __func__, enable_ps | (enable_als << 1));
    mutex_lock(&sensor_enable_mutex);

#if 0
    
    epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor_I2C_Write(epld->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

    
    epl_sensor.als.compare_reset = EPL_CMP_RESET;
    epl_sensor.als.lock = EPL_UN_LOCK;
    epl_sensor_I2C_Write(epld->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
#endif

    
    polling_flag = false;
    epl_sensor.ps.interrupt_flag = EPL_INT_CLEAR;
    epl_sensor.als.interrupt_flag = EPL_INT_CLEAR;

    als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

    als_frame_time = als_time;
    ps_frame_time = ps_time;

    
    switch((enable_als << 1) | enable_ps)
    {
        case 0: 
            epl_sensor.mode = EPL_MODE_IDLE;
            break;

        case 1: 
            epl_sensor.mode = EPL_MODE_PS;
            break;

        case 2: 
            epl_sensor.mode = EPL_MODE_ALS;
            break;

        case 3: 
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    
    
    
    
    
    epl_sensor_I2C_Write(epld->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);
    
    read_factory_calibration();

    LOG_INFO("[%s]: ALS cycle = %d \r\n", __func__, epl_sensor.als.cycle);
    epl_sensor_I2C_Write(epld->client, 0x02, epl_sensor.als.adc |
					     epl_sensor.als.cycle);
    set_als_ps_intr_type(epld->client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    epl_sensor_I2C_Write(epld->client, 0x06, epl_sensor.interrupt_control |
					     epl_sensor.ps.persist |
					     epl_sensor.ps.interrupt_type);
    epl_sensor_I2C_Write(epld->client, 0x07, epl_sensor.als.interrupt_channel_select |
					     epl_sensor.als.persist |
					     epl_sensor.als.interrupt_type);

#if ALS_DYN_INTT
    epl_sensor_I2C_Write(client, 0x01, epl_sensor.als.integration_time |
					       epl_sensor.als.gain);
#endif

    if(epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 0 && epl_sensor.ps.polling_mode == 0){
        int wait = 0;
        wait = epl_sensor_get_wait_time(ps_time, als_time);
        epl_sensor_I2C_Write(epld->client, 0x00, wait | epl_sensor.mode);
        epl_sensor.wait = wait;
        LOG_INFO("[%s]: epl_sensor.als.polling_mode=%d \r\n", __func__, epl_sensor.als.polling_mode);
    }
    else{
        epl_sensor_I2C_Write(epld->client, 0x00, epl_sensor.wait | epl_sensor.mode);
    }

    
    epl_sensor.als.compare_reset = EPL_CMP_RUN;
    epl_sensor.als.lock = EPL_UN_LOCK;
    epl_sensor_I2C_Write(epld->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

    
    epl_sensor.ps.compare_reset = EPL_CMP_RUN;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor_I2C_Write(epld->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

    
    mutex_unlock(&sensor_enable_mutex);

    if(COMMON_DEBUG_FLAG) {
    	
    	if(enable_ps == 1)
   	{
        	LOG_INFO("[%s] PS:low_thd = %d, high_thd = %d \n",__func__,
			 epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
    	}

    	if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
    	{
        	LOG_INFO("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__,
			 epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    	}

	LOG_INFO("[%s] reg0x00= 0x%x \n", __func__, epl_sensor.wait | epl_sensor.mode);
	LOG_INFO("[%s] reg0x07= 0x%x \n", __func__, epl_sensor.als.interrupt_channel_select |
						    epl_sensor.als.persist |
						    epl_sensor.als.interrupt_type);
	LOG_INFO("[%s] reg0x06= 0x%x \n", __func__, epl_sensor.interrupt_control | epl_sensor.ps.persist |
						    epl_sensor.ps.interrupt_type);
	LOG_INFO("[%s] reg0x11= 0x%x \n", __func__, epl_sensor.power | epl_sensor.reset);
	LOG_INFO("[%s] reg0x12= 0x%x \n", __func__, epl_sensor.als.compare_reset | epl_sensor.als.lock);
	LOG_INFO("[%s] reg0x1b= 0x%x \n", __func__, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
    }

    if(epl_sensor.mode == EPL_MODE_PS && epl_sensor.ps.polling_mode == 1)
    {
        msleep(ps_time);
        LOG_INFO("[%s] PS only(%dms)\r\n", __func__, ps_time);
    }
    else if (epl_sensor.mode == EPL_MODE_ALS && epl_sensor.als.polling_mode==1)
    {
        msleep(als_time);
        LOG_INFO("[%s] ALS only(%dms)\r\n", __func__, als_time);
    }
    else if (epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 1)
    {
        msleep(ps_time+als_time+wait_value[epl_sensor.wait>>4]);
        LOG_INFO("[%s] PS+ALS(%dms)\r\n", __func__, ps_time+als_time+wait_value[epl_sensor.wait>>4]);
    }

    if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
    {
       
        epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(epld->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
    }

    if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
    {
        
               epl_sensor.als.compare_reset = EPL_CMP_RUN;
       epl_sensor.als.lock = EPL_UN_LOCK;
       epl_sensor_I2C_Write(epld->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
    }

    if((enable_als==1 && epl_sensor.als.polling_mode==1) || (enable_ps==1 && epl_sensor.ps.polling_mode==1))
    {
        epl_sensor_restart_polling();
    }

#if PS_DYN_K
    if(enable_ps == 1)
    {
        epl_sensor_restart_dynk_polling();
    }
#endif
#if HTC_ATTR
    p_status = 9;
#endif
    polling_flag = true;

    LOG_INFO("[%s]: --\n", __func__);
}

static void epl_sensor_polling_work(struct work_struct *work)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	struct i2c_client *client = epld->client;

	bool enable_ps = epld->enable_pflag == 1 && epld->ps_suspend == 0;
	bool enable_als = epld->enable_lflag == 1 && epld->als_suspend == 0;

	if(COMMON_DEBUG_FLAG) {
		LOG_INFO("enable_pflag = %d, enable_lflag = %d \n", enable_ps, enable_als);
	}
	if(polling_flag == false && eint_flag == false) {
		LOG_INFO("[%s]: epl_sensor_update_mode(%d) or eint_work(%d)"
			 " is running !\r\n", __func__, polling_flag, eint_flag);
        	return;
    	}

    	cancel_delayed_work(&epld->polling_work);

	if( (enable_als && epl_sensor.als.polling_mode == 1) ||
	    (enable_ps && epl_sensor.ps.polling_mode == 1) ) {
		if (epld->lp_wq)
			queue_delayed_work(epld->lp_wq, &epld->polling_work,
						 msecs_to_jiffies(polling_time));
	}

	if(enable_als && epl_sensor.als.polling_mode == 1) {
		int report_lux = 0;
		if(polling_flag == true && eint_flag == true) {
			mutex_lock(&sensor_mutex);
        		
			epl_sensor_read_als(client);
			mutex_unlock(&sensor_mutex);
			report_lux = epl_sensor_get_als_value(epld, epl_sensor.als.data.channels[1]);
			if(epl_sensor.als.report_type != CMC_BIT_DYN_INT) {
		            epl_sensor_report_lux(report_lux);
        		}
    		}
	}

	if(enable_ps && epl_sensor.ps.polling_mode == 1) {
		if(polling_flag == true && eint_flag == true) {
			mutex_lock(&sensor_mutex);
			epl_sensor_read_ps_status(client);
			epl_sensor_read_ps(client);
			mutex_unlock(&sensor_mutex);
			epl_sensor_report_ps_status();
		}
	}

	if(enable_als==false && enable_ps==false) {
		cancel_delayed_work(&epld->polling_work);
		LOG_INFO("disable sensor\n");
	}
}

static irqreturn_t epl_sensor_eint_func(int irqNo, void *handle)
{
	struct epl_sensor_priv *epld = (struct epl_sensor_priv*)handle;
	disable_irq_nosync(epld->irq);

	if (epld->lp_wq)
		queue_work(epld->lp_wq, &epl_sensor_irq_work);

	return IRQ_HANDLED;
}
static void epl_sensor_intr_als_report_lux(void)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;

	int report_lux = 0;
	int old_dynamic_intt_idx = 0;
	int als_time = 0, ps_time = 0;

	epl_sensor_I2C_Write(epld->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);
	epl_sensor_read_als(epld->client);
	old_dynamic_intt_idx = dynamic_intt_idx;

#if 1 
    epl_sensor.als.cycle = epld->als_cycle;
    als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);
    epl_sensor.wait = epl_sensor_get_wait_time(ps_time, als_time);

    epl_sensor_I2C_Write(epld->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
#endif

	report_lux = epl_sensor_get_als_value(epld, epl_sensor.als.data.channels[1]);

	if(epl_sensor.als.report_type == CMC_BIT_DYN_INT) {
		if(old_dynamic_intt_idx == dynamic_intt_idx) {
			if ((report_lux == 0) || (epl_sensor.als.data.channels[1] == 0))
				LOG_INFO("[LS] light interrupt: ADC=0x%03X, Level=%d, INTT=%d, l_thd = 0x0, h_thd = 0x%x, calibration %d, MODE=0x%x \n",
				         epl_sensor.als.data.channels[1], report_lux, als_dynamic_intt_value[dynamic_intt_idx],
					 *(epld->cali_table + report_lux), lightsensor_cali, epl_sensor.mode);
			else
				LOG_INFO("[LS] light interrupt: ADC=0x%03X, Level=%d, INTT=%d, l_thd = 0x%x, h_thd = 0x%x, calibration %d, MODE=0x%x \n",
					 epl_sensor.als.data.channels[1], report_lux, als_dynamic_intt_value[dynamic_intt_idx],
					 *(epld->cali_table + (report_lux - 1)) + 1, *(epld->cali_table + report_lux), lightsensor_cali, epl_sensor.mode);

				epl_sensor_report_lux(report_lux);
		} else {
			if ((report_lux == 0) || (epl_sensor.als.data.channels[1] == 0)){
			        LOG_INFO("[LS] light interrupt: ADC=0x%03X, Level=%d(Layer changed), INTT=%d, l_thd = 0x0, h_thd = 0x%x, calibration %d, MODE=0x%x \n",
        			         epl_sensor.als.data.channels[1], report_lux, als_dynamic_intt_value[dynamic_intt_idx],
			                 *(epld->cali_table + report_lux), lightsensor_cali, epl_sensor.mode);

			}else
			        LOG_INFO("[LS] light interrupt: ADC=0x%03X, Level=%d(Layer changed), INTT=%d, l_thd = 0x%x, h_thd = 0x%x, calibration %d, MODE=0x%x \n",
			                epl_sensor.als.data.channels[1], report_lux, als_dynamic_intt_value[dynamic_intt_idx],
			                *(epld->cali_table + (report_lux - 1)) + 1, *(epld->cali_table + report_lux), lightsensor_cali, epl_sensor.mode);

			epl_sensor_report_lux(report_lux);
		}
	} else {
        	epl_sensor_report_lux(report_lux);
	}

	epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

#if HTC_ALS

    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        if(dynamic_intt_idx == 1)
        {
            epl_sensor.als.low_threshold = *(epld->cali_table_high_lux + (report_lux - 1)) + 1;
            epl_sensor.als.high_threshold = *(epld->cali_table_high_lux + report_lux);

            if(epl_sensor.als.low_threshold == 0)
                epl_sensor.als.low_threshold = 1;
        }
        else
        {
            if ((report_lux == 0) || (epl_sensor.als.data.channels[1] == 0)){
                epl_sensor.als.low_threshold = 0;
            }else{
                epl_sensor.als.low_threshold = *(epld->cali_table + (report_lux - 1)) + 1;
            }
            epl_sensor.als.high_threshold = *(epld->cali_table + report_lux);

	    if(ALS_DEBUG_FLAG) {
            	LOG_INFO("[%s]: ALS thd (%d/%d) \r\n", __func__,
				 epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
	    }

            
        if(epl_sensor.als.high_threshold <= (0xffff - als_thd_add))
            epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + als_thd_add;
        else
            epl_sensor.als.high_threshold = 0xffff;

            if(epl_sensor.als.low_threshold < 0)
                epl_sensor.als.low_threshold = 0;
            if(epl_sensor.als.high_threshold > 0xffff)
                epl_sensor.als.high_threshold = 65535;

	    if(ALS_DEBUG_FLAG) {
            	LOG_INFO("[%s]: ALS thd(changed) (%d/%d) \r\n", __func__,
				 epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
	    }

            if(epl_sensor.als.high_threshold == 0xffff)
                epl_sensor.als.high_threshold = 65534;
        }
    }
    else
    {
#if 0
        epl_sensor.als.low_threshold = *(epld->cali_table + (report_lux - 1)) + 1;
        epl_sensor.als.high_threshold = *(epld->cali_table + report_lux);
#else
        epl_sensor.als.low_threshold = *(epld->cali_table_high_lux + (report_lux - 1)) + 1;
        epl_sensor.als.high_threshold = *(epld->cali_table_high_lux + report_lux);
#endif
    }


	set_lsensor_intr_threshold(((report_lux == 0) || (epl_sensor.als.data.channels[1] == 0)) ? 0 :
				epl_sensor.als.low_threshold, epl_sensor.als.high_threshold );

	epl_sensor_I2C_Write(epld->client, 0x00, epl_sensor.wait | epl_sensor.mode);
#else
	
	if(epl_sensor.als.compare_high >> 4)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;

		if (epl_sensor.als.high_threshold > 60000)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold -250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;
		}
	}
	if(epl_sensor.als.compare_low>> 3)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold -250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;

		if (epl_sensor.als.high_threshold < 250)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;
		}
	}

	if(epl_sensor.als.high_threshold < epl_sensor.als.low_threshold)
	{
	    LOG_INFO("[%s]:recover default setting \r\n", __FUNCTION__);
	    epl_sensor.als.high_threshold = ALS_HIGH_THRESHOLD;
	    epl_sensor.als.low_threshold = ALS_LOW_THRESHOLD;
	}

	
	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
#endif
}
static void epl_sensor_eint_work(struct work_struct *work)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int  eint_debug_flag = 0;

	eint_flag = false;
	mutex_lock(&sensor_mutex);

	LOG_INFO("epl_sensor_eint_work\n");

	epl_sensor_read_ps_status(epld->client);
	eint_debug_flag |= epl_sensor.ps.interrupt_flag;

	if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER) {

		epl_sensor_read_ps(epld->client);
		wake_lock_timeout(&(epld->ps_wake_lock), 2*HZ);
		epl_sensor_report_ps_status();

		
		epl_sensor.ps.compare_reset = EPL_CMP_RUN;
		epl_sensor.ps.lock = EPL_UN_LOCK;
		epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
	}

	epl_sensor_read_als_status(epld->client);
	eint_debug_flag |= epl_sensor.als.interrupt_flag;

	if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER) {
		epl_sensor_intr_als_report_lux();

		
		epl_sensor.als.compare_reset = EPL_CMP_RUN;
		epl_sensor.als.lock = EPL_UN_LOCK;
		epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
	}

	
	if(!(eint_debug_flag & EPL_INT_TRIGGER)) {

		LOG_INFO("[%s]: unknown interrupt-->mode= %d \r\n", __func__, epl_sensor.mode);
		LOG_INFO("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x00));
		LOG_INFO("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x01));
		LOG_INFO("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x02));
		LOG_INFO("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x03));
		LOG_INFO("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x04));
		LOG_INFO("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x05));
		LOG_INFO("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x06));
		LOG_INFO("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x07));
		LOG_INFO("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x11));
		LOG_INFO("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x12));
		LOG_INFO("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x1B));
		LOG_INFO("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x20));
		LOG_INFO("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x21));
		LOG_INFO("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x24));
		LOG_INFO("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(epld->client, 0x25));

                epl_sensor.ps.compare_reset = EPL_CMP_RUN;
                epl_sensor.ps.lock = EPL_UN_LOCK;
                epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
                epl_sensor.als.compare_reset = EPL_CMP_RUN;
                epl_sensor.als.lock = EPL_UN_LOCK;
                epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
	}

	enable_irq(epld->irq);
	mutex_unlock(&sensor_mutex);
	eint_flag = true;
}

static int epl_sensor_setup_interrupt(struct epl_sensor_priv *epld)
{
	struct i2c_client *client = epld->client;
	unsigned int irq_gpio;
	unsigned int irq_gpio_flags;
	struct device_node *np = client->dev.of_node;
	int err = 0;
	msleep(5);

	irq_gpio = of_get_named_gpio_flags(np, "epl88051,irq-gpio", 0, &irq_gpio_flags);

	epld->intr_pin = irq_gpio;
	if (epld->intr_pin < 0) {
		goto initial_fail;
    	}

	if (gpio_is_valid(epld->intr_pin)) {
		err = gpio_request(epld->intr_pin, "epl_irq_gpio");
		if (err) {
			LOG_ERR( "irq gpio request failed");
			goto initial_fail;
		}

		err = gpio_direction_input(epld->intr_pin);
		if (err) {
		        LOG_ERR("set_direction for irq gpio failed\n");
		        goto initial_fail;
		}

		err = epl88051_pinctrl_init(epld);
		if (err) {
		        LOG_ERR("set pinctrl for irq gpio failed\n");
		        goto initial_fail;
		}
	}

	err = request_irq(epld->irq,epl_sensor_eint_func, IRQF_TRIGGER_LOW,
        			              client->dev.driver->name, epld);
	if(err <0) {
		LOG_ERR("request irq pin %d fail for gpio\n",err);
		goto fail_free_intr_pin;
	}

	return err;

initial_fail:
fail_free_intr_pin:
	gpio_free(epld->intr_pin);
	free_irq(epld->irq, epld);
	return err;
}


static ssize_t epl_sensor_show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{

    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;

    if(!epl_sensor_obj)
    {
        LOG_ERR("epl_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    if(epl_sensor.als.polling_mode == 0)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x08));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x09));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0A));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0B));
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x22));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x23));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));

    return len;

}

static ssize_t epl_sensor_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
    bool enable_als = epld->enable_lflag==1 && epld->als_suspend==0;

    if(!epl_sensor_obj)
    {
        LOG_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", EPL_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", epl_sensor.als.polling_mode, epl_sensor.ps.polling_mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d\n",epl_sensor.wait >> 4, epl_sensor.mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "interrupt control = %d\n", epl_sensor.interrupt_control >> 4);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);


    if(enable_ps)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.ps.integration_time >> 2, epl_sensor.ps.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d, ir drive = %d\n", epl_sensor.ps.adc >> 3, epl_sensor.ps.cycle, epl_sensor.ps.ir_drive);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", epl_sensor.ps.saturation >> 5, epl_sensor.ps.interrupt_flag >> 2);
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
#if PS_DYN_K
        len += snprintf(buf+len, PAGE_SIZE-len, "Dyn thr(L/H) = (%ld/%ld)\n", (long)dynk_thd_low, (long)dynk_thd_high);
#endif
        len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", epl_sensor.ps.data.ir_data, epl_sensor.ps.data.data);
    }
    if(enable_als)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.als.integration_time >> 2, epl_sensor.als.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d\n", epl_sensor.als.adc >> 3, epl_sensor.als.cycle);
#if ALS_DYN_INTT
        if(epl_sensor.als.lsrc_type != CMC_BIT_LSRC_NON)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "lsource_thd_low=%d, lsource_thd_high=%d \n", epld->lsource_thd_low, epld->lsource_thd_high);
            len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d\n", epl_sensor.als.saturation >> 5);

            if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SCALE || epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
            {
                len += snprintf(buf+len, PAGE_SIZE-len, "real_ratio = %d\n", epld->ratio);
                len += snprintf(buf+len, PAGE_SIZE-len, "use_ratio = %d\n", epld->last_ratio);
            }
            else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SLOPE)
            {
                len += snprintf(buf+len, PAGE_SIZE-len, "ratio = %d\n", epld->ratio);
            }
        }
        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "c_gain = %d\n", c_gain);
            len += snprintf(buf+len, PAGE_SIZE-len, "dynamic_intt_lux = %d\n", dynamic_intt_lux);
        }
#endif
        if(epl_sensor.als.polling_mode == 0)
            len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", epl_sensor.als.data.channels[0], epl_sensor.als.data.channels[1]);
    }

    return len;
}

static ssize_t epl_sensor_store_als_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();


    sscanf(buf, "%hu",&mode);
    if(epld->enable_lflag != mode)
    {
#if ALS_DYN_INTT
        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            epl_sensor.als.cycle = epld->als_init_cycle;
            dynamic_intt_idx = dynamic_intt_init_idx;
            epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
            epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
            dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
            dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
        }
#endif
        if(epld->enable_lflag == 1)
        {
            epl_sensor.als.high_threshold = epl_sensor.als.low_threshold;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        }

        epld->enable_lflag = mode;

        epl_sensor_update_mode(epld->client);
    }

    return count;
}

static ssize_t epl_sensor_store_ps_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%hu",&mode);
    if(epld->enable_pflag != mode)
    {
        epld->enable_pflag = mode;
        if(mode)
        {
            wake_lock(&ps_lock);
#if PS_DYN_K
#if PS_RAW_8BIT
            dynk_min_ps_raw_data = 0xff;
#else
            dynk_min_ps_raw_data = 0xffff;
#endif
#endif
        }
        else
        {
#if PS_DYN_K
            cancel_delayed_work(&epld->dynk_thd_polling_work);
#endif
            wake_unlock(&ps_lock);
        }

        epl_sensor_update_mode(epld->client);
    }

    return count;
}

static ssize_t epl_sensor_show_cal_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    u16 ch1 = 0;
    u32 ch1_all=0;
    int count =5;
    int i;
    ssize_t len = 0;
#if !PS_DYN_K
    bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
#endif

    if(!epl_sensor_obj)
    {
        LOG_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }



    for(i=0; i<count; i++)
    {
    	switch(epl_sensor.mode)
    	{
    	    msleep(50);
    		case EPL_MODE_PS:
#if !PS_DYN_K
                if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
                    epl_sensor_read_ps(epld->client);
#endif
    			ch1 = epl_sensor.ps.data.data;
		    break;

    		case EPL_MODE_ALS:
    		    if(epl_sensor.als.polling_mode == 0)
    		 	    epl_sensor_read_als(epld->client);
    			ch1 = epl_sensor.als.data.channels[1];
		    break;
    	}

    	ch1_all = ch1_all + ch1;
    	if(epl_sensor.wait == EPL_WAIT_SINGLE)
    		epl_sensor_I2C_Write(epld->client,0x11,  epl_sensor.power | epl_sensor.reset);
    }

    ch1 = (u16)(ch1_all/count);

    LOG_INFO("cal_raw = %d \r\n" , ch1);

    len += snprintf(buf + len, PAGE_SIZE - len, "%d \r\n", ch1);

    return  len;
}


static ssize_t epl_sensor_store_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int hthr = 0, lthr = 0;
	if(!epld)
	{
	    LOG_ERR("epl_sensor_obj is null!!\n");
	    return 0;
	}

	switch(epl_sensor.mode)
	{
		case EPL_MODE_PS:
			sscanf(buf, "%d,%d", &lthr, &hthr);
			epl_sensor.ps.low_threshold = lthr;
			epl_sensor.ps.high_threshold = hthr;
			set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
		break;

		case EPL_MODE_ALS:
			sscanf(buf, "%d,%d", &lthr, &hthr);
			epl_sensor.als.low_threshold = lthr;
			epl_sensor.als.high_threshold = hthr;
			set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
		break;

	}
	return count;
}

static ssize_t epl_sensor_store_wait_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    sscanf(buf, "%d",&val);

    epl_sensor.wait = (val & 0xf) << 4;

    return count;
}
static ssize_t epl_sensor_store_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;
    LOG_FUN();

    sscanf(buf, "%d", &value);

    value = value & 0x03;

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
            epl_sensor.ps.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
		break;

        case EPL_MODE_ALS: 
            epl_sensor.als.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
		break;
    }

	epl_sensor_update_mode(epld->client);

	return count;
}

static ssize_t epl_sensor_store_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value=0;

	LOG_FUN();

    epld->enable_pflag = 0;
    epld->enable_lflag = 0;

	sscanf(buf, "%d",&value);

	switch (value)
	{
		case 0:
			epl_sensor.mode = EPL_MODE_IDLE;
		break;

		case 1:
            epld->enable_lflag = 1;
			epl_sensor.mode = EPL_MODE_ALS;
		break;

		case 2:
            epld->enable_pflag = 1;
			epl_sensor.mode = EPL_MODE_PS;
		break;

		case 3:
		    epld->enable_lflag = 1;
            epld->enable_pflag = 1;
			epl_sensor.mode = EPL_MODE_ALS_PS;
		break;
	}

       epl_sensor_update_mode(epld->client);

	return count;
}

static ssize_t epl_sensor_store_ir_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d", &value);

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
		switch(value)
			{
				case 0:
			    		epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
				break;

				case 1:
			    		epl_sensor.ps.ir_mode = EPL_IR_MODE_VOLTAGE;
				break;
			}

			epl_sensor_I2C_Write(epld->client,0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
		break;
	}

	epl_sensor_I2C_Write(epld->client,0x00,epl_sensor.wait | epl_sensor.mode);

	return count;
}

static ssize_t epl_sensor_store_ir_contrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;
	uint8_t  data;

	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d",&value);

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
			switch(value)
			{
				case 0:
			    		epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_OFF;
				break;

				case 1:
			    		epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
				break;
			}

			data = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
			LOG_INFO("[%s]: 0x05 = 0x%x\n", __FUNCTION__, data);

			epl_sensor_I2C_Write(epld->client,0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
		break;
	}

	epl_sensor_I2C_Write(epld->client,0x00,epl_sensor.wait | epl_sensor.mode);

	return count;
}

static ssize_t epl_sensor_store_ir_drive(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;
	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d", &value);

	switch(epl_sensor.mode)
	{
		case EPL_MODE_PS:
			epl_sensor.ps.ir_drive = (value & 0x03);
			epl_sensor_I2C_Write(epld->client,0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
		break;
	}

	epl_sensor_I2C_Write(epld->client,0x00,epl_sensor.wait | epl_sensor.mode);

	return count;
}

static ssize_t epl_sensor_store_interrupt_type(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d",&value);

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
			if(!epl_sensor.ps.polling_mode)
			{
		    	epl_sensor.ps.interrupt_type = value & 0x03;
				epl_sensor_I2C_Write(epld->client,0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
				LOG_INFO("[%s]: 0x06 = 0x%x\n", __FUNCTION__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
			}
		break;

		case EPL_MODE_ALS: 
			if(!epl_sensor.als.polling_mode)
			{
		    	epl_sensor.als.interrupt_type = value & 0x03;
				epl_sensor_I2C_Write(epld->client,0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
				LOG_INFO("[%s]: 0x07 = 0x%x\n", __FUNCTION__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
			}
		break;
	}

	return count;
}


static ssize_t epl_sensor_store_ps_polling_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int polling_mode = 0;


    sscanf(buf, "%d",&polling_mode);
    epl_sensor.ps.polling_mode = polling_mode;
    set_als_ps_intr_type(epld->client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);

    epl_sensor_update_mode(epld->client);

    return count;
}

static ssize_t epl_sensor_store_als_polling_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int polling_mode = 0;

    sscanf(buf, "%d",&polling_mode);
    epl_sensor.als.polling_mode = polling_mode;
    set_als_ps_intr_type(epld->client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);

    epl_sensor_update_mode(epld->client);

    return count;
}

static ssize_t epl_sensor_store_integration(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d",&value);

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
			epl_sensor.ps.integration_time = (value & 0xf) << 2;
			epl_sensor_I2C_Write(epld->client,0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
			epl_sensor_I2C_Read(epld->client, 0x03, 1);
			LOG_INFO("[%s]: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.integration_time | epl_sensor.ps.gain, gRawData.raw_bytes[0]);
		break;

		case EPL_MODE_ALS: 
			epl_sensor.als.integration_time = (value & 0xf) << 2;
			epl_sensor_I2C_Write(epld->client,0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
			epl_sensor_I2C_Read(epld->client, 0x01, 1);
			LOG_INFO("[%s]: 0x01 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.integration_time | epl_sensor.als.gain, gRawData.raw_bytes[0]);
		break;
	}

	epl_sensor_update_mode(epld->client);
	return count;
}

static ssize_t epl_sensor_store_adc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d",&value);

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
			epl_sensor.ps.adc = (value & 0x3) << 3;
			epl_sensor_I2C_Write(epld->client,0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
			epl_sensor_I2C_Read(epld->client, 0x04, 1);
			LOG_INFO("[%s]:0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
		break;

		case EPL_MODE_ALS: 
			epl_sensor.als.adc = (value & 0x3) << 3;
			epl_sensor_I2C_Write(epld->client,0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
			epl_sensor_I2C_Read(epld->client, 0x02, 1);
			LOG_INFO("[%s]:0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
		break;
	}

	epl_sensor_update_mode(epld->client);
	return count;
}

static ssize_t epl_sensor_store_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	struct epl_sensor_priv *epld = epl_sensor_obj;

	LOG_FUN();

	sscanf(buf, "%d",&value);

	switch (epl_sensor.mode)
	{
		case EPL_MODE_PS: 
			epl_sensor.ps.cycle = (value & 0x7);
			epl_sensor_I2C_Write(epld->client,0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
			LOG_INFO("[%s]:0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
		break;

		case EPL_MODE_ALS: 
			epl_sensor.als.cycle = (value & 0x7);
			epl_sensor_I2C_Write(epld->client,0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
			LOG_INFO("[%s]:0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
		break;
	}

	epl_sensor_update_mode(epld->client);
	return count;
}

static ssize_t epl_sensor_store_als_report_type(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	LOG_FUN();

	sscanf(buf, "%d", &value);
	epl_sensor.als.report_type = value & 0xf;

	return count;
}
static ssize_t epl_sensor_store_ps_w_calfile(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
	LOG_FUN();

	if(!epl_sensor_obj)
    {
        LOG_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d,%d",&ps_cancelation, &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d",  ps_cancelation, ps_hthr, ps_lthr);

    write_factory_calibration(epld, ps_calibration, ps_cal_len);
	return count;
}

static ssize_t epl_sensor_store_reg_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg;
    int data;
    LOG_FUN();

    sscanf(buf, "%x,%x",&reg, &data);

    LOG_INFO("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);
    epl_sensor_I2C_Write(epld->client, reg, data);

    return count;
}

static ssize_t epl_sensor_store_unlock(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int mode;
    LOG_FUN();

    sscanf(buf, "%d",&mode);

    LOG_INFO("mode = %d \r\n", mode);
	switch (mode)
	{
		case 0: 
			
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 1: 
			
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;

        case 2: 
    		epl_sensor.als.compare_reset = EPL_CMP_RESET;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
        break;

		case 3: 
		    
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

			
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;
	}
    


	return count;
}

static ssize_t epl_sensor_store_als_ch_sel(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ch_sel;
    LOG_FUN();

    sscanf(buf, "%d",&ch_sel);

    LOG_INFO("channel selection = %d \r\n", ch_sel);
	switch (ch_sel)
	{
		case 0: 
		    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_0;
		break;

		case 1: 
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
		break;
	}
    epl_sensor_I2C_Write(epld->client,0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_update_mode(epld->client);

	return count;
}

static ssize_t epl_sensor_store_ps_cancelation(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int cancelation;
    LOG_FUN();

    sscanf(buf, "%d",&cancelation);

    epl_sensor.ps.cancelation = cancelation;

    LOG_INFO("epl_sensor.ps.cancelation = %d \r\n", epl_sensor.ps.cancelation);

    epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

	return count;
}

static ssize_t epl_sensor_show_ps_polling(struct device *dev, struct device_attribute *attr, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.ps.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_als_polling(struct device *dev, struct device_attribute *attr, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.als.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_ps_run_cali(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	ssize_t len = 0;
    int ret;

    LOG_FUN();

    ret = epl_run_ps_calibration(epld);

    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\r\n", ret);

	return len;
}

static ssize_t epl_sensor_show_pdata(struct device *dev, struct device_attribute *attr, char *buf)
{
      struct epl_sensor_priv *epld = epl_sensor_obj;
      ssize_t len = 0;
      bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
      LOG_FUN();

      if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
      {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps(epld->client);
        mutex_unlock(&sensor_mutex);
      }
      LOG_INFO("[%s]: epl_sensor.ps.data.data = %d \r\n", __func__, epl_sensor.ps.data.data);
      len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.ps.data.data);
      return len;

}

static ssize_t epl_sensor_show_als_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    ssize_t len = 0;
    bool enable_als = epld->enable_lflag==1 && epld->als_suspend==0;
    u16 orig_ch1 = 0;
    LOG_FUN();

#if ALS_DYN_INTT
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
#if HTC_ATTR
        if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
        {
            orig_ch1 = epl_sensor.als.data.channels[1];
            mutex_lock(&sensor_mutex);
    	    epl_sensor_read_als(epld->client);
    	    
    	    if(orig_ch1 != epl_sensor.als.data.channels[1]) 
    	        epl_sensor.als.dyn_intt_raw =  epl_sensor.als.data.channels[1]/(als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); 
    	    mutex_unlock(&sensor_mutex);
    	}
        LOG_INFO("[%s]: epl_sensor.als.dyn_intt_raw = %d \r\n", __func__, epl_sensor.als.dyn_intt_raw);
        len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.als.dyn_intt_raw);
#else
        LOG_INFO("[%s]: dynamic_intt_lux = %d \r\n", __func__, dynamic_intt_lux);
        len += snprintf(buf + len, PAGE_SIZE - len, "%d", dynamic_intt_lux);
#endif
    }
    else
#endif
    {
        if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
    	    epl_sensor_read_als(epld->client);
    	    mutex_unlock(&sensor_mutex);
    	}
    	len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.als.data.channels[1]);
    }

    return len;
}

#if ALS_DYN_INTT
static ssize_t epl_sensor_store_c_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int c_h,c_l;
    LOG_FUN();

    if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_NON)
    {
        sscanf(buf, "%d",&c_h);
        c_gain = c_h;
        LOG_INFO("c_gain = %d \r\n", c_gain);
    }
    else
    {
        sscanf(buf, "%d,%d",&c_l, &c_h);
        epld->c_gain_h = c_h;
        epld->c_gain_l = c_l;
    }

	return count;
}

static ssize_t epl_sensor_store_als_dyn_intt(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    
    int intt_h, intt_l;
    int i = 0;
    LOG_FUN();
    sscanf(buf, "%d,%d",&intt_l, &intt_h);

    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        for(i=0; i<2; i++)
        {
            if(i == 0)
            {
                als_dynamic_intt_intt[i] = intt_l<<2;
                als_dynamic_intt_value[i] = als_intt_value[intt_l];
            }
            else
            {
                als_dynamic_intt_intt[i] = intt_h<<2;
                als_dynamic_intt_value[i] = als_intt_value[intt_h];
            }
            LOG_INFO("[%d]: als_dynamic_intt_value=%d \r\n", i, als_dynamic_intt_value[i]);
        }
    }
	return count;
}
static ssize_t epl_sensor_show_als_dyn_intt(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t len = 0;
    LOG_FUN();

    len += snprintf(buf + len, PAGE_SIZE - len, "%d,%d\n", als_dynamic_intt_intt[0]>>2, als_dynamic_intt_intt[1]>>2);

    return len;
}
static ssize_t epl_sensor_store_lsrc_type(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int type;
    LOG_FUN();

    sscanf(buf, "%d",&type);

    epl_sensor.als.lsrc_type = type;

    LOG_INFO("epl_sensor.als.lsrc_type = %d \r\n", epl_sensor.als.lsrc_type);

	return count;
}

static ssize_t epl_sensor_store_lsrc_thd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int lsrc_thrl, lsrc_thrh;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%d,%d",&lsrc_thrl, &lsrc_thrh);

    epld->lsource_thd_low = lsrc_thrl;
    epld->lsource_thd_high = lsrc_thrh;

    LOG_INFO("lsource_thd=(%d,%d) \r\n", epld->lsource_thd_low, epld->lsource_thd_high);

	return count;
}

#endif

#if PS_DYN_K

static ssize_t epl_sensor_store_dyn_th_add(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int dyn_th_add;
    LOG_FUN();

    sscanf(buf, "%d",&dyn_th_add);
    epld->ps_th_add = dyn_th_add;

    return count;
}

static ssize_t epl_sensor_store_dyn_thd_diff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int dyn_diff;
    LOG_FUN();

    sscanf(buf, "%d",&dyn_diff);
    epld->ps_threshold_diff = dyn_diff;
    return count;
}

static ssize_t epl_sensor_store_dyn_max_ir_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ps_max_ir;
    LOG_FUN();

    sscanf(buf, "%d",&ps_max_ir);

    dynk_max_ir_data = ps_max_ir;

    return count;
}

#endif

static ssize_t epl_sensor_store_als_thd_add(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ald_add;


    sscanf(buf, "%d",&ald_add);

    als_thd_add = ald_add;
    LOG_INFO("[%s]: als_thd_add=%d \r\n", __func__, als_thd_add);

    return count;
}


#if HTC_ATTR
static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint16_t ps_en=0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    sscanf(buf, "%hu",&ps_en);

    if (ps_en != 0 && ps_en != 1
			&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	LOG_INFO("[PS][epl88051] %s: ps_en=%d\n", __func__, ps_en);

    if(epld->enable_pflag != ps_en)
    {
        epld->enable_pflag = ps_en;
        if(ps_en)
        {
            wake_lock(&ps_lock);
#if PS_DYN_K
#if PS_RAW_8BIT
            dynk_min_ps_raw_data = 0xff;
#else
            dynk_min_ps_raw_data = 0xffff;
#endif
#endif
        }
        else
        {
#if PS_DYN_K
            cancel_delayed_work(&epld->dynk_thd_polling_work);
#endif
            wake_unlock(&ps_lock);
        }

        epl_sensor_update_mode(epld->client);
    }

    return count;
}

static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
	int ps_adc = 0;
	int ret;
	bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
	int int_gpio;

    LOG_FUN();
#if ZERADUG || QCOM
	int_gpio = gpio_get_value_cansleep(epld->intr_pin);
#else
    int_gpio = 0;
#endif
    if(enable_ps == 0)
    {
        epld->enable_pflag=1;
        epl_sensor_update_mode(epld->client);
    }

    if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps(epld->client);
        mutex_unlock(&sensor_mutex);
    }

    ps_adc = epl_sensor.ps.data.data;
    LOG_INFO("[%s]: ps_adc = %d \r\n", __func__, ps_adc);
	ret = sprintf(buf, "ADC[0x%02X], ENABLE = %d, intr_pin = %d, "
	        "ps_pocket_mode = %d\n",
			ps_adc, enable_ps, int_gpio, epld->ps_pocket_mode);

	return ret;
}

static ssize_t ps_kadc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int ret = 0;

	if(epld->emmc_ps_kadc1 >> 16 == PS_CALIBRATED || kcalibrated == 1)
		ret = sprintf(buf, "P-sensor calibrated,"
    				"INTE_PS1_CANC = (0x%02X), "
    				"INTE_PS2_CANC = (0x%02X)\n",
    				epl_sensor.ps.cancelation, mfg_thd);
	else
               ret = sprintf(buf, "P-sensor NOT calibrated,"
                               "INTE_PS1_CANC = (0x%02X), "
                               "INTE_PS2_CANC = (0x%02X)\n",
                               epl_sensor.ps.cancelation, mfg_thd);

	return ret;
}


static ssize_t ps_kadc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int param1, param2;
    LOG_FUN();

    sscanf(buf, "0x%x 0x%x", &param1, &param2);
	LOG_INFO("[PS][epl88051]%s: store value = 0x%X, 0x%X\n", __func__, param1, param2);

    ps_canc_set = epl_sensor.ps.cancelation = (param2 & 0xFFFF);
    mfg_thd = ((param2 >> 16) & 0xFFFF);

    LOG_INFO("epl_sensor.ps.cancelation = %d \r\n", epl_sensor.ps.cancelation);

    epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

    LOG_INFO("[PS]%s: inte_ps_canc = 0x%02X, mfg_thd = 0x%02X, lpi->ps_conf1_val  = ?\n",
			__func__, epl_sensor.ps.cancelation, mfg_thd);

    kcalibrated = 1;

	return count;
}

static ssize_t ps_canc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;

	ret = sprintf(buf, "PS1_CANC = 0x%02X, PS2_CANC = 0x%02X\n",
			epl_sensor.ps.cancelation, mfg_thd);

    return ret;
}


static ssize_t ps_canc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps1_canc, ps2_canc;
    LOG_FUN();

    sscanf(buf, "0x%x 0x%x", &ps1_canc, &ps2_canc);

    epl_sensor.ps.cancelation = (uint16_t) ps1_canc;
    mfg_thd = (uint16_t) ps2_canc;

    epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

    LOG_INFO("[PS] %s: PS1_CANC = 0x%02X, PS2_CANC = 0x%02X\n",
			    __func__, epl_sensor.ps.cancelation, mfg_thd);

	return count;
}

static ssize_t ps_i2c_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int ret = 0, i;
        struct epl_sensor_priv *epld = epl_sensor_obj;
        uint8_t data[36] = {0};

        for (i = 0; i <= 35; i++) {
                data[i] = i2c_smbus_read_byte_data(epld->client, i);
        }
        ret = sprintf(buf,
                "0x0=0x%02X, 0x1=0x%02X, 0x2=0x%02X, 0x3=0x%02X,\n"
                "0x4=0x%02X, 0x5=0x%02X, 0x6=0x%02X, 0x7=0x%02X,\n"
                "0x8=0x%02X, 0x9=0x%02X, 0xA=0x%02X, 0xB=0x%02X,\n"
                "0xC=0x%02X, 0xD=0x%02X, 0xE=0x%02X, 0xF=0x%02X,\n"
                "0x10=0x%02X, 0x11=0x%02X, 0x12=0x%02X, 0x13=0x%02X,\n"
                "0x14=0x%02X, 0x15=0x%02X, 0x16=0x%02X, 0x17=0x%02X,\n"
                "0x18=0x%02X, 0x19=0x%02X, 0x1A=0x%02X, 0x1B=0x%02X,\n"
                "0x1C=0x%02X, 0x1D=0x%02X, 0x1E=0x%02X, 0x1F=0x%02X,\n"
                "0x20=0x%02X, 0x21=0x%02X, 0x22=0x%02X, 0x23=0x%02X,\n",
                data[0], data[1], data[2], data[3],
                data[4], data[5], data[6], data[7],
                data[8], data[9], data[10], data[11],
                data[12], data[13], data[14], data[15],
                data[16], data[17], data[18], data[19],
                data[20], data[21], data[22], data[23],
                data[24], data[25], data[26], data[27],
                data[28], data[29], data[30], data[31],
                data[32], data[33], data[34], data[35]);

        LOG_INFO("0x0=0x%02X, 0x1=0x%02X, 0x2=0x%02X, 0x3=0x%02X,\n"
                 "0x4=0x%02X, 0x5=0x%02X, 0x6=0x%02X, 0x7=0x%02X,\n"
                 "0x8=0x%02X, 0x9=0x%02X, 0xA=0x%02X, 0xB=0x%02X,\n"
                 "0xC=0x%02X, 0xD=0x%02X, 0xE=0x%02X, 0xF=0x%02X,\n"
                 "0x10=0x%02X, 0x11=0x%02X, 0x12=0x%02X, 0x13=0x%02X,\n"
                 "0x14=0x%02X, 0x15=0x%02X, 0x16=0x%02X, 0x17=0x%02X,\n"
                 "0x18=0x%02X, 0x19=0x%02X, 0x1A=0x%02X, 0x1B=0x%02X,\n"
                 "0x1C=0x%02X, 0x1D=0x%02X, 0x1E=0x%02X, 0x1F=0x%02X,\n"
                 "0x20=0x%02X, 0x21=0x%02X, 0x22=0x%02X, 0x23=0x%02X,\n",
                 data[0], data[1], data[2], data[3],
                 data[4], data[5], data[6], data[7],
                 data[8], data[9], data[10], data[11],
                 data[12], data[13], data[14], data[15],
                 data[16], data[17], data[18], data[19],
                 data[20], data[21], data[22], data[23],
                 data[24], data[25], data[26], data[27],
                 data[28], data[29], data[30], data[31],
                 data[32], data[33], data[34], data[35]);

        return ret;
}

static ssize_t ps_i2c_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg, data;
    LOG_FUN();

    sscanf(buf, "%x,%x",&reg, &data);

    LOG_INFO("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);
    epl_sensor_I2C_Write(epld->client, reg, data);

    return count;
}

static ssize_t ps_hw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;
	ret = sprintf(buf, "PS: ps_conf1_val=?, ps_thd_set=0x%x, "
			"inte_ps_canc=0x%02X, mfg_thd=0x%02X, "
			"ps_conf2_val=?, LS: ls_cmd=?\n",
			    epld->ps_thd_set,
			    epl_sensor.ps.cancelation,
			    mfg_thd);

	return ret;
}

static ssize_t ps_hw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int code;
	struct epl_sensor_priv *epld = epl_sensor_obj;

	sscanf(buf, "0x%x", &code);

	LOG_INFO("[PS]%s: store value = 0x%x\n", __func__, code);
	if (code == 1) {
		epl_sensor.ps.cancelation = 0;
		mfg_thd = 0;
		epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

		LOG_INFO("[PS]%s: Reset ps1_canc=%d, ps2_canc=%d\n",
				__func__, epl_sensor.ps.cancelation, mfg_thd);
	} else {
		epl_sensor.ps.cancelation = ps_canc_set;
		
		epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

		LOG_INFO("[PS]%s: Recover ps1_canc=%d, ps2_canc=%d\n", __func__, epl_sensor.ps.cancelation,
				mfg_thd);
	}

	return count;
}


static ssize_t ps_headset_bt_plugin_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
#if 0
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "ps_conf1_val = 0x%02X, ps_conf2_val = 0x%02X\n",
			lpi->ps_conf1_val, lpi->ps_conf2_val);
#endif
	return ret;
}

static ssize_t ps_headset_bt_plugin_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int headset_bt_plugin = 0;
	
	

	sscanf(buf, "%d", &headset_bt_plugin);
	LOG_INFO("[PS] %s: headset_bt_plugin = %d\n", __func__, headset_bt_plugin);
#if 0
	if (lpi->no_need_change_setting == 1) {
		D("[PS] %s: no_need_change_setting = 0x%x.\n", __func__, lpi->no_need_change_setting);
		return count;
	} else {
		if (headset_bt_plugin == 1) {
			D("[PS][cm36686] %s, Headset or BT or Speaker ON\n", __func__);

			_cm36686_I2C_Read2(lpi->cm36686_slave_address, PS_config, cmd, 2);
			D("[PS][cm36686] %s, read value => cmd[0] = 0x%x, cmd[1] = 0x%x\n",
					__func__, cmd[0], cmd[1]);

			D("[PS][cm36686] %s, Before setting: ps_conf1_val = 0x%x\n",
					__func__, lpi->ps_conf1_val);
			lpi->ps_conf1_val = (cmd[0] & 0x3) | (CM36686_PS_DR_1_320 |
					CM36686_PS_IT_1_5T |
					CM36686_PS_PERS_1);
			D("[PS][cm36686] %s, After setting: ps_conf1_val = 0x%x\n",
					__func__, lpi->ps_conf1_val);

			D("[PS][cm36686] %s, Before setting: ps_conf2_val = 0x%x\n",
					__func__, lpi->ps_conf2_val);
			lpi->ps_conf2_val = cmd[1] & 0xF;
			D("[PS][cm36686] %s, After setting: ps_conf2_val = 0x%x\n",
					__func__, lpi->ps_conf2_val);

			cmd[0] = lpi->ps_conf1_val;
			cmd[1] = lpi->ps_conf2_val;
			D("[PS][cm36686] %s, write cmd[0] = 0x%x, cmd[1] = 0x%x\n",
					__func__, cmd[0], cmd[1]);
			_cm36686_I2C_Write2(lpi->cm36686_slave_address,
					PS_config, cmd, 3);

			_cm36686_I2C_Read2(lpi->cm36686_slave_address, PS_config, cmd, 2);
			D("[PS][cm36686] %s, read 0x3 cmd value after set =>"
					" cmd[0] = 0x%x, cmd[1] = 0x%x\n",
					__func__, cmd[0], cmd[1]);
		} else {
			D("[PS][cm36686] %s, Headset or BT or Speaker OFF\n", __func__);

			_cm36686_I2C_Read2(lpi->cm36686_slave_address, PS_config, cmd, 2);
			D("[PS][cm36686] %s, read value => cmd[0] = 0x%x, cmd[1] = 0x%x\n",
					__func__, cmd[0], cmd[1]);

			lpi->ps_conf1_val = lpi->ps_conf1_val_from_board;
			lpi->ps_conf2_val = lpi->ps_conf2_val_from_board;

			cmd[0] = ((cmd[0] & 0x3) | lpi->ps_conf1_val);
			cmd[1] = ((cmd[1] & 0xF) | lpi->ps_conf2_val);
			D("[PS][cm36686] %s, write cmd[0] = 0x%x, cmd[1] = 0x%x\n",
					__func__, cmd[0], cmd[1]);
			_cm36686_I2C_Write2(lpi->cm36686_slave_address,
					PS_config, cmd, 3);

			_cm36686_I2C_Read2(lpi->cm36686_slave_address, PS_config, cmd, 2);
			D("[PS][cm36686] %s, read 0x3 cmd value after set =>"
					" cmd[0] = 0x%x, cmd[1] = 0x%x\n",
					__func__, cmd[0], cmd[1]);
		}

	}
#endif
	return count;
}

static ssize_t ps_workaround_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
	struct cm36686_info *lpi = lp_info;
	int i = 0;
	char table_str[952] = "";
	char temp_str[64] = "";

	sprintf(table_str, "mapping table size = %d\n", lpi->mapping_size);
	printk(KERN_DEBUG "%s: table_str = %s\n", __func__, table_str);
	for (i = 0; i < lpi->mapping_size; i++) {
		memset(temp_str, 0, 64);
		if ((i == 0) || ((i % 10) == 1))
			sprintf(temp_str, "[%d] = 0x%x", i, lpi->mapping_table[i]);
		else
			sprintf(temp_str, ", [%d] = 0x%x", i, lpi->mapping_table[i]);
		strcat(table_str, temp_str);
		printk(KERN_DEBUG "%s: [%d]: table_str = %s\n", __func__, i, table_str);
		if ((i != 0) && (i % 10) == 0)
			strcat(table_str, "\n");
	}

	return sprintf(buf, "%s\n", table_str);
#else
    return 0;
#endif
}

static ssize_t ps_workaround_table_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
	struct cm36686_info *lpi = lp_info;
	int index = 0;
	unsigned int value = 0;

	sscanf(buf, "%d 0x%x", &index, &value);

	D("%s: input: index = %d, value = 0x%x\n", __func__, index, value);

	if ((index < lpi->mapping_size) && (index >= 0) && (value <= PS_max) && (index >= 0))
		lpi->mapping_table[index] = value;

	if ((index < lpi->mapping_size) && (index >= 0)) {
		printk(KERN_INFO "%s: lpi->mapping_table[%d] = 0x%x, lpi->mapping_size = %d\n",
				__func__, index, lpi->mapping_table[index], lpi->mapping_size);
	}
#endif
	return count;
}


static ssize_t ps_fixed_thd_add_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
#if PS_DYN_K
	return sprintf(buf, "Fixed added threshold = %d\n", epld->ps_th_add);
#else
    return 0;
#endif
}

static ssize_t ps_fixed_thd_add_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;

	sscanf(buf, "%d", &value);

	LOG_INFO("%s: input: value = %d\n", __func__, value);
#if PS_DYN_K
	if ((value >= 0) && (value <= PS_max))
		epld->ps_th_add = value;

	LOG_INFO("%s: epld->ps_th_add = %d\n", __func__, epld->ps_th_add);
#endif
	return count;
}

static ssize_t p_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
       return sprintf(buf,"%d\n", p_status);
}

static ssize_t debug_flag_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int value = 0;

        sscanf(buf, "%d", &value);

	polling_flag 	= (value >> 3) & 0x01;
	eint_flag 	= (value >> 2) & 0x01;
	delay_ls_adc 	= (value >> 1) & 0x01;
	debug_flag 	= value & 0x01;

        return count;
}


static ssize_t debug_flag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
       return sprintf(buf,"polling_flag= %d, eint_flag= %d, delay_ls_adc= %d, "
			"debug_flag= %d\n", polling_flag, eint_flag, delay_ls_adc, debug_flag);
}

static ssize_t ls_adc_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int ret;
	bool enable_als = epld->enable_lflag==1 && epld->als_suspend==0;
	int als_time=0, ps_time=0;
	
	LOG_FUN();
	if(enable_als == 0) {
		epld->enable_lflag=1;
		epl_sensor_update_mode(epld->client);
	}

#if ALS_DYN_INTT
	if(epl_sensor.als.report_type == CMC_BIT_DYN_INT) {
#if 0
		if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0) {
			orig_ch1 = epl_sensor.als.data.channels[1];
			mutex_lock(&sensor_mutex);
			epl_sensor_read_als(epld->client);
			
			if(orig_ch1 != epl_sensor.als.data.channels[1]) 
			epl_sensor.als.dyn_intt_raw =  epl_sensor.als.data.channels[1]/(als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); 
			mutex_unlock(&sensor_mutex);
    		}
#else
	        if(polling_flag == true && eint_flag == true) {
			if(delay_ls_adc == 1) {
				als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
				ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);
			}

			LOG_INFO("[LS] %s: sleep time= %d \n", __func__, (als_time + ps_time));
			mutex_lock(&sensor_mutex);
			epl_sensor_intr_als_report_lux();
			mutex_unlock(&sensor_mutex);

			if(delay_ls_adc == 1)
				msleep(als_time + ps_time);
	        }
#endif
		
		LOG_INFO("[LS] %s: ADC[0x%04X](%d) => level %d, INTT=%d \n", __func__, epld->current_adc, epld->current_adc, epld->current_level, als_dynamic_intt_value[dynamic_intt_idx]);
		
		ret = sprintf(buf, "ADC[0x%04X] => level %d\n", epld->current_adc, epld->current_level);
	} else
#endif
	{
        	if(polling_flag == true && eint_flag == true) {
			mutex_lock(&sensor_mutex);
			epl_sensor_intr_als_report_lux();
			mutex_unlock(&sensor_mutex);
	    	}

		LOG_INFO("[LS] %s: Cali--> ADC[0x%04X](%d) => level %d \n", __func__, epld->current_adc, epld->current_adc, epld->current_level);
    		ret = sprintf(buf, "ADC[0x%04X] => level %d\n", epld->current_adc, epld->current_level);
	}
	return ret;
}

static ssize_t ls_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct epl_sensor_priv *epld = epl_sensor_obj;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n", epld->enable_lflag);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct epl_sensor_priv *epld = epl_sensor_obj;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147 && ls_auto != 148 && ls_auto != 149) {
		return -EINVAL;
	}

	if (ls_auto) {
		epld->enable_lflag = 1;
		if(ls_auto == 147) {
			epl_sensor.als.report_type = CMC_BIT_LEVEL;
			epl_sensor.als.integration_time = als_dynamic_intt_intt[1];
			epl_sensor.als.gain = EPL_GAIN_LOW;
			LOG_INFO("[%s]: change als_report_type=%d \r\n", __func__, epl_sensor.als.report_type);
		} else if(ls_auto == 149) {
			epl_sensor.als.report_type = CMC_BIT_DYN_INT;
			LOG_INFO("[%s]: recover als_report_type=%d \r\n", __func__, epl_sensor.als.report_type);
		}
#if ALS_DYN_INTT
		if(epl_sensor.als.report_type == CMC_BIT_DYN_INT) {
		    epl_sensor.als.cycle = epld->als_init_cycle;
			dynamic_intt_idx = dynamic_intt_init_idx;
			epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
			epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
			dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
			dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
		}
#endif
        if(epld->enable_lflag == 1)
        {
            epl_sensor.als.high_threshold = epl_sensor.als.low_threshold;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        }
	} else {
		epld->enable_lflag = 0;
	}
	epl_sensor_update_mode(epld->client);

	epld->ls_calibrate = (ls_auto == 147) ? 1 : 0;

	LOG_INFO("[LS][epl88051] %s: epld->enable_lflag = %d, ls_auto=%d\n",
			__func__, epld->enable_lflag, ls_auto);

	if (ret < 0)
		pr_err("[LS][epl88051 error]%s: set auto light sensor fail\n", __func__);

	return count;
}

static ssize_t ls_kadc_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct epl_sensor_priv *epld = epl_sensor_obj;
	int ret;

	
	
    ret = sprintf(buf, "kadc = 0x%x, kadc_high_lux = 0x%x, gadc = 0x%x, kadc while this boot = 0x%x\n",
			epld->als_kadc, epld->als_kadc_high_lux, epld->als_gadc, als_kadc);
	return ret;
}

static ssize_t ls_kadc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int kadc_temp = 0;
    int als_value = 0;
	sscanf(buf, "%d", &kadc_temp);
	printk(KERN_INFO "[LS]%s: kadc_temp=0x%x \n", __func__, kadc_temp);

	if (kadc_temp <= 0) {
		printk(KERN_ERR "[LS][epl88051 error] %s: kadc_temp=0x%x\n",
							__func__, kadc_temp);
		return -EINVAL;
	}
	
	epld->als_kadc_high_lux = kadc_temp;
	epld->als_gadc = golden_adc;
    if(als_dynamic_intt_gain[0] == EPL_GAIN_MID){
            als_value = 8;
	    }else{
            als_value = 1;
	    }
	epld->als_kadc = kadc_temp * (als_value * als_dynamic_intt_value[0] / als_dynamic_intt_value[1]);
	if(epld->als_kadc > 0xffff) {
		LOG_INFO("[%s]: set epld->als_kadc %d\r\n", __func__, epld->als_kadc);
		epld->als_kadc = 0xffff;
	}

	LOG_INFO("[LS]%s: als_kadc=0x%x, als_kadc_high_lux=0x%x, als_gadc=0x%x\n",
		 __func__, epld->als_kadc, epld->als_kadc_high_lux, epld->als_gadc);
	return count;
}

static ssize_t ls_update_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int ret = 0;

    lightsensor_update_table(epld); 

    return ret;
}

static ssize_t ls_gadc_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct epl_sensor_priv *epld = epl_sensor_obj;
	int ret;

	ret = sprintf(buf, "als_kadc=0x%x, als_gadc=0x%x, golden_adc=0x%x\n",
			epld->als_kadc, epld->als_gadc, golden_adc);

	return ret;
}

static ssize_t ls_gadc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int gadc_temp = 0;

	sscanf(buf, "%d", &gadc_temp);
	printk(KERN_INFO "[LS]%s: gadc_temp=0x%x \n", __func__, gadc_temp);

	if (gadc_temp <= 0) {
		printk(KERN_ERR "[LS][epl88051 error] %s: gadc_temp=0x%x\n",
				__func__, gadc_temp);
		return -EINVAL;
	}

	epld->als_gadc = gadc_temp;
	golden_adc = gadc_temp;
	printk(KERN_INFO "[LS]%s: als_kadc=0x%x, als_gadc=0x%x, golden_adc=0x%x\n",
				__func__, epld->als_kadc, epld->als_gadc, golden_adc);

	return count;
}


static ssize_t ls_adc_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
             struct epl_sensor_priv *epld = epl_sensor_obj;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
				"[epl88051]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
				i, *(epld->adc_table + i),
				*(epld->adc_table + i),
				i, *(epld->cali_table + i),
				*(epld->cali_table + i));
	}

	if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
	{
	    for (i = 0; i < 10; i++) {
            length += sprintf(buf + length,
    				"[epl88051]Get adc_table_high_lux[%d] =  0x%x ; %d, Get cali_table_high_lux[%d] =  0x%x ; %d, \n",
    				i, *(epld->adc_table_high_lux + i),
    				*(epld->adc_table_high_lux + i),
    				i, *(epld->cali_table_high_lux + i),
    				*(epld->cali_table_high_lux + i));
		}
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;

	char *token[10];
	unsigned long tempdata[10];
	int i, ret;

	printk(KERN_INFO "[LS][epl88051]%s\n", buf);
	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		ret = strict_strtoul(token[i], 16, &(tempdata[i]));
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			printk(KERN_ERR "[LS][epl88051 error] adc_table[%d] =  0x%lx Err\n", i, tempdata[i]);
			return count;
		}
	}
	mutex_lock(&als_get_adc_mutex);
	for (i = 0; i < 10; i++) {
		epld->adc_table[i] = tempdata[i];
		printk(KERN_INFO "[LS][epl88051]Set lpi->adc_table[%d] =  0x%x\n", i, *(epld->adc_table + i));
	}
	if (lightsensor_update_table(epld) < 0)
		printk(KERN_ERR "[LS][cm36686 error] %s: update ls table fail\n", __func__);
	mutex_unlock(&als_get_adc_mutex);
	LOG_INFO("[LS][epl88051] %s\n", __func__);
	return count;
}

static ssize_t ls_fLevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fLevel = %d\n", f_epl88051_level);
}
static ssize_t ls_fLevel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;
	sscanf(buf, "%d", &value);
	(value >= 0)?(value = min(value, 10)):(value = max(value, -1));
	f_epl88051_level = value;
	input_report_abs(epld->als_input_dev, ABS_MISC, f_epl88051_level);
	input_sync(epld->als_input_dev);
	printk(KERN_INFO "[LS]set fLevel = %d\n", f_epl88051_level);

	msleep(1000);
	f_epl88051_level = -1;
	return count;
}

static ssize_t phone_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "phone_status = %d\n", phone_status);

	return ret;
}

static ssize_t phone_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int phone_status1 = 0;
	

	sscanf(buf, "%d" , &phone_status1);

	phone_status = phone_status1;
	LOG_INFO("[PS][epl88051] %s: phone_status = %d\n", __func__, phone_status);


	if ((phone_status == 1 || phone_status == 3) && (call_count < 2))
		call_count++;

	if (phone_status == 1 || phone_status == 2) {  
        dynk_min_ps_raw_data = PS_max;
		
	}

	return count;
}

static ssize_t ls_dark_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
#if 0
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "LS_dark_level = %d\n", lpi->dark_level);
#endif
	return ret;
}
static ssize_t ls_dark_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
	int ls_dark_level = 0;
	struct cm36686_info *lpi = lp_info;

	sscanf(buf, "%d" , &ls_dark_level);

	lpi->dark_level = (uint8_t) ls_dark_level;

	D("[LS] %s: LS_dark_level = %d\n", __func__, lpi->dark_level);
#endif
	return count;
}

#endif

static DEVICE_ATTR(elan_status,	S_IROTH | S_IWOTH, epl_sensor_show_status, NULL);
static DEVICE_ATTR(elan_reg, S_IROTH | S_IWOTH, epl_sensor_show_reg, NULL);
static DEVICE_ATTR(mode, S_IROTH | S_IWOTH, NULL, epl_sensor_store_mode);
static DEVICE_ATTR(wait_time, S_IROTH | S_IWOTH, NULL,	epl_sensor_store_wait_time);
static DEVICE_ATTR(set_threshold, S_IROTH | S_IWOTH, NULL, epl_sensor_store_threshold);
static DEVICE_ATTR(cal_raw, S_IROTH | S_IWOTH, epl_sensor_show_cal_raw, NULL);
static DEVICE_ATTR(als_enable, S_IROTH | S_IWOTH, NULL, epl_sensor_store_als_enable);
static DEVICE_ATTR(als_report_type, S_IROTH | S_IWOTH, NULL, epl_sensor_store_als_report_type);
static DEVICE_ATTR(ps_enable, S_IROTH | S_IWOTH, NULL, epl_sensor_store_ps_enable);
static DEVICE_ATTR(ps_polling_mode, S_IROTH | S_IWOTH, epl_sensor_show_ps_polling, epl_sensor_store_ps_polling_mode);
static DEVICE_ATTR(als_polling_mode, S_IROTH | S_IWOTH, epl_sensor_show_als_polling, epl_sensor_store_als_polling_mode);
static DEVICE_ATTR(gain, S_IROTH | S_IWOTH, NULL, epl_sensor_store_gain);
static DEVICE_ATTR(ir_mode, S_IROTH | S_IWOTH, NULL, epl_sensor_store_ir_mode);
static DEVICE_ATTR(ir_drive, S_IROTH | S_IWOTH, NULL, epl_sensor_store_ir_drive);
static DEVICE_ATTR(ir_on, S_IROTH | S_IWOTH, NULL, epl_sensor_store_ir_contrl);
static DEVICE_ATTR(interrupt_type, S_IROTH | S_IWOTH, NULL, epl_sensor_store_interrupt_type);
static DEVICE_ATTR(integration, S_IROTH  | S_IWOTH, NULL, epl_sensor_store_integration);
static DEVICE_ATTR(adc, S_IROTH | S_IWOTH, NULL, epl_sensor_store_adc);
static DEVICE_ATTR(cycle, S_IROTH | S_IWOTH, NULL, epl_sensor_store_cycle);
static DEVICE_ATTR(ps_w_calfile, S_IROTH | S_IWOTH, NULL, epl_sensor_store_ps_w_calfile);
static DEVICE_ATTR(i2c_w, S_IROTH | S_IWOTH, NULL, epl_sensor_store_reg_write);
static DEVICE_ATTR(unlock, S_IROTH | S_IWOTH, NULL, epl_sensor_store_unlock);
static DEVICE_ATTR(als_ch, S_IROTH | S_IWOTH, NULL, epl_sensor_store_als_ch_sel);
static DEVICE_ATTR(ps_cancel, S_IROTH | S_IWOTH, NULL, epl_sensor_store_ps_cancelation);
static DEVICE_ATTR(run_ps_cali, S_IROTH | S_IWOTH, epl_sensor_show_ps_run_cali, NULL);
static DEVICE_ATTR(pdata, S_IROTH | S_IWOTH, epl_sensor_show_pdata, NULL);
static DEVICE_ATTR(als_data, S_IROTH | S_IWOTH, epl_sensor_show_als_data, NULL);
#if ALS_DYN_INTT
static DEVICE_ATTR(als_dyn_c_gain, S_IROTH | S_IWOTH, NULL, epl_sensor_store_c_gain);
static DEVICE_ATTR(als_dyn_intt,                S_IROTH  | S_IWOTH, epl_sensor_show_als_dyn_intt,                               epl_sensor_store_als_dyn_intt);
static DEVICE_ATTR(als_dyn_lsrc_type, S_IROTH | S_IWOTH, NULL, epl_sensor_store_lsrc_type);
static DEVICE_ATTR(als_dyn_lsrc_thd, S_IROTH | S_IWOTH, NULL, epl_sensor_store_lsrc_thd);
#endif
#if PS_DYN_K
static DEVICE_ATTR(ps_dyn_th_add, S_IROTH | S_IWOTH, NULL, epl_sensor_store_dyn_th_add);
static DEVICE_ATTR(ps_dyn_th_diff, S_IROTH | S_IWOTH, NULL, epl_sensor_store_dyn_thd_diff);
static DEVICE_ATTR(ps_dyn_max_ir, S_IROTH | S_IWOTH, NULL, epl_sensor_store_dyn_max_ir_data);
#endif
static DEVICE_ATTR(als_thd_add, S_IROTH | S_IWOTH, NULL, epl_sensor_store_als_thd_add);

#if HTC_ATTR
static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);
static DEVICE_ATTR(ps_kadc, 0664, ps_kadc_show, ps_kadc_store);
static DEVICE_ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);
static DEVICE_ATTR(ps_i2c, 0664, ps_i2c_show, ps_i2c_store);
static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);
static DEVICE_ATTR(ps_headset_bt_plugin, 0664, ps_headset_bt_plugin_show, ps_headset_bt_plugin_store);
static DEVICE_ATTR(ps_workaround_table, 0664, ps_workaround_table_show, ps_workaround_table_store);
static DEVICE_ATTR(ps_fixed_thd_add, 0664, ps_fixed_thd_add_show, ps_fixed_thd_add_store);
static DEVICE_ATTR(p_status, 0444, p_status_show, NULL);
static DEVICE_ATTR(PhoneApp_status, 0666, phone_status_show, phone_status_store);
static DEVICE_ATTR(debug_flag, 0664, debug_flag_show, debug_flag_store);

static DEVICE_ATTR(ls_adc, 0444, ls_adc_show, NULL);
static DEVICE_ATTR(ls_auto, 0664, ls_enable_show, ls_enable_store);
static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);
static DEVICE_ATTR(ls_gadc, 0664, ls_gadc_show, ls_gadc_store);
static DEVICE_ATTR(ls_adc_table, 0664, ls_adc_table_show, ls_adc_table_store);
static DEVICE_ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store);
static DEVICE_ATTR(ls_dark_level, 0664, ls_dark_level_show, ls_dark_level_store);

static DEVICE_ATTR(ls_update_table, 0664, ls_update_table_show, NULL);
#endif
static struct attribute *epl_sensor_attr_list[] =
{
    &dev_attr_elan_status.attr,
    &dev_attr_elan_reg.attr,
    &dev_attr_als_enable.attr,
    &dev_attr_ps_enable.attr,
    &dev_attr_cal_raw.attr,
    &dev_attr_set_threshold.attr,
    &dev_attr_wait_time.attr,
    &dev_attr_gain.attr,
    &dev_attr_mode.attr,
    &dev_attr_ir_mode.attr,
    &dev_attr_ir_drive.attr,
    &dev_attr_ir_on.attr,
    &dev_attr_interrupt_type.attr,
    &dev_attr_integration.attr,
    &dev_attr_adc.attr,
    &dev_attr_cycle.attr,
    &dev_attr_als_report_type.attr,
    &dev_attr_ps_polling_mode.attr,
    &dev_attr_als_polling_mode.attr,
    &dev_attr_ps_w_calfile.attr,
    &dev_attr_i2c_w.attr,
    &dev_attr_unlock.attr,
    &dev_attr_als_ch.attr,
    &dev_attr_ps_cancel.attr,
    &dev_attr_run_ps_cali.attr,
    &dev_attr_pdata.attr,
    &dev_attr_als_data.attr,
#if ALS_DYN_INTT
    &dev_attr_als_dyn_c_gain.attr,
    &dev_attr_als_dyn_intt.attr,
    &dev_attr_als_dyn_lsrc_type.attr,
    &dev_attr_als_dyn_lsrc_thd.attr,
#endif
#if PS_DYN_K
    &dev_attr_ps_dyn_th_add.attr,
    &dev_attr_ps_dyn_th_diff.attr,
    &dev_attr_ps_dyn_max_ir.attr,
#endif
    &dev_attr_als_thd_add.attr,
};

static struct attribute_group epl_sensor_attr_group =
{
    .attrs = epl_sensor_attr_list,
};

static int epl_sensor_als_open(struct inode *inode, struct file *file)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    LOG_FUN();

    if (epld->als_opened)
    {
        LOG_ERR("[%s]: busy \r\n", __func__);
        return -EBUSY;
    }
    epld->als_opened = 1;

    return 0;
}

static int epl_sensor_als_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int buf[1];
    if(epld->read_flag ==1)
    {
        buf[0] = epl_sensor.als.data.channels[1];
        if(copy_to_user(buffer, &buf , sizeof(buf)))
            return 0;
        epld->read_flag = 0;
        return 12;
    }
    else
    {
        return 0;
    }
}

static int epl_sensor_als_release(struct inode *inode, struct file *file)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    LOG_FUN();

    epld->als_opened = 0;

    return 0;
}

static long epl_sensor_als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int flag;
    unsigned long buf[1];
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_als = epld->enable_lflag==1 && epld->als_suspend==0;
    void __user *argp = (void __user *)arg;

    LOG_INFO("als io ctrl cmd %d\n", _IOC_NR(cmd));

    switch(cmd)
    {
#if S5PV210
        case ELAN_EPL8800_IOCTL_GET_LFLAG:
#else
        case LIGHTSENSOR_IOCTL_GET_ENABLED:
#endif
            LOG_INFO("elan ambient-light IOCTL Sensor get lflag \n");
            flag = epld->enable_lflag;
            if (copy_to_user(argp, &flag, sizeof(flag)))
                return -EFAULT;

            LOG_INFO("elan ambient-light Sensor get lflag %d\n",flag);
            break;
#if S5PV210
        case ELAN_EPL8800_IOCTL_ENABLE_LFLAG:
#else
        case LIGHTSENSOR_IOCTL_ENABLE:
#endif
            LOG_INFO("elan ambient-light IOCTL Sensor set lflag \n");
            if (copy_from_user(&flag, argp, sizeof(flag)))
                return -EFAULT;
            if (flag < 0 || flag > 1)
                return -EINVAL;

            
            {
#if ALS_DYN_INTT
                if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
                {
                    epl_sensor.als.cycle = epld->als_init_cycle;
                    dynamic_intt_idx = dynamic_intt_init_idx;
                    epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                    epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                    dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                    dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                }
#endif
                epld->enable_lflag = flag;
                if(epld->enable_lflag == 1)
                {
                    epl_sensor.als.high_threshold = epl_sensor.als.low_threshold;
                    set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
                }
                epl_sensor_update_mode(epld->client);
            }

            LOG_INFO("elan ambient-light Sensor set lflag %d\n",flag);
            break;

        case ELAN_EPL8800_IOCTL_GETDATA:
            if(enable_als == 0)
            {
                epld->enable_lflag = 1;
                epl_sensor_update_mode(epld->client);
                msleep(30);
            }

            if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(epld->client);
                mutex_unlock(&sensor_mutex);
            }
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                buf[0] = dynamic_intt_lux;
                LOG_INFO("[%s]: als lux = %d \r\n", __func__, dynamic_intt_lux);
            }
            else
#else
            {
                buf[0] = epl_sensor.als.data.channels[1];
                LOG_INFO("[%s]: epl_sensor.als.data.channels[1] = %d \r\n", __func__, epl_sensor.als.data.channels[1]);
            }
#endif

            if(copy_to_user(argp, &buf , sizeof(buf)))
                return -EFAULT;

            break;

        default:
            LOG_ERR("invalid cmd %d\n", _IOC_NR(cmd));
            return -EINVAL;
    }

    return 0;


}

static struct file_operations epl_sensor_als_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_als_open,
    .read = epl_sensor_als_read,
    .release = epl_sensor_als_release,
    .unlocked_ioctl = epl_sensor_als_ioctl
};

static struct miscdevice epl_sensor_als_device =
{
    .minor = MISC_DYNAMIC_MINOR,
#if S5PV210
    .name = "elan_als",
#else
    .name = "lightsensor",
#endif
    .fops = &epl_sensor_als_fops
};

static int epl_sensor_ps_open(struct inode *inode, struct file *file)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    LOG_FUN();

    if (epld->ps_opened)
        return -EBUSY;

    epld->ps_opened = 1;

    return 0;
}

static int epl_sensor_ps_release(struct inode *inode, struct file *file)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;

    LOG_FUN();

    epld->ps_opened = 0;

    return 0;
}

static long epl_sensor_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int value;
    int flag;
    int ret;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    bool enable_ps = epld->enable_pflag==1 && epld->ps_suspend==0;
    void __user *argp = (void __user *)arg;

    LOG_INFO("ps io ctrl cmd %d\n", _IOC_NR(cmd));

    
    switch(cmd)
    {
#if S5PV210
        case ELAN_EPL8800_IOCTL_GET_PFLAG:
#else
        case CAPELLA_CM3602_IOCTL_GET_ENABLED:
#endif
            LOG_INFO("elan Proximity Sensor IOCTL get pflag \n");
            flag = epld->enable_pflag;
            if (copy_to_user(argp, &flag, sizeof(flag)))
                return -EFAULT;

            LOG_INFO("elan Proximity Sensor get pflag %d\n",flag);
            break;
#if S5PV210
        case ELAN_EPL8800_IOCTL_ENABLE_PFLAG:
#else
        case CAPELLA_CM3602_IOCTL_ENABLE:
#endif
		LOG_INFO("elan Proximity IOCTL Sensor set pflag \n");
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;

		if(epld->enable_pflag != flag) {
			epld->enable_pflag = flag;
			if(flag) {
				epld->j_start = jiffies;

				
				input_report_abs(epld->ps_input_dev, ABS_DISTANCE, -1);
				input_sync(epld->ps_input_dev);

				
				epl_sensor_I2C_Write(epld->client, 0x22, (u8)(ps_canc_set & 0xff));
				epl_sensor_I2C_Write(epld->client, 0x23, (u8)((ps_canc_set & 0xff00) >> 8));
				set_psensor_intr_threshold(dynk_thd_low, dynk_thd_high);

				if (epld->dynamical_threshold == 1) {
					LOG_INFO("[PS] default report FAR\n");
					input_report_abs(epld->ps_input_dev, ABS_DISTANCE, 1);
					input_sync(epld->ps_input_dev);
				}

			        ret = irq_set_irq_wake(epld->irq, 1);
			        if (ret < 0) {
					LOG_ERR("%s: fail to enable irq %d as wake interrupt\n", __func__, epld->irq);
					return ret;
				}
#if PS_DYN_K
#if PS_RAW_8BIT
				dynk_min_ps_raw_data = 0xff;
#else
				dynk_min_ps_raw_data = 0xffff;
#endif
#endif
			} else {
                                ret = irq_set_irq_wake(epld->irq, 0);
                                if (ret < 0) {
                                        LOG_ERR("%s: fail to disable irq %d as wake interrupt\n", __func__, epld->irq);
                                        return ret;
                                }
#if PS_DYN_K
				cancel_delayed_work(&epld->dynk_thd_polling_work);
#endif
			}
			epl_sensor_update_mode(epld->client);
		}
		LOG_INFO("elan Proximity Sensor set pflag %d\n",flag);
		break;

        case ELAN_EPL8800_IOCTL_GETDATA:
            if(enable_ps == 0)
            {
                epld->enable_pflag = 1;
                epl_sensor_update_mode(epld->client);
                msleep(30);
            }

            if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps(epld->client);
                mutex_unlock(&sensor_mutex);
            }

            LOG_INFO("[%s]: epl_sensor.ps.data.data = %d \r\n", __func__, epl_sensor.ps.data.data);

            value = epl_sensor.ps.data.data;
            if(copy_to_user(argp, &value , sizeof(value)))
                return -EFAULT;

            LOG_INFO("elan proximity Sensor get data (%d) \n",value);
            break;

        default:
            LOG_ERR("invalid cmd %d\n", _IOC_NR(cmd));
            return -EINVAL;
    }

    return 0;

}

static struct file_operations epl_sensor_ps_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_ps_open,
    .release = epl_sensor_ps_release,
    .unlocked_ioctl = epl_sensor_ps_ioctl
};

static struct miscdevice epl_sensor_ps_device =
{
    .minor = MISC_DYNAMIC_MINOR,
#if S5PV210
    .name = "elan_ps",
#else
    .name = "cm3602",
#endif
    .fops = &epl_sensor_ps_fops
};

static ssize_t light_enable_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct epl_sensor_priv *epld  = epl_sensor_obj;
	LOG_INFO("%s: ALS_status=%d\n", __func__, epld->enable_lflag);
	return sprintf(buf, "%d\n", epld->enable_lflag);
}

static ssize_t light_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    uint16_t als_enable = 0;
    LOG_INFO("light_enable_store: enable=%s \n", buf);

    sscanf(buf, "%hu",&als_enable);

    if(epld->enable_lflag != als_enable)
    {
        epld->enable_lflag = als_enable;
#if ALS_DYN_INTT
        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            epl_sensor.als.cycle = epld->als_init_cycle;
            dynamic_intt_idx = dynamic_intt_init_idx;
            epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
            epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
            dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
            dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
        }
#endif
        epl_sensor_update_mode(epld->client);
    }

	return size;
}
static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRWXUGO,
	   light_enable_show, light_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};
static int epl_sensor_setup_lsensor(struct epl_sensor_priv *epld) {
	int err = 0;
	LOG_INFO("epl_sensor_setup_lsensor enter.\n");

	epld->als_input_dev = input_allocate_device();
	if (!epld->als_input_dev) {
		LOG_ERR( "could not allocate ls input device\n");
		return -ENOMEM;
	}

	epld->als_input_dev->name = ElanALsensorName;
	set_bit(EV_ABS, epld->als_input_dev->evbit);
	input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 9, 0, 0);

	err = input_register_device(epld->als_input_dev);

	if (err < 0) {
		LOG_ERR("can not register ls input device\n");
		goto err_free_ls_input_device;
	}

	err = misc_register(&epl_sensor_als_device);
	if (err < 0) {
		LOG_ERR("can not register ls misc device\n");
		goto err_unregister_ls_input_device;
	}

	err = sysfs_create_group(&epld->als_input_dev->dev.kobj, &light_attribute_group);

	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
	    	goto err_free_ls_input_device;
	}
	return err;

err_unregister_ls_input_device:
	input_unregister_device(epld->als_input_dev);
err_free_ls_input_device:
	input_free_device(epld->als_input_dev);
	return err;
}
static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epl_sensor_priv *epld  = epl_sensor_obj;
	LOG_INFO("%s: PS status=%d\n", __func__, epld->enable_pflag);
	return sprintf(buf, "%d\n", epld->enable_pflag);
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	uint16_t ps_enable = 0;
	LOG_INFO("proximity_enable_store: enable=%s \n", buf);

	sscanf(buf, "%hu",&ps_enable);

	if(epld->enable_pflag != ps_enable) {
	        epld->enable_pflag = ps_enable;
        	if(ps_enable) {
			wake_lock(&ps_lock);
#if PS_DYN_K
#if PS_RAW_8BIT
			dynk_min_ps_raw_data = 0xff;
#else
			dynk_min_ps_raw_data = 0xffff;
#endif
#endif
		}
	        else {
#if PS_DYN_K
			cancel_delayed_work(&epld->dynk_thd_polling_work);
#endif
			wake_unlock(&ps_lock);
        	}
	        epl_sensor_update_mode(epld->client);
	}
	return size;
}
static struct device_attribute dev_attr_psensor_enable =
__ATTR(enable, S_IRWXUGO,
	   proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_psensor_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};
static int epl_sensor_setup_psensor(struct epl_sensor_priv *epld)
{
	int err = 0;
	LOG_INFO("epl_sensor_setup_psensor enter.\n");


	epld->ps_input_dev = input_allocate_device();
	if (!epld->ps_input_dev) {
		LOG_ERR("could not allocate ps input device\n");
		return -ENOMEM;
	}

	epld->ps_input_dev->name = ElanPsensorName;

	set_bit(EV_ABS, epld->ps_input_dev->evbit);
	input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(epld->ps_input_dev);
	if (err < 0) {
		LOG_ERR("could not register ps input device\n");
		goto err_free_ps_input_device;
	}

	err = misc_register(&epl_sensor_ps_device);
	if (err < 0) {
		LOG_ERR("could not register ps misc device\n");
		goto err_unregister_ps_input_device;
	}

	err = sysfs_create_group(&epld->ps_input_dev->dev.kobj, &proximity_attribute_group);

	if (err) {
		pr_err("%s: PS could not create sysfs group\n", __func__);
		goto err_free_ps_input_device;
	}
	return err;

err_unregister_ps_input_device:
	input_unregister_device(epld->ps_input_dev);
err_free_ps_input_device:
	input_free_device(epld->ps_input_dev);
	return err;
}
static int epl88051_pinctrl_init(struct epl_sensor_priv *epld)
{
        int retval;
        struct i2c_client *client = epld->client;
        int ret;

        LOG_INFO("epl88051_pinctrl_init");
        
        epld->pinctrl = devm_pinctrl_get(&client->dev);
        if (IS_ERR_OR_NULL(epld->pinctrl)) {
                pr_err("[PS][epl88051 error]%s: Target does not use pinctrl\n", __func__);
                retval = PTR_ERR(epld->pinctrl);
                epld->pinctrl = NULL;
                return retval;
        }

        epld->gpio_state_init = pinctrl_lookup_state(epld->pinctrl, "cm36686_ps_init");
        if (IS_ERR_OR_NULL(epld->gpio_state_init)) {
                pr_err("[PS][epl88051 error]%s: Cannot get pintctrl state\n", __func__);
                retval = PTR_ERR(epld->gpio_state_init);
                epld->pinctrl = NULL;
                return retval;
        }

        ret = pinctrl_select_state(epld->pinctrl, epld->gpio_state_init);
        if (ret) {
                pr_err("[PS][epl88051 error]%s: Cannot init INT gpio\n", __func__);
                return ret;
        }

        return 0;
}
#ifdef CONFIG_SUSPEND
static int epl_sensor_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();
    LOG_INFO("[%s]: ps=%d, als=%d \r\n", __func__, epld->enable_pflag, epld->enable_lflag);
    epld->als_suspend=1;
#if 0
    if(epld->enable_pflag == 1){
        epld->ps_suspend=0;
        LOG_INFO("[%s]: ps enabled! \r\n", __func__);
        if(epld->enable_lflag == 1)
        {
            LOG_INFO("[%s]: als enabled! \r\n", __func__);
            
        }
    }
    else{
        epld->ps_suspend=1;
        LOG_INFO("[%s]: ps disabled! \r\n", __func__);
        epl_sensor_update_mode(epld->client);
    }
#else
    if(epld->enable_pflag == 1)
    {
        LOG_INFO("[%s]: ps enabled! \r\n", __func__);
        cancel_delayed_work(&epld->dynk_thd_polling_work);
    }

#endif
    return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void epl_sensor_early_suspend(struct early_suspend *h)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    epld->als_suspend=1;

    if(epld->enable_pflag == 1){
        epld->ps_suspend=0;
        LOG_INFO("[%s]: ps enabled! \r\n", __func__);
        if(epld->enable_lflag == 1)
        {
            LOG_INFO("[%s]: als enabled! \r\n", __func__);
            
        }
    }
    else{
        epld->ps_suspend=1;
        LOG_INFO("[%s]: ps disabled! \r\n", __func__);
        epl_sensor_update_mode(epld->client);
    }

}
#endif

static void epl_sensor_resume_work(struct work_struct *work)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	LOG_FUN();
    LOG_INFO("[%s]: ps=%d, als=%d \r\n", __func__, epld->enable_pflag, epld->enable_lflag);
	epld->als_suspend=0;
	epld->ps_suspend=0;

#if 0
	if(epld->enable_pflag == 1) {
		LOG_INFO("[%s]: ps enabled! \r\n", __func__);
		if(epld->enable_lflag == 1) {
			LOG_INFO("[%s]: als enabled! \r\n", __func__);
			epl_sensor_restart_polling();
		}
	} else {
#if ALS_DYN_INTT
        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
        {
            epl_sensor.als.cycle = epld->als_init_cycle;
            dynamic_intt_idx = dynamic_intt_init_idx;
            epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
            epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
            dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
            dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
        }
#endif
        epl_sensor.als.high_threshold = epl_sensor.als.low_threshold;
        set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
		LOG_INFO("[%s]: ps disabled! \r\n", __func__);
		epl_sensor_update_mode(epld->client);
	}
#else
    if(epld->enable_pflag == 1)
    {
        LOG_INFO("[%s]: ps enabled! \r\n", __func__);
        epl_sensor_restart_dynk_polling();
    }

#endif
}

static int epl_sensor_resume(struct i2c_client *client)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	LOG_FUN();

	LOG_INFO("[%s]++\n", __func__);
	schedule_work(&epld->resume_work);
	LOG_INFO("[%s]--\n", __func__);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void epl_sensor_late_resume(struct early_suspend *h)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    LOG_FUN();

    epld->als_suspend=0;
    epld->ps_suspend=0;

    if(epld->enable_pflag == 1){
        LOG_INFO("[%s]: ps enabled! \r\n", __func__);
        if(epld->enable_lflag == 1)
        {
            LOG_INFO("[%s]: als enabled! \r\n", __func__);
            epl_sensor_restart_polling();
        }
    }
    else{
        LOG_INFO("[%s]: ps disabled! \r\n", __func__);
        epl_sensor_update_mode(epld->client);
    }
}
#endif

#endif

#if HTC_ATTR
static void lightsensor_set_kvalue(struct epl_sensor_priv *epld)
{
    int als_value = 0;
	if (!epld) {
		pr_err("[LS][epl88051 error]%s: ls_info is empty\n", __func__);
		return;
	}

	if(als_dynamic_intt_gain[0] == EPL_GAIN_MID){
		als_value = 8;
	} else {
		als_value = 1;
	}

	LOG_INFO("[LS] %s: ALS calibrated als_kadc=0x%x\n",
						 __func__, epld->emmc_als_kadc);

	if (epld->emmc_als_kadc >> 16 == ALS_CALIBRATED) {

		epld->als_kadc_high_lux = epld->emmc_als_kadc & 0xFFFF;
		epld->als_kadc = epld->emmc_als_kadc & 0xFFFF;
		epld->als_kadc = epld->als_kadc * (als_value * als_dynamic_intt_value[0]
						   / als_dynamic_intt_value[1]);
		if(epld->als_kadc > 0xFFFF) {
			LOG_INFO("[LS] %s:set epld->als_kadc= %d, epld->als_kadc_high_lux= %d\n",
				__func__, epld->als_kadc, epld->als_kadc_high_lux);
			epld->als_kadc = 0xFFFF;
		}
		lightsensor_cali = 1;

	} else {
		epld->als_kadc_high_lux = 0;
		epld->als_kadc = 0;
		lightsensor_cali = 0;
		LOG_INFO("[LS] %s: no ALS calibrated\n", __func__);
	}

	if (epld->als_kadc && epld->als_kadc_high_lux && golden_adc > 0) {
		epld->als_kadc = (epld->als_kadc > 0) ? epld->als_kadc : golden_adc;
		epld->als_kadc_high_lux = (epld->als_kadc_high_lux > 0) ?
					       epld->als_kadc_high_lux : golden_adc;
		epld->als_gadc = golden_adc;
	} else {

		epld->als_kadc_high_lux = 1;
		epld->als_kadc = 1 * (als_value * als_dynamic_intt_value[0]
						   / als_dynamic_intt_value[1]);
		epld->als_gadc = 1;
	}
	LOG_INFO("[LS] %s: als_kadc=0x%x, als_kadc_high_lux=0x%x, als_gadc=0x%x\n",
		 __func__, epld->als_kadc, epld->als_kadc_high_lux, epld->als_gadc);
}

static void psensor_set_kvalue(struct epl_sensor_priv *epld)
{

        LOG_INFO("[PS] %s: PS calibrated emmc_ps_kadc1 = 0x%04X, emmc_ps_kadc2 = 0x%04X\n",
                               __func__, epld->emmc_ps_kadc1, epld->emmc_ps_kadc2);

        
        if (epld->emmc_ps_kadc1 >> 16 == PS_CALIBRATED) {
                psensor_cali = 1;
                epld->inte_ps_canc = (uint16_t) (epld->emmc_ps_kadc2 & 0xFFFF);
                epld->mfg_thd = (uint16_t) ((epld->emmc_ps_kadc2 >> 16) & 0xFFFF);
                epld->ps_thd_set = epld->mfg_thd;

                LOG_INFO("[PS] %s: PS calibrated inte_ps_canc = 0x%02X, mfg_thd = 0x%02X\n",
                                               __func__, epld->inte_ps_canc, epld-> mfg_thd);
        } else {
                psensor_cali = 0;
                epld->ps_thd_set = epld->ps_thd_no_cal;
                LOG_INFO("[PS] %s: PS_THD=%d, no calibration\n", __func__, epld->ps_thd_set);
                LOG_INFO("[PS] %s: Proximity NOT calibrated\n", __func__);
        }
}

static int epl_sensor_parse_dt(struct device *dev, struct epl_sensor_priv *epld)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	struct device_node *offset = NULL;
	int cali_size = 0;
	unsigned char *cali_data = NULL;
	int i = 0;
	uint32_t temp = 0;

	LOG_INFO("[PS][epl88051] %s: +\n", __func__);
#if 1 
	prop = of_find_property(dt, "epl88051,levels", NULL);
	if (prop) {
		of_property_read_u32_array(dt, "epl88051,levels", adctable, 10);
		epld->adc_table = &adctable[0];
		epld->adc_table_high_lux = &adctable[0];
	}

	prop = of_find_property(dt, "epl88051,golden_adc", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,golden_adc", &golden_adc);
		epld->als_gadc = golden_adc;
	}

	prop = of_find_property(dt, "epl88051,epl88051_slave_address", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,epl88051_slave_address", &epld->epl88051_slave_address);
	}

	prop = of_find_property(dt, "epl88051,ps1_thd_set", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps1_thd_set", &epld->ps_thd_set);
	}

	prop = of_find_property(dt, "epl88051,ps1_thd_no_cal", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps1_thd_no_cal", &epld->ps_thd_no_cal);
	}

	prop = of_find_property(dt, "epl88051,ps_th_add", NULL);
        if (prop) {
                of_property_read_u32(dt, "epl88051,ps_th_add", &epld->ps_th_add);
        }

	prop = of_find_property(dt, "epl88051,dynamical_threshold", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,dynamical_threshold", &epld->dynamical_threshold);
	}

	 prop = of_find_property(dt, "epl88051,dark_level", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,dark_level", &epld->dark_level);
	}

	prop = of_find_property(dt, "epl88051,ps_duty", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps_duty", &temp);
	}
	epld->ps_duty = (u8)temp;

	prop = of_find_property(dt, "epl88051,ps_pers", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps_pers", &temp);
	}
	epld->ps_pers = (u8)temp;

	prop = of_find_property(dt, "epl88051,ps_it", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps_it", &temp);
	}
	epld->ps_it = (u8)temp;

	prop = of_find_property(dt, "epl88051,ps_hd", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps_hd", &temp);
	}
	epld->ps_hd = (u8)temp; 

	prop = of_find_property(dt, "epl88051,ps_led_current", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps_led_current", &temp);
	}
	epld->ps_led_current = (u8)temp;

#else
    epld->ps_duty = EPL_CYCLE_64;
    epld->ps_pers = EPL_PERIST_1;
	epld->ps_it = EPL_PS_INTT_80;
    epld->ps_led_current = EPL_IR_DRIVE_100;

    epld->adc_table = &adctable[0];
    for (i = 0; i < 10; i++) {
		epld->adc_table[i] = als_intr_level[i];
		LOG_INFO("[LS][epl88051]Set epld->adc_table[%d] =  0x%x\n", i, *(epld->adc_table + i));
	}
    epld->als_gadc = golden_adc;
    epld->als_kadc = k_adc;

    epld->adc_table_high_lux = &adctable[0];
    for (i = 0; i < 10; i++) {
		epld->adc_table_high_lux[i] = als_intr_level[i];
		LOG_INFO("[LS][epl88051]Set epld->adc_table_high_lux[%d] =  0x%x\n", i, *(epld->adc_table_high_lux + i));
	}
    epld->als_kadc_high_lux = k_adc_high_lux;
#endif
	epld->emmc_als_kadc = 0;
#if 1 
	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, LIGHT_SENSOR_FLASH_DATA, &cali_size);

		LOG_INFO("%s: Light sensor cali_size = %d", __func__, cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				LOG_INFO("cali_data[%d] = %02x ", i, cali_data[i]);
				epld->emmc_als_kadc |= (cali_data[i] << (i * 8));
			}
		}
	} else
		LOG_INFO("%s: Light sensor calibration data offset not found", __func__);

#endif
	epld->emmc_ps_kadc1 = 0;
	epld->emmc_ps_kadc2 = 0;
#if 1   
	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, PSENSOR_FLASH_DATA, &cali_size);

		LOG_INFO("%s: Psensor cali_size = %d", __func__, cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				LOG_INFO("cali_data[%d] = %02x ", i, cali_data[i]);
				epld->emmc_ps_kadc1 |= (cali_data[i] << (i * 8));
			}
			for (i = 4; (i < cali_size) && (i < 8); i++) {
				LOG_INFO("cali_data[%d] = %02x ", i, cali_data[i]);
				epld->emmc_ps_kadc2 |= (cali_data[i] << ((i-4) * 8));
			}
		}
	} else
		LOG_INFO("%s: Psensor calibration data offset not found", __func__);

#endif
	
	
	

	return 0;

}
#endif

#if HTC_ALS
static int lightsensor_update_table(struct epl_sensor_priv *epld)
{
	uint16_t data[10], data_low[10];
	int i;
	for (i = 0; i < 10; i++) {
		if (*(epld->adc_table + i) < 0xFFFF) {
			data[i] = *(epld->adc_table + i)
				* epld->als_kadc / epld->als_gadc;

            if(i != 0 && data[i] <= data[i-1])
    		{
                data[i] = 0xffff;
    		}
		} else {
			data[i] = *(epld->adc_table + i);
		}
		LOG_INFO("[LS][epl88051] %s: Calibrated adc_table: data[%d], %x\n",
								__func__, i, data[i]);
	}
	memcpy(epld->cali_table, data, 20);

	i = 0;
	for (i = 0; i < 10; i++) {
		if (*(epld->adc_table_high_lux + i) < 0xFFFF) {
			data_low[i] = *(epld->adc_table_high_lux + i)
				* epld->als_kadc_high_lux / epld->als_gadc;
		} else {
			data_low[i] = *(epld->adc_table_high_lux + i);
		}
		LOG_INFO("[LS][epl88051] %s: Calibrated adc_table_high_lux: data[%d], %x\n",
								__func__, i, data_low[i]);
	}
	memcpy(epld->cali_table_high_lux, data_low, 20);

	return 0;
}
#endif
static int epl_sensor_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int err = 0;
	struct epl_sensor_priv *epld ;

	LOG_INFO("elan sensor probe enter.\n");

	epld = kzalloc(sizeof(struct epl_sensor_priv), GFP_KERNEL);
	if (!epld)
	    return -ENOMEM;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,"No supported i2c func what we need?!!\n");
		err = -ENOTSUPP;
		goto i2c_fail;
	}

        if((i2c_smbus_read_byte_data(client, 0x21)) != 0x81) {
                LOG_ERR("elan ALS/PS sensor is failed. \n");
		err = -ENXIO;
                goto i2c_fail;
        }

	LOG_INFO("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
	LOG_INFO("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
	LOG_INFO("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
	LOG_INFO("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
	LOG_INFO("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
	LOG_INFO("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
	LOG_INFO("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
	LOG_INFO("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
	LOG_INFO("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
	LOG_INFO("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
	LOG_INFO("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
	LOG_INFO("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x20));
	LOG_INFO("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x21));
	LOG_INFO("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
	LOG_INFO("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));

	epld->als_level_num = sizeof(epld->als_level)/sizeof(epld->als_level[0]);
	epld->als_value_num = sizeof(epld->als_value)/sizeof(epld->als_value[0]);
	BUG_ON(sizeof(epld->als_level) != sizeof(als_level));
	memcpy(epld->als_level, als_level, sizeof(epld->als_level));
	BUG_ON(sizeof(epld->als_value) != sizeof(als_value));
	memcpy(epld->als_value, als_value, sizeof(epld->als_value));

	epld->client = client;
	epld->irq = client->irq;
	epld->j_start = 0;
	epld->j_end = 0;


	i2c_set_clientdata(client, epld);

	epl_sensor_obj = epld;
	INIT_WORK(&epld->resume_work, epl_sensor_resume_work);
	INIT_DELAYED_WORK(&epld->eint_work, epl_sensor_eint_work);
	INIT_DELAYED_WORK(&epld->polling_work, epl_sensor_polling_work);
#if PS_DYN_K
	INIT_DELAYED_WORK(&epld->dynk_thd_polling_work, epl_sensor_dynk_thd_polling_work);
#endif
	mutex_init(&sensor_mutex);
	mutex_init(&sensor_enable_mutex);
#if HTC_ATTR
	err = epl_sensor_parse_dt(&client->dev, epld);
#if HTC_ALS
	mutex_init(&als_get_adc_mutex);
	lightsensor_set_kvalue(epld);
	err = lightsensor_update_table(epld);
	if (err < 0) {
		pr_err("[LS][epl88051 error]%s: update ls table fail\n",
				__func__);
		goto i2c_fail;
	}
#endif
#endif

	psensor_set_kvalue(epld);
	ps_canc_set    = epld->inte_ps_canc;
	mfg_thd        = epld->mfg_thd;

	
	initial_global_variable(client, epld);

	err = epl_sensor_setup_lsensor(epld);
	if (err < 0) {
		LOG_ERR("epl_sensor_setup_lsensor error!!\n");
	        goto err_lightsensor_setup;
	}

	err = epl_sensor_setup_psensor(epld);
	if (err < 0) {
		LOG_ERR("epl_sensor_setup_psensor error!!\n");
		goto err_psensor_setup;
	}

	epld->lp_wq = create_singlethread_workqueue("cm36686_wq");
	if (!epld->lp_wq) {
		LOG_ERR("create_singlethread workqueue, cm36686_wq error!!\n");
		err = -ENOMEM;
		goto err_fail;
	}

	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
	wake_lock_init(&(epld->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	if (epl_sensor.als.polling_mode==0 || epl_sensor.ps.polling_mode==0) {
		err = epl_sensor_setup_interrupt(epld);
		if (err < 0) {
			LOG_ERR("setup error!\n");
			goto err_sensor_setup;
		}
	}

#ifdef CONFIG_SUSPEND
#if defined(CONFIG_HAS_EARLYSUSPEND)
	epld->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	epld->early_suspend.suspend = epl_sensor_early_suspend;
	epld->early_suspend.resume = epl_sensor_late_resume;
	register_early_suspend(&epld->early_suspend);
#endif
#endif

	sensor_dev = platform_device_register_simple("elan_alsps", -1, NULL, 0);
	if (IS_ERR(sensor_dev)) {
		printk ("sensor_dev_init: error\n");
		goto err_fail;
	}


	err = sysfs_create_group(&sensor_dev->dev.kobj, &epl_sensor_attr_group);
	if (err !=0) {
		dev_err(&client->dev, "%s:create sysfs group error", __func__);
		goto err_fail;
	}

#if HTC_ATTR
	epld->epl_sensor_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(epld->epl_sensor_class)) {
		err = PTR_ERR(epld->epl_sensor_class);
		epld->epl_sensor_class = NULL;
		goto err_create_class;
	}

	epld->ls_dev = device_create(epld->epl_sensor_class,
					NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(epld->ls_dev))) {
		err = PTR_ERR(epld->ls_dev);
		epld->ls_dev = NULL;
		goto err_create_ls_device;
	}

	err = device_create_file(epld->ls_dev, &dev_attr_ls_adc);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_auto);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_kadc);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_gadc);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_adc_table);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_dark_level);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_flevel);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_update_table);
	if (err)
		goto err_create_ls_device_file;

        err = device_create_file(epld->ls_dev, &dev_attr_debug_flag);
        if (err)
                goto err_create_ls_device_file;

	epld->ps_dev = device_create(epld->epl_sensor_class,
					NULL, 0, "%s", "proximity");

	if (unlikely(IS_ERR(epld->ps_dev))) {
		err = PTR_ERR(epld->ps_dev);
		epld->ps_dev = NULL;
		goto err_create_ps_device;
	}

	err = device_create_file(epld->ps_dev, &dev_attr_ps_adc);
	if (err)
    		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_kadc);
	if (err)
    		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_canc);
	if (err)
    		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_hw);
	if (err)
		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_p_status);
	if (err)
		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_headset_bt_plugin);
	if (err)
		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_workaround_table);
	if (err)
		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_fixed_thd_add);
	if (err)
		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_ps_i2c);
	if (err)
		goto err_create_ps_device_file;

	err = device_create_file(epld->ps_dev, &dev_attr_PhoneApp_status);
	if (err)
		goto err_create_ps_device_file;

#endif
    LOG_INFO("sensor probe success.\n");
    return err;
#if HTC_ATTR
err_create_ps_device_file:
	device_unregister(epld->ps_dev);
err_create_ls_device_file:
	device_unregister(epld->ls_dev);
err_create_ps_device:
err_create_ls_device:
	class_destroy(epld->epl_sensor_class);
err_create_class:
#endif
err_fail:
	destroy_workqueue(epld->lp_wq);
	wake_lock_destroy(&(epld->ps_wake_lock));
	wake_lock_destroy(&ps_lock);
	input_unregister_device(epld->als_input_dev);
	input_unregister_device(epld->ps_input_dev);
	input_free_device(epld->als_input_dev);
	input_free_device(epld->ps_input_dev);
err_lightsensor_setup:
err_psensor_setup:
err_sensor_setup:
	misc_deregister(&epl_sensor_ps_device);
	misc_deregister(&epl_sensor_als_device);
i2c_fail:
	kfree(epld);
	return err;
}

static int epl_sensor_remove(struct i2c_client *client)
{
    struct epl_sensor_priv *epld = i2c_get_clientdata(client);

    dev_dbg(&client->dev, "%s: enter.\n", __func__);
#if defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&epld->early_suspend);
#endif
    sysfs_remove_group(&sensor_dev->dev.kobj, &epl_sensor_attr_group);
    platform_device_unregister(sensor_dev);
    input_unregister_device(epld->als_input_dev);
    input_unregister_device(epld->ps_input_dev);
    input_free_device(epld->als_input_dev);
    input_free_device(epld->ps_input_dev);
    misc_deregister(&epl_sensor_ps_device);
    misc_deregister(&epl_sensor_als_device);
    free_irq(epld->irq,epld);
    kfree(epld);
    return 0;
}

static const struct i2c_device_id epl_sensor_id[] =
{
    { EPL_DEV_NAME, 0 },
    { }
};


#if QCOM
static struct of_device_id epl_match_table[] = {
	{.compatible = "epl88051_KPW"},
	{},
};
#endif

static struct i2c_driver epl_sensor_driver =
{
    .probe	= epl_sensor_probe,
    .remove	= epl_sensor_remove,
    .id_table	= epl_sensor_id,
    .driver	= {
        .name = EPL_DEV_NAME,
        .owner = THIS_MODULE,
#if QCOM
        .of_match_table =epl_match_table,
#endif
    },
#ifdef CONFIG_SUSPEND
    .suspend = epl_sensor_suspend,
    .resume = epl_sensor_resume,
#endif
};

static int __init epl_sensor_init(void)
{
    return i2c_add_driver(&epl_sensor_driver);
}

static void __exit  epl_sensor_exit(void)
{
    i2c_del_driver(&epl_sensor_driver);
}

module_init(epl_sensor_init);
module_exit(epl_sensor_exit);

MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl88051_KPW driver");
MODULE_LICENSE("GPL");






