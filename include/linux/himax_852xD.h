#ifndef HIMAX8528_H
#define HIMAX8528_H
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#define HIMAX8528_NAME "Himax8528"
#define HIMAX8528_FINGER_SUPPORT_NUM 10

#define HX_TP_SYS_DIAG											// Support Sys : Diag function				,default is open
#define HX_TP_SYS_REGISTER									// Support Sys : Register function		,default is open
#define HX_TP_SYS_DEBUG										// Support Sys : Debug Level function	,default is open
#define HX_TP_SYS_FLASH_DUMP								// Support Sys : Flash dump function	,default is open
#define HX_TP_SYS_SELF_TEST									// Support Sys : Self Test Function		,default is open
#define HX_TP_SYS_HITOUCH										// Support Sys : Hi-touch command			,default is open
//#define HX_EN_SEL_BUTTON									// Support Self Virtual key						,default is close
//#define HX_EN_MUT_BUTTON									// Support Mutual Virtual Key					,default is close
//#define HX_EN_GESTURE											// Support Gesture , need porting			,default is close
#define HX_RST_PIN_FUNC											// Support HW Reset										,default is open
#define HX_LOADIN_CONFIG									// Support Common FW,load in config
//#define HX_PORTING_DEB_MSG								// Support Driver Porting Message			,default is close
//#define HX_IREF_MODIFY										// Support IREF Modify Function				,default is close
//#define HX_FW_UPDATE_BY_I_FILE						// Support Update FW by i file				,default is close</div>
//TODO END

#ifdef HX_RST_PIN_FUNC
	//#define HX_RST_BY_POWER									// Support Reset by power pin						,default is close
	#if defined(CONFIG_TOUCHSCREEN_HIMAX_ESD_EN)
	#define HX_ESD_WORKAROUND								// Support ESD Workaround               ,default is close
	#endif
	#define ENABLE_CHIP_RESET_MACHINE					// Support Chip Reset Workqueue         ,default is open
#endif	

#ifdef ENABLE_CHIP_RESET_MACHINE 
	#define HX_TP_SYS_RESET										// Support Sys : HW Reset function			,default is open
	//#define ENABLE_CHIP_STATUS_MONITOR			// Support Polling ic status            ,default is close
#endif

//------------------------------------------// Support Different IC. Select one at one time.
#define HX_85XX_A_SERIES_PWON		1
#define HX_85XX_B_SERIES_PWON		2
#define HX_85XX_C_SERIES_PWON		3
#define HX_85XX_D_SERIES_PWON		4

//------------------------------------------// Supoort ESD Issue

//------------------------------------------// Support FW Bin checksum method,mapping with Hitouch *.bin
#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC	3
	
//=============================================================================================================
//
//	Segment : Himax Define Variable
//
//=============================================================================================================
//TODO START : Modify follows deinfe variable
//#define HX_ENABLE_EDGE_TRIGGER						// define:Level triggle , un-defined:Level triggle
#define HX_KEY_MAX_COUNT             4			// Max virtual keys
#define DEFAULT_RETRY_CNT            3			// For I2C Retry count
//TODO END

//TODO START : Modify follows power gpio / interrupt gpio / reset gpio
//------------------------------------------// power supply , i2c , interrupt gpio
#define HIMAX_PWR_GPIO				59
#define HIMAX_INT_GPIO				60
#define HIMAX_RST_GPIO				62
//TODO END

//TODO START : Modify the I2C address
//------------------------------------------// I2C
#define HIMAX_I2C_ADDR				0x48
#define HIMAX_TS_NAME			"himax-ts"
//TODO END

//------------------------------------------// Input Device
#define INPUT_DEV_NAME	"himax-touchscreen"	

//------------------------------------------// Flash dump file
#define FLASH_DUMP_FILE "/sdcard/Flash_Dump.bin"

//------------------------------------------// Diag Coordinate dump file
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

//------------------------------------------// Virtual key
#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOME
#define HX_VKEY_2   KEY_RESERVED
#define HX_VKEY_3   KEY_RESERVED
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}


struct himax_config_init_api {
	int (*i2c_himax_master_write)(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t retry);
	int (*i2c_himax_write_command)(struct i2c_client *client, uint8_t command, uint8_t retry);
	int (*i2c_himax_read_command)(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t retry);
};

struct himax_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
};

struct himax_config {
	uint8_t default_cfg;
	uint32_t sensor_id;
	uint32_t fw_ver;
	uint16_t length;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	uint8_t c1[5];/*config register start*/
	uint8_t c2[2];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[2];
	uint8_t c19[4];
	uint8_t c20[9];
	uint8_t c21[5];
	uint8_t c22[16];
	uint8_t c23[3];
	uint8_t c24[2];
	uint8_t c25[2];
	uint8_t c26[5];
	uint8_t c27[3];
	uint8_t c28[9];
	uint8_t c29[11];
	uint8_t c30[4];
	uint8_t c31[65];
	uint8_t c32[13];
	uint8_t c33[3];
	uint8_t c34[3];
	uint8_t c35[17];
	uint8_t c36[31];
	uint8_t c37[31];
	uint8_t c38[17];
	uint8_t c39[25];
	uint8_t c40[23];
	uint8_t c41[29];
	uint8_t c42[9];
	uint8_t c43_1[32];
	uint8_t c43_2[30];
	uint8_t c44_1[32];
	uint8_t c44_2[6];
	uint8_t c45[3];/*config register end*/
};

struct himax_i2c_platform_data_config_type28
{
	uint8_t version;
	uint8_t tw_id;
	uint8_t common;
	uint8_t x_fuzz;
	uint8_t y_fuzz;
	uint8_t z_fuzz;

	uint8_t c1[5];
	uint8_t c2[2];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[2];
	uint8_t c19[4];
	uint8_t c20[9];
	uint8_t c21[5];
	uint8_t c22[16];
	uint8_t c23[3];
	uint8_t c24[2];
	uint8_t c25[2];
	uint8_t c26[5];
	uint8_t c27[3];
	uint8_t c28[9];
	uint8_t c29[11];
	uint8_t c30[4];
	uint8_t c31[65];
	uint8_t c32[13];
	uint8_t c33[3];
	uint8_t c34[3];
	uint8_t c35[17];
	uint8_t c36[31];
	uint8_t c37[31];
	uint8_t c38[17];
	uint8_t c39[25];
	uint8_t c40[23];
	uint8_t c41[29];
	uint8_t c42[9];
	uint8_t c43_1[32];
	uint8_t c43_2[30];
	uint8_t c44_1[32];
	uint8_t c44_2[6];
	uint8_t c45[3];
};

struct himax_i2c_platform_data {
	/* common variables */
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	uint8_t powerOff3V3;
	int (*power)(int on);
	struct himax_virtual_key *virtual_key;
	int gpio_irq;
	int gpio_reset;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;
	uint8_t slave_addr;
	uint32_t event_htc_enable;
	uint8_t cable_config[2];

	/* To support Sprint 2a2b request, inject 2a2b in google format. Naming as synaptic solution*/
	uint8_t support_htc_event;
	/* Support Sprint 2a2b --End--*/

	/* To decide Protocol A+ID(0) or B+ID(1) */
	uint8_t protocol_type;

	/* For fake event --start-- */
	int screenWidth;
	int screenHeight;
	/* For fake event --end-- */

	/* Touch Window Vendor Names --Start-- */
	char ID0[20];
	char ID1[20];
	char ID2[20];
	char ID3[20];
	/* Touch Window Vendor Names --End-- */

	void (*reset)(void);
	int (*loadSensorConfig)(struct i2c_client *client, struct himax_i2c_platform_data *pdata, struct himax_config_init_api *i2c_api);
	/* for compatible and caching purpose */
	uint8_t version;
	uint8_t fw_version;
	uint8_t tw_id;
	/* for resume ESD recovery clock divider restore */
	uint8_t *regCD;
	/* types of configurations */
	struct himax_i2c_platform_data_config_type28 *type28;
	int type28_size;
	
	/*STE power framwork, not work in 7x27*/
	int (*init)(struct device *dev, struct himax_i2c_platform_data *pdata);
	int (*enable)(struct device *dev, struct himax_i2c_platform_data *pdata);
	int (*disable)(struct device *dev, struct himax_i2c_platform_data *pdata);
	void (*exit)(struct device *dev, struct himax_i2c_platform_data *pdata);
	void *extra;
	/* Setting to enable/disable to always keep power on to faster resume */
	int power_keep_on;
};

//------------------------------------------// Himax TP COMMANDS -> Do not modify the below definition
#define HX_CMD_NOP					 0x00	/* no operation */
#define HX_CMD_SETMICROOFF			 0x35	/* set micro on */
#define HX_CMD_SETROMRDY			 0x36	/* set flash ready */
#define HX_CMD_TSSLPIN				 0x80	/* set sleep in */
#define HX_CMD_TSSLPOUT 			 0x81	/* set sleep out */
#define HX_CMD_TSSOFF				 0x82	/* sense off */
#define HX_CMD_TSSON				 0x83	/* sense on */
#define HX_CMD_ROE					 0x85	/* read one event */
#define HX_CMD_RAE					 0x86	/* read all events */
#define HX_CMD_RLE					 0x87	/* read latest event */
#define HX_CMD_CLRES				 0x88	/* clear event stack */
#define HX_CMD_TSSWRESET			 0x9E	/* TS software reset */
#define HX_CMD_SETDEEPSTB			 0xD7	/* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN		 0xDD	/* set cache function */
#define HX_CMD_SETIDLE				 0xF2	/* set idle mode */
#define HX_CMD_SETIDLEDELAY 		 0xF3	/* set idle delay */
#define HX_CMD_SELFTEST_BUFFER		 0x8D	/* Self-test return buffer */
#define HX_CMD_MANUALMODE			 0x42
#define HX_CMD_FLASH_ENABLE 		 0x43
#define HX_CMD_FLASH_SET_ADDRESS	 0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND	 0x47
#define HX_CMD_FLASH_WRITE_BUFFER	 0x48
#define HX_CMD_FLASH_PAGE_ERASE 	 0x4D
#define HX_CMD_FLASH_SECTOR_ERASE	 0x4E
#define HX_CMD_CB					 0xCB
#define HX_CMD_EA					 0xEA
#define HX_CMD_4A					 0x4A
#define HX_CMD_4F					 0x4F
#define HX_CMD_B9					 0xB9
#define HX_CMD_76					 0x76
//------------------------------------------------------------------------------------------

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};
extern int proximity_enable_from_ps(int on);
#endif
