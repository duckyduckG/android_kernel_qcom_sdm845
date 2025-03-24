/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/platform_data/atmel_mxt_ts.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>


#include "../../fih/fih_touch.h"

#include <linux/msm_drm_notify.h>

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	255

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71 71
#define MXT_OBJ_NOISE_T72		72
#define MXT_OBJ_RETRANSMISSION		80
#define MXT_UNLOCK_GESTURE_T81		81
#define MXT_MOISTURE_SUPPRESSION_T136		136
#define MXT_PROCI_SYMBOLGESTUREPROCESSOR	92
#define MXT_PROCI_TOUCHSEQUENCELOGGER	93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100
#define MXT_PROCI_ACTIVESTYLUS_T107	107

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	BIT(7)
#define MXT_T6_STATUS_OFL	BIT(6)
#define MXT_T6_STATUS_SIGERR	BIT(5)
#define MXT_T6_STATUS_CAL	BIT(4)
#define MXT_T6_STATUS_CFGERR	BIT(3)
#define MXT_T6_STATUS_COMSERR	BIT(2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP	1
#define MXT_POWER_CFG_POWERSAVE	2
#define MXT_POWER_CFG_GESTURE		3


/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_CTRL		0
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		BIT(0)
#define MXT_T9_SUPPRESS		BIT(1)
#define MXT_T9_AMP		BIT(2)
#define MXT_T9_VECTOR		BIT(3)
#define MXT_T9_MOVE		BIT(4)
#define MXT_T9_RELEASE		BIT(5)
#define MXT_T9_PRESS		BIT(6)
#define MXT_T9_DETECT		BIT(7)

struct t9_range {
	__le16 x;
	__le16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	BIT(0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      BIT(6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	BIT(0)

/* T100 Multiple Touch Touchscreen */

//SW-PROD-DH-TP_SUSPEND-RESUME-00+[
#define MXT_T100_CTRL_ENABLE	(1 << 0)
#define MXT_T100_CTRL_RPTEN	(1 << 1)
#define MXT_T100_CTRL_DISSCRMSG	(1 << 2)
#define MXT_T100_CTRL_SCANEN	(1 << 7)
//SW-PROD-DH-TP_SUSPEND-RESUME-00+]

#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	BIT(5)

#define MXT_T100_TCHAUX_VECT	BIT(0)
#define MXT_T100_TCHAUX_AMPL	BIT(1)
#define MXT_T100_TCHAUX_AREA	BIT(2)

#define MXT_T100_DETECT		BIT(7)
#define MXT_T100_TYPE_MASK	0x70

enum t100_type {
	MXT_T100_TYPE_FINGER		= 1,
	MXT_T100_TYPE_PASSIVE_STYLUS	= 2,
	MXT_T100_TYPE_ACTIVE_STYLUS	= 3,
	MXT_T100_TYPE_HOVERING_FINGER	= 4,
	MXT_T100_TYPE_GLOVE		= 5,
	MXT_T100_TYPE_LARGE_TOUCH	= 6,
};

#define MXT_DISTANCE_ACTIVE_TOUCH	0
#define MXT_DISTANCE_HOVERING		1

#define MXT_TOUCH_MAJOR_DEFAULT		1
#define MXT_PRESSURE_DEFAULT		1

/* Gen2 Active Stylus */
#define MXT_T107_STYLUS_STYAUX		42
#define MXT_T107_STYLUS_STYAUX_PRESSURE	BIT(0)
#define MXT_T107_STYLUS_STYAUX_PEAK	BIT(4)

#define MXT_T107_STYLUS_HOVER		BIT(0)
#define MXT_T107_STYLUS_TIPSWITCH	BIT(1)
#define MXT_T107_STYLUS_BUTTON0		BIT(2)
#define MXT_T107_STYLUS_BUTTON1		BIT(3)

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_CHG_DELAY	        100	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */
#define MXT_BOOTLOADER_WAIT	36E5	/* 1 minute */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	BIT(5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

//SW-PROD-DH-TP_SUSPEND-RESUME-00+[
#define MXT_T93_ENABLE  	1
#define MXT_T93_DISABLE  	2
// MXT T93 Configuration
#define MXT_T93_CTRL_ENABLE  (1 << 0)
#define MXT_T93_CTRL_RPTEN   (1 << 1)
#define MXT_T80_ENABLE 1
#define MXT_T80_DISABLE 2
#define MXT_T80_COMPTHR_OFFSET 1

#define MXT_T81_ENABLE_OFFSET 0
#define MXT_T81_GOOD_AREA_OFFSET 7
#define MXT_T81_CTRL_ENABLE  (1 << 0)
#define MXT_T81_CTRL_RPTEN   (1 << 1)
#define MXT_T81_CTRL_PRESSREQ   (1 << 2)

#define MXT_T81_TOUCH_TIME 17

#define MXT_T136_ENABLE_OFFSET 1

#define MXT_T8_ENABLE 1
#define MXT_T8_DISABLE 2
#define MXT_T8_RECAL_OFFSET 6

#define MXT_T100_MOVING_FILTER_NORMAL 1
#define MXT_T100_MOVING_FILTER_RECAL 2 //For ghost point in double tap/glance enabled
#define MXT_T100_MOVING_FILTER_OFFSET 47
//SW-PROD-DH-TP_SUSPEND-RESUME-00+]

#define DO_NOT_CHECK_CFG_VER 0
#define CHECK_CFG_VER 1
#define GET_FW_CFG_VER_ONLY 2

#define TOUCH_SUSPEND 0
#define TOUCH_RESUME 1
#define BBOX_TOUCH_PROBE_FAIL do {printk("BBox::UEC;7::0\n");printk("BBox::EHCS:51203:i:Touch probe fail");} while (0);
#define BBOX_TOUCH_I2C_READ_FAIL do {printk("BBox::UEC;7::1\n");printk("BBox::EHCS:51204:i:Touch i2c read fail");} while (0);
#define BBOX_TOUCH_I2C_WRITE_FAIL do {printk("BBox::UEC;7::2\n");printk("BBox::EHCS:51205:i:Touch i2c write fail");} while (0);
#define BBOX_TOUCH_FIRMWARE_UPDATE_FAIL do {printk("BBox::UEC;7::6\n");printk("BBox::EHCS:51206:i:Touch firmware upgrade fail");} while (0);

#define CALIBRATION_TIMEOUT 50

//To define touch driver need to  re-register input device after config/firmware update
#define RE_REGISTER_INPUT_DEVICE

#define LONG_PRESSE_TOUCH_COUNT 1
#define NODE_NUMBER_MIN_GOOD_1 1//2
#define NODE_NUMBER_MIN_GOOD_2 1//3
#define NODE_NUMBER_MIN_GOOD_3 1//4
#define NODE_NUMBER_MIN_PARTIAL 1//5
#define PV_MIN_GOOD_2 0//50
#define PV_MIN_GOOD_3 0//70
#define PV_MIN_PARTIAL 0//100

enum long_press_type {
	TYPE_AREA_GOOD_1	= 1,
	TYPE_AREA_GOOD_2	= 2,
	TYPE_AREA_GOOD_3	= 3,
	TYPE_AREA_PARTIAL	= 4,
};

enum test_results {
	TEST_PASS = 0,
	TEST_FAIL,
	TEST_INVALID,
};


struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

/* Firmware frame structure */
struct mxt_fw_frame {
	__be16 size;
	u8 data[];
};

/* Config update context */
struct mxt_cfg {
	u8 *raw;
	size_t raw_size;
	off_t raw_pos;

	u8 *mem;
	size_t mem_size;
	int start_ofs;

	struct mxt_info info;
};

/* Firmware update context */
struct mxt_flash {
	struct mxt_data *data;
	const struct firmware *fw;
	struct mxt_fw_frame *frame;
	loff_t pos;
	size_t frame_size;
	unsigned int count;
	unsigned int retry;
	u8 previous;
	struct completion flash_completion;
	struct delayed_work work;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool xy_switch;
	bool in_bootloader;
	bool force_update_fw;
	bool force_update_config;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 multitouch;
	struct t7_config t7_cfg;
	unsigned long t15_keystatus;
	u8 stylus_aux_pressure;
	u8 stylus_aux_peak;
	bool use_retrigen_workaround;
	u8 double_tap_enable;
	u8 T81_enable; //Record Long press enable/disable
	u8 FP_enable; //Record if UD Finger print enable/disable
	u8 prox_status;
	u16 long_press_time; //Record time to trigger long press event
	u16 long_press_size; //Record time to trigger long press size
	u8 gpio_vdd_1p8_ldo;
	u8 maj_version;
	u8 min_version;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	char *fw_name;
	char *cfg_name;
	struct mxt_flash *flash;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T8_address;
	u8 T8_msg_size;
	u16 T71_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T25_msg_size; //Selftest object
	u16 T25_address;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T42_address; //Palm supression
	u16 T44_address;
	u8 T48_reportid;
	u16 T72_address; //Noise supression
	u16 T80_address;//Retransmission Compensation
	u8 T81_reportid; //Unlocker
	u16 T81_address;
	u16 T136_address;
	u16 T92_address;
	u8 T92_reportid;
	u16 T93_address;
	u8 T93_reportid;
	u16 T100_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u16 T107_address;
	struct notifier_block dsi_panel_notif;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	/* for power up handling */
	struct completion chg_completion;

	/* for auto firmware update at boot up */
	struct workqueue_struct *mxt_wq;
	struct delayed_work touch_chip_firmware_upgrade_work;
	struct wake_lock mxt_wake_lock;

	/* Indicates whether device is in suspend */
	bool suspended;

	/* Indicates whether device is updating configuration */
	bool updating_config;
	bool long_press_sent;
	u16 long_press_touch_id;//Record long press finger id
};
struct mxt_data *g_data;
int fw_file_build = 0;
int fw_file_version = 0;
static int Touch_TestResult = TEST_PASS;
static u8 active_rep = 8;
static u8 idle_rep = 16;//32;
u8 gesture_active_rep = 15;
u8 gesture_idle_rep = 50;
int g_TouchDownCount = 0;
int g_TouchEvent[2][2] = {{0}, {0}};
u8 is_i2c_bus_suspend = 0;
u8 touch_debug_test = 0;
u16 long_press_area_x[2] = {545, 895}; //UD_FP_x, C Area
u16 long_press_area_y[2] = {1740, 2100}; //UD_FP_y, C Area

u16 long_press_area_x_good_1[2] = {697, 733};
u16 long_press_area_y_good_1[2] = {1915, 1950};
u16 long_press_area_x_good_2[2] = {661, 769};
u16 long_press_area_y_good_2[2] = {1880, 1985};
u16 long_press_area_x_good_3[2] = {625, 805}; //UD_FP_x, D Area
u16 long_press_area_y_good_3[2] = {1845, 2020}; //UD_FP_y, D Area
u8 T81_good_area[4] = {0xA4, 0x6F, 0x0F, 0x20}; //X: 625 ~ 805, Y: 1845 ~ 2020 (180px x 175px)
u16 x_value[LONG_PRESSE_TOUCH_COUNT] = {0};
u16 y_value[LONG_PRESSE_TOUCH_COUNT] = {0};
u16 node_value[LONG_PRESSE_TOUCH_COUNT] = {0};
u16 pv_value[LONG_PRESSE_TOUCH_COUNT] = {0};
u16 long_press_count = 0;

//touch file node+[
extern struct fih_touch_cb touch_cb;
extern int tp_probe_success;
static void touch_tpfwver_read(char *fw_ver);
static unsigned int touch_double_tap_read(void);
static int touch_double_tap_write(unsigned int enable);
static int touch_selftest_result_read(void);
static void touch_selftest(void);
static unsigned int touch_long_press_read(void);
static int touch_long_press_write(unsigned int enable);
static unsigned int touch_fp_en_read(void);
static int touch_fp_en_write(unsigned int enable);
unsigned int touch_prox_status_read(void);
int touch_prox_status_write(unsigned int enable);
static unsigned int touch_long_press_time_read(void);
static int touch_long_press_time_write(unsigned int long_press_time);
static unsigned int touch_long_press_size_read(void);
static int touch_long_press_size_write(unsigned int long_press_size);

//touch file node+]
static void mxt_autoupdate_firmware_cfg(struct work_struct *work);

static int mxt_start(struct mxt_data *data);
static int mxt_stop(struct mxt_data *data);
int mxt_touch_state(int state);
EXPORT_SYMBOL(mxt_touch_state);
static int __maybe_unused mxt_resume(struct device *dev);
static int __maybe_unused mxt_suspend(struct device *dev);

static int mxt_cfg_read_version(struct mxt_data *data, const struct firmware *fw,  unsigned int data_pos, int config_match);
static int mxt_read_t38_data(struct mxt_data *data);
static int mxt_load_fw(struct device *dev);
static int mxt_remove(struct i2c_client *client);


static int dsi_panel_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	int blank;
	struct msm_drm_notifier *evdata = data;
	struct mxt_data *mxt_ts_data = container_of(self, struct mxt_data, dsi_panel_notif);
	struct device *dev = &mxt_ts_data->client->dev;

	if (!evdata || (evdata->id != 0))
		return 0;

	if (mxt_ts_data && (event == MSM_DRM_EARLY_EVENT_BLANK)) {
		blank = *(int *)(evdata->data);
		if (blank == MSM_DRM_BLANK_UNBLANK) {
			pr_debug("%s: receives EARLY_BLANK:UNBLANK\n", __func__);
			dev_dbg(dev, "%s: receives EARLY_BLANK:UNBLANK\n", __func__);
		} else if (blank == MSM_DRM_BLANK_POWERDOWN) {
			pr_debug("%s: receives EARLY_BLANK:POWERDOWN\n", __func__);
			dev_dbg(dev, "%s: receives EARLY_BLANK:POWERDOWN\n", __func__);
			//mxt_suspend(dev);
		} else {
			pr_err("%s: receives wrong data EARLY_BLANK:%d\n",
				__func__, blank);
		}
	}

	if (mxt_ts_data && event == MSM_DRM_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
		if (blank == MSM_DRM_BLANK_POWERDOWN) {
			pr_debug("%s: receives BLANK:POWERDOWN\n", __func__);
			dev_dbg(dev, "%s: receives BLANK:POWERDOWN\n", __func__);
		} else if (blank == MSM_DRM_BLANK_UNBLANK) {
			pr_debug("%s: receives BLANK:UNBLANK\n", __func__);
			dev_dbg(dev, "%s: receives BLANK:UNBLANK\n", __func__);
			//mxt_resume(dev);
		} else {
			pr_err("%s: receives wrong data BLANK:%d\n",
				__func__, blank);
		}
	}
	return 0;
}

static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
	case MXT_OBJ_NOISE_T72:
	case MXT_UNLOCK_GESTURE_T81:
	case MXT_MOISTURE_SUPPRESSION_T136:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	dev_dbg(&data->client->dev, "MXT MSG: %*ph\n",
		       data->T5_msg_size, message);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data)
		return;

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_dbg(dev, "Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_dbg(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				      &data->debug_msg_attr);
}

static int mxt_wait_for_completion(struct mxt_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c recv failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	u8* buff = NULL;
	buff = (u8*) kmalloc(count, GFP_KERNEL);

	if(buff == NULL)
	{
		dev_err(&data->client->dev, "%s, %d, alloc bootloader error\n", __func__, __LINE__);
		return -ENOMEM;
	}
	memcpy(buff, val, count);
	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = buff;
	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}
	kfree(buff);
	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = data->info ? data->info->family_id : 0;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if (retry || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;

	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool alt_address)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 buf[3];
	bool crc_failure, extended_id;

	error = mxt_lookup_bootloader_address(data, alt_address);
	if (error)
		return error;

	/* Check bootloader status and version information */
	error = mxt_bootloader_read(data, buf, sizeof(buf));
	dev_info(&data->client->dev, "%s, %d, buf[0] = %x, buf[1] = %x, buf[2] = %x, error = %d\n",  __func__, __LINE__, buf[0], buf[1], buf[2], error);
	if (error)
		return error;

	crc_failure = (buf[0] & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;
	extended_id = buf[0] & MXT_BOOT_EXTENDED_ID;

	dev_info(dev, "Found bootloader addr:%02x ID:%u%s%u%s\n",
		 data->bootloader_addr,
		 extended_id ? (buf[1] & MXT_BOOT_ID_MASK) : buf[0],
		 extended_id ? " version:" : "",
		 extended_id ? buf[2] : 0,
		 crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock);

static int mxt_write_firmware_frame(struct mxt_data *data, struct mxt_flash *f)
{
	f->frame = (struct mxt_fw_frame *)(f->fw->data + f->pos);

	/* Take account of CRC bytes */
	f->frame_size = __be16_to_cpu(f->frame->size) + 2U;

	/* Write one frame to device */
	return mxt_bootloader_write(data, f->fw->data + f->pos,
				   f->frame_size);
}

static int mxt_check_bootloader(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_flash *f = data->flash;
	u8 state;
	int ret;

	if(f == NULL)
	{
		dev_err(dev, "%s mxt_flash = null\n", __func__);
		return EINVAL;
	}
	/* Handle interrupt after download/flash process */
	if (f->pos >= f->fw->size) {
		complete(&f->flash_completion);
		return 0;
	}

	ret = mxt_bootloader_read(data, &state, 1);
	if (ret)
		return ret;

	/* Remove don't care bits */
	if (state & ~MXT_BOOT_STATUS_MASK)
		state &= ~MXT_BOOT_STATUS_MASK;
	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
		dev_info(dev, "Unlocking bootloader\n");
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			return ret;

		break;

	case MXT_WAITING_FRAME_DATA:
		if ((f->previous != MXT_WAITING_BOOTLOAD_CMD)
		    && (f->previous != MXT_FRAME_CRC_PASS)
		    && (f->previous != MXT_FRAME_CRC_FAIL))
			goto unexpected;

		ret = mxt_write_firmware_frame(data, f);
		if (ret)
			return ret;

		break;

	case MXT_FRAME_CRC_CHECK:
		if (f->previous != MXT_WAITING_FRAME_DATA)
			goto unexpected;
		break;

	case MXT_FRAME_CRC_PASS:
		if (f->previous != MXT_FRAME_CRC_CHECK)
			goto unexpected;

		/* Next frame */
		f->retry = 0;
		f->pos += f->frame_size;
		f->count++;

		if (f->pos >= f->fw->size)
			dev_info(dev, "Sent %u frames, %zu bytes\n",
				f->count, f->fw->size);
		else if (f->count % 200 == 0)
			dev_info(dev, "Sent %u frames, %lld/%zu bytes\n",
				f->count, f->pos, f->fw->size);
		break;

	case MXT_FRAME_CRC_FAIL:
		if (f->retry > 20) {
			dev_err(dev, "Retry count exceeded\n");
			return -EIO;
		}

		/* Back off by 20ms per retry */
		dev_dbg(dev, "Bootloader frame CRC failure\n");
		f->retry++;
		msleep(f->retry * 20);
		break;

	default:
		return -EINVAL;
	}

	f->previous = state;

	/* Poll after 0.1s if no interrupt received */
	schedule_delayed_work(&f->work, HZ / 10);
	dev_dbg(dev, "%s- state = %x\n", __func__, state);
	return 0;

unexpected:
	dev_err(dev, "Unexpected state transition\n");
	return -EINVAL;
}

int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}
	dev_err(&data->client->dev, "%s, %d, count = 2\n", __func__, __LINE__);
	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;
	bool retry = false;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			BBOX_TOUCH_I2C_READ_FAIL
			return -EIO;
		}
	}

	return 0;
}

static int mxt_read_blks(struct mxt_data *data, u16 start, u16 count, u8 *buf)
{
	u16 offset = 0;
	int error;
	u16 size;

	while (offset < count) {
		size = min(MXT_MAX_BLOCK_WRITE, count - offset);

		error = __mxt_read_reg(data->client,
				       start + offset,
				       size, buf + offset);
		if (error)
			return error;

		offset += size;
	}

	return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret;
	bool retry = false;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret != count) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			BBOX_TOUCH_I2C_WRITE_FAIL
			ret = -EIO;
		}
	} else {
		ret = 0;
	}

	kfree(buf);
	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_dbg(dev, "T6 Config Checksum: 0x%06X\n", crc);
	}

	complete(&data->crc_completion);

	/* Detect reset */
	if (status & MXT_T6_STATUS_RESET)
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		dev_dbg(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			status == 0 ? " OK" : "",
			status & MXT_T6_STATUS_RESET ? " RESET" : "",
			status & MXT_T6_STATUS_OFL ? " OFL" : "",
			status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
			status & MXT_T6_STATUS_CAL ? " CAL" : "",
			status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
			status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object || offset >= mxt_obj_size(object))
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	int i;

	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;

		/* Active-low switch */
		input_report_key(input, pdata->t19_keymap[i],
				 !(message[1] & BIT(i)));
	}
}

static void mxt_input_sync(struct mxt_data *data)
{
	if (data->input_dev) {
		input_mt_report_pointer_emulation(data->input_dev,
				data->pdata->t19_num_keys);
		input_sync(data->input_dev);
	}
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/*
		 * Multiple bits may be set if the host is slow to read
		 * the status messages, indicating all the events that
		 * have happened.
		 */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev, 0, 0);
			mxt_input_sync(data);
		}

		/* A size of zero indicates touch is from a linked T47 Stylus */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_DEFAULT;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* if active, pressure must be non-zero */
		if (!amplitude)
			amplitude = MXT_PRESSURE_DEFAULT;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		//input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	data->update_input = true;
}

int long_press_trigger_save_file(u16 x, u16 y, u8 major, u8 pressure, int avg_flage)
{
	struct file *pfile = NULL;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;
	struct timeval time;
	struct rtc_time tm;
	unsigned long local_time;
	char data_buf[256] = {0};

	do_gettimeofday(&time);
	local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &tm);

	if(avg_flage == 0) { // Raw data
		snprintf(data_buf, sizeof(data_buf),
				"[%04d%02d%02d%02d%02d%02d] , [RAW] , X:%04d , Y:%04d , NODE:%02d , P:%03d \n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, x, y, major, pressure);
	} else if(avg_flage == 1) { // Good avg data
		snprintf(data_buf, sizeof(data_buf),
				"[%04d%02d%02d%02d%02d%02d] , [GOOD] , X:%04d , Y:%04d , NODE:%02d , P:%03d \n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, x, y, major, pressure);
	} else if(avg_flage == 2) { // Partial avg data
		snprintf(data_buf, sizeof(data_buf),
				"[%04d%02d%02d%02d%02d%02d] , [PARTIAL] , X:%04d , Y:%04d , NODE:%02d , P:%03d \n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, x, y, major, pressure);
	}

	dev_info(&g_data->client->dev, "%s", data_buf);

	if(g_data == NULL) {
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}

	dev_info(&g_data->client->dev, "%s \n", __func__);
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", "/data/vendor/misc/touch/", "long_press_trigger_result.log");

	if (NULL == pfile) {
		pfile = filp_open(filepath, O_CREAT | O_RDWR | O_APPEND, 0644);
	}
	if (IS_ERR(pfile)) {
		dev_err(&g_data->client->dev, "error occured while opening file %s. \n", filepath);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = pfile->f_pos;
	vfs_write(pfile, data_buf, strlen(data_buf), &pos);
	pfile->f_pos = pos;
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	u8 type = 0;
	u16 x;
	u16 y;
	int distance = 0;
	int tool = 0;
	u8 major = 0;
	u8 pressure = 0;
	u8 orientation = 0;
	bool active = false;
	bool hover = false;

	int i, long_press_type = 0;
	bool ignore = false;
	u16 x_total = 0, y_total = 0, node_total = 0, pv_total = 0;
	u16 x_avg = 0, y_avg = 0, node_avg = 0, pv_avg = 0;

	id = message[0] - data->T100_reportid_min - 2;

	/* ignore SCRSTATUS events */
	if (id < 0)
		return;

	status = message[1];
	x = get_unaligned_le16(&message[2]);
	y = get_unaligned_le16(&message[4]);

	if (status & MXT_T100_DETECT) {
		type = (status & MXT_T100_TYPE_MASK) >> 4;

		switch (type) {
		case MXT_T100_TYPE_HOVERING_FINGER:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_HOVERING;
			hover = true;
			active = true;
			break;

		case MXT_T100_TYPE_FINGER:
		case MXT_T100_TYPE_GLOVE:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;
			active = true;

			if (data->t100_aux_area)
				major = message[data->t100_aux_area];

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl];

			if (data->t100_aux_vect)
				orientation = message[data->t100_aux_vect];

			break;

		case MXT_T100_TYPE_PASSIVE_STYLUS:
			tool = MT_TOOL_PEN;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;
			active = true;

			/*
			 * Passive stylus is reported with size zero so
			 * hardcode.
			 */
			major = MXT_TOUCH_MAJOR_DEFAULT;

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl];

			break;

		case MXT_T100_TYPE_ACTIVE_STYLUS:
			/* Report input buttons */
			input_report_key(input_dev, BTN_STYLUS,
					 message[6] & MXT_T107_STYLUS_BUTTON0);
			input_report_key(input_dev, BTN_STYLUS2,
					 message[6] & MXT_T107_STYLUS_BUTTON1);

			/* stylus in range, but position unavailable */
			if (!(message[6] & MXT_T107_STYLUS_HOVER))
				break;

			tool = MT_TOOL_PEN;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			active = true;
			major = MXT_TOUCH_MAJOR_DEFAULT;

			if (!(message[6] & MXT_T107_STYLUS_TIPSWITCH)) {
				hover = true;
				distance = MXT_DISTANCE_HOVERING;
			} else if (data->stylus_aux_pressure) {
				pressure = message[data->stylus_aux_pressure];
			}

			break;

		case MXT_T100_TYPE_LARGE_TOUCH:
			/* Ignore suppressed touch */
			break;

		default:
			dev_dbg(dev, "Unexpected T100 type\n");
			return;
		}
	}

	/*
	 * Values reported should be non-zero if tool is touching the
	 * device
	 */
	if (!pressure && !hover)
		pressure = MXT_PRESSURE_DEFAULT;

	input_mt_slot(input_dev, id);

	if (active) {
		dev_dbg(dev, "[%u] type:%d x:%04d y:%04d a:%02d p:%03d v:%02X\n", id, type, x, y, major, pressure, orientation);

		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, major);
		//input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
		input_report_abs(input_dev, ABS_MT_DISTANCE, distance);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, orientation);

		x_value[long_press_count] = x;
		y_value[long_press_count] = y;
		node_value[long_press_count] = major;
		pv_value[long_press_count] = pressure;

		if((status & 0x0F) == 0x04 && (id == 0)) //1st finger Touch Down
		{
			g_TouchEvent[g_TouchDownCount][0] = x;
			g_TouchEvent[g_TouchDownCount][1] = y;

			if(g_TouchDownCount == 1)
			{
				dev_info(dev, "Touch down, Current (%d, %d), Previous (%d, %d)\n", g_TouchEvent[1][0], g_TouchEvent[1][1], g_TouchEvent[0][0], g_TouchEvent[0][1]);
				g_TouchDownCount = 0;
			}
			else g_TouchDownCount++;
		}

		if(data->long_press_sent && data->long_press_touch_id == 0xFF && ((status & 0x0F) == 0x00 || (status & 0x0F) == 0x01))
		{
			for (i=0; i < LONG_PRESSE_TOUCH_COUNT; i++)
			{
				if (x_value[i] == 0 || y_value[i] == 0 || node_value[i] == 0 || pv_value[i] == 0)
				{
					ignore = true;
				}
				x_total += x_value[i];
				y_total += y_value[i];
				node_total += node_value[i];
				pv_total += pv_value[i];
				dev_dbg(dev, "[count %d] x_value = %d, y_value = %d, node_value = %d, pv_value = %d \n", i, x_value[i], y_value[i], node_value[i], pv_value[i]);
			}

			if (ignore) {
				x_avg = 0;
				y_avg = 0;
				node_avg = 0;
				pv_avg = 0;
				ignore = false;
				dev_info(dev, "[ignore] x_avg = %d, y_avg = %d, node_avg = %d, pv_avg = %d \n", x_avg, y_avg, node_avg, pv_avg);
			} else {
				x_avg = x_total / LONG_PRESSE_TOUCH_COUNT;
				y_avg = y_total / LONG_PRESSE_TOUCH_COUNT;
				node_avg = node_total / LONG_PRESSE_TOUCH_COUNT;
				pv_avg = pv_total / LONG_PRESSE_TOUCH_COUNT;
				dev_info(dev, "[avg_check] x_avg = %d, y_avg = %d, node_avg = %d, pv_avg = %d \n", x_avg, y_avg, node_avg, pv_avg);
			}

			if(x_avg> long_press_area_x[0] && x_avg < long_press_area_x[1] && y_avg > long_press_area_y[0] && y_avg < long_press_area_y[1]) {
				if(x_avg > long_press_area_x_good_1[0] && x_avg < long_press_area_x_good_1[1]
						&& y_avg > long_press_area_y_good_1[0] && y_avg < long_press_area_y_good_1[1] && node_avg > NODE_NUMBER_MIN_GOOD_1) {
					long_press_type = TYPE_AREA_GOOD_1;
					dev_info(dev, "[Good_1][DOWN][%u] type:%u x_avg = %d, y_avg = %d, node_avg = %d, pv_avg = %d \n", id, type, x_avg, y_avg, node_avg, pv_avg);
				} else if(x_avg > long_press_area_x_good_2[0] && x_avg < long_press_area_x_good_2[1]
						&& y_avg > long_press_area_y_good_2[0] && y_avg < long_press_area_y_good_2[1] && node_avg > NODE_NUMBER_MIN_GOOD_2 && pv_avg > PV_MIN_GOOD_2) {
					long_press_type = TYPE_AREA_GOOD_2;
					dev_info(dev, "[Good_2][DOWN][%u] type:%u x_avg = %d, y_avg = %d, node_avg = %d, pv_avg = %d \n", id, type, x_avg, y_avg, node_avg, pv_avg);
				} else if(x_avg > long_press_area_x_good_3[0] && x_avg < long_press_area_x_good_3[1]
						&& y_avg > long_press_area_y_good_3[0] && y_avg < long_press_area_y_good_3[1] && node_avg > NODE_NUMBER_MIN_GOOD_3 && pv_avg > PV_MIN_GOOD_3) {
					long_press_type = TYPE_AREA_GOOD_3;
					dev_info(dev, "[Good_3][DOWN][%u] type:%u x_avg = %d, y_avg = %d, node_avg = %d, pv_avg = %d \n", id, type, x_avg, y_avg, node_avg, pv_avg);
				} else if(node_avg > NODE_NUMBER_MIN_PARTIAL && pv_avg > PV_MIN_PARTIAL) {
					long_press_type = TYPE_AREA_PARTIAL;
					dev_info(dev, "[Partial][DOWN][%u] type:%u x_avg = %d, y_avg = %d, node_avg = %d, pv_avg = %d,\n", id, type, x_avg, y_avg, node_avg, pv_avg);
				}

				switch(long_press_type) {
					case TYPE_AREA_GOOD_1:
					case TYPE_AREA_GOOD_2:
					case TYPE_AREA_GOOD_3:
						data->long_press_touch_id = id;
						input_report_key(data->input_dev, UDFP_LONG_PRESSED_DOWN, 1);
						input_sync(data->input_dev);
						input_report_key(data->input_dev, UDFP_LONG_PRESSED_DOWN, 0);
						input_sync(data->input_dev);
						//long_press_trigger_save_file(x_avg, y_avg, node_avg, pv_avg, 1);
						break;
					case TYPE_AREA_PARTIAL:
						data->long_press_touch_id = id;
						input_report_key(data->input_dev, UDFP_LONG_PRESSED_PARTIAL_DOWN, 1);
						input_sync(data->input_dev);
						input_report_key(data->input_dev, UDFP_LONG_PRESSED_PARTIAL_DOWN, 0);
						input_sync(data->input_dev);
						//long_press_trigger_save_file(x_avg, y_avg, node_avg, pv_avg, 2);
						break;
					default:
						break;
				}

				for (i=0; i < LONG_PRESSE_TOUCH_COUNT; i++)
				{
					x_value[i] = 0;
					y_value[i] = 0;
					node_value[i] = 0;
					pv_value[i] = 0;
				}
			 }

			dev_info(dev, "[%u] type:%u x:%u y:%u a:%02d p:%02d v:%02d, major*100/256:%d, data->long_press_size:%d\n", id, type, x, y, major, pressure, orientation, major*100/256, data->long_press_size);
		} else if(!data->long_press_sent && data->long_press_touch_id == 0xFF && ((status & 0x0F) == 0x00 || (status & 0x0F) == 0x01)) {
			if(x > 500 && x < 900 && y > 1700 && y < 2200) {
				//long_press_trigger_save_file(x, y, major, pressure, 0);
			}
		}

		long_press_count ++;
		if(long_press_count >= LONG_PRESSE_TOUCH_COUNT) {
			long_press_count = 0;
		}

	} else {
		dev_dbg(dev, "[%u] release\n", id);
		for (i=0; i < LONG_PRESSE_TOUCH_COUNT; i++)
		{
			x_value[i] = 0;
			y_value[i] = 0;
			node_value[i] = 0;
			pv_value[i] = 0;
		}

		//long_press up event
		if(data->long_press_sent == true && id == data->long_press_touch_id)
		{
			if(x > long_press_area_x_good_3[0] && x < long_press_area_x_good_3[1] && y > long_press_area_y_good_3[0] && y < long_press_area_y_good_3[1])
			{
				input_report_key(data->input_dev, UDFP_LONG_PRESSED_UP, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, UDFP_LONG_PRESSED_UP, 0);
				input_sync(data->input_dev);
				dev_info(dev, "[Good][UP] [%u] type:%u x:%u y:%u a:%02X p:%02X v:%02X\n", id, type, x, y, major, pressure, orientation);
			}
			else
			{
				input_report_key(data->input_dev, UDFP_LONG_PRESSED_PARTIAL_UP, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, UDFP_LONG_PRESSED_PARTIAL_UP, 0);
				input_sync(data->input_dev);
				dev_info(dev, "[Partial][UP] [%u] type:%u x:%u y:%u a:%02X p:%02X v:%02X\n", id, type, x, y, major, pressure, orientation);
			}
			data->long_press_sent = false;
			data->long_press_touch_id = 0xFF;
		}
		/* close out slot */
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	data->update_input = true;
}

static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	for (key = 0; key < data->pdata->t15_num_keys; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			dev_dbg(dev, "T15 key press: %u\n", key);
			__set_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			dev_dbg(dev, "T15 key release: %u\n", key);
			__clear_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n", state, status,
		status & 0x01 ? "FREQCHG " : "",
		status & 0x02 ? "APXCHG " : "",
		status & 0x04 ? "ALGOERR " : "",
		status & 0x10 ? "STATCHG " : "",
		status & 0x20 ? "NLVLCHG " : "");

	return 0;
}
static void mxt_proc_t81_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if(data->input_dev == NULL)
	{
		dev_info(dev, "T81 report unlock %d\n", status);
		return;
	}
	dev_info(dev, "T81 report unlock %d\n", status);
	data->long_press_sent = false;
	if(data->T81_enable)
	{
		//Device resume , send long press down if touch area meet spec, judge it in mxt_proc_t81_messages()
		//Device suspend, send long press down directly.
		if(!data->suspended && data->long_press_sent != true)
		{
			dev_info(dev, "Set long_press_sent true.\n");
			data->long_press_sent = true;
		}
		else
		{
			dev_info(dev, "Sent LONG_PRESSED_DOWN.\n");
			input_report_key(data->input_dev, UDFP_LONG_PRESSED_DOWN, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, UDFP_LONG_PRESSED_DOWN, 0);
			input_sync(data->input_dev);
		}
	}
}

static void mxt_proc_t92_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	dev_info(dev, "T92 long stroke LSTR=%d %d\n",
		 (status & 0x80) ? 1 : 0,
		 status & 0x0F);
}

static void mxt_proc_t93_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if(data->input_dev == NULL)
	{
		dev_info(dev, "T93 report double tap %d\n", status);
		return;
	}
	dev_info(dev, "T93 report double tap %d\n", status);
	input_report_key(data->input_dev, KEY_WAKEUP, 1);
	input_sync(data->input_dev);
	input_report_key(data->input_dev, KEY_WAKEUP, 0);
	input_sync(data->input_dev);
	printk("BBox::EHCS:51207:i:Double Tap success");
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	//dev_info(&data->client->dev, "%s, %d, report_id = %d \n", __func__, __LINE__, report_id);
	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id == data->T93_reportid) {
		mxt_proc_t93_messages(data, message);
	} else if (report_id == data->T81_reportid) {
		mxt_proc_t81_messages(data, message);
	} else if (!data->input_dev || data->suspended) {
		/*
		 * Do not report events if input device is not
		 * yet registered or returning from suspend
		 */
		mxt_dump_message(data, message);
	} else if (report_id >= data->T9_reportid_min &&
		   report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id >= data->T100_reportid_min &&
		   report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	} else if (report_id == data->T92_reportid) {
		mxt_proc_t92_messages(data, message);
	} else {
		dump = true;
	}

	if (dump)
		mxt_dump_message(data, message);

	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	//dev_info(&data->client->dev, "%s, %d, ret = %d \n", __func__, __LINE__, ret);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;
	int iRetry = 0;
	//To prevent touch driver access i2c bus when it in suspend mode.
	while(is_i2c_bus_suspend && iRetry++ < 30)
	{
		usleep_range(5000, 5000);
	}
	if (iRetry > 0)
		dev_dbg(dev, "is_i2c_bus_suspend = %d, iRetry = %d\n", is_i2c_bus_suspend, iRetry);
	else if(iRetry == 31)
		dev_info(dev, "is_i2c_bus_suspend = %d, iRetry = %d\n", is_i2c_bus_suspend, iRetry);
	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	/*
	 * This condition may be caused by the CHG line being configured in
	 * Mode 0. It results in unnecessary I2C operations but it is benign.
	 */
	if (count == 0)
		return IRQ_NONE;

	if (count > data->max_reportid) {
		dev_warn(dev, "T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_warn(dev, "Unexpected invalid message\n");
	}

end:
	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	complete(&data->chg_completion);

	if (data->in_bootloader) {
		if (data->flash && &data->flash->work)
			cancel_delayed_work_sync(&data->flash->work);

		return IRQ_RETVAL(mxt_check_bootloader(data));
	}

	if (!data->object_table)
		return IRQ_HANDLED;

	if (data->T44_address) {
		return mxt_process_messages_t44(data);
	} else {
		return mxt_process_messages(data);
	}
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	dev_dbg(&data->client->dev, "Write T6 object for calibration, value  = 0x%X\n", value);
	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
	{
		dev_err(&data->client->dev, "Write T6 object fail, err  = %d\n", ret);
		return ret;
	}

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
		{
			dev_err(&data->client->dev, "Read T6 object fail, err  = %d\n", ret);
			return ret;
		}
	} while (command_register != 0 && timeout_counter++ <= CALIBRATION_TIMEOUT);
	dev_dbg(&data->client->dev, "Wait T6 calibration done, timeout_counter = %d\n", timeout_counter);

	if (timeout_counter > CALIBRATION_TIMEOUT) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting device\n");

	disable_irq(data->irq);

	reinit_completion(&data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	/* Ignore CHG line for 100ms after reset */
	msleep(100);

	enable_irq(data->irq);

	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;
	dev_info(dev, "%s -\n", __func__);
	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	data->config_crc = 0;
	reinit_completion(&data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	if (irq_get_trigger_type(data->irq) & IRQF_TRIGGER_LOW)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

static int mxt_prepare_cfg_mem(struct mxt_data *data, struct mxt_cfg *cfg)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	unsigned int type, instance, size, byte_offset;
	int offset;
	int ret;
	int i;
	u16 reg;
	u8 val;

	while (cfg->raw_pos < cfg->raw_size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->raw + cfg->raw_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			return -EINVAL;
		}
		cfg->raw_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					dev_err(dev, "Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				cfg->raw_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			return -EINVAL;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n",
				     &val,
				     &offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d at %d\n",
					type, i);
				return -EINVAL;
			}
			cfg->raw_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg->start_ofs;

			if (byte_offset >= 0 && byte_offset < cfg->mem_size) {
				*(cfg->mem + byte_offset) = val;
			} else {
				dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int mxt_upload_cfg_mem(struct mxt_data *data, struct mxt_cfg *cfg)
{
	unsigned int byte_offset = 0;
	int error;

	/* Write configuration as blocks */
	while (byte_offset < cfg->mem_size) {
		unsigned int size = cfg->mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		error = __mxt_write_reg(data->client,
					cfg->start_ofs + byte_offset,
					size, cfg->mem + byte_offset);
		if (error) {
			dev_err(&data->client->dev,
				"Config write error, ret=%d\n", error);
			return error;
		}

		byte_offset += size;
	}

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *data, const struct firmware *fw, int check_cfg_ver)
{
	struct device *dev = &data->client->dev;
	struct mxt_cfg cfg;
	int ret;
	int offset;
	int i;
	int config_match = 1;
	u32 info_crc, config_crc, calculated_crc;
	u16 crc_start = 0;
	dev_info(dev, "%s +, check_cfg_ver = %d\n", __func__, check_cfg_ver);
	/* Make zero terminated copy of the OBP_RAW file */
	cfg.raw = kzalloc(fw->size + 1, GFP_KERNEL);
	if (!cfg.raw)
		return -ENOMEM;

	memcpy(cfg.raw, fw->data, fw->size);
	cfg.raw[fw->size] = '\0';
	cfg.raw_size = fw->size;

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg.raw, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release_raw;
	}

	cfg.raw_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg.raw + cfg.raw_pos, "%hhx%n",
			     (unsigned char *)&cfg.info + i,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release_raw;
		}

		cfg.raw_pos += offset;
	}

	if (cfg.info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release_raw;
	}

	if (cfg.info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release_raw;
	}
	fw_file_version = cfg.info.version;
	fw_file_build = cfg.info.build;

	/* Read CRCs */
	ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release_raw;
	}
	cfg.raw_pos += offset;

	ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release_raw;
	}
	cfg.raw_pos += offset;
	dev_info(dev, "info_crc = 0x%X, data->info_crc = 0x%X, config_crc = 0x%X, data->config_crc = 0x%X\n", info_crc, data->info_crc, config_crc, data->config_crc);

	if(check_cfg_ver == GET_FW_CFG_VER_ONLY)
	{
		ret = 0;
		goto release_raw;
	}

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			dev_dbg(dev, "Config CRC 0x%06X: OK\n",
				 data->config_crc);
			//return 0; //Disable for CHEK_CFG_VER
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
			config_match = 0;
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\n",
			 data->info_crc, info_crc);
		config_match = 0;
	}

	if(check_cfg_ver == CHECK_CFG_VER)
	{
		ret = mxt_cfg_read_version(data, fw, (unsigned int) cfg.raw_pos, config_match);
		if(ret)
		{
			dev_info(dev, "%s, Going to update config\n", __func__);
		}
		else
		{
			dev_info(dev, "%s, Skipped config updating\n", __func__);
			goto release_raw;
		}
	}

	/* Malloc memory to store configuration */
	cfg.start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +
			MXT_INFO_CHECKSUM_SIZE;
	cfg.mem_size = data->mem_size - cfg.start_ofs;
	cfg.mem = kzalloc(cfg.mem_size, GFP_KERNEL);
	if (!cfg.mem) {
		ret = -ENOMEM;
		goto release_raw;
	}

	ret = mxt_prepare_cfg_mem(data, &cfg);
	if (ret)
		goto release_mem;

	/* Calculate crc of the received configs (not the raw config file) */
	if (data->T71_address)
		crc_start = data->T71_address;
	else if (data->T7_address)
		crc_start = data->T7_address;
	else
		dev_warn(dev, "Could not find CRC start\n");

	if (crc_start > cfg.start_ofs) {
		calculated_crc = mxt_calculate_crc(cfg.mem,
						   crc_start - cfg.start_ofs,
						   cfg.mem_size);

		if (config_crc > 0 && config_crc != calculated_crc)
			dev_warn(dev, "Config CRC in file inconsistent, calculated=%06X, file=%06X\n",
				 calculated_crc, config_crc);
	}

	ret = mxt_upload_cfg_mem(data, &cfg);
	if (ret)
		goto release_mem;

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	ret = mxt_check_retrigen(data);
	if (ret)
		goto release_mem;

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	dev_info(dev, "Config successfully updated\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(cfg.mem);
release_raw:
	kfree(cfg.raw);

	return ret;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	if (!data->irq) {
		data->irq = gpio_to_irq(data->pdata->gpio_irq);
		error = request_threaded_irq(data->irq, NULL,
				mxt_interrupt,
				data->pdata->irqflags | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				data->client->name, data);
		if (error) {
			dev_err(&data->client->dev, "Error requesting irq\n");
			return error;
		}

		/* Presence of data->irq means IRQ initialised */
		dev_info(&data->client->dev, "%s data->irq = %d\n", __func__, data->irq);
	} else {
		enable_irq(data->irq);
	}

	if (data->object_table && data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}
	dev_info(&data->client->dev, "%s -\n", __func__);
	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	dev_info(&data->client->dev, "%s\n",  __func__);
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
	msleep(100);
}

static void mxt_free_object_table(struct mxt_data *data)
{
	data->object_table = NULL;
	data->info = NULL;
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T8_address = 0;
	data->T8_msg_size=0;
	data->T71_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T42_address = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T72_address = 0;
	data->T80_address = 0;
	data->T81_reportid = 0;
	data->T81_address = 0;
	data->T136_address = 0;
	data->T92_reportid = 0;
	data->T92_address = 0;
	data->T93_reportid = 0;
	data->T93_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;
	dev_info(&client->dev, "%s, object_num = %d\n", __func__, data->info->object_num);

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_info(&data->client->dev,
			"T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80 &&
			    data->info->version < 0x20) {
				/*
				 * On mXT224 firmware versions prior to V2.0
				 * read and discard unused CRC byte otherwise
				 * DMA reads are misaligned.
				 */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			data->T71_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			data->multitouch = MXT_TOUCH_MULTI_T9;
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			data->T42_address = object->start_address;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_OBJ_NOISE_T72:
			data->T72_address = object->start_address;
		case MXT_OBJ_RETRANSMISSION:
			data->T80_address = object->start_address;
		case MXT_UNLOCK_GESTURE_T81:
			data->T81_reportid = min_id;
			data->T81_address = object->start_address;
		case MXT_MOISTURE_SUPPRESSION_T136:
			data->T136_address = object->start_address;
		case MXT_PROCI_SYMBOLGESTUREPROCESSOR:
			data->T92_reportid = min_id;
			data->T92_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSEQUENCELOGGER:
			data->T93_reportid = min_id;
			data->T93_address = object->start_address;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->multitouch = MXT_TOUCH_MULTITOUCHSCREEN_T100;
			data->T100_address = object->start_address;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		case MXT_PROCI_ACTIVESTYLUS_T107:
			data->T107_address = object->start_address;
			break;
		case MXT_GEN_ACQUIRE_T8:
			data->T8_address = object->start_address;
			data->T8_msg_size = mxt_obj_size(object);
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf)
		return -ENOMEM;

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = mxt_read_blks(data, MXT_OBJECT_START,
			      size - MXT_OBJECT_START,
			      buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error;
	dev_info(&data->client->dev, "%s gpio_vdd_1p8_ldo : %d\n", __func__, data->gpio_vdd_1p8_ldo);
	if(data->pdata->enable_vdd_by_gpio)
	{
		if (!gpio_is_valid(data->gpio_vdd_1p8_ldo))
			return;
		if ( !data->reg_avdd)
			return;

		gpio_set_value(data->pdata->gpio_reset, 0);

		error = gpio_direction_output(data->gpio_vdd_1p8_ldo, 1);
		if (error)
			return;

		error =  regulator_set_voltage(data->reg_avdd, 3296000, 3312000);
		if (error)
			dev_info(&data->client->dev, "%s regulator_set_voltage failed, err = %d\n", __func__, error);
		error = regulator_enable(data->reg_avdd);
		if (error)
		{
			dev_info(&data->client->dev, "%s regulator_enable failed, err = %d\n", __func__, error);
			return;
		}
	}
	else
	{
		if (!data->reg_vdd || !data->reg_avdd)
			return;

		gpio_set_value(data->pdata->gpio_reset, 0);

		error = regulator_enable(data->reg_vdd);
		if (error)
			return;

		error = regulator_enable(data->reg_avdd);
		if (error)
			return;
	}

	/*
	 * According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage
	 */
	msleep(MXT_REGULATOR_DELAY);
	gpio_set_value(data->pdata->gpio_reset, 1);
	msleep(MXT_CHG_DELAY);

retry_wait:
	dev_dbg(&data->client->dev, "%s reinit_completion : chg_completion\n", __func__);
	reinit_completion(&data->chg_completion);
	data->in_bootloader = true;
	error = mxt_wait_for_completion(data, &data->chg_completion,
					MXT_POWERON_DELAY);
	if (error == -EINTR)
		goto retry_wait;
	dev_dbg(&data->client->dev, "%s reinit_completion : chg_completion, complete\n", __func__);
	data->in_bootloader = false;
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	if(data->pdata->enable_vdd_by_gpio)
	{
		if (!gpio_is_valid(data->gpio_vdd_1p8_ldo)) return;

		gpio_direction_output(data->gpio_vdd_1p8_ldo, 0);
		regulator_disable(data->reg_avdd);
	}
	else
	{
		if (!data->reg_vdd || !data->reg_avdd)
			return;

		regulator_disable(data->reg_vdd);
		regulator_disable(data->reg_avdd);
	}
}

static int mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct device_node *np = dev->of_node;
	int error;

	/* Must have reset GPIO to use regulator support */
	if (!gpio_is_valid(data->pdata->gpio_reset)) {
		error = -EINVAL;
		goto fail;
	}
	if (!gpio_is_valid(data->pdata->gpio_irq)) {
		error = -EINVAL;
		goto fail;
	}

	if(data->pdata->enable_vdd_by_gpio)
	{
		data->gpio_vdd_1p8_ldo = of_get_named_gpio_flags(np, "fih,vdd_1p8_ldo_gpio", 0, NULL);
		error = gpio_request(data->gpio_vdd_1p8_ldo, "gpio_vdd_1p8_ldo");
		if (error) {
			dev_err(dev, "request vdd_1p8_ldo_en gpio failed, rc=%d\n",   error);
			goto fail;
		}
	}
	else
	{
		data->reg_vdd = regulator_get(dev, "vdd");
		if (IS_ERR(data->reg_vdd)) {
			error = PTR_ERR(data->reg_vdd);
			dev_err(dev, "Error %d getting vdd regulator\n", error);
			goto fail;
		}
	}

	data->reg_avdd = regulator_get(dev, "avdd");
	if (IS_ERR(data->reg_avdd)) {
		error = PTR_ERR(data->reg_avdd);
		dev_err(dev, "Error %d getting avdd regulator\n", error);
		goto fail_release;
	}

	mxt_regulator_enable(data);

	dev_dbg(dev, "Initialised regulators\n");
	return 0;

fail_release:
	regulator_put(data->reg_vdd);
	regulator_put(data->reg_avdd);

fail:
	data->reg_vdd = NULL;
	data->reg_avdd = NULL;
	gpio_free(data->pdata->gpio_reset);
	gpio_free(data->pdata->gpio_irq);
	if(data->pdata->enable_vdd_by_gpio)
		gpio_free(data->gpio_vdd_1p8_ldo);
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	data->max_x = get_unaligned_le16(&range.x);
	data->max_y = get_unaligned_le16(&range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	data->xy_switch = orient & MXT_T9_ORIENT_SWITCH;

	return 0;
}

static int mxt_set_up_active_stylus(struct input_dev *input_dev,
				    struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u8 styaux;
	int aux;
	u8 ctrl;

	object = mxt_get_object(data, MXT_PROCI_ACTIVESTYLUS_T107);
	if (!object)
		return 0;

	error = __mxt_read_reg(client, object->start_address, 1, &ctrl);
	if (error)
		return error;

	/* Check enable bit */
	if (!(ctrl & 0x01))
		return 0;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T107_STYLUS_STYAUX,
			       1, &styaux);
	if (error)
		return error;

	/* map aux bits */
	aux = 7;

	if (styaux & MXT_T107_STYLUS_STYAUX_PRESSURE)
		data->stylus_aux_pressure = aux++;

	if (styaux & MXT_T107_STYLUS_STYAUX_PEAK)
		data->stylus_aux_peak = aux++;

	input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
	input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);

	dev_dbg(&client->dev,
		"T107 active stylus, aux map pressure:%u peak:%u\n",
		data->stylus_aux_pressure, data->stylus_aux_peak);

	return 0;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	/* read touchscreen dimensions */
	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	data->max_x = get_unaligned_le16(&range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	data->max_y = get_unaligned_le16(&range_y);

	/* read orientation config */
	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	data->xy_switch = cfg & MXT_T100_CFG_SWITCHXY;

	/* allocate aux bytes */
	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_dbg(&client->dev,
		"T100 aux mappings vect:%u ampl:%u area:%u\n",
		data->t100_aux_vect, data->t100_aux_ampl, data->t100_aux_area);

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static void mxt_set_up_as_touchpad(struct input_dev *input_dev,
				   struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	int i;

	input_dev->name = "Atmel maXTouch Touchpad";

	__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

	input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_X,
			  MXT_PIXELS_PER_MM);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
			  MXT_PIXELS_PER_MM);

	for (i = 0; i < pdata->t19_num_keys; i++)
		if (pdata->t19_keymap[i] != KEY_RESERVED)
			input_set_capability(input_dev, EV_KEY,
					     pdata->t19_keymap[i]);
}

static int mxt_initialize_input_device(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	unsigned int mt_flags = 0;
	int i;

	switch (data->multitouch) {
	case MXT_TOUCH_MULTI_T9:
		num_mt_slots = data->T9_reportid_max - data->T9_reportid_min + 1;
		error = mxt_read_t9_resolution(data);
		if (error)
			dev_warn(dev, "Failed to initialize T9 resolution\n");
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T100:
		num_mt_slots = data->num_touchids;
		error = mxt_read_t100_config(data);
		if (error)
			dev_warn(dev, "Failed to read T100 config\n");
		break;

	default:
		dev_err(dev, "Invalid multitouch object\n");
		return -EINVAL;
	}

	/* Handle default values and orientation switch */
	if (data->max_x == 0)
		data->max_x = 1023;

	if (data->max_y == 0)
		data->max_y = 1023;

	if (data->xy_switch)
		swap(data->max_x, data->max_y);

	dev_info(dev, "Touchscreen size X%uY%u\n", data->max_x, data->max_y);

#ifdef RE_REGISTER_INPUT_DEVICE
	if(data->input_dev){
		dev_info(dev, "data->input_dev exist, skipped initialize input device\n");
		return 0;
	}
#endif
	/* Register input device */
	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	if (data->pdata->input_name)
		input_dev->name = data->pdata->input_name;
	else
		input_dev->name = "Atmel maXTouch Touchscreen";

	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, data->max_y, 0, 0);

	/*
	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	     data->t100_aux_ampl)) {
		input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	}
	*/

	/* If device has buttons we assume it is a touchpad */
	if (pdata->t19_num_keys) {
		mxt_set_up_as_touchpad(input_dev, data);
		mt_flags |= INPUT_MT_POINTER;
	} else {
		mt_flags |= INPUT_MT_DIRECT;
	}

	/* For multi touch */
	error = input_mt_init_slots(input_dev, num_mt_slots, mt_flags);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	if (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100) {
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_FINGER, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_DISTANCE,
				     MXT_DISTANCE_ACTIVE_TOUCH,
				     MXT_DISTANCE_HOVERING,
				     0, 0);
	}

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	     data->t100_aux_area)) {
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				     0, MXT_MAX_AREA, 0, 0);
	}

	/*
	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	     (data->t100_aux_ampl || data->stylus_aux_pressure))) {
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				     0, 255, 0, 0);
	}
	*/

	if (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	    data->t100_aux_vect) {
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);
	}

	if (data->multitouch == MXT_TOUCH_MULTI_T9 ||
	    (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	    data->t100_aux_vect)) {
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);
	}

	/* For T107 Active Stylus */
	if (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	    data->T107_address) {
		error = mxt_set_up_active_stylus(input_dev, data);
		if (error)
			dev_warn(dev, "Failed to read T107 config\n");
	}

	/* For T15 Key Array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					data->pdata->t15_keymap[i]);
	}

	/* For double tap */
	if (data->T93_address)
		input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);

	if (data->T81_address)
	{
		input_set_capability(input_dev, EV_KEY, UDFP_LONG_PRESSED_DOWN);
		input_set_capability(input_dev, EV_KEY, UDFP_LONG_PRESSED_UP);
		input_set_capability(input_dev, EV_KEY, UDFP_LONG_PRESSED_PARTIAL_DOWN);
		input_set_capability(input_dev, EV_KEY, UDFP_LONG_PRESSED_PARTIAL_UP);
	}

	input_set_drvdata(input_dev, data);
	dev_info(dev, "atmel, %s, Register Input device\n", __func__);
	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}
//SW-PROD-DH-TP_SUSPEND-RESUME-00+[
static int mxt_t93_configuration(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T93_address + cmd_offset;
	ret = __mxt_read_reg(data->client, reg, 1, &command_register);
	if (ret)
		return ret;

	if (status == MXT_T93_ENABLE)
		command_register |= (MXT_T93_CTRL_ENABLE|MXT_T93_CTRL_RPTEN);
	if (status == MXT_T93_DISABLE)
		command_register &= ~(MXT_T93_CTRL_ENABLE|MXT_T93_CTRL_RPTEN);
	dev_dbg(&data->client->dev, "Write T93 config, value  = 0x%X\n", command_register);
	ret = mxt_write_reg(data->client, reg, command_register);

	if (ret)
		return ret;

	return 0;
}
static int mxt_set_t100_multitouchscreen_cfg(struct mxt_data *data, u16 cmd_offset, u8 type)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T100_address + cmd_offset;
	command_register = type;
	dev_dbg(&data->client->dev, "Write T100 config, value  = 0x%X\n", command_register);
	ret = mxt_write_reg(data->client, reg, command_register);

	if (ret)
	{
		dev_err(&data->client->dev, "Write T100 object fail, err  = %d\n", ret);
		return ret;
	}

	return 0;
}
//To config long press time for triggering under display finger print.
static int mxt_t81_configuration(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T81_address + cmd_offset;
	ret = __mxt_read_reg(data->client, reg, 1, &command_register);

	if (ret)
	{
		dev_err(&data->client->dev, "Read T81 config fail, error = %d\n", ret);
		return ret;
	}

	command_register = status;
	
	ret = mxt_write_reg(data->client, reg, command_register);

	if (ret)
	{
		dev_err(&data->client->dev, "Write T81 config fail, error = %d\n", ret);
		return ret;
	}

	return 0;
}

static int mxt_t81_configuration_area(struct mxt_data *data, u16 cmd_offset)
{
	u16 reg;
	int ret;

	reg = data->T81_address + cmd_offset;
	ret = __mxt_write_reg(data->client, reg, sizeof(T81_good_area), T81_good_area);

	if (ret)
	{
		dev_err(&data->client->dev, "Write T81 config fail, error = %d\n", ret);
		return ret;
	}

	return 0;
}

static int mxt_t136_configuration(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T136_address + cmd_offset;
	ret = __mxt_read_reg(data->client, reg, 1, &command_register);

	if (ret)
	{
		dev_err(&data->client->dev, "Read T136 config fail, error = %d\n", ret);
		return ret;
	}

	command_register = status;

	ret = mxt_write_reg(data->client, reg, command_register);

	if (ret)
	{
		dev_err(&data->client->dev, "Write T136 config fail, error = %d\n", ret);
		return ret;
	}

	return 0;
}

//SW-PROD-DH-TP_SUSPEND-RESUME-00+]
#if 0
static int mxt_t100_configuration(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	u16 reg;
	u8 T100_recal[4] = {0xF4, 0x01, 0xF4, 0x01};
	u8 T100_normal[4] = {0x50, 0x00, 0x50, 0x00};
	u8 T100_object[4] = {0};
	int ret;

	reg = data->T100_address + cmd_offset;

	ret = __mxt_read_reg(data->client, reg, 4, T100_object);
	if (ret)
	{
		dev_err(&data->client->dev, "Read T100 config fail, error = %d\n", ret);
		return ret;
	}
	dev_info(&data->client->dev, "T100 value  = 0x%X, 0x%X, 0x%X, 0x%X\n", T100_object[0], T100_object[1], T100_object[2], T100_object[3]);
	if (status == MXT_T100_MOVING_FILTER_NORMAL)
	{
		dev_info(&data->client->dev, "Write T100 config, value  = 0x%X, 0x%X, 0x%X, 0x%X\n", T100_normal[0], T100_normal[1], T100_normal[2], T100_normal[3]);
		ret = __mxt_write_reg(data->client, reg, sizeof(T100_normal), T100_normal);
	}
	else if (status == MXT_T100_MOVING_FILTER_RECAL)
	{
		dev_info(&data->client->dev, "Write T100 config, value  = 0x%X, 0x%X, 0x%X, 0x%X\n", T100_recal[0], T100_recal[1], T100_recal[2], T100_recal[3]);
		ret = __mxt_write_reg(data->client, reg, sizeof(T100_recal), T100_recal);
	}

	if (ret)
	{
		dev_err(&data->client->dev, "Write T100 config fail, error = %d\n", ret);
		return ret;
	}

	return 0;
}


static int mxt_t80_configuration(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	u16 reg;
	u8 command_register;
	int ret;

	reg = data->T80_address + cmd_offset;
	ret = __mxt_read_reg(data->client, reg, 1, &command_register);
	if (ret)
		return ret;

	dev_info(&data->client->dev, "Read T80 config, value  = 0x%X\n", command_register);
	if (status == MXT_T80_ENABLE)
		command_register = 1;
	if (status == MXT_T80_DISABLE)
		command_register = 0;
	dev_info(&data->client->dev, "Write T80 config, value  = 0x%X\n", command_register);
	ret = mxt_write_reg(data->client, reg, command_register);

	if (ret)
		return ret;

	return 0;
}
#endif
static int mxt_t8_configuration(struct mxt_data *data, u16 cmd_offset, u8 status)
{
	u16 reg;
	u8 T8_Recal_dis[4] = {0xFF, 0x01, 0x00, 0x00};
	u8 T8_Recal_en[4] = {0x00, 0x00, 0x0A, 0x01};
	u8 T8_Recal[4] = {0};
	int ret;

	reg = data->T8_address + cmd_offset;

	ret = __mxt_read_reg(data->client, reg, 4, T8_Recal);
	if (ret)
	{
		dev_err(&data->client->dev, "Read T8 config fail, error = %d\n", ret);
		return ret;
	}
	dev_dbg(&data->client->dev, "T8 value  = 0x%X, 0x%X, 0x%X, 0x%X\n", T8_Recal[0], T8_Recal[1], T8_Recal[2], T8_Recal[3]);
	if (status == MXT_T8_ENABLE)
	{
		dev_dbg(&data->client->dev, "Write T8 config, value  = 0x%X, 0x%X, 0x%X, 0x%X\n", T8_Recal_en[0], T8_Recal_en[1], T8_Recal_en[2], T8_Recal_en[3]);
		ret = __mxt_write_reg(data->client, reg, sizeof(T8_Recal_en), T8_Recal_en);
	}
	else if (status == MXT_T8_DISABLE)
	{
		dev_dbg(&data->client->dev, "Write T8 config, value  = 0x%X, 0x%X, 0x%X, 0x%X\n", T8_Recal_dis[0], T8_Recal_dis[1], T8_Recal_dis[2], T8_Recal_dis[3]);
		ret = __mxt_write_reg(data->client, reg, sizeof(T8_Recal_dis), T8_Recal_dis);
	}

	if (ret)
	{
		dev_err(&data->client->dev, "Write T8 config fail, error = %d\n", ret);
		return ret;
	}

	return 0;
}

static int mxt_sysfs_init(struct mxt_data *data);
static void mxt_sysfs_remove(struct mxt_data *data);

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg, int check_cfg_ver);

static void mxt_config_cb(const struct firmware *cfg, void *ctx)
{
	mxt_configure_objects(ctx, cfg, CHECK_CFG_VER);
	release_firmware(cfg);
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int recovery_attempts = 0;
	int error;
	dev_info(&client->dev, "%s %d+\n", __func__, __LINE__);

	while (1) {
		error = mxt_read_info_block(data);
		if (!error)
			break;

		/* Check bootloader state */
		error = mxt_probe_bootloader(data, false);
		if (error) {
			dev_info(&client->dev, "Trying alternate bootloader address\n");
			error = mxt_probe_bootloader(data, true);
			if (error) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}
		}

		/* OK, we are in bootloader, see if we can recover */
		if (++recovery_attempts > 1) {
			dev_err(&client->dev, "Could not recover from bootloader mode\n");
			/*
			 * We can reflash from this state, so do not
			 * abort initialization.
			 */
			data->in_bootloader = true;
			data->force_update_fw = true;
			return 0;
		}

		/* Attempt to exit bootloader into app mode */
		mxt_send_bootloader_cmd(data, false);
		msleep(MXT_FW_RESET_TIME);
	}

	error = mxt_check_retrigen(data);
	if (error)
		goto err_free_object_table;

	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object_table;

	error = mxt_sysfs_init(data);
	if (error)
		goto err_free_object_table;

	error = mxt_debug_msg_init(data);
	if (error)
		goto err_free_object_table;

	//Disable it because we update firmware by mxt_autoupdate_firmware_cfg()
	//if (data->cfg_name) {dev_info(&client->dev, "%s %d-\n", __func__, __LINE__);
	if (0) {
		error = request_firmware_nowait(THIS_MODULE, true,
					data->cfg_name, &data->client->dev,
					GFP_KERNEL, data, mxt_config_cb);
		if (error) {
			dev_err(&client->dev, "Failed to invoke firmware loader: %d\n",
				error);
			goto err_free_object_table;
		}
	} else {
		error = mxt_configure_objects(data, NULL, DO_NOT_CHECK_CFG_VER);
		if (error)
			goto err_free_object_table;
	}

	mxt_t81_configuration_area(data, MXT_T81_GOOD_AREA_OFFSET);
	dev_info(&client->dev, "%s -\n", __func__);

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };
	struct t7_config normal_config = { .active = active_rep, .idle = idle_rep };
	struct t7_config gesture_mode = { .active = gesture_active_rep, .idle = gesture_idle_rep };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else if (sleep == MXT_POWER_CFG_GESTURE)
		new_config = &gesture_mode;
	else
		new_config = &normal_config;
		//new_config = &data->t7_cfg;
	dev_dbg(dev, "Write T7 config, active = %d, idle = %d\n", new_config->active, new_config->idle);
	error = __mxt_write_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), new_config);
	if (error)
	{
		dev_err(dev, "Write T7 config fail, error = %d\n", error);
		return error;
	}
	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_dbg(dev, "T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			dev_dbg(dev, "T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	}

	dev_dbg(dev, "Initialized power cfg: ACTV %d, IDLE %d\n",
		data->t7_cfg.active, data->t7_cfg.idle);
	return 0;
}
static int mxt_set_t72_enable(struct mxt_data *data, u8 enable)
{
	struct device *dev = &data->client->dev;
	int error;
	dev_dbg(dev, "Set  T72 object, 0x%02X\n", enable);

	error = __mxt_write_reg(data->client, data->T72_address,
				sizeof(enable), &enable);
	if (error)
	{
		dev_err(dev, "Write T72 object fail, error = %d\n", error);
		return error;
	}

	return 0;
}

//Palm Supression+[
#if 0
static int mxt_set_t42_enable(struct mxt_data *data, u8 enable)
{
	struct device *dev = &data->client->dev;
	int error;
	dev_dbg(dev, "Set  T42 object, 0x%02X\n", enable);

	error = __mxt_write_reg(data->client, data->T42_address,
				sizeof(enable), &enable);
	if (error)
	{
		dev_err(dev, "Write T42 object fail, error = %d\n", error);
		return error;
	}

	return 0;
}
#endif
//Palm Supression+]


static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg, int check_cfg_ver)
{
	struct device *dev = &data->client->dev;
	int error;

	dev_err(dev, "%s +\n", __func__);
	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(dev, "Failed to initialize power cfg\n");
		goto err_free_object_table;
	}

	if (cfg) {
		error = mxt_update_cfg(data, cfg, check_cfg_ver);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	}

	if (data->multitouch) {
		error = mxt_initialize_input_device(data);
		if (error)
			goto err_free_object_table;
	} else {
		dev_warn(dev, "No touch object detected\n");
	}
	mxt_read_t38_data(data);
	dev_err(dev, "%s -\n", __func__);

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

// Read T38[0] and T38[1] from TP chip.
static int mxt_read_t38_data(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &data->client->dev;
	int error;
	struct mxt_object *object;
	u8 Ver1 = 0, Ver2 = 0;

	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object)
		return -EINVAL;
	error = __mxt_read_reg(client, object->start_address + 0, sizeof(Ver1), &Ver1);
	if (error)
		return error;
	error = __mxt_read_reg(client, object->start_address + 1, sizeof(Ver2), &Ver2);
	if (error)
		return error;
	data->maj_version = Ver1;
	data->min_version = Ver2;
	dev_info(dev, "T38 User data2 : %d.%d\n",data->maj_version,data->min_version);
	return 0;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_crc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = mxt_read_blks(data, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static int mxt_enter_bootloader(struct mxt_data *data)
{
	int ret;

	if (data->suspended) {
		if (data->pdata->suspend_mode == MXT_SUSPEND_REGULATOR)
			mxt_regulator_enable(data);

		data->suspended = false;
	}

	if (!data->in_bootloader) {
		disable_irq(data->irq);

		/* Change to the bootloader mode */
		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			return ret;

		msleep(MXT_RESET_TIME);

		/* Do not need to scan since we know family ID */
		ret = mxt_probe_bootloader(data, 0);
		if (ret)
			return ret;

		data->in_bootloader = true;
		mxt_debug_msg_remove(data);
		mxt_sysfs_remove(data);
#ifndef RE_REGISTER_INPUT_DEVICE
		mxt_free_input_device(data);
#endif
		mxt_free_object_table(data);
	}

	dev_dbg(&data->client->dev, "Entered bootloader\n");

	return 0;
}

static void mxt_fw_work(struct work_struct *work)
{
	struct mxt_flash *f =
		container_of(work, struct mxt_flash, work.work);
	mxt_check_bootloader(f->data);
}

static int mxt_load_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	data->flash = devm_kzalloc(dev, sizeof(struct mxt_flash), GFP_KERNEL);
	if (!data->flash)
		return -ENOMEM;

	data->flash->data = data;

	ret = request_firmware(&data->flash->fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		goto free;
	}

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, data->flash->fw);
	if (ret)
		goto release_firmware;

	init_completion(&data->flash->flash_completion);
	INIT_DELAYED_WORK(&data->flash->work, mxt_fw_work);
	reinit_completion(&data->flash->flash_completion);

	if (!data->in_bootloader) {
		ret = mxt_enter_bootloader(data);
		if (ret)
			goto release_firmware;
	}

	ret = mxt_acquire_irq(data);
	if (ret)
		goto release_firmware;

	/* Poll after 0.1s if no interrupt received */
	schedule_delayed_work(&data->flash->work, HZ / 10);
	dev_err(dev, "%s %d\n", __func__, __LINE__);
	/* Wait for flash. */
	ret = mxt_wait_for_completion(data, &data->flash->flash_completion,
				      MXT_BOOTLOADER_WAIT);
	dev_err(dev, "%s %d\n", __func__, __LINE__);
	disable_irq(data->irq);
	cancel_delayed_work_sync(&data->flash->work);
	data->in_bootloader = false;
release_firmware:
	release_firmware(data->flash->fw);
free:
	devm_kfree(dev, data->flash);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp)
		return -ENOMEM;

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct mxt_platform_data *pdata = data->pdata;
	const struct firmware *cfg;
	int ret;

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			enable_irq(data->irq);
			mxt_regulator_enable(data);
		} else if (pdata->suspend_mode == MXT_SUSPEND_DEEP_SLEEP) {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg, DO_NOT_CHECK_CFG_VER);
	if (ret)
		goto release;

	ret = count;

release:
	release_firmware(cfg);
out:
	data->updating_config = false;
	return ret;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);

static struct attribute *mxt_fw_attrs[] = {
	&dev_attr_update_fw.attr,
	NULL
};

static const struct attribute_group mxt_fw_attr_group = {
	.attrs = mxt_fw_attrs,
};

static ssize_t mxt_act_interval_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "active = %d, idle = %d\n", gesture_active_rep, gesture_idle_rep);
}
static ssize_t mxt_act_interval_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int active_int = 0, idle_int = 0;
	char *p =(char *)buf;

	if(count != 6 && count != 7)
	{
		dev_info(dev, "*buf = %s, count = %d\n", buf, (int)count);
		return count;
	}

	dev_info(dev, "*buf = %s, count = %d\n", buf, (int)count);

	sscanf(p, "%d ", &active_int);
	p+=3;
	sscanf(p, "%d ", &idle_int);
	dev_info(dev, "active_int = %d, idle_int = %d\n", active_int, idle_int);
	gesture_active_rep = active_int;
	gesture_idle_rep = idle_int;

	return count;
}
static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(config_crc, S_IRUGO, mxt_config_crc_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL,
		   mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);


static DEVICE_ATTR(act_interval, S_IWUSR | S_IRUSR, mxt_act_interval_show,
		   mxt_act_interval_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_config_crc.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_act_interval.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_sysfs_init(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		return error;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	error = sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr);
	if (error) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	return 0;

err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	return error;
}

static void mxt_sysfs_remove(struct mxt_data *data)
{
	struct i2c_client *client = data->client;

	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
}

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	int id;

	if (!input_dev)
		return;

	for (id = 0; id < data->num_touchids; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, 0, 0);
	}

	mxt_input_sync(data);
}

static int mxt_start(struct mxt_data *data)
{
	int ret;
	struct i2c_client *client = data->client;

	dev_info(&client->dev, "%s +, suspend_mode = %d, data->suspended = %d, double_tap_enable = %d, FP_enable = %d\n",__func__, data->pdata->suspend_mode, data->suspended, data->double_tap_enable, data->FP_enable);

	if (!data->suspended || data->in_bootloader)
	{
		dev_info(&client->dev, "%s , data->suspended = %d, data->in_bootloader = %d\n",__func__, data->suspended, data->in_bootloader);
		return 0;
	}

	switch (data->pdata->suspend_mode) {
	case MXT_SUSPEND_T9_CTRL:
		mxt_soft_reset(data);

		/* Touch enable */
		/* 0x83 = SCANEN | RPTEN | ENABLE */
		mxt_write_object(data,
				MXT_TOUCH_MULTI_T9, MXT_T9_CTRL, 0x83);
		break;

	case MXT_SUSPEND_REGULATOR:
		enable_irq(data->irq);
		mxt_regulator_enable(data);
		break;

	case MXT_SUSPEND_DEEP_SLEEP:
	default:
		/*
		 * Discard any touch messages still in message buffer
		 * from before chip went to sleep
		 */
		mxt_process_messages_until_invalid(data);

#if 0
		ret = mxt_t100_configuration(data, MXT_T100_MOVING_FILTER_OFFSET, MXT_T100_MOVING_FILTER_RECAL);
		if (ret)
			return ret;
#endif
		ret = mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		if (ret)
			return ret;
#if 0
		ret = mxt_t80_configuration(data, MXT_T80_COMPTHR_OFFSET, MXT_T80_DISABLE);
		if (ret)
			return ret;
#endif
		ret = mxt_t8_configuration(data, MXT_T8_RECAL_OFFSET, MXT_T8_ENABLE);
		if (ret)
			return ret;

		ret = mxt_set_t72_enable(data, 0x7D);

		if(ret)
			dev_err(&client->dev, "%s set T72 error, ret = %d\n", __func__, ret);

		/* Recalibrate since chip has been in deep sleep */
		//Set wait flag to true to prevent noise.
		ret = mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, true);
		if (ret)
			return ret;

		ret = mxt_t8_configuration(data, MXT_T8_RECAL_OFFSET, MXT_T8_DISABLE);
		if (ret)
			return ret;
#if 0
		ret = mxt_t80_configuration(data, MXT_T80_COMPTHR_OFFSET, MXT_T80_ENABLE);
		if (ret)
			return ret;
#endif
		ret = mxt_set_t100_multitouchscreen_cfg(data, 0, (MXT_T100_CTRL_ENABLE | MXT_T100_CTRL_RPTEN | MXT_T100_CTRL_DISSCRMSG | MXT_T100_CTRL_SCANEN)); //SW-PROD-DH-TP_SUSPEND-RESUME-00+
		if (ret)
			return ret;
#if 0
		ret = mxt_t100_configuration(data, MXT_T100_MOVING_FILTER_OFFSET, MXT_T100_MOVING_FILTER_NORMAL);
		if (ret)
			return ret;
#endif
		if (ret)
			return ret;

		//SW-PROD-DH-DOUBLE_TAP-00+[
		if (data->T93_address)
		{
			mxt_t93_configuration(data, 0, MXT_T93_DISABLE);
		}
		//SW-PROD-DH-TDOUBLE_TAP-00+]

		if(data->double_tap_enable == 0 && data->FP_enable== 0)
		{
			ret = mxt_acquire_irq(data);
			if (ret)
				return ret;
		}
		else
			disable_irq_wake(data->irq);
		break;
	}

	data->suspended = false;
	dev_info(&client->dev, "%s -\n",__func__);
	return 0;
}

static int mxt_stop(struct mxt_data *data)
{
	int ret;
	int i;
	struct i2c_client *client = data->client;

	dev_info(&client->dev, "%s +, suspend mode = %d, double_tap_enable = %d, FP_enable = %d\n",__func__, data->pdata->suspend_mode, data->double_tap_enable, data->FP_enable);
	if (data->suspended || data->in_bootloader || data->updating_config)
	{
		dev_info(&client->dev, "%s , data->suspended = %d, data->in_bootloader = %d, data->updating_config = %d, \n",__func__, data->suspended, data->in_bootloader, data->updating_config);
		return 0;
	}

	switch (data->pdata->suspend_mode) {
	case MXT_SUSPEND_T9_CTRL:
		/* Touch disable */
		ret = mxt_write_object(data,
				MXT_TOUCH_MULTI_T9, MXT_T9_CTRL, 0);
		if (ret)
			return ret;

		break;

	case MXT_SUSPEND_REGULATOR:
		disable_irq(data->irq);
		mxt_regulator_disable(data);
		mxt_reset_slots(data);
		break;

	case MXT_SUSPEND_DEEP_SLEEP:
	default:
		disable_irq(data->irq);

		//SW-PROD-DH-TP_DOUBLE_TAP-00+[
		if(data->double_tap_enable == 1 || data->FP_enable == 1)
		{
			ret = mxt_set_t7_power_cfg(data, MXT_POWER_CFG_GESTURE);
			if(ret)
				dev_err(&client->dev, "Failed to write T7 Power Gesture mode, ret = %d\n", ret);
			if(data->double_tap_enable == 1)
			{
				ret = mxt_t93_configuration(data, 0, MXT_T93_ENABLE);
				if(ret)
					dev_err(&client->dev, "Failed to write T93 Enable, ret = %d\n", ret);
			}
		}
		//SW-PROD-DH-TP_DOUBLE_TAP-00+]
		else {
		ret = mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
		if (ret)
			return ret;
		}
		ret = mxt_set_t72_enable(data, 0x0);
		if(ret)
			dev_err(&client->dev, "Failed to write T72 Disable, ret = %d\n", ret);
		mxt_set_t100_multitouchscreen_cfg(data, 0, (MXT_T100_CTRL_ENABLE | MXT_T100_CTRL_DISSCRMSG | MXT_T100_CTRL_SCANEN));
		mxt_reset_slots(data);
		data->long_press_sent = false;

		for (i=0; i < LONG_PRESSE_TOUCH_COUNT; i++)
		{
			x_value[i] = 0;
			y_value[i] = 0;
			node_value[i] = 0;
			pv_value[i] = 0;
		}

		data->long_press_touch_id = 0xFF;
		//SW-PROD-DH-TP_DOUBLE_TAP-00+[
		if(data->double_tap_enable == 1 || data->FP_enable == 1)
		{
			enable_irq(data->irq);
			enable_irq_wake(data->irq);	
		}
		//SW-PROD-DH-TP_DOUBLE_TAP-00+]
		break;
	}
	data->suspended = true;
	dev_info(&client->dev, "%s -\n",__func__);
	return 0;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int ret;
	dev_info(&data->client->dev, "%s \n", __func__);
	ret = mxt_start(data);

	if (ret)
		dev_err(&data->client->dev, "%s failed rc=%d\n", __func__, ret);

	return ret;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int ret;
	dev_info(&data->client->dev, "%s \n", __func__);
	ret = mxt_stop(data);

	if (ret)
		dev_err(&data->client->dev, "%s failed rc=%d\n", __func__, ret);
}

#ifdef CONFIG_OF
static const struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;
	struct device_node *np = client->dev.of_node;
	u32 *keymap;
	int proplen, ret;

	if (!np)
		return ERR_PTR(-ENOENT);

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->gpio_reset = of_get_named_gpio_flags(np, "atmel,reset-gpio",
						    0, NULL);
	ret = pdata->gpio_irq = of_get_named_gpio_flags(np, "atmel,irq-gpio",
						    0, NULL);

	ret = of_property_read_string(np, "atmel,cfg_name", &pdata->cfg_name);
		if (ret)
			dev_err(&client->dev,  "Couldn't read atmel,cfg_name: %d\n", ret);

	ret = of_property_read_string(np, "atmel,fw_name", &pdata->fw_name);
		if (ret)
			dev_err(&client->dev,  "Couldn't read atmel,fw_name: %d\n", ret);

	of_property_read_string(np, "atmel,input_name", &pdata->input_name);

	if (of_find_property(np, "linux,gpio-keymap", &proplen)) {
		pdata->t19_num_keys = proplen / sizeof(u32);

		keymap = devm_kzalloc(&client->dev,
				pdata->t19_num_keys * sizeof(keymap[0]),
				GFP_KERNEL);
		if (!keymap)
			return ERR_PTR(-ENOMEM);

		ret = of_property_read_u32_array(np, "linux,gpio-keymap",
						 keymap, pdata->t19_num_keys);
		if (ret)
			dev_warn(&client->dev,
				 "Couldn't read linux,gpio-keymap: %d\n", ret);

		pdata->t19_keymap = keymap;
	}

	of_property_read_u32(np, "atmel,suspend-mode", &pdata->suspend_mode);

	of_property_read_u32(np, "fih,enable_vdd_by_gpio", &pdata->enable_vdd_by_gpio);
	dev_info(&client->dev, "i2c address = 0x%x, enable_vdd_by_gpio = %d\n", client->addr, pdata->enable_vdd_by_gpio);


	return pdata;
}
#else
static const struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	return ERR_PTR(-ENOENT);
}
#endif

#ifdef CONFIG_ACPI

struct mxt_acpi_platform_data {
	const char *hid;
	struct mxt_platform_data pdata;
};

static unsigned int samus_touchpad_buttons[] = {
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	BTN_LEFT
};

static struct mxt_acpi_platform_data samus_platform_data[] = {
	{
		/* Touchpad */
		.hid	= "ATML0000",
		.pdata	= {
			.t19_num_keys	= ARRAY_SIZE(samus_touchpad_buttons),
			.t19_keymap	= samus_touchpad_buttons,
		},
	},
	{
		/* Touchscreen */
		.hid	= "ATML0001",
	},
	{ }
};

static unsigned int chromebook_tp_buttons[] = {
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	BTN_LEFT
};

static struct mxt_acpi_platform_data chromebook_platform_data[] = {
	{
		/* Touchpad */
		.hid	= "ATML0000",
		.pdata	= {
			.t19_num_keys	= ARRAY_SIZE(chromebook_tp_buttons),
			.t19_keymap	= chromebook_tp_buttons,
		},
	},
	{
		/* Touchscreen */
		.hid	= "ATML0001",
	},
	{ }
};

static const struct dmi_system_id mxt_dmi_table[] = {
	{
		/* 2015 Google Pixel */
		.ident = "Chromebook Pixel 2",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "GOOGLE"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Samus"),
		},
		.driver_data = samus_platform_data,
	},
	{
		/* Other Google Chromebooks */
		.ident = "Chromebook",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "GOOGLE"),
		},
		.driver_data = chromebook_platform_data,
	},
	{ }
};

static const struct mxt_platform_data *mxt_parse_acpi(struct i2c_client *client)
{
	struct acpi_device *adev;
	const struct dmi_system_id *system_id;
	const struct mxt_acpi_platform_data *acpi_pdata;

	/*
	 * Ignore ACPI devices representing bootloader mode.
	 *
	 * This is a bit of a hack: Google Chromebook BIOS creates ACPI
	 * devices for both application and bootloader modes, but we are
	 * interested in application mode only (if device is in bootloader
	 * mode we'll end up switching into application anyway). So far
	 * application mode addresses were all above 0x40, so we'll use it
	 * as a threshold.
	 */
	if (client->addr < 0x40)
		return ERR_PTR(-ENXIO);

	adev = ACPI_COMPANION(&client->dev);
	if (!adev)
		return ERR_PTR(-ENOENT);

	system_id = dmi_first_match(mxt_dmi_table);
	if (!system_id)
		return ERR_PTR(-ENOENT);

	acpi_pdata = system_id->driver_data;
	if (!acpi_pdata)
		return ERR_PTR(-ENOENT);

	while (acpi_pdata->hid) {
		if (!strcmp(acpi_device_hid(adev), acpi_pdata->hid))
			return &acpi_pdata->pdata;

		acpi_pdata++;
	}

	return ERR_PTR(-ENOENT);
}
#else
static const struct mxt_platform_data *mxt_parse_acpi(struct i2c_client *client)
{
	return ERR_PTR(-ENOENT);
}
#endif

static struct mxt_platform_data *mxt_default_pdata(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* Set default parameters */
	pdata->irqflags = IRQF_TRIGGER_FALLING;

	return pdata;
}

static const struct mxt_platform_data *
mxt_get_platform_data(struct i2c_client *client)
{
	const struct mxt_platform_data *pdata;

	pdata = dev_get_platdata(&client->dev);
	if (pdata)
		return pdata;

	pdata = mxt_parse_dt(client);
	if (!IS_ERR(pdata) || PTR_ERR(pdata) != -ENOENT)
		return pdata;

	pdata = mxt_parse_acpi(client);
	if (!IS_ERR(pdata) || PTR_ERR(pdata) != -ENOENT)
		return pdata;

	pdata = mxt_default_pdata(client);
	if (!IS_ERR(pdata))
		return pdata;

	dev_err(&client->dev, "No platform data specified\n");
	return ERR_PTR(-EINVAL);
}

static int mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mxt_data *data;
	const struct mxt_platform_data *pdata;
	int error;

	if(tp_probe_success)
	{
		printk("%s: Another driver probe success. skip atmel touch driver!\n", __func__);
		return 0;
	}
	pdata = mxt_get_platform_data(client);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	dev_info(&client->dev, "%s +\n", __func__);
	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	data->pdata = pdata;
	i2c_set_clientdata(client, data);

	if (data->pdata->cfg_name)
		mxt_update_file_name(&data->client->dev,
				     &data->cfg_name,
				     data->pdata->cfg_name,
				     strlen(data->pdata->cfg_name));

	if (data->pdata->fw_name)
		mxt_update_file_name(&data->client->dev,
				     &data->fw_name,
				     data->pdata->fw_name,
				     strlen(data->pdata->fw_name));

	dev_info(&client->dev, "%s data->fw_name: %s, data->cfg_name: %s \n", __func__, data->fw_name, data->cfg_name);

	init_completion(&data->chg_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);

	error = gpio_request(data->pdata->gpio_reset, "reset-gpio");
	if (error)
	{
		pr_err("%s: Failed to get gpio %d (code: %d)",	__func__, data->pdata->gpio_reset, error);
		goto err_free_irq;
	}

	error = gpio_direction_output(data->pdata->gpio_reset, 0);
	if (error)
		goto err_free_irq;

	error = gpio_request(data->pdata->gpio_irq, "irq-gpio");
	if (error)
	{
		pr_err("%s: Failed to get gpio %d (code: %d)",	__func__, data->pdata->gpio_irq, error);
		goto err_free_irq;
	}

	error = gpio_direction_input(data->pdata->gpio_irq);
	if (error)
		goto err_free_irq;

	//Enable regulator even if suspend mode = deepsleep
	if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR || pdata->suspend_mode == MXT_SUSPEND_DEEP_SLEEP) {
		error = mxt_acquire_irq(data);
		if (error)
			goto err_free_mem;
		disable_irq(data->irq);

		error = mxt_probe_regulators(data);
		if (error)
			goto err_free_irq;
	}

	error = sysfs_create_group(&client->dev.kobj, &mxt_fw_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating fw sysfs group\n",
			error);
		return error;
	}

	error = mxt_initialize(data);
	if (error)
		goto err_free_irq;
	data->dsi_panel_notif.notifier_call = dsi_panel_notifier_callback;

	error = msm_drm_register_client(&data->dsi_panel_notif);
	if (error) {
		dev_err(&client->dev, "Unable to register dsi_panel_notifier: %d\n", error);
		goto err_unregister_notifier;
	}

	if(data->info != NULL)
		dev_info(&client->dev, "%s Panel fw Version %02X:%02X, config Version : %02X.%02X\n", __func__, data->info->version, data->info->build, data->maj_version,data->min_version);
	else
	{
		dev_err(&client->dev, "data->info = NULL\n");
		if(data->force_update_fw == false)
			goto err_unregister_notifier;
	}
	wake_lock_init(&data->mxt_wake_lock, WAKE_LOCK_SUSPEND, "atmel_wake_lock");
	data->mxt_wq = create_singlethread_workqueue("mxt_wq");
	INIT_DELAYED_WORK(&data->touch_chip_firmware_upgrade_work, mxt_autoupdate_firmware_cfg);
	queue_delayed_work(data->mxt_wq, &data->touch_chip_firmware_upgrade_work, 1*HZ);
	data->FP_enable = 0;
	data->T81_enable = 0;
	data->long_press_time = 6; //6 x 32 ms = 192 ms
	data->long_press_touch_id = 0xFF;

	g_data = data;
	touch_cb.touch_tpfwver_read = touch_tpfwver_read;
	touch_cb.touch_double_tap_read = touch_double_tap_read;
	touch_cb.touch_double_tap_write = touch_double_tap_write;
	touch_cb.touch_selftest = touch_selftest;
	touch_cb.touch_selftest_result = touch_selftest_result_read;
	touch_cb.touch_long_press_read = touch_long_press_read;
	touch_cb.touch_long_press_write = touch_long_press_write;
	touch_cb.touch_fp_en_read = touch_fp_en_read;
	touch_cb.touch_fp_en_write = touch_fp_en_write;
	touch_cb.touch_prox_status_write = touch_prox_status_write;
	touch_cb.touch_prox_status_read = touch_prox_status_read;
	touch_cb.touch_long_press_time_read = touch_long_press_time_read;
	touch_cb.touch_long_press_time_write = touch_long_press_time_write;
	touch_cb.touch_long_press_size_read = touch_long_press_size_read;
	touch_cb.touch_long_press_size_write = touch_long_press_size_write;
	tp_probe_success = 1;

	dev_info(&client->dev, "%s -\n", __func__);
	return 0;

err_unregister_notifier:
	msm_drm_unregister_client(&data->dsi_panel_notif);
err_free_irq:
	if (data->irq)
		free_irq(data->irq, data);

	gpio_free(data->gpio_vdd_1p8_ldo);
	gpio_free(data->pdata->gpio_reset);
	gpio_free(data->pdata->gpio_irq);
	if(data->reg_vdd) 	regulator_put(data->reg_vdd);
	if(data->reg_avdd) regulator_put(data->reg_avdd);
err_free_mem:
	kfree(data);
	dev_info(&client->dev, "%s error\n", __func__);
	BBOX_TOUCH_PROBE_FAIL
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	dev_info(&client->dev, "%s \n", __func__);

	sysfs_remove_group(&client->dev.kobj, &mxt_fw_attr_group);
	mxt_debug_msg_remove(data);
	mxt_sysfs_remove(data);
	msm_drm_unregister_client(&data->dsi_panel_notif);

	if (data->irq)
		free_irq(data->irq, data);

	if(data->pdata->enable_vdd_by_gpio)
		gpio_free(data->gpio_vdd_1p8_ldo);

	gpio_free(data->pdata->gpio_reset);
	gpio_free(data->pdata->gpio_irq);
	wake_lock_destroy(&data->mxt_wake_lock);

	if(data->reg_avdd) regulator_put(data->reg_avdd);
	if(data->reg_vdd) regulator_put(data->reg_vdd);
	mxt_free_input_device(data);
	mxt_free_object_table(data);
	kfree(data);

	return 0;
}

static int __maybe_unused mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int __maybe_unused mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

int mxt_touch_state(int state)
{
	struct input_dev *input_dev;

	if(g_data == NULL)
	{
		printk("%s, g_data = NULL, No Solomon Touch found\n", __func__);
		return 0;
	}
	input_dev = g_data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
	{
		switch(state)
		{
			case TOUCH_SUSPEND:
				mxt_stop(g_data);
				break;
			case TOUCH_RESUME:
				mxt_start(g_data);
				break;
			default:
				dev_err(&g_data->client->dev, "%s,  Unknown command, state = %d\n",__func__, state);
				break;
		}
	}

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int __maybe_unused mxt_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	dev_dbg(&client->dev, "%s \n", __func__);
	is_i2c_bus_suspend = 1;

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int __maybe_unused mxt_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	dev_dbg(&client->dev, "%s \n", __func__);
	is_i2c_bus_suspend = 0;

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_pm_suspend, mxt_pm_resume);

static void touch_tpfwver_read(char *fw_ver)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return;
	}
	if(g_data->info == NULL)
	{
		printk("%s, g_data->info = NULL\n", __func__);
		snprintf(fw_ver, 128, "TP-0.0.00_%02x%02X\n", g_data->maj_version, g_data->min_version);
		return;
	}
	snprintf(fw_ver, 128, "TP-%X.%X.%02X_%02X%02X\n", g_data->info->version >> 4, g_data->info->version & 0xf, g_data->info->build, g_data->maj_version, g_data->min_version);
}

unsigned int touch_prox_status_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, prox_status = %d\n", __func__, g_data->prox_status);
	return g_data->prox_status;
}
int touch_prox_status_write(unsigned int enable)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, Set prox_status = %d\n", __func__, enable);
	g_data->prox_status = enable;
	mutex_lock(&g_data->input_dev->mutex);
	if(g_data->suspended && (g_data->double_tap_enable || g_data->FP_enable))
	{
		if(g_data->prox_status)
			mxt_set_t7_power_cfg(g_data, MXT_POWER_CFG_DEEPSLEEP);
		else
			mxt_set_t7_power_cfg(g_data, MXT_POWER_CFG_GESTURE);
	}
	mutex_unlock(&g_data->input_dev->mutex);
	return 0;
}

static unsigned int touch_fp_en_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, FP_enable = %d\n", __func__, g_data->FP_enable);
	return g_data->FP_enable;
}
static int touch_fp_en_write(unsigned int enable)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, Set FP_enable = %d\n", __func__, enable);
	g_data->FP_enable = enable;
	return 0;
}

static unsigned int touch_long_press_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, touch_long_press unlock = %d\n", __func__, g_data->T81_enable);
	return g_data->T81_enable;
}
static int touch_long_press_write(unsigned int enable)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, Set long_press enable = %d\n", __func__, enable);
	g_data->T81_enable = enable;
	//Disable palm supression when long press enable.
#if 0
	mxt_set_t42_enable(g_data, !enable);
#endif
	if(enable) {
		mxt_t81_configuration(g_data, MXT_T81_ENABLE_OFFSET, MXT_T81_CTRL_PRESSREQ |MXT_T81_CTRL_RPTEN |MXT_T81_CTRL_ENABLE);
		mxt_t136_configuration(g_data, MXT_T136_ENABLE_OFFSET, 0x0A);
		dev_info(&g_data->client->dev, "%s, MXT_T136_VALUE = 0x0A\n", __func__);
	} else {
		mxt_t81_configuration(g_data, MXT_T81_ENABLE_OFFSET, MXT_T81_CTRL_PRESSREQ |MXT_T81_CTRL_ENABLE);
		mxt_t136_configuration(g_data, MXT_T136_ENABLE_OFFSET, 0x02);
		dev_info(&g_data->client->dev, "%s, MXT_T136_VALUE = 0x02\n", __func__);
    }
	return 0;
}

static unsigned int touch_long_press_time_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, touch_long_press_time = %d\n", __func__, g_data->long_press_time);
	return g_data->long_press_time;
}
static int touch_long_press_time_write(unsigned int long_press_time)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, Set long_press_time = %d\n", __func__, long_press_time);
	g_data->long_press_time = long_press_time;
	mxt_t81_configuration(g_data, MXT_T81_TOUCH_TIME, long_press_time);
	return 0;
}
static unsigned int touch_long_press_size_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, touch_long_press_size = %d\n", __func__, g_data->long_press_size);
	return g_data->long_press_size;
}
static int touch_long_press_size_write(unsigned int long_press_size)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, Set long_press_size = %d\n", __func__, long_press_size);
	g_data->long_press_size = long_press_size;
	return 0;
}


static unsigned int touch_double_tap_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, touch_double_tap_read double_tap = %d\n", __func__, g_data->double_tap_enable);
	return g_data->double_tap_enable;
}
static int touch_double_tap_write(unsigned int enable)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, Set double_tap enable = %d\n", __func__, enable);
	g_data->double_tap_enable = enable;
	return 0;
}
static int touch_selftest_result_read(void)
{
	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		return -1;
	}
	dev_info(&g_data->client->dev, "%s, touch_selftest_result_read selftest result = %d\n", __func__, Touch_TestResult);
	return Touch_TestResult;
}
static void touch_selftest(void)
{
	char* argv_test[] = {"/vendor/bin/mxt-app-PIE", "-t ", NULL};
	char* argv_dump[] = {"/vendor/bin/mxt-app-PIE", "-v",  "0",  "--reference",  "--debug-dump",  "/data/vendor/misc/touch/TPRawdataFile.txt", NULL};

	int iRet = 0;

	if(g_data == NULL)
	{
		printk("%s, g_data = NULL\n", __func__);
		Touch_TestResult = TEST_INVALID;
		return ;
	}
	dev_info(&g_data->client->dev, "%s, Do touch selftest \n", __func__);

	//Call mxt-app to do self-test
	iRet = call_usermodehelper(argv_test [0], argv_test, NULL, UMH_WAIT_PROC);

	Touch_TestResult = (iRet == 0)? TEST_PASS: TEST_FAIL;

	dev_info(&g_data->client->dev, "iRet of selftest: %d, TestResult = %d\n", iRet, Touch_TestResult);

	//3. Call mxt-app to generate raw data
	iRet = call_usermodehelper(argv_dump [0], argv_dump, NULL, UMH_WAIT_PROC);
	dev_info(&g_data->client->dev, "iRet of dump: %d\n", iRet);

	return;
}

//Reference from mxt_prepare_cfg_mem()
static int mxt_cfg_read_version(struct mxt_data *data, const struct firmware *fw, unsigned int data_pos, int config_match)
{
	struct device *dev = &data->client->dev;
	unsigned int type, instance, size;
	int offset, i;
	int ret = 0;
	u8 val = 0;
	u8 cfg_maj_version = 0;
	u8 cfg_min_version = 0;

	while (data_pos < fw->size)
	{
		/* Read type, instance, length */
		ret = sscanf(fw->data + data_pos, "%x %x %x%n", &type, &instance, &size, &offset);
		if (ret == 0)
		{
			/* EOF */
			break;
		}
		else if (ret != 3)
		{
			dev_err(dev, "Bad format: failed to parse object\n");
			return -EINVAL;
		}
		data_pos += offset;
		if(type == 0x0026)
		{
			for (i = 0; i < 2; i++)
			{
				ret = sscanf(fw->data + data_pos, "%hhx%n", &val, &offset);

				if(i == 0)
					cfg_maj_version = val;
				if(i == 1)
					cfg_min_version = val;
				if (ret != 1)
				{
					dev_err(dev, "Bad format in T%d at %d\n",	type, i);
					return -EINVAL;
				}
				data_pos += offset;
			}
			break;
		}
		else
		{
			data_pos += 3*size;
		}
	}
	dev_info(dev, "%s, File config version(%02X.%02X) , chip config version(%02X.%02X)\n", __func__, cfg_maj_version, cfg_min_version, data->maj_version,data->min_version);
	if((cfg_maj_version >= data->maj_version) && (cfg_min_version > data->min_version))
	{
		dev_info(dev, "%s, File config version(%02X.%02X) is large than chip config version(%02X.%02X)\n", __func__, cfg_maj_version, cfg_min_version, data->maj_version,data->min_version);
		return 1;
	}
	//Disable it because touch firmware adopt dynamic setting, confic_crc might be changed during touch auto-calibration.
	/*
	if((cfg_maj_version == data->maj_version) && (cfg_min_version == data->min_version) && config_match == 0)
	{
		dev_info(dev, "%s, File config version(%02X.%02X) and chip config version(%02X.%02X) is the same, but checksum is difficult\n", __func__, cfg_maj_version, cfg_min_version, data->maj_version,data->min_version);
		return 1;
	}
	*/
	return 0;
}

//Reference from mxt_update_fw_store()
static int mxt_auto_update_firmware(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct firmware *fw;
	int ret = 0;

	dev_info(dev, "%s +, Starting\n", __func__);
	//If touch stay in bootloader in mxt_initialize, force update FW.
	if(data->in_bootloader == true && data->force_update_fw == true)
	{
		dev_info(dev, "%s Touch is in bootloader, Force update FW\n", __func__);
		goto Force_update_FW;
	}
	ret = request_firmware(&fw, data->cfg_name, dev);
	if (ret < 0)
	{
		dev_err(dev, "Failure to request config file %s\n",  data->cfg_name);
		ret = -ENOENT;
		goto release;
	}
	ret = mxt_update_cfg(data, fw, GET_FW_CFG_VER_ONLY);
	if (ret < 0)
	{
		dev_err(dev, "Failure to Get config file %s, ret = %d\n",  data->cfg_name, ret);
		goto release;
	}

	dev_info(dev, "%s +, panel fw version = (%02X.%02X), fw file version (%02X.%02X)\n", __func__, data->info->version, data->info->build, fw_file_version, fw_file_build);
	if((fw_file_version > data->info->version) || (fw_file_build > data->info->build))
	{
Force_update_FW:
		dev_info(dev, "%s, call mxt_load_fw() \n", __func__);
		wake_lock(&data->mxt_wake_lock);
		ret = mxt_load_fw(dev);
		wake_unlock(&data->mxt_wake_lock);

		if(ret)
		{
			dev_err(dev, "%s, update_fw() fail, ret = %d\n", __func__, ret);
			if(data->force_update_fw)
			{
				BBOX_TOUCH_PROBE_FAIL
			}
		}
		else
		{
			data->suspended = false;
			ret = mxt_initialize(data);
			//Update touch config if touch FW been updated.
			data->force_update_config = true ;
		}

	}
	else
		dev_info(&data->client->dev, "%s, skip fw update\n", __func__);
	dev_info(dev, "%s -, panel fw version = (%02X.%02X), fw file version (%02X.%02X)\n", __func__, data->info->version, data->info->build, fw_file_version, fw_file_build);
release:
	release_firmware(fw);
	return ret;
}

//Reference from mxt_update_cfg_store()
static int mxt_auto_update_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	const struct firmware *fw;
	int ret =0;
	dev_info(dev, "%s +, panel config version = (%02X.%02X)\n", __func__, data->maj_version, data->min_version);

	if (data->in_bootloader)
	{
		dev_err(&data->client->dev, "%s Not in app mode\n", __func__);
		return -EINVAL;
	}
	wake_lock(&data->mxt_wake_lock);
	ret = request_firmware(&fw, data->cfg_name, dev);
	if (ret < 0)
	{
		dev_err(dev, "Failure to request config file %s\n",  data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

#ifndef RE_REGISTER_INPUT_DEVICE
	mxt_free_input_device(data);
#endif
	if (data->suspended) {
		if (pdata->suspend_mode == MXT_SUSPEND_REGULATOR) {
			enable_irq(data->irq);
			mxt_regulator_enable(data);
		} else if (pdata->suspend_mode == MXT_SUSPEND_DEEP_SLEEP) {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}
	if(data->force_update_fw == true || data->force_update_config == true)
	{
		ret = mxt_configure_objects(data, fw, DO_NOT_CHECK_CFG_VER);
		data->force_update_fw = false;
		data->force_update_config = false;
	}
	else
		ret = mxt_configure_objects(data, fw, CHECK_CFG_VER);
	if (ret)
		goto release;

release:
	release_firmware(fw);
out:
	data->updating_config = false;
	wake_unlock(&data->mxt_wake_lock);
	dev_info(dev, "%s -, panel config version = (%02X.%02X)\n", __func__, data->maj_version, data->min_version);
	printk("BBox::EHCS:51202:i:Firmware Version=%X.%X.%02X_%02X%02X", g_data->info->version >> 4, g_data->info->version & 0xf, g_data->info->build, g_data->maj_version, g_data->min_version);
	return ret;
}

static void mxt_autoupdate_firmware_cfg(struct work_struct *work)
{
	struct device *dev = &g_data->client->dev;
	int ret = 0;

	if(g_data == NULL)
	{
		dev_err(dev, "%s, g_data = NULL, skip firmware check\n", __func__);
		return;
	}
	if(g_data->fw_name == NULL || g_data->cfg_name == NULL)
	{
		dev_err(dev, "%s, Could not found fw/config file\n", __func__);
		return;
	}

	//Check firmware first, because config might be dependent with firmware.
	ret = mxt_auto_update_firmware(g_data);
	if(ret != 0)
	{
		if(ret == -ENOENT)
		{
			return;
		}
		dev_err(dev, "%s, update_firmware FAIL, ret = %d\n", __func__, ret);
		BBOX_TOUCH_FIRMWARE_UPDATE_FAIL
		return;
	}
	mdelay(100);
	ret = mxt_auto_update_cfg(g_data);
	if(ret != 0)
	{
		dev_err(dev, "%s, update_config FAIL, ret = %d\n", __func__, ret);
		BBOX_TOUCH_FIRMWARE_UPDATE_FAIL
	}
}


static const struct of_device_id mxt_of_match[] = {
	{ .compatible = "atmel,maxtouch", },
	{},
};

MODULE_DEVICE_TABLE(of, mxt_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id mxt_acpi_id[] = {
	{ "ATML0000", 0 },	/* Touchpad */
	{ "ATML0001", 0 },	/* Touchscreen */
	{ }
};
MODULE_DEVICE_TABLE(acpi, mxt_acpi_id);
#endif

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "maxtouch", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mxt_of_match),
		.acpi_match_table = ACPI_PTR(mxt_acpi_id),
		.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

module_i2c_driver(mxt_driver);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
