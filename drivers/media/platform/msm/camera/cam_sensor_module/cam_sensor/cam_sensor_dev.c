/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "cam_sensor_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_sensor_soc.h"
#include "cam_sensor_core.h"
#include "cam_sensor_i2c.h"
#include "cam_sync_api.h"

#include <linux/circ_buf.h>
#include <media/cam_defs.h>
#include <media/msm_light_messaging.h>

#define LIGHT_MAX_I2C_QUEUE_SIZE 64

typedef struct queueEntry
{
    int32_t    size;
    uint8_t    buffer[LIGHT_MAX_MESSAGE_SIZE];
} qEntry_t;

/*
 * Circular queue used to hold I2C messages received from
 * the ASIC.  This is a lockless queue because the
 * interrupt routine only increments the head and the sysfs
 * routine only increments tail.
 *
 * They each read the other's index but this does not require a
 * lock.
 */
typedef struct circBuf {
    int32_t         head;
    int32_t         tail;
    qEntry_t        i2cResponses[LIGHT_MAX_I2C_QUEUE_SIZE];
} circBuf_t;

typedef struct i2cStats
{
    uint32_t      messagesReceived;
    uint32_t      messagesDropped;
    uint32_t      messagesSent;
    uint32_t      bytesSent;
    uint32_t      bytesReceived;
    uint32_t      bytesDropped;
    uint32_t      sysfsReads;
    uint32_t      lccReadShow;
    uint32_t      lccReadStore;
    uint32_t      lccWriteStore;
    uint32_t      lccWriteBinary;
    uint32_t      writeAsicPower;
    uint32_t      readAsicPower;
    uint32_t      i2cResponseCalled;
    uint32_t      asicInterruptCalled;
} i2cStats_t;

static i2cStats_t _i2cStats;
static circBuf_t _lightI2cQueue;

#define FTM
#ifdef FTM
static int8_t sensor_id_sysfs = 0;
extern int8_t g_camera_ping;
#endif
#if defined(CONFIG_SERIAL_MSM_GENI)
extern int asic_uart_clk_enable(int line, int on);
#endif

static long cam_sensor_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_sensor_driver_cmd(s_ctrl, arg);
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid ioctl cmd: %d", cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int cam_sensor_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "s_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	cam_sensor_shutdown(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return 0;
}

#ifdef CONFIG_COMPAT
static long cam_sensor_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_SENSOR, "Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_sensor_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR, "cam_sensor_subdev_ioctl failed");
			break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid compat ioctl cmd_type: %d", cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_SENSOR,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}

#endif
static struct v4l2_subdev_core_ops cam_sensor_subdev_core_ops = {
	.ioctl = cam_sensor_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_sensor_init_subdev_do_ioctl,
#endif
	.s_power = cam_sensor_power,
};

static struct v4l2_subdev_ops cam_sensor_subdev_ops = {
	.core = &cam_sensor_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_sensor_internal_ops = {
	.close = cam_sensor_subdev_close,
};

static int cam_sensor_init_subdev_params(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	s_ctrl->v4l2_dev_str.internal_ops =
		&cam_sensor_internal_ops;
	s_ctrl->v4l2_dev_str.ops =
		&cam_sensor_subdev_ops;
	strlcpy(s_ctrl->device_name, CAMX_SENSOR_DEV_NAME,
		sizeof(s_ctrl->device_name));
	s_ctrl->v4l2_dev_str.name =
		s_ctrl->device_name;
	s_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	s_ctrl->v4l2_dev_str.ent_function =
		CAM_SENSOR_DEVICE_TYPE;
	s_ctrl->v4l2_dev_str.token = s_ctrl;

	rc = cam_register_subdev(&(s_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_SENSOR, "Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static struct kobject *asic_sysfs_kobj = NULL;
static uint16_t asic_i2c_read_bytes = 50;
static uint32_t asic_reg_addr = 0;

static ssize_t asic_i2c_read_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint32_t *block_data;
	int i = 0, rc =0;
	struct i2c_client *m_client = to_i2c_client(dev);

	block_data = kzalloc(asic_i2c_read_bytes, GFP_KERNEL);
	if (!block_data) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s:%d no memory", __func__, __LINE__);
		return -ENOMEM;
	}

	rc = cam_qup_i2c_read(m_client,asic_reg_addr, block_data, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc < 0){
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s:%d I2C read failed",__func__,__LINE__);
		sprintf(buf, "i2c_read failed");
		goto exit;
	}

	for (i = 0 ; i < asic_i2c_read_bytes ; i++) {
		CAM_DBG(CAM_SENSOR, "[%d] = 0x%2.2X", i, block_data[i]);
		if(i < 341) // 1024 Bytes
		sprintf(buf + (i * 3), "%2.2X ", block_data[i]);
	}
    _i2cStats.lccReadShow++;
exit:
	kfree(block_data);
	return strlen(buf);
}

static ssize_t
asic_i2c_read_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	unsigned int  num_byte=0, subaddr=0;

	if (sscanf(buf, "%u %x" , &num_byte, &subaddr) <= 0) {
		printk(KERN_ERR "Could not tranform the register value");
		return -EINVAL;
	}
	asic_i2c_read_bytes = num_byte;
	asic_reg_addr = subaddr;
	CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s, num_of_bytes = %d, reg_addr = 0x%x", __func__, asic_i2c_read_bytes, asic_reg_addr);

    _i2cStats.lccReadStore++;
	return n;
}

static ssize_t
asic_i2c_write_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	uint8_t *data;
	uint8_t tmp[5] = {0x00};
	uint32_t block_addr;
	uint32_t number_of_data = 0;
	int i = 0, first_pos = 0, cmd_shift = 0, rc = 0;
	char input[6];
	char output[1024];

	struct i2c_client *m_client = to_i2c_client(dev);
	struct camera_io_master *sensor_i2c_client = container_of(&m_client, struct camera_io_master, client);

	/* parsing number of data */
	for(i=0 ; i<n ; i++) {
		if(buf[i] == ' ') {
			first_pos = i;
			break;
		};
	};

	for(i=0 ; i<first_pos ; i++) {
		tmp[i] = buf[i];
		number_of_data = (uint32_t) simple_strtol(tmp, NULL, 10);
	}

	cmd_shift = first_pos + 1;

	memset(tmp, 0, 5);
	/* parsing bytes of address */
	sprintf(tmp, "%c%c%c%c", buf[cmd_shift+2], buf[cmd_shift+3], buf[cmd_shift+4], buf[cmd_shift+5]);
	block_addr = (uint32_t) simple_strtol(tmp, NULL, 16);

	CAM_DBG(CAM_SENSOR, "number of data = %d", number_of_data);
	CAM_DBG(CAM_SENSOR, "block_addr = 0x%4.4X", block_addr);

	if (number_of_data < 1) {
		CAM_ERR(CAM_SENSOR, "Fail! need more than 1 bytes input");
		return -ENOEXEC;
	}

	data = kzalloc(number_of_data, GFP_KERNEL);
	if (!data) {
		CAM_ERR(CAM_SENSOR, "No memory");
		return -ENOMEM;
	}

	/* parsing register value */
	memset(tmp, 0, 5);
	for (i = 0 ; i < number_of_data ; i++) {
		if (7+(2+i*5) > n) {
			CAM_ERR(CAM_SENSOR, "Data format error!!!");
			goto exit;
		}
		sprintf(tmp, "%c%c", buf[cmd_shift+7+(2+i*5)], buf[cmd_shift+7+(3+i*5)]);
		data[i] = (uint8_t) simple_strtol(tmp, NULL, 16);
	}

	memset(input, 0, 5);
	memset(output, 0, 1024);
	for (i = 0 ; i < number_of_data ; i++) {
		CAM_DBG(CAM_SENSOR, "[%d] = %02X, ", i, data[i]);
		snprintf(input, 6, "0x%02X  ", data[i]);
		strcat(output, input);
	}
	CAM_DBG(CAM_SENSOR, "write_data : %s", output);

	rc = cam_qup_i2c_write_seq_light(sensor_i2c_client, block_addr, CAMERA_SENSOR_I2C_TYPE_WORD, data, number_of_data);

	if (rc < 0){
		CAM_ERR(CAM_SENSOR, "I2C block write failed");
		goto exit;
	}
    _i2cStats.lccWriteStore++;
exit:
	kfree(data);
	return n;
}

static ssize_t
asic_i2c_write_binary( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
    int rc = 0;
	struct i2c_client *m_client = to_i2c_client(dev);
	struct camera_io_master *sensor_i2c_client = container_of(&m_client, struct camera_io_master, client);

    light_i2c_lcc_t* bufPtr = (light_i2c_lcc_t*)buf;

    if ( buf == NULL )
    {
		CAM_ERR(CAM_SENSOR, "I2C Buffer pointer NULL");
		return -EINVAL;
    }

	CAM_DBG(CAM_SENSOR, "number of data = %d", bufPtr->msgLength);
	CAM_DBG(CAM_SENSOR, "block_addr = 0x%4.4X", bufPtr->address);

	if (bufPtr->msgLength < 1) {
		CAM_ERR(CAM_SENSOR, "Fail! need more than 1 byte input");
		return -ENOEXEC;
	}

    /*
     * Message length when sent down to the kernel includes the
     * I2C address, remove that here because the QUP driver sends
     * it as part of the Address sequence and not part of the
     * data transmit sequence.
     */
	rc = cam_qup_i2c_write_seq_light(sensor_i2c_client, bufPtr->address, CAMERA_SENSOR_I2C_TYPE_WORD, &(bufPtr->data[0]), bufPtr->msgLength - sizeof(uint16_t));

	if (rc < 0){
		CAM_ERR(CAM_SENSOR, "I2C block write failed");
		goto exit;
	}
    _i2cStats.lccWriteBinary++;
exit:
	return n;
}

#define DEFAULT_POWER_UP_SIZE 7
static struct cam_sensor_power_setting default_power_up[DEFAULT_POWER_UP_SIZE] = {
	{.seq_type = SENSOR_CUSTOM_GPIO2,	.config_val = 0,	.delay = 1,},
	{.seq_type = SENSOR_CUSTOM_GPIO1,	.config_val = 0,	.delay = 0,},
	{.seq_type = SENSOR_STANDBY,		.config_val = 0,	.delay = 1,},
	{.seq_type = SENSOR_RESET,		.config_val = 0,	.delay = 11},
	{.seq_type = SENSOR_CUSTOM_GPIO1,	.config_val = 1,	.delay = 1,},
	{.seq_type = SENSOR_STANDBY,		.config_val = 1,	.delay = 1,},
	{.seq_type = SENSOR_RESET,		.config_val = 1,	.delay = 1,},
};
#define DEFAULT_POWER_DOWN_SIZE 1
static struct cam_sensor_power_setting default_power_down[DEFAULT_POWER_DOWN_SIZE] = {
	{.seq_type = SENSOR_CUSTOM_GPIO1,	.config_val = 0,	.delay = 0,},
};
static ssize_t
asic_power_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t n)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t*)i2c_get_clientdata(client);
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;
	struct cam_sensor_power_setting *power_setting = NULL;
	int power_setting_size = 0;
	int status = 0, type = 0, value = 0, delay = 0, i = 0;

	if (!s_ctrl) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] No s_ctrl data", __func__);
		return -ENODEV;
	}

	if (!s_ctrl->sensordata) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] No s_ctrl sensordata", __func__);
		return -ENODEV;
	}

	gpio_num_info = s_ctrl->sensordata->power_info.gpio_num_info;
	if (!gpio_num_info) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] No gpio_num_info data", __func__);
		return -ENODATA;
	}

	if (sscanf(buf, "%d" , &status) <= 0) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] get value fail", __func__);
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
#if defined(CONFIG_SERIAL_MSM_GENI)
	if (status) //before power-on to enable uart clock
		asic_uart_clk_enable(s_ctrl->soc_info.asic_uart_line, 1);
#endif

	if (status) {
		power_setting = default_power_up;
		power_setting_size = DEFAULT_POWER_UP_SIZE;
	} else {
		power_setting = default_power_down;
		power_setting_size = DEFAULT_POWER_DOWN_SIZE;
	}

	for (i = 0; i < power_setting_size; i++) {
		type = (power_setting + i)->seq_type;
		value = !!((power_setting + i)->config_val);
		delay = (power_setting + i)->delay;

		if (gpio_num_info->valid[type]) {
			gpio_direction_output(gpio_num_info->gpio_num[type], value);
			CAM_DBG(CAM_SENSOR, "[%s] type: %d, gpio(%d): %s, delay: %dms", __func__, type,
				gpio_num_info->gpio_num[type], (value?"high":"low"), delay);
			if (delay)
				msleep(delay);
		}
	}

#if defined(CONFIG_SERIAL_MSM_GENI)
	if (!status) //after power-down to disable uart clock
		asic_uart_clk_enable(s_ctrl->soc_info.asic_uart_line, 0);
#endif

    _i2cStats.writeAsicPower++;
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return n;
}

static ssize_t asic_power_show(
    struct device *dev,
	struct device_attribute *attr,
    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t*)i2c_get_clientdata(client);
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;
	struct cam_sensor_power_setting *power_setting = default_power_down;
	int type = 0, value = 0;

	if (!s_ctrl) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] No s_ctrl data", __func__);
		return -ENODEV;
	}

	if (!s_ctrl->sensordata) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] No s_ctrl sensordata", __func__);
		return -ENODEV;
	}

	gpio_num_info = s_ctrl->sensordata->power_info.gpio_num_info;
	if (!gpio_num_info) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "[%s] No gpio_num_info data", __func__);
		return -ENODATA;
	}

	type = power_setting->seq_type; //use array[0] item as power-status
	if (gpio_num_info->valid[type]) {
		value = gpio_get_value(gpio_num_info->gpio_num[type]);
		CAM_DBG(CAM_SENSOR, "[%s] type: %d, gpio(%d): %s", __func__, type,
			gpio_num_info->gpio_num[type], (value?"high":"low"));
	}

    _i2cStats.readAsicPower++;
	return sprintf(buf, "%d\n", !!value);
}

/*
 * Pop 1 i2c message from the circular buffer and return to the
 * client.  Results are binary so using "cat" from shell is not
 * a good way to look at these results.
 */
static ssize_t asic_i2c_results(
    struct device *dev,
	struct device_attribute *attr,
    char *buf)
{
    int head;
    int tail;
    ssize_t length = 0;
    /*
     * Read the depth of the FIFO and pop one off if there
     * is at least 1 present.  This can be optimized later
     * to read off multiple if needed.
     */
    head = _lightI2cQueue.head;
    tail = _lightI2cQueue.tail;
    if ( CIRC_CNT(head, tail, LIGHT_MAX_I2C_QUEUE_SIZE) > 0 )
    {
        _i2cStats.messagesSent++;
        _i2cStats.sysfsReads++;
        _i2cStats.bytesSent += _lightI2cQueue.i2cResponses[tail].size;
        length = _lightI2cQueue.i2cResponses[tail].size;
        _lightI2cQueue.i2cResponses[tail].size = 0;
        memcpy(buf, &(_lightI2cQueue.i2cResponses[tail].buffer[0]), length);
        _lightI2cQueue.tail = ((tail + 1) & (LIGHT_MAX_I2C_QUEUE_SIZE - 1));
    }

	return length;
}

/*
 * Print out the stats structure being kept which is tracking i2c responses from
 * the ASIC
 */
static ssize_t asic_i2c_stats(
    struct device *dev,
	struct device_attribute *attr,
    char *buf)
{
    sprintf(buf,
            "  I2C FIFO Size:%d\n  I2C FIFO Depth:%d\n  Head Index:%d\n  Tail Index:%d\n  Userspace Messages Sent:%d\n  Userspace Bytes Sent:%d\n  I2C Messages Received:%d\n  I2C Messages Dropped:%d\n  I2C Bytes Received:%d\n  I2C Bytes Dropped:%d\n  Sysfs Reads:%d\n  LCC Read Show:%d\n  LCC Read Store:%d\n  LCC Write Store:%d\n  LCC Write Binary:%d\n  Write ASIC Power:%d\n  Read ASIC Power:%d\n  I2C Response Called:%d\n  ASIC Interrupt Called:%d\n",
            CIRC_SPACE(_lightI2cQueue.head, _lightI2cQueue.tail, LIGHT_MAX_I2C_QUEUE_SIZE),
            CIRC_CNT(_lightI2cQueue.head, _lightI2cQueue.tail, LIGHT_MAX_I2C_QUEUE_SIZE),
            _lightI2cQueue.head,
            _lightI2cQueue.tail,
            _i2cStats.messagesSent,
            _i2cStats.bytesSent,
            _i2cStats.messagesReceived,
            _i2cStats.messagesDropped,
            _i2cStats.bytesReceived,
            _i2cStats.bytesDropped,
            _i2cStats.sysfsReads,
            _i2cStats.lccReadShow,
            _i2cStats.lccReadStore,
            _i2cStats.lccWriteStore,
            _i2cStats.lccWriteBinary,
            _i2cStats.writeAsicPower,
            _i2cStats.readAsicPower,
            _i2cStats.i2cResponseCalled,
            _i2cStats.asicInterruptCalled);


	return strlen(buf);
}

static DEVICE_ATTR(i2c_br, 0644, asic_i2c_read_show, asic_i2c_read_store);
static DEVICE_ATTR(i2c_w, 0644, NULL, asic_i2c_write_store);
static DEVICE_ATTR(i2c_wb, 0644, NULL, asic_i2c_write_binary);
static DEVICE_ATTR(i2c_results, 0644, asic_i2c_results, NULL);
static DEVICE_ATTR(i2c_stats, 0644, asic_i2c_stats, NULL);
static DEVICE_ATTR_RW(asic_power);

static struct attribute *asic_i2c_attributes[] = {
		&dev_attr_i2c_br.attr,
		&dev_attr_i2c_w.attr,
		&dev_attr_i2c_wb.attr,
		&dev_attr_i2c_results.attr,
		&dev_attr_i2c_stats.attr,
		&dev_attr_asic_power.attr,
		NULL
};

static const struct attribute_group asic_i2c_attr_group = {
		.attrs = asic_i2c_attributes,
};

#ifdef FTM
static ssize_t cam_ping_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	CAM_ERR_RATE_LIMIT(CAM_SENSOR, "show cam_ping=%d", g_camera_ping);
	return sprintf(buf, "%d", g_camera_ping);
}
static DEVICE_ATTR(cam_ping, 0444, cam_ping_show, NULL);

static struct attribute *cam_ping_attributes[] = {
	&dev_attr_cam_ping.attr,
	NULL
};
static const struct attribute_group cam_ping_attr_group = {
	.attrs = cam_ping_attributes,
};
#endif

static void i2c_queue_init(void)
{
    int i;
    static int32_t inited = 0;

    if ( inited == 0 )
    {
        inited = 1;
        _lightI2cQueue.head = 0;
        _lightI2cQueue.tail = 0;
    
        for ( i = 0 ; i < LIGHT_MAX_I2C_QUEUE_SIZE; i++ )
        {
            _lightI2cQueue.i2cResponses[i].size = 0;
        }
        CAM_DBG(CAM_SENSOR, "Initialized i2c response queue, size:%d", LIGHT_MAX_I2C_QUEUE_SIZE);
        memset(&_i2cStats, 0, sizeof(_i2cStats));
    }
}

static void i2c_response(char* msg_buffer, uint16_t msg_length)
{
    int head;
    int tail;
    /*
     * We attempt to queue up the response in the next read
     * index unless we are full.  If we are full we drop
     * the message and bump some stats which can be dumped
     * via debugfs
     */
    head = _lightI2cQueue.head;
    tail = _lightI2cQueue.tail;
    if ( CIRC_SPACE(head, tail, LIGHT_MAX_I2C_QUEUE_SIZE) > 0 )
    {
        /*
         * Create the protocol message which can be read out of
         * the sysfs.
         */
        light_i2c_lcc_response_t* lightMsgPtr;
        lightMsgPtr = (light_i2c_lcc_response_t*)&(_lightI2cQueue.i2cResponses[head].buffer[0]);
        lightMsgPtr->msgLength = msg_length;
        memcpy(lightMsgPtr->data, msg_buffer, msg_length);
        _i2cStats.messagesReceived++;
        _i2cStats.bytesReceived += msg_length;
        _lightI2cQueue.i2cResponses[head].size = msg_length + sizeof(light_i2c_lcc_response_t);
        /*
         * Increment to the next element in the circular buffer.
         */
        _lightI2cQueue.head =  ((head + 1) & (LIGHT_MAX_I2C_QUEUE_SIZE - 1));
    }
    else
    {
        _i2cStats.messagesDropped++;
        _i2cStats.bytesDropped += msg_length;
    }
    _i2cStats.i2cResponseCalled++;
}

enum asic_index {
	ASIC1 = 0,
	NUM_OF_ASIC,	/* must be the last  */
};

struct asic_irq_info {
	const char *label;
	irq_handler_t thread_fn;
};

static irqreturn_t asic_main_interrupt(int irq, void *dev);

static struct asic_irq_info asic_irq_infos[NUM_OF_ASIC] = {
	[ASIC1] = {"asic-irq1", asic_main_interrupt},
};

static struct cam_sensor_ctrl_t *main_asic_sctrl = NULL;

static irqreturn_t asic_main_interrupt(int irq, void *dev)
{
	int rc = 0;
	uint16_t transaction_id = 0;
	uint16_t command_status = 0;
	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)dev;
	bool done = false;

	if (!s_ctrl)
		return IRQ_HANDLED;

    _i2cStats.asicInterruptCalled++;

	while (!done) {
		/* LCC Response message buffer */
		uint16_t msg_length;
		/* Read header plus 4 bytes of response */
		lcc_resp_msg_t lcc_resp;
		light_msg_header_t* msg_header;

		light_i2c_read_request_t read_request;
		read_request.req = START_READ_REQUEST;

		rc = cam_qup_i2c_read_seq(s_ctrl->io_master_info.client, read_request.req,
			(uint8_t *)&lcc_resp, CAMERA_SENSOR_I2C_TYPE_WORD, sizeof(lcc_resp_msg_t));
		if (rc < 0) {
			CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s:%d I2C read failed", __func__, __LINE__);
			done = true;
		} else {
			msg_header = &lcc_resp.msg_header;
			msg_length = msg_header->length;

			if (msg_header->msg_type >= NUM_LIGHT_MSG_TYPES) {
				CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s: Invalid message. Message Type: 0x%x, length = 0x%d",
					__func__, msg_header->msg_type, msg_length);
				done = true;
			} else
				done = (msg_header->msg_type == LCC_CMD_DUMMY_MSG);
		}

		if (!done) {
			const size_t DEFAULT_RESP_LENGTH = sizeof(lcc_cmd_resp_t);

			if (msg_length < DEFAULT_RESP_LENGTH) {
				/* Flag an error if the response was not a dummy response either */
				CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s: Invalid message length. Message Type: 0x%x, length = 0x%d",
					__func__, msg_header->msg_type, msg_length);
				done = true;
			} else {
				uint16_t full_msg_length = msg_length + sizeof(light_msg_header_t);
				char* msg_buffer;

				if ((msg_header->msg_type == LCC_CMD_RESP_MSG) ||
					(msg_header->msg_type == TRANSFER_DESCRIPTOR_MSG)) {
					/* FIXME Swap to undo msm_camera_qup_i2c_write TID byte order */
					transaction_id = (((lcc_resp.slave_resp.tid & 0xFF) << 8) | (lcc_resp.slave_resp.tid >> 8));
					command_status = lcc_resp.slave_resp.status;
					lcc_resp.slave_resp.tid = transaction_id;
				}

                msg_buffer = kzalloc(full_msg_length, GFP_KERNEL);
                if (!msg_buffer) {
                    CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s: msg_buffer failed\n", __func__);
                    done = true;
                    break;
                }
				memcpy(msg_buffer, &lcc_resp, sizeof(lcc_resp));
				if (msg_length > DEFAULT_RESP_LENGTH) {
					uint16_t write_offset = sizeof(lcc_resp_msg_t);
					read_request.req = CONTINUE_READ_REQUEST;

					rc = cam_qup_i2c_read_seq(s_ctrl->io_master_info.client, read_request.req,
						&msg_buffer[write_offset], CAMERA_SENSOR_I2C_TYPE_WORD,
						msg_length-DEFAULT_RESP_LENGTH);
					if (rc < 0) {
						CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s:%d I2C read failed",__func__,__LINE__);
						done = true;
                        kfree(msg_buffer);
						break;
					}
				}

				if (s_ctrl->soc_info.asic_sync_obj) {
					cam_sync_signal_with_info(s_ctrl->soc_info.asic_sync_obj,
											  CAM_SYNC_STATE_SIGNALED_SUCCESS,
											  msg_buffer,
											  full_msg_length);
				}
				i2c_response(msg_buffer, full_msg_length);
                kfree(msg_buffer);
			}
		}
	}

	return IRQ_HANDLED;
}

static int init_asic_irq(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct device_node *of_node = s_ctrl->of_node;
	uint16_t gpio_array_size = 0;
	uint16_t *gpio_array = NULL;
	uint8_t i = 0;
	int rc = -1;

	gpio_array_size = of_gpio_named_count(of_node, "asic-irq");
	CAM_ERR_RATE_LIMIT(CAM_SENSOR, " gpio_array_size = %d",gpio_array_size);

	if (gpio_array_size > NUM_OF_ASIC || gpio_array_size < 1)
		return -EINVAL;

	gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
	if (!gpio_array) {
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "%s alloc memoery failed",__func__);
		goto FREE_GPIO_CONF;
	}

	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_named_gpio_flags(of_node, "asic-irq", i, NULL);
		CAM_ERR_RATE_LIMIT(CAM_SENSOR, "gpio_array[%d] = %d", i, gpio_array[i]);

		if (gpio_is_valid(gpio_array[i])) {
			CAM_ERR_RATE_LIMIT(CAM_SENSOR, "gpio_irq  is_valid ");
			rc = gpio_request(gpio_array[i], asic_irq_infos[i].label);
			if (rc) {
				CAM_ERR_RATE_LIMIT(CAM_SENSOR, "unable to request gpio [%d]",gpio_array[i]);
				continue;
			}
			rc = gpio_direction_input(gpio_array[i]);
			if (rc) {
				CAM_ERR_RATE_LIMIT(CAM_SENSOR, "unable to  set direction for gpio [%d]",gpio_array[i]);
				gpio_free(gpio_array[i]);
				continue ;
			}

			rc = request_threaded_irq(gpio_to_irq(gpio_array[i]), NULL, asic_irq_infos[i].thread_fn,
					IRQF_TRIGGER_RISING| IRQF_ONESHOT,
					asic_irq_infos[i].label, (void *)s_ctrl);
			if (rc < 0) {
				CAM_ERR_RATE_LIMIT(CAM_SENSOR, "failed to register interrupt");
				gpio_free(gpio_array[i]);
			}
		} else {
			CAM_ERR_RATE_LIMIT(CAM_SENSOR, "asic%d_irq is invalid", i);
		}
	}
FREE_GPIO_CONF:
	kfree(gpio_array);
	return rc;
}

static int32_t cam_sensor_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc = 0;
	int i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_hw_soc_info   *soc_info = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_SENSOR,
			"%s :i2c_check_functionality failed", client->name);
		return -EFAULT;
	}

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl)
		return -ENOMEM;

	i2c_set_clientdata(client, s_ctrl);

	s_ctrl->io_master_info.client = client;
	soc_info = &s_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;

	/* Initialize sensor device type */
	s_ctrl->of_node = client->dev.of_node;
	s_ctrl->io_master_info.master_type = I2C_MASTER;
	s_ctrl->is_probe_succeed = 0;

	rc = cam_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	rc = cam_sensor_init_subdev_params(s_ctrl);
	if (rc)
		goto free_s_ctrl;

	if(soc_info->asic_supported) {
		rc = sysfs_create_group(&client->dev.kobj, &asic_i2c_attr_group);
		asic_sysfs_kobj = kobject_create_and_add("asic", NULL);
		if (asic_sysfs_kobj)
			rc = sysfs_create_link(asic_sysfs_kobj, &client->dev.kobj, "link");
		else
			CAM_ERR_RATE_LIMIT(CAM_SENSOR, "Create asic kobj fail");
	}

	s_ctrl->i2c_data.per_frame =
		(struct i2c_settings_array *)
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (s_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(s_ctrl->i2c_data.per_frame[i].list_head));

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.ops.get_dev_info = cam_sensor_publish_dev_info;
	s_ctrl->bridge_intf.ops.link_setup = cam_sensor_establish_link;
	s_ctrl->bridge_intf.ops.apply_req = cam_sensor_apply_request;
	s_ctrl->bridge_intf.ops.flush_req = cam_sensor_flush_request;

	s_ctrl->sensordata->power_info.dev = soc_info->dev;
	v4l2_set_subdevdata(&(s_ctrl->v4l2_dev_str.sd), s_ctrl);

	if(soc_info->asic_supported) {
		i2c_queue_init();
		if (!main_asic_sctrl)
			main_asic_sctrl = s_ctrl;

		init_asic_irq(s_ctrl);
	}

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
free_s_ctrl:
	kfree(s_ctrl);
	return rc;
}

static int cam_sensor_platform_remove(struct platform_device *pdev)
{
	int                        i;
	struct cam_sensor_ctrl_t  *s_ctrl;
	struct cam_hw_soc_info    *soc_info;

	s_ctrl = platform_get_drvdata(pdev);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "sensor device is NULL");
		return 0;
	}

	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	kfree(s_ctrl->i2c_data.per_frame);
	devm_kfree(&pdev->dev, s_ctrl);

	return 0;
}

static int cam_sensor_driver_i2c_remove(struct i2c_client *client)
{
	int                        i;
	struct cam_sensor_ctrl_t  *s_ctrl = i2c_get_clientdata(client);
	struct cam_hw_soc_info    *soc_info;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "sensor device is NULL");
		return 0;
	}

	if(s_ctrl->soc_info.asic_supported) {
		if (main_asic_sctrl)
			main_asic_sctrl = NULL;
	}

	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	kfree(s_ctrl->i2c_data.per_frame);
	kfree(s_ctrl);

	return 0;
}

static const struct of_device_id cam_sensor_driver_dt_match[] = {
	{.compatible = "qcom,cam-sensor"},
	{}
};

static int32_t cam_sensor_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc = 0, i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_hw_soc_info *soc_info = NULL;

	/* Create sensor control structure */
	s_ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_ctrl_t), GFP_KERNEL);
	if (!s_ctrl)
		return -ENOMEM;

	soc_info = &s_ctrl->soc_info;
	soc_info->pdev = pdev;
	soc_info->dev = &pdev->dev;
	soc_info->dev_name = pdev->name;

	/* Initialize sensor device type */
	s_ctrl->of_node = pdev->dev.of_node;
	s_ctrl->is_probe_succeed = 0;

	/*fill in platform device*/
	s_ctrl->pdev = pdev;

	s_ctrl->io_master_info.master_type = CCI_MASTER;

	rc = cam_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed: cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	/* Fill platform device id*/
	pdev->id = soc_info->index;

#ifdef FTM
	if(sensor_id_sysfs == 0) {
		rc = sysfs_create_group(&pdev->dev.parent->kobj, &cam_ping_attr_group);
		sensor_id_sysfs = 1;
	}
#endif
	rc = cam_sensor_init_subdev_params(s_ctrl);
	if (rc)
		goto free_s_ctrl;

	s_ctrl->i2c_data.per_frame =
		(struct i2c_settings_array *)
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (s_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(s_ctrl->i2c_data.per_frame[i].list_head));

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.ops.get_dev_info = cam_sensor_publish_dev_info;
	s_ctrl->bridge_intf.ops.link_setup = cam_sensor_establish_link;
	s_ctrl->bridge_intf.ops.apply_req = cam_sensor_apply_request;
	s_ctrl->bridge_intf.ops.flush_req = cam_sensor_flush_request;

	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	platform_set_drvdata(pdev, s_ctrl);
	v4l2_set_subdevdata(&(s_ctrl->v4l2_dev_str.sd), s_ctrl);

	s_ctrl->sensor_state = CAM_SENSOR_INIT;

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
free_s_ctrl:
	devm_kfree(&pdev->dev, s_ctrl);
	return rc;
}

MODULE_DEVICE_TABLE(of, cam_sensor_driver_dt_match);

static struct platform_driver cam_sensor_platform_driver = {
	.probe = cam_sensor_driver_platform_probe,
	.driver = {
		.name = "qcom,camera",
		.owner = THIS_MODULE,
		.of_match_table = cam_sensor_driver_dt_match,
		.suppress_bind_attrs = true,
	},
	.remove = cam_sensor_platform_remove,
};

static const struct i2c_device_id i2c_id[] = {
	{SENSOR_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_sensor_driver_i2c = {
	.id_table = i2c_id,
	.probe = cam_sensor_driver_i2c_probe,
	.remove = cam_sensor_driver_i2c_remove,
	.driver = {
		.name = SENSOR_DRIVER_I2C,
	},
};

static int __init cam_sensor_driver_init(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&cam_sensor_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "platform_driver_register Failed: rc = %d",
			rc);
		return rc;
	}

	rc = i2c_add_driver(&cam_sensor_driver_i2c);
	if (rc)
		CAM_ERR(CAM_SENSOR, "i2c_add_driver failed rc = %d", rc);

	return rc;
}

static void __exit cam_sensor_driver_exit(void)
{
	platform_driver_unregister(&cam_sensor_platform_driver);
	i2c_del_driver(&cam_sensor_driver_i2c);
}

module_init(cam_sensor_driver_init);
module_exit(cam_sensor_driver_exit);
MODULE_DESCRIPTION("cam_sensor_driver");
MODULE_LICENSE("GPL v2");
