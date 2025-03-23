/*
 * dw7912.c  --  Vibrator driver for dw7912
 *
 * Copyright (C) 2017 Dongwoon Anatech Co. Ltd. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <dw7912.h>

// inset code <DWA 18.01.29>
#include <linux/ktime.h>



struct dw7912_priv {
	int enable;
	u8 wave_buf[300];
	struct i2c_client *dwclient;
};

static struct dw7912_priv *dw7912;
static struct hrtimer hr_timer;
static struct work_struct vib_work;

void dw_i2c_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value);
int real_time_playback(struct dw7912_priv *p, u8 *data, u32 size, int loopcount);
void boost_mode_set(struct dw7912_priv *p, int delay, int mode);
void play_mode_set(struct dw7912_priv *p, int mode);
void play_on(struct dw7912_priv *p, int mode);
int init_memory_set(struct dw7912_priv *p, int mode);
int memory_mode_play_set(struct dw7912_priv *p, int mode, u32 play_time);
int rtp_mode_play_set(struct dw7912_priv *p, int mode);
int memory_wave_write(struct dw7912_priv *p, u8 *head_data, u8 *data, u32 size);
int hrtimer_set_time(unsigned long in_ms);
enum hrtimer_restart play_hrtimer_callback(struct hrtimer *timer);
static void stop_vib_func(struct work_struct *work);


#if 1
	#define gprintk(fmt, x... ) printk( "[VIB] %s: " fmt, __FUNCTION__ , ## x)
#else
	#define gprintk(x...) do { } while (0)
#endif


#define PLAY_ON		1
#define PLAY_OFF	0

#define MEM_MODE	1
#define RTP_MODE	0

#define BST_DEFAULT	0
#define BST_EN 		1
#define BST_BYPASS 	2
#define BST_LIMP 	4
#define BST_ADAPT 	8

#define BBOX_HAPTIC_PROBE_FAIL do {printk("BBox::UEC;19::0\n");} while (0);
#define BBOX_HAPTIC_SET_FAIL do {printk("BBox::UEC;19::2\n");} while (0);
#define BBOX_HAPTIC_WRITE_REGISTER_FAIL do {printk("BBox::UEC;19::7\n");} while (0);
#define BBOX_HAPTIC_ENABLE_FAIL do {printk("BBox::UEC;19::3\n");} while (0);

void dw_i2c_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
        int rc;

        rc = i2c_smbus_write_byte_data(client, command, value);
        if (rc < 0) {
                BBOX_HAPTIC_WRITE_REGISTER_FAIL
	}
}

void boost_mode_set(struct dw7912_priv *p, int delay, int mode)
{
	struct i2c_client *i2c_fnc = p->dwclient;

	u8 value = (delay << 4) | mode;
	dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, value);
}


void play_mode_set(struct dw7912_priv *p, int mode)
{
	struct i2c_client *i2c_fnc = p->dwclient;

	dw_i2c_smbus_write_byte_data(i2c_fnc, 0x03, mode);
}


void play_on(struct dw7912_priv *p, int mode)
{
	struct i2c_client *i2c_fnc = p->dwclient;

	//gprintk("play_on = %d", mode);
	dw_i2c_smbus_write_byte_data(i2c_fnc, 0x09, mode);
}

int init_memory_set(struct dw7912_priv *p, int mode)
{
	//struct dw7912_priv *pDW = dw7912;

	switch(mode) {
		case 0:
		// mem_wave1 play time is 5.875ms
		memory_wave_write(p, (u8*)mem_header1, (u8*)mem_wave1, 282);	// long vibration
		//memory_wave_write(p, (u8*)mem_header4, (u8*)mem_wave4, sizeof(mem_wave4));	// sizeof... problem we need the test
		break;

	case 1:
		break;
	}

	//gprintk("init memory done\n");
	return 0;
}


int rtp_mode_play_set(struct dw7912_priv *p, int mode)
{
	struct i2c_client *i2c_fnc = p->dwclient;

	play_mode_set(p, RTP_MODE);

	switch(mode) {
		case 0:
			//Short click
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0xC8); //8V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 1:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x32); //2V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 2:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x26); //1.52V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 3:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x19); //1V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 4:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x19); //0.52V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 5:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x96); //6V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 6:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x7D); //5V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 7:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x64); //4V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		case 8:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x4B); //3V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x06, 0x64); //set vd-calmp wait time 996us
			break;
		default:
			BBOX_HAPTIC_SET_FAIL
			gprintk("undefined play mode\n");
			break;
	}

	//gprintk("rtp reg write done\n");
	return 0;
}


unsigned long play_time;

enum hrtimer_restart play_hrtimer_callback(struct hrtimer *timer)
{
	ktime_t currtime, interval;

	currtime = ktime_get();
	interval = ktime_set(0, play_time);
	hrtimer_forward(timer, currtime, interval);
	schedule_work(&vib_work);

	return HRTIMER_NORESTART;
}

int hrtimer_set_time(unsigned long in_ms)
{
	ktime_t ktime;
	play_time = in_ms * 1000000;
	ktime = ktime_set(0, play_time);
	hrtimer_start(&hr_timer, ktime, HRTIMER_MODE_REL);

	return 0;
}

static void stop_vib_func(struct work_struct *work)
{
	struct dw7912_priv *pDW = dw7912;
	play_on(pDW, PLAY_OFF);
}


int memory_mode_play_set(struct dw7912_priv *p, int mode, u32 play_time)
{
	struct i2c_client *i2c_fnc = p->dwclient;
	struct dw7912_priv *pDW = dw7912;

	play_mode_set(p, MEM_MODE);

	switch(mode) {
		case 0:
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x08, 0x58); //3.52V
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x23, 0x06); //boost offset
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x0B, 0x00); //mem gain

			// this register set time is 9.8sec
			// memory call address
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x0C, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x0D, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x0E, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x0F, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x10, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x11, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x12, 0x01);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x13, 0x01);

			// memory call loop
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x14, 0xEE);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x15, 0xEE);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x16, 0xEE);
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x17, 0xEE);

			// total memory loop call
			dw_i2c_smbus_write_byte_data(i2c_fnc, 0x18, 0x0E);

			play_on(pDW, PLAY_ON);
			hrtimer_set_time((unsigned long)play_time);
			break;
		case 1:

			break;
		case 2:

			break;
		default:
			BBOX_HAPTIC_SET_FAIL
			gprintk("undefined play mode\n");
			break;
	}

	//gprintk("memory data write done\n");
	return 0;
}



int memory_wave_write(struct dw7912_priv *p, u8 *head_data, u8 *data, u32 size)
{
	int rc;
	int k, j, loop, tail, start_addr, st_addr, trans_size;
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc= p->dwclient;

	play_mode_set(p, MEM_MODE);

	xfer[0].addr  = 0x59;
	xfer[0].len   = 8;
	xfer[0].flags = 0;
	xfer[0].buf   = head_data;
	rc = i2c_transfer(i2c_fnc->adapter, xfer, 1);

	st_addr = ((int)head_data[3]<<8) | head_data[4];

	trans_size = 250;

	if(size >= trans_size) {
		loop = size / trans_size;
		tail = size % trans_size;
	}
	else {
		trans_size = size;
		loop = size / trans_size;
		tail = size % trans_size;
	}


	#if 0
	loop = sizeof(data) / trans_size;
	tail = sizeof(data) % trans_size;
	#endif

	loop = size / trans_size;
	tail = size % trans_size;

	for(k=0; k<loop; k++) {
		start_addr = st_addr + k * trans_size;
		p->wave_buf[0] = 0x1b;
		p->wave_buf[1] = start_addr >> 8;
		p->wave_buf[2] = start_addr;

		for(j=0; j<trans_size; j++) {
			p->wave_buf[j + 3] = data[j + k * trans_size];
		}

		xfer[0].addr  = 0x59;
		xfer[0].len   = trans_size + 3;
		xfer[0].flags = 0;
		xfer[0].buf   = p->wave_buf;
		rc = i2c_transfer(i2c_fnc->adapter, xfer, 1);
	}

	start_addr = st_addr + loop * trans_size;
	p->wave_buf[0] = 0x1b;
	p->wave_buf[1] = start_addr >> 8;
	p->wave_buf[2] = start_addr;

	for(j=0; j<tail; j++) {
		p->wave_buf[j + 3] = data[j + loop * trans_size];
	}

	xfer[0].addr  = 0x59;
	xfer[0].len   = tail + 3;
	xfer[0].flags = 0;
	xfer[0].buf   = p->wave_buf;
	rc = i2c_transfer(i2c_fnc->adapter, xfer, 1);
	//gprintk("memory data write done\n");
	return 0;
}



int real_time_playback(struct dw7912_priv *p, u8 *data, u32 size, int loopcount)
{

	int rc;
	int i, k, j, loop, tail, trans_size;
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc= p->dwclient;

	if(size * loopcount > 2047) {
		gprintk("Wave size exceeded fifo size!!, play stop!\n");
		return -1;
	}

	play_mode_set(p, RTP_MODE);

	trans_size = 250;

	if(size >= trans_size) {
		loop = size / trans_size;
		tail = size % trans_size;
	}
	else {
		trans_size = size;
		loop = size / trans_size;
		tail = size % trans_size;
	}

	for(i=0; i<loopcount; i++) {
		for(k=0; k<loop; k++) {
			p->wave_buf[0] = 0x0A;
			for(j=0; j<trans_size; j++) {
				p->wave_buf[j + 1] = data[j + k * trans_size];
			}

			xfer[0].addr  = 0x59;
			xfer[0].len   = trans_size + 1;
			xfer[0].flags = 0;
			xfer[0].buf   = p->wave_buf;
			rc = i2c_transfer(i2c_fnc->adapter, xfer, 1);
		}

		p->wave_buf[0] = 0x0A;

		for(j=0; j<tail; j++) {
			p->wave_buf[j + 1] = data[j + loop * trans_size];
		}

		xfer[0].addr  = 0x59;
		xfer[0].len   = tail + 1;
		xfer[0].flags = 0;
		xfer[0].buf   = p->wave_buf;
		rc = i2c_transfer(i2c_fnc->adapter, xfer, 1);

	}

	//gprintk("rtp data write done\n");
	return 0;
}


static ssize_t patternVIB_set (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = dw7912;

	gprintk("patternVIB_set vlaue = %s", buf);
	play_on(pDW, PLAY_OFF);

	if(buf[0] == '1') {
		rtp_mode_play_set(pDW, 0);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);	// limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '2') {
		// new function here!!
		rtp_mode_play_set(pDW, 1);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '3') {
		rtp_mode_play_set(pDW, 2);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '4') {
		rtp_mode_play_set(pDW, 3);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '5') {
		rtp_mode_play_set(pDW, 4);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '6') {
		rtp_mode_play_set(pDW, 5);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '7') {
		rtp_mode_play_set(pDW, 6);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '8') {
		rtp_mode_play_set(pDW, 7);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	else if(buf[0] == '9') {
		rtp_mode_play_set(pDW, 8);
		real_time_playback(pDW, (u8*)rtp_short_click1, sizeof(rtp_short_click1), 1);    // limit rtp transter size 2kbyte
		real_time_playback(pDW, (u8*)rtp_short_click2, sizeof(rtp_short_click2), 1);
		play_on(pDW, PLAY_ON);
	}

	return count;
}

static ssize_t enableVIB_set (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw7912_priv *pDW = dw7912;
	int ret = 0, play_time = 0;

	gprintk("enableVIB_set vlaue = %s", buf);
	if (buf[0] == '0') { 				// stop
		play_on(pDW, PLAY_OFF);
	}
	else {
		sscanf(buf, "%d", &play_time);
		if(play_time > 0) {
			play_on(pDW, PLAY_OFF);
			memory_mode_play_set(pDW, 0, play_time);
		}
	}

	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x01);
	gprintk("chip status : %x\n", ret);
	return count;
}


static ssize_t enableVIB_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = dw7912;
	int ret = 0;
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x01);
	gprintk("chip status : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x06);
	gprintk("BST MODE : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x08);
	gprintk("VD_CLAMP : %x\n", ret);
	return snprintf(buf, PAGE_SIZE, "[VIB] status = %x\n", ret);
}


static ssize_t patternVIB_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw7912_priv *pDW = dw7912;
	int ret = 0;
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x01);
	gprintk("chip status : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x06);
	gprintk("BST MODE : %x\n", ret);
	ret = i2c_smbus_read_byte_data(pDW->dwclient, 0x08);
	gprintk("VD_CLAMP : %x\n", ret);
	return snprintf(buf, PAGE_SIZE, "[VIB] status = %x\n", ret);

}


DEVICE_ATTR(enableVIB, (S_IWUSR|S_IRUGO), enableVIB_show, enableVIB_set);
DEVICE_ATTR(patternVIB, (S_IWUSR|S_IRUGO), patternVIB_show, patternVIB_set);


static struct kobject *android_vibrator_kobj;

static int dw7912_vibrator_sysfs_init(void)
{
	int ret = 0;
	android_vibrator_kobj = kobject_create_and_add("android_vibratorDW", NULL);

	if (android_vibrator_kobj == NULL) {
		gprintk("%s:subsystem_register_failed", __func__);
	}

	ret = sysfs_create_file(android_vibrator_kobj, &dev_attr_enableVIB.attr);
	if (ret) {
		gprintk("%s: sysfs_create_file enableVIB failed\n", __func__);
	}
	gprintk("attribute enableVIB file register Done");

	ret = sysfs_create_file(android_vibrator_kobj, &dev_attr_patternVIB.attr);
	if (ret) {
		gprintk("%s: sysfs_create_file patternVIB failed\n", __func__);
	}
	gprintk("attribute patternVIB file register Done");

	return 0;
}


#ifdef CONFIG_OF
static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_priv *p)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;

	if (!np) return -1;

	p->enable = of_get_named_gpio(np, "dw7912,en-gpio", 0);
	if (p->enable < 0) {
		BBOX_HAPTIC_PROBE_FAIL
		printk("Looking up %s property in node %s failed %d\n",
				"dw7912,en-gpio", dev->of_node->full_name,
				p->enable);
		p->enable = -1;
	}

	if( !gpio_is_valid(p->enable) ) {
		BBOX_HAPTIC_PROBE_FAIL
		printk(KERN_ERR "dw7912 enable pin(%u) is invalid\n", p->enable);
	}

	return 0;
}
#else

static int dw7912_i2c_parse_dt(struct i2c_client *i2c, struct dw7912_priv *p)
{
	return NULL;
}
#endif

static int dw7912_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;

	gprintk("dw7912 probe\n");

	dw7912 = kzalloc(sizeof(struct dw7912_priv), GFP_KERNEL);
	if (dw7912 == NULL) {
		BBOX_HAPTIC_PROBE_FAIL
		return -ENOMEM;
	}

	ret = dw7912_i2c_parse_dt(client, dw7912);

	ret = gpio_direction_output(dw7912->enable, 0);
	mdelay(200);
	ret = gpio_direction_output(dw7912->enable, 1);

	if (ret < 0) {
		BBOX_HAPTIC_ENABLE_FAIL
		dev_err(&client->dev, "enable pin level set failed");
		ret = -EIO;
		gpio_free(dw7912->enable);
	}

	i2c_set_clientdata(client, dw7912);
	dw7912->dwclient = client; /* i2c client pointer save */
	dw7912_vibrator_sysfs_init();
	mdelay(100);

	ret = i2c_smbus_read_byte_data(dw7912->dwclient, 0x00);
	gprintk("chip ID : %x\n", ret);

	ret = i2c_smbus_read_byte_data(dw7912->dwclient, 0x01);
	gprintk("chip status : %x\n", ret);

	init_memory_set(dw7912, 0);		// write memory wave data 0

	//init hrtimer and worker
	hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer.function = &play_hrtimer_callback;
	INIT_WORK(&vib_work, stop_vib_func);

	gprintk("dw7912 probe success\n");
	return 0;

}


static int dw7912_i2c_remove(struct i2c_client *client)
{
	struct dw7912_priv *dw7912 = i2c_get_clientdata(client);

	gprintk("1\n");
	gpio_free(dw7912->enable);
	i2c_set_clientdata(client, NULL);
	kfree(dw7912);
	return 0;
}


#ifdef CONFIG_PM
static int dw7912_suspend(struct device *dev)
{
	return 0;
}

static int dw7912_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(dw7912_pm_ops, dw7912_suspend, dw7912_resume);

#define DW7912_VIBRATOR_PM_OPS (&dw7912_pm_ops)
#else
#define DW7912_VIBRATOR_PM_OPS NULL
#endif


#ifdef CONFIG_OF
static struct of_device_id dw7912_i2c_dt_ids[] = {
	{ .compatible = "dwanatech,dw7912"},
	{ }
};
#endif

static const struct i2c_device_id dw7912_i2c_id[] = {
	{"dw7912", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, dw7912_i2c_id);

static struct i2c_driver dw7912_i2c_driver = {
	.probe = dw7912_i2c_probe,
	.remove = dw7912_i2c_remove,
	.id_table = dw7912_i2c_id,
	.driver = {
		.name = "dw7912-codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw7912_i2c_dt_ids),
#endif
#ifdef CONFIG_PM
		.pm	= DW7912_VIBRATOR_PM_OPS,
#endif
		},
};

static int __init dw7912_modinit(void)
{
	int ret;

	gprintk("\n");
	ret = i2c_add_driver(&dw7912_i2c_driver);
	if (ret)
		pr_err("Failed to register dw7912 I2C driver: %d\n", ret);

	return ret;
}

late_initcall(dw7912_modinit);
static void __exit dw7912_exit(void)
{
	int ret;

	i2c_del_driver(&dw7912_i2c_driver);
	ret = hrtimer_cancel(&hr_timer);
	if (ret) printk("The timer was still in use...\n");
	printk("HR Timer module uninstalling\n");
}


module_exit(dw7912_exit);

MODULE_DESCRIPTION("Vibrator DW7912 codec driver");
MODULE_AUTHOR("jks8051@dwanatech.com");
MODULE_LICENSE("GPL");
