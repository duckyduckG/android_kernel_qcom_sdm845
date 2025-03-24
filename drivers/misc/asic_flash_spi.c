/* Copyright (c) 2013-2015, FIH. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/vmalloc.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

#ifdef CONFIG_OF
static struct of_device_id asic_flash_spi_table[] = {
	{ .compatible = "asic,flash-spi",},
	{ },
};
#else
#define asic_flash_spi_table NULL
#endif

#define SPI_MAX_RSIZE 1024
struct asic_flash_pdata {
	struct spi_device *spi;
	int rlen;
	int wlen;
	u8 spi_data[SPI_MAX_RSIZE];
};

int asic_flash_spi_read(struct spi_device *spi, void *buf, size_t len) {
	int ret = 0;
	struct asic_flash_pdata *pdata = spi_get_drvdata(spi);
	ret = spi_read(pdata->spi, buf, len);
	return ret;
}

int asic_flash_spi_write(struct spi_device *spi, const void *buf, size_t len) {
	int ret = 0;
	struct asic_flash_pdata *pdata = spi_get_drvdata(spi);
	ret = spi_write(pdata->spi, buf, len);
	return ret;
}

static ssize_t asic_flash_spi_eeprom_show(struct device *dev,	struct device_attribute *attr, char *buf) {
	int i=0;
	int ret = 0;
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);

	pr_debug("%s\n", __func__);
	for (i=0 ; i<pdata->rlen ; i++) {
		ret += sprintf(buf + ret, "0x%02X, \n", pdata->spi_data[i]);
	}
	return ret;
}

static ssize_t asic_flash_spi_eeprom_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t size) {
	struct file *camdata = NULL;
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);
	mm_segment_t old_fs;
	loff_t pos;
	int i = 0, tmp = 0;
	char xfer = 0;

	pr_debug("%s\n", __func__);
	if (sscanf(buf, "%d", &tmp) <= 0) {
		pr_err("%s: get user-space data failed\n", __func__);
		return size;
	}
	pos = (loff_t)tmp;
	pr_debug("%s pos = %d (%04X)\n", __func__, (int)pos, (int)pos);
	camdata = filp_open("/dev/block/platform/fc594000.ufshc/by-name/camdata", O_CREAT | O_RDWR, 0);
	if(IS_ERR(camdata)) {
		pr_err("file not found!\n");
		goto err;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	for(i=0 ; i<pdata->rlen ; i++) {
		pr_debug("[%d|%04X]: %02X\n", i, (int)pos, pdata->spi_data[i]);
		xfer = pdata->spi_data[i];
		vfs_write(camdata, &xfer, sizeof(char), &pos);
	}
	set_fs(old_fs);
err:
	filp_close(camdata, NULL);
	return size;
}

static ssize_t asic_flash_spi_read_show(struct device *dev, struct device_attribute *attr, char *buf){
	int i=0;
	int ret = 0;
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);

	pr_debug("%s\n", __func__);
	ret += sprintf(buf + ret, "data --->\n");
	for (i=0 ; i<pdata->rlen ; i++) {
		ret += sprintf(buf + ret, "0x%02X,", pdata->spi_data[i]);
		if ((i > 0) && !(i % 16))
			ret += sprintf(buf + ret, "\n");
	}
	ret += sprintf(buf + ret, "\n<--- data\n");
	return ret;
}
static ssize_t asic_flash_spi_read_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	int ret = 0;
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);

	pr_debug("%s\n", __func__);
	if (sscanf(buf, "%d", &pdata->rlen) <= 0) {
		pr_err("%s: get user-space data failed\n", __func__);
		return size;
	}
	if (pdata->rlen <= 0)
		pdata->rlen = 1;
	else if (pdata->rlen > SPI_MAX_RSIZE)
		pdata->rlen = SPI_MAX_RSIZE;
	pr_debug("%s: rlen = %d\n", __func__, pdata->rlen);

	memset(pdata->spi_data, 0, SPI_MAX_RSIZE);
	ret = asic_flash_spi_read(pdata->spi, pdata->spi_data, pdata->rlen);
	if (ret < 0)
		pr_err("%s: spi failed to access register(%d)\n", __func__, ret);

	return size;
}

static ssize_t asic_flash_spi_opread_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	int ret = 0;
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);
	unsigned int opcode = 0;

	pr_debug("%s\n", __func__);
	if (sscanf(buf, "%x %d", &opcode, &pdata->rlen) <= 0) {
		pr_err("%s: get user-space data failed\n", __func__);
		return size;
	}
	if (pdata->rlen <= 0)
		pdata->rlen = 1;
	else if (pdata->rlen > SPI_MAX_RSIZE)
		pdata->rlen = SPI_MAX_RSIZE;
	pr_debug("%s: op:0x%02X, rlen = %d\n", __func__, opcode, pdata->rlen);

	memset(pdata->spi_data, 0, SPI_MAX_RSIZE);
	ret = spi_write_then_read(pdata->spi, &opcode, 1, pdata->spi_data, pdata->rlen);
	if (ret < 0)
		pr_err("%s: spi failed to access register(%d)\n", __func__, ret);

	return size;
}

static ssize_t asic_flash_spi_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	u8* reg_value = NULL;
	int i=0, ret=0;
	char tmp[5] = "0";
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);

	pr_debug("%s\n", __func__);
	reg_value = (unsigned char *) kmalloc((int)(size/3)+1 , GFP_KERNEL);
	if(reg_value == NULL) {
		pr_err("  ---> %s ERROR alloc memory failed\n", __func__);
		goto out;
	} else {
		pr_info("  ---> %s alloc %d bytes size for transfer!\n", __func__, (int)(size)/3+1);
	}

	pdata->wlen = 0;
	/* parsing register value */
	for (i=0 ; i<size-1 ; i+=3) {
		sprintf(tmp, "%c%c", buf[i], buf[i+1]);
		reg_value[(i/3)] = (u8) simple_strtol(tmp, NULL, 16);
		pdata->wlen++;
	}

	for (i=0 ; i<pdata->wlen ; i++) {
		pr_debug("  ---> buf[%d] = 0x%02X\n", i, reg_value[i]);
	}
	pr_debug("  ---> wlen = %d\n", pdata->wlen);

	ret = asic_flash_spi_write(pdata->spi, reg_value, pdata->wlen);
	if (ret < 0) {
		pr_err("%s: spi failed to access register(%d)\n", __func__, ret);
	}

out:
	kfree(reg_value);
	return size;
}

static int GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	pr_debug("filepath=%s\n", filepath);

	if(NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}

	if(IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return PTR_ERR(pfile);
	}

	inode=pfile->f_path.dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ReadFirmware(char * firmware_name, unsigned char * firmware_buf, off_t mem_size)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize, r_size = 0, t_size = 0;
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	pr_debug("  ---> filepath=%s\n", filepath);
	if(NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}

	if(IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return PTR_ERR(pfile);
	}

	inode=pfile->f_path.dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;

	if (mem_size < fsize) {
		t_size = -ENOMEM;
		goto out;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	do {
		r_size = vfs_read(pfile, firmware_buf, (fsize - t_size), &pos);
		pr_debug("Get fware size:%ld\n", r_size);
		if (r_size < 0) {
			t_size = r_size;
			break;
		}
		t_size += r_size;
	} while(t_size < fsize);
	set_fs(old_fs);

out:
	filp_close(pfile, NULL);
	return (int)t_size;
}

static ssize_t asic_flash_spi_write_firmware_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char* reg_value;
	int i=0;
	int ret=0;
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);
	char fwname[128];
	int fwsize = 0;

	pr_debug("%s\n", __func__);
	/* copy firmware name */
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[size-1] = '\0';

	fwsize = GetFirmwareSize(fwname);
	reg_value = (unsigned char *) kmalloc(fwsize+1 , GFP_KERNEL | GFP_DMA);
	if(reg_value == NULL) {
		pr_err("  ---> %s ERROR alloc %d memory failed\n", __func__, fwsize+1);
		goto out;
	} else {
		pr_info("  ---> %s alloc %d bytes size for transfer!\n", __func__, fwsize+1);
	}
	memset(reg_value, 0, fwsize+1);

	fwsize = ReadFirmware(fwname, reg_value, fwsize);
	if(fwsize < 0) {
		pr_err("  ---> %s ERROR: request firmware failed(%d)\n", __func__, fwsize);
		goto out;
	}

	pdata->wlen = fwsize;
	pr_debug("  ---> fwsize = %d\n", pdata->wlen);
	for (i=0 ; i<pdata->wlen ; i++) {
		pr_debug("  ---> buf[%d] = 0x%02X\n", i, reg_value[i]);
	}

	ret = asic_flash_spi_write(pdata->spi, reg_value, pdata->wlen);
	if (ret < 0) {
		pr_err("%s: spi failed to access register(%d)\n", __func__, ret);
	}

out:
	kfree(reg_value);
	return size;
}

static ssize_t asic_flash_spi_freq_show(struct device *dev, struct device_attribute *attr, char *buf){
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);
	int ret = 0;

	pr_debug("%s\n", __func__);
	ret += sprintf(buf + ret, "%dHz\n", pdata->spi->max_speed_hz);
	return ret;
}

static ssize_t asic_flash_spi_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	struct asic_flash_pdata *pdata = dev_get_drvdata(dev);

	pr_debug("%s\n", __func__);
	if (sscanf(buf, "%d", &pdata->spi->max_speed_hz) <= 0) {
		pr_err("%s: get user-space data failed\n", __func__);
		return size;
	}

	pr_info("%s: Set new frequency as %dHz\n", __func__, pdata->spi->max_speed_hz);
	return size;
}
static DEVICE_ATTR(eeprom, 0644, asic_flash_spi_eeprom_show, asic_flash_spi_eeprom_store);
static DEVICE_ATTR(read, 0644, asic_flash_spi_read_show, asic_flash_spi_read_store);
static DEVICE_ATTR(opread, 0644, asic_flash_spi_read_show, asic_flash_spi_opread_store);
static DEVICE_ATTR(write, 0200, NULL, asic_flash_spi_write_store);
static DEVICE_ATTR(firmware, 0200, NULL, asic_flash_spi_write_firmware_store);
static DEVICE_ATTR(frequency, 0644, asic_flash_spi_freq_show, asic_flash_spi_freq_store);

static struct attribute *asic_flash_spi_attr[] = {
	&dev_attr_eeprom.attr,
	&dev_attr_read.attr,
	&dev_attr_write.attr,
	&dev_attr_firmware.attr,
	&dev_attr_frequency.attr,
	NULL
};

const struct attribute_group spi_attr_group = {
	.attrs = asic_flash_spi_attr,
};

static int asic_flash_spi_probe(struct spi_device *spi){
	int cs, cpha, cpol, cs_high;
	int ret = 0;
	u32 max_speed;
	struct asic_flash_pdata *pdata = NULL;

	pr_info("%s\n", __func__);

	pdata = kmalloc(sizeof *(pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: Can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	cs = spi->chip_select;
	cpha = ( spi->mode & SPI_CPHA ) ? 1:0;
	cpol = ( spi->mode & SPI_CPOL ) ? 1:0;
	cs_high = ( spi->mode & SPI_CS_HIGH ) ? 1:0;
	max_speed = spi->max_speed_hz;

	pr_info("[CCB] cs [%x] CPHA [%x] CPOL[%x] CS_HIGH [%x]\n", cs, cpha, cpol, cs_high);
	pr_info("[CCB] Max_speed [%d]\n", max_speed);

	spi->bits_per_word = 8;

	/* Put platform data into spi structure */
	spi_set_drvdata(spi, pdata);
	pdata->spi = spi;

	/* initialize */
	memset( pdata->spi_data, 0, SPI_MAX_RSIZE*sizeof(u8) );
	pdata->wlen = 0;
	pdata->rlen = 0;

	ret = device_create_file(&spi->dev, &dev_attr_eeprom);
	if (ret) {
		pr_err("%s: Failed to register device %s:(%d)\n", __func__, dev_attr_eeprom.attr.name, ret);
	}

	ret = device_create_file(&spi->dev, &dev_attr_read);
	if (ret) {
		pr_err("%s: Failed to register device %s:(%d)\n", __func__, dev_attr_read.attr.name, ret);
	}

	ret = device_create_file(&spi->dev, &dev_attr_opread);
	if (ret) {
		pr_err("%s: Failed to register device %s:(%d)\n", __func__, dev_attr_opread.attr.name, ret);
	}

	ret = device_create_file(&spi->dev, &dev_attr_write);
	if (ret) {
		pr_err("%s: Failed to register device %s:(%d)\n", __func__, dev_attr_write.attr.name, ret);
	}

	ret = device_create_file(&spi->dev, &dev_attr_firmware);
	if (ret) {
		pr_err("%s: Failed to register devive %s:(%d)\n", __func__, dev_attr_firmware.attr.name, ret);
	}

	ret = device_create_file(&spi->dev, &dev_attr_frequency);
	if (ret) {
		pr_err("%s: Failed to register devive %s:(%d)\n", __func__, dev_attr_frequency.attr.name, ret);
	}

	return ret;
}

static struct spi_driver asic_flash_spi = {
	.driver = {
		.name = "asic_flash_spi",
		.owner  = THIS_MODULE,
		.of_match_table = asic_flash_spi_table,
	},
	.probe    = asic_flash_spi_probe,
};

static int __init asic_flash_spi_driver_init(void)
{
	int32_t rc = 0;

	rc = spi_register_driver(&asic_flash_spi);
	if (rc)
		pr_err("register spi driver fail:%d\n", rc);

	return rc;
}


static void __exit asic_flash_spi_driver_exit(void)
{
	spi_unregister_driver(&asic_flash_spi);
	return;
}

module_init(asic_flash_spi_driver_init);
module_exit(asic_flash_spi_driver_exit);
MODULE_DESCRIPTION("asic_flash_spi driver");
MODULE_LICENSE("GPL v2");
