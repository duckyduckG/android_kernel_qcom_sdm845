/*
 * MTD SPI driver for ST M25Pxx (and similar) serial flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

#define	MAX_CMD_SIZE		6
struct m25p {
	struct spi_device	*spi;
	struct spi_nor		spi_nor;
	u8			command[MAX_CMD_SIZE];
#ifdef CONFIG_ASIC_FLASH_SPI
#define READ_SIZE_MIN		1
#define READ_SIZE_MAX		4096
	unsigned int		raddr;
	int			rlen;
	u8			op_flag;
	u8			*read_data;
#endif
};

static int m25p80_read_reg(struct spi_nor *nor, u8 code, u8 *val, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	int ret;

	ret = spi_write_then_read(spi, &code, 1, val, len);
	if (ret < 0)
		dev_err(&spi->dev, "error %d reading %x\n", ret, code);

	return ret;
}

static void m25p_addr2cmd(struct spi_nor *nor, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (nor->addr_width * 8 -  8);
	cmd[2] = addr >> (nor->addr_width * 8 - 16);
	cmd[3] = addr >> (nor->addr_width * 8 - 24);
	cmd[4] = addr >> (nor->addr_width * 8 - 32);
}

static int m25p_cmdsz(struct spi_nor *nor)
{
	return 1 + nor->addr_width;
}

static int m25p80_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;

	flash->command[0] = opcode;
	if (buf)
		memcpy(&flash->command[1], buf, len);

	return spi_write(spi, flash->command, len + 1);
}

static ssize_t m25p80_write(struct spi_nor *nor, loff_t to, size_t len,
			    const u_char *buf)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	struct spi_transfer t[2] = {};
	struct spi_message m;
	int cmd_sz = m25p_cmdsz(nor);
	ssize_t ret;

	spi_message_init(&m);

	if (nor->program_opcode == SPINOR_OP_AAI_WP && nor->sst_write_second)
		cmd_sz = 1;

	flash->command[0] = nor->program_opcode;
	m25p_addr2cmd(nor, to, flash->command);

	t[0].tx_buf = flash->command;
	t[0].len = cmd_sz;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(spi, &m);
	if (ret)
		return ret;

	ret = m.actual_length - cmd_sz;
	if (ret < 0)
		return -EIO;
	return ret;
}

static inline unsigned int m25p80_rx_nbits(struct spi_nor *nor)
{
	switch (nor->flash_read) {
	case SPI_NOR_DUAL:
		return 2;
	case SPI_NOR_QUAD:
		return 4;
	default:
		return 0;
	}
}

/*
 * Read an address range from the nor chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static ssize_t m25p80_read(struct spi_nor *nor, loff_t from, size_t len,
			   u_char *buf)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	struct spi_transfer t[2];
	struct spi_message m;
	unsigned int dummy = nor->read_dummy;
	ssize_t ret;

	/* convert the dummy cycles to the number of bytes */
	dummy /= 8;

	if (spi_flash_read_supported(spi)) {
		struct spi_flash_read_message msg;

		memset(&msg, 0, sizeof(msg));

		msg.buf = buf;
		msg.from = from;
		msg.len = len;
		msg.read_opcode = nor->read_opcode;
		msg.addr_width = nor->addr_width;
		msg.dummy_bytes = dummy;
		/* TODO: Support other combinations */
		msg.opcode_nbits = SPI_NBITS_SINGLE;
		msg.addr_nbits = SPI_NBITS_SINGLE;
		msg.data_nbits = m25p80_rx_nbits(nor);

		ret = spi_flash_read(spi, &msg);
		if (ret < 0)
			return ret;
		return msg.retlen;
	}

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	flash->command[0] = nor->read_opcode;
	m25p_addr2cmd(nor, from, flash->command);

	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(nor) + dummy;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].rx_nbits = m25p80_rx_nbits(nor);
	t[1].len = min3(len, spi_max_transfer_size(spi),
			spi_max_message_size(spi) - t[0].len);
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(spi, &m);
	if (ret)
		return ret;

	ret = m.actual_length - m25p_cmdsz(nor) - dummy;
	if (ret < 0)
		return -EIO;
	return ret;
}

#ifdef CONFIG_ASIC_FLASH_SPI
static ssize_t asic_debug_read_show(struct device *dev, struct device_attribute *attr, char *buf) {
	struct m25p *flash = dev_get_drvdata(dev);
	int i = 0;
	int ret = 0;

	pr_debug("%s\n", __func__);
	if (flash->rlen < READ_SIZE_MIN)
		return -ENXIO;

	ret += sprintf(buf + ret, "From %s 0x%08X (len:%d):\n", (flash->op_flag?"reg":"offset"), flash->raddr, flash->rlen);
	for (i = 0; i < flash->rlen;) {
		ret += sprintf(buf + ret, "0x%02X ", *(flash->read_data+i));
		i++;
		if (!(i % 16))
			ret += sprintf(buf + ret, "\n");
	}
	if (i % 16)
		ret += sprintf(buf + ret, "\n");

	return ret;
}

//Format:$type(reg/ofs) $register/$start_address $length
static ssize_t asic_debug_read_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	struct m25p *flash = dev_get_drvdata(dev);
	struct spi_nor *nor = &flash->spi_nor;
	int ret = 0;
	int read_size = 0;
	int type_flag = 0;
	unsigned int read_addr = 0;

	pr_debug("%s\n", __func__);
	if (!strncmp(buf, "reg", 3))
		type_flag = 1;
	else if (!strncmp(buf, "ofs", 3))
		type_flag = 0;
	else {
		pr_err("%s: valid type is {reg, ofs}\n", __func__);
		return -EINVAL;
	}

	if (sscanf(buf, "%*s %x %u", &read_addr, &read_size) <= 0) {
		pr_err("%s: get user-space data failed\n", __func__);
		return -ENOMEM;
	}

	if ((read_size < READ_SIZE_MIN) || (read_size > READ_SIZE_MAX)) {
		pr_err("%s: size range %d~%d\n", __func__, READ_SIZE_MIN, READ_SIZE_MAX);
		return -EINVAL;
	}

	//allocate needed memory
	pr_debug("%s: %s = 0x%08X, read_size = %d\n", __func__, (type_flag?"reg":"offset"), read_addr, read_size);
	if (flash->rlen)
		kfree(flash->read_data); //release last memory
	flash->read_data = (u8*)kzalloc(read_size, GFP_KERNEL);
	if (!flash->read_data) {
		flash->rlen = 0;
		return -ENOMEM;
	}

	if (type_flag)
		ret = m25p80_read_reg(nor, (u8)read_addr, flash->read_data, read_size);
	else
		ret = m25p80_read(nor, read_addr, read_size, flash->read_data);
	if (ret < 0) {
		pr_err("%s: spi read failed(%d)\n", __func__, ret);
		return ret;
	}

	flash->raddr = read_addr;
	flash->rlen = read_size;
	flash->op_flag = type_flag;
	return size;
}

//Format:$type(reg/ofs) $register/$start_address $data...
static ssize_t asic_debug_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	struct m25p *flash = dev_get_drvdata(dev);
	struct spi_nor *nor = &flash->spi_nor;
	u8 *reg_value = NULL;
	int i = 0, ret = 0, item_num = 0, type_flag = 0, write_len = 0;
	char *tmp_buf[2] = {NULL}, *token = NULL, *src_tmp = NULL;
	unsigned int parse_value = 0, reg_addr = 0;

	pr_debug("%s\n", __func__);
	if (!strncasecmp(buf, "reg ", 4))
		type_flag = 1;
	else if (!strncasecmp(buf, "ofs ", 4))
		type_flag = 0;
	else {
		pr_err("%s: valid type is {reg, ofs}\n", __func__);
		return -EINVAL;
	}

	if (size <= 4)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		tmp_buf[i] = (char*)kzalloc(size-3, GFP_KERNEL);
		if (!tmp_buf[i]) {
			pr_err("%s: Allocate temp[%d] buffer failed\n", __func__, i);
			if (i == 1)
				kfree(tmp_buf[0]);
			return -ENOMEM;
		}
		memcpy(tmp_buf[i], buf+4, size-4);
	}

	//sort the number of register and value
	src_tmp = tmp_buf[0];
	while((token = strsep(&src_tmp, " ")) != NULL) {
		item_num++;
	}
	kfree(tmp_buf[0]);
	pr_debug("%s: sort %d items\n", __func__, item_num);

	if (item_num < 1) { //at least address
		pr_err("%s: No valid register and data to write\n", __func__);
		kfree(tmp_buf[1]);
		return -EINVAL;
	}

	//not allocat space for the fisrt item which is register or offset address
	if (item_num > 1) {
		reg_value = (u8*)kzalloc(item_num - 1, GFP_KERNEL);
		if(reg_value == NULL) {
			pr_err("%s: Allocate register buffer failed\n", __func__);
			kfree(tmp_buf[1]);
			return -ENOMEM;
		}
	}

	/* parsing register value */
	src_tmp = tmp_buf[1];
	for(i = 0; i < item_num; i++) {
		token = strsep(&src_tmp, " ");
		if (!token)
			break;
		ret = (u8)kstrtou32(token, 16, &parse_value);
		if (ret) {
			pr_err("%s: parse failed in %d token\n", __func__, i);
			break;
		}

		if (!i) { //first item is register/address
			reg_addr = parse_value;
		} else {
			reg_value[i-1] = (u8)(parse_value & 0xFF);
			write_len++;
		}
	}
	kfree(tmp_buf[1]);

	pr_debug("%s: To %s 0x%08X (len:%d)\n", __func__, (type_flag?"reg":"offset"), reg_addr, write_len);
	for (i = 0; i < write_len; i++) {
		pr_debug(" buf[%d] = 0x%02X\n", i, reg_value[i]);
	}

	if (type_flag)
		ret = m25p80_write_reg(nor, (u8)reg_addr, reg_value, write_len);
	else
		ret = m25p80_write(nor, reg_addr, write_len, reg_value);
	if (ret < 0) {
		pr_err("%s: spi write failed(%d)\n", __func__, ret);
		goto out;
	}

	return size;

out:
	if (item_num)
		kfree(reg_value);
	return ret;
}

static DEVICE_ATTR(read, 0644, asic_debug_read_show, asic_debug_read_store);
static DEVICE_ATTR(write, 0200, NULL, asic_debug_write_store);

static struct attribute *asic_spi_debug_attr[] = {
	&dev_attr_read.attr,
	&dev_attr_write.attr,
	NULL
};

const struct attribute_group asic_debug_attr_group = {
	.name = "asic_debug",
	.attrs = asic_spi_debug_attr,
};
#endif

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int m25p_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data;
	struct m25p *flash;
	struct spi_nor *nor;
	enum read_mode mode = SPI_NOR_NORMAL;
	char *flash_name;
	int ret;

	data = dev_get_platdata(&spi->dev);

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	nor = &flash->spi_nor;

	/* install the hooks */
	nor->read = m25p80_read;
	nor->write = m25p80_write;
	nor->write_reg = m25p80_write_reg;
	nor->read_reg = m25p80_read_reg;

	nor->dev = &spi->dev;
	spi_nor_set_flash_node(nor, spi->dev.of_node);
	nor->priv = flash;

	spi_set_drvdata(spi, flash);
	flash->spi = spi;

	if (spi->mode & SPI_RX_QUAD)
		mode = SPI_NOR_QUAD;
	else if (spi->mode & SPI_RX_DUAL)
		mode = SPI_NOR_DUAL;

	if (data && data->name)
		nor->mtd.name = data->name;

	/* For some (historical?) reason many platforms provide two different
	 * names in flash_platform_data: "name" and "type". Quite often name is
	 * set to "m25p80" and then "type" provides a real chip name.
	 * If that's the case, respect "type" and ignore a "name".
	 */
	if (data && data->type)
		flash_name = data->type;
	else if (!strcmp(spi->modalias, "spi-nor"))
		flash_name = NULL; /* auto-detect */
	else
		flash_name = spi->modalias;

	ret = spi_nor_scan(nor, flash_name, mode);
	if (ret)
		return ret;

#ifdef CONFIG_ASIC_FLASH_SPI
	flash->raddr = flash->rlen = flash->op_flag = 0;
	flash->read_data = NULL;
	ret = sysfs_create_group(&spi->dev.kobj, &asic_debug_attr_group);
	if (ret)
		return ret;
#endif

	return mtd_device_register(&nor->mtd, data ? data->parts : NULL,
				   data ? data->nr_parts : 0);
}


static int m25p_remove(struct spi_device *spi)
{
	struct m25p	*flash = spi_get_drvdata(spi);

#ifdef CONFIG_ASIC_FLASH_SPI
	if (flash->rlen)
		kfree(flash->read_data);
	sysfs_remove_group(&spi->dev.kobj, &asic_debug_attr_group);
#endif

	/* Clean up MTD stuff. */
	return mtd_device_unregister(&flash->spi_nor.mtd);
}

/*
 * Do NOT add to this array without reading the following:
 *
 * Historically, many flash devices are bound to this driver by their name. But
 * since most of these flash are compatible to some extent, and their
 * differences can often be differentiated by the JEDEC read-ID command, we
 * encourage new users to add support to the spi-nor library, and simply bind
 * against a generic string here (e.g., "jedec,spi-nor").
 *
 * Many flash names are kept here in this list (as well as in spi-nor.c) to
 * keep them available as module aliases for existing platforms.
 */
static const struct spi_device_id m25p_ids[] = {
	/*
	 * Allow non-DT platform devices to bind to the "spi-nor" modalias, and
	 * hack around the fact that the SPI core does not provide uevent
	 * matching for .of_match_table
	 */
	{"spi-nor"},

	/*
	 * Entries not used in DTs that should be safe to drop after replacing
	 * them with "spi-nor" in platform data.
	 */
	{"s25sl064a"},	{"w25x16"},	{"m25p10"},	{"m25px64"},

	/*
	 * Entries that were used in DTs without "jedec,spi-nor" fallback and
	 * should be kept for backward compatibility.
	 */
	{"at25df321a"},	{"at25df641"},	{"at26df081a"},
	{"mr25h256"},
	{"mx25l4005a"},	{"mx25l1606e"},	{"mx25l6405d"},	{"mx25l12805d"},
	{"mx25l25635e"},{"mx66l51235l"},
	{"n25q064"},	{"n25q128a11"},	{"n25q128a13"},	{"n25q512a"},
	{"s25fl256s1"},	{"s25fl512s"},	{"s25sl12801"},	{"s25fl008k"},
	{"s25fl064k"},
	{"sst25vf040b"},{"sst25vf016b"},{"sst25vf032b"},{"sst25wf040"},
	{"m25p40"},	{"m25p80"},	{"m25p16"},	{"m25p32"},
	{"m25p64"},	{"m25p128"},
	{"w25x80"},	{"w25x32"},	{"w25q32"},	{"w25q32dw"},
	{"w25q80bl"},	{"w25q128"},	{"w25q256"},
	/* ASIC SPI-NOR Flash */
	{"n25q016a11"}, {"w25q16fw"},

	/* Flashes that can't be detected using JEDEC */
	{"m25p05-nonjedec"},	{"m25p10-nonjedec"},	{"m25p20-nonjedec"},
	{"m25p40-nonjedec"},	{"m25p80-nonjedec"},	{"m25p16-nonjedec"},
	{"m25p32-nonjedec"},	{"m25p64-nonjedec"},	{"m25p128-nonjedec"},

	{ },
};
MODULE_DEVICE_TABLE(spi, m25p_ids);

static const struct of_device_id m25p_of_table[] = {
	/*
	 * Generic compatibility for SPI NOR that can be identified by the
	 * JEDEC READ ID opcode (0x9F). Use this, if possible.
	 */
	{ .compatible = "jedec,spi-nor" },
	{}
};
MODULE_DEVICE_TABLE(of, m25p_of_table);

static struct spi_driver m25p80_driver = {
	.driver = {
		.name	= "m25p80",
		.of_match_table = m25p_of_table,
	},
	.id_table	= m25p_ids,
	.probe	= m25p_probe,
	.remove	= m25p_remove,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

module_spi_driver(m25p80_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
