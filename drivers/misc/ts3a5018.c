/*
 * Driver for ts3a5018 switch chip.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>


enum ts3a5018_mode {
	MODE_SW_NONE = 0,
	MODE_SW_NC,
	MODE_SW_NO,

	/* This should be the last one */
	MODE_SW_COUNT
};

#define LABEL_SZ 8
const char default_labels[MODE_SW_COUNT][LABEL_SZ] = {
	[MODE_SW_NONE]	= "NONE",
	[MODE_SW_NC]	= "CLOSE",
	[MODE_SW_NO]	= "OPEN"
};

const int mode_gpio_map[MODE_SW_COUNT][2] = { /* en-gpio, in-gpio */
	[MODE_SW_NONE]	= {1, -1},
	[MODE_SW_NC]	= {0, 0},
	[MODE_SW_NO]	= {0, 1},
};

struct ts_gpio_info {
	int			exist;
	int			number;
	int			fixed_value;
};

struct ts3a5018_data {
	struct device 		*dev;
	struct regulator	*supply;
	struct mutex		io_lock;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*state_default;
	int			use_pinctrl;

	struct ts_gpio_info	gpio_en;
	struct ts_gpio_info	gpio_in;

	const char		*mode_labels[MODE_SW_COUNT];
	int			default_mode;
	int			current_mode;
};

static int ts3a5018_mode_change(struct ts3a5018_data *data, int mode)
{
	int en_value = 0, in_value = 0;

	pr_debug("Mode: %d to %d\n", data->current_mode, mode);

	if (mode == data->current_mode)
		return 0;

	if (mode < 0 || mode >= MODE_SW_COUNT) {
		pr_err("Wrong mode:%d\n", mode);
		return -EINVAL;
	}

	en_value = mode_gpio_map[mode][0];
	in_value = mode_gpio_map[mode][1];

	mutex_lock(&data->io_lock);
	if (data->gpio_en.exist && en_value != -1) {
		gpio_direction_output(data->gpio_en.number, en_value);
		pr_debug("After set gpio_en(%d) as %d\n", data->gpio_en.number, en_value);
	}
	if (data->gpio_in.exist && in_value != -1) {
		gpio_direction_output(data->gpio_in.number, in_value);
		pr_debug("After set gpio_in(%d) as %d\n", data->gpio_in.number, in_value);
	}
	mutex_unlock(&data->io_lock);

	data->current_mode = mode;
	return 0;
}

static ssize_t ts3a5018_sysfs_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ts3a5018_data *data = dev_get_drvdata(dev);
	int en_value = 0, in_value = 0, mode;

	if (!data)
		return -ENODEV;

	mutex_lock(&data->io_lock);
	if (data->gpio_en.exist)
		en_value = gpio_get_value(data->gpio_en.number);
	else
		en_value = data->gpio_en.fixed_value;
	if (data->gpio_in.exist)
		in_value = gpio_get_value(data->gpio_in.number);
	else
		in_value = data->gpio_in.fixed_value;
	mutex_unlock(&data->io_lock);

	if (en_value)
		mode = MODE_SW_NONE;
	else if (in_value)
		mode = MODE_SW_NO;
	else
		mode = MODE_SW_NC;

	if (mode != data->current_mode) {
		pr_debug("Mode change from %d to %d by pin value\n", data->current_mode, mode);
		data->current_mode = mode;
	}

	return sprintf(buf, "%s\n", data->mode_labels[data->current_mode]);
}

static ssize_t ts3a5018_sysfs_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ts3a5018_data *data = dev_get_drvdata(dev);
	int i;

	if (!data)
		return -ENODEV;

	for (i = 0; i < MODE_SW_COUNT; i++) {
		pr_debug("Buf:%s label[%d]:%s\n", buf, i, data->mode_labels[i]);
		if (!strncasecmp(buf, data->mode_labels[i], strlen(data->mode_labels[i]))) //match
			break;
	}

	if (i >= MODE_SW_COUNT) {
		pr_err("Wrong mode string:%s\n", buf);
		return -EINVAL;
	}

	if (!ts3a5018_mode_change(data, i))
		return count;
	else
		return -EFAULT;
}

static DEVICE_ATTR(mode, (S_IWUSR | S_IRUGO), ts3a5018_sysfs_mode_show, ts3a5018_sysfs_mode_store);
static struct attribute *ts3a5018_attr[] = {
	&dev_attr_mode.attr,
	NULL
};
const struct attribute_group ts3a5018_attr_group = {
	.name = "control",
	.attrs = ts3a5018_attr,
};

/* ----------------------------- Driver functions ----------------------------- */
static int parse_dts_to_data(struct ts3a5018_data *data, struct platform_device *pdev)
{
	struct device_node *dev_node = pdev->dev.of_node;
	int rc = 0, i = 0;

	if (!dev_node) {
		pr_err("No dev node\n");
		return -ENODEV;
	}

	data->supply = devm_regulator_get(&pdev->dev, "vin");
	if (IS_ERR(data->supply)) {
		if (PTR_ERR(data->supply) != -EPROBE_DEFER)
			pr_err("Get vin-supply failed:%ld\n", PTR_ERR(data->supply));
		return PTR_ERR(data->supply);
	}

	rc = of_get_named_gpio(dev_node, "gpio-en", 0);
	if (rc >= 0) {
		data->gpio_en.exist = 1;
		data->gpio_en.number = rc;
	} else {
		rc = of_property_read_s32(dev_node, "en-fix-value", &data->gpio_en.fixed_value);
		if (rc < 0) {
			pr_err("Get gpio-en fix value fail:%d\n", rc);
			return rc;
		}
	}

	rc = of_get_named_gpio(dev_node, "gpio-in", 0);
	if (rc >= 0) {
		data->gpio_in.exist = 1;
		data->gpio_in.number = rc;
	} else {
		rc = of_property_read_s32(dev_node, "in-fix-value", &data->gpio_in.fixed_value);
		if (rc < 0) {
			pr_err("Get gpio-in fix value fail:%d\n", rc);
			return rc;
		}
	}

	/* option */
	rc = of_property_count_strings(dev_node, "mode-labels");
	if (rc >= MODE_SW_COUNT) {
		for (i = 0; i < MODE_SW_COUNT; i++) {
			rc = of_property_read_string_index(dev_node, "mode-labels", i, &data->mode_labels[i]);
			if (rc) {
				pr_err("Get mode-lables[%d] failed:%d\n", i, rc);
				break;
			}
		}
	}
	if (i < MODE_SW_COUNT) {
		pr_debug("Use default mode lables\n");
		for (i = 0; i < MODE_SW_COUNT; i++)
			data->mode_labels[i] = default_labels[i];
	}

	rc = of_property_read_u32(dev_node, "default-mode", &data->default_mode);
	if (rc < 0)
		data->default_mode = MODE_SW_NONE;

	data->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(data->pinctrl))
		pr_err("Failed to get pinctrl:%ld\n", PTR_ERR(data->pinctrl));
	else
		data->use_pinctrl = 1;

	if (data->use_pinctrl) {
		data->state_default = pinctrl_lookup_state(data->pinctrl, "default");
		if (IS_ERR(data->state_default)) {
			pr_err("Could not get pinctrl default state\n");
			data->state_default = NULL;
		}
	} else {
		pr_info("Not use pinctrl\n");
	}

	return 0;
}

static int ts3a5018_hw_init(struct ts3a5018_data *data)
{
	int rc = 0;

	rc = regulator_enable(data->supply);
	if (rc) {
		pr_err("Enable regulator failed:%d\n", rc);
		return rc;
	}

	if (data->use_pinctrl && data->state_default)
		pinctrl_select_state(data->pinctrl, data->state_default);

	if (data->gpio_en.exist) {
		rc = gpio_request(data->gpio_en.number, "ts3a_en");
		if (rc < 0) {
			pr_err("Request gpio_en(%d) failed (%d)\n", data->gpio_en.number, rc);
			rc = -EBUSY;
			goto err_req1;
		}
	}

	if (data->gpio_in.exist) {
		rc = gpio_request(data->gpio_in.number, "ts3a_in");
		if (rc < 0) {
			pr_err("Request gpio_in(%d) failed (%d)\n", data->gpio_in.number, rc);
			rc = -EBUSY;
			goto err_req2;
		}
	}

	return 0;

err_req2:
	gpio_free(data->gpio_en.number);
err_req1:
	regulator_disable(data->supply);
	return rc;
}

static void ts3a5018_hw_deinit(struct ts3a5018_data *data)
{
	if (data->gpio_in.exist)
		gpio_free(data->gpio_in.number);
	if (data->gpio_en.exist)
		gpio_free(data->gpio_en.number);
	regulator_disable(data->supply);
}

static int ts3a5018_probe(struct platform_device *pdev)
{
	struct ts3a5018_data *data = NULL;
	int ret = 0;

	pr_info("Start\n");

	data = (struct ts3a5018_data*)kzalloc(sizeof(struct ts3a5018_data), GFP_KERNEL);
	if (!data) {
		pr_err("Failed to allocate data\n");
		return -ENOMEM;
	}

	ret = parse_dts_to_data(data, pdev);
	if (ret < 0)
		goto err_pdts;

	data->dev = &pdev->dev;
	data->current_mode = MODE_SW_NONE;
	mutex_init(&data->io_lock);
	platform_set_drvdata(pdev, data);

	ret = ts3a5018_hw_init(data);
	if (ret)
		goto err_hwinit;

	ret = sysfs_create_group(&pdev->dev.kobj, &ts3a5018_attr_group);
	if (ret) {
		pr_err("Create sysfs failed:%d\n", ret);
		goto err_sysfs;
	}

	ts3a5018_mode_change(data, data->default_mode);

	pr_info("Done\n");
	return 0;

err_sysfs:
	ts3a5018_hw_deinit(data);
err_hwinit:
	platform_set_drvdata(pdev, NULL);
err_pdts:
	kfree(data);
	return ret;
}

static int ts3a5018_remove(struct platform_device *pdev)
{
	struct ts3a5018_data *data = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &ts3a5018_attr_group);
	ts3a5018_hw_deinit(data);
	platform_set_drvdata(pdev, NULL);
	kfree(data);
	return 0;
}

static const struct of_device_id ts3a5018_match[] = {
	{.compatible = "ti,ts3a5018"},
	{}
};

static struct platform_driver ts3a5018_driver = {
	.driver = {
		.name = "ts3a5018",
		.of_match_table = ts3a5018_match,
	},
	.probe		= ts3a5018_probe,
	.remove		= ts3a5018_remove,
};

static int __init ts3a5018_init(void)
{
	return platform_driver_register(&ts3a5018_driver);
}

static void __exit ts3a5018_exit(void)
{
	platform_driver_unregister(&ts3a5018_driver);
}

module_init(ts3a5018_init);
module_exit(ts3a5018_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI ts3a5018 analog switch driver");
