/*
 * Egis ET7xx Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the Egis fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of controlling GPIOs such as sensor reset line,
 * sensor  IRQ line.
 *
 * Copyright (c) 2017 Egis Technology Inc. <www.egistec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#include "et7xx.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sysfs.h>

#include <linux/pinctrl/consumer.h>

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static struct etspi_data *g_data;
static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);
static unsigned int bufsiz = 1024;
static int ET7XX_MAJOR;
module_param(bufsiz, uint, 0444);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");


/*-------------------------------------------------------------------------*/

static void etspi_reset(struct etspi_data *etspi)
{
	pr_info("%s start\n", __func__);
	gpio_set_value(etspi->rst_gpio, 0);
	usleep_range(10000, 10500);
	gpio_set_value(etspi->rst_gpio, 1);
	pr_info("%s end\n", __func__);
}


static void etspi_power_control(struct etspi_data *etspi, int status)
{
	pr_info("%s status = %d\n", __func__, status);
	if (status == 1) {
		if (etspi->enable_gpio)
			gpio_set_value(etspi->enable_gpio, 1);
	} else if (status == 0) {
		if (etspi->enable_gpio)
			gpio_set_value(etspi->enable_gpio, 0);
	} else {
		pr_err("%s can't support this value. %d\n", __func__, status);
	}
}

static ssize_t etspi_read(struct file *filp,
						char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t etspi_write(struct file *filp,
						const char __user *buf,
						size_t count,
						loff_t *f_pos)
{
/*Implement by vendor if needed*/
	return 0;
}

static long etspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int err = 0;
	struct etspi_data *etspi;
	u32 tmp;
	struct egis_ioc_transfer *ioc = NULL;
#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
	struct egis_ioc_transfer_32 *ioc_32 = NULL;
	u64 tx_buffer_64, rx_buffer_64;
#endif
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != EGIS_IOC_MAGIC) {
		pr_err("%s _IOC_TYPE(cmd) != EGIS_IOC_MAGIC", __func__);
		return -ENOTTY;
	}


	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err) {
		pr_err("%s err", __func__);
		return -EFAULT;
	}

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	etspi = filp->private_data;
#if defined(USE_SPI_BUS)
	spin_lock_irq(&etspi->spi_lock);
	spi = spi_dev_get(etspi->spi);
	spin_unlock_irq(&etspi->spi_lock);

	if (spi == NULL) {
		pr_err("%s spi == NULL", __func__);
		return -ESHUTDOWN;
	}
#endif
	mutex_lock(&etspi->buf_lock);

	/* segmented and/or full-duplex I/O request */
	if (_IOC_NR(cmd) != _IOC_NR(EGIS_IOC_MESSAGE(0))
					|| _IOC_DIR(cmd) != _IOC_WRITE) {
		retval = -ENOTTY;
		goto out;
	}

	/*
	 *	If platform is 32bit and kernel is 64bit
	 *	We will alloc egis_ioc_transfer for 64bit and 32bit
	 *	We use ioc_32(32bit) to get data from user mode.
	 *	Then copy the ioc_32 to ioc(64bit).
	 */
#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct egis_ioc_transfer_32)) != 0) {
		pr_err("%s ioc_32 size error\n", __func__);
		retval = -EINVAL;
		goto out;
	}
	ioc_32 = kmalloc(tmp, GFP_KERNEL);
	if (ioc_32 == NULL) {
		retval = -ENOMEM;
		pr_err("%s ioc_32 kmalloc error\n", __func__);
		goto out;
	}
	if (__copy_from_user(ioc_32, (void __user *)arg, tmp)) {
		retval = -EFAULT;
		pr_err("%s ioc_32 copy_from_user error\n", __func__);
		goto out;
	}
	ioc = kmalloc(sizeof(struct egis_ioc_transfer), GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		pr_err("%s ioc kmalloc error\n", __func__);
		goto out;
	}
	tx_buffer_64 = (u64)ioc_32->tx_buf;
	rx_buffer_64 = (u64)ioc_32->rx_buf;
	ioc->tx_buf = (u8 *)tx_buffer_64;
	ioc->rx_buf = (u8 *)rx_buffer_64;
	ioc->len = ioc_32->len;
	ioc->speed_hz = ioc_32->speed_hz;
	ioc->delay_usecs = ioc_32->delay_usecs;
	ioc->bits_per_word = ioc_32->bits_per_word;
	ioc->cs_change = ioc_32->cs_change;
	ioc->opcode = ioc_32->opcode;
	ioc->sector = ioc_32->sector;
	ioc->umax = ioc_32->umax;
	ioc->umin = ioc_32->umin;
	ioc->flag = ioc_32->flag;
	memcpy(ioc->pad, ioc_32->pad, 3);
	kfree(ioc_32);
#else
	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct egis_ioc_transfer)) != 0) {
		pr_err("%s ioc size error\n", __func__);
		retval = -EINVAL;
		goto out;
	}
	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		goto out;
	}
	if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
		pr_err("%s __copy_from_user error\n", __func__);
		retval = -EFAULT;
		goto out;
	}
#endif
	switch (ioc->opcode) {

	case FP_SENSOR_RESET:
		pr_info("%s FP_SENSOR_RESET\n", __func__);
		etspi_reset(etspi);
		break;

	case FP_RESET_SET:
		break;


	case FP_SET_IRQ_HIGH:
		gpio_set_value(etspi->irq_gpio, 1);
		pr_info("%s FP_SET_IRQ_HIGH  \n", __func__);
		break;

	case FP_SET_IRQ_LOW:
		gpio_set_value(etspi->irq_gpio, 0);
		pr_info("%s FP_SET_IRQ_LOW  \n", __func__);
		break;
	case FP_SET_RESET_HIGH:
		gpio_set_value(etspi->rst_gpio, 1);
		pr_info("%s FP_SET_RESET_HIGH  \n", __func__);
		break;

	case FP_SET_RESET_LOW:
		gpio_set_value(etspi->rst_gpio, 0);
		pr_info("%s FP_SET_RESET_LOW  \n", __func__);
		break;


	case FP_POWER_CONTROL:
	case FP_POWER_CONTROL_ET7XX:
		pr_info("%s FP_POWER_CONTROL, status = %d\n",
			__func__, ioc->len);
		etspi_power_control(etspi, ioc->len);
		break;
#if defined(USE_SPI_BUS)
	case FP_SET_SPI_CLOCK:
		pr_info("%s FP_SET_SPI_CLOCK, clock = %d\n",
			__func__, ioc->speed_hz);
		spi->max_speed_hz = ioc->speed_hz;
		break;
#endif
	case FP_IOCTL_RESERVED_01:
		break;
	case FP_IOCTL_RESERVED_02:
		break;
	default:
		retval = -EFAULT;
		break;

	}

out:
	if (ioc != NULL)
		kfree(ioc);

	mutex_unlock(&etspi->buf_lock);
#if defined(USE_SPI_BUS)
	spi_dev_put(spi);
#endif
	if (retval < 0)
		pr_err("%s retval = %d\n", __func__, retval);
	return retval;
}

#ifdef CONFIG_COMPAT
static long etspi_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return etspi_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define etspi_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int etspi_open(struct inode *inode, struct file *filp)
{
	struct etspi_data *etspi;
	int	status = -ENXIO;

	pr_info("%s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(etspi, &device_list, device_entry) {
		if (etspi->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (status == 0) {
			etspi->users++;
			filp->private_data = etspi;
			nonseekable_open(inode, filp);
			etspi->bufsiz = bufsiz;
		}
	} else
		pr_debug("%s nothing for minor %d\n"
			, __func__, iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int etspi_release(struct inode *inode, struct file *filp)
{
	struct etspi_data *etspi;

	pr_info("%s\n", __func__);
	mutex_lock(&device_list_lock);
	etspi = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	etspi->users--;
	if (etspi->users == 0) {
		int	dofree;


		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&etspi->spi_lock);
		dofree = (etspi->spi == NULL);
		spin_unlock_irq(&etspi->spi_lock);

		if (dofree)
			kfree(etspi);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

int etspi_platform_init(struct etspi_data *etspi)
{
	int status = 0;

	pr_info("%s\n", __func__);
	/* gpio setting for vdd, vcc, irq */
	if (etspi != NULL) {
		etspi->drdy_irq_flag = DRDY_IRQ_DISABLE;

		if (etspi->vdd_dig_gpio) {
			status = gpio_request(etspi->vdd_dig_gpio,
					"etspi-gpio-vdd_dig");
			if (status < 0) {
				pr_err("%s gpio_request etspi-gpio-vdd_dig failed\n",
					__func__);
				goto etspi_platform_init_vdd_failed;
			}
			status = gpio_direction_output(etspi->vdd_dig_gpio, 0);
			if (status < 0) {
				pr_err("%s gpio_direction_output etspi-gpio-vdd_dig  failed\n",
						__func__);
				status = -EBUSY;
				goto etspi_platform_init_vdd_failed;
			}
		}
		if (etspi->rst_gpio) {
			status = gpio_request(etspi->rst_gpio,
					"egisfp-gpio-reset");
			if (status < 0) {
				pr_err("%s gpio_request egisfp-gpio-reset failed\n",
					__func__);
				goto etspi_platform_init_rst_failed;
			}

			status = gpio_direction_output(etspi->rst_gpio,1);
			if (status < 0) {
				pr_err("%s gpio_direction_input egisfp-gpio-reset failed\n",
					__func__);
				goto etspi_platform_init_rst_failed;
			}
			gpio_set_value(etspi->rst_gpio, 1);
		}
		if (etspi->irq_gpio) {
			status = gpio_request(etspi->irq_gpio,
					"egisfp-gpio-irq");
			if (status < 0) {
				pr_err("%s gpio_request egisfp-gpio-irq failed\n",
					__func__);
				goto etspi_platform_init_irq_failed;
			}

			status = gpio_direction_output(etspi->irq_gpio,0);
			if (status < 0) {
				pr_err("%s gpio_direction_input egisfp-gpio-irq failed\n",
					__func__);
				goto etspi_platform_init_irq_failed;
			}
			gpio_set_value(etspi->irq_gpio, 0);
		}
		if (etspi->enable_gpio) {
                        status = gpio_request(etspi->enable_gpio,
                                        "egisfp-gpio-enable");
                        if (status < 0) {
                                pr_err("%s gpio_request egisfp-gpio-enable failed\n",
                                        __func__);
                                goto etspi_platform_init_enable_failed;
                        }

                        status = gpio_direction_output(etspi->enable_gpio, 0);
                        if (status < 0) {
                                pr_err("%s gpio_direction_output egisfp-gpio-enable failed\n",
                                        __func__);
                                goto etspi_platform_init_enable_failed;
                        }
                }

		pr_info("%s vdd_dig_gpio value =%d\n"
				"%s irq_gpio value =%d\n"
				"%s enable_gpio value =%d\n"
				"%s rst_gpio value =%d\n",
				__func__, gpio_get_value(etspi->vdd_dig_gpio),
				__func__, gpio_get_value(etspi->irq_gpio),
				__func__, gpio_get_value(etspi->enable_gpio),
				__func__, gpio_get_value(etspi->rst_gpio));
	} else {
		status = -EFAULT;
	}

	if (status == -EFAULT)
		pr_err("%s failed status=%d\n", __func__, status);
	else
		pr_info("%s successful status=%d\n", __func__, status);
	return status;
etspi_platform_init_irq_failed:
	gpio_free(etspi->irq_gpio);
etspi_platform_init_rst_failed:
	if (etspi->rst_gpio)
		gpio_free(etspi->rst_gpio);
etspi_platform_init_vdd_failed:
	if (etspi->vdd_dig_gpio)
		gpio_free(etspi->vdd_dig_gpio);
etspi_platform_init_enable_failed:
        if (etspi->enable_gpio)
                gpio_free(etspi->enable_gpio);
	pr_err("%s is failed\n", __func__);
	return status;
}

void etspi_platform_uninit(struct etspi_data *etspi)
{
	pr_info("%s\n", __func__);

	if (etspi != NULL) {
		etspi->drdy_irq_flag = DRDY_IRQ_DISABLE;
		if (etspi->vdd_dig_gpio)
			gpio_free(etspi->vdd_dig_gpio);
		if (etspi->irq_gpio)
			gpio_free(etspi->irq_gpio);
		if (etspi->enable_gpio)
			gpio_free(etspi->enable_gpio);
		if (etspi->rst_gpio)
			gpio_free(etspi->rst_gpio);
	}
}

static int etspi_parse_dt(struct device *dev,
	struct etspi_data *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int errorno = 0;
	int gpio;

	gpio = of_get_named_gpio_flags(np, "egisfp-gpio-enable",
		0, &flags);
	if (gpio < 0) {
		errorno = gpio;
		pr_err("%s: fail to get enable_gpio\n", __func__);
		goto dt_exit;
	} else {
		data->enable_gpio = gpio;
		pr_info("%s: enable_gpio=%d\n",
			__func__, data->enable_gpio);
	}
	gpio = of_get_named_gpio_flags(np, "egisfp-gpio-reset",
		0, &flags);
	if (gpio < 0) {
		errorno = gpio;
		pr_err("%s: fail to get rst_gpio\n", __func__);
		goto dt_exit;
	} else {
		data->rst_gpio = gpio;
		pr_info("%s: rst_gpio=%d\n",
			__func__, data->rst_gpio);
	}
	gpio = of_get_named_gpio_flags(np, "egisfp-gpio-irq",
		0, &flags);
	if (gpio < 0) {
		data->irq_gpio = 0;
		pr_err("%s: fail to get vdd_dig_gpio\n", __func__);
		goto dt_exit;
	} else {
		data->irq_gpio = gpio;
		pr_info("%s: irq_gpio=%d\n",
			__func__, data->irq_gpio);
	}

	pr_info("%s is successful\n", __func__);
	return errorno;
dt_exit:
	pr_err("%s is failed\n", __func__);
	return errorno;
}

static const struct file_operations etspi_fops = {
	.owner = THIS_MODULE,
	.write = etspi_write,
	.read = etspi_read,
	.unlocked_ioctl = etspi_ioctl,
	.compat_ioctl = etspi_compat_ioctl,
	.open = etspi_open,
	.release = etspi_release,
	.llseek = no_llseek,
};

static struct class *etspi_class;

#if defined(USE_SPI_BUS)
static int etspi_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int etspi_probe(struct platform_device *pdev)
#endif
{
	struct etspi_data *etspi;
	int status;
	unsigned long minor;

	pr_info("%s\n", __func__);

	/* Allocate driver data */
	etspi = kzalloc(sizeof(*etspi), GFP_KERNEL);
	if (etspi == NULL)
		return -ENOMEM;

	/* device tree call */
	pr_info("%s, etspi_parse_dt\n", __func__);
#if defined(USE_SPI_BUS)
	status = etspi_parse_dt(&spi->dev, etspi);
#elif defined(USE_PLATFORM_BUS)
	status = etspi_parse_dt(&pdev->dev, etspi);
#endif
	if (status) {
		pr_err("%s - Failed to parse DT\n", __func__);
		goto etspi_probe_parse_dt_failed;
	}

	/* Initialize the driver data */
#if defined(USE_SPI_BUS)
	etspi->spi = spi;
#elif defined(USE_PLATFORM_BUS)
	etspi->spi = pdev;
#endif
	g_data = etspi;

	spin_lock_init(&etspi->spi_lock);
	mutex_init(&etspi->buf_lock);
	mutex_init(&device_list_lock);

	INIT_LIST_HEAD(&etspi->device_entry);

	/* platform init */
	status = etspi_platform_init(etspi);
	if (status != 0) {
		pr_err("%s platforminit failed\n", __func__);
		goto etspi_probe_platform_init_failed;
	}
#if defined(USE_SPI_BUS)
	spi->bits_per_word = 8;
	spi->max_speed_hz = SLOW_BAUD_RATE;
	spi->mode = SPI_MODE_0;
	spi->chip_select = 0;
#endif
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		etspi->devt = MKDEV(ET7XX_MAJOR, minor);
		dev = device_create(etspi_class, &etspi->spi->dev, etspi->devt,
				    etspi, "esfp0");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else{
		dev_dbg(&etspi->spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&etspi->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
#if defined(USE_SPI_BUS)
	if (status == 0)
		spi_set_drvdata(etspi->spi, etspi);
	else
		goto etspi_create_failed;
#endif
	pr_info("%s is successful\n", __func__);

	return status;
#if defined(USE_SPI_BUS)
etspi_create_failed:
	etspi_platform_uninit(etspi);
#endif
etspi_probe_platform_init_failed:
etspi_probe_parse_dt_failed:
	kfree(etspi);
	pr_err("%s is failed\n", __func__);

	return status;
}

#if defined(USE_SPI_BUS)
static int etspi_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int etspi_remove(struct platform_device *pdev)
#endif
{
	pr_info("%s\n", __func__);

	if (g_data != NULL) {
		etspi_platform_uninit(g_data);
		/* make sure ops on existing fds can abort cleanly */
		spin_lock_irq(&g_data->spi_lock);
		g_data->spi = NULL;
	#if defined(USE_SPI_BUS)
		spi_set_drvdata(spi, NULL);
	#endif
		spin_unlock_irq(&g_data->spi_lock);

		/* prevent new opens */
		mutex_lock(&device_list_lock);

		list_del(&g_data->device_entry);
		device_destroy(etspi_class, g_data->devt);
		clear_bit(MINOR(g_data->devt), minors);
		if (g_data->users == 0)
			kfree(g_data);
		mutex_unlock(&device_list_lock);
	}
	return 0;
}


static const struct of_device_id etspi_match_table[] = {
	{ .compatible = "egis,fingerprint",},
	{},
};

#if defined(USE_SPI_BUS)
static struct spi_driver etspi_spi_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver etspi_spi_driver = {
#endif
	.driver = {
		.name =	"egis_fingerprint",
		.owner = THIS_MODULE,
		.of_match_table = etspi_match_table
	},
	.probe = etspi_probe,
	.remove = etspi_remove,
};

/*-------------------------------------------------------------------------*/

static int __init etspi_init(void)
{
	int status;

	pr_info("%s\n", __func__);

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(ET7XX_MAJOR, "egis_fingerprint", &etspi_fops);
	if (status < 0) {
		pr_err("%s register_chrdev error.\n", __func__);
		return status;
	}
	ET7XX_MAJOR = status;
	etspi_class = class_create(THIS_MODULE, "egis_fingerprint");
	if (IS_ERR(etspi_class)) {
		pr_err("%s class_create error.\n", __func__);
		unregister_chrdev(ET7XX_MAJOR, etspi_spi_driver.driver.name);
		return PTR_ERR(etspi_class);
	}
#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&etspi_spi_driver);
#elif defined(USE_SPI_BUS)
        status = spi_register_driver(&etspi_spi_driver);
#endif
	if (status < 0) {
		pr_err("%s spi_register_driver error.\n", __func__);
		class_destroy(etspi_class);
		unregister_chrdev(ET7XX_MAJOR, etspi_spi_driver.driver.name);
		return status;
	}

	pr_info("%s is successful\n", __func__);

	return status;
}

static void __exit etspi_exit(void)
{
	pr_info("%s\n", __func__);
#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&etspi_spi_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&etspi_spi_driver);
#endif
	class_destroy(etspi_class);
	unregister_chrdev(ET7XX_MAJOR, etspi_spi_driver.driver.name);
}

module_init(etspi_init);
module_exit(etspi_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Kao, <vincent.kao@egistec.com>");
MODULE_DESCRIPTION("Egis ET7xx Fingerprint sensor device driver.");
