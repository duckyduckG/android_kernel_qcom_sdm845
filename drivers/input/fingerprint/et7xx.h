/*
 * Copyright (c) 2017 Egis Technology Inc. <www.egistec.com>. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef _ET7XX_LINUX_DIRVER_H_
#define _ET7XX_LINUX_DIRVER_H_

#ifdef ENABLE_SENSORS_FPRINT_SECURE
#define FEATURE_SPI_WAKELOCK
#endif /* CONFIG_SEC_FACTORY */

#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/platform_data/spi-s3c64xx.h>
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spidev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#if defined(CONFIG_SECURE_OS_BOOSTER_API)
#if defined(CONFIG_SOC_EXYNOS8890) || defined(CONFIG_SOC_EXYNOS7870) \
	|| defined(CONFIG_SOC_EXYNOS7880) || defined(CONFIG_SOC_EXYNOS7570) \
	|| defined(CONFIG_SOC_EXYNOS7885)
#include <soc/samsung/secos_booster.h>
#else
#include <mach/secos_booster.h>
#endif
#endif

struct sec_spi_info {
	int		port;
	unsigned long	speed;
};
#endif

/*
 * This feature is temporary for exynos AP only.
 * It's for control GPIO config on enabled TZ before enable GPIO protection.
 * If it's still defined this feature after enable GPIO protection,
 * it will be happened kernel panic
 * So it should be un-defined after enable GPIO protection
 */
#undef DISABLED_GPIO_PROTECTION


#ifdef ET7XX_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...) pr_err(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

#define VENDOR						"EGISTEC"
#define CHIP_ID						"ET7XX"
#define  USE_PLATFORM_BUS     				1
/* assigned */
//#define ET7XX_MAJOR					153
/* ... up to 256 */
#define N_SPI_MINORS					32

#define OP_CIS_ADDR_R					0x20
#define OP_CIS_REG_W					0x30
#define OP_GET_FRAME					0x61
#define OP_PRE_CAPTURE					0x62
#define OP_REG_W					0x70
#define OP_REG_R					0x71
#define BITS_PER_WORD					8

#define SLOW_BAUD_RATE					12500000

#define DRDY_IRQ_ENABLE					1
#define DRDY_IRQ_DISABLE				0


#define FP_REGISTER_READ				0x01
#define FP_REGISTER_WRITE				0x02
#define FP_GET_FRAME					0x03
#define FP_SENSOR_RESET					0x04
#define FP_POWER_CONTROL				0x05
#define FP_SET_SPI_CLOCK				0x06
#define FP_RESET_SET					0x07
#define FP_CIS_REGISTER_READ				0x08
#define FP_CIS_REGISTER_WRITE				0x09
#define FP_CIS_PRE_CAPTURE				0x0A
#define FP_CIS_PRE_CAPTURE_EX				0x0B
#define FP_GET_FRAME_EX					0x0C
#define FP_TRANSFER_COMMAND				0x0D
#define FP_TRANSFER_COMMAND_EX				0x0E
#define FP_WAIT_DRDY					0x0F

#define FP_SET_IRQ_HIGH					0x20
#define FP_SET_IRQ_LOW					0x21
#define FP_SET_RESET_HIGH				0x22
#define FP_SET_RESET_LOW				0x23


#ifdef ENABLE_SENSORS_FPRINT_SECURE
#define FP_DIABLE_SPI_CLOCK				0x10
#define FP_CPU_SPEEDUP					0x11
#define FP_SET_SENSOR_TYPE				0x14
/* Do not use ioctl number 0x15 */
#define FP_SET_LOCKSCREEN				0x16
#define FP_SET_WAKE_UP_SIGNAL				0x17
#endif
#define FP_SENSOR_ORIENT				0x19
#define FP_POWER_CONTROL_ET7XX				0x18
#define FP_IOCTL_RESERVED_01				0x12
#define FP_IOCTL_RESERVED_02				0x13

/* trigger signal initial routine */
#define INT_TRIGGER_INIT				0xa4
/* trigger signal close routine */
#define INT_TRIGGER_CLOSE				0xa5
/* read trigger status */
#define INT_TRIGGER_READ				0xa6
/* polling trigger status */
#define INT_TRIGGER_POLLING				0xa7
/* polling abort */
#define INT_TRIGGER_ABORT				0xa8

#define SHIFT_BYTE_OF_IMAGE 0
#define DIVISION_OF_IMAGE 4
#define LARGE_SPI_TRANSFER_BUFFER	64
#define DETECT_ADM 1


struct egis_ioc_transfer {
	u8 *tx_buf;
	u8 *rx_buf;
	u32 len;
	u32 speed_hz;
	u16 delay_usecs;
	u8 bits_per_word;
	u8 cs_change;
	u8 opcode;
	u8 sector;
	u32 umax;
	u32 umin;
	u8 flag;
	u8 pad[3];
};

/*
 *	If platform is 32bit and kernel is 64bit
 *	We will alloc egis_ioc_transfer for 64bit and 32bit
 *	We use ioc_32(32bit) to get data from user mode.
 *	Then copy the ioc_32 to ioc(64bit).
 */
#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
struct egis_ioc_transfer_32 {
	u32 tx_buf;
	u32 rx_buf;
	u32 len;
	u32 speed_hz;
	u16 delay_usecs;
	u8 bits_per_word;
	u8 cs_change;
	u8 opcode;
	u8 sector;
	u32 umax;
	u32 umin;
	u8 flag;
	u8 pad[3];
};
#endif

#define EGIS_IOC_MAGIC			'k'
#define EGIS_MSGSIZE(N) \
	((((N)*(sizeof(struct egis_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof(struct egis_ioc_transfer))) : 0)
#define EGIS_IOC_MESSAGE(N) _IOW(EGIS_IOC_MAGIC, 0, char[EGIS_MSGSIZE(N)])

struct etspi_data {
	dev_t devt;
	spinlock_t spi_lock;
#if defined(USE_SPI_BUS)
	struct spi_device *spi;
#elif defined(USE_PLATFORM_BUS)
	struct platform_device *spi;
#endif
	struct list_head device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned int users;
	u8 *buf;/* tx buffer for sensor register read/write */
	unsigned int bufsiz; /* MAX size of tx and rx buffer */
	unsigned int enable_gpio; /* 3.3V power */ 
	unsigned int irq_gpio;	/* IRQ GPIO pin number */
	unsigned int rst_gpio;	/* RESET_N */
	unsigned int vdd_dig_gpio;	/* 1.8V power */
	struct pinctrl *p;
	struct pinctrl_state *pins_poweron;
	struct pinctrl_state *pins_poweroff;
	unsigned int ldocontrol;


	unsigned int drdy_irq_flag;	/* irq flag */
	bool ldo_onoff;

	/* For polling interrupt */
	int int_count;
	struct timer_list timer;
	struct work_struct work_debug;
	struct workqueue_struct *wq_dbg;
	struct timer_list dbg_timer;
	struct device *fp_device;

	int detect_period;
	int detect_threshold;
	bool finger_on;
	const char *chipid;
};

#endif

