/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/ioctl.h>
#include "cam/hb_vin.h"

#include "inc/hb_i2c.h"
#include "inc/hb_cam_utility.h"

static int sensor_func(sensor_info_t *sensor_info, const char *func)
{
	pr_info("port%d: %s -- %s --\n",
		sensor_info->port, sensor_info->sensor_name, func);
	return RET_OK;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i, req;
	int setting_size = 0;
	char mipi_cfg[128], *gpio;

	sensor_func(sensor_info, __func__);
	pr_info("%s: %dx%d@%dfps 0x%02x\n",
		sensor_info->sensor_name, sensor_info->height,
		sensor_info->width, sensor_info->fps,
		sensor_info->format);
	snprintf(mipi_cfg, sizeof(mipi_cfg), sensor_info->config_path,
		sensor_info->fps, sensor_info->resolution);
	pr_info("mipi%d: %s\n", sensor_info->entry_num, mipi_cfg);
	if (sensor_info->bus_type) {
		pr_info("spi%d: cs%d mode %d speed %d\n",
			sensor_info->bus_num, sensor_info->spi_info.spi_cs,
			sensor_info->spi_info.spi_mode, sensor_info->spi_info.spi_speed);
	} else {
		pr_info("i2c%d: 0x%02x 0x%02x width %d\n",
			sensor_info->bus_num, sensor_info->sensor_addr,
			sensor_info->sensor1_addr, sensor_info->reg_width);
	}
	if (sensor_info->serial_addr || sensor_info->serial_addr1)
		pr_info("serial: 0x%02x 0x%02x\n", sensor_info->serial_addr,
			sensor_info->serial_addr1);

	/* Set sensor registers */

	pr_info("isp: 0x%02x, imu: 0x%02x, eep: 0x%02x\n",
		sensor_info->isp_addr, sensor_info->imu_addr,
		sensor_info->eeprom_addr);
	pr_info("sensor_mode: %d, extra_mode: %d, config_index: %d\n",
		sensor_info->sensor_mode, sensor_info->extra_mode,
		sensor_info->config_index);
	pr_info("power_mode: %d, power_delay: %d\n",
		sensor_info->power_mode, sensor_info->power_delay);
	if (sensor_info->gpio_num) {
		gpio = mipi_cfg;
		for (i = 0; i < sensor_info->gpio_num; i++)
			gpio += sprintf(gpio, "%d-%d, ", sensor_info->gpio_pin[i],
					sensor_info->gpio_level[i]);
		pr_info("gpio %d: %s\n", sensor_info->gpio_num, gpio);
	}
	if (sensor_info->power_delay && !sensor_info->power_mode) {
		pr_info("sleep %dms\n", sensor_info->power_delay);
		usleep(sensor_info->power_delay * 1000);
	}

	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK, req;
	int setting_size = 0;
	uint32_t *setting_array;

	sensor_func(sensor_info, __func__);

	/* Set sensor register, Start stream */

	if (sensor_info->power_delay && sensor_info->power_mode) {
		pr_info("sleep %dms\n", sensor_info->power_delay);
		usleep(sensor_info->power_delay * 1000);
	}

	return RET_OK;
}

static int sensor_power_on(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_power_off(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_power_reset(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_extern_isp_poweron(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_extern_isp_poweroff(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_extern_isp_reset(sensor_info_t *sensor_info)
{
	return sensor_func(sensor_info, __func__);
}

static int sensor_spi_read(sensor_info_t *sensor_info,  uint32_t reg_addr, char *buffer, uint32_t sizee)
{
	sensor_func(sensor_info, __func__);
	pr_info("reg_addr: 0x%02x, buffser: %d, sizee: %d\n",
			reg_addr, *buffer, sizee);
	return RET_OK;
}

static int sensor_spi_write(sensor_info_t *sensor_info,  uint32_t reg_addr, char *buffer, uint32_t sizee)
{
	sensor_func(sensor_info, __func__);
	pr_info("reg_addr: 0x%02x, buffser: %d, sizee: %d\n",
			reg_addr, *buffer, sizee);
	return RET_OK;
}

static int sensor_set_awb(int i2c_bus, int sensor_addr, float rg_gain, float b_gain)
{
	pr_info("dummy -- %s --", __func__);
	pr_info("i2c_bus: %d, sensor_addr: 0x%02x, rg_gain: %.2f, b_gain: %.2f\n",
			i2c_bus, sensor_addr, rg_gain, b_gain);
	return RET_OK;

}

static int sensor_set_ex_gain( int i2c_bus, int sensor_addr, uint32_t exposure_setting,
			uint32_t gain_setting_0, uint16_t gain_setting_1)
{
	pr_info("dummy -- %s --", __func__);
	pr_info("i2c_bus: %d, sensor_addr: 0x%02x, exposure_setting: %d, gain_setting_0: %d, gain_setting_1: %d\n",
			i2c_bus, sensor_addr, exposure_setting, gain_setting_0, gain_setting_1);
	return RET_OK;
}


#define SIF_IOC_MAGIC 'x'
#define SIF_IOC_PATTERN_CFG  _IOW(SIF_IOC_MAGIC, 10, int)

struct sif_pattern_cfg {
	uint32_t instance;
	uint32_t framerate;
};

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	sensor_func(sensor_info, __func__);
	pr_info("sensor_dynamic_switch_fps fps: %d\n", fps);
	int fd_main, ret = RET_OK;
	struct sif_pattern_cfg pattern_fps;

	pattern_fps.framerate = fps;
	pattern_fps.instance = sensor_info->extra_mode;
	fd_main = open("/dev/sif_capture", O_RDWR | O_CLOEXEC, 0666);
	if (fd_main < 0) {
		pr_err("sif main node open failed.\n");
		return -RET_ERROR;
	}

	ret =  ioctl(fd_main, SIF_IOC_PATTERN_CFG, &pattern_fps);
	if (ret < 0) {
		pr_err("SIF_IOC_PATTERN_CFG ioctl failed !\n");
		if (fd_main >= 0) {
			close(fd_main);
			fd_main = -1;
		}
		return ret;
	}
	close(fd_main);
	fd_main = -1;
	pr_info("sensor_dynamic_switch_fps end\n");
	return RET_OK;
}

sensor_module_t dummy = {
	.module = "dummy",
	.init = sensor_init,
	.deinit = sensor_deinit,
	.start = sensor_start,
	.stop = sensor_stop,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
	.power_reset = sensor_power_reset,
	.extern_isp_poweron = sensor_extern_isp_poweron,
	.extern_isp_poweroff = sensor_extern_isp_poweroff,
	.extern_isp_reset = sensor_extern_isp_reset,
	.spi_read = sensor_spi_read,
	.spi_write = sensor_spi_write,
	.set_awb = sensor_set_awb,
	.set_ex_gain = sensor_set_ex_gain,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};

