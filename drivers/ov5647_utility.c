/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics.
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
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include "inc/hb_cam_utility.h"
#include "inc/hb_i2c.h"
#include "inc/ov5647_setting.h"
#include "inc/sensor_effect_common.h"
#define OV5647_EXP_REG_ADDR (0x3500)
#define OV5647_GAIN_ADDR_HI (0x350A)
#define OV5647_GAIN_ADDR_LO (0x350B)
#define MCLK (24000000)
static int power_ref;
int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if (power_ref > 0)
	{
		power_ref++;
		return ret;
	}
	if (sensor_info->power_mode)
	{
		/*extra_mode: xj3som-xj3board:0  j3som-x3board:1*/
		ret = hb_cam_set_mclk(sensor_info->extra_mode, MCLK);
		if (ret < 0)
		{
			pr_err("%d : set clock %s fail\n",
				   __LINE__, sensor_info->sensor_name);
			return ret;
		}
		ret = hb_cam_enable_mclk(sensor_info->extra_mode);
		if (ret < 0)
		{
			pr_err("%d : enable clock %s fail\n",
				   __LINE__, sensor_info->sensor_name);
			return ret;
		}

		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] >= 0)
			{
				ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay * 1000);
				ret |= camera_power_ctrl(sensor_info->gpio_pin[gpio],
										 1 - sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					pr_err("camera_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(5 * 1000);
			}
		}
	}
	power_ref++;
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0)
	{
		pr_err("%d : sensor reset %s fail\n",
			   __LINE__, sensor_info->sensor_name);
		return ret;
	}

	if (sensor_info->resolution == 1080)
	{
		setting_size =
			sizeof(ov5647_2lane_1080_init_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, ov5647_2lane_1080_init_setting);

		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}

		ret = ov5647_linear_data_init(sensor_info);
		if (ret < 0)
		{
			pr_debug("%d : turning data init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	else if (sensor_info->resolution == 1944)
	{
		setting_size =
			sizeof(ov5647_2lane_1944p_init_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, ov5647_2lane_1944p_init_setting);
		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	else
	{
		pr_err("config mode is err\n");
		return -RET_ERROR;
	}
	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	// if (sensor_info->config_index == 0 || sensor_info->config_index == 2)
	if (1)
	{
		setting_size =
			sizeof(ov5647_2lane_stream_on_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, ov5647_2lane_stream_on_setting);
		if (ret < 0)
		{
			pr_debug("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
	else
	{
		pr_err("config mode is err\n");
		return -RET_ERROR;
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	setting_size =
		sizeof(ov5647_2lane_stream_off_setting) / sizeof(uint32_t) / 2;
	printf("sensor_name %s, setting_size = %d\n",
		   sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, ov5647_2lane_stream_off_setting);
	if (ret < 0)
	{
		pr_debug("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	if (sensor_info->power_mode)
	{
		power_ref--;
		if (power_ref == 0)
		{
			ret = hb_cam_disable_mclk(sensor_info->extra_mode);
			if (ret < 0)
			{
				pr_err("%d : disable clock %s fail\n",
					   __LINE__, sensor_info->sensor_name);
			}
		}
	}
	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0)
	{
		pr_err("%d : deinit %s fail\n",
			   __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}
#define MAX_EXPO 1100
// turning data init
int ov5647_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
			sizeof(turning_data.sensor_name));
	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;
	// turning sensor_data
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 33120;	   // TBC
	turning_data.sensor_data.exposure_time_max = MAX_EXPO; // TBC
	turning_data.sensor_data.gain_max = 128 * 8192;		   // TBC
	turning_data.sensor_data.analog_gain_max = 128 * 8192;  // TBC
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;				// TBC
	turning_data.sensor_data.exposure_time_long_max = MAX_EXPO; // TBC

	turning_data.normal.s_line_length = 0;
	// aGain
	turning_data.normal.again_control_num = 0;

	// dGain ,ov5647 don't have dgain register
	turning_data.normal.dgain_control_num = 0;

	// setting stream ctrl
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(ov5647_2lane_stream_on_setting))
	{
		memcpy(stream_on, ov5647_2lane_stream_on_setting, sizeof(ov5647_2lane_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(ov5647_2lane_stream_off_setting))
	{
		memcpy(stream_off, ov5647_2lane_stream_off_setting, sizeof(ov5647_2lane_stream_off_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	// look-up table ----
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL)
	{
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, ov5647_gain_lut,
			   sizeof(ov5647_gain_lut));
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut)
	{
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (ret < 0)
	{
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}
static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	// *enable = 0;
	return 0;
}


static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
	int bus = info->bus_num;
	int sensor_addr = info->sensor_addr;
	char temp = 0, temp1 = 0, temp2 = 0;
	uint32_t sline = line[0];
	const uint32_t max_lines = MAX_EXPO;
	if (sline > max_lines)
	{
		sline = max_lines;
	}
	if (sline < 1)
	{
		sline = 1;
	}
	temp1 = (sline << 4) & 0xff;
	camera_i2c_write8(bus, 16, sensor_addr, 0x3502, temp1);
	temp2 = ((sline >> 4) & 0xff);
	camera_i2c_write8(bus, 16, sensor_addr, 0x3501, temp2);
	temp2 = (sline >> 12) & 0xff;
	camera_i2c_write8(bus, 16, sensor_addr, 0x3500, temp2);
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
	int bus = info->bus_num;
	int sensor_addr = info->sensor_addr;
	char hi = 0, lo = 0;
	uint32_t Again = again[0];
	if (Again > 255)
		return -1;
	int ret = 0;
	hi = (ov5647_gain_lut[Again] >> 8) & 0x03;
	ret = camera_i2c_write8(bus, 16, sensor_addr, OV5647_GAIN_ADDR_HI, hi);
	if (ret != 0)
	{
		printf("error while writing OV5647_GAIN_ADDR_HI!\n");
	}
	lo = (ov5647_gain_lut[Again] & 0xff);
	ret = camera_i2c_write8(bus, 16, sensor_addr, OV5647_GAIN_ADDR_LO, lo);
	if (ret != 0)
	{
		printf("error while writing OV5647_GAIN_ADDR_LO!\n");
	}
	return 0;
}

sensor_module_t ov5647 = {
	.module = "ov5647",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.aexp_line_control = sensor_aexp_line_control,
	.aexp_gain_control = sensor_aexp_gain_control,
	.userspace_control = sensor_userspace_control,
};
