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
#include "inc/imx219_setting.h"
#include "inc/sensor_effect_common.h"
#define MCLK (24000000)
#define IMX219_EXP_REG_ADDR 0x015A
#define IMX219_AGAIN_REG_ADDR 0x0157
#define IMX219_DGAIN_REG_ADDR 0x0158
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
			sizeof(imx219_1080_init_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, imx219_1080_init_setting);

		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
		ret = imx219_linear_data_init(sensor_info);
		printf("jiale:imx219 turing tool\n");
		if (ret < 0)
		{
			pr_debug("%d : turning data init %s fail\n",
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
	printf("jiale:start streaming...\n");
	setting_size =
		sizeof(imx219_stream_on_setting) / sizeof(uint32_t) / 2;
	pr_debug("sensor_name %s, setting_size = %d\n",
			 sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, imx219_stream_on_setting);
	if (ret < 0)
	{
		pr_debug("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if (1)
	{
		setting_size =
			sizeof(imx219_stream_off_setting) / sizeof(uint32_t) / 2;
		printf("sensor_name %s, setting_size = %d\n",
			   sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, imx219_stream_off_setting);
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
// turning data init
int imx219_linear_data_init(sensor_info_t *sensor_info)
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
	turning_data.sensor_data.conversion = 1;
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 52230;	   // TBC
	turning_data.sensor_data.exposure_time_max = 1736;	   // TBC
	turning_data.sensor_data.gain_max = 109 * 8192;		   // TBC
	turning_data.sensor_data.analog_gain_max = 109 * 8192; // TBC
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;		// TBC
	turning_data.sensor_data.exposure_time_long_max = 1736; // TBC
	//   turning normal
	turning_data.normal.line_p.ratio = 256;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 1736;

	// s_line means exposure related registers
	turning_data.normal.s_line = IMX219_EXP_REG_ADDR;
	turning_data.normal.s_line_length = 2;
	// aGain
	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = IMX219_AGAIN_REG_ADDR;
	turning_data.normal.again_control_length[0] = 1;
	// dGain
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;

	// setting stream ctrl
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx219_stream_on_setting))
	{
		memcpy(stream_on, imx219_stream_on_setting, sizeof(imx219_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx219_stream_off_setting))
	{
		memcpy(stream_off, imx219_stream_off_setting, sizeof(imx219_stream_off_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL)
	{
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx219_gain_lut,
			   sizeof(imx219_gain_lut));
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

sensor_module_t imx219 = {
	.module = "imx219",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
