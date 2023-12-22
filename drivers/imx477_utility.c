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
#include "inc/imx477_setting.h"
#include "inc/sensor_effect_common.h"
#define IMX477_EXP_REG_ADDR_HI 0x0202
#define IMX477_FRM_LENGTH_HI 0x0340
#define IMX477_FRM_LENGTH_LO 0x0341
#define IMX477_AGAIN_REG_ADDR_HI 0x0204
#define IMX477_AGAIN_REG_ADDR_LO 0x0205
#define IMX477_DGAIN_REG_ADDR 0x020e
#define MCLK (24000000)
static int power_ref;
int imx477_linear_data_init(sensor_info_t *sensor_info);
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
	/** common setting start**/
	setting_size =
		sizeof(imx477_common_regs) / sizeof(uint32_t) / 2;
	pr_debug("sensor_name %s, common setting_size = %d\n",
			 sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, imx477_common_regs);
	if (ret < 0)
	{
		pr_debug("%d : common setting %s fail\n",
				 __LINE__, sensor_info->sensor_name);
		return ret;
	}
	/** common setting end**/

	/** quality setting start**/
	setting_size =
		sizeof(imx477_image_quality) / sizeof(uint32_t) / 2;
	pr_debug("sensor_name %s, quality setting_size = %d\n",
			 sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, imx477_image_quality);
	if (ret < 0)
	{
		pr_debug("%d : quality setting %s fail\n",
				 __LINE__, sensor_info->sensor_name);
		return ret;
	}
	/** quality setting end**/

	if (sensor_info->resolution == 1080)
	{
		setting_size =
			sizeof(imx477_1080p_50fps_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, imx477_1080p_50fps_setting);

		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	else if (sensor_info->resolution == 3000)
	{
		setting_size =
			sizeof(imx477_3000p_10fps_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, imx477_3000p_10fps_setting);

		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
		// fixme:There is currently no function to set linear
	}
	else if (sensor_info->resolution == 960)
	{
		setting_size =
			sizeof(imx477_990p_10fps_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, imx477_990p_10fps_setting);
 
		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
		// fixme:There is currently no function to set linear
	}
	else if (sensor_info->resolution == 1520)
	{
		setting_size =
			sizeof(imx477_1520p_10fps_setting) / sizeof(uint32_t) / 2;
		pr_debug("sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = camera_write_array(sensor_info->bus_num,
								 sensor_info->sensor_addr, 2,
								 setting_size, imx477_1520p_10fps_setting);

		if (ret < 0)
		{
			pr_debug("%d : init %s fail\n",
					 __LINE__, sensor_info->sensor_name);
			return ret;
		}
		// fixme:There is currently no function to set linear
	}
	else
	{
		printf("sensor_info->resolution == %d\n", sensor_info->resolution);
		pr_err("config mode is err\n");
		return -RET_ERROR;
	}

	ret = imx477_linear_data_init(sensor_info);
	if (ret < 0)
	{
		pr_debug("%d : turning data init %s fail\n",
				 __LINE__, sensor_info->sensor_name);
		return ret;
	}

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	pr_info("IMX477: start streaming...\n");
	setting_size =
		sizeof(imx477_stream_on_setting) / sizeof(uint32_t) / 2;
	pr_debug("sensor_name %s, setting_size = %d\n",
			 sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, imx477_stream_on_setting);
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
	setting_size =
		sizeof(imx477_stream_off_setting) / sizeof(uint32_t) / 2;
	pr_info("sensor_name %s, setting_size = %d\n",
		   sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, imx477_stream_off_setting);
	if (ret < 0)
	{
		pr_debug("stop %s fail\n", sensor_info->sensor_name);
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
void imx477_common_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	turning_data->sensor_data.conversion = 1;
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
			sizeof(turning_data->sensor_name));
	// s_line means exposure related registers
	turning_data->normal.s_line = IMX477_EXP_REG_ADDR_HI;
	turning_data->normal.s_line_length = 2;

	// aGain
	turning_data->normal.again_control_num = 0;
	turning_data->normal.again_control[0] = 0;
	turning_data->normal.again_control_length[0] = 0;
	// high register 2bits

	// dGain
	turning_data->normal.dgain_control_num = 0;
	turning_data->normal.dgain_control[0] = IMX477_DGAIN_REG_ADDR;
	turning_data->normal.dgain_control_length[0] = 0;
}
void imx477_param_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{

	int vts_hi = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX477_FRM_LENGTH_HI);
	int vts_lo = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX477_FRM_LENGTH_LO);
	uint32_t vts = vts_hi;
	vts = vts << 8 | vts_lo;
	pr_info("IMX477: vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi, vts_lo, vts);
	uint32_t max_expo = vts;
	turning_data->sensor_data.active_width = sensor_info->width;
	turning_data->sensor_data.active_height = sensor_info->height;
	turning_data->sensor_data.turning_type = 6;
	turning_data->sensor_data.fps = sensor_info->fps;
	turning_data->sensor_data.lines_per_second = vts * sensor_info->fps; // TBC
	turning_data->sensor_data.exposure_time_max = max_expo;				 // TBC
	turning_data->sensor_data.gain_max = 143 * 8192;					 // TBC
	turning_data->sensor_data.analog_gain_max = 143 * 8192;				 // TBC
	turning_data->sensor_data.digital_gain_max = 0;
	turning_data->sensor_data.exposure_time_min = 1;			 // TBC
	turning_data->sensor_data.exposure_time_long_max = max_expo; // TBC
	turning_data->normal.line_p.ratio = 256;
	turning_data->normal.line_p.offset = 0;
	turning_data->normal.line_p.max = max_expo;
}

// void imx477_1080p_param_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
// {
// 	uint32_t max_expo = 1313;
// 	turning_data->sensor_data.active_width = 1920;
// 	turning_data->sensor_data.active_height = 1080;
// 	turning_data->sensor_data.turning_type = 6;
// 	turning_data->sensor_data.fps = sensor_info->fps;
// 	turning_data->sensor_data.lines_per_second = 65650;		// TBC
// 	turning_data->sensor_data.exposure_time_max = max_expo; // TBC
// 	turning_data->sensor_data.gain_max = 143 * 8192;		// TBC
// 	turning_data->sensor_data.analog_gain_max = 143 * 8192; // TBC
// 	turning_data->sensor_data.digital_gain_max = 0 * 1024;
// 	turning_data->sensor_data.exposure_time_min = 8;			 // TBC
// 	turning_data->sensor_data.exposure_time_long_max = max_expo; // TBC
// 	turning_data->normal.line_p.ratio = 256;
// 	turning_data->normal.line_p.offset = 0;
// 	turning_data->normal.line_p.max = max_expo;
// }

// turning data init
int imx477_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	imx477_common_data_init(sensor_info, &turning_data);
	imx477_param_init(sensor_info, &turning_data);
	// switch (sensor_info->resolution)
	// {
	// case 1080:
	// 	imx477_1080p_param_init(sensor_info, &turning_data);
	// 	break;
	// case 3000:
	// 	imx477_12MP_param_init(sensor_info, &turning_data);
	// 	break;

	// default:
	// imx477_1080p_param_init(sensor_info, &turning_data);
	// 	//pr_err("resolution error!\n");
	// 	break;
	// }

	// setting stream ctrl
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx477_stream_on_setting))
	{
		memcpy(stream_on, imx477_stream_on_setting, sizeof(imx477_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx477_stream_off_setting))
	{
		memcpy(stream_off, imx477_stream_off_setting, sizeof(imx477_stream_off_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	// look-up table
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL)
	{
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx477_gain_lut,
			   sizeof(imx477_gain_lut));
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

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
	int bus = info->bus_num;
	int sensor_addr = info->sensor_addr;
	uint8_t hi = 0, lo = 0;
	uint32_t reg_val = imx477_gain_lut[again[0]];
	int ret = 0;
	hi = (reg_val >> 8) & 0x03;
	ret = camera_i2c_write8(bus, 16, sensor_addr, IMX477_AGAIN_REG_ADDR_HI, hi);
	if (ret != 0)
	{
		printf("error while writing IMX477_AGAIN_REG_ADDR_HI!\n");
	}
	lo = (reg_val & 0xff);
	ret = camera_i2c_write8(bus, 16, sensor_addr, IMX477_AGAIN_REG_ADDR_LO, lo);
	if (ret != 0)
	{
		printf("error while writing IMX477_AGAIN_REG_ADDR_LO!\n");
	}
	// printf("bus:%d,sensor_addr:0x%2x,reg_val:0x%2x,AGAIN_HI:0x%2x,AGAIN_LO:0x%2x\n", bus, sensor_addr, reg_val, hi, lo);
	return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	*enable = HAL_GAIN_CONTROL;
	return 0;
}

sensor_module_t imx477 = {
	.module = "imx477",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.aexp_gain_control = sensor_aexp_gain_control,
	.userspace_control = sensor_userspace_control,
};
