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
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

//#include "inc/logging.h"
#include "logging.h"
#include "inc/hb_cam_utility.h"
#include "inc/f37_setting.h"
#include "inc/sensor_effect_common.h"

#define F37_PROGRAM_GAIN	(0x00)
#define F37_EXP_LINE		(0x01)
#define F37_DOL2_SHORT_EXP_LINE		(0x05)


int f37_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
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

	// turning sensor_data
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 33783;

	turning_data.sensor_data.exposure_time_max = 1088;

	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;
	turning_data.sensor_data.gain_max = 128 * 8192;
	turning_data.sensor_data.analog_gain_max = 255*8192;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 4000;
	//turning_data->sensor_data.conversion = 1;

	// turning normal
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 1088;

	turning_data.normal.s_line = F37_EXP_LINE;
	turning_data.normal.s_line_length = 2;
	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = F37_PROGRAM_GAIN;
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control_length[0] = 0;
	turning_data.normal.dgain_control[0] = 0;

	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(f37_linear_stream_on_setting)) {
		memcpy(stream_on, f37_linear_stream_on_setting, sizeof(f37_linear_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(f37_stream_off_setting)) {
		memcpy(stream_off, f37_stream_off_setting, sizeof(f37_stream_off_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, f37_gain_lut,
			sizeof(f37_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(f37_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int f37_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
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

	// turning sensor_data
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 33333;
	turning_data.sensor_data.exposure_time_max = 66;

	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;
	turning_data.sensor_data.gain_max = 128 * 8192;
	turning_data.sensor_data.analog_gain_max = 255*8192;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 2176;
	//turning_data->sensor_data.conversion = 1;

	// turning normal
	// short frame
	turning_data.dol2.s_line = F37_DOL2_SHORT_EXP_LINE;
	turning_data.dol2.s_line_length = 1;
	// long frame
	turning_data.dol2.m_line = F37_EXP_LINE; //0x05
	turning_data.dol2.m_line_length = 2;

	turning_data.dol2.line_p[0].ratio = 1 << 8;
	turning_data.dol2.line_p[0].offset = 0;
	turning_data.dol2.line_p[0].max = 66;
	turning_data.dol2.line_p[1].ratio = 1 << 8;
	turning_data.dol2.line_p[1].offset = 0;
	turning_data.dol2.line_p[1].max = 2176;

	turning_data.dol2.again_control_num = 1;
	turning_data.dol2.again_control[0] = F37_PROGRAM_GAIN;
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.dgain_control_num = 0;
	turning_data.dol2.dgain_control_length[0] = 0;
	turning_data.dol2.dgain_control[0] = 0;

	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(f37_hdr_stream_on_setting)) {
		memcpy(stream_on, f37_hdr_stream_on_setting, sizeof(f37_hdr_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(f37_stream_off_setting)) {
		memcpy(stream_off, f37_stream_off_setting, sizeof(f37_stream_off_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, f37_gain_lut,
			sizeof(f37_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(f37_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}


int sensor_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:	  //  normal
			pr_info("in normal mode\n");
			setting_size = sizeof(f37_linear_init_setting)/sizeof(uint32_t)/2;
			pr_debug("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 1,
						setting_size, f37_linear_init_setting);
			if (ret < 0) {
				pr_debug("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = f37_linear_data_init(sensor_info);
			if (ret < 0) {
				pr_debug("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			pr_info("in dol2 mode\n");
			setting_size = sizeof(f37_HDR_init_setting)/sizeof(uint32_t)/2;
			pr_debug("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 1,
						setting_size, f37_HDR_init_setting);
			if (ret < 0) {
				pr_debug("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = f37_dol2_data_init(sensor_info);
			if (ret < 0) {
				pr_debug("%d : dol2 data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			pr_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	pr_info("f37 config success under %d mode\n\n", sensor_info->sensor_mode);
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			setting_size = sizeof(f37_linear_stream_on_setting)/sizeof(uint32_t)/2;
			printf(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 1,
						setting_size, f37_linear_stream_on_setting);
			if(ret < 0) {
				pr_debug("start %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:
			setting_size = sizeof(f37_hdr_stream_on_setting)/sizeof(uint32_t)/2;
			printf("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 1,
						setting_size, f37_hdr_stream_on_setting);
			if(ret < 0) {
				pr_debug("start %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(f37_stream_off_setting)/sizeof(uint32_t)/2;
	printf(" stop sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
	ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 1,
		setting_size, f37_stream_off_setting);
	if(ret < 0) {
		pr_debug("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

sensor_module_t f37 = {
	.module = "f37",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
