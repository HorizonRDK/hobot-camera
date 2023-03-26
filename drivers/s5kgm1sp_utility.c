/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "cam/hb_vin.h"
#include "inc/hb_cam_utility.h"
#include "inc/hb_i2c.h"
#include "inc/s5kgm1sp_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

int sensor_reset(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] >= 0) {
				pr_debug("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,
                        sensor_info->gpio_pin[gpio],
                        sensor_info->gpio_level[gpio],
                        sensor_info->gpio_level[gpio]);
				ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= camera_power_ctrl(sensor_info->gpio_pin[gpio],
                                        1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					pr_err("camera_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	return ret;
}

void sensor_common_data_init(sensor_info_t *sensor_info,
		sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
		sizeof(turning_data->sensor_name));
	return;
}
int sensor_12M_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;

	turning_data->sensor_data.active_width = 4000;
	turning_data->sensor_data.active_height = 3000;
	turning_data->sensor_data.gain_max = 127 * 8192;
	turning_data->sensor_data.analog_gain_max = 127 * 8192;
	turning_data->sensor_data.exposure_time_min = 2;
	turning_data->sensor_data.exposure_time_max = 0xfffc;
	turning_data->sensor_data.exposure_time_long_max = 0xfffc;
	turning_data->sensor_data.lines_per_second = 95939;   // 482M/5024
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.fps = sensor_info->fps;  // fps
	turning_data->sensor_data.conversion = 1;
	return ret;
}

int sensor_4K_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;

	turning_data->sensor_data.active_width = 3840;
	turning_data->sensor_data.active_height = 2160;
	turning_data->sensor_data.gain_max = 127 * 8192;
	turning_data->sensor_data.analog_gain_max = 127 * 8192;
	turning_data->sensor_data.exposure_time_min = 2;
	turning_data->sensor_data.exposure_time_max = 0xfffc;
	turning_data->sensor_data.exposure_time_long_max = 0xfffc;
	turning_data->sensor_data.lines_per_second = 74938;	 // 482M/6432
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.fps = sensor_info->fps;  // fps
	turning_data->sensor_data.conversion = 1;
	return ret;
}

int sensor_2m_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;

	turning_data->sensor_data.active_width = 1920;
	turning_data->sensor_data.active_height = 1080;
	turning_data->sensor_data.gain_max = 127 * 8192;
	turning_data->sensor_data.analog_gain_max = 127 * 8192;
	turning_data->sensor_data.exposure_time_min = 2;
	turning_data->sensor_data.exposure_time_max = 0xfffc;
	turning_data->sensor_data.exposure_time_long_max = 0xfffc;
	turning_data->sensor_data.lines_per_second = 95939;	 // 482M/5024
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.fps = sensor_info->fps;  // fps
	turning_data->sensor_data.conversion = 1;
	return ret;
}


static int sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	int size;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	size = sizeof(turning_data->stream_ctrl.stream_on);
	if(size >= sizeof(s5kgm1sp_stream_on_setting)) {
		memcpy(stream_on, s5kgm1sp_stream_on_setting,
				sizeof(s5kgm1sp_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	size = sizeof(turning_data->stream_ctrl.stream_off);
	if(size >= sizeof(s5kgm1sp_stream_off_setting)) {
		memcpy(stream_off, s5kgm1sp_stream_off_setting,
				sizeof(s5kgm1sp_stream_off_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}
int sensor_turning_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->extra_mode == 0) {
		sensor_12M_param_init(sensor_info, &turning_data);
	} else if (sensor_info->extra_mode == 1) {
		sensor_4K_param_init(sensor_info, &turning_data);
	} else if (sensor_info->extra_mode == 2) {
		sensor_2m_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = S5KGM1SP_PARAM_HOLD;
	turning_data.normal.param_hold_length = 2;
	turning_data.normal.s_line = S5KGM1SP_LINE;
	turning_data.normal.s_line_length = 2;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 0xfffc;

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = S5KGM1SP_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.again_lut = malloc(256*1*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*1*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, s5kgm1sp_again_lut,
			sizeof(s5kgm1sp_again_lut));
		for (open_cnt =0; open_cnt <
			sizeof(s5kgm1sp_again_lut)/sizeof(uint32_t); open_cnt++) {
			printf("num %d, data %x", open_cnt,
			        turning_data.normal.again_lut[open_cnt]);
			DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}

	return ret;
}

int sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	/*linear mode*/
	if(sensor_info->extra_mode == 0) {
		setting_size = sizeof(s5kgm1sp_init_setting)/sizeof(uint16_t)/2;
		pr_info("x3 setting_size %d\n", setting_size);
		for(i = 0; i < setting_size; i++) {
			ret = hb_i2c_write_reg16_data16(sensor_info->bus_num,
											sensor_info->sensor_addr,
											s5kgm1sp_init_setting[i*2],
											s5kgm1sp_init_setting[i*2 + 1]);
			if (ret < 0) {
				pr_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
					sensor_info->sensor_name, sensor_info->bus_num,
					sensor_info->sensor_addr, i,
					s5kgm1sp_init_setting[i*2], s5kgm1sp_init_setting[i*2 + 1]);
				return ret;
			}
			if(i == 3)
				usleep(3*1000);
		}
		pr_info("S5KGM1SP_12M_config OK!\n");
	} else if (sensor_info->extra_mode == 1) {
		setting_size = sizeof(s5kgm1sp_4K_init_setting)/sizeof(uint16_t)/2;
		pr_info("x3 setting_size %d\n", setting_size);
		for(i = 0; i < setting_size; i++) {
			ret = hb_i2c_write_reg16_data16(sensor_info->bus_num,
											sensor_info->sensor_addr,
											s5kgm1sp_4K_init_setting[i*2],
											s5kgm1sp_4K_init_setting[i*2 + 1]);
			if (ret < 0) {
				pr_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
					sensor_info->sensor_name, sensor_info->bus_num,
					sensor_info->sensor_addr, i,
					s5kgm1sp_4K_init_setting[i*2], s5kgm1sp_4K_init_setting[i*2 + 1]);
				return ret;
			}
			if(i == 3)
				usleep(3*1000);
		}
		pr_info("S5KGM1SP_4K_config OK!\n");
	} else if (sensor_info->extra_mode == 2) {
		setting_size = sizeof(s5kgm1sp_2m_init_setting)/sizeof(uint16_t)/2;
		pr_info("x3 setting_size %d\n", setting_size);
		for(i = 0; i < setting_size; i++) {
			ret = hb_i2c_write_reg16_data16(sensor_info->bus_num,
											sensor_info->sensor_addr,
											s5kgm1sp_2m_init_setting[i*2],
											s5kgm1sp_2m_init_setting[i*2 + 1]);
			if (ret < 0) {
				pr_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
					sensor_info->sensor_name, sensor_info->bus_num,
					sensor_info->sensor_addr, i,
					s5kgm1sp_2m_init_setting[i*2], s5kgm1sp_2m_init_setting[i*2 + 1]);
				return ret;
			}
			if(i == 3)
				usleep(3*1000);
		}
		pr_info("S5KGM1SP_2M_config OK!\n");
	} else {
		pr_err("config mode is err\n");
		return -RET_ERROR;
	}
	ret = sensor_turning_data_init(sensor_info);
	if(ret < 0) {
		pr_err("sensor_turning_data_init %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
			pr_err("port_%d open fail\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
		sensor_info->dev_port, sensor_info->sen_devfd);

	ret = sensor_reset(sensor_info);
	if (ret < 0) {
		pr_err("%d : reset %s fail\n", __LINE__, sensor_info->sensor_name);
		return -RET_ERROR;
	}
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0)
		pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	setting_size = sizeof(s5kgm1sp_stream_on_setting)/sizeof(uint32_t)/2;
	pr_info("%s sensor_start setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_i2c_write_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, s5kgm1sp_stream_on_setting[i*2],
			s5kgm1sp_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			pr_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;

	setting_size = sizeof(s5kgm1sp_stream_off_setting)/sizeof(uint32_t)/2;
	pr_info("%s sensor_stop setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_i2c_write_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, s5kgm1sp_stream_off_setting[i*2],
			s5kgm1sp_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			pr_err("%d : stop %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	}
	return ret;
}
int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int gpio;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					pr_err("camera_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
        printf(" gain mode %d, --again %d, dgain %d \n", mode, again[0], dgain[0]);
        return 0;
}
static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
        printf(" line mode %d, --line %d \n", mode, line[0]);
        return 0;
}
static int sensor_awb_control(hal_control_info_t *info, uint32_t mode, uint32_t rgain, uint32_t bgain, uint32_t grgain, uint32_t gbgain)
{
        printf(" awb ! \n");
        return 0;
}

static int sensor_af_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
        // printf(" af pos %d ! \n", pos);
	uint32_t temp = 0xc000;
	temp = temp + pos;
	hb_i2c_write_reg8_data8(2, 0x0c, (temp >> 8) & 0xff, temp & 0xff);

        return 0;
}

static int sensor_zoom_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
        printf(" zoom ! \n");
        return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
        // *enable = HAL_AF_CONTROL;
        *enable = 0;
        return 0;
}

sensor_module_t s5kgm1sp = {
	.module = "s5kgm1sp",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.aexp_gain_control = sensor_aexp_gain_control,
	.aexp_line_control = sensor_aexp_line_control,
	.awb_control = sensor_awb_control,
	.userspace_control = sensor_userspace_control,
	.af_control = sensor_af_control,
	.zoom_control = sensor_zoom_control,
};

