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
#include "inc/imx307_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

static void sensor_common_data_init(sensor_info_t *sensor_info,
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
static int sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	char init_d[3];
	int fps, Hmax = 2252;
	float time_ofline;

	fps = sensor_info->fps;
        turning_data->sensor_data.VMAX = 1125;
        turning_data->sensor_data.HMAX = 2200; 
 
        turning_data->sensor_data.active_height = 1080;
        turning_data->sensor_data.active_width = 1920;       

	turning_data->sensor_data.gain_max = 160;
	turning_data->sensor_data.analog_gain_max = 255*8192;
	turning_data->sensor_data.digital_gain_max = 0*8192;
	turning_data->sensor_data.exposure_time_min = 1;
        //turning_data->sensor_data.conversion = 1;
	if (sensor_info->sensor_mode == NORMAL_M) {
		turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.VMAX - 2;
		// turning_data->sensor_data.exposure_time_max = 2250;
	} else if (sensor_info->sensor_mode == DOL2_M) {
		turning_data->sensor_data.exposure_time_max = 11;
	}
	// time_ofline = 2200.0*Hmax/4400/74.25MHZ; 30帧是2200 25帧是2640，结果算出来一样
	// DOL2的文档 P10 Linear 的文档 P49
	if (sensor_info->sensor_mode == NORMAL_M) {
		time_ofline = 148500000.0/fps/Hmax;
		turning_data->sensor_data.exposure_time_max = time_ofline;  // 一帧总行数，包括blanking
	} else if (sensor_info->sensor_mode == DOL2_M) {
		time_ofline = 148500000.0/fps/Hmax;
		turning_data->sensor_data.exposure_time_long_max = time_ofline;
	}
	pr_info("exposure_time_max %d\n", turning_data->sensor_data.exposure_time_max);
	pr_info("exposure_time_long_max %d\n", turning_data->sensor_data.exposure_time_long_max);
	pr_info("Hmax %d, Vmax %d\n", turning_data->sensor_data.HMAX, turning_data->sensor_data.VMAX);
	turning_data->sensor_data.fps = sensor_info->fps;

		turning_data->sensor_data.lane = 2;
		turning_data->sensor_data.clk = 37125000;
	turning_data->sensor_data.lines_per_second =  28125;// 74250000.0 / 1097;   //  一行的时间
	pr_info("lines_per_second %d \n", turning_data->sensor_data.lines_per_second);

	pr_info("active_height %d active_width %d\n", turning_data->sensor_data.active_width, turning_data->sensor_data.active_height);
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.step_gain = 3;   // step_gain
	return ret;
}

static int sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(imx307_stream_on_setting)) {
		memcpy(stream_on, imx307_stream_on_setting, sizeof(imx307_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(imx307_stream_on_setting)) {
		memcpy(stream_off, imx307_stream_off_setting, sizeof(imx307_stream_off_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}
static int sensor_normal_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	// if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	// }
	
	turning_data.normal.param_hold = IMX307_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX307_SHS1;
	turning_data.normal.s_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT

	pr_info("sensor_info VMAX %d, HMAX %d \n", turning_data.sensor_data.VMAX, turning_data.sensor_data.HMAX);
        // Sets the shutter sweep time.
	turning_data.normal.line_p.ratio = -256;
	turning_data.normal.line_p.offset = turning_data.sensor_data.VMAX - 2;
	turning_data.normal.line_p.max = turning_data.sensor_data.VMAX - 4;

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = IMX307_GAIN;
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx307_gain_lut,
			sizeof(imx307_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx307_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}

	return ret;
}

static int sensor_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	// if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	//} 

	turning_data.dol2.param_hold = IMX307_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;
	turning_data.dol2.s_line = IMX307_SHS1;   //  SEF1
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX307_SHS2;   //  LEF
	turning_data.dol2.m_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.dol2.line_p[0].ratio = -256;
	turning_data.dol2.line_p[0].offset = turning_data.sensor_data.RHS1 - 1;
	turning_data.dol2.line_p[0].max = turning_data.sensor_data.RHS1 - 2;
	turning_data.dol2.line_p[1].ratio = -256;
	turning_data.dol2.line_p[1].offset = turning_data.sensor_data.FSC_DOL2 - 1;
	turning_data.dol2.line_p[1].max = turning_data.sensor_data.FSC_DOL2 - 2;

	turning_data.dol2.again_control_num = 2;
	turning_data.dol2.again_control[0] = IMX307_GAIN;     //   LEF  SHS2
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_control[0] = IMX307_GAIN1;    //   SEF1  SHS1
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx307_gain_lut,
			sizeof(imx307_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx307_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}

	return ret;
}
static int sensor_dol2_long_frame_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX307_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX307_SHS2;   //  LEF
	turning_data.normal.s_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = -256;
	turning_data.normal.line_p.offset = turning_data.sensor_data.VMAX - 1;
	turning_data.normal.line_p.max = turning_data.sensor_data.VMAX - 2;

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = IMX307_GAIN;   //  LEF SHS2
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx307_gain_lut,
			sizeof(imx307_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx307_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	return ret;
}
static int sensor_dol2_short_frame_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX307_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX307_SHS1;   //  SEF1
	turning_data.normal.s_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = -256;
	turning_data.normal.line_p.offset = turning_data.sensor_data.VMAX - 1;
	turning_data.normal.line_p.max = turning_data.sensor_data.VMAX - 2;

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = IMX307_GAIN1; // SEF1 GAIN1
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx307_gain_lut,
			sizeof(imx307_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx307_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	return ret;
}

static int sensor_new_config_func(sensor_info_t *sensor_info, uint32_t *imx307_config, int size)
{
	int ret = RET_OK;
	int setting_size, i;
	char temp_data;

	setting_size = size/sizeof(uint32_t)/2;
               pr_info("imx307 setting_size %d\n", setting_size);
                for(i = 0; i < setting_size; i++) {
                        ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
                                                                        sensor_info->sensor_addr,
                                                                        imx307_config[i*2],
                                                                        imx307_config[i*2 + 1]);
                        if (ret < 0) {
                                pr_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
                                        sensor_info->sensor_name, sensor_info->bus_num,
                                        sensor_info->sensor_addr, i,
                                        imx307_config[i*2], imx307_config[i*2 + 1]);
                                return ret;
                        }
                }

	return ret;
}

int sensor_imx307_normal_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	uint32_t setting_size = 0;
	int fps = sensor_info->fps;
	int resolution = sensor_info->resolution;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			pr_err("sensor_%d open fail\n", sensor_info->dev_port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n", sensor_info->dev_port, sensor_info->sen_devfd);

	switch (fps) {
		case IMX307_25FPS:
			if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx307_720p_2line_linear_25fps_setting,
					sizeof(imx307_720p_2line_linear_25fps_setting));
			}else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx307_1080p_2line_linear_25fps_setting,
					sizeof(imx307_1080p_2line_linear_25fps_setting));
			}else {
				pr_err("not support 25fps resolution= %d\n",resolution);
			}
			break;
		case IMX307_30FPS:
			if(resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx307_1080p_2lane_linear_30fps_setting,
				 	sizeof(imx307_1080p_2lane_linear_30fps_setting));
				// ret = sensor_new_config_func(sensor_info, imx307_1080p_2lane_linear_30fps_setting_sony,
				// 	sizeof( imx307_1080p_2lane_linear_30fps_setting_sony ));
			}
			pr_info("1080P 30FPS imx307_1080p_2lane_linear_30fps_setting ...\n");
			break;
		case IMX307_60FPS:
			if(resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx307_1080p_2lane_linear_60fps_setting,
					sizeof(imx307_1080p_2lane_linear_60fps_setting));
			}
			break;
		default:
			pr_err("not support fps type %d\n", fps);
			ret = -RET_ERROR;
			break;
	}
	pr_info("imx307_linear_config %dfps_%dP success\n", fps, resolution);
	return ret;
}
int sensor_imx307_dol2_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i, req;
	int setting_size = 0;
	int fps = sensor_info->fps;
	char temp_data;
	int resolution = sensor_info->resolution;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			pr_err("sensor_%d open fail\n", sensor_info->dev_port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
		sensor_info->dev_port, sensor_info->sen_devfd);

	req = hb_vin_pre_request(sensor_info->entry_num, 0, 0);
	if (req != 0) {
		pr_info("sensor_%s has alrady inited\n", sensor_info->sensor_name);
		return RET_OK;
	}

	switch (fps) {
		case IMX307_15FPS:
			if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx307_1080p_2lane_dol2_15fps_setting,
									sizeof(imx307_1080p_2lane_dol2_15fps_setting));
			}
			pr_info("imx307 dol2 15fps init success req %d\n",req);
			break;
		case IMX307_25FPS:
			if(resolution == 2228) {
				ret = sensor_new_config_func(sensor_info, imx307_2228p_dol2_25fps_setting,
					sizeof(imx307_2228p_dol2_25fps_setting));
			} else if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx307_720p_dol2_25fps_setting,
									sizeof(imx307_720p_dol2_25fps_setting));
			} else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx307_1080p_dol2_25fps_setting,
									sizeof(imx307_1080p_dol2_25fps_setting));
			}
			pr_info("imx307 dol2 25fps init success req %d\n",req);
			break;
		case IMX307_30FPS:
			if(resolution == 2228) {
				ret = sensor_new_config_func(sensor_info, imx307_2228p_dol2_30fps_setting,
					sizeof(imx307_2228p_dol2_30fps_setting));
			} else if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx307_720p_dol2_30fps_setting,
					sizeof(imx307_720p_dol2_30fps_setting));
			} else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx307_1080p_dol2_30fps_setting,
									sizeof(imx307_1080p_dol2_30fps_setting));
			}
			pr_info("imx307 dol2 30fps init success req %d\n", req);
			break;
		default:
			pr_err("not support fps type %d\n", fps);
			ret = -RET_ERROR;
			break;
	}
	hb_vin_pre_result(sensor_info->entry_num, 0, ret);
	if (ret < 0) {
		pr_err("imx307 dol2 init fail\n");
	}
	return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:    //  normal
			ret = sensor_imx307_normal_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_imx307_normal_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			ret = sensor_normal_data_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			ret = sensor_imx307_dol2_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_imx307_dol2_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			pr_info("imx307 dol2 15fps sensor_imx307_dol2_init \n");
			if(sensor_info->extra_mode == 0) {    //   dol2
			        pr_info("imx307 dol2 15fps sensor_dol2_data_init extra_mode \n");
				ret = sensor_dol2_data_init(sensor_info);
				if(ret < 0) {
					pr_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == 1) {   //  Long frame
				ret = sensor_dol2_long_frame_data_init(sensor_info);
				if(ret < 0) {
					pr_err("sensor_dol2_long_frame_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == 2) {    //  short
				ret = sensor_dol2_short_frame_data_init(sensor_info);
				if(ret < 0) {
					pr_err("sensor_dol2_long_frame_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			}
			break;
		default:
			pr_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	pr_info("imx307 config success under %d mode\n\n", sensor_info->sensor_mode);
	return ret;
}
static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;
	char temp_data;
	setting_size = sizeof(imx307_stream_off_setting)/sizeof(uint32_t)/2;
        pr_info("%s sensor_stop setting_size %d\n",
                sensor_info->sensor_name, setting_size);
        for(i = 0; i < setting_size; i++) {
                ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
                        sensor_info->sensor_addr, imx307_stream_off_setting[i*2],
                        imx307_stream_off_setting[i*2 + 1]);
                if (ret < 0) {
                        pr_err("%d : stop %s fail\n", __LINE__, sensor_info->sensor_name);
                        return ret;
                }
        }

	return ret;
}
static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;
	char temp_data;

	// sensor_stop(sensor_info);
	setting_size = sizeof(imx307_stream_on_setting)/sizeof(uint32_t)/2;

        for(i = 0; i < setting_size; i++) {
                ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
                        sensor_info->sensor_addr, imx307_stream_on_setting[i*2],
                        imx307_stream_on_setting[i*2 + 1]);
                if (ret < 0) {
                        pr_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);
                        return ret;
                }
        }
	return ret;
}
static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

sensor_module_t imx307 = {
	.module = "imx307",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
};

