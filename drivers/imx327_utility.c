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
#include "inc/imx327_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

static int sensor_spi_write(sensor_info_t *sensor_info,
			uint32_t reg_addr, char *buffer, uint32_t sizee)
{
	int ret = RET_OK;

	if(reg_addr > 0xffff)
		   return -RET_ERROR;
	char *senbuf = malloc(sizee + 2);
	if(senbuf == NULL) {
		pr_err("malloc fail\n");
		return -RET_ERROR;
	}
	memset(senbuf, 0, sizee + 2);
	senbuf[0] = ((reg_addr >> 8) - 0x30) + 0x02;
	senbuf[1] = reg_addr & 0xff;
	memcpy(&senbuf[2], buffer, sizee);
	ret = camera_spi_write_block(sensor_info->bus_num, senbuf, sizee + 2);
	if(ret < 0) {
		free(senbuf);
		pr_err("malloc fail\n");
		return -RET_ERROR;
	}
	free(senbuf);
	return ret;
}

static int sensor_spi_read(sensor_info_t *sensor_info,
				uint32_t reg_addr, char *buffer, uint32_t sizee)
{
	int ret = RET_OK;

	char *revbuf = malloc(sizee + 2);
	if(revbuf == NULL) {
		pr_err("malloc fail\n");
		return -RET_ERROR;
	}
	memset(revbuf, 0, sizee + 2);
	revbuf[0] = ((reg_addr >> 8) - 0x30) + 0x82;
	revbuf[1] = reg_addr & 0xff;
	ret = camera_spi_read_block(sensor_info->bus_num, revbuf, sizee + 2);
	if(ret) {
		free(revbuf);
		pr_err("malloc fail\n");
		return -RET_ERROR;
	}
	memcpy(buffer, &revbuf[2], sizee);
	free(revbuf);
	return ret;
}

static void sensor_common_data_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->cs = sensor_info->spi_info.spi_cs;
	turning_data->spi_mode = sensor_info->spi_info.spi_mode;
	turning_data->spi_speed = sensor_info->spi_info.spi_speed;
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
	int fps, Hmax;
	float time_ofline;

	fps = sensor_info->fps;
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX327_VAMX, init_d, 3);
	turning_data->sensor_data.VMAX = init_d[2];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[0];
	turning_data->sensor_data.FSC_DOL2 = turning_data->sensor_data.VMAX * 2;
	turning_data->sensor_data.FSC_DOL3 = turning_data->sensor_data.VMAX * 4;
	turning_data->sensor_data.gain_max = 160;
	turning_data->sensor_data.analog_gain_max = 255*8192;
	turning_data->sensor_data.digital_gain_max = 0*8192;
	turning_data->sensor_data.exposure_time_min = 1;
	if (sensor_info->sensor_mode == NORMAL_M) {
		turning_data->sensor_data.exposure_time_max =
				turning_data->sensor_data.VMAX - 2;
	} else if (sensor_info->sensor_mode == DOL2_M) {
		turning_data->sensor_data.exposure_time_max = 11;
	}
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX327_HAMX, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[1];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[0];

	Hmax = turning_data->sensor_data.HMAX;
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
	turning_data->sensor_data.fps = sensor_info->fps;
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX327_RHS1, init_d, 3);
	turning_data->sensor_data.RHS1 = init_d[2];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[1];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[0];

	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX327_RHS2, init_d, 3);
	turning_data->sensor_data.RHS2 = init_d[2];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[1];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[0];
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX327_CSI_LANE_MODE, init_d, 1);
	if (init_d[0] == 3)
		turning_data->sensor_data.lane = 4;
	else
		turning_data->sensor_data.lane = 2;
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				IMX327_INCKSEL6, init_d, 1);
	if (init_d[0] == 0x1A)
		turning_data->sensor_data.clk = 37125000;
	else
		turning_data->sensor_data.clk = 74250000;
        /*1000000/2200 *(HMAX/4400)*1000000/74.25MHZ 先算一秒多少行取倒数*/
	turning_data->sensor_data.lines_per_second = 148500000/Hmax;   //  一行的时间
	pr_info("lines_per_second %d \n", turning_data->sensor_data.lines_per_second);
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				IMX327_X_SIZE, init_d, 2);
	turning_data->sensor_data.active_width = init_d[1];
	turning_data->sensor_data.active_width = (turning_data->sensor_data.active_width << 8) | init_d[0];

	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				IMX327_Y_SIZE, init_d, 2);
	turning_data->sensor_data.active_height = init_d[1];
	turning_data->sensor_data.active_height = (turning_data->sensor_data.active_height << 8) | init_d[0];
	pr_info("active_height %d active_width %d\n", turning_data->sensor_data.active_width, turning_data->sensor_data.active_height);
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.step_gain = 3;   // step_gain
	return ret;
}

static int sensor_spi_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	char init_d[3];

	ret = sensor_spi_read(sensor_info, IMX327_VAMX, init_d, 3);
	turning_data->sensor_data.VMAX = init_d[2];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[0];
	turning_data->sensor_data.FSC_DOL2 = turning_data->sensor_data.VMAX * 2;
	turning_data->sensor_data.FSC_DOL3 = turning_data->sensor_data.VMAX * 4;
	turning_data->sensor_data.gain_max = 210;
	turning_data->sensor_data.exposure_time_min = 1;
	turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.VMAX;

	ret = sensor_spi_read(sensor_info, IMX327_HAMX, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[1];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[0];
	ret = sensor_spi_read(sensor_info, IMX327_RHS1, init_d, 3);
	turning_data->sensor_data.RHS1 = init_d[2];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[1];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[0];

	ret = sensor_spi_read(sensor_info, IMX327_RHS2, init_d, 3);
	turning_data->sensor_data.RHS2 = init_d[2];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[1];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[0];

	turning_data->sensor_data.lines_per_second = 11250;
	ret = sensor_spi_read(sensor_info, IMX327_X_SIZE, init_d, 2);
	turning_data->sensor_data.active_width = init_d[1];
	turning_data->sensor_data.active_width = (turning_data->sensor_data.active_width << 8) | init_d[0];

	ret = sensor_spi_read(sensor_info, IMX327_Y_SIZE, init_d, 2);
	turning_data->sensor_data.active_height = init_d[1];
	turning_data->sensor_data.active_height = (turning_data->sensor_data.active_height << 8) | init_d[0];
	turning_data->sensor_data.turning_type = 7;   // imx327 calc
	return ret;
}

static int sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 1;
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(imx327_stream_on_setting)) {
		memcpy(stream_on, imx327_stream_on_setting, sizeof(imx327_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_off) >=
			sizeof(imx327_stream_off_setting)) {
		memcpy(stream_off, imx327_stream_off_setting, sizeof(imx327_stream_off_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}
static int sensor_normal_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX327_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX327_SHS1;
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
	turning_data.normal.again_control[0] = IMX327_GAIN;
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
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

static int sensor_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}

	turning_data.dol2.param_hold = IMX327_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;
	turning_data.dol2.s_line = IMX327_SHS1;   //  SEF1
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX327_SHS2;   //  LEF
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
	turning_data.dol2.again_control[0] = IMX327_GAIN;     //   LEF  SHS2
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_control[0] = IMX327_GAIN1;    //   SEF1  SHS1
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
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
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX327_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX327_SHS2;   //  LEF
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
	turning_data.normal.again_control[0] = IMX327_GAIN;   //  LEF SHS2
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
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
static int sensor_dol2_short_frame_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX327_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX327_SHS1;   //  SEF1
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
	turning_data.normal.again_control[0] = IMX327_GAIN1; // SEF1 GAIN1
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
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
static int sensor_dol3_long_frame_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX327_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX327_SHS3;   //  LEF
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
	turning_data.normal.again_control[0] = IMX327_GAIN;   // LEF
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
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
static int sensor_dol3_short_frame_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX327_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX327_SHS2;   // SEF2
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
	turning_data.normal.again_control[0] = IMX327_GAIN2;   // SEF2
	turning_data.normal.again_control_length[0] = 1;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
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

static int sensor_dol3_LEF_SEF1_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.dol2.param_hold = IMX327_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;
	turning_data.dol2.s_line = IMX327_SHS1;    //  SEF1
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX327_SHS3;    //  LEF
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
	turning_data.dol2.again_control[0] = IMX327_GAIN;
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_control[0] = IMX327_GAIN1;
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}

	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	return ret;
}
static int sensor_dol3_LEF_SEF2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.dol2.param_hold = IMX327_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;
	turning_data.dol2.s_line = IMX327_SHS2;    //  SEF2
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX327_SHS3;   //  LEF
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
	turning_data.dol2.again_control[0] = IMX327_GAIN;     //  LEF
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_control[0] = IMX327_GAIN2;     //  LEF
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}

	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	return ret;
}
static int sensor_dol3_SEF1_SEF2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.dol2.param_hold = IMX327_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;
	turning_data.dol2.s_line = IMX327_SHS2;   //  SEF2
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX327_SHS1;   //  SEF1
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
	turning_data.dol2.again_control[0] = IMX327_GAIN1;   //  SEF1
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_control[0] = IMX327_GAIN2;   //  SEF2
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}

	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	return ret;
}
static int sensor_dol3_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
			pr_err("sensor_%d open fail\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.dol3.param_hold = IMX327_PARAM_HOLD;
	turning_data.dol3.param_hold_length = 1;
	turning_data.dol3.s_line = IMX327_SHS2;    //  SEF2
	turning_data.dol3.s_line_length = 3;
	turning_data.dol3.m_line = IMX327_SHS1;    //  SEF1
	turning_data.dol3.m_line_length = 3;
	turning_data.dol3.l_line = IMX327_SHS3;    //  LEF
	turning_data.dol3.l_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}
#ifdef TUNING_LUT
	turning_data.dol3.line_p[0].ratio = -256;
	turning_data.dol3.line_p[0].offset = turning_data.sensor_data.RHS1 - 1;
	turning_data.dol3.line_p[0].max = turning_data.sensor_data.RHS1 - 2;
	turning_data.dol3.line_p[1].ratio = -256;
	turning_data.dol3.line_p[1].offset = turning_data.sensor_data.RHS2 - 1;
	turning_data.dol3.line_p[1].max = turning_data.sensor_data.RHS2 - 2;
	turning_data.dol3.line_p[2].ratio = -256;
	turning_data.dol3.line_p[2].offset = turning_data.sensor_data.FSC_DOL3 - 1;
	turning_data.dol3.line_p[2].max = turning_data.sensor_data.FSC_DOL3 - 2;

	turning_data.dol3.again_control_num = 3;
	turning_data.dol3.again_control[0] = IMX327_GAIN;      //  LEF    SHS3
	turning_data.dol3.again_control_length[0] = 1;
	turning_data.dol3.again_control[0] = IMX327_GAIN1;     //  SEF1   SHS1
	turning_data.dol3.again_control_length[0] = 1;
	turning_data.dol3.again_control[0] = IMX327_GAIN2;     //  SEF2   SHS2
	turning_data.dol3.again_control_length[0] = 1;
	turning_data.dol3.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol3.again_lut != NULL) {
		memset(turning_data.dol3.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol3.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}
	if (turning_data.dol3.again_lut) {
		free(turning_data.dol3.again_lut);
		turning_data.dol3.again_lut = NULL;
	}
	return ret;
}

int sensor_normal_update_notify_driver(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}
	turning_data.normal.param_hold = IMX327_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX327_SHS1;
	turning_data.normal.s_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}
	pr_info("sensor_normal_update_notify_driver\n");
	pr_info("sensor_info fps %d\n\n", sensor_info->fps);
#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = -256;
	turning_data.normal.line_p.offset = turning_data.sensor_data.VMAX - 1;
	turning_data.normal.line_p.max = turning_data.sensor_data.VMAX - 2;

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = IMX327_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
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

int sensor_dol2_update_notify_driver(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	} else {
		sensor_spi_param_init(sensor_info, &turning_data);
	}

	turning_data.dol2.param_hold = IMX327_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;
	turning_data.dol2.s_line = IMX327_SHS1;
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX327_SHS2;
	turning_data.dol2.m_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}
	pr_info("sensor_dol2_update_notify_driver\n");
	pr_info("sensor_info fps %d\n\n", sensor_info->fps);
#ifdef TUNING_LUT
	turning_data.dol2.line_p[0].ratio = -256;
	turning_data.dol2.line_p[0].offset = turning_data.sensor_data.RHS1 - 1;
	turning_data.dol2.line_p[0].max = turning_data.sensor_data.RHS1 - 2;
	turning_data.dol2.line_p[1].ratio = -256;
	turning_data.dol2.line_p[1].offset = turning_data.sensor_data.FSC_DOL2 - 1;
	turning_data.dol2.line_p[1].max = turning_data.sensor_data.FSC_DOL2 - 2;

	turning_data.dol2.again_control_num = 1;
	turning_data.dol2.again_control[0] = IMX327_GAIN;
	turning_data.dol2.again_control_length[0] = 2;
	turning_data.dol2.dgain_control_num = 0;
	turning_data.dol2.dgain_control[0] = 0;
	turning_data.dol2.dgain_control_length[0] = 0;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx327_gain_lut,
			sizeof(imx327_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx327_gain_lut)/sizeof(uint32_t); open_cnt++) {
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor devfd %d ioctl fail %d\n", sensor_info->sen_devfd, ret);
		return -RET_ERROR;
	}
	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	return ret;
}

int sensor_update_fps_notify_driver(sensor_info_t *sensor_info)
{
		int ret = RET_OK;

		switch(sensor_info->sensor_mode) {
			case NORMAL_M:  //  normal
				ret = sensor_normal_update_notify_driver(sensor_info);
				if (ret < 0) {
					pr_err("sensor_normal_update_notify_driver fail\n");
					return ret;
				}
				break;
			case DOL2_M:
				ret = sensor_dol2_update_notify_driver(sensor_info);
				if (ret < 0) {
					pr_err("sensor_dol2_update_notify_driver fail\n");
					return ret;
				}
				break;
			default:
				break;
		}
		return ret;
}
static int sensor_10fps_dynamic_switch(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if(sensor_info->sensor_mode == NORMAL_M) {
		setting_size = sizeof(imx327_10fps_normal_setting)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx327_10fps_normal_setting);
			if (ret < 0) {
				pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}
	usleep(20*1000);
	return ret;
}
static int sensor_15fps_dynamic_switch(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if(sensor_info->sensor_mode == NORMAL_M) {
		setting_size = sizeof(imx327_15fps_normal_setting)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx327_15fps_normal_setting);
			if (ret < 0) {
				pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
		setting_size = sizeof(imx327_15fps_dol2_setting)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx327_15fps_dol2_setting);
			if (ret < 0) {
				pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}
	usleep(20*1000);
	return ret;
}
static int sensor_25fps_dynamic_switch(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if(sensor_info->sensor_mode == NORMAL_M) {
		setting_size = sizeof(imx327_25fps_normal_setting)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx327_25fps_normal_setting);
			if (ret < 0) {
				pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
		setting_size = sizeof(imx327_25fps_dol2_setting)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx327_25fps_dol2_setting);
			if (ret < 0) {
				pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
	    }
	}
	usleep(20*1000);
	return ret;
}
static int sensor_30fps_dynamic_switch(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	if(sensor_info->sensor_mode == NORMAL_M) {
		setting_size = sizeof(imx327_30fps_normal_setting)/sizeof(uint32_t)/2;
		if(sensor_info->bus_type == I2C_BUS) {
			ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					2, setting_size, imx327_30fps_normal_setting);
			if (ret < 0) {
				pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
				return ret;
			}
		}
	} else if (sensor_info->sensor_mode == DOL2_M) {
		setting_size = sizeof(imx327_30fps_dol2_setting)/sizeof(uint32_t)/2;
			if(sensor_info->bus_type == I2C_BUS) {
				ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
						2, setting_size, imx327_30fps_dol2_setting);
				if (ret < 0) {
					pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			}
	}
	usleep(20*1000);
	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int ret = RET_OK;
	char buf[2];

	switch (fps) {
		case IMX327_10FPS:
			pr_info("IMX327_10FPS\n");
			buf[0] = 0x01;
		    ret = hb_i2c_write_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
					IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			ret = sensor_10fps_dynamic_switch(sensor_info);
			if(ret < 0) {
				pr_err("sensor_10fps_dynamic_switch fail\n");
				return -RET_ERROR;
			}
			buf[0] = 0x00;
			ret = hb_i2c_write_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
					IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			break;
		case IMX327_15FPS:
			pr_info("IMX327_15FPS\n");
			buf[0] = 0x01;
		    ret = hb_i2c_write_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
					IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			ret = sensor_15fps_dynamic_switch(sensor_info);
			if(ret < 0) {
				pr_err("sensor_15fps_dynamic_switch fail\n");
				return -RET_ERROR;
			}
			buf[0] = 0x00;
			ret = hb_i2c_write_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
					IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			break;
		case IMX327_25FPS:
			pr_info("IMX327_25FPS\n");
			buf[0] = 0x01;
		    ret = hb_i2c_write_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
					IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			ret = sensor_25fps_dynamic_switch(sensor_info);
			if(ret < 0) {
				pr_err("sensor_25fps_dynamic_switch fail\n");
				return -RET_ERROR;
			}
			buf[0] = 0x00;
			ret = hb_i2c_write_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
					IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			break;
		case IMX327_30FPS:
			pr_info("IMX327_30FPS\n");
			buf[0] = 0x01;
		    ret = hb_i2c_write_block_reg16(sensor_info->bus_num,
					sensor_info->sensor_addr, IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			ret = sensor_30fps_dynamic_switch(sensor_info);
			if(ret < 0) {
				pr_err("sensor_30fps_dynamic_switch fail\n");
				return -RET_ERROR;
			}
			buf[0] = 0x00;
			ret = hb_i2c_write_block_reg16(sensor_info->bus_num,
					sensor_info->sensor_addr, IMX327_PARAM_HOLD, buf, 1);
			if(ret < 0) {
				pr_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
				return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
			}
			break;
		default:
			pr_err("not suport fps type %d\n", fps);
			break;
	}
	sensor_info->fps = fps;
	sensor_update_fps_notify_driver(sensor_info);
	pr_info("dynamic_switch to %dfps success\n", fps);
	return RET_OK;
}

static int sensor_new_config_func(sensor_info_t *sensor_info, uint32_t *imx327_config, int size)
{
	int ret = RET_OK;
	int setting_size, i;
	char temp_data;

	setting_size = size/sizeof(uint32_t)/2;
	if(sensor_info->bus_type == I2C_BUS) {
		ret = camera_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, imx327_config);
		if (ret < 0) {
			pr_err("camera_write_array %s fail\n", sensor_info->sensor_name);
			return ret;
		}
	} else {
		for(i = 0; i < setting_size; i++) {
			temp_data = imx327_config[i*2 + 1];
			ret = sensor_spi_write(sensor_info, imx327_config[i*2], &temp_data, 1);
			if (ret < 0) {
				pr_err("sensor_spi_write %s fail\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}
	return ret;
}

int sensor_imx327_normal_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int fps = sensor_info->fps;
	int resolution = sensor_info->resolution;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
			pr_err("sensor_%d open fail\n", sensor_info->dev_port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n", sensor_info->dev_port, sensor_info->sen_devfd);

	switch (fps) {
		case IMX327_25FPS:
			if(resolution == 1097) {
				ret = sensor_new_config_func(sensor_info, imx327_1097p_linear_25fps_setting,
					sizeof(imx327_1097p_linear_25fps_setting));
			} else if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx327_720p_linear_25fps_setting,
					sizeof(imx327_720p_linear_25fps_setting));
			} else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx327_1080p_linear_25fps_setting,
					sizeof(imx327_1080p_linear_25fps_setting));
			}
			break;
		case IMX327_30FPS:
			if(resolution == 1097) {
				ret = sensor_new_config_func(sensor_info, imx327_1097p_linear_30fps_setting,
					sizeof(imx327_1097p_linear_30fps_setting));
			} else if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx327_720p_linear_30fps_setting,
					sizeof(imx327_720p_linear_30fps_setting));
			} else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx327_1080p_linear_30fps_setting,
					sizeof(imx327_1080p_linear_30fps_setting));
			}
			break;
		default:
			pr_err("not support fps type %d\n", fps);
			ret = -RET_ERROR;
			break;
	}
	pr_info("imx327_linear_config %dfps_%dP success\n", fps, resolution);
	return ret;
}
int sensor_imx327_dol2_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, req;
	int fps = sensor_info->fps;
	int resolution = sensor_info->resolution;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
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
		case IMX327_25FPS:
			if(resolution == 2228) {
				ret = sensor_new_config_func(sensor_info, imx327_2228p_dol2_25fps_setting,
					sizeof(imx327_2228p_dol2_25fps_setting));
			} else if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx327_720p_dol2_25fps_setting,
									sizeof(imx327_720p_dol2_25fps_setting));
			} else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx327_1080p_dol2_25fps_setting,
									sizeof(imx327_1080p_dol2_25fps_setting));
			}
			pr_info("imx327 dol2 25fps init success req %d\n",req);
			break;
		case IMX327_30FPS:
			if(resolution == 2228) {
				ret = sensor_new_config_func(sensor_info, imx327_2228p_dol2_30fps_setting,
					sizeof(imx327_2228p_dol2_30fps_setting));
			} else if (resolution == 720) {
				ret = sensor_new_config_func(sensor_info, imx327_720p_dol2_30fps_setting,
					sizeof(imx327_720p_dol2_30fps_setting));
			} else if (resolution == 1080) {
				ret = sensor_new_config_func(sensor_info, imx327_1080p_dol2_30fps_setting,
									sizeof(imx327_1080p_dol2_30fps_setting));
			}
			pr_info("imx327 dol2 30fps init success req %d\n", req);
			break;
		default:
			pr_err("not support fps type %d\n", fps);
			ret = -RET_ERROR;
			break;
	}
	hb_vin_pre_result(sensor_info->entry_num, 0, ret);
	if (ret < 0) {
		pr_err("imx327 dol2 init fail\n");
	}
	return ret;
}

int sensor_imx327_dol3_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK, req;
	int fps = sensor_info->fps;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR | O_CLOEXEC)) < 0) {
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
		case IMX327_15FPS:
			ret = sensor_new_config_func(sensor_info, imx327_3609p_dol3_15fps_setting,
					sizeof(imx327_3609p_dol3_15fps_setting));
			pr_info("imx327 dol3 15fps init success req %d\n", req);
			break;
		default:
			pr_err("not support fps type %d\n", fps);
			ret = -RET_ERROR;
			break;
	}
	hb_vin_pre_result(sensor_info->entry_num, 0, ret);
	if (ret < 0) {
		pr_err("imx327 dol3 init fail\n");
	}
	return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:    //  normal
			ret = sensor_imx327_normal_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_imx327_normal_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			ret = sensor_normal_data_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:  //  DOl2
			ret = sensor_imx327_dol2_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_imx327_dol2_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			if(sensor_info->extra_mode == 0) {    //   dol2
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
		case DOL3_M:  //  DOl3
			ret = sensor_imx327_dol3_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_imx327_dol3_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			if(sensor_info->extra_mode == 0) {     //   dol3
				ret = sensor_dol3_data_init(sensor_info);
				if(ret < 0) {
					pr_err("sensor_dol3_data_init %s fail\n", sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == 1) {    // DOL2+linear
				ret = sensor_dol3_LEF_SEF2_data_init(sensor_info);     // LEF + SHS2 长+中
				if(ret < 0) {
					pr_err("sensor_dol3_LEF_SEF1_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			}
			#if 0
			else if (sensor_info->extra_mode == 2) {  // DOL2+linear
				ret = sensor_dol3_LEF_SEF2_data_init(sensor_info);     //  LEF + SHS2 长+短
				if(ret < 0) {
					pr_err("sensor_dol3_LEF_SEF2_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == 3) {    // DOL2+linear
				ret = sensor_dol3_SEF1_SEF2_data_init(sensor_info);  //  SEF1 + SEF2 中+短
				if(ret < 0) {
					pr_err("sensor_dol3_SEF1_SEF2_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			}
			#endif
			else if (sensor_info->extra_mode == 2) {    //  three linear
				ret = sensor_dol3_long_frame_data_init(sensor_info);    //  LEF SHS3
				if(ret < 0) {
					pr_err("sensor_dol3_long_frame_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == 3) {
				ret = sensor_dol2_short_frame_data_init(sensor_info);   //  medium SEF1  SHS1
				if(ret < 0) {
					pr_err("sensor_dol2_short_frame_data_init %s fail\n",
						sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == 4) {
				ret = sensor_dol3_short_frame_data_init(sensor_info);    //  SEF2  SHS2
				if(ret < 0) {
					pr_err("sensor_dol3_short_frame_data_init %s fail\n",
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
	pr_info("imx327 config success under %d mode\n\n", sensor_info->sensor_mode);
	return ret;
}
static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;
	char temp_data;

	setting_size = sizeof(imx327_stream_off_setting)/sizeof(uint32_t)/2;
	if(sensor_info->bus_type == I2C_BUS) {
		ret = camera_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, 2, setting_size,
			imx327_stream_off_setting);
		if(ret < 0) {
			pr_err("sensor_stop %s fail\n", sensor_info->sensor_name);
		}
	} else {
		for(i = 0; i < setting_size; i++) {
			temp_data = imx327_stream_off_setting[i*2 + 1];
			ret = sensor_spi_write(sensor_info,
				imx327_stream_off_setting[i*2],
				&temp_data, 1);
			if (ret < 0) {
				pr_err("sensor_spi_write %s fail\n", sensor_info->sensor_name);
			}
		}
	}

	return ret;
}
static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK, i;
	int setting_size = 0;
	char temp_data;

	setting_size = sizeof(imx327_stream_on_setting)/sizeof(uint32_t)/2;
	if(sensor_info->bus_type == I2C_BUS) {
			for(i = 0; i < setting_size; i++) {
				ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
						imx327_stream_on_setting[i*2], imx327_stream_on_setting[i*2 + 1]);
				if(i == 0) {
					usleep(25000);
				}
				if (ret < 0) {
					pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
	} else {
		for(i = 0; i < setting_size; i++) {
			temp_data = imx327_stream_on_setting[i*2 + 1];
			ret = sensor_spi_write(sensor_info, imx327_stream_on_setting[i*2],
				&temp_data, 1);
			if (ret < 0) {
				pr_err("sensor_spi_write %s fail\n", sensor_info->sensor_name);
			}
		}
	}

	return ret;
}
static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

sensor_module_t imx327 = {
	.module = "imx327",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.spi_write = sensor_spi_write,
	.spi_read = sensor_spi_read,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};

