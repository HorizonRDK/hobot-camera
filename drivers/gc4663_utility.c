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
#include "inc/gc4663_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

int sensor_reset(sensor_info_t *sensor_info)
{
        pr_info( "sensor_reset gpio_num %d", sensor_info->gpio_num ); 
	int gpio, ret = RET_OK;
	// if(sensor_info->power_mode) {
	if( 1 ) {
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
int sensor_4M_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;

	turning_data->sensor_data.active_width = 2560;
	turning_data->sensor_data.active_height = 1440;
	turning_data->sensor_data.gain_max =  512 * 8192;
	turning_data->sensor_data.analog_gain_max = 512 * 8192;
	turning_data->sensor_data.digital_gain_max = 0 * 8192;
	turning_data->sensor_data.exposure_time_min = 2;
	turning_data->sensor_data.exposure_time_max = 1480;
	turning_data->sensor_data.exposure_time_long_max = 1480;
	turning_data->sensor_data.lines_per_second = 45004;   // 482M/5024
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.fps = sensor_info->fps;  // fps
	// turning_data->sensor_data.conversion = 1;
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
	if(size >= sizeof(gc4663_stream_on_setting)) {
		memcpy(stream_on, gc4663_stream_on_setting,
				sizeof(gc4663_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	size = sizeof(turning_data->stream_ctrl.stream_off);
	if(size >= sizeof(gc4663_stream_off_setting)) {
		memcpy(stream_off, gc4663_stream_off_setting,
				sizeof(gc4663_stream_off_setting));
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
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	sensor_4M_param_init(sensor_info, &turning_data);

	turning_data.normal.again_control_num = 0;
        turning_data.normal.dgain_control_num = 0;
        turning_data.normal.s_line_length = 0;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}
  
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;

	/*linear mode*/
	// if(sensor_info->config_index >= 0) {
	if( 1 ) {
		setting_size = sizeof(gc4663_init_2560X1440_linear_setting)/sizeof(uint32_t)/2;
		pr_info("x3 setting_size %d\n", setting_size);
		for(i = 0; i < setting_size; i++) {
			ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr,
				gc4663_init_2560X1440_linear_setting[i*2],
				gc4663_init_2560X1440_linear_setting[i*2 + 1]);
			if (ret < 0) {
				pr_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
					sensor_info->sensor_name, sensor_info->bus_num,
					sensor_info->sensor_addr, i,
					gc4663_init_2560X1440_linear_setting[i*2], gc4663_init_2560X1440_linear_setting[i*2 + 1]);
				return ret;
			}
			// if(i == 3)
			// 	usleep(3*1000);
		}
		pr_info("GC4663_2560X1440_linear_10bit_config OK!\n");
	}else {
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
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			pr_err("port_%d open fail\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
		sensor_info->dev_port, sensor_info->sen_devfd);

/*
	ret = sensor_reset(sensor_info);
	if (ret < 0) {
		pr_err("%d : reset %s fail\n", __LINE__, sensor_info->sensor_name);
		return -RET_ERROR;
	}
*/
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0)
		pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	setting_size = sizeof(gc4663_stream_on_setting)/sizeof(uint32_t)/2;
	pr_info("%s sensor_start setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	usleep(10 *1000);
	for(i = 0; i < setting_size; i++) {
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, gc4663_stream_on_setting[i*2],
			gc4663_stream_on_setting[i*2 + 1]);
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

	// sleep(50);
	setting_size = sizeof(gc4663_stream_off_setting)/sizeof(uint32_t)/2;
	pr_info("%s sensor_stop setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, gc4663_stream_off_setting[i*2],
			gc4663_stream_off_setting[i*2 + 1]);
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

	// sleep(50);
	// if(sensor_info->power_mode) {
	if( 0 ) {
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
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	} else {
            pr_info( "gc4663:  sensor_info->sen_devfd == 0 \n"); 
        }
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
        // return 0;
        // printf("%s %s mode:%d gain_num:%d again[0]:%d, dgain[0]:%d\n", __FILE__, __FUNCTION__, mode, gain_num, again[0], dgain[0]);
        int bus = info->bus_num;
        int sensor_addr = info->sensor_addr;

        uint32_t gain = again[0];
	uint32_t i;
	uint32_t total;
	uint32_t tol_dig_gain = 0;
	int ret = RET_OK;

	if (gain > 190) {
		gain = 190;
	}
        gain = analog_gain_ratio[gain];

	total = sizeof(analog_gain_table) / sizeof( uint32_t );
	for(i = 0; i < total - 1 ; i++)
	{
	  if((analog_gain_table[i] <= gain)&&(gain < analog_gain_table[i+1]))
	    break;
	}
	tol_dig_gain = gain * 64 / analog_gain_table[i];

        // printf("gc4663 gain, gain= %d, i= %d, tol_dig_gain=%d \n", gain, i, tol_dig_gain);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x2b3, regValTable[i][0]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x2b4, regValTable[i][1]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x2b8, regValTable[i][2]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x2b9, regValTable[i][3]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x515, regValTable[i][4]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x519, regValTable[i][5]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x2d9, regValTable[i][6]);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x20e, (tol_dig_gain>>6));
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x20f, ((tol_dig_gain&0x3f)<<2));

        return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
    // return 0;
        // printf("%s %s mode:%d line_num:%d line[0]:%d\n", __FILE__, __FUNCTION__, mode, line_num, line[0]);
// extern int camera_i2c_write8(int bus, int reg_width, int sensor_addr, uint32_t reg_addr, uint8_t value); 
        int bus = info->bus_num;
        int sensor_addr = info->sensor_addr;
        char temp = 0, temp1 = 0, temp2 = 0; 
        uint32_t sline = line[0];
        if ( sline > 1480) { sline = 1480; }
        if ( sline < 5) { sline = 5; }

	temp1 = ((sline) >> 8) & 0xff;
        // camera_i2c_write8( bus, 16, sensor_addr, 0x202, temp1);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x202, temp1);
	temp2 = (sline & 0xff);
        // camera_i2c_write8( bus, 16, sensor_addr, 0x203, temp2);
        hb_i2c_write_reg16_data8( bus, sensor_addr, 0x203, temp2);

        // printf(" bus=%d, sensor_addr=%02X \n", bus, sensor_addr );
        // printf("_line_control 0x202 0x203: %02X, %02X\n", temp1, temp2);
    return 0;
}


static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
        *enable |= (HAL_GAIN_CONTROL | HAL_LINE_CONTROL);
        // *enable = 0;
        return 0;
}
       

sensor_module_t gc4663 = {
	.module = "gc4663",
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
        .aexp_gain_control = sensor_aexp_gain_control,
        .aexp_line_control = sensor_aexp_line_control,
        .userspace_control = sensor_userspace_control,
};

