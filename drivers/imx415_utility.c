/*
* fam 2021.05
*/
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
#include <arpa/inet.h>
#include <linux/spi/spidev.h>

#include "inc/hb_cam_utility.h"
#include "inc/hb_i2c.h"
#include "inc/imx415_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT


static uint32_t imx415_max(int32_t a, int32_t b)
{
    if (a > b) return a;
    else  return b;
}


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

static int sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	int size;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	size = sizeof(turning_data->stream_ctrl.stream_on);
	if(size >= sizeof(imx415_stream_on_setting)) {
		memcpy(stream_on, imx415_stream_on_setting,
				sizeof(imx415_stream_on_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	size = sizeof(turning_data->stream_ctrl.stream_off);
	if(size >= sizeof(imx415_stream_off_setting)) {
		memcpy(stream_off, imx415_stream_off_setting,
				sizeof(imx415_stream_off_setting));
	} else {
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

static int sensor_mode_config_i2c_write(sensor_info_t *sensor_info, uint16_t *pbuf, size_t size)
{
    int ret = RET_OK;
    int setting_size = 0, i;

    setting_size = size / sizeof(uint16_t) / 2;
    pr_info("imx415 setting_size %d\n", setting_size);
    for (i = 0; i < setting_size; i++) {
        ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
                                       sensor_info->sensor_addr,
                                       pbuf[i * 2],
                                       pbuf[i * 2 + 1]);
        if (ret < 0) {
            pr_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
                   sensor_info->sensor_name, sensor_info->bus_num,
                   sensor_info->sensor_addr, i,
                   pbuf[i * 2], pbuf[i * 2 + 1]);
            return ret;
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

static int sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	char init_d[3];
	int fps, Hmax;
	float time_ofline;

	fps = sensor_info->fps;
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VAMX, init_d, 3);
	turning_data->sensor_data.VMAX = init_d[2];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[0];
	turning_data->sensor_data.FSC_DOL2 = turning_data->sensor_data.VMAX * 2;
	turning_data->sensor_data.FSC_DOL3 = turning_data->sensor_data.VMAX * 4;
	pr_info("FSC_DOL2 %d,  FSC_DOL3 %d \n", turning_data->sensor_data.FSC_DOL2, turning_data->sensor_data.FSC_DOL3);
	turning_data->sensor_data.gain_max = 160;
	turning_data->sensor_data.analog_gain_max = 255*8192;
	turning_data->sensor_data.digital_gain_max = 0*8192;
	turning_data->sensor_data.exposure_time_min = 1;
	if (sensor_info->sensor_mode == NORMAL_M) {
		turning_data->sensor_data.exposure_time_max =
				turning_data->sensor_data.VMAX - 2;
	} else if (sensor_info->sensor_mode == DOL2_M) {
		turning_data->sensor_data.exposure_time_max = 9;
	}
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_HAMX, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[1];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[0];

	pr_info("HMAX %d,  VMAX %d \n", turning_data->sensor_data.HMAX,  turning_data->sensor_data.VMAX);

	Hmax = turning_data->sensor_data.HMAX;
	// time_ofline = 2200.0*Hmax/4400/74.25MHZ; 30帧是2200 25帧是2640，结果算出来一样
	// DOL2的文档 P10 Linear 的文档 P49
	if (sensor_info->sensor_mode == NORMAL_M) {
		time_ofline = 237600000.0/fps/Hmax;
		turning_data->sensor_data.exposure_time_max = time_ofline;  // 一帧总行数，包括blanking
	} else if (sensor_info->sensor_mode == DOL2_M) {
		time_ofline = 237600000.0/fps/Hmax;
		turning_data->sensor_data.exposure_time_long_max = time_ofline;
	}
	turning_data->sensor_data.fps = sensor_info->fps;
	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_RHS1, init_d, 3);
	turning_data->sensor_data.RHS1 = init_d[2];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[1];
	turning_data->sensor_data.RHS1 = (turning_data->sensor_data.RHS1 << 8) | init_d[0];

	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_RHS2, init_d, 3);
	turning_data->sensor_data.RHS2 = init_d[2];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[1];
	turning_data->sensor_data.RHS2 = (turning_data->sensor_data.RHS2 << 8) | init_d[0];
	pr_info("RHS1 %d,  RHS2 %d \n",  turning_data->sensor_data.RHS1, turning_data->sensor_data.RHS2);

	turning_data->sensor_data.lane = 4;
	turning_data->sensor_data.clk = 27000000;
	turning_data->sensor_data.lines_per_second = 134960; //67114;
	pr_info("lines_per_second %d \n", turning_data->sensor_data.lines_per_second);

	turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.RHS1 - 9 - 1;
	turning_data->sensor_data.exposure_time_long_max = turning_data->sensor_data.FSC_DOL2 - (turning_data->sensor_data.RHS1 + 9) - 1;
	pr_info("exposure_time_max %d\n", turning_data->sensor_data.exposure_time_max);
	pr_info("exposure_time_long_max %d\n", turning_data->sensor_data.exposure_time_long_max);

	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				PIX_HWIDTH, init_d, 2);
	turning_data->sensor_data.active_width = init_d[1];
	turning_data->sensor_data.active_width = (turning_data->sensor_data.active_width << 8) | init_d[0];

	ret = hb_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				PIX_VWIDTH, init_d, 2);
	turning_data->sensor_data.active_height = init_d[1];
	turning_data->sensor_data.active_height = (turning_data->sensor_data.active_height << 8) | init_d[0];
        turning_data->sensor_data.active_height = turning_data->sensor_data.active_height >> 1;

        // turning_data->sensor_data.active_height = 2160;
        // turning_data->sensor_data.active_width = 3840;

	pr_info("active_width %d active_height %d\n", turning_data->sensor_data.active_width, turning_data->sensor_data.active_height);
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.step_gain = 3;   // step_gain
	return ret;
}


static int sensor_normal_data_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
	char str[12] = {0};
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
        // sensor_param_init(sensor_info, &turning_data);

        int Hmax = 4164, Vmax = 0x08ca;

        // turning_data.sensor_data.VMAX = 0x08ca;
        // turning_data.sensor_data.HMAX = 0x042a;
        // Hmax = turning_data.sensor_data.HMAX;

	turning_data.sensor_data.active_width = 3840;
	turning_data.sensor_data.active_height = 2160;
	turning_data.sensor_data.analog_gain_max = 255 * 8192;
	turning_data.sensor_data.digital_gain_max = 0 * 8192;	
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_max = Vmax - 8;
	// turning_data.sensor_data.exposure_time_long_max = Vmax - 1;
	// turning_data.sensor_data.lines_per_second = 288000000 / Hmax;  
	turning_data.sensor_data.lines_per_second = 67114; // 67114  2250*30// 
	turning_data.sensor_data.turning_type = 6;   // gain calc
	turning_data.sensor_data.fps = sensor_info->fps;  // fps
   
        //   sensor turning data init 

	turning_data.normal.param_hold = IMX415_PARAM_HOLD;
	turning_data.normal.param_hold_length = 1;
	turning_data.normal.s_line = IMX415_LINE;
	turning_data.normal.s_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = -256;
	turning_data.normal.line_p.offset = 2246; 
	turning_data.normal.line_p.max = 2237;

	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = IMX415_GAIN;
	turning_data.normal.again_control_length[0] = 1;

	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));

		memcpy(turning_data.normal.again_lut, imx415_gain_lut,
			sizeof(imx415_gain_lut));
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
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
        sensor_param_init(sensor_info, &turning_data);

        //   sensor turning data init 
	turning_data.dol2.param_hold = IMX415_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 1;


	turning_data.dol2.s_line = IMX415_SHR0;   //  SEF1
	turning_data.dol2.s_line_length = 3;
	turning_data.dol2.m_line = IMX415_SHR1;   //  LEF
	turning_data.dol2.m_line_length = 3;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		pr_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

        int open_cnt = 0;

#ifdef TUNING_LUT
	turning_data.dol2.line_p[0].ratio = -256;
	turning_data.dol2.line_p[0].offset = turning_data.sensor_data.RHS1; // max
	turning_data.dol2.line_p[0].max = turning_data.sensor_data.RHS1 - 9; // 
	turning_data.dol2.line_p[1].ratio = -256;
	turning_data.dol2.line_p[1].offset = turning_data.sensor_data.FSC_DOL2; // max
	turning_data.dol2.line_p[1].max = turning_data.sensor_data.FSC_DOL2 - (turning_data.sensor_data.RHS1 + 9);
	pr_info(" line_p[0]: offset-%d max-%d,  line_p[1]: offset-%d max-%d\n", turning_data.dol2.line_p[0].offset, turning_data.dol2.line_p[0].max, turning_data.dol2.line_p[1].offset, turning_data.dol2.line_p[1].max);
 
	turning_data.dol2.again_control_num = 2;
	turning_data.dol2.again_control[0] = GAIN_PCG0;     //   LEF  SHS2
	turning_data.dol2.again_control_length[0] = 1;
	turning_data.dol2.again_control[1] = GAIN_PCG1;    //   SEF1  SHS1
	turning_data.dol2.again_control_length[1] = 1;
	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx415_gain_lut,
			sizeof(imx415_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(imx415_gain_lut)/sizeof(uint32_t); open_cnt++) {
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


int sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint16_t tmp;

	switch (sensor_info->sensor_mode) {
	case NORMAL_M:
		if(sensor_info->fps == 30){
			if ((ret = sensor_mode_config_i2c_write(sensor_info,
							imx415_init_3840x2160_linear_setting,
							sizeof(imx415_init_3840x2160_linear_setting))) < 0)
				return ret;
			ret = sensor_normal_data_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			printf("imx415_init_3840x2160_linear_setting OK!\n");

		}else if(sensor_info->fps == 60){
			if ((ret = sensor_mode_config_i2c_write(sensor_info,
							imx415_init_3840x2160_60_fps_linear_setting,
							sizeof(imx415_init_3840x2160_60_fps_linear_setting))) < 0)
				return ret;
			ret = sensor_normal_data_init(sensor_info);
			if(ret < 0) {
				pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			printf("imx415_init_3840x2160_60_fps_linear_setting OK!\n");
		}
		break;

	case DOL2_M:
		if ((ret = sensor_mode_config_i2c_write(sensor_info, imx415_init_3840x2160_dol2_setting, sizeof(imx415_init_3840x2160_dol2_setting))) < 0)
			// if ((ret = sensor_mode_config_i2c_write(sensor_info, imx415_sony_init_3840x2160_dol2_setting,  sizeof(imx415_sony_init_3840x2160_dol2_setting))) < 0)
			return ret;
		ret = sensor_dol2_data_init(sensor_info);
		if(ret < 0) {
			pr_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
			return ret;
		}
		pr_info("imx415_init_3840x2160_dol2_setting OK!\n");
		break;

	default:
		pr_err("config mode is err, sensor_mode:%d\n", sensor_info->sensor_mode);
		return -RET_ERROR;
	}
	/*
	if(sensor_info->extra_mode == 0) {
	*/
	return ret;
}


static int sensor_init(sensor_info_t *sensor_info)
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

static int sensor_start(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	uint16_t tmp;

	setting_size = sizeof(imx415_stream_on_setting)/sizeof(uint16_t)/2;
	pr_info("%s sensor_start setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, imx415_stream_on_setting[i*2],
			imx415_stream_on_setting[i*2 + 1]);
		if (ret < 0) {
			pr_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		//tmp = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, imx415_stream_on_setting[i*2]);
		//printf("start : reg_add = 0x%x, data = 0x%x\n",imx415_stream_on_setting[i*2], tmp);

	}

	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;


	setting_size = sizeof(imx415_stream_off_setting)/sizeof(uint16_t)/2;
	pr_info("%s sensor_stop setting_size %d\n",
		sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, imx415_stream_off_setting[i*2],
			imx415_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			pr_err("%d : stop %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		//printf("stop:reg_add = 0x%x, data = 0x%x\n",imx415_stream_off_setting[i*2], imx415_stream_off_setting[i*2 + 1]);

	}

	return ret;

}

static int sensor_deinit(sensor_info_t *sensor_info)
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
	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
    // printf(" gain mode %d, --again %d, dgain %d \n", mode, again[0], dgain[0]);
    return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
    return 0;
}

static int sensor_awb_control(hal_control_info_t *info, uint32_t mode, uint32_t rgain, uint32_t bgain, uint32_t grgain, uint32_t gbgain)
{
    printf(" awb ! \n");
    return 0;
}


static int sensor_af_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
    printf(" af ! \n");
    return 0;
}

static int sensor_zoom_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
    printf(" zoom ! \n");
    return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
    // *enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
    // *enable = HAL_LINE_CONTROL;
    *enable = 0;
    return 0;
}

sensor_module_t imx415 = {
	.module = "imx415",
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
