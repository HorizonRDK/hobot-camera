/*
 * KiKi 2022.04
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
#include "inc/sc132gs_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

static int camera_power_fd = 0;
static int sc132gs_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	trace("GPIO:%d  %s\n", sensor_info->gpio_pin[0], sensor_info->sensor_name);

	// ret = camera_power_ctrl(sensor_info->gpio_pin[0], 0);
	// if (ret < 0) {
	// 	pr_err("%d : %s set PWDN = 1 fail\n", __LINE__, sensor_info->sensor_name);
	// 	return ret;
	// }

	// usleep(1 * 1000);

	// Enable MCLK
	ret = hb_cam_set_mclk(sensor_info->entry_num, MCLK);
	// ret = hb_cam_set_mclk(0, MCLK);
	if (ret < 0)
	{
		pr_err("%d : %s set mclk fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	ret = hb_cam_enable_mclk(sensor_info->entry_num);
	// ret = hb_cam_enable_mclk(0);
	if (ret < 0)
	{
		pr_err("%d : %s enable mclk fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	usleep(10 * 1000);

	// XSHUTDOWN=1
	// ret = camera_power_ctrl(sensor_info->gpio_pin[0], 1);
	// if (ret < 0) {
	// 	pr_err("%d : %s set RSTB = 0 fail\n", __LINE__, sensor_info->sensor_name);
	// 	return ret;
	// }
	// usleep(10 * 1000);

	return ret;
}

static int sc132gs_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	trace("\n");
	// DVB PWDN--GPIO6  RSTB--GPIO5
	ret = camera_power_ctrl(sensor_info->gpio_pin[0], 0);
	if (ret < 0)
	{
		pr_debug("%d : %s set PWDN = 0 fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	usleep(100);

	// Disable MCLK
	ret = hb_cam_disable_mclk(sensor_info->entry_num);
	if (ret < 0)
	{
		pr_debug("%d : %s disable MCLK fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	usleep(1000);
#if SC132GS_PMIC
	// disable PMIC
	close(camera_power_fd);
#endif
	return ret;
}

int sc132gs_reset(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if (sensor_info->power_mode)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] >= 0)
			{
				pr_debug("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,
						 sensor_info->gpio_pin[gpio],
						 sensor_info->gpio_level[gpio],
						 sensor_info->gpio_level[gpio]);
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
			}
		}
	}
	return ret;
}

static int sc132gs_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	int size;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 1;
	size = sizeof(turning_data->stream_ctrl.stream_on);
	if (size >= sizeof(sc132gs_stream_on_setting))
	{
		memcpy(stream_on, sc132gs_stream_on_setting, sizeof(sc132gs_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	size = sizeof(turning_data->stream_ctrl.stream_off);
	if (size >= sizeof(sc132gs_stream_off_setting))
	{
		memcpy(stream_off, sc132gs_stream_off_setting, sizeof(sc132gs_stream_off_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

static int sc132gs_mode_config_i2c_write(sensor_info_t *sensor_info, uint32_t *pbuf, size_t size)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint16_t reg, data;
	setting_size = size / sizeof(uint32_t) / 2;
	pr_info("sc132gs setting_size %d\n", setting_size);
	for (i = 0; i < setting_size; i++)
	{
		reg = pbuf[i * 2] & 0xffff;
		data = pbuf[i * 2 + 1] & 0xffff;
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg, data);
		if (ret < 0)
		{
			printf("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
				   sensor_info->sensor_name, sensor_info->bus_num,
				   sensor_info->sensor_addr, i,
				   pbuf[i * 2], pbuf[i * 2 + 1]);
			return ret;
		}
	}
	return ret;
}
void sc132gs_common_data_init(sensor_info_t *sensor_info,
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

int sc132gs_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;
	trace("\n");
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
	turning_data.sensor_data.lines_per_second = 42016;
	turning_data.sensor_data.exposure_time_max = 1500;

	turning_data.sensor_data.active_width = 1088;
	turning_data.sensor_data.active_height = 1280;
	// turning_data.sensor_data.gain_max = 128 * 8192;
	turning_data.sensor_data.analog_gain_max = 153 * 8192;
	turning_data.sensor_data.digital_gain_max = 159 * 8192;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 4000;
	// turning_data.sensor_data.conversion = 1;

	// turning normal
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 1500;

#if 0
	turning_data.normal.s_line = SC132GS_EXP_LINE;
	turning_data.normal.s_line_length = 2;
	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = SC132GS_PROGRAM_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = SC132GS_DIGITAL_GAIN;
	turning_data.normal.dgain_control_length[0] = 2;
#endif
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc132gs_stream_on_setting))
	{
		memcpy(stream_on, sc132gs_stream_on_setting, sizeof(sc132gs_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc132gs_stream_off_setting))
	{
		memcpy(stream_off, sc132gs_stream_off_setting, sizeof(sc132gs_stream_off_setting));
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
		if (256 * sizeof(uint32_t) >= sizeof(sc132gs_again_lut))
		{
			memcpy(turning_data.normal.again_lut, sc132gs_again_lut, sizeof(sc132gs_again_lut));
		}
		else
		{
			pr_err("Number of registers on stream over %d\n", 256 * sizeof(uint32_t));
			return -RET_ERROR;
		}
	}

	turning_data.normal.dgain_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL)
	{
		memset(turning_data.normal.dgain_lut, 0xff, 256 * sizeof(uint32_t));
		if (256 * sizeof(uint32_t) >= sizeof(sc132gs_dgain_lut))
		{
			memcpy(turning_data.normal.dgain_lut, sc132gs_dgain_lut, sizeof(sc132gs_dgain_lut));
		}
		else
		{
			pr_err("Number of registers on stream over %d\n", 256 * sizeof(uint32_t));
			return -RET_ERROR;
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut)
	{
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut)
	{
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}
	if (ret < 0)
	{
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sc132gs_mode_config_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint16_t tmp;

	switch (sensor_info->sensor_mode)
	{
	case NORMAL_M:
		if (sensor_info->extra_mode == 4) 
		{
			printf("sc132gs 4 lane mode\n");
			ret = sc132gs_mode_config_i2c_write(sensor_info, sc132gs_4lane_init_1088x1280_slave_setting, sizeof(sc132gs_4lane_init_1088x1280_slave_setting));
			if (ret < 0)
				return ret;
			ret = sc132gs_linear_data_init(sensor_info);
			if (ret < 0)
			{
				pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			pr_info("sc132gs_init_1088x1280_linear_setting OK!\n");
			break;
		}
		else if (sensor_info->extra_mode == 2)
		{
			printf("sc132gs 2 lane mode\n");
			ret = sc132gs_mode_config_i2c_write(sensor_info, sc132gs_2lane_init_1088x1280_slave_setting, sizeof(sc132gs_2lane_init_1088x1280_slave_setting));
			if (ret < 0)
				return ret;
			ret = sc132gs_linear_data_init(sensor_info);
			if (ret < 0)
			{
				pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			pr_info("sc132gs_init_1088x1280_linear_setting OK!\n");
			break;
		}

	default:
		pr_err("config mode is err, sensor_mode:%d\n", sensor_info->sensor_mode);
		return -RET_ERROR;
	}
	/*
		if(sensor_info->extra_mode == 0) {
	*/
	return ret;
}

static int sc132gs_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if (sensor_info->sen_devfd <= 0)
	{
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0)
		{
			pr_err("port_%d open fail\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
			sensor_info->dev_port, sensor_info->sen_devfd);

	ret = sc132gs_poweron(sensor_info);
	if (ret < 0)
	{
		pr_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return -RET_ERROR;
	}

	ret = sc132gs_reset(sensor_info);
	if (ret < 0)
	{
		pr_err("%d : reset %s fail\n", __LINE__, sensor_info->sensor_name);
		return -RET_ERROR;
	}

	ret = sc132gs_mode_config_init(sensor_info);
	if (ret < 0)
		pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	return ret;
}

static int sc132gs_start(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	uint16_t reg, data;

	setting_size = sizeof(sc132gs_stream_on_setting) / sizeof(uint32_t) / 2;
	pr_info("%s sensor_start setting_size %d\n",
			sensor_info->sensor_name, setting_size);
	for (i = 0; i < setting_size; i++)
	{
		reg = sc132gs_stream_on_setting[i * 2];
		data = sc132gs_stream_on_setting[i * 2 + 1];
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg, data);
		if (ret < 0)
		{
			pr_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		// tmp = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, sc132gs_stream_on_setting[i*2]);
		// printf("start : reg_add = 0x%x, data = 0x%x\n",sc132gs_stream_on_setting[i*2], tmp);
	}

	return ret;
}

static int sc132gs_stop(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	uint16_t reg, data;

	setting_size = sizeof(sc132gs_stream_off_setting) / sizeof(uint32_t) / 2;
	pr_info("%s sensor_stop setting_size %d\n", sensor_info->sensor_name, setting_size);
	for (i = 0; i < setting_size; i++)
	{
		reg = sc132gs_stream_off_setting[i * 2];
		data = sc132gs_stream_off_setting[i * 2 + 1];
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg, data);
		if (ret < 0)
		{
			pr_err("%d : stop %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		// printf("stop:reg_add = 0x%x, data = 0x%x\n",sc132gs_stream_off_setting[i*2], sc132gs_stream_off_setting[i*2 + 1]);
	}

	return ret;
}

static int sc132gs_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int gpio;

	if (sensor_info->power_mode)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] != -1)
			{
				ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					pr_err("camera_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	if (sensor_info->sen_devfd != 0)
	{
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

static int sc132gs_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{

	//    printf(" again[0]:%x, dgain[0]:%x\n",again[0], dgain[0]);
	const uint16_t ANA_GAIN = 0x3e08;
	const uint16_t ANA_FINE_GAIN = 0x3e09;
	const uint16_t DIG_GAIN = 0x3e06;
	const uint16_t DIF_FINE_GAIN = 0x3e07;
	char ana_gain = 0, ana_fine_gain = 0;
	char dig_gain = 0, dig_fine_gain = 0;
	int again_index = 0, dgain_index = 0;
	if (mode == NORMAL_M || mode == DOL2_M)
	{
		if (again[0] >= sizeof(sc132gs_again_lut) / sizeof(uint32_t))
			again_index = sizeof(sc132gs_again_lut) / sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (dgain[0] >= sizeof(sc132gs_dgain_lut) / sizeof(uint32_t))
			dgain_index = sizeof(sc132gs_dgain_lut) / sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		ana_gain = (sc132gs_again_lut[again_index] >> 8) & 0x000000FF;
		ana_fine_gain = sc132gs_again_lut[again_index] & 0x000000FF;

		dig_gain = (sc132gs_dgain_lut[dgain_index] >> 8) & 0x000000FF;
		dig_fine_gain = sc132gs_dgain_lut[dgain_index] & 0x000000FF;
		//		printf("i2c:%d-%x ",info->bus_num,info->sensor_addr);
		camera_i2c_write8(info->bus_num, 16, info->sensor_addr, ANA_GAIN, ana_gain);
		camera_i2c_write8(info->bus_num, 16, info->sensor_addr, ANA_FINE_GAIN, ana_fine_gain);
		camera_i2c_write8(info->bus_num, 16, info->sensor_addr, DIG_GAIN, dig_gain);
		camera_i2c_write8(info->bus_num, 16, info->sensor_addr, DIF_FINE_GAIN, dig_fine_gain);
//		printf(" {%d %d} %02x%02x %02x%02x",again[0], dgain[0], ana_gain, ana_fine_gain, dig_gain, dig_fine_gain);
#if 0
		ana_gain = camera_i2c_read8(info->bus_num, 16, info->sensor_addr, ANA_GAIN);
		ana_fine_gain = camera_i2c_read8(info->bus_num, 16, info->sensor_addr, ANA_FINE_GAIN);
		dig_gain = camera_i2c_read8(info->bus_num, 16, info->sensor_addr, DIG_GAIN);
		dig_fine_gain = camera_i2c_read8(info->bus_num, 16, info->sensor_addr, DIF_FINE_GAIN);
//		printf(" -> %02x%02x %02x%02x\n",ana_gain, ana_fine_gain, dig_gain, dig_fine_gain);
#endif
	}
	else
	{
		pr_err(" unsupport mode %d\n", mode);
	}
	return 0;
}

static int sc132gs_ae_set(uint32_t bus, uint32_t addr, uint32_t line)
{
	const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	const uint16_t S_EXP_LINE0 = 0x3e04;
	const uint16_t S_EXP_LINE1 = 0x3e05;
	char temp0 = 0, temp1 = 0, temp2 = 0;

	uint32_t sline = line;

	temp0 = (sline & 0xF000) >> 12;
	temp1 = (sline & 0xFF0) >> 4;
	temp2 = (sline & 0x0F) << 4;
	camera_i2c_write8(bus, 16, 0x30, EXP_LINE0, temp0);
	camera_i2c_write8(bus, 16, 0x30, EXP_LINE1, temp1);
	camera_i2c_write8(bus, 16, 0x30, EXP_LINE2, temp2);

	camera_i2c_write8(bus, 16, 0x32, EXP_LINE0, temp0);
	camera_i2c_write8(bus, 16, 0x32, EXP_LINE1, temp1);
	camera_i2c_write8(bus, 16, 0x32, EXP_LINE2, temp2);

	return 0;
}

#define SAMPLECNT 8
static uint32_t sc132gs_line_agv(uint32_t line)
{
	uint32_t average, i;
	uint64_t sum = 0;
	static uint32_t sample_ae[SAMPLECNT];
	static uint32_t index = 0;
	sample_ae[index++] = line;
	if (index == SAMPLECNT)
		index = 0;
	for (i = 0; i < SAMPLECNT; i++)
		sum += sample_ae[i];

	average = sum / SAMPLECNT;
	return average;
}

static int sc132gs_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
	uint32_t val;
	if (mode == NORMAL_M)
	{
		val = sc132gs_line_agv(line[0]);
		sc132gs_ae_set(info->bus_num, info->sensor_addr, val);
	}
	else
	{
		pr_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sc132gs_userspace_control(uint32_t port, uint32_t *enable)
{
	trace("enable userspace control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	//*enable = HAL_LINE_CONTROL;
	return 0;
}

sensor_module_t sc132gs = {
	.module = "sc132gs",
	.init = sc132gs_init,
	.start = sc132gs_start,
	.stop = sc132gs_stop,
	.deinit = sc132gs_deinit,
	.power_on = sc132gs_poweron,
	.power_off = sc132gs_poweroff,
	.aexp_gain_control = sc132gs_aexp_gain_control,
	.aexp_line_control = sc132gs_aexp_line_control,
	.userspace_control = sc132gs_userspace_control,
};
