/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/i2c-dev.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "inc/hb_cam_utility.h"
#include "inc/hb_i2c.h"
#include "inc/imx586_setting.h"
#include "inc/sensor_effect_common.h"
#include "cam/hb_vin.h"

enum {
    MODE_LINEAR_3840x2160 = 0,
    MODE_LINEAR_4000x3000,
    MODE_LINEAR_4000x2250,

};

enum {
    MODE_HDR_3840x2160 = 0,
    MODE_HDR_3840x2160_RATIO_1,
    MODE_HDR_3840x2160_RATIO_2,
    MODE_HDR_3840x2160_RATIO_4,
    MODE_HDR_3840x2160_RATIO_8,
    MODE_HDR_4000x3000,
    MODE_HDR_4000x3000_RATIO_1,
    MODE_HDR_4000x3000_RATIO_2,
    MODE_HDR_4000x3000_RATIO_4,
    MODE_HDR_4000x3000_RATIO_8,
};

#define TUNING_LUT

static int sensor_reset(sensor_info_t *sensor_info)
{
    int gpio, ret = RET_OK;
    if (sensor_info->power_mode) {
        for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
            if (sensor_info->gpio_pin[gpio] >= 0) {
                pr_debug("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,
                         sensor_info->gpio_pin[gpio],
                         sensor_info->gpio_level[gpio],
                         sensor_info->gpio_level[gpio]);
                ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
                usleep(sensor_info->power_delay * 1000);
                ret |= camera_power_ctrl(sensor_info->gpio_pin[gpio],
                                         1 - sensor_info->gpio_level[gpio]);
                if (ret < 0) {
                    pr_err("camera_power_ctrl fail\n");
                    return -HB_CAM_SENSOR_POWERON_FAIL;
                }
            }
        }
    }
    return ret;
}

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
}
static int sensor_param_init(sensor_info_t *sensor_info,
                             sensor_turning_data_t *turning_data)
{
    int ret = RET_OK;

    turning_data->sensor_data.active_width = 3840;
    turning_data->sensor_data.active_height = 2160;
    turning_data->sensor_data.analog_gain_max = 160 * 8192;
    turning_data->sensor_data.digital_gain_max = 128 * 8192;
    turning_data->sensor_data.exposure_time_min = 2;
    turning_data->sensor_data.exposure_time_max = 0xfffc;
    turning_data->sensor_data.exposure_time_long_max = 0xfffc;
    turning_data->sensor_data.lines_per_second = 91996; // 103.2M/8976
    turning_data->sensor_data.turning_type = 6;         // gain calc
    turning_data->sensor_data.fps = sensor_info->fps;   // fps
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
    if (size >= sizeof(imx586_stream_on_setting)) {
        memcpy(stream_on, imx586_stream_on_setting,
               sizeof(imx586_stream_on_setting));
    } else {
        pr_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }
    size = sizeof(turning_data->stream_ctrl.stream_off);
    if (size >= sizeof(imx586_stream_off_setting)) {
        memcpy(stream_off, imx586_stream_off_setting,
               sizeof(imx586_stream_off_setting));
    } else {
        pr_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }
    return ret;
}
static int sensor_turning_data_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    uint32_t open_cnt = 0;
    sensor_turning_data_t turning_data;
    uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
    uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

    memset(&turning_data, 0, sizeof(sensor_turning_data_t));
    sensor_common_data_init(sensor_info, &turning_data);
    sensor_param_init(sensor_info, &turning_data);

    turning_data.normal.param_hold = IMX586_PARAM_HOLD;
    turning_data.normal.param_hold_length = 2;
    turning_data.normal.s_line = IMX586_LINE;
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
    turning_data.normal.again_control[0] = IMX586_GAIN;
    turning_data.normal.again_control_length[0] = 2;

    turning_data.normal.dgain_control_num = 1;
    turning_data.normal.dgain_control[0] = IMX586_DGAIN;
    turning_data.normal.dgain_control_length[0] = 2;
    turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
    turning_data.normal.dgain_lut = malloc(256 * sizeof(uint32_t));

    if (turning_data.normal.again_lut != NULL &&
        turning_data.normal.dgain_lut != NULL) {
        memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
        memset(turning_data.normal.dgain_lut, 0xff, 256 * sizeof(uint32_t));

        memcpy(turning_data.normal.again_lut, imx586_again_lut,
               sizeof(imx586_again_lut));
        for (open_cnt = 0; open_cnt <
             sizeof(imx586_again_lut) / sizeof(uint32_t);
             open_cnt++) {
            DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
        }
        memcpy(turning_data.normal.dgain_lut, imx586_dgain_lut,
               sizeof(imx586_dgain_lut));
        for (open_cnt = 0; open_cnt <
             sizeof(imx586_dgain_lut) / sizeof(uint32_t);
             open_cnt++) {
            DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
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

    if (turning_data.normal.dgain_lut) {
        free(turning_data.normal.dgain_lut);
        turning_data.normal.dgain_lut = NULL;
    }

    return ret;
}

static int sensor_config_fps(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int frame_line = 0;
    if (sensor_info->fps > 30 || sensor_info->fps < 5) {
        pr_err("fps range 5-30fps, not support %d fps\n", sensor_info->fps);
        return -RET_ERROR;
    } else {
        pr_info("IMX586 FPS IS %d\n", sensor_info->fps);
        frame_line = DEFAULT_FPS_REGVALUE * DEFAULT_FPS / sensor_info->fps;
        ret = hb_i2c_write_reg16_data16(sensor_info->bus_num,
                                        sensor_info->sensor_addr, IMX586_VTS, frame_line);
        if (ret < 0) {
            pr_err("[%s]%s err!\n", __func__, __LINE__);
            ret = -RET_ERROR;
        }
    }
    pr_info("imx586 config %dfps success\n", sensor_info->fps);
    return ret;
}

static int sensor_mode_config_i2c_write(sensor_info_t *sensor_info, uint16_t *pbuf, size_t size)
{
    int ret = RET_OK;
    int setting_size = 0, i;

    setting_size = size / sizeof(uint16_t) / 2;
    pr_info("imx586 setting_size %d\n", setting_size);
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

static int sensor_mode_config_init_ratio(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    uint16_t *init_setting = NULL;

    switch (sensor_info->extra_mode) {
    case MODE_HDR_3840x2160_RATIO_1:
    case MODE_HDR_4000x3000_RATIO_1:
        init_setting = imx586_init_hdr_ratio_1_setting;
        break;
    case MODE_HDR_3840x2160_RATIO_2:
    case MODE_HDR_4000x3000_RATIO_2:
        init_setting = imx586_init_hdr_ratio_2_setting;
        break;
    case MODE_HDR_3840x2160_RATIO_4:
    case MODE_HDR_4000x3000_RATIO_4:
        init_setting = imx586_init_hdr_ratio_4_setting;
        break;
    case MODE_HDR_3840x2160_RATIO_8:
    case MODE_HDR_4000x3000_RATIO_8:
        init_setting = imx586_init_hdr_ratio_8_setting;
        break;
    default:
        return RET_OK;
    }

    return sensor_mode_config_i2c_write(sensor_info, init_setting, sizeof(init_setting));
}

static int sensor_mode_config_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;

    switch (sensor_info->sensor_mode) {
    case NORMAL_M:
        if (sensor_info->extra_mode == MODE_LINEAR_3840x2160) {
            if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_init_3840x2160_setting, sizeof(imx586_init_3840x2160_setting))) < 0)
                return ret;
        } else if (sensor_info->extra_mode == MODE_LINEAR_4000x3000) {
            if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_init_4000x3000_setting, sizeof(imx586_init_4000x3000_setting))) < 0)
                return ret;
        } else if (sensor_info->extra_mode == MODE_LINEAR_4000x2250) {
            if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_init_4000x2250_setting, sizeof(imx586_init_4000x2250_setting))) < 0)
                return ret;
        } else {
            pr_err("linear extra mode is err, extra_mode:%d\n", sensor_info->extra_mode);
            return -RET_ERROR;
        }
        break;
    case DOL2_M:
    case DOL3_M:
        if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_init_default_hdr_setting, sizeof(imx586_init_default_hdr_setting))) < 0)
            return ret;
        switch (sensor_info->extra_mode) {
        case MODE_HDR_3840x2160:
        case MODE_HDR_3840x2160_RATIO_1:
        case MODE_HDR_3840x2160_RATIO_2:
        case MODE_HDR_3840x2160_RATIO_4:
        case MODE_HDR_3840x2160_RATIO_8:
            if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_init_3840x2160_hdr_setting, sizeof(imx586_init_3840x2160_hdr_setting))) < 0)
                return ret;
            if ((ret = sensor_mode_config_init_ratio(sensor_info)) < 0)
                return ret;
            break;
        case MODE_HDR_4000x3000:
        case MODE_HDR_4000x3000_RATIO_1:
        case MODE_HDR_4000x3000_RATIO_2:
        case MODE_HDR_4000x3000_RATIO_4:
        case MODE_HDR_4000x3000_RATIO_8:
            if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_init_4000x3000_hdr_setting, sizeof(imx586_init_4000x3000_hdr_setting))) < 0)
                return ret;
            if ((ret = sensor_mode_config_init_ratio(sensor_info)) < 0)
                return ret;
            break;
        default:
            pr_err("dol extra mode is err, extra_mode:%d\n", sensor_info->extra_mode);
            return -RET_ERROR;
        }
        break;
    default:
        pr_err("config mode is err, sensor_mode:%d\n", sensor_info->sensor_mode);
        return -RET_ERROR;
    }

    if (sensor_info->fps != DEFAULT_FPS) {
        if ((ret = sensor_config_fps(sensor_info)) < 0) {
            pr_err("%d : imx586 config fps %s fail\n", __LINE__,
                   sensor_info->sensor_name);
            return ret;
        }
    }

    if ((ret = sensor_turning_data_init(sensor_info)) < 0) {
        pr_err("sensor_turning_data_init %s fail\n", sensor_info->sensor_name);
        return ret;
    }

    pr_info("IMX586_config OK!\n");
    return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    char str[12] = {0};

    snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
    if (sensor_info->sen_devfd <= 0) {
        if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
            pr_err("port_%d open fail\n", sensor_info->port);
            return -RET_ERROR;
        }
    }
    pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
            sensor_info->dev_port, sensor_info->sen_devfd);

    if ((ret = sensor_reset(sensor_info)) < 0) {
        pr_err("%d : reset %s fail\n", __LINE__, sensor_info->sensor_name);
        return -RET_ERROR;
    }

    if ((ret = sensor_mode_config_init(sensor_info)) < 0)
        pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);

    return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
    int ret = RET_OK;

    if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_stream_on_setting, sizeof(imx586_stream_on_setting))) < 0)
        pr_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);

    return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
    int ret = RET_OK;

    if ((ret = sensor_mode_config_i2c_write(sensor_info, imx586_stream_off_setting, sizeof(imx586_stream_off_setting))) < 0)
        pr_err("%d : stop %s fail\n", __LINE__, sensor_info->sensor_name);

    return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int gpio;

    if (sensor_info->power_mode) {
        for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
            if (sensor_info->gpio_pin[gpio] != -1) {
                ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
                if (ret < 0) {
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

static const uint32_t max_pos = 10;
static uint32_t last_pos = 0;
static uint32_t move_start_flag = 0;

static int sensor_af_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
    if (move_start_flag) {
        if ((last_pos > pos) && (last_pos > max_pos)) {
            if ((pos + max_pos) < last_pos) {
                pos = last_pos - max_pos;
            }
        }
    } else {
        move_start_flag = 1;
    }
    // printf(" af pos %d ! \n", pos);
    uint32_t temp = 512;
    temp = temp + pos;
    hb_i2c_write_reg8_data16(2, 0x0c, 0x03, temp);
    last_pos = pos;

    return 0;
}

static int sensor_zoom_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
    printf(" zoom ! \n");
    return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
    *enable = HAL_AF_CONTROL;
    return 0;
}

sensor_module_t imx586 = {
    .module = "imx586",
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
