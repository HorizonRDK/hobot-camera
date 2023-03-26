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

#include "inc/hb_cam_utility.h"

#define SIF_IOC_MAGIC 'x'
#define SIF_IOC_PATTERN_CFG  _IOW(SIF_IOC_MAGIC, 10, int)

struct sif_pattern_cfg {
	uint32_t instance;
	uint32_t framerate;
};

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

static int sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	pr_info("sensor_dynamic_switch_fps fps: %d\n", fps);
	int fd_main, ret = RET_OK;
	struct sif_pattern_cfg pattern_fps;

	pattern_fps.framerate = fps;
	pattern_fps.instance = sensor_info->extra_mode;
	fd_main = open("/dev/sif_capture", O_RDWR | O_CLOEXEC, 0666);
	if (fd_main < 0) {
		pr_err("sif main node open failed.\n");
		return -RET_ERROR;
	}

	ret =  ioctl(fd_main, SIF_IOC_PATTERN_CFG, &pattern_fps);
	if (ret < 0) {
		pr_err("SIF_IOC_PATTERN_CFG ioctl failed !\n");
		if (fd_main >= 0) {
			close(fd_main);
			fd_main = -1;
		}
		return ret;
	}
	close(fd_main);
	fd_main = -1;
	pr_info("sensor_dynamic_switch_fps end\n");
	return RET_OK;
}

sensor_module_t virtual = {
	.module = "virtual",
	.init = sensor_init,
	.deinit = sensor_deinit,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
};

