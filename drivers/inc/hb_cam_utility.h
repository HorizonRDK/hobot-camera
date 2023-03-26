/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_CAM_UTILITY_H__
#define __HB_CAM_UTILITY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/ipc.h>
#include <sys/shm.h>
#include <pthread.h>
#include <errno.h>

#include "cam/cam_common.h"
#define GPIO_HIGH	1
#define GPIO_LOW	0
#define REG_WIDTH_16bit 16
#define REG_WIDTH_8bit 8
#define I2C_BUS  0
#define SPI_BUS  1

#define ENTRY0_KEY 111
#define ENTRY1_KEY 222
#define ENTRY2_KEY 333
#define ENTRY3_KEY 444
#define YUV_RAW_KEY 555

#define CAMERA_IOC_MAGIC   'x'
#define SENSOR_TURNING_PARAM   _IOW(CAMERA_IOC_MAGIC, 0, sensor_turning_data_t)
#define SENSOR_TURNING_PARAM_EX   _IOW(CAMERA_IOC_MAGIC, 7, sensor_turning_data_ex_t)
#define SENSOR_GPIO_CONTROL  _IOW(CAMERA_IOC_MAGIC, 10, gpio_info_t)

enum CAMERA_STATUS{
	CAM_INIT = 1,
	CAM_DEINIT,
	CAM_START,
	CAM_STOP,
	CAM_POWERON,
	CAM_POWEROFF
};

typedef struct _x2_camera_i2c_t {
	uint32_t i2c_addr;
	uint32_t reg_size;
	uint32_t reg;
	uint32_t data;
} x2_camera_i2c_t;

typedef struct _x2_camera_spi_t {
	uint64_t tx_buf;
	uint64_t rx_buf;
	uint32_t reg;
	uint32_t reg_size;
	uint32_t len;
	uint32_t data;
	uint32_t speed_hz;
	uint32_t bits_per_word;
} x2_camera_spi_t;

inline int camera_create_flag(int index_of_key, unsigned int length)
{
	char *file_path = "/etc";
	key_t key = ftok(file_path, index_of_key);

	int shmid = shmget(key, length, IPC_CREAT | 0666);
	return shmid;
}

inline mutex_package_t* camera_create_mutex_package(int index)
{
	int shmid = camera_create_flag(index, sizeof(mutex_package_t));
	mutex_package_t *mp = (mutex_package_t*)shmat(shmid, NULL, SHM_R | SHM_W);
	if(mp == NULL) {
		return NULL;
	}
	mp->shmid = shmid;
	// Initialize the lock state and set the state to -- process sharing
	pthread_mutexattr_init(&(mp->lock_attr));
	pthread_mutexattr_setpshared(&(mp->lock_attr), PTHREAD_PROCESS_SHARED);
	// Initialize lock with lock state
	pthread_mutex_init(&(mp->lock), &(mp->lock_attr));
	return mp;
}

inline int camera_destory_mutex_package(mutex_package_t *mp)
{
    if (mp) {
		pthread_mutex_destroy(&(mp->lock));
		pthread_mutexattr_destroy(&(mp->lock_attr));
		#if 0
		ret = shmctl(mp->flag, IPC_RMID, NULL);
        HANDLE_RET_ERR(ret, "shmctl");
		#endif
		shmdt(mp);
        mp = NULL;
    }
    return 0;
}

typedef struct camera_gpio_info_t {
	uint32_t gpio;
	uint32_t gpio_level;
} gpio_info_t;

extern void DOFFSET(uint32_t *x, uint32_t n);
extern int camera_power_ctrl(uint32_t gpio, uint32_t on_off);
extern int camera_read_reg(uint16_t reg, uint8_t *val);
extern int camera_write_reg(uint16_t reg, uint8_t val);
extern int camera_write_block(uint16_t reg, uint32_t val, int conti_cnt);
extern int camera_write_addr16_reg16(uint16_t addr, uint16_t value);
extern int camera_read_reg16_data16(int bus, int i2c_addr, uint16_t reg_addr);
extern int camera_i2c_read16(int bus, int reg_width, uint8_t sensor_addr,
			uint32_t reg_addr);
extern int camera_i2c_read8(int bus, int reg_width, uint8_t sensor_addr,
			uint32_t reg_addr);
extern int camera_i2c_write16(int bus, int reg_width, uint8_t sensor_addr,
			uint32_t reg_addr, uint16_t value);
extern int camera_i2c_write8(int bus, int reg_width, uint8_t sensor_addr,
			uint32_t reg_addr, uint8_t value);
extern int camera_i2c_write_block(int bus, int reg_width, int device_addr,
			uint32_t reg_addr, char *buffer, uint32_t size);
extern int camera_i2c_read_block(int bus, int reg_width, int device_addr,
			uint32_t reg_addr, char *buffer, uint32_t size);
extern int camera_write_array(int bus, uint32_t i2c_addr, int reg_width,
			int setting_size, uint32_t *cam_setting);
extern int hb_deserial_init(deserial_info_t *deserial_info);
extern int hb_deserial_deinit(deserial_info_t *deserial_info);
extern int32_t hb_cam_utility(int32_t cam_ctl, sensor_info_t *sensor_info);
extern int32_t hb_cam_utility_ex(int32_t cam_ctl, sensor_info_t *sensor_info);
extern int hb_deserial_start_physical(deserial_info_t *deserial_info);
extern int hb_cam_ae_share_init(uint32_t port, uint32_t flag);
extern int camera_spi_write_block(int bus, char *buffer, int size);
extern int camera_spi_read_block(int bus, char *buffer, int size);
extern int camera_wait_init(uint32_t port, int time);
int camera_init_utility(sensor_info_t *sensor_info);
void camera_deinit_utility(sensor_info_t *sensor_info);
int camera_start_utility(sensor_info_t *sensor_info);
int camera_stop_utility(sensor_info_t *sensor_info);
int hb_camera_sensorlib_init(sensor_info_t *sensor_info);

//extern int camera_i2c_write_block(sensor_info_t *sensor_info, int subdev, char *buffer, int size)

#ifdef __cplusplus
}
#endif
#endif
