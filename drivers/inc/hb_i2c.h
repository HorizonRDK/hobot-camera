/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_I2C_H__
#define __HB_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_FD_NUMBER 10

int hb_i2c_init(int bus);
int hb_i2c_deinit(int bus);

int hb_i2c_read_reg16_data16(int fd, uint8_t i2c_addr, uint16_t reg_addr);
int hb_i2c_read_reg16_data8(int fd, uint8_t i2c_addr, uint16_t reg_addr);
int hb_i2c_read_reg8_data8(int fd, uint8_t i2c_addr, uint8_t reg_addr);
int hb_i2c_read_reg8_data16(int fd, uint8_t i2c_addr, uint8_t reg_addr);
int hb_i2c_write_reg16_data16(int fd, uint8_t i2c_addr,
		uint16_t reg_addr, uint16_t value);
int hb_i2c_write_reg16_data8(int fd, uint8_t i2c_addr,
		uint16_t reg_addr, uint8_t value);
int hb_i2c_write_reg8_data16(int fd, uint8_t i2c_addr,
		uint8_t reg_addr, uint16_t value);
int hb_i2c_write_reg8_data8(int fd, uint8_t i2c_addr,
		uint8_t reg_addr, uint8_t value);
int hb_i2c_write_block(int fd, uint8_t i2c_addr,
		uint16_t reg_addr, uint32_t value, uint8_t cnt);
int hb_i2c_read_block_reg16(int fd, uint8_t i2c_addr,
			uint16_t reg_addr, char *buf, uint32_t count);
int hb_i2c_read_block_reg8(int fd, uint8_t i2c_addr,
		uint16_t reg_addr, char *buf, uint32_t count);
int hb_i2c_write_block_reg16(int fd, uint8_t i2c_addr,
		uint16_t reg_addr, char *buf, uint32_t count);
int hb_i2c_write_block_reg8(int fd,	uint8_t i2c_addr,
		uint16_t reg_addr, char *buf, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif
