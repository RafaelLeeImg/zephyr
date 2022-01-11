/*
 * Copyright (c) 2022 Rafael Lee <rafaellee.img@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AHT20_AHT20_H_
#define ZEPHYR_DRIVERS_SENSOR_AHT20_AHT20_H_

#include <device.h>
#include <sys/util.h>

#define AHT20_STATUS_LENGTH 1
#define AHT20_READ_LENGTH 6
#define AHT20_CRC_POLY 0x31 // polynomial 1 + x^4 + x^5 + x^8
#define AHT20_CRC_INIT 0xff

#define AHT20_CMD_RESET 0xBA
#define AHT20_CMD_TRIGGER_MEASURE 0xAC
#define AHT20_TRIGGER_MEASURE_BYTE_0 0x33
#define AHT20_TRIGGER_MEASURE_BYTE_1 0x00
#define AHT20_CMD_GET_STATUS 0x71
#define AHT20_CMD_INITIALIZE 0xBE

#define AHT20_FULL_RANGE_BITS 20
#define AHT20_TEMPERATURE_RANGE 200
#define AHT20_TEMPERATURE_OFFSET 50.0

typedef union {
	struct {
		uint8_t : 3;            // bit [0:2]
		uint8_t cal_enable : 1; // bit [3]
		uint8_t : 1;            // bit [4]
		uint8_t : 2;            // bit [5:6], AHT20 datasheet v1.1 removed 2 mode bits
		uint8_t busy : 1;       // bit [7]
	};
	uint8_t all;
} __attribute__((__packed__)) aht20_status;

struct aht20_data {
	struct i2c_dt_spec bus;
	uint32_t humidity;
	uint32_t temperature;
};

static int aht20_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_AHT20_AHT20_H_ */
