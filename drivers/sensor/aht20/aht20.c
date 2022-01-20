/*
 * Copyright (c) 2022 Rafael Lee <rafaellee.img@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT asair_aht20

#include <kernel.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <sys/crc.h>
#include <sys/byteorder.h>

#include "aht20.h"

LOG_MODULE_REGISTER(AHT20, CONFIG_SENSOR_LOG_LEVEL);

static int aht20_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct aht20_data *drv_data = dev->data;
	float temperature;

	switch (chan) {
	case SENSOR_CHAN_HUMIDITY:
		val->val1 = 0;
		val->val2 = (drv_data->humidity) * 1000000LL / (1 << AHT20_FULL_RANGE_BITS);
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		temperature = (float)drv_data->temperature / (1 << AHT20_FULL_RANGE_BITS) *
			      AHT20_TEMPERATURE_RANGE -
			      AHT20_TEMPERATURE_OFFSET;
		val->val1 = (int32_t)temperature;
		val->val2 = (temperature - val->val1) * 1000000LL;
		break;
	default:
		break;
	}
	return 0;
}

static int aht20_init(const struct device *dev)
{
	struct aht20_data *drv_data = dev->data;

	k_sleep(K_MSEC(40)); // wait for 40ms

	int ret = 0;
	aht20_status sensor_status = { 0 };

	static uint8_t const inquire_status_seq[] = { AHT20_CMD_GET_STATUS };

	ret = i2c_write_dt(&drv_data->bus, inquire_status_seq, sizeof(inquire_status_seq));
	if (0 != ret) {
		LOG_ERR("Failed to inquire status");
	}

	ret = i2c_read_dt(&drv_data->bus, &sensor_status.all, AHT20_STATUS_LENGTH);
	if (!sensor_status.cal_enable) {
		static uint8_t const initialize_seq[] = { AHT20_CMD_INITIALIZE };
		ret = i2c_write_dt(&drv_data->bus, initialize_seq, sizeof(initialize_seq));
		if (0 != ret) {
			LOG_ERR("Failed to inquire status");
		}
		k_busy_wait(10000); // wait for 10ms
	}

	return ret;
}

static int aht20_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct aht20_data *drv_data = dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP ||
			chan == SENSOR_CHAN_HUMIDITY);

	uint8_t tx_buf[] = { AHT20_CMD_TRIGGER_MEASURE, AHT20_TRIGGER_MEASURE_BYTE_0,
			     AHT20_TRIGGER_MEASURE_BYTE_1 };
	uint8_t rx_buf[7] = { 0 };

	int rc = i2c_write_dt(&drv_data->bus, tx_buf, sizeof(tx_buf));

	if (rc < 0) {
		return -EIO;
	}

	aht20_status sensor_status = { 0 };

	// tested with AHT20, 40ms is enough for measuring, datasheet said wait 80ms
	k_sleep(K_MSEC(40));
	bool wait = false;

	do {
		if (wait) {
			k_sleep(K_MSEC(5)); // skip waiting for the first time, less kernel call involve
		}
		wait = true;
		rc = i2c_read_dt(&drv_data->bus, rx_buf, AHT20_STATUS_LENGTH + AHT20_READ_LENGTH);
		if (rc < 0) {
			return -EIO;
		}
		sensor_status.all = rx_buf[0];
	} while (sensor_status.busy);

	// the read 7 bytes contains the following data:
	// status: 8 bits
	// humidity: 20 bits
	// temperature: 20 bits
	// crc8: 8 bits
	drv_data->humidity = sys_get_be24(rx_buf + 1);
	drv_data->humidity >>= 24 - AHT20_FULL_RANGE_BITS;
	drv_data->temperature = sys_get_be24(rx_buf + 3);
	drv_data->temperature &= (1L << AHT20_FULL_RANGE_BITS) - 1;

	uint8_t crc = crc8(rx_buf, AHT20_STATUS_LENGTH + AHT20_READ_LENGTH - 1, AHT20_CRC_POLY,
			   AHT20_CRC_INIT, false);

	if (crc != rx_buf[AHT20_STATUS_LENGTH + AHT20_READ_LENGTH - 1]) {
		return -EIO;
	}
	return 0;
}

static const struct sensor_driver_api aht20_driver_api = {
	.sample_fetch = aht20_sample_fetch,
	.channel_get = aht20_channel_get,
};

#define AHT20_INIT(n)									\
	static struct aht20_data aht20_data_##n = {					\
		.bus = I2C_DT_SPEC_INST_GET(n),						\
	};										\
	DEVICE_DT_INST_DEFINE(n, &aht20_init, NULL, &aht20_data_##n, NULL, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &aht20_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AHT20_INIT)
