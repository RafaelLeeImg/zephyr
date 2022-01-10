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
		val->val2 = (drv_data->humidity) * 1000000LL / (1 << 20);
		break;
	case SENSOR_CHAN_AMBIENT_TEMP:
		temperature = (float)drv_data->temperature / (1 << 20) * 200 - 50.0;
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

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!", DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	k_busy_wait(40000); // wait for 40ms

	int ret = 0;
	aht20_status sensor_status = { 0 };

	{
		static uint8_t const inquire_status_seq[] = { AHT20_CMD_GET_STATUS };
		ret = i2c_write(drv_data->i2c, inquire_status_seq, sizeof(inquire_status_seq),
				DT_INST_REG_ADDR(0));
		if (0 != ret) {
			LOG_ERR("Failed to inquire status");
		}

		ret = i2c_read(drv_data->i2c, &sensor_status.all, AHT20_STATUS_LENGTH,
			       DT_INST_REG_ADDR(0));
		if (!sensor_status.cal_enable) {
			static uint8_t const initialize_seq[] = { AHT20_CMD_INITIALIZE };
			ret = i2c_write(drv_data->i2c, initialize_seq, sizeof(initialize_seq),
					DT_INST_REG_ADDR(0));
			if (0 != ret) {
				LOG_ERR("Failed to inquire status");
			}
			k_busy_wait(10000); // wait for 10ms
		}
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

	int rc = i2c_write(drv_data->i2c, tx_buf, sizeof(tx_buf), DT_INST_REG_ADDR(0));

	if (rc < 0) {
		return -EIO;
	}

	aht20_status sensor_status = { 0 };

	// tested with AHT20, 40ms is enough for measuring, datasheet said wait 80ms
	k_usleep(40000);
	bool wait = false;

	do {
		if (wait) {
			k_usleep(5000); // skip waiting for the first time, less kernel call involve
		}
		wait = true;
		rc = i2c_read(drv_data->i2c, rx_buf, AHT20_STATUS_LENGTH + AHT20_READ_LENGTH,
			      DT_INST_REG_ADDR(0));
		if (rc < 0) {
			return -EIO;
		}
		sensor_status.all = rx_buf[0];
	} while (sensor_status.busy);

	drv_data->humidity = (rx_buf[1] << 12) + (rx_buf[2] << 4) + ((rx_buf[3] & 0xf0) >> 4);
	drv_data->temperature = ((rx_buf[3] & 0xf) << 16) + (rx_buf[4] << 8) + rx_buf[5];

	uint8_t crc = crc8(rx_buf, 6, AHT20_CRC_POLY, AHT20_CRC_INIT, false);

	if (crc != rx_buf[6]) {
		return -EIO;
	}
	return 0;
}

static struct aht20_data aht20_driver;

static const struct sensor_driver_api aht20_driver_api = {
	.sample_fetch = aht20_sample_fetch,
	.channel_get = aht20_channel_get,
};

DEVICE_DT_INST_DEFINE(0, &aht20_init, NULL, &aht20_driver, NULL, POST_KERNEL,
		      CONFIG_SENSOR_INIT_PRIORITY, &aht20_driver_api);
