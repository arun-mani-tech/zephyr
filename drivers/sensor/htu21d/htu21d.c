/* HTU21D temperature & humidity sensor with i2c
 *
 * Copyright (c) 2022 Linumiz
 * Author: Arunmani <arunmani@linumiz.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT te_htu21d
#include <zephyr.h>
#include <stdio.h>
#include "htu21d.h"
#include <logging/log.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/sensor.h> 
#include <drivers/i2c.h>
#define SIXTEEN_BIT	65536.0     
#define TEMPERATURE_COEFF_MUL	175.72
#define TEMPERATURE_COEFF_ADD	-46.85
#define HUMIDITY_COEFF_MUL	125.0
#define HUMIDITY_COEFF_ADD	-6.0
#define DECIMAL		10000
#define MICRO	100

LOG_MODULE_REGISTER(htu21d, CONFIG_SENSOR_LOG_LEVEL);

struct htu21d_data {
	const struct device *i2c;
	uint16_t temp;
	int16_t humidity;
};

static int send_device_cmd(struct htu21d_data *drv_data, uint16_t *val, const char cmd)
{
	int ret;
	int counter = 0;
	uint8_t value[3];
	value[0] = cmd;
		
		/*to write register address*/
	if (i2c_write(drv_data->i2c, &value[0], 1, HTU21D_I2CADDR) != 0) {
		LOG_ERR("Error writing to HTU21D! (0x%X)\n", value[0]);
	}

		/*read values*/
	if (i2c_read(drv_data->i2c, &value, 3, HTU21D_I2CADDR) != 0) { 
		LOG_ERR("Retry! (0x%X)....\n",value[0]);
	}

	if(IS_ENABLED(CONFIG_HTU21D_NH_MASTER)) {
		k_sleep(K_MSEC(MEASURE_WAIT_MS));
	}
	
	*val = (value[0] << 8 | value[1]) & 0xFFFC;
	
	return 0;
}

static int htu21d_sample_fetch(const struct device *dev,
		enum sensor_channel chan)
{
	struct htu21d_data *drv_data = dev->data;
	uint16_t value,value2;
	drv_data->temp = 0U;
	drv_data->humidity = 0U;

	if(chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_HUMIDITY) {
                return -ENOTSUP;
        } 
	
	switch (chan) {
        case SENSOR_CHAN_AMBIENT_TEMP:
			/* Trigger Temperature Measurement*/
		if (send_device_cmd(drv_data, &value,
					(IS_ENABLED(CONFIG_HTU21D_NH_MASTER)
					? HTU21D_READ_TEMP_NH : 
					HTU21D_READ_TEMP_H)) != 0) {
			return -EIO;
		}

		drv_data->temp = value; 
		break;
	case SENSOR_CHAN_HUMIDITY:
			/* Trigger Humidity Measurement*/
		if (send_device_cmd(drv_data, &value,(IS_ENABLED(CONFIG_HTU21D_NH_MASTER)
					? HTU21D_READ_HUMID_NH : 
					HTU21D_READ_HUMID_H)) != 0) {
			return -EIO;
		}
	
		drv_data->humidity = value;
		break;
	}
	
	return 0;
}

static int htu21d_channel_get(const struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	struct htu21d_data *drv_data = dev->data;
	int32_t temp_val,humidity_val;
	double sensor_tmp_val,sensor_humidity_val;
	double result;
	float float_val;
	temp_val = drv_data->temp;
	sensor_tmp_val = temp_val/SIXTEEN_BIT;
	humidity_val = drv_data->humidity;
	sensor_humidity_val = humidity_val/SIXTEEN_BIT;

	if(chan != SENSOR_CHAN_AMBIENT_TEMP && chan != SENSOR_CHAN_HUMIDITY) {
		return -ENOTSUP;
	}

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*temperature calc*/
		/* Temp = -46.85 + 175.72 * temp_val/2^16 */
		result = TEMPERATURE_COEFF_ADD  + (TEMPERATURE_COEFF_MUL * sensor_tmp_val);
		val->val1 = (int)result;
		float_val = (result-(int)result)*DECIMAL;
		val->val2 = (int)float_val/MICRO;
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*humidity calc*/
		/* RH = -6 + 125 * humidity_val/2^16 */
		result = HUMIDITY_COEFF_ADD + (HUMIDITY_COEFF_MUL * sensor_humidity_val);
		val->val1 = (int)result;
		float_val = (result-(int)result)*DECIMAL;
		val->val2 = (int)float_val/MICRO;
		break;
	}	
	
	return 0;
}

static const struct sensor_driver_api htu21d_driver_api = {
	.sample_fetch = htu21d_sample_fetch,
	.channel_get = htu21d_channel_get,
};

static int htu21d_init(const struct device *dev)
{
	struct htu21d_data *drv_data = dev->data;

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));

	if (drv_data->i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
				DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	/*HTU21D reset*/
	i2c_write(drv_data->i2c, HTU21D_SOFT_RESET, 1, HTU21D_I2CADDR);
	k_sleep(K_MSEC(STARTUP_WAIT_MS));

	return 0;
}
static struct htu21d_data htu21d_drv_data;
DEVICE_DT_INST_DEFINE(0, htu21d_init, NULL, &htu21d_drv_data, NULL, POST_KERNEL,
		CONFIG_SENSOR_INIT_PRIORITY, &htu21d_driver_api);
