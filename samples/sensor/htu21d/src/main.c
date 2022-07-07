/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/printk.h>

static void do_main(const struct device *dev)
{
	int ret;
	struct sensor_value temp_value;
	struct sensor_value hum_value;
	
	while (1) {
			/*temperature*/
		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return;
		}
		
		ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		if (ret) {
			printk("sensor_channel_get failed ret %d\n", ret);
			return;
		}
		
		printk("temp is %d (%d micro)\n", temp_value.val1,
					       temp_value.val2);
		k_sleep(K_MSEC(1000));

			/*humidity*/
		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_HUMIDITY);
                if (ret) {
                        printk("sensor_sample_fetch failed ret %d\n", ret);
                        return;
                }

		ret = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum_value);
                if (ret) {
                        printk("sensor_channel_get failed ret %d\n", ret);
                        return;
                }
                
		printk("humidity is %d (%d micro)\n", hum_value.val1,
                       				hum_value.val2);
                k_sleep(K_MSEC(1000));
	}
}

void main(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(te_htu21d);
	
	if(dev == NULL) {
		printk("Failed to get the device\n");
		return;
	}
	
	do_main(dev);
}
