/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#define GPIO_PIN_0 0

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printf("button pressed\r\n");
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	int ret;
	const struct device *gpio_leds_dev;

	gpio_leds_dev = device_get_binding("axi_gpio@41210000");
	if (gpio_leds_dev == NULL) {
		printf("Failed to bind to AXI GPIO device\n");
		return;
	}
	ret = gpio_pin_configure(gpio_leds_dev, GPIO_PIN_0, GPIO_OUTPUT);
	if (ret != 0) {
		printf("Failed to configure GPIO pin %d\n", GPIO_PIN_0);
		return;
	}
	ret = gpio_pin_set(gpio_leds_dev, GPIO_PIN_0, 1);
	if (ret != 0) {
		printf("Failed to set GPIO pin %d\n", GPIO_PIN_0);
	}


	for (int i = 0; i < 50; i++) {
		int val = 0;
		printf("toggle btn = 0x%x\r\n", val);
		gpio_pin_toggle(gpio_leds_dev, GPIO_PIN_0);
		k_msleep(500);
	}

	printf("test ended.\r\n");
	return 0;
}
