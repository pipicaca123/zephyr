/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/sys/util.h>

#define GPIO_PIN_0 0

#define UART0_DEVICE_NODE DT_NODELABEL(uart0)
#define UARTLITE_DEVICE_NODE DT_NODELABEL(uartlite0)
#define MSG_SIZE 32
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const uart0_dev = DEVICE_DT_GET(UART0_DEVICE_NODE);
static const struct device *const uartlite0_dev = DEVICE_DT_GET(UARTLITE_DEVICE_NODE);
static int rx_buf_pos;

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	if (!uart_irq_update(uart0_dev)) {
		return;
	}
	if (!uart_irq_rx_ready(uart0_dev)) {
		return;
	}
	/* read until FIFO empty */
	while (uart_fifo_read(uart0_dev, &c, 1) == 1) {
		printf("%c",c);
	}
}
void serial_lite_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	if (!uart_irq_update(uartlite0_dev)) {
		return;
	}
	if (!uart_irq_rx_ready(uartlite0_dev)) {
		return;
	}
	/* read until FIFO empty */
	print_uart("echo: ");
	while (uart_fifo_read(uartlite0_dev, &c, 1) == 1) {
		print_uart("%c",c);
	}
	print_uart("\r\n");
}
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uartlite0_dev, buf[i]);
	}
}


void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printf("button pressed\r\n");
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	int ret;
	const struct device *gpio_leds_dev;
	const struct device *gpio_btns_dev;
	struct gpio_callback button_cb_data;

	//gpio
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

	gpio_btns_dev = device_get_binding("axi_gpio@41200000");
	if (gpio_btns_dev == NULL) {
		printf("Failed to bind to AXI GPIO device\n");
		return;
	}
	ret = gpio_pin_configure(gpio_btns_dev, GPIO_PIN_0, GPIO_INPUT);
	if (ret != 0) {
		printf("Failed to configure GPIO pin %d\n", GPIO_PIN_0);
		return;
	}
	ret = gpio_pin_interrupt_configure(gpio_btns_dev,GPIO_PIN_0, GPIO_INT_EDGE_RISING);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on gpio_btns\n",
			ret);
		return 0;
	}
	gpio_init_callback(&button_cb_data, button_pressed, 0xFFFF);
	gpio_add_callback(gpio_btns_dev,&button_cb_data);
	// uart
	if (!device_is_ready(uart0_dev)) {
		printf("UART device not found!");
		return 0;
	}
	ret = uart_irq_callback_user_data_set(uart0_dev, serial_cb, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printf("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printf("UART device does not support interrupt-driven API\n");
		} else {
			printf("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart0_dev);

	//uartlite
	if (!device_is_ready(uartlite0_dev)) {
		printf("UART device not found!");
		return 0;
	}
	ret = uart_irq_callback_user_data_set(uartlite0_dev, serial_lite_cb, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printf("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printf("UART device does not support interrupt-driven API\n");
		} else {
			printf("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uartlite0_dev);
	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");

	// app
	for (int i = 0; i < 20; i++) {
		int val = 0;
		// printf("toggle btn = 0x%x\r\n", val);
		gpio_pin_toggle(gpio_leds_dev, GPIO_PIN_0);
		k_msleep(500);
	}
	printf("test ended.\r\n");

	while(1){}
	return 0;
}
