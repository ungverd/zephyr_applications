/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <stdint.h>

#include "common.h"

static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});

static const struct bt_ias_client_cb ias_client_cb = {
	.discover = discover_cb,
};

static void test_alert_high(struct bt_conn *conn)
{
	int err;

	err = bt_ias_client_alert_write(conn, BT_IAS_ALERT_LVL_HIGH_ALERT);
	if (err == 0) {
		printk("High alert sent\n");
	} else {
	}
}

static void test_alert_mild(struct bt_conn *conn)
{
	int err;

	err = bt_ias_client_alert_write(conn, BT_IAS_ALERT_LVL_MILD_ALERT);
	if (err == 0) {
		printk("Mild alert sent\n");
	}
}

static void test_alert_stop(struct bt_conn *conn)
{
	int err;

	err = bt_ias_client_alert_write(conn, BT_IAS_ALERT_LVL_NO_ALERT);
	if (err == 0) {
		printk("Stop alert sent\n");
	}
}

#define SLEEP_TIME_MS	1

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

int main(void)
{
	int val0 = 0;
	int ret;
	bool state = false;

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !device_is_ready(led.port)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led.port->name);
		led.port = NULL;
	}
	if (led.port) {
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led.port->name, led.pin);
			led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}

	int err;

	err = bt_enable(NULL);
	if (err < 0) {
		printk("Bluetooth initialisation failed!\n");
		return 1;
	}

	printk("Bluetooth initialized\n");

	err = bt_ias_client_cb_register(&ias_client_cb);
	if (err < 0) {
		printk("IAS callback initialisation failed!\n");
		return 1;
	}

	printk("IAS callback initialised\n");

	start_scan();
	WAIT_FOR_FLAG(flag_connected);

	printk("Press the button\n");
	if (led.port) {
		while (1) {
			if (!TEST_FLAG(flag_connected)) {
				WAIT_FOR_FLAG(flag_connected);
			}
			/* If we have an LED, match its state to the button's. */
			int val = gpio_pin_get_dt(&button);

			if (val != val0) {
                val0 = val;
				if (val == 1) {
					state = !state;
					if (state) {
						gpio_pin_set_dt(&led, 1);
						test_alert_mild(default_conn);
					} else {
						gpio_pin_set_dt(&led, 0);
						test_alert_stop(default_conn);
					}
				}
			}
			k_msleep(SLEEP_TIME_MS);
		}
	}
	return 0;
}
