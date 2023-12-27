/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

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
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});

#define SERVICE_VAL 0x2788
#define CHAR_VAL 0x2789
#define CHAR2_VAL 0x278A

static struct bt_uuid_16 vnd_uuid = BT_UUID_INIT_16(SERVICE_VAL);

static struct bt_uuid_16 char_uuid = BT_UUID_INIT_16(CHAR_VAL);
static struct bt_uuid_16 char2_uuid = BT_UUID_INIT_16(CHAR2_VAL);

static uint8_t data_to_send[12];
#define RADIO_OFF 'n'
#define FILE_SWITCH 's'
static char stat[] = {RADIO_OFF,};
static struct bt_conn* default_conn = NULL;

static ssize_t write_3D(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(data_to_send)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}
static ssize_t write_stat(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset,
			uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

BT_GATT_SERVICE_DEFINE(vnd_svc,
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
	BT_GATT_CHARACTERISTIC(&char_uuid.uuid,
    BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_WRITE,
	NULL, write_3D, &data_to_send),
	BT_GATT_CCC(NULL, BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&char2_uuid.uuid,
    BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_WRITE,
	NULL, write_stat, &stat),
	BT_GATT_CCC(NULL, BT_GATT_PERM_WRITE)
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(SERVICE_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
		default_conn = conn;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
	default_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

#define ACCEL_ALIAS(i) DT_ALIAS(_CONCAT(accel, i))
#define ACCELEROMETER_DEVICE(i, _) \
	IF_ENABLED(DT_NODE_EXISTS(ACCEL_ALIAS(i)), (DEVICE_DT_GET(ACCEL_ALIAS(i)),))

/* support up to 10 accelerometer sensors */
static const struct device *const sensors[] = {LISTIFY(10, ACCELEROMETER_DEVICE, ())};

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
};


void floats2Bytes(float* val,uint8_t* bytes_array){
  // Create union of shared memory space
  union {
    float float_variables[3];
    uint8_t temp_array[12];
  } u;
  // Overite bytes of union with float variable
  u.float_variables[0] = val[0];
  u.float_variables[1] = val[1];
  u.float_variables[2] = val[2];
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 12);
}

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static int print_accels(const struct device *dev, struct bt_gatt_attr *attr)
{
	int ret;
	struct sensor_value accel[3];

	ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		printk("%s: sensor_sample_fetch() failed: %d\n", dev->name, ret);
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		ret = sensor_channel_get(dev, channels[i], &accel[i]);
		if (ret < 0) {
			printk("%s: sensor_channel_get(%c) failed: %d\n", dev->name, 'X' + i, ret);
			return ret;
		}
	}

	printk("%16s [m/s^2]:    (%12.6f, %12.6f, %12.6f)\n", dev->name,
	       sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
	       sensor_value_to_double(&accel[2]));
	float floats[] = {sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
	       sensor_value_to_double(&accel[2])};
	floats2Bytes(floats, data_to_send);
	ret = bt_gatt_notify(default_conn, attr, &data_to_send, sizeof(data_to_send));
	if (ret < 0) {
		printk("bt_gatt_notify failed: %d\n", ret);
	}

	return 0;
}

static struct bt_gatt_attr *attr = NULL;
static struct bt_gatt_attr *attr2 = NULL;
static int waiting = 0;
static bool radio_on = true;
static int val;
static bool wait2 = false;
static int ret;
static size_t i;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	stat[0] = FILE_SWITCH;
	waiting = 30;
}

void my_work_handler(struct k_work *work)
{
    if (wait2) {
        ret = bt_gatt_notify(default_conn, attr2, &stat, sizeof(stat));
        if (ret < 0) {
            printk("bt_gatt_notify failed: %d\n", ret);
        }
        wait2 = false;
    } 
    if (waiting == 0) {
        if (radio_on) {
            for (i = 0; i < ARRAY_SIZE(sensors); i++) {
                ret = print_accels(sensors[i], attr);
                if (ret < 0) {
                    printk("print_accels failed: %d\n", ret);
                }
            }
        }
    }
    else {
        if (waiting == 29) {
            k_msleep(10);
            int ret = bt_gatt_notify(default_conn, attr2, &stat, sizeof(stat));
            if (ret < 0) {
                printk("bt_gatt_notify failed: %d\n", ret);
            }
        }
        waiting--;
        if (waiting == 2) {
            val = gpio_pin_get_dt(&button);
            if (val > 0) {
                radio_on = false;
                stat[0] = RADIO_OFF;
                gpio_pin_set_dt(&led, 0);
                wait2 = true;
            }
        }
    }
}

K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *dummy)
{


    k_work_submit(&my_work);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

int main(void)
{
	struct sensor_value val;
	sensor_g_to_ms2(4, &val);
	sensor_attr_set	(sensors[0],
                     SENSOR_CHAN_ACCEL_X,
                     SENSOR_ATTR_FULL_SCALE,
                     &val);

    sensor_attr_set	(sensors[0],
                     SENSOR_CHAN_ACCEL_Y,
                     SENSOR_ATTR_FULL_SCALE,
                     &val);

    sensor_attr_set	(sensors[0],
                     SENSOR_CHAN_ACCEL_Z,
                     SENSOR_ATTR_FULL_SCALE,
                     &val);
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	attr = bt_gatt_find_by_uuid(NULL, 0, &char_uuid.uuid);
	attr2 = bt_gatt_find_by_uuid(NULL, 0, &char2_uuid.uuid);

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

	for (i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			printk("sensor: device %s not ready.\n", sensors[i]->name);
			return 0;
		}
	}

	gpio_pin_set_dt(&led, 1);

	k_timer_start(&my_timer, K_MSEC(20), K_MSEC(20));
	return 0;
}
