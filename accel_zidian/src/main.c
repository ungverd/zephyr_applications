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
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <stdint.h>

#include <math.h>

#include "common.h"

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

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

#define NUM_REFS 40
#define NUM_QUADS 5


static int ret;
static size_t i;

static const float refs[] = {-16.18150010822602, -16.851133120288644, -17.627103121397486, -18.546644068236184, -19.500555090034485, -20.237453749533334, -20.949958659703977, -21.415427330017668, -21.372459079583006, -20.29288232441535, -18.049488553123446, -14.740112482790911, -11.235202462707647, -7.7033405554093815, -4.56711661767822, -1.6978364414059741, 0.9405981651953731, 3.929610601045472, 5.664074588864557, 6.931302445227937, 7.414429881195919, 6.948369533453288, 5.618226697964065, 3.6056720099436306, 1.0785245114790136, -1.6001666314626801, -4.3479638102273075, -7.218281006742853, -9.982187645012335, -12.575833182659219, -14.964995517827342, -16.903230498473885, -18.070358655587317, -18.96003978126614, -19.175521787599724, -19.077841712988004, -18.684305455894577, -17.971944164261817, -17.501567029753975, -17.02714106971356};
static const float refs_perp[] = {17.235607590851018, 16.11807706989459, 14.388800116100336, 12.268533857333592, 10.109776868424397, 8.49587969777523, 7.321566553414531, 6.712735134052423, 6.650623518505525, 7.197554847808849, 7.8108622600162505, 8.441567951849931, 8.932096342719197, 9.36405240767186, 10.05752239787336, 11.242356711167567, 14.147085033914854, 17.3405114123049, 19.167389403601245, 20.022744731927464, 19.93956940117027, 19.032896626454793, 17.361337046107224, 14.956981178623682, 12.403189418462143, 10.279180846651775, 8.192519280692785, 6.573295633345318, 5.5834512557880585, 5.030575758858761, 4.882653129690117, 5.032772549438026, 5.407272271316912, 6.30686793099932, 6.7702532046394985, 7.379187111181647, 8.267777146145027, 9.917877559976876, 10.883903609335489, 11.97828067021648};

static const float threshold = 2000;
static const float threshold_perp = 2500;
static const int threshold_hit = 800;
static const int threshold_hit_min = 400;

static float q_s[NUM_QUADS];
static float a_s[NUM_REFS];
static float a_s_perp[NUM_REFS];
static int a_counter = 0;
static int quad_counter = 0;
static float to_calibrate[] = {0, 0, 0};
static const int calibrate_min = 40;
static const int calibrate_max = 80;
static int calibrate_counter = 0;

static float val_q_s(int i) {
	return q_s[(quad_counter + i) % NUM_QUADS];
}

static bool verify_hit() {
	return (val_q_s(2) > threshold_hit && 
            ((val_q_s(2) - val_q_s(1) > threshold_hit_min && val_q_s(2) - val_q_s(4) > threshold_hit_min) ||
			 (val_q_s(2) - val_q_s(3) > threshold_hit_min && val_q_s(2) - val_q_s(0) > threshold_hit_min) ||
			 (val_q_s(2) - val_q_s(3) > threshold_hit_min && val_q_s(2) - val_q_s(1) > threshold_hit_min)));
}

static void calibrate_sum(float x, float y, float z) {
    to_calibrate[0] += x;
    to_calibrate[1] += y;
    to_calibrate[2] += z;
}

static void calibrate_normalize() {
    float norm = sqrtf(to_calibrate[0]*to_calibrate[0]
                         + to_calibrate[1]*to_calibrate[1]
                         + to_calibrate[2]*to_calibrate[2]);
    for(i=0; i<3; i++) {
        to_calibrate[i] = -to_calibrate[i] / norm;
    }
}

static void add_a(float x, float y, float z) {
    a_s[a_counter] = x*to_calibrate[0]
                   + y*to_calibrate[1]
                   + z*to_calibrate[2];
	float sqr = 0;
	float val;
	float xyz[] = {x, y, z};
	for (i=0; i<3; i++) {
        val = xyz[i] - to_calibrate[i]*a_s[a_counter];
        sqr += val * val;
	}
	a_s_perp[a_counter] = sqrtf(sqr);
    a_counter++;
    if (a_counter >= NUM_REFS) {
        a_counter = 0;
    }
	q_s[quad_counter] = x*x + y*y + z*z;
	quad_counter++;
	if (quad_counter >= NUM_QUADS) {
        quad_counter = 0;
    }
}

static bool verify_error() {
    int ii;
    float err = 0;
    float err_perp = 0;
    for (i=0; i<NUM_REFS; i++) {
        ii = (i + a_counter) % NUM_REFS;
        err += (refs[i] - a_s[ii])*(refs[i] - a_s[ii]);
        err_perp += (refs_perp[i] - a_s_perp[ii])*(refs_perp[i] - a_s_perp[ii]);
    }
    return (err < threshold && err_perp < threshold_perp);
}

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

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	test_alert_stop(default_conn);
}

void my_work_handler(struct k_work *work)
{
    struct sensor_value accel[3];

    ret = sensor_sample_fetch(sensors[0]);
    if (ret < 0) {
        printk("%s: sensor_sample_fetch() failed: %d\n", sensors[0]->name, ret);
        return;
    }

    for (i = 0; i < ARRAY_SIZE(channels); i++) {
        ret = sensor_channel_get(sensors[0], channels[i], &accel[i]);
        if (ret < 0) {
            printk("%s: sensor_channel_get(%c) failed: %d\n", sensors[0]->name, 'X' + i, ret);
            return;
        }
    }

    float floats[] = {sensor_value_to_double(&accel[0]),
                      sensor_value_to_double(&accel[1]),
                      sensor_value_to_double(&accel[2])};
    if (calibrate_counter < calibrate_max) {
        calibrate_counter++;
        if (calibrate_counter > calibrate_min) {
            calibrate_sum(floats[0], floats[1], floats[2]);
        }
    } else {
        if(calibrate_counter == calibrate_max) {
            calibrate_counter++;
            calibrate_normalize();
        } else {
            add_a(floats[0], floats[1], floats[2]);
            if (verify_error()) {
                test_alert_mild(default_conn);
            }
		    if (verify_hit()) {
		        test_alert_mild(default_conn);
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


int main(void) {
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

	k_timer_start(&my_timer, K_MSEC(20), K_MSEC(20));
	
	return 0;
}
