/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#define DAT_LEN 25


static void start_scan(void);

static float res_floats[3];

void bytes2Floats(float* val, const void* bytes_array){
  // Create union of shared memory space
  union {
    float float_variables[3];
    uint8_t temp_array[12];
  } u;
  memcpy(u.temp_array, bytes_array, 12);
  val[0] = u.float_variables[0];
  val[1] = u.float_variables[1];
  val[2] = u.float_variables[2];
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	uint8_t *dat = user_data;

	switch (data->type) {
	case BT_DATA_MANUFACTURER_DATA:
	    if (data->data_len == DAT_LEN) {
			char ch = (char)data->data[DAT_LEN - 1];
			printk("ch %c %d \n", ch, ch);
			if (ch == 'n' || ch == 'y' || ch == 's') {
                (void)memcpy(dat, data->data, DAT_LEN);
			    return false;
			}
		}
		printk("wrong len %d\n", data->data_len);
		return true;
	default:
		return true;
	}
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	uint8_t dat[DAT_LEN];
	uint8_t dat0[DAT_LEN];

	(void)memset(dat, 0, sizeof(dat));
	(void)memset(dat0, 0, sizeof(dat0));
	bt_data_parse(ad, data_cb, dat);

	if (memcmp(dat, dat0, DAT_LEN) != 0) {
		if ((char)dat[DAT_LEN - 1] == 's'){
			printk("s\n");
		} else {
			bytes2Floats(res_floats, dat);
			printk("first %f %f %f\n", res_floats[0], res_floats[1], res_floats[2]);
			bytes2Floats(res_floats, dat + 12);
			printk("second %f %f %f\n", res_floats[0], res_floats[1], res_floats[2]);
		}
	}
}

static void start_scan(void)
{
	int err;

	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = 0,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	start_scan();
	return 0;
}
