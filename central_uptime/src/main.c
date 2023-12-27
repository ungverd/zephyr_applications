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


static void start_scan(void);

static struct bt_conn *default_conn;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_subscribe_params subscribe_params2;
#define TARGET_VAL 0x2788
#define CHAR_VAL 0x2789
#define CHAR2_VAL 0x278A
#define TARGET_UUID BT_UUID_DECLARE_16(TARGET_VAL)
#define CHAR_UUID BT_UUID_DECLARE_16(CHAR_VAL)
#define CHAR2_UUID BT_UUID_DECLARE_16(CHAR2_VAL)
static uint16_t target_uuid_arr[] = {TARGET_VAL};
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

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	if (length != 12) {
		printk("ERROR LENGTH %u", length);
		return BT_GATT_ITER_CONTINUE;
	}

	union {
	    uint8_t u_bytes[12];
	    int64_t u_int;
	} u;

	memcpy(u.u_bytes, data, 12);
	printk("res %lld\n", u.u_int);

	/* bytes2Floats(res_floats, data);

	printk("%f %f %f\n", res_floats[0], res_floats[1], res_floats[2]); */

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t notify_func2(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	if (length != 1) {
		printk("ERROR LENGTH %u", length);
		return BT_GATT_ITER_CONTINUE;
	}
	char *d = (char*)data;

	printk("%c\n", *d);

	return BT_GATT_ITER_CONTINUE;
}

static bool first_discovered = false; 

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, TARGET_UUID)) {
		memcpy(&uuid, CHAR_UUID, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				CHAR_UUID)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				CHAR2_UUID)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params2.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		if (!first_discovered) {
			first_discovered = true;
			subscribe_params.notify = notify_func;
			subscribe_params.value = BT_GATT_CCC_NOTIFY;
			subscribe_params.ccc_handle = attr->handle;

			err = bt_gatt_subscribe(conn, &subscribe_params);
			if (err && err != -EALREADY) {
				printk("Subscribe failed (err %d)\n", err);
			} else {
				printk("[SUBSCRIBED]\n");
			}
			memcpy(&uuid, CHAR2_UUID, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				printk("Discover failed (err %d)\n", err);
			}
		} else {
			first_discovered = false;
			subscribe_params2.notify = notify_func2;
			subscribe_params2.value = BT_GATT_CCC_NOTIFY;
			subscribe_params2.ccc_handle = attr->handle;

			err = bt_gatt_subscribe(conn, &subscribe_params2);
			if (err && err != -EALREADY) {
				printk("Subscribe2 failed (err %d)\n", err);
			} else {
				printk("[SUBSCRIBED2]\n");
			}
		    return BT_GATT_ITER_STOP;
		}
	}

	return BT_GATT_ITER_STOP;
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	uint8_t *uuid_ = user_data;

	switch (data->type) {
	case BT_DATA_UUID16_ALL:
		(void)memcpy(uuid_, data->data, 2);
		return false;
	default:
		return true;
	}
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	uint8_t uuid_[2];
	int err;

	if (default_conn) {
		return;
	}

	(void)memset(uuid_, 0, sizeof(uuid_));
	bt_data_parse(ad, data_cb, uuid_);

	if (memcmp(uuid_, target_uuid_arr, 2) == 0) {
		if (bt_le_scan_stop()) {
		    return;
		}
	    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
		if (err) {
			printk("Create conn failed");
			start_scan();
		}
	}
}

static void start_scan(void)
{
	int err;

	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
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

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);
	if (conn == default_conn) {
		memcpy(&uuid, TARGET_UUID, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

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
