/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#define DAT_LEN 1

#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define UUID_LEN 16

static uint8_t reference_uuid[UUID_LEN];

static void numbers_to_uuid(uint8_t *res, long n1, int n2, int n3, int n4, long long n5) {
	int i;
	for(i = 0; i < 6; i++) {
		res[i] = n5;
		n5 >>= 8;
	}
	for(i = 6; i < 8; i++) {
		res[i] = n4;
		n4 >>= 8;
	}
	for(i = 8; i < 10; i++) {
		res[i] = n3;
		n3 >>= 8;
	}
	for(i = 10; i < 12; i++) {
		res[i] = n2;
		n2 >>= 8;
	}
	for(i = 12; i < 16; i++) {
		res[i] = n1;
		n1 >>= 8;
	}
	uint8_t *uuid = res;
	printk("my_uuid: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", \
	uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7], uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
		
}

/*static bool data_cb(struct bt_data *data, void *user_data)
{
	uint8_t *uuid = user_data;

	switch (data->type) {
	case BT_DATA_UUID16_ALL:
		(void)memcpy(uuid, data->data, 2);
		return false;
	default:
		return true;
	}
}*/

static bool uuid_cb(struct bt_data *data, void *user_data)
{
	uint8_t *uuid = user_data;

	switch (data->type) {
	case BT_DATA_UUID128_ALL:
	    (void)memcpy(uuid, data->data, UUID_LEN);
		printk("uuid: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", \
		uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7], uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
		return false;
	default:
		return true;
	}
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	uint8_t *dat = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_MANUFACTURER_DATA:
		len = MIN(data->data_len, DAT_LEN);
		(void)memcpy(dat, data->data, len);
		return false;
	default:
		return true;
	}
}

static uint8_t mfg_data[] = { 0x78, 0x9a, 0xbc };

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	/*if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}*/
	printk("dev\n");

	uint8_t uuid[UUID_LEN];
	/* char name[NAME_LEN]; */

	(void)memset(uuid, 0, UUID_LEN);


	bt_data_parse(ad, uuid_cb, uuid);
	if (memcmp(uuid, reference_uuid, UUID_LEN) == 0) {
	    printk("Device found: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x dat len %d\n", \
		uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7], uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15], ad->len);
	}
	/* printk("Device found: %x %x uuid\n", uuid[0], uuid[1]); */

	/*bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	if (memcmp(name, TARGET_NAME, sizeof(TARGET_NAME)) == 0) {
	    printk("Device found: %s name %s (RSSI %d), type %u, AD data len %u\n",
	       name, addr_str, rssi, type, ad->len);
	}*/
}

#if defined(CONFIG_BT_EXT_ADV)

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_NONE: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];
	uint8_t data_status;
	uint16_t data_len;

	(void)memset(name, 0, sizeof(name));

	data_len = buf->len;
	bt_data_parse(buf, data_cb, name);

	data_status = BT_HCI_LE_ADV_EVT_TYPE_DATA_STATUS(info->adv_props);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i "
	       "Data status: %u, AD data len: %u Name: %s "
	       "C:%u S:%u D:%u SR:%u E:%u Pri PHY: %s, Sec PHY: %s, "
	       "Interval: 0x%04x (%u ms), SID: %u\n",
	       le_addr, info->adv_type, info->tx_power, info->rssi,
	       data_status, data_len, name,
	       (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
	       phy2str(info->primary_phy), phy2str(info->secondary_phy),
	       info->interval, info->interval * 5 / 4, info->sid);
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};
#endif /* CONFIG_BT_EXT_ADV */

int observer_start(void)
{
	numbers_to_uuid(reference_uuid, 0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0);
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

#if defined(CONFIG_BT_EXT_ADV)
	bt_le_scan_cb_register(&scan_callbacks);
	printk("Registered scan callbacks\n");
#endif /* CONFIG_BT_EXT_ADV */

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Start scanning failed (err %d)\n", err);
		return err;
	}
	printk("Started scanning...\n");

	return 0;
}
