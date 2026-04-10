#include "ble_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>
#include <string.h>

/* Globals */
static struct bt_conn *active_conn = NULL;

/* --- BLE Advertising Data --- */

// PRIMARY PACKET: Advertising the NUS Service
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// SCAN RESPONSE: Providing the Device Name
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param adv_param = {
    .options      = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

/* --- Connection Callbacks --- */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connect failed (err %u)\n", err);
        return;
    }
    active_conn = bt_conn_ref(conn);
    printk("BLE connected\n");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (active_conn) {
        bt_conn_unref(active_conn);
        active_conn = NULL;
    }
    printk("BLE disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

/* --- API Implementation --- */

/* Define empty callbacks if you don't need to handle RX yet */
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    // Handle incoming data from mobile here
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

int ble_handler_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) return err;

    /* CRITICAL: Initialize the NUS service before advertising */
    err = bt_nus_init(&nus_cb);
    if (err) {
        printk("NUS init failed (err %d)\n", err);
        return err;
    }

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    // ... rest of your code
}

int ble_handler_send(const uint8_t *data, uint16_t len)
{
    if (!active_conn) {
        return -ENOTCONN;
    }
    
    // In this specific task, we ignore the passed data and just send Hello World
    const char *hello = "Hello World\n";
    return bt_nus_send(active_conn, (const uint8_t *)hello, strlen(hello));
}