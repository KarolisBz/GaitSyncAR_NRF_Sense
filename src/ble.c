#include "ble_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>
#include <string.h>
#include "device_role.h"

/* --- GLOBALS --- */
static struct bt_conn *active_conn = NULL;
static struct k_work_delayable adv_start_work; // <--- DECLARE THIS AT THE TOP

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// The dynamic names for the Left and Right sensors
static const struct bt_data sd_primary[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "GaitSync-Right", 14),
};

static const struct bt_data sd_secondary[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "GaitSync-Left", 13),
};

static struct bt_le_adv_param adv_param = {
    .options = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

/* --- WORK HANDLERS --- */

/* This runs on the system workqueue to safely restart advertising */
static void restart_ad_handler(struct k_work *work)
{
    bt_le_adv_stop();
    k_sleep(K_MSEC(50));

    int err;
    bool is_primary = device_role_is_primary();

    err = bt_le_adv_start(&adv_param, 
                            ad, 
                            ARRAY_SIZE(ad), 
                            is_primary ? sd_primary : sd_secondary, 
                            is_primary ? ARRAY_SIZE(sd_primary) : ARRAY_SIZE(sd_secondary));
    
    if (err) {
        printk("Adv restart failed (err %d). Retrying...\n", err);
        k_work_reschedule(&adv_start_work, K_MSEC(1000));
    } else {
        printk(">>> Advertising restarted successfully! <<<\n");
    }
}

/* --- CONNECTION CALLBACKS --- */

static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }
    active_conn = bt_conn_ref(conn);
    printk(">>> BLE Connected <<<\n");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    printk(">>> BLE Disconnected (Reason %u) <<<\n", reason);

    if (active_conn) {
        bt_conn_unref(active_conn);
        active_conn = NULL;
    }

    /* Wait 500ms for the stack to clear before restarting */
    k_work_reschedule(&adv_start_work, K_MSEC(500));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

/* --- API IMPLEMENTATION --- */

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    // RX logic here (not used yet)
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

int ble_handler_init(void) {
    int err;

    printk("  -> Init Workqueue...\n");
    k_work_init_delayable(&adv_start_work, restart_ad_handler);

    printk("  -> Enabling Bluetooth Stack...\n");
    err = bt_enable(NULL);
    if (err) return err;

    printk("  -> Initializing NUS Service...\n");
    err = bt_nus_init(&nus_cb);
    if (err) return err;

    printk("  -> Starting Advertising...\n");
    k_sleep(K_MSEC(100)); // Add a tiny 100ms delay to let the USB flush before we hit the radio

    bool is_primary = device_role_is_primary();
    err = bt_le_adv_start(&adv_param, 
                          ad, 
                          ARRAY_SIZE(ad), 
                          is_primary ? sd_primary : sd_secondary, 
                          is_primary ? ARRAY_SIZE(sd_primary) : ARRAY_SIZE(sd_secondary));
    if (err) return err;

    printk("  -> BLE Ready!\n");
    return 0;
}

int ble_handler_send(const uint8_t *data, uint16_t len) {
    if (!active_conn) return -ENOTCONN;
    return bt_nus_send(active_conn, data, len);
}