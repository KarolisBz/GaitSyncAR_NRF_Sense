#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>
#include <string.h>

// ─── Config ───────────────────────────────────────────────────────────────────

#define LED_PIN          29        // P0.29 on XIAO BLE Sense
#define LOOP_INTERVAL_MS 1000

#define ADC_CHANNEL      1
#define ADC_RESOLUTION   12
#define VDIVIDER_MULT    2.10f     // Voltage divider correction factor

// ─── Globals ──────────────────────────────────────────────────────────────────

static const struct device *gpio0    = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *adc_dev  = DEVICE_DT_GET(DT_NODELABEL(adc));
static struct bt_conn *active_conn   = NULL;

// ─── BLE Advertising Data ─────────────────────────────────────────────────────

// PRIMARY PACKET
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// SCAN RESPONSE: The Device Name
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param adv_param = {
    .options      = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

// ─── BLE Callbacks ────────────────────────────────────────────────────────────

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

// ─── ADC Configuration ────────────────────────────────────────────────────────

static const struct adc_channel_cfg adc_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    // 40 microsecond shutter speed allows the 1M ohm resistors to fill the capacitor
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40), 
    .channel_id       = ADC_CHANNEL,
    .input_positive   = SAADC_CH_PSELP_PSELP_AnalogInput1, 
};

static float read_battery_voltage(void)
{
    int16_t raw = 0;
    struct adc_sequence seq = {
        .channels     = BIT(ADC_CHANNEL),
        .buffer       = &raw,
        .buffer_size  = sizeof(raw),
        .resolution   = ADC_RESOLUTION,
        .oversampling = 4, // 16 samples averaged
    };

    if (!device_is_ready(adc_dev)) {
        return -1.0f;
    }

    if (adc_read(adc_dev, &seq) != 0) {
        return -1.0f;
    }

    // Sometimes the SAADC reads slightly below 0 due to ground noise
    if (raw < 0) {
        raw = 0;
    }

    // Calculating the actual voltage sitting on pin A1
    float pin_voltage = ((float)raw / 4095.0f) * 3.6f;

    // Multiplying by voltage divider to get the total battery voltage
    float battery_voltage = pin_voltage * VDIVIDER_MULT;

    return battery_voltage;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(void)
{
    int err;

    // 1. GPIO Init
    if (device_is_ready(gpio0)) {
        gpio_pin_configure(gpio0, LED_PIN, GPIO_OUTPUT_ACTIVE);
    }

    // 2. ADC Init
    adc_channel_setup(adc_dev, &adc_cfg);

    // 3. Bluetooth Init
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    // Starting advertising with both primary (ad) and secondary (sd) packets
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return 0;
    }

    printk("Advertising started as: %s\n", CONFIG_BT_DEVICE_NAME);

    // 4. Main Loop
    while (1) {
        if (active_conn) {
            float batt = read_battery_voltage();

            char msg[32];
            snprintf(msg, sizeof(msg), "Batt: %.2fV\n", (double)batt);
            
            // Sending the text string via Nordic UART Service
            bt_nus_send(active_conn, (uint8_t *)msg, strlen(msg));
            printk("%s", msg);
        }

        // gpio_pin_toggle(gpio0, LED_PIN);

        k_msleep(LOOP_INTERVAL_MS);
    }

    return 0;
}