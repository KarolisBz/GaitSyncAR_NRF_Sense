#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
// custom modules
#include "ble_handler.h"
#include "battery_monitor.h"
#include "imu_step.h"
#include "nfc_handler.h"
#include "device_role.h"
#include "led_pwm.h"
#include "clock_sync.h"
#include <zephyr/drivers/sensor.h>

// Fields
struct k_timer main_timer;
static const struct device *const imu_dev = DEVICE_DT_GET_ANY(st_lsm6dsl);
K_MSGQ_DEFINE(step_msgq, sizeof(uint64_t), 10, 4); // Queue holds up to 10 events

int main(void) {
    // Initialize LED module
    led_pwm_init();

    // Wait for USB console to stabilize //
    k_sleep(K_MSEC(3000));
    printk("\n--- GaitSyncAR Booting Up ---\n");

    // fetch device identity and assign role //
    device_role_init();

    // Initialize BLE //
    if (ble_handler_init() != 0) {
        printk("FATAL: Failed to start BLE!\n");
    }

    // Initialize NFC //
    int nfc_err = nfc_handler_init();
    if (nfc_err != 0) {
        printk("NFC Init Failed: %d\n", nfc_err);
    } else {
        printk("  -> NFC Tag Emulation Started!\n");
    }

    // Initialize Battery Monitor //
    int err = battery_monitor_init();
    if (err != 0) {
        printk("Battery Monitor Init Failed: %d\n", err);
    }

    // Initialize Clock Synchronization //
    clock_sync_init();

    // configuring imu step detection //
    imu_step_config_t step_cfg = {
        .threshold = 1600,    // MUST spike above 16 m/s^2 to count as a landing
        .noise_floor = 650,   // MUST drop below 6.5 m/s^2 to count as swinging in the air
        .cooldown_ms = 350
    };

    if (imu_step_init(step_cfg) != 0) {
        printk("IMU Init Failed!\n");
    }

    printk("  -> Starting loop...\n");

    init_imu_interrupts();

    // Track the time for slower tasks
    uint64_t last_slow_task_time = k_uptime_get();

    // Start metronome
    k_timer_init(&main_timer, NULL, NULL); // No ISR needed
    k_timer_start(&main_timer, K_MSEC(10), K_MSEC(10));

    while (1) {
        // This blocks the thread natively until the timer expires
        k_timer_status_sync(&main_timer);

        // prevents synchronus code running in system background thread
        // prevents stalling other system interrupts
        uint64_t pending_step_timestamp;
        if (k_msgq_get(&step_msgq, &pending_step_timestamp, K_NO_WAIT) == 0) {
            send_step_event(pending_step_timestamp);
        }

        if (is_hardware_synced && !sync_ack_sent) {
            send_sync_ack_event(global_sync_baseline_us / 1000);
            sync_ack_sent = true; 
            printk("HARDWARE SYNC COMPLETE at baseline %llu µs\n", global_sync_baseline_us);
        }

        // Fetch uptime
        uint64_t now = k_uptime_get();

        // unsticking pin
        sensor_sample_fetch(imu_dev);

        // Slow tasks (2000ms) 0.5Hz -------------------------------------------------------------------
        if ((now - last_slow_task_time) >= 2000) {
            last_slow_task_time = now; // Reset the timer

            // Read battery voltage and send over BLE //
            uint8_t batt_percentage = battery_monitor_get_percentage();
            send_battery_event(batt_percentage);
            // printk("Battery Percentage: %u%%\n", batt_percentage);
        }
    }

    return 0;
}