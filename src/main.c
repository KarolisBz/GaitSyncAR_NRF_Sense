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

// Synchronization Semaphore
K_SEM_DEFINE(main_loop_sem, 0, 1);
struct k_timer metronome_timer;

// Metronome ISR
void metronome_handler(struct k_timer *timer_id) {
    k_sem_give(&main_loop_sem);
}

int main(void) {
    // Initialize LED module
    led_pwm_init();

    // Starting the metronome at exactly 100Hz (10ms)
    k_timer_init(&metronome_timer, metronome_handler, NULL);
    k_timer_start(&metronome_timer, K_MSEC(10), K_MSEC(10));

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

    // configuring imu step detection //
    imu_step_config_t step_cfg = {
        .threshold = 1600,    // MUST spike above 16 m/s^2 to count as a landing
        .noise_floor = 650,   // MUST drop below 6.5 m/s^2 to count as swinging in the air
        .cooldown_ms = 350
    };
    imu_step_init(step_cfg);

    if (imu_step_init(step_cfg) != 0) {
        printk("IMU Init Failed!\n");
    }

    printk("  -> Starting loop...\n");

    // Track the time for slower tasks
    uint64_t last_slow_task_time = k_uptime_get();

    while (1) {
        // Waiting indefinitely until the metronome drops a token
        // The CPU sleeps, background tasks (like BLE) can still run and will preempt this when needed
        // This prevent time drifts that can occur with k_sleep() and ensures our 100Hz loop runs as accurately as possible
        k_sem_take(&main_loop_sem, K_FOREVER);

        // Fetch uptime
        uint64_t now = k_uptime_get();

        // Fast tasks (10ms) 100Hz -------------------------------------------------------------------
        int32_t mag;
        bool step_detected = imu_step_update(&mag);
        // printk("Magnitude: %d\n", mag);

        if (step_detected) {
           send_step_event();
        }

        // Slow tasks (2000ms) 0.5Hz -------------------------------------------------------------------
        if ((now - last_slow_task_time) >= 2000) {
            last_slow_task_time = now; // Reset the timer

            // Read battery voltage and send over BLE //
            uint8_t batt_percentage = battery_monitor_get_percentage();
            send_battery_event(batt_percentage);
            printk("Battery Percentage: %u%%\n", batt_percentage);
        }

        // main loop 10ms, 100Hz
        k_msleep(10);
    }

    return 0;
}