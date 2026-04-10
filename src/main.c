#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include "ble_handler.h"
#include <stdio.h>
#include "battery_monitor.h"
#include "imu_step.h"

const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

int main(void) {
    if (!device_is_ready(gpio0)) {
        return 0;
    }

    // module led pin
    gpio_pin_configure(gpio0, 6, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);

    // external pin, p0.29 blue led 
    gpio_pin_configure(gpio0, 29, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);

    // Wait for USB console to stabilize //
    k_sleep(K_MSEC(3000));
    printk("\n--- GaitSyncAR Booting Up ---\n");

    // Initialize Battery Monitor //
    int err = battery_monitor_init();
    if (err != 0) {
        printk("Battery Monitor Init Failed: %d\n", err);
    }

    // configuring imu step detection //
    imu_step_config_t step_cfg = {
        .threshold = 1500,    // MUST spike above 12.0 m/s^2 to count as a landing
        .noise_floor = 700,   // MUST drop below 7.0 m/s^2 to count as swinging in the air
        .cooldown_ms = 350
    };
    imu_step_init(step_cfg);

    if (imu_step_init(step_cfg) != 0) {
        printk("IMU Init Failed!\n");
    }

    printk("  -> Starting loop...\n");

    // Initialize BLE //
    if (ble_handler_init() != 0) {
        printk("FATAL: Failed to start BLE!\n");
    }

    // Track the time for slower tasks
    uint64_t last_slow_task_time = k_uptime_get();

    while (1) {
        // Fetch uptime
        uint64_t now = k_uptime_get();

        // Fast tasks (50ms) 20Hz -------------------------------------------------------------------
        int32_t mag;
        bool step_detected = imu_step_update(&mag);
        printk("Magnitude: %d\n", mag);

        if (step_detected) {
            printk(">>> STEP DETECTED! (Mag: %d) <<<\n", mag);
            
            // Instantly send the step event to the phone
            char step_msg[32];
            snprintf(step_msg, sizeof(step_msg), "STEP:%d\n", mag);
            ble_handler_send((uint8_t *)step_msg, strlen(step_msg));
        }

        // Slow tasks (2000ms) 0.5Hz -------------------------------------------------------------------
        if ((now - last_slow_task_time) >= 2000) {
            last_slow_task_time = now; // Reset the timer

             // Toggle leds //
            gpio_pin_toggle(gpio0, 6);
            gpio_pin_toggle(gpio0, 29);

            // Read battery voltage and send over BLE //
            uint32_t batt_mv = battery_monitor_get_mv();
            
            char tx_buffer[64];
            snprintf(tx_buffer, sizeof(tx_buffer), "Batt: %u mV\n", batt_mv);
            
            /* Send to BLE and print to USB for debugging */
            ble_handler_send((uint8_t *)tx_buffer, strlen(tx_buffer));
            printk("%s", tx_buffer);

        }

        // main loop 50ms, 20Hz
        k_msleep(50);
    }

    return 0;
}