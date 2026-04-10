#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include "ble_handler.h"
#include <stdio.h>
#include "battery_monitor.h"

const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

int main(void) {
    if (!device_is_ready(gpio0)) {
        return 0;
    }

    // module led pin
    gpio_pin_configure(gpio0, 6, GPIO_OUTPUT_ACTIVE);

    // external pin, p0.29 blue led
    gpio_pin_configure(gpio0, 29, GPIO_OUTPUT_ACTIVE);

    // Wait for USB console to stabilize
    k_sleep(K_MSEC(3000));
    printk("\n--- GaitSyncAR Booting Up ---\n");

    // Initialize BLE
    if (ble_handler_init() != 0) {
        printk("FATAL: Failed to start BLE!\n");
    }

    // Initialize Battery Monitor
    int err = battery_monitor_init();
    if (err != 0) {
        printk("Battery Monitor Init Failed: %d\n", err);
    }

    while (1) {
        // Toggle leds
        gpio_pin_toggle(gpio0, 6);
        gpio_pin_toggle(gpio0, 29);

        uint32_t batt_mv = battery_monitor_get_mv();
        
        char tx_buffer[64];
        snprintf(tx_buffer, sizeof(tx_buffer), "Batt: %u mV\n", batt_mv);
        
        /* Send to BLE and print to USB for debugging */
        ble_handler_send((uint8_t *)tx_buffer, strlen(tx_buffer));
        printk("%s", tx_buffer);
        
        k_msleep(2000);
    }

    return 0;
}