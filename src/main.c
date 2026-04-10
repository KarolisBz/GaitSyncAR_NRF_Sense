#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include "ble_handler.h"
#include <stdio.h> /* Required for snprintf */

const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

int main(void) {
    if (!device_is_ready(gpio0)) {
        return 0;
    }

    gpio_pin_configure(gpio0, 6, GPIO_OUTPUT_ACTIVE);

    k_sleep(K_MSEC(3000));
    printk("\n--- GaitSyncAR Booting Up ---\n");

    if (ble_handler_init() != 0) {
        printk("FATAL: Failed to start BLE!\n");
    }

    /* fake voltage variable to test dynamic data */
    int simulated_voltage_mv = 3300; 

    while (1) {
        gpio_pin_toggle(gpio0, 6);
        
        /* 1. empty character buffer to hold message */
        char tx_buffer[64]; 
        
        /* 2. Formatting the string */
        snprintf(tx_buffer, sizeof(tx_buffer), "Batt: %d mV\n", simulated_voltage_mv);
        
        /* 3. Sending the formatted buffer over BLE */
        int err = ble_handler_send((const uint8_t *)tx_buffer, strlen(tx_buffer));
        
        if (err == 0) {
            /* Printing the exact same buffer to USB so we can verify it */
            printk("BLE Sent: %s", tx_buffer);
            
            /* Changing the voltage slightly for the next loop so we see it update */
            simulated_voltage_mv -= 10; 
            if (simulated_voltage_mv < 3000) simulated_voltage_mv = 3300; 
            
        } else if (err == -ENOTCONN) {
            printk("Waiting for phone to connect...\n");
        }
        
        k_msleep(1000);
    }

    return 0;
}