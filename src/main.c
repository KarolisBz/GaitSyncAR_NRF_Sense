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
#include "app_events.h"
#include <hal/nrf_ppi.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_gpio.h>


// Fields
static const struct device *const imu_dev = DEVICE_DT_GET_ANY(st_lsm6dsl);
#define IMU_INTERRUPT_PIN 11 

// Allocates the actual RAM using the rules defined in the header
K_MSGQ_DEFINE(app_msgq, sizeof(app_event_t), APP_MSGQ_MAX_EVENTS, APP_MSGQ_ALIGNMENT);

// Hardware timestamping of the IMU interrupt using PPI and GPIOTE
void init_ppi_timestamping(void) {
    // 1. Configure the GPIO pin itself
    nrf_gpio_cfg_input(IMU_INTERRUPT_PIN, NRF_GPIO_PIN_PULLDOWN);

    // 2. Configure GPIOTE Channel 0 manually for High Accuracy
    // We use the direct register access to ensure 'Mode' is set to 'Event' (0x01)
    // and 'Polarity' is set to 'LoToHi' (0x01)
    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (IMU_INTERRUPT_PIN << GPIOTE_CONFIG_PSEL_Pos) |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    // 3. Wire the PPI Channel 0
    // Task: Capture Timer 1 into CC[3] when GPIOTE Event 0 fires
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        NRF_PPI_CHANNEL0,
        (uint32_t)&NRF_GPIOTE->EVENTS_IN[0],
        nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE3)
    );

    // 4. Enable the PPI channel
    nrf_ppi_channel_enable(NRF_PPI, NRF_PPI_CHANNEL0);
    
    printk("Hardware High-Accuracy PPI Timestamping Enabled.\n");
}

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

    // Initialize hardware timestamping of the IMU interrupt
    init_ppi_timestamping();

    // Initialize IMU interrupts for step detection
    init_imu_interrupts();

    // Starting event driven main loop
    printk("--- Starting Event-Driven System ---\n");
    app_event_t pending_event;
    uint64_t last_slow_task_time = k_uptime_get();

    while (1) {
        // Sleep for 1ms. 
        int msg_status = k_msgq_get(&app_msgq, &pending_event, K_MSEC(1));

        // ROUTING EVENT IMMEDIATELY
        if (msg_status == 0) {
            
            if (pending_event.type == EVENT_STEP_DETECTED) {
                // Send step to Unity
                send_step_event(pending_event.step_timestamp);
                //printk("Sent Step to Unity: %llu us\n", pending_event.step_timestamp);
                
            } else if (pending_event.type == EVENT_SYNC_COMPLETED) {
                extern volatile uint64_t global_sync_baseline_us; 
                
                // sending acknowledgment back to Unity that sync is complete, along with the baseline timestamp for reference
                send_sync_ack_event(global_sync_baseline_us); 
                //printk("HARDWARE SYNC COMPLETE at baseline %llu us\n", global_sync_baseline_us);
            }
        }
        else {
            // If 1ms pass with no taps, the CPU wakes up and runs this block.
            // If the IMU pin is stuck HIGH, this fetch clears the hardware register,
            // dropping the pin to LOW so interrupt is fixed
            sensor_sample_fetch(imu_dev);
        }


        // --- BACKGROUND TASKS ---
        // 2 seconds
        uint64_t now = k_uptime_get();
        if ((now - last_slow_task_time) >= 2000) {
            last_slow_task_time = now; 
            
            // reading battery level every 2 seconds and sending to Unity
            uint8_t batt = battery_monitor_get_percentage();
            send_battery_event(batt);
        }
    }

    return 0;
}