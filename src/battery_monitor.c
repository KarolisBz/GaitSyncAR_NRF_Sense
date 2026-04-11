#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include "battery_monitor.h"

/* Grabs the configuration directly from overlay */
static const struct adc_dt_spec adc_chan0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

// Rolling average variables to smooth out BLE voltage sag
#define NUM_SAMPLES 8
static uint8_t sample_history[NUM_SAMPLES] = {0};
static uint8_t sample_index = 0;
static bool array_filled = false;
static uint8_t last_reported_percentage = 255; // 255 means "uninitialized"

// Millivolts vs Percentage for LiFePO4
static const uint32_t lut_mv[] = { 3550, 3400, 3300, 3200, 3100, 2500 };
static const uint8_t lut_pct[] = { 100,  99,   70,   30,   10,   0    };
#define LUT_SIZE (sizeof(lut_pct) / sizeof(lut_pct[0]))


// -------------------------------------------------------------------------
// Initialization
// -------------------------------------------------------------------------
int battery_monitor_init(void) {
    if (!adc_is_ready_dt(&adc_chan0)) {
        return -ENODEV;
    }

    printk("  -> Enabling Battery Monitor (LiFePO4)...\n");
    return adc_channel_setup_dt(&adc_chan0);
}

// -------------------------------------------------------------------------
// Hardware Voltage Read
// -------------------------------------------------------------------------
uint32_t battery_monitor_get_mv(void) {
    int16_t buf;
    struct adc_sequence seq = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    /* This helper automatically pulls resolution/gain/acq-time from the overlay */
    adc_sequence_init_dt(&adc_chan0, &seq);
    adc_read_dt(&adc_chan0, &seq);

    int32_t mv = (int32_t)buf;
    adc_raw_to_millivolts_dt(&adc_chan0, &mv);
    
    // Multiplier for 1:1 (1M Ohm / 1M Ohm) resistor divider
    return (uint32_t)(mv * 2); 
}

// -------------------------------------------------------------------------
// Raw LiFePO4 Percentage Calculation (private)
// -------------------------------------------------------------------------
static uint8_t get_raw_battery_percent(void) {
    uint32_t mv = battery_monitor_get_mv();

    // Checking Out-of-Bounds
    if (mv >= lut_mv[0]) return 100;
    if (mv <= lut_mv[LUT_SIZE - 1]) return 0;

    // Finding the segment in the table
    for (int i = 0; i < LUT_SIZE - 1; i++) {
        if (mv >= lut_mv[i + 1]) {
            uint32_t start_mv = lut_mv[i];
            uint32_t end_mv = lut_mv[i + 1];
            uint8_t start_pct = lut_pct[i];
            uint8_t end_pct = lut_pct[i + 1];

            // Linear Interpolation within this specific segment
            return end_pct + (uint8_t)((mv - end_mv) * (start_pct - end_pct) / (start_mv - end_mv));
        }
    }
    return 0;
}

// -------------------------------------------------------------------------
// Smoothed Output (public)
// -------------------------------------------------------------------------
uint8_t battery_monitor_get_percentage(void) {
    uint8_t current_sample = get_raw_battery_percent(); 

    sample_history[sample_index] = current_sample;
    sample_index++;

    if (sample_index >= NUM_SAMPLES) {
        sample_index = 0;
        array_filled = true; 
    }

    if (!array_filled) {
        // Just return the raw value while booting up so we don't send 0
        last_reported_percentage = current_sample;
        return current_sample;
    }

    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += sample_history[i];
    }
    
    uint8_t true_average = (uint8_t)(sum / NUM_SAMPLES);

    // Hysteresus filter
    // If this is our very first time calculating the average, lock it in.
    if (last_reported_percentage == 255) {
        last_reported_percentage = true_average;
    } 
    // Otherwise, ONLY updating the reported value if the true average has 
    // drifted by MORE than 1% away from the last reported value.
    else {
        // We cast to int to safely check absolute difference without unsigned wrapping
        int diff = (int)true_average - (int)last_reported_percentage;
        
        // If the change is +2% or -2%, accept the new value
        if (diff >= 2 || diff <= -2) {
            last_reported_percentage = true_average;
        }
    }

    // Return the locked-in value
    return last_reported_percentage;
}