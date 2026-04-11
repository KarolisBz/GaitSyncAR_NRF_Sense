#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include "battery_monitor.h"

/* This grabs the configuration directly from your new overlay */
static const struct adc_dt_spec adc_chan0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

int battery_monitor_init(void) {
    if (!adc_is_ready_dt(&adc_chan0)) {
        return -ENODEV;
    }

    printk("  -> Enabling Battery Monitor...\n");
    return adc_channel_setup_dt(&adc_chan0);
}

uint32_t battery_monitor_get_mv(void) {
    int16_t buf;
    struct adc_sequence seq = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    /* This helper automatically pulls resolution/gain from the overlay */
    adc_sequence_init_dt(&adc_chan0, &seq);
    adc_read_dt(&adc_chan0, &seq);

    int32_t mv = (int32_t)buf;
    adc_raw_to_millivolts_dt(&adc_chan0, &mv);
    
    return (uint32_t)(mv * 2); // Multiplier for 1:1 resistor divider
}

// Millivolts vs Percentage for LiFePO4
static const uint32_t lut_mv[] = { 3600, 3400, 3300, 3200, 3100, 2600 };
static const uint8_t lut_pct[] = { 100,  99,   70,   30,   10,   0    };
#define LUT_SIZE (sizeof(lut_pct) / sizeof(lut_pct[0]))

uint8_t battery_monitor_get_percentage(void) {
    uint32_t mv = battery_monitor_get_mv();

    // Checking Out-of-Bounds
    if (mv >= lut_mv[0]) return 100;
    if (mv <= lut_mv[LUT_SIZE - 1]) return 0;

    // Finding the segment in the table
    for (int i = 0; i < LUT_SIZE - 1; i++) {
        if (mv >= lut_mv[i + 1]) {
            // Found the segment between index i and i+1
            uint32_t start_mv = lut_mv[i];
            uint32_t end_mv = lut_mv[i + 1];
            uint8_t start_pct = lut_pct[i];
            uint8_t end_pct = lut_pct[i + 1];

            // Linear Interpolation within this specific segment
            // formula: pct = end_pct + (mv - end_mv) * (start_pct - end_pct) / (start_mv - end_mv)
            return end_pct + (uint8_t)((mv - end_mv) * (start_pct - end_pct) / (start_mv - end_mv));
        }
    }

    return 0;
}