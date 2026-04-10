#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>

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