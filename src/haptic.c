#include "haptic.h"
#include <zephyr/drivers/gpio.h>
#include "device_role.h"

// --------- CONFIGURATION ---------
#define ERM_PULSE_DURATION_MS 235 
#define HAPTIC_NODE DT_ALIAS(haptic) 
static const struct gpio_dt_spec haptic_pin = GPIO_DT_SPEC_GET(HAPTIC_NODE, gpios);

// --------- KERNEL OBJECTS ---------
static struct k_timer bpm_timer;
static struct k_work_delayable motor_off_work;

// --------- STATES ---------
static bool next_beat_is_right = false;

// --------- PRIVATE FUNCTIONS ---------

static void motor_off_handler(struct k_work *work)
{
    gpio_pin_set_dt(&haptic_pin, 0);
}

static void bpm_timer_expiry_fn(struct k_timer *timer_id)
{
    bool i_am_right_sensor = device_role_is_primary();
    
    // Turn motor ON if it's the correct beat for this sensor
    if (next_beat_is_right == i_am_right_sensor) {
        gpio_pin_set_dt(&haptic_pin, 1);
        k_work_schedule(&motor_off_work, K_MSEC(ERM_PULSE_DURATION_MS));
    }

    // inverse the state for the next beat
    next_beat_is_right = !next_beat_is_right;
}

// --------- PUBLIC API ---------

int haptic_init(void)
{
    if (!gpio_is_ready_dt(&haptic_pin)) {
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&haptic_pin, GPIO_OUTPUT_INACTIVE);
    if (err != 0) {
        return err;
    }

    k_timer_init(&bpm_timer, bpm_timer_expiry_fn, NULL);
    k_work_init_delayable(&motor_off_work, motor_off_handler);

    return 0;
}

void haptic_sync(bool play, uint8_t bpm, uint16_t phase_offset_ms, bool is_next_beat_right)
{
    if (!play || bpm == 0) {
        k_timer_stop(&bpm_timer);
        k_work_cancel_delayable(&motor_off_work);
        gpio_pin_set_dt(&haptic_pin, 0); 
        return;
    }

    next_beat_is_right = is_next_beat_right;

    uint32_t period_ms = 60000 / bpm;
    uint32_t first_fire_ms = period_ms;
    if (phase_offset_ms < period_ms) {
        first_fire_ms = period_ms - phase_offset_ms;
    }

    // edge case handler
    if (phase_offset_ms < ERM_PULSE_DURATION_MS) {
        // If we are inside the ms window of the CURRENT beat
        // We must evaluate against the OPPOSITE of the next beat.
        bool current_beat_is_right = !next_beat_is_right;
        bool i_am_right_sensor = device_role_is_primary();

        if (current_beat_is_right == i_am_right_sensor) {
            gpio_pin_set_dt(&haptic_pin, 1);
            uint32_t remaining = ERM_PULSE_DURATION_MS - phase_offset_ms;
            k_work_schedule(&motor_off_work, K_MSEC(remaining));
        } else {
            gpio_pin_set_dt(&haptic_pin, 0);
        }
    } else {
        gpio_pin_set_dt(&haptic_pin, 0);
    }

    k_timer_start(&bpm_timer, K_MSEC(first_fire_ms), K_MSEC(period_ms));
}