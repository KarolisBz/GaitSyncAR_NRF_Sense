#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include "led_pwm.h"

// Grabing hardware bindings from the DeviceTree overlay
static const struct pwm_dt_spec led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
static const struct pwm_dt_spec led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));

// Defining timing in nanoseconds
#define PWM_PERIOD_NS  10000000  // 10ms (100Hz)
#define PWM_PULSE_NS     500000  // 0.5ms (5% Duty Cycle)

void led_pwm_init(void) {
    // Checking if the hardware is ready
    if (!pwm_is_ready_dt(&led1) || !pwm_is_ready_dt(&led2)) {
        printk("Error: PWM hardware is not ready.\n");
        return;
    }

    // Commanding hardware chip to start pulsing. 
    // No cpu burden after this, the PWM controller handles the timing autonomously.
    pwm_set_dt(&led1, PWM_PERIOD_NS, PWM_PULSE_NS);
    pwm_set_dt(&led2, PWM_PERIOD_NS, PWM_PULSE_NS);
    
    printk("Hardware PWM Module Initialized (100Hz, 5%% Brightness)\n");
}