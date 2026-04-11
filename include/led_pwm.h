#ifndef LED_PWM_H
#define LED_PWM_H

// Initializes the GPIO pins and starts the 100Hz background PWM timer using the hardware PWM controller.
// After this is called, the LEDs will pulse autonomously without any further CPU intervention.
void led_pwm_init(void);

#endif // LED_PWM_H