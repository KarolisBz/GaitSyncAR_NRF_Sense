#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Pin 29 corresponds to P0.29 */
#define LED_PIN 29
#define SLEEP_TIME_MS 1000

int main(void)
{
    /* Grab the hardware device for GPIO Port 0 */
    const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    /* Check if the GPIO port is ready */
    if (!device_is_ready(gpio0_dev)) {
        return 0;
    }

    /* Configure Pin 29 as an output */
    int ret = gpio_pin_configure(gpio0_dev, LED_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    /* Infinite blink loop */
    while (1) {
        gpio_pin_toggle(gpio0_dev, LED_PIN);
        k_msleep(SLEEP_TIME_MS);
    }
    
    return 0;
}