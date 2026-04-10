#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

int main(void) {
    if (!device_is_ready(gpio0)) {
        return 0;
    }

    /* Initializing the built-in Blue LED on Pin 0.06 */
    gpio_pin_configure(gpio0, 6, GPIO_OUTPUT_ACTIVE);

    /* Waiting 3 seconds for the USB COM ports to show up in Windows */
    k_sleep(K_MSEC(3000));

    while (1) {
        /* checking if crashed, as console broken, If this blinks, the board hasn't crashed! */
        gpio_pin_toggle(gpio0, 6);
        
        /* Print to USB */
        printk("HEARTBEAT: Board is alive and printing!\n");
        
        k_msleep(500);
    }

    return 0;
}