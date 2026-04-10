#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "nfc_handler.h"

int main(void)
{
    printk("Starting Seeed Sense Project...\n");

    int err = nfc_handler_init();
    if (err) {
        printk("NFC Handler failed to initialize (err %d)\n", err);
    } else {
        printk("NFC Emulation running. Tap your phone!\n");
    }

    while (1) {
        k_sleep(K_FOREVER);
    }
    
    return 0;
}