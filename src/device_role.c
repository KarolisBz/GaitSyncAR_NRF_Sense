#include "device_role.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_ficr.h>

/* --- CONFIGURATION --- */
static const uint32_t PRIMARY_IDS[] = {
    0x261F9881,
};

static device_role_t current_role = ROLE_UNKNOWN;

void device_role_init(void) {
    // Reading the lower 32 bits of the factory-burned MAC address
    uint32_t hardware_id = NRF_FICR->DEVICEADDR[0];
    
    printk("\n==================================\n");
    printk("  HARDWARE ID: 0x%08X\n", hardware_id);
    
    for (size_t i = 0; i < sizeof(PRIMARY_IDS) / sizeof(PRIMARY_IDS[0]); i++) {
        if (hardware_id == PRIMARY_IDS[i]) {
            current_role = ROLE_PRIMARY;
            printk("  ROLE ASSIGNED: PRIMARY (Master)\n");
            return;
        }
    }
    current_role = ROLE_SECONDARY;
    printk("  ROLE ASSIGNED: SECONDARY (Slave)\n");
    printk("==================================\n\n");
}

device_role_t device_role_get(void) {
    return current_role;
}

bool device_role_is_primary(void) {
    return (current_role == ROLE_PRIMARY);
}