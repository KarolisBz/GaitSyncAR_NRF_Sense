#ifndef DEVICE_ROLE_H
#define DEVICE_ROLE_H

#include <stdbool.h>

// available roles for the device, determined by hardware ID
// Primary is the master node, secondary is the follower
// primary should be the right ankle sensor, secondary should be the left ankle sensor
typedef enum {
    ROLE_UNKNOWN = 0,
    ROLE_PRIMARY,
    ROLE_SECONDARY
} device_role_t;

/**
 * @brief Reads the hardware ID and assigns the device role.
 */
void device_role_init(void);

/**
 * @brief Returns the current role of the device.
 */
device_role_t device_role_get(void);

/**
 * @brief Helper function. Returns true if this board is the Primary node.
 */
bool device_role_is_primary(void);

#endif /* DEVICE_ROLE_H */