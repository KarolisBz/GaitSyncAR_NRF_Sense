#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

/**
 * @brief Initialize the ADC for battery monitoring on P0.03
 * @return 0 on success, negative error code otherwise.
 */
int battery_monitor_init(void);

/**
 * @brief Read the battery voltage through the 1:1 divider
 * @return The battery voltage in millivolts (mV)
 */
uint32_t battery_monitor_get_mv(void);

#endif // BATTERY_MONITOR_H