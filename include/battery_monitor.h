#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

#define BATT_MAX_MV 3600
#define BATT_MIN_MV 2500

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

/**
 * @brief Get battery capacity as a percentage (0-100)
 * @return uint8_t percentage
 */
uint8_t battery_monitor_get_percentage(void);

#endif // BATTERY_MONITOR_H