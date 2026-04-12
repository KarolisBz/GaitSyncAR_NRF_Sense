#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <zephyr/types.h>

/**
 * @brief Initializes the Bluetooth stack, the Nordic UART Service (NUS),
 * and starts advertising to mobile devices.
 * * @return 0 on success, negative error code on failure.
 */
int ble_handler_init(void);

/**
 * @brief Sends a string or byte array to the connected Unity app via NUS.
 * * @param data Pointer to the data buffer.
 * @param len  Length of the data.
 * @return 0 on success, negative error code on failure.
 */
int ble_handler_send(const uint8_t *data, uint16_t len);

/**
 * @brief Sends a step event to the connected Unity app via NUS.
 */
void send_step_event(void);

/**
 * @brief Sends a battery level event to the connected Unity app via NUS.
 * @param battery_level Battery level as a percentage (0-100).
 */
void send_battery_event(uint8_t battery_level);

/// @brief Sends a synchronization acknowledgment event to the connected Unity app via NUS.
/// @param baseline_time The global sync baseline time in milliseconds to include in the event.
void send_sync_ack_event(uint32_t baseline_time);

#endif /* BLE_HANDLER_H */