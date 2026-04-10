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

#endif /* BLE_HANDLER_H */