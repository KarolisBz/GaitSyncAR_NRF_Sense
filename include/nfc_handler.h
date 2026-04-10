#ifndef NFC_HANDLER_H
#define NFC_HANDLER_H

#include <zephyr/types.h>

/**
 * @brief Initializes Bluetooth to retrieve the MAC address, 
 * sets up the NFC T2T library, and starts emulation.
 * * @return 0 on success, negative error code on failure.
 */
int nfc_handler_init(void);

#endif /* NFC_HANDLER_H */