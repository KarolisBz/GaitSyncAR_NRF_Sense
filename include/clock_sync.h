#ifndef CLOCK_SYNC_H
#define CLOCK_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initializes the MPSL timeslot session.
 * * Based on the device role (Primary/Secondary), it registers the 
 * appropriate hardware callbacks to handle raw radio operations.
 */
void clock_sync_init(void);

/**
 * @brief Requests a radio timeslot from the MPSL.
 * * This temporarily pauses BLE activity to allow for high-priority 
 * hardware-to-hardware clock synchronization.
 */
void request_sync_timeslot(void);

/**
 * @brief Hardware-captured timestamp of the last sync event.
 * * On the Secondary device, this value is updated via PPI 
 * the exact microsecond the radio pulse is detected.
 */
extern volatile uint32_t captured_hardware_time_us;

/**
 * @brief Flag indicating if a valid hardware sync has occurred.
 */
extern volatile bool is_hardware_synced;

// This is the global baseline timestamp that all step events will be relative to after sync.
extern volatile uint32_t global_sync_baseline_ms;

#endif // CLOCK_SYNC_H