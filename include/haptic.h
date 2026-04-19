#ifndef HAPTIC_H
#define HAPTIC_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

// Initializes the timer, work queues, and GPIO for the haptic motor
int haptic_init(void);

/** @brief Syncs the hardware timer to Unity.
 * play: 1 to run, 0 to stop
 * bpm: Beats per minute
 * phase_offset_ms: How far into the current beat cycle we are
 */
void haptic_sync(bool play, uint8_t bpm, uint16_t phase_offset_ms, bool is_next_beat_right);

#endif // HAPTIC_H