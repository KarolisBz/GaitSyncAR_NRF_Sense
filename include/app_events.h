#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

// --- QUEUE CONFIGURATION ---
#define APP_MSGQ_MAX_EVENTS 10
#define APP_MSGQ_ALIGNMENT  8

// --- EVENT DEFINITIONS ---
typedef enum {
    EVENT_STEP_DETECTED,
    EVENT_SYNC_COMPLETED,
    EVENT_FLUSH_IMU,
    EVENT_METRONOME_SYNC
} app_event_type_t;

typedef struct {
    app_event_type_t type;

    // saving memory, we never use both in 1 event
    union {
        uint64_t step_timestamp; 
        
        struct {
            bool play;
            uint8_t bpm;
            uint16_t phase_offset;
            bool is_next_beat_right;
        } metronome;
    };
} app_event_t;

// --- QUEUE DECLARATION ---
extern struct k_msgq app_msgq;

#endif // APP_EVENTS_H