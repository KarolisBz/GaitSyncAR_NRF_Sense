#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <zephyr/kernel.h>

// --- QUEUE CONFIGURATION ---
#define APP_MSGQ_MAX_EVENTS 10
#define APP_MSGQ_ALIGNMENT  8

// --- EVENT DEFINITIONS ---
typedef enum {
    EVENT_STEP_DETECTED,
    EVENT_SYNC_COMPLETED
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    uint64_t step_timestamp; 
} app_event_t;

// --- QUEUE DECLARATION ---
extern struct k_msgq app_msgq;

#endif // APP_EVENTS_H