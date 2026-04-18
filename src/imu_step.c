#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_timer.h>
#include "clock_sync.h"
#include "ble_handler.h"
#include "app_events.h"
#include "imu_step.h"

/* Get the IMU device for the XIAO Sense */
static const struct device *const imu_dev = DEVICE_DT_GET_ANY(st_lsm6dsl);

/* Internal State */
static imu_step_config_t current_config;
static uint64_t last_step_time = 0;

typedef enum {
    PHASE_GROUND = 0,
    PHASE_AIR = 1
} foot_phase_t;

static foot_phase_t current_phase = PHASE_GROUND;

/* forward declarations */
static void init_imu_interrupts(void);

/* methods */
int imu_step_init(imu_step_config_t config) {
    if (!device_is_ready(imu_dev)) {
        printk("FATAL: IMU not ready.\n");
        return -ENODEV;
    }

    current_config = config;
    int64_t thresh_um = (int64_t)(current_config.threshold) * 10000; 
    int64_t floor_um  = (int64_t)(current_config.noise_floor) * 10000;

    // Square them ONCE at boot so the interrupt handler never has to
    current_config.threshold_sq = thresh_um * thresh_um;
    current_config.noise_floor_sq = floor_um * floor_um;

    last_step_time = 0;
    current_phase = PHASE_GROUND;
    
    init_imu_interrupts();
    
    printk("IMU Step Detector Initialized.\n");
    return 0;
}

bool imu_step_update(int64_t mag_sq, uint64_t now_us) {
    bool step_detected = false;

    if (current_phase == PHASE_GROUND) {
        /* Wait for the foot to lift off and swing */
        if (mag_sq < current_config.noise_floor_sq) {
            current_phase = PHASE_AIR;
        }
    } 
    else if (current_phase == PHASE_AIR) {
        /* Foot is in the air. Wait for the hard impact */
        if (mag_sq > current_config.threshold_sq) {
            
            /* Debounce to prevent vibrations from triggering twice */
            if ((now_us - last_step_time) > (current_config.cooldown_ms * 1000)) {
                step_detected = true;
                current_phase = PHASE_GROUND; // Lock it back to the ground state
                last_step_time = now_us;
            }
        }
    }

    return step_detected;
}

// --- THE INTERRUPT HANDLER ---
// This function sits entirely dormant. The CPU uses 0% power on it.
// The instant the IMU feels a shock, the physical wire goes HIGH, and the CPU jumps here.

// --- THE DATA_READY INTERRUPT HANDLER ---
static void imu_data_ready_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    // Capture the exact hardware time the interrupt fired
    uint32_t hardware_now_us = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL3);
    uint64_t current_local_time_us = k_ticks_to_us_floor64(k_uptime_ticks());

    // Fetching data directly from the IMU registers
    struct sensor_value accel[3];
    if (sensor_sample_fetch(dev) < 0) return;
    if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) return;

    // Convert to micro-m/s^2 (integers)
    int32_t ax_um = accel[0].val1 * 1000000 + accel[0].val2;
    int32_t ay_um = accel[1].val1 * 1000000 + accel[1].val2;
    int32_t az_um = accel[2].val1 * 1000000 + accel[2].val2;
    
    // Safely calculate squared magnitude (mag_sq = x^2 + y^2 + z^2)
    int64_t mag_sq = ((int64_t)ax_um * ax_um) + ((int64_t)ay_um * ay_um) + ((int64_t)az_um * az_um);

    // Pass the squared magnitude and local time into the step detection state machine
    if (imu_step_update(mag_sq, current_local_time_us)) {
        
        // If true is returned, the state machine confirmed a valid step!
        uint64_t true_global_step_time_us = global_sync_baseline_us + hardware_now_us;

        // Add step event to event queue
        app_event_t step_event;
        step_event.type = EVENT_STEP_DETECTED;
        step_event.step_timestamp = true_global_step_time_us;
        
        // Use K_NO_WAIT since we are in an interrupt context
        k_msgq_put(&app_msgq, &step_event, K_NO_WAIT);
    }
}

// --- INITIALIZATION ---
static void init_imu_interrupts(void) {
    if (!device_is_ready(imu_dev)) {
        printk("FATAL: IMU not ready.\n");
        return;
    }

    struct sensor_value odr_attr;
    
    // 1.2 ms between samples (approx 833 Hz)
    odr_attr.val1 = 833; 
    odr_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Warning: Could not set IMU sample rate.\n");
    } else {
        printk("IMU Sample rate cranked to %d Hz.\n", odr_attr.val1);
    }
    // ----------------------------------------------------

    struct sensor_value pulse_cfg;
    pulse_cfg.val1 = 0; 
    sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_CONFIGURATION, &pulse_cfg);

    // Set the trigger to DATA_READY
    struct sensor_trigger data_trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    if (sensor_trigger_set(imu_dev, &data_trigger, imu_data_ready_handler) < 0) {
        printk("FATAL: Could not configure DATA_READY trigger.\n");
        return;
    }
    
    printk("IMU DATA_READY hardware interrupt armed.\n");
}

void imu_step_reset_trigger(void) {
    if (!device_is_ready(imu_dev)) {
        return;
    }

    // Let the radio peripheral and I2C bus settle before acting
    k_msleep(5); 

    struct sensor_trigger data_trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    // Wiping internal GPIO state and unmasks stuck interrupts
    sensor_trigger_set(imu_dev, &data_trigger, NULL);

    // Flushing IMU BUS
    sensor_sample_fetch(imu_dev);

    // Zephyr cleanly attaches the interrupt again
    sensor_trigger_set(imu_dev, &data_trigger, imu_data_ready_handler);
}