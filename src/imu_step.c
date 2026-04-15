#include "imu_step.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <hal/nrf_timer.h>
#include "clock_sync.h"
#include "ble_handler.h"

/* Get the IMU device for the XIAO Sense */
static const struct device *const imu_dev = DEVICE_DT_GET_ANY(st_lsm6dsl);

/* Internal State */
static imu_step_config_t current_config;
static uint64_t last_step_time = 0;
static bool is_stepping = false;
#define STEP_THRESHOLD_MAG 2700
#define STEP_COOLDOWN_US 500000 // 500ms debounce in microseconds
extern struct k_msgq step_msgq;

int imu_step_init(imu_step_config_t config) {
    // Checking if the IMU device is ready before proceeding
    if (!device_is_ready(imu_dev)) {
        printk("FATAL: IMU device not ready.\n");
        return -ENODEV;
    }

    // waking up IMU from sleep mode
    struct sensor_value odr_attr;
    odr_attr.val1 = 104; /* 104 Hz */
    odr_attr.val2 = 0;
    
    int err = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, 
                              SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
    if (err) {
        printk("Warning: Failed to set IMU sampling frequency (err %d)\n", err);
    }

    // configuring step detection parameters
    current_config = config;
    last_step_time = 0;
    is_stepping = false;
    
    printk("IMU Step Detector Initialized.\n");
    return 0;
}

static int32_t previous_mag = 980; // Start assumed at 1G
typedef enum {
    PHASE_GROUND = 0,
    PHASE_AIR = 1
} foot_phase_t;

static foot_phase_t current_phase = PHASE_GROUND;

bool imu_step_update(int32_t *out_magnitude) {
    struct sensor_value accel[3];
    uint64_t now = k_ticks_to_us_floor64(k_uptime_ticks());
    bool step_detected = false;

    if (sensor_sample_fetch(imu_dev) < 0) return false;
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) return false;

    double ax = sensor_value_to_double(&accel[0]);
    double ay = sensor_value_to_double(&accel[1]);
    double az = sensor_value_to_double(&accel[2]);

    /* Calculate raw magnitude (1G = ~980) */
    double magnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
    int32_t scaled_mag = (int32_t)(magnitude * 100);

    if (out_magnitude != NULL) {
        *out_magnitude = scaled_mag;
    }

    if (current_phase == PHASE_GROUND) {
        /* Wait for the foot to lift off and swing (Magnitude drops below gravity) */
        if (scaled_mag < current_config.noise_floor) {
            current_phase = PHASE_AIR;
        }
    } 
    else if (current_phase == PHASE_AIR) {
        /* Foot is in the air. Wait for the hard impact (Magnitude spikes above gravity) */
        if (scaled_mag > current_config.threshold) {
            
            /* 3. Debounce to prevent vibrations from triggering twice */
            if ((now - last_step_time) > (current_config.cooldown_ms * 1000)) {
                step_detected = true;
                current_phase = PHASE_GROUND; // Lock it back to the ground state
                last_step_time = now;
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
    nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE3);
    uint32_t hardware_now_us = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL3);
    uint64_t current_local_time_us = k_ticks_to_us_floor64(k_uptime_ticks());

    // Fetching data directly from the IMU registers
    struct sensor_value accel[3];
    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    double ax = sensor_value_to_double(&accel[0]);
    double ay = sensor_value_to_double(&accel[1]);
    double az = sensor_value_to_double(&accel[2]);

    // Calculating magnitude
    double magnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
    int32_t scaled_mag = (int32_t)(magnitude * 100);

    // Checking if the desk strike happened
    if (scaled_mag > STEP_THRESHOLD_MAG) {

        // Debounce
        if ((current_local_time_us - last_step_time) > STEP_COOLDOWN_US) {
            last_step_time = current_local_time_us;
            uint64_t true_global_step_time_us = global_sync_baseline_us + hardware_now_us;

            k_msgq_put(&step_msgq, &true_global_step_time_us, K_NO_WAIT);

            printk("\n--- STEP DETECTED ---\n");
            printk("Absolute Global Step Time: %llu us\n", true_global_step_time_us); 
        }
    }
}

// --- INITIALIZATION ---
void init_imu_interrupts(void) {
    if (!device_is_ready(imu_dev)) {
        printk("FATAL: IMU not ready.\n");
        return;
    }

    struct sensor_value odr_attr;
    
    // 0.6 ms between samples
    odr_attr.val1 = 1666; 
    odr_attr.val2 = 0;

    if (sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Warning: Could not set IMU sample rate.\n");
    } else {
        printk("IMU Sample rate cranked to %d Hz.\n", odr_attr.val1);
    }
    // ----------------------------------------------------

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