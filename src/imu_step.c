#include "imu_step.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/* Get the IMU device for the XIAO Sense */
static const struct device *const imu_dev = DEVICE_DT_GET_ANY(st_lsm6dsl);

/* Internal State */
static imu_step_config_t current_config;
static uint64_t last_step_time = 0;
static bool is_stepping = false;

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

bool imu_step_update(int32_t *out_magnitude) {
    struct sensor_value accel[3];
    uint64_t now = k_uptime_get();
    bool step_detected = false;

    // Fetch fresh data from the IMU hardware
    if (sensor_sample_fetch(imu_dev) < 0) {
        return false; /* Hardware read failed this cycle */
    }
    
    if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) {
        return false;
    }

    // Converting Zephyr structs to standard m/s^2 doubles
    double ax = sensor_value_to_double(&accel[0]);
    double ay = sensor_value_to_double(&accel[1]);
    double az = sensor_value_to_double(&accel[2]);

    // Calculating 3D Vector Magnitude
    double magnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
    int32_t scaled_mag = (int32_t)(magnitude * 100);

    // Passing the magnitude out if the caller asked for it
    if (out_magnitude != NULL) {
        *out_magnitude = scaled_mag;
    }

    // Algorithm Step 1: Hysteresis Reset
    if (is_stepping && (scaled_mag < current_config.noise_floor)) {
        is_stepping = false;
    }

    // Algorithm Step 2: Trigger & Cooldown Check
    if (!is_stepping && (scaled_mag > current_config.threshold)) {
        if ((now - last_step_time) > current_config.cooldown_ms) {
            step_detected = true;
            is_stepping = true;    // Lock state
            last_step_time = now;  // Record timestamp
        }
    }

    return step_detected;
}