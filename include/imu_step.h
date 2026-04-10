#ifndef IMU_STEP_H
#define IMU_STEP_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Configuration parameters for the IMU step detection algorithm.
 */
typedef struct {
    int32_t threshold;       /* Trigger magnitude (e.g. 1500 for 15.00 m/s^2) */
    int32_t noise_floor;     /* Reset magnitude (e.g. 1100 for 11.00 m/s^2) */
    uint32_t cooldown_ms;    /* Minimum milliseconds allowed between two steps */
} imu_step_config_t;

/**
 * @brief Initializes the IMU hardware and the step algorithm.
 * * @param config The threshold and timing configuration.
 * @return 0 on success, or a negative error code if the IMU is missing.
 */
int imu_step_init(imu_step_config_t config);

/**
 * @brief Fetches fresh IMU data, calculates magnitude, and checks for a step.
 * * @param out_magnitude Pointer to store the calculated magnitude (optional, pass NULL if you don't need it).
 * @return true if a NEW footstep was detected at this exact moment.
 */
bool imu_step_update(int32_t *out_magnitude);

#endif /* IMU_STEP_H */