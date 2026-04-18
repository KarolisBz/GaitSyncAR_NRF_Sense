#ifndef IMU_STEP_H
#define IMU_STEP_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Configuration structure for the IMU step detection algorithm.
 * * Contains raw thresholds and temporal constraints for the gait state machine.
 * The _sq members are pre-calculated during init to avoid costly square root 
 * operations in high-frequency interrupt contexts.
 */
typedef struct {
    int32_t threshold;
    int32_t noise_floor;
    uint32_t cooldown_ms;
    
    // Auto-calculated integer squared thresholds
    int64_t threshold_sq;    
    int64_t noise_floor_sq;
} imu_step_config_t;

/**
 * @brief Initializes the LSM6DSL sensor and prepares the step detection algorithm.
 * * Attaches to the sensor device, configures the 833Hz ODR (Output Data Rate),
 * enables pulsed interrupts on the INT1 pin, and pre-calculates the squared 
 * thresholds for the magnitude logic.
 * * @param config The user-defined thresholds and cooldown parameters.
 * @return 0 on success, or a negative error code if the sensor cannot be reached.
 */
int imu_step_init(imu_step_config_t config);

#endif // IMU_STEP_H