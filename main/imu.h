#pragma once

/*
 * imu.h — MPU6050 Driver Interface
 *
 * Attitude estimation using complementary filter.
 * Roll/Pitch: accelerometer + gyro fusion (CF_ALPHA = 0.98)
 * Yaw:        gyro integration only — drifts over time. Acceptable
 *             for short flights or when magnetometer is not available.
 *
 * Physical axis convention (standard MPU6050 flat mount, chip facing up):
 *   X → Forward (nose direction)
 *   Y → Left
 *   Z → Up
 *
 *   Roll  positive = right side down
 *   Pitch positive = nose down
 *   Yaw   positive = clockwise from above
 *
 * If your sensor is mounted differently, adjust axis signs in imu.c.
 *
 * I2C: GPIO4 (SDA), GPIO6 (SCL), 400kHz, address 0x68 (AD0 = GND)
 * Task: 100Hz on Core 1
 * Calibration: 300 samples (~3s) at boot — drone must be flat and still
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// Configuration — adjust here, not in imu.c
// ============================================================
#define IMU_I2C_SDA_GPIO        4           // GPIO4
#define IMU_I2C_SCL_GPIO        6           // GPIO6
#define IMU_I2C_FREQ_HZ         400000      // Fast mode
#define IMU_MPU6050_ADDR        0x68        // AD0 = GND

#define IMU_CF_ALPHA            0.98f       // Complementary filter weight on gyro
#define IMU_TASK_PERIOD_MS      10          // 100Hz (1 FreeRTOS tick @ 100Hz config)
#define IMU_CALIB_SAMPLES       300         // Samples collected at boot (~3 seconds)
#define IMU_HEALTH_TIMEOUT_MS   200         // Declare unhealthy if no update for 200ms

// ============================================================
// Shared Data Structure
// This struct is what flight control will read.
// All fields valid only when calibrated == true && healthy == true.
// ============================================================
typedef struct {
    // Attitude (degrees)
    float roll;         // deg, positive = right side down
    float pitch;        // deg, positive = nose down
    float yaw;          // deg, gyro-integrated, drifts — reset on arm

    // Body rates (degrees/second)
    float gyro_x;       // Roll rate
    float gyro_y;       // Pitch rate
    float gyro_z;       // Yaw rate

    // Calibrated accelerations (g)
    float accel_x;
    float accel_y;
    float accel_z;

    // Diagnostics
    float       temp_c;         // MPU6050 die temperature
    bool        calibrated;     // True after boot calibration completes
    bool        healthy;        // False if I2C errors or task stalled
    uint32_t    update_count;   // Increments every successful read
} imu_data_t;

// ============================================================
// Public API
// ============================================================

/**
 * @brief Initialize I2C bus, configure MPU6050, run boot calibration,
 *        and start the 100Hz IMU task on Core 1.
 *
 * BLOCKING: Returns after ~3 seconds (calibration).
 * Drone must be flat and stationary during this call.
 *
 * @return ESP_OK on success, error code otherwise (check logs).
 */
esp_err_t imu_init(void);

/**
 * @brief Thread-safe copy of the latest IMU data.
 *
 * Safe to call from any task. Will block briefly on mutex.
 * @param out  Pointer to caller-allocated imu_data_t. Must not be NULL.
 */
void imu_get_data(imu_data_t *out);

/**
 * @brief Returns true if IMU task is running and producing fresh data.
 *
 * Checks timestamp of last successful read against IMU_HEALTH_TIMEOUT_MS.
 * Call this periodically from flight control as a safety check.
 */
bool imu_is_healthy(void);

/**
 * @brief Reset accumulated yaw to 0.
 *
 * Call this when arming so the drone has a known heading reference.
 */
void imu_reset_yaw(void);

#ifdef __cplusplus
}
#endif