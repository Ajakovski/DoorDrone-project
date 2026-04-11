#pragma once

/*
 * flight_control.h — Cascade PID Flight Controller (Phase 3)
 *
 * Emergency landing procedure replaces immediate disarm for all
 * communication-loss and remote emergency trigger scenarios.
 *
 * State machine:
 *
 *   DISARMED
 *     └─[arm()]──► ARMED_NORMAL
 *                    │
 *                    ├─[link loss / MQTT drop / UDP flags bit 1 (0x02)]──►
 *                    │
 *                 EMERGENCY_LEVEL  (hold throttle, force roll/pitch → 0°, max 2s)
 *                    │
 *                    ├─[|roll|<5° AND |pitch|<5°]  OR  [timeout 2s]──►
 *                    │
 *                 EMERGENCY_LAND   (fixed FC_LANDING_THROTTLE_US for FC_LANDING_DURATION_MS)
 *                    │
 *                    └─[duration elapsed]──► DISARMED
 *
 * IMU failure is out of scope — IMU is assumed healthy during all
 * emergency procedures.
 *
 * Manual disarm (Blynk V0) always goes directly to DISARMED immediately —
 * it is a deliberate pilot command, not a fault condition.
 *
 * UDP flags byte:
 *   bit 0 (0x01) — reserved (future use)
 *   bit 1 (0x02) — EMERGENCY: triggers emergency landing procedure
 *
 * Blynk virtual pin map:
 *   V0  → ARM/DISARM (1=arm, 0=immediate disarm)
 *   V2  → Roll  telemetry (°)       [read-only]
 *   V3  → Pitch telemetry (°)       [read-only]
 *   V4  → Yaw   telemetry (°)       [read-only]
 *   V5  → Outer loop Kp (roll=pitch, float 0.0–20.0)
 *   V6  → LED
 *   V7  → ESC calibration trigger
 *   V8  → Motor test select (1–4)
 *   V9  → Motor test pulse (µs)
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// Frame orientation correction — negate if axis is reversed
// ============================================================
#define FC_SIGN_ROLL    (+1.0f)
#define FC_SIGN_PITCH   (+1.0f)
#define FC_SIGN_YAW     (+1.0f)

// ============================================================
// ESC GPIO assignments
// ============================================================
#define FC_ESC1_GPIO    1       // M1 Front-Right CW
#define FC_ESC2_GPIO    5       // M2 Front-Left  CCW
#define FC_ESC3_GPIO    10      // M3 Rear-Right  CCW
#define FC_ESC4_GPIO    11      // M4 Rear-Left   CW

// ============================================================
// ESC pulse parameters
// ============================================================
#define FC_ESC_MIN_US           1000
#define FC_ESC_MAX_US           2000
#define FC_MOTOR_MIN_ARMED_US   1050
#define FC_THROTTLE_IDLE_US     1080

// ============================================================
// Emergency landing parameters — tune on bench before flight
//
// FC_LANDING_THROTTLE_US:
//   Motors spin but cannot sustain hover — drone sinks steadily.
//   Find this with props on over a soft surface.
//   Start at 1100, increase until drone barely sinks, back off 50us.
//
// FC_LANDING_DURATION_MS:
//   Time at landing throttle before automatic disarm.
//   5000ms = 5 seconds. Increase if regularly flying above ~2m.
//
// FC_EMERGENCY_LEVEL_TIMEOUT_MS:
//   Max time in leveling phase before proceeding to land regardless.
//
// FC_LEVEL_THRESHOLD_DEG:
//   Attitude "close enough to level" to begin descent.
// ============================================================
#define FC_LANDING_THROTTLE_US          1250
#define FC_LANDING_DURATION_MS          5000
#define FC_EMERGENCY_LEVEL_TIMEOUT_MS   2000
#define FC_LEVEL_THRESHOLD_DEG          5.0f

// ============================================================
// Outer loop (angle) — P-only
// ============================================================
#define FC_OUTER_RATE_LIMIT     200.0f
#define FC_ANGLE_ROLL_KP        3.0f
#define FC_ANGLE_PITCH_KP       3.0f
#define FC_MAX_ANGLE_DEG        45.0f

// ============================================================
// Inner loop (rate) — PID (I and D currently zero)
// ============================================================
#define FC_ROLL_KP      0.8f
#define FC_ROLL_KI      0.0f
#define FC_ROLL_KD      0.005f

#define FC_PITCH_KP     0.8f
#define FC_PITCH_KI     0.0f
#define FC_PITCH_KD     0.005f

#define FC_YAW_KP       1.6f
#define FC_YAW_KI       0.0f
#define FC_YAW_KD       0.0f

// ============================================================
// PID limits
// ============================================================
#define FC_PID_OUTPUT_LIMIT     300.0f
#define FC_PID_INTEGRAL_LIMIT   150.0f

// ============================================================
// Safety timing
// ============================================================
#define FC_SETPOINT_STALE_DISARM_MS     600
#define FC_PERIOD_MS                     10     // 100 Hz

// ============================================================
// Flight state enum
// ============================================================
typedef enum {
    FC_STATE_DISARMED           = 0,
    FC_STATE_ARMED_NORMAL       = 1,
    FC_STATE_EMERGENCY_LEVEL    = 2,    // Phase 1: level drone, hold throttle
    FC_STATE_EMERGENCY_LAND     = 3,    // Phase 2: fixed landing throttle then disarm
} fc_flight_state_t;

// ============================================================
// Status struct — thread-safe snapshot for telemetry and logging
// ============================================================
typedef struct {
    fc_flight_state_t state;

    // Outer loop (angle controller)
    float    angle_sp[2];           // Setpoint   [0=roll, 1=pitch] (deg)
    float    angle_meas[2];         // Measured   [0=roll, 1=pitch] (deg)
    float    angle_err[2];          // Error      sp - meas         (deg)
    float    angle_pid_out[2];      // P output → rate setpoint     (deg/s)

    // Inner loop (rate controller)
    float    rate_sp[3];            // Setpoint   [0=roll,1=pitch,2=yaw] (deg/s)
    float    rate_meas[3];          // Measured   raw gyro               (deg/s)
    float    rate_pid_out[3];       // PID output → mixer input          (us)

    // Motor outputs
    uint16_t motor_us[4];           // PWM [M1, M2, M3, M4] (us)

    // Emergency timing — ms remaining in current emergency phase, 0 if normal
    uint32_t emergency_phase_ms_remaining;

    uint32_t loop_count;
} fc_status_t;

// ============================================================
// Public API
// ============================================================

/**
 * @brief Initialise MCPWM, PID instances, and start 100 Hz task on Core 1.
 *        Call AFTER imu_init().
 */
esp_err_t flight_control_init(void);

/**
 * @brief Arm the drone. Refuses if IMU unhealthy or not in DISARMED state.
 *        Resets yaw reference and all PID state.
 */
void flight_control_arm(void);

/**
 * @brief Immediate disarm — motors to minimum, all PID state reset.
 *        USE ONLY for intentional pilot-commanded disarm (Blynk V0).
 *        For fault conditions, call flight_control_trigger_emergency() instead.
 */
void flight_control_disarm(void);

/**
 * @brief Trigger the emergency landing procedure.
 *        Phase 1: level drone (max FC_EMERGENCY_LEVEL_TIMEOUT_MS).
 *        Phase 2: FC_LANDING_THROTTLE_US for FC_LANDING_DURATION_MS.
 *        Disarms automatically after Phase 2 completes.
 *        Safe to call from any task. No-op if already in emergency or disarmed.
 */
void flight_control_trigger_emergency(void);

/** @brief Returns true only when in ARMED_NORMAL state. */
bool flight_control_is_armed(void);

/** @brief Returns current flight state. */
fc_flight_state_t flight_control_get_state(void);

/** @brief Thread-safe status snapshot. */
void flight_control_get_status(fc_status_t *out);

/**
 * @brief Update outer (angle) P gains at runtime via Blynk V5.
 *        Resets all PID state. Safe to call while armed.
 */
void flight_control_set_angle_gains(float roll_kp, float pitch_kp);

/**
 * @brief Update inner (rate) PID gains at runtime.
 *        Resets all PID state.
 */
void flight_control_set_rate_gains(
    float roll_kp,  float roll_ki,  float roll_kd,
    float pitch_kp, float pitch_ki, float pitch_kd,
    float yaw_kp,   float yaw_ki,   float yaw_kd);

/**
 * @brief ESC calibration sequence (blocking ~10s). Must be DISARMED.
 */
void flight_control_calibrate_escs(void);

/**
 * @brief Spin one motor at a specific pulse, all others at minimum.
 *        For bench testing only. DISARMED state required.
 */
void flight_control_motor_test(int motor_num, uint16_t pulse_us);

#ifdef __cplusplus
}
#endif