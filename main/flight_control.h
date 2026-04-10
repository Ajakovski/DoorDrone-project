#pragma once

/*
 * flight_control.h — Cascade PID Flight Controller (Phase 3)
 *
 * Phase 3 adds the outer angle loop (P controller) on top of the Phase 2
 * inner rate PID. The two loops run sequentially every 10 ms inside the
 * same 100 Hz Core 1 task.
 *
 * ── Cascade architecture ────────────────────────────────────────────────
 *
 *   Stick (roll/pitch °, yaw °/s)
 *          │
 *     ┌────▼────────────────────────────────────┐
 *     │  OUTER LOOP — Angle controller (P only)  │  100 Hz
 *     │                                          │
 *     │  input : stick angle  (°)                │
 *     │  sensor: imu.roll / imu.pitch (CF fused) │
 *     │  output: target rate  (°/s)              │
 *     └────┬─────────────────────────────────────┘
 *          │  target rate (°/s)   [clamped ±FC_OUTER_RATE_LIMIT]
 *     ┌────▼────────────────────────────────────┐
 *     │  INNER LOOP — Rate controller (PID)      │  100 Hz
 *     │                                          │
 *     │  input : target rate  (°/s)              │
 *     │  sensor: imu.gyro_x/y/z  (raw gyro)      │
 *     │  output: torque correction (µs)          │
 *     └────┬─────────────────────────────────────┘
 *          │  per-axis correction (µs)
 *     ┌────▼──────────┐
 *     │  Motor Mixer   │  Quad-X
 *     └────┬──────────┘
 *          │  M1–M4 (µs)
 *     MCPWM → ESCs
 *
 * ── Why outer loop is P-only ────────────────────────────────────────────
 *
 *   The outer plant already contains an integrator: angular rate integrates
 *   to angle. Adding I to the outer loop gives double integration, which
 *   causes low-frequency oscillation. The inner I term handles steady-state
 *   disturbances (off-centre CoG, prop imbalance). Outer D is omitted
 *   because the CF filter introduces phase lag that makes D unstable.
 *
 * ── Yaw note ────────────────────────────────────────────────────────────
 *
 *   Yaw has no outer angle loop. The stick yaw_rate is fed directly into
 *   the inner rate PID as a rate setpoint (heading-hold mode requires a
 *   magnetometer, which is not fitted). This is standard for rate-mode yaw.
 *
 * ── Motor Layout (Quad-X, viewed from above) ────────────────────────────
 *
 *   M2 (FL CCW) ──── M1 (FR CW)
 *        │    [FRONT]    │
 *        │     ──X──     │
 *        │               │
 *   M4 (RL CW)  ──── M3 (RR CCW)
 *
 *   M1 → ESC1 → GPIO1      M3 → ESC3 → GPIO10
 *   M2 → ESC2 → GPIO5      M4 → ESC4 → GPIO11
 *
 * ── Quad-X motor mixing ─────────────────────────────────────────────────
 *
 *   Sign convention (correct once confirmed in Phase 2):
 *     roll  positive = right side down  (left motors speed up)
 *     pitch positive = nose down        (rear motors speed up)
 *     yaw   positive = CW from above    (CCW motors speed up)
 *
 *   M1 = thr - roll - pitch - yaw ALL INVERTED FOR HOVER MODE
 *   M2 = thr + roll - pitch + yaw
 *   M3 = thr - roll + pitch + yaw
 *   M4 = thr + roll + pitch - yaw
 *
 *   Negate FC_SIGN_ROLL / _PITCH / _YAW if your frame responds backwards.
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
#define FC_MOTOR_MIN_ARMED_US   1050    // Minimum while armed — keeps motors spinning
#define FC_THROTTLE_IDLE_US     1080    // Below this → idle, PID integrators reset

// ============================================================
// Outer loop (angle) — Phase 3
//
//   Max rate the outer loop can command the inner loop.
//   At full stick deflection (±45°) the outer loop will command ±FC_OUTER_RATE_LIMIT.
//
//   Starting Kp: 4.5 — conservative for first tethered flights.
//   If the drone oscillates at hover: reduce Kp.
//   If the drone is sluggish / drifts: increase Kp slightly.
//   Do NOT add Ki or Kd to the outer loop — see header comment above.
// ============================================================
#define FC_OUTER_RATE_LIMIT     200.0f  // °/s — outer loop output clamp

#define FC_ANGLE_ROLL_KP        4.5f    // P gain for roll angle loop
#define FC_ANGLE_PITCH_KP       4.5f    // P gain for pitch angle loop

// Max stick angle setpoint (must match setpoint.h limits — both are ±45°)
#define FC_MAX_ANGLE_DEG        45.0f

// ============================================================
// Inner loop (rate) — carried forward from Phase 2
//
//   Tuning order (already done in Phase 2):
//     1. Kp until borderline oscillation, then back off ~20%
//     2. Kd to damp Kp oscillation
//     3. Ki only if steady-state drift persists at hover
//
//   With the outer loop now closing the angle, inner Kp may
//   need slight reduction if the cascade feels overly aggressive.
// ============================================================
#define FC_ROLL_KP      1.0f
#define FC_ROLL_KI      0.0f
#define FC_ROLL_KD      0.0f

#define FC_PITCH_KP     1.0f
#define FC_PITCH_KI     0.0f
#define FC_PITCH_KD     0.0f

#define FC_YAW_KP       2.0f
#define FC_YAW_KI       0.0f
#define FC_YAW_KD       0.0f

// ============================================================
// PID limits per axis (µs — inner; °/s — outer is bounded by FC_OUTER_RATE_LIMIT)
// ============================================================
#define FC_PID_OUTPUT_LIMIT     300.0f
#define FC_PID_INTEGRAL_LIMIT   150.0f

// ============================================================
// Safety
// ============================================================
#define FC_SETPOINT_STALE_DISARM_MS     600
#define FC_PERIOD_MS                     10     // 100 Hz

// ============================================================
// Status struct — thread-safe snapshot for main.c / logging
// ============================================================
typedef struct {
    bool     armed;

    // Outer loop (angle)
    float    angle_sp[2];       // Setpoint roll/pitch (°)   [0=roll, 1=pitch]
    float    angle_meas[2];     // Measured roll/pitch (°)   from complementary filter
    float    angle_err[2];      // Error (°)                 sp − meas
    float    angle_pid_out[2];  // Outer P output (°/s)      → inner setpoint

    // Inner loop (rate)
    float    rate_sp[3];        // Setpoint rate  (°/s)      [0=roll,1=pitch,2=yaw]
    float    rate_meas[3];      // Measured rate  (°/s)      from raw gyro
    float    rate_pid_out[3];   // Inner PID output (µs)

    // Motor outputs
    uint16_t motor_us[4];       // PWM µs [M1, M2, M3, M4]

    uint32_t loop_count;
} fc_status_t;

// ============================================================
// Public API
// ============================================================

/**
 * @brief Initialise MCPWM and start the 100 Hz control task on Core 1.
 *        Call AFTER imu_init() and setpoint_init().
 */
esp_err_t flight_control_init(void);

/**
 * @brief Arm — enables PID loop. Refuses if IMU unhealthy.
 */
void flight_control_arm(void);

/**
 * @brief Disarm — motors to minimum, all PID state reset.
 */
void flight_control_disarm(void);

/** @brief Returns true while armed. */
bool flight_control_is_armed(void);

/** @brief Thread-safe status snapshot. */
void flight_control_get_status(fc_status_t *out);

/**
 * @brief Update outer (angle) P gains at runtime.
 *        Resets all PID state when called.
 *        Safe to call while armed — takes effect on the next loop tick.
 */
void flight_control_set_angle_gains(float roll_kp, float pitch_kp);

/**
 * @brief Update inner (rate) PID gains at runtime.
 *        Resets all PID state when called.
 */
void flight_control_set_rate_gains(
    float roll_kp,  float roll_ki,  float roll_kd,
    float pitch_kp, float pitch_ki, float pitch_kd,
    float yaw_kp,   float yaw_ki,   float yaw_kd);

/**
 * @brief ESC calibration sequence (blocking ~10 s). Must be DISARMED.
 */
void flight_control_calibrate_escs(void);

/**
 * @brief Spin one motor at a specific pulse, all others at minimum.
 *        For bench testing. DISARMED only.
 */
void flight_control_motor_test(int motor_num, uint16_t pulse_us);

#ifdef __cplusplus
}
#endif
