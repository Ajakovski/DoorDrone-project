#pragma once

/*
 * pid.h — Generic PID Controller
 *
 * Derivative-on-measurement (not error) to prevent derivative kick on
 * setpoint changes. Anti-windup via integrator clamping.
 *
 * All values in consistent units — this module is unit-agnostic.
 * In this project: error in °/s, output in µs (PWM correction).
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // ── Gains (tune these) ────────────────────────────────────
    float kp;
    float ki;
    float kd;

    // ── Limits ────────────────────────────────────────────────
    float integral_limit;   // Anti-windup: clamp |integral| to this value
    float output_limit;     // Clamp |total output| to this value

    // ── Runtime state (do not write externally) ───────────────
    float integral;
    float prev_measurement; // Used for derivative-on-measurement
    bool  first_run;        // Prevents derivative spike on first call
} pid_t;

/**
 * @brief Initialise a PID instance with gains and limits.
 *        Clears all runtime state.
 */
void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit);

/**
 * @brief Run one PID cycle.
 *
 * @param pid         PID instance
 * @param setpoint    Desired value (e.g. target angular rate in °/s)
 * @param measurement Actual value  (e.g. gyro reading in °/s)
 * @param dt          Time step in seconds (must be > 0)
 * @return            PID correction output (e.g. µs to add/subtract)
 */
float pid_update(pid_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief Reset integrator and derivative state.
 *        Call when: disarming, throttle at idle, mode change.
 */
void pid_reset(pid_t *pid);

#ifdef __cplusplus
}
#endif