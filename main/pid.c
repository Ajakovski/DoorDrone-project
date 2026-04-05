/*
 * pid.c — Generic PID Controller Implementation
 */

#include "pid.h"
#include <string.h>

void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    memset(pid, 0, sizeof(pid_t));
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->first_run      = true;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // ── P term ────────────────────────────────────────────────
    float p_term = pid->kp * error;

    // ── I term (with anti-windup clamp) ──────────────────────
    pid->integral += pid->ki * error * dt;
    if      (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // ── D term — derivative on MEASUREMENT, not error ────────
    // Avoids a large derivative spike when the setpoint changes suddenly
    // (e.g. fast stick input). Uses negative sign because increasing
    // measurement reduces the output.
    float d_term = 0.0f;
    if (!pid->first_run) {
        d_term = pid->kd * (pid->prev_measurement - measurement) / dt;
    }
    pid->prev_measurement = measurement;
    pid->first_run        = false;

    // ── Sum and clamp ─────────────────────────────────────────
    float output = p_term + pid->integral + d_term;
    if      (output >  pid->output_limit) output =  pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run        = true;
}