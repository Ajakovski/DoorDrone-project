/*
 * flight_control.c — Cascade PID + Emergency Landing (Phase 3)
 *
 * State machine:
 *   DISARMED → ARMED_NORMAL → EMERGENCY_LEVEL → EMERGENCY_LAND → DISARMED
 *
 * Emergency is triggered by:
 *   - Setpoint stale > FC_SETPOINT_STALE_DISARM_MS (UDP link loss)
 *   - flight_control_trigger_emergency() called from main.c (MQTT disconnect)
 *   - UDP flags bit 1 (0x02) set by HTML RC controller emergency button
 *
 * Manual disarm (Blynk V0) always goes directly to DISARMED immediately.
 * IMU failure is out of scope — IMU assumed healthy throughout.
 */

#include "flight_control.h"
#include "pid.h"
#include "imu.h"
#include "setpoint.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "[FC]";

// ============================================================
// Task configuration
// ============================================================
#define FC_TASK_STACK       4096
#define FC_TASK_PRIO        9
#define FC_TASK_CORE        1
#define FC_LOG_EVERY_N      100

// ============================================================
// MCPWM hardware handles
// ============================================================
static mcpwm_timer_handle_t s_esc_timer[2] = {NULL};
static mcpwm_oper_handle_t  s_esc_oper[4]  = {NULL};
static mcpwm_cmpr_handle_t  s_esc_cmpr[4]  = {NULL};
static mcpwm_gen_handle_t   s_esc_gen[4]   = {NULL};

static const int s_esc_gpio[4] = {
    FC_ESC1_GPIO,   // M1 Front-Right CW
    FC_ESC2_GPIO,   // M2 Front-Left  CCW
    FC_ESC3_GPIO,   // M3 Rear-Right  CCW
    FC_ESC4_GPIO,   // M4 Rear-Left   CW
};

// ============================================================
// PID instances
// ============================================================
static pid_t s_pid_angle_roll;
static pid_t s_pid_angle_pitch;
static pid_t s_pid_rate_roll;
static pid_t s_pid_rate_pitch;
static pid_t s_pid_rate_yaw;

// ============================================================
// Shared state
// ============================================================
static SemaphoreHandle_t          s_mutex   = NULL;
static volatile fc_flight_state_t s_state   = FC_STATE_DISARMED;
static bool                       s_started = false;
static fc_status_t                s_status;

// Emergency phase tracking — written and read only inside FC task
static int64_t  s_emerg_phase_start_us = 0;
static uint16_t s_emerg_hold_throttle  = 1000;

// ============================================================
// Internal helpers
// ============================================================
static inline uint16_t clamp_us(int32_t v, uint16_t lo, uint16_t hi)
{
    if (v < (int32_t)lo) return lo;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void esc_write_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    mcpwm_comparator_set_compare_value(s_esc_cmpr[0], m1);
    mcpwm_comparator_set_compare_value(s_esc_cmpr[1], m2);
    mcpwm_comparator_set_compare_value(s_esc_cmpr[2], m3);
    mcpwm_comparator_set_compare_value(s_esc_cmpr[3], m4);
}

static void esc_all_to_min(void)
{
    esc_write_all(FC_ESC_MIN_US, FC_ESC_MIN_US, FC_ESC_MIN_US, FC_ESC_MIN_US);
}

static void esc_all_to_idle(void)
{
    esc_write_all(FC_MOTOR_MIN_ARMED_US, FC_MOTOR_MIN_ARMED_US,
                  FC_MOTOR_MIN_ARMED_US, FC_MOTOR_MIN_ARMED_US);
}

static void pids_reset_all(void)
{
    pid_reset(&s_pid_angle_roll);
    pid_reset(&s_pid_angle_pitch);
    pid_reset(&s_pid_rate_roll);
    pid_reset(&s_pid_rate_pitch);
    pid_reset(&s_pid_rate_yaw);
}

// ============================================================
// Mixer — Quad-X
//
//   M1 (FR CW)  = thr - roll - pitch - yaw
//   M2 (FL CCW) = thr + roll - pitch + yaw
//   M3 (RR CCW) = thr - roll + pitch + yaw
//   M4 (RL CW)  = thr + roll + pitch - yaw
//
//   Uniform ceiling pull-down preserves roll/pitch/yaw ratios
//   when any motor would exceed FC_ESC_MAX_US.
// ============================================================
static void mixer_apply(uint16_t thr, float roll, float pitch, float yaw,
                         uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4)
{
    roll  *= FC_SIGN_ROLL;
    pitch *= FC_SIGN_PITCH;
    yaw   *= FC_SIGN_YAW;

    int32_t v1 = (int32_t)thr - (int32_t)roll - (int32_t)pitch - (int32_t)yaw;
    int32_t v2 = (int32_t)thr + (int32_t)roll - (int32_t)pitch + (int32_t)yaw;
    int32_t v3 = (int32_t)thr - (int32_t)roll + (int32_t)pitch + (int32_t)yaw;
    int32_t v4 = (int32_t)thr + (int32_t)roll + (int32_t)pitch - (int32_t)yaw;

    int32_t peak = v1;
    if (v2 > peak) peak = v2;
    if (v3 > peak) peak = v3;
    if (v4 > peak) peak = v4;
    if (peak > FC_ESC_MAX_US) {
        int32_t excess = peak - FC_ESC_MAX_US;
        v1 -= excess; v2 -= excess; v3 -= excess; v4 -= excess;
    }

    *m1 = clamp_us(v1, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
    *m2 = clamp_us(v2, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
    *m3 = clamp_us(v3, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
    *m4 = clamp_us(v4, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
}

// ============================================================
// Cascade control pass — shared by NORMAL and EMERGENCY_LEVEL
//
// Runs outer P loop + inner PID loop + mixer.
// Populates st with all telemetry fields.
// ============================================================
static void run_cascade(float roll_sp, float pitch_sp, float yaw_rate_sp,
                         uint16_t throttle,
                         const imu_data_t *imu, float dt,
                         uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4,
                         fc_status_t *st)
{
    // Outer loop: angle → target rate
    float target_roll_rate  = pid_update(&s_pid_angle_roll,
                                          roll_sp,  imu->roll,  dt);
    float target_pitch_rate = pid_update(&s_pid_angle_pitch,
                                          pitch_sp, imu->pitch, dt);

    target_roll_rate  = clampf(target_roll_rate,
                               -FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);
    target_pitch_rate = clampf(target_pitch_rate,
                               -FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);

    // Inner loop: rate → torque correction
    float out_roll  = pid_update(&s_pid_rate_roll,
                                  target_roll_rate,  imu->gyro_x, dt);
    float out_pitch = pid_update(&s_pid_rate_pitch,
                                  target_pitch_rate, imu->gyro_y, dt);
    float out_yaw   = pid_update(&s_pid_rate_yaw,
                                  yaw_rate_sp,       imu->gyro_z, dt);

    mixer_apply(throttle, out_roll, out_pitch, out_yaw, m1, m2, m3, m4);

    // Telemetry
    st->angle_sp[0]      = roll_sp;
    st->angle_sp[1]      = pitch_sp;
    st->angle_meas[0]    = imu->roll;
    st->angle_meas[1]    = imu->pitch;
    st->angle_err[0]     = roll_sp  - imu->roll;
    st->angle_err[1]     = pitch_sp - imu->pitch;
    st->angle_pid_out[0] = target_roll_rate;
    st->angle_pid_out[1] = target_pitch_rate;
    st->rate_sp[0]       = target_roll_rate;
    st->rate_sp[1]       = target_pitch_rate;
    st->rate_sp[2]       = yaw_rate_sp;
    st->rate_meas[0]     = imu->gyro_x;
    st->rate_meas[1]     = imu->gyro_y;
    st->rate_meas[2]     = imu->gyro_z;
    st->rate_pid_out[0]  = out_roll;
    st->rate_pid_out[1]  = out_pitch;
    st->rate_pid_out[2]  = out_yaw;
    st->motor_us[0]      = *m1;
    st->motor_us[1]      = *m2;
    st->motor_us[2]      = *m3;
    st->motor_us[3]      = *m4;
}

// ============================================================
// Flight control task — 100 Hz, Core 1
// ============================================================
static void flight_control_task(void *arg)
{
    ESP_LOGI(TAG, "FC task started — 100 Hz, Core %d", FC_TASK_CORE);

    TickType_t last_wake  = xTaskGetTickCount();
    int64_t    prev_us    = esp_timer_get_time();
    uint32_t   loop_count = 0;
    uint32_t   stale_cnt  = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FC_PERIOD_MS));
        loop_count++;

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - prev_us) * 1e-6f;
        prev_us  = now_us;
        if (dt <= 0.0f || dt > 0.1f) dt = FC_PERIOD_MS * 0.001f;

        fc_flight_state_t state = s_state; // single atomic read for this tick

        // ════════════════════════════════════════════════════════
        // STATE: DISARMED
        // ════════════════════════════════════════════════════════
        if (state == FC_STATE_DISARMED) {
            esc_all_to_min();
            pids_reset_all();
            stale_cnt = 0;

            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                memset(&s_status, 0, sizeof(s_status));
                s_status.state      = FC_STATE_DISARMED;
                s_status.loop_count = loop_count;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        // ════════════════════════════════════════════════════════
        // STATE: ARMED_NORMAL
        // ════════════════════════════════════════════════════════
        if (state == FC_STATE_ARMED_NORMAL) {

            // Setpoint freshness — if stale long enough, enter emergency
            if (!setpoint_is_fresh()) {
                stale_cnt++;
                if (stale_cnt >= (FC_SETPOINT_STALE_DISARM_MS / FC_PERIOD_MS)) {
                    ESP_LOGE(TAG, "Setpoint stale > %dms — EMERGENCY LANDING",
                             FC_SETPOINT_STALE_DISARM_MS);
                    setpoint_t sp_last;
                    setpoint_get(&sp_last);
                    s_emerg_hold_throttle  = sp_last.throttle;
                    s_emerg_phase_start_us = esp_timer_get_time();
                    s_state                = FC_STATE_EMERGENCY_LEVEL;
                    pids_reset_all();
                    stale_cnt = 0;
                }
                continue; // hold last motor state this tick
            }
            stale_cnt = 0;

            imu_data_t imu;
            setpoint_t sp;
            imu_get_data(&imu);
            setpoint_get(&sp);

            // Remote emergency trigger via UDP flags bit 1
            if (sp.flags & 0x02) {
                ESP_LOGW(TAG, "Emergency flag (0x02) received via UDP — EMERGENCY LANDING");
                s_emerg_hold_throttle  = sp.throttle;
                s_emerg_phase_start_us = esp_timer_get_time();
                s_state                = FC_STATE_EMERGENCY_LEVEL;
                pids_reset_all();
                continue;
            }

            // Low throttle: idle motors and reset integrators
            if (sp.throttle < FC_THROTTLE_IDLE_US) {
                esc_all_to_idle();
                pids_reset_all();

                if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                    memset(&s_status, 0, sizeof(s_status));
                    s_status.state      = FC_STATE_ARMED_NORMAL;
                    s_status.loop_count = loop_count;
                    for (int i = 0; i < 4; i++)
                        s_status.motor_us[i] = FC_MOTOR_MIN_ARMED_US;
                    xSemaphoreGive(s_mutex);
                }
                continue;
            }

            // Normal cascade
            uint16_t m1, m2, m3, m4;
            fc_status_t st_local = {0};
            run_cascade(sp.roll, sp.pitch, sp.yaw_rate, sp.throttle,
                        &imu, dt, &m1, &m2, &m3, &m4, &st_local);
            esc_write_all(m1, m2, m3, m4);

            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                s_status                              = st_local;
                s_status.state                        = FC_STATE_ARMED_NORMAL;
                s_status.loop_count                   = loop_count;
                s_status.emergency_phase_ms_remaining = 0;
                xSemaphoreGive(s_mutex);
            }

            if (loop_count % FC_LOG_EVERY_N == 0) {
                ESP_LOGI(TAG, "─ Loop #%lu ── NORMAL ──────────────────────",
                         (unsigned long)loop_count);
                ESP_LOGI(TAG, "  THR:%d  R:%+5.1f°  P:%+5.1f°  M:%u %u %u %u",
                         sp.throttle, imu.roll, imu.pitch, m1, m2, m3, m4);
                ESP_LOGI(TAG, "  OUTER  Roll : sp=%+6.1f°  meas=%+6.1f°  err=%+5.1f°  → %+6.1f°/s",
                         sp.roll,  imu.roll,  sp.roll  - imu.roll,  st_local.angle_pid_out[0]);
                ESP_LOGI(TAG, "  OUTER  Pitch: sp=%+6.1f°  meas=%+6.1f°  err=%+5.1f°  → %+6.1f°/s",
                         sp.pitch, imu.pitch, sp.pitch - imu.pitch, st_local.angle_pid_out[1]);
                ESP_LOGI(TAG, "  INNER  Roll : sp=%+6.1f°/s  gyro=%+6.1f°/s  out=%+6.1fµs",
                         st_local.rate_sp[0], imu.gyro_x, st_local.rate_pid_out[0]);
                ESP_LOGI(TAG, "  INNER  Pitch: sp=%+6.1f°/s  gyro=%+6.1f°/s  out=%+6.1fµs",
                         st_local.rate_sp[1], imu.gyro_y, st_local.rate_pid_out[1]);
                ESP_LOGI(TAG, "  INNER  Yaw  : sp=%+6.1f°/s  gyro=%+6.1f°/s  out=%+6.1fµs",
                         sp.yaw_rate,         imu.gyro_z, st_local.rate_pid_out[2]);
                ESP_LOGI(TAG, "────────────────────────────────────────────");
            }
            continue;
        }

        // ════════════════════════════════════════════════════════
        // STATE: EMERGENCY_LEVEL
        //
        // Hold the throttle that was active when the fault occurred.
        // Run cascade with roll=0°, pitch=0°, yaw_rate=0°/s.
        // Setpoint freshness NOT checked — link is gone.
        //
        // Transition to EMERGENCY_LAND when:
        //   a) |roll| < FC_LEVEL_THRESHOLD_DEG AND
        //      |pitch| < FC_LEVEL_THRESHOLD_DEG, OR
        //   b) FC_EMERGENCY_LEVEL_TIMEOUT_MS has elapsed
        // ════════════════════════════════════════════════════════
        if (state == FC_STATE_EMERGENCY_LEVEL) {

            imu_data_t imu;
            imu_get_data(&imu);

            int64_t elapsed_ms = (esp_timer_get_time() - s_emerg_phase_start_us)
                                 / 1000LL;
            bool timed_out = (elapsed_ms >= (int64_t)FC_EMERGENCY_LEVEL_TIMEOUT_MS);
            bool level_ok  = (fabsf(imu.roll)  < FC_LEVEL_THRESHOLD_DEG &&
                              fabsf(imu.pitch) < FC_LEVEL_THRESHOLD_DEG);

            if (timed_out) {
                ESP_LOGW(TAG,
                         "EMERG LEVEL: timeout (%lldms) — R=%.1f° P=%.1f° — proceeding to LAND",
                         elapsed_ms, imu.roll, imu.pitch);
            } else if (level_ok) {
                ESP_LOGI(TAG,
                         "EMERG LEVEL: leveled (R=%.1f° P=%.1f°) in %lldms — proceeding to LAND",
                         imu.roll, imu.pitch, elapsed_ms);
            }

            if (level_ok || timed_out) {
                s_emerg_phase_start_us = esp_timer_get_time();
                s_state                = FC_STATE_EMERGENCY_LAND;
                pids_reset_all();
                ESP_LOGW(TAG, "╔══════════════════════════════════════════╗");
                ESP_LOGW(TAG, "║       EMERGENCY LANDING INITIATED        ║");
                ESP_LOGW(TAG, "║  Throttle : %4d µs                      ║",
                         FC_LANDING_THROTTLE_US);
                ESP_LOGW(TAG, "║  Duration : %d ms                       ║",
                         FC_LANDING_DURATION_MS);
                ESP_LOGW(TAG, "╚══════════════════════════════════════════╝");
                continue;
            }

            // Still leveling — run cascade with zeroed attitude setpoints
            uint16_t m1, m2, m3, m4;
            fc_status_t st_local = {0};
            run_cascade(0.0f, 0.0f, 0.0f, s_emerg_hold_throttle,
                        &imu, dt, &m1, &m2, &m3, &m4, &st_local);
            esc_write_all(m1, m2, m3, m4);

            int32_t ms_remain = (int32_t)FC_EMERGENCY_LEVEL_TIMEOUT_MS -
                                (int32_t)elapsed_ms;
            if (ms_remain < 0) ms_remain = 0;

            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                s_status                              = st_local;
                s_status.state                        = FC_STATE_EMERGENCY_LEVEL;
                s_status.loop_count                   = loop_count;
                s_status.emergency_phase_ms_remaining = (uint32_t)ms_remain;
                xSemaphoreGive(s_mutex);
            }

            // Log every 200ms during emergency
            if (loop_count % 20 == 0) {
                ESP_LOGW(TAG, "EMERG LEVEL: R=%+5.1f°  P=%+5.1f°  — %lldms / %dms",
                         imu.roll, imu.pitch,
                         elapsed_ms, FC_EMERGENCY_LEVEL_TIMEOUT_MS);
            }
            continue;
        }

        // ════════════════════════════════════════════════════════
        // STATE: EMERGENCY_LAND
        //
        // All four motors at FC_LANDING_THROTTLE_US.
        // No PID — attitude assumed level from previous phase.
        // No setpoint read — link is gone.
        // After FC_LANDING_DURATION_MS, transition to DISARMED.
        // ════════════════════════════════════════════════════════
        if (state == FC_STATE_EMERGENCY_LAND) {

            int64_t elapsed_ms = (esp_timer_get_time() - s_emerg_phase_start_us)
                                 / 1000LL;

            if (elapsed_ms >= (int64_t)FC_LANDING_DURATION_MS) {
                ESP_LOGI(TAG, "EMERG LAND: %dms complete — DISARMING",
                         FC_LANDING_DURATION_MS);
                s_state = FC_STATE_DISARMED;
                esc_all_to_min();
                pids_reset_all();
                continue;
            }

            uint16_t lt = FC_LANDING_THROTTLE_US;
            esc_write_all(lt, lt, lt, lt);

            int32_t ms_remain = (int32_t)FC_LANDING_DURATION_MS -
                                (int32_t)elapsed_ms;

            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                memset(&s_status, 0, sizeof(s_status));
                s_status.state                        = FC_STATE_EMERGENCY_LAND;
                s_status.loop_count                   = loop_count;
                for (int i = 0; i < 4; i++) s_status.motor_us[i] = lt;
                s_status.emergency_phase_ms_remaining = (uint32_t)ms_remain;
                xSemaphoreGive(s_mutex);
            }

            if (loop_count % 20 == 0) {
                ESP_LOGW(TAG, "EMERG LAND: %lldms / %dms — throttle=%d µs",
                         elapsed_ms, FC_LANDING_DURATION_MS, lt);
            }
            continue;
        }
    }
}

// ============================================================
// MCPWM hardware initialisation
// ============================================================
static esp_err_t esc_pwm_init(void)
{
    for (int grp = 0; grp < 2; grp++) {
        mcpwm_timer_config_t tcfg = {
            .group_id      = grp,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,
            .period_ticks  = 20000,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        };
        esp_err_t r = mcpwm_new_timer(&tcfg, &s_esc_timer[grp]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "Timer %d failed: %s", grp, esp_err_to_name(r));
            return r;
        }
    }

    for (int i = 0; i < 4; i++) {
        int grp = (i < 2) ? 0 : 1;

        mcpwm_operator_config_t ocfg = { .group_id = grp };
        ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &s_esc_oper[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_esc_oper[i], s_esc_timer[grp]));

        mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
        ESP_ERROR_CHECK(mcpwm_new_comparator(s_esc_oper[i], &ccfg, &s_esc_cmpr[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_esc_cmpr[i], FC_ESC_MIN_US));

        mcpwm_generator_config_t gcfg = { .gen_gpio_num = s_esc_gpio[i] };
        ESP_ERROR_CHECK(mcpwm_new_generator(s_esc_oper[i], &gcfg, &s_esc_gen[i]));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_esc_gen[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_esc_gen[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            s_esc_cmpr[i],
                                            MCPWM_GEN_ACTION_LOW)));

        ESP_LOGI(TAG, "ESC%d (M%d): GPIO%d  Group%d",
                 i+1, i+1, s_esc_gpio[i], grp);
    }

    for (int grp = 0; grp < 2; grp++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(s_esc_timer[grp]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_esc_timer[grp],
                                                MCPWM_TIMER_START_NO_STOP));
    }

    ESP_LOGI(TAG, "ESC PWM running — all motors at %d µs", FC_ESC_MIN_US);
    return ESP_OK;
}

// ============================================================
// Public API
// ============================================================

esp_err_t flight_control_init(void)
{
    if (s_started) {
        ESP_LOGW(TAG, "Already initialised");
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed — out of heap?");
        return ESP_ERR_NO_MEM;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_state   = FC_STATE_DISARMED;
    s_started = false;

    // Outer loop — P-only (ki=0, kd=0)
    pid_init(&s_pid_angle_roll,
             FC_ANGLE_ROLL_KP,  0.0f, 0.0f,
             FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);
    pid_init(&s_pid_angle_pitch,
             FC_ANGLE_PITCH_KP, 0.0f, 0.0f,
             FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);

    // Inner loop — full PID (I and D currently 0)
    pid_init(&s_pid_rate_roll,
             FC_ROLL_KP,  FC_ROLL_KI,  FC_ROLL_KD,
             FC_PID_INTEGRAL_LIMIT, FC_PID_OUTPUT_LIMIT);
    pid_init(&s_pid_rate_pitch,
             FC_PITCH_KP, FC_PITCH_KI, FC_PITCH_KD,
             FC_PID_INTEGRAL_LIMIT, FC_PID_OUTPUT_LIMIT);
    pid_init(&s_pid_rate_yaw,
             FC_YAW_KP,   FC_YAW_KI,   FC_YAW_KD,
             FC_PID_INTEGRAL_LIMIT, FC_PID_OUTPUT_LIMIT);

    esp_err_t ret = esc_pwm_init();
    if (ret != ESP_OK) return ret;

    BaseType_t rc = xTaskCreatePinnedToCore(
        flight_control_task, "fc_task",
        FC_TASK_STACK, NULL, FC_TASK_PRIO, NULL, FC_TASK_CORE);

    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;

    ESP_LOGI(TAG, "Flight control ready — emergency landing enabled");
    ESP_LOGI(TAG, "  Landing throttle : %d µs for %d ms",
             FC_LANDING_THROTTLE_US, FC_LANDING_DURATION_MS);
    ESP_LOGI(TAG, "  Level timeout    : %d ms  threshold ±%.0f°",
             FC_EMERGENCY_LEVEL_TIMEOUT_MS, FC_LEVEL_THRESHOLD_DEG);
    ESP_LOGI(TAG, "  Outer  Roll : Kp=%.2f  Pitch: Kp=%.2f  (limit ±%.0f°/s)",
             s_pid_angle_roll.kp, s_pid_angle_pitch.kp, FC_OUTER_RATE_LIMIT);
    ESP_LOGI(TAG, "  Inner  Roll : Kp=%.2f Ki=%.2f Kd=%.2f",
             s_pid_rate_roll.kp,  s_pid_rate_roll.ki,  s_pid_rate_roll.kd);
    ESP_LOGI(TAG, "  Inner  Pitch: Kp=%.2f Ki=%.2f Kd=%.2f",
             s_pid_rate_pitch.kp, s_pid_rate_pitch.ki, s_pid_rate_pitch.kd);
    ESP_LOGI(TAG, "  Inner  Yaw  : Kp=%.2f Ki=%.2f Kd=%.2f",
             s_pid_rate_yaw.kp,   s_pid_rate_yaw.ki,   s_pid_rate_yaw.kd);

    return ESP_OK;
}

void flight_control_arm(void)
{
    if (s_state != FC_STATE_DISARMED) {
        ESP_LOGW(TAG, "ARM ignored — not in DISARMED state (state=%d)", (int)s_state);
        return;
    }
    if (!imu_is_healthy()) {
        ESP_LOGE(TAG, "ARM REFUSED — IMU unhealthy");
        return;
    }
    if (!s_started) {
        ESP_LOGE(TAG, "ARM REFUSED — not initialised");
        return;
    }

    esc_all_to_min();
    vTaskDelay(pdMS_TO_TICKS(1000));
    pids_reset_all();
    s_state = FC_STATE_ARMED_NORMAL;
    imu_reset_yaw();

    ESP_LOGI(TAG, "ARMED — cascade PID active, yaw reference reset");
    ESP_LOGW(TAG, "SAFETY: Remove hands. Throttle > %d µs spins motors.",
             FC_THROTTLE_IDLE_US);
}

void flight_control_disarm(void)
{
    // Intentional pilot command — always immediate
    s_state = FC_STATE_DISARMED;
    esc_all_to_min();
    pids_reset_all();
    ESP_LOGI(TAG, "DISARMED (manual command) — all motors at %d µs", FC_ESC_MIN_US);
}

void flight_control_trigger_emergency(void)
{
    fc_flight_state_t cur = s_state;

    if (cur == FC_STATE_DISARMED) {
        ESP_LOGW(TAG, "Emergency trigger ignored — already disarmed");
        return;
    }
    if (cur == FC_STATE_EMERGENCY_LEVEL || cur == FC_STATE_EMERGENCY_LAND) {
        ESP_LOGW(TAG, "Emergency trigger ignored — already in emergency (state=%d)",
                 (int)cur);
        return;
    }

    setpoint_t sp_last;
    setpoint_get(&sp_last);
    s_emerg_hold_throttle  = sp_last.throttle;
    s_emerg_phase_start_us = esp_timer_get_time();
    s_state                = FC_STATE_EMERGENCY_LEVEL;
    pids_reset_all();

    ESP_LOGE(TAG, "EMERGENCY LANDING TRIGGERED (external call)");
    ESP_LOGE(TAG, "  Holding throttle %d µs, leveling drone...",
             s_emerg_hold_throttle);
}

bool flight_control_is_armed(void)
{
    return (s_state == FC_STATE_ARMED_NORMAL);
}

fc_flight_state_t flight_control_get_state(void)
{
    return s_state;
}

void flight_control_get_status(fc_status_t *out)
{
    if (!out || !s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(out, &s_status, sizeof(fc_status_t));
        xSemaphoreGive(s_mutex);
    }
}

void flight_control_set_angle_gains(float roll_kp, float pitch_kp)
{
    if (roll_kp  < 0.0f)  roll_kp  = 0.0f;
    if (pitch_kp < 0.0f)  pitch_kp = 0.0f;
    if (roll_kp  > 20.0f) roll_kp  = 20.0f;
    if (pitch_kp > 20.0f) pitch_kp = 20.0f;

    s_pid_angle_roll.kp  = roll_kp;
    s_pid_angle_pitch.kp = pitch_kp;
    pids_reset_all();

    ESP_LOGI(TAG, "Outer gains updated — Roll Kp=%.2f  Pitch Kp=%.2f",
             roll_kp, pitch_kp);
}

void flight_control_set_rate_gains(
    float roll_kp,  float roll_ki,  float roll_kd,
    float pitch_kp, float pitch_ki, float pitch_kd,
    float yaw_kp,   float yaw_ki,   float yaw_kd)
{
    s_pid_rate_roll.kp  = roll_kp;
    s_pid_rate_roll.ki  = roll_ki;
    s_pid_rate_roll.kd  = roll_kd;
    s_pid_rate_pitch.kp = pitch_kp;
    s_pid_rate_pitch.ki = pitch_ki;
    s_pid_rate_pitch.kd = pitch_kd;
    s_pid_rate_yaw.kp   = yaw_kp;
    s_pid_rate_yaw.ki   = yaw_ki;
    s_pid_rate_yaw.kd   = yaw_kd;
    pids_reset_all();

    ESP_LOGI(TAG, "Inner gains — Roll %.2f/%.2f/%.2f  "
                  "Pitch %.2f/%.2f/%.2f  Yaw %.2f/%.2f/%.2f",
             roll_kp,  roll_ki,  roll_kd,
             pitch_kp, pitch_ki, pitch_kd,
             yaw_kp,   yaw_ki,   yaw_kd);
}

void flight_control_calibrate_escs(void)
{
    if (s_state != FC_STATE_DISARMED) {
        ESP_LOGE(TAG, "CALIBRATE REFUSED — must be disarmed first");
        return;
    }

    ESP_LOGW(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGW(TAG, "║   ESC CALIBRATION — REMOVE PROPELLERS    ║");
    ESP_LOGW(TAG, "╚══════════════════════════════════════════╝");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Step 1: MAX signal (%d µs)", FC_ESC_MAX_US);
    esc_write_all(FC_ESC_MAX_US, FC_ESC_MAX_US, FC_ESC_MAX_US, FC_ESC_MAX_US);
    ESP_LOGW(TAG, "Connect battery NOW. Listen for 3 beeps. Waiting 8s...");
    vTaskDelay(pdMS_TO_TICKS(8000));

    ESP_LOGI(TAG, "Step 2: MIN signal (%d µs)", FC_ESC_MIN_US);
    esc_all_to_min();
    ESP_LOGI(TAG, "ESC calibration complete — listen for double-beep.");
}

void flight_control_motor_test(int motor_num, uint16_t pulse_us)
{
    if (s_state != FC_STATE_DISARMED) {
        ESP_LOGE(TAG, "Motor test refused — must be disarmed");
        return;
    }
    if (motor_num < 1 || motor_num > 4) {
        ESP_LOGE(TAG, "Bad motor number: %d (must be 1–4)", motor_num);
        return;
    }

    pulse_us = clamp_us(pulse_us, FC_ESC_MIN_US, FC_ESC_MAX_US);
    uint16_t v[4] = {FC_ESC_MIN_US, FC_ESC_MIN_US, FC_ESC_MIN_US, FC_ESC_MIN_US};
    v[motor_num - 1] = pulse_us;
    esc_write_all(v[0], v[1], v[2], v[3]);

    ESP_LOGI(TAG, "Motor %d (GPIO%d) → %d µs  (others at %d µs)",
             motor_num, s_esc_gpio[motor_num - 1], pulse_us, FC_ESC_MIN_US);
}