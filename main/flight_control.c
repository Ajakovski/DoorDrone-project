/*
 * flight_control.c — Cascade PID Flight Controller (Phase 3)
 *
 * Owns MCPWM hardware, six PID instances (2 outer + 3 inner + 1 yaw),
 * and the 100 Hz Core 1 task.
 *
 * Execution order per tick (10 ms):
 *   1. Read IMU snapshot (mutex-protected, ~2 µs)
 *   2. Read setpoint snapshot (mutex-protected, ~1 µs)
 *   3. Outer angle P loop → target rate (°/s)  [roll, pitch]
 *   4. Inner rate PID loop → torque (µs)       [roll, pitch, yaw]
 *   5. Motor mixer → M1–M4 (µs)
 *   6. Write MCPWM comparators
 *   7. Update shared status
 */

#include "flight_control.h"
#include "pid.h"
#include "imu.h"
#include "setpoint.h"

#include <string.h>

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
#define FC_TASK_PRIO        9       // Below IMU task (10) — reads fresh IMU data
#define FC_TASK_CORE        1
#define FC_LOG_EVERY_N      100     // Serial log every 100 loops ≈ 1 s

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
//
//   Outer (angle, P-only):  s_pid_angle_roll, s_pid_angle_pitch
//   Inner (rate, PID):      s_pid_rate_roll, s_pid_rate_pitch, s_pid_rate_yaw
//
//   Outer loop uses pid_t with ki=0, kd=0. The generic pid_t works for
//   both loops — no need for a separate "P-only" type.
// ============================================================
static pid_t s_pid_angle_roll;
static pid_t s_pid_angle_pitch;

static pid_t s_pid_rate_roll;
static pid_t s_pid_rate_pitch;
static pid_t s_pid_rate_yaw;

// ============================================================
// Shared state
// ============================================================
static SemaphoreHandle_t s_mutex   = NULL;
static volatile bool     s_armed   = false;
static bool              s_started = false;
static fc_status_t       s_status;

// ============================================================
// ESC helpers
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

// ============================================================
// Reset all PID state (both loops)
// ============================================================
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
//   Uniform ceiling pull-down: if any motor exceeds FC_ESC_MAX_US,
//   subtract the excess from all motors equally to preserve the
//   roll/pitch/yaw ratio.
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

    // Pull all motors down by the same amount if any exceeds the ceiling
    int32_t peak = v1;
    if (v2 > peak) peak = v2;
    if (v3 > peak) peak = v3;
    if (v4 > peak) peak = v4;

    if (peak > FC_ESC_MAX_US) {
        int32_t excess = peak - FC_ESC_MAX_US;
        v1 -= excess;  v2 -= excess;  v3 -= excess;  v4 -= excess;
    }

    *m1 = clamp_us(v1, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
    *m2 = clamp_us(v2, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
    *m3 = clamp_us(v3, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
    *m4 = clamp_us(v4, FC_MOTOR_MIN_ARMED_US, FC_ESC_MAX_US);
}

// ============================================================
// Flight control task — 100 Hz, Core 1
// ============================================================
static void flight_control_task(void *arg)
{
    ESP_LOGI(TAG, "FC task started — cascade PID, 100 Hz, Core %d", FC_TASK_CORE);

    TickType_t last_wake  = xTaskGetTickCount();
    int64_t    prev_us    = esp_timer_get_time();
    uint32_t   loop_count = 0;
    uint32_t   stale_cnt  = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FC_PERIOD_MS));
        loop_count++;

        // Real elapsed time — more accurate than fixed 10 ms for integration
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - prev_us) * 1e-6f;
        prev_us  = now_us;
        if (dt <= 0.0f || dt > 0.1f) dt = FC_PERIOD_MS * 0.001f;

        // ── Disarmed path ────────────────────────────────────
        if (!s_armed) {
            esc_all_to_min();
            pids_reset_all();
            stale_cnt = 0;

            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                memset(&s_status, 0, sizeof(s_status));
                s_status.loop_count = loop_count;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        // ── Safety: IMU ──────────────────────────────────────
        if (!imu_is_healthy()) {
            ESP_LOGE(TAG, "IMU unhealthy while ARMED — DISARM");
            s_armed = false;
            esc_all_to_min();
            pids_reset_all();
            continue;
        }

        // ── Safety: setpoint freshness ───────────────────────
        if (!setpoint_is_fresh()) {
            stale_cnt++;
            if (stale_cnt >= (FC_SETPOINT_STALE_DISARM_MS / FC_PERIOD_MS)) {
                ESP_LOGE(TAG, "Setpoint stale > %d ms — DISARM",
                         FC_SETPOINT_STALE_DISARM_MS);
                s_armed = false;
                esc_all_to_min();
                pids_reset_all();
                stale_cnt = 0;
            }
            continue;  // Hold last motor state until fresh or disarm
        }
        stale_cnt = 0;

        // ── Read sensors ─────────────────────────────────────
        imu_data_t imu;
        setpoint_t sp;
        imu_get_data(&imu);
        setpoint_get(&sp);

        // ── Low throttle: idle, reset integrators ────────────
        if (sp.throttle < FC_THROTTLE_IDLE_US) {
            esc_all_to_idle();
            pids_reset_all();

            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                memset(&s_status, 0, sizeof(s_status));
                s_status.armed      = true;
                s_status.loop_count = loop_count;
                for (int i = 0; i < 4; i++)
                    s_status.motor_us[i] = FC_MOTOR_MIN_ARMED_US;
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        // ════════════════════════════════════════════════════
        // OUTER LOOP — Angle P controller (roll and pitch only)
        //
        //   input  : sp.roll / sp.pitch  — target angle (°) from stick
        //   sensor : imu.roll / imu.pitch — CF-fused angle (°)
        //   output : target_roll_rate / target_pitch_rate (°/s)
        //            clamped to ±FC_OUTER_RATE_LIMIT
        //
        //   The pid_t is used in P-only mode (ki=0, kd=0).
        //   pid_update() derivative-on-measurement uses prev_measurement,
        //   which has no effect when kd=0 — no wasted computation.
        // ════════════════════════════════════════════════════
        float target_roll_rate  = pid_update(&s_pid_angle_roll,
                                              sp.roll,  imu.roll,  dt);
        float target_pitch_rate = pid_update(&s_pid_angle_pitch,
                                              sp.pitch, imu.pitch, dt);

        // Clamp outer loop output — prevent extreme rate commands
        target_roll_rate  = clampf(target_roll_rate,
                                   -FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);
        target_pitch_rate = clampf(target_pitch_rate,
                                   -FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);

        // Yaw: no outer loop — stick yaw_rate is the inner setpoint directly
        float target_yaw_rate = sp.yaw_rate;

        // ════════════════════════════════════════════════════
        // INNER LOOP — Rate PID (roll, pitch, yaw)
        //
        //   input  : target_*_rate from outer loop (or stick for yaw)
        //   sensor : imu.gyro_x / gyro_y / gyro_z — raw gyro (°/s)
        //            Raw gyro intentional — lower latency than CF angle,
        //            derivative-on-measurement has clean derivative signal.
        //   output : per-axis torque correction in µs
        // ════════════════════════════════════════════════════
        float out_roll  = pid_update(&s_pid_rate_roll,
                                      target_roll_rate,  imu.gyro_x, dt);
        float out_pitch = pid_update(&s_pid_rate_pitch,
                                      target_pitch_rate, imu.gyro_y, dt);
        float out_yaw   = pid_update(&s_pid_rate_yaw,
                                      target_yaw_rate,   imu.gyro_z, dt);

        // ── Motor mixing ──────────────────────────────────────
        uint16_t m1, m2, m3, m4;
        mixer_apply(sp.throttle, out_roll, out_pitch, out_yaw,
                    &m1, &m2, &m3, &m4);
        esc_write_all(m1, m2, m3, m4);

        // ── Update shared status ──────────────────────────────
        if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
            s_status.armed = true;
            s_status.loop_count = loop_count;

            s_status.angle_sp[0]      = sp.roll;
            s_status.angle_sp[1]      = sp.pitch;
            s_status.angle_meas[0]    = imu.roll;
            s_status.angle_meas[1]    = imu.pitch;
            s_status.angle_err[0]     = sp.roll  - imu.roll;
            s_status.angle_err[1]     = sp.pitch - imu.pitch;
            s_status.angle_pid_out[0] = target_roll_rate;
            s_status.angle_pid_out[1] = target_pitch_rate;

            s_status.rate_sp[0]       = target_roll_rate;
            s_status.rate_sp[1]       = target_pitch_rate;
            s_status.rate_sp[2]       = target_yaw_rate;
            s_status.rate_meas[0]     = imu.gyro_x;
            s_status.rate_meas[1]     = imu.gyro_y;
            s_status.rate_meas[2]     = imu.gyro_z;
            s_status.rate_pid_out[0]  = out_roll;
            s_status.rate_pid_out[1]  = out_pitch;
            s_status.rate_pid_out[2]  = out_yaw;

            s_status.motor_us[0] = m1;
            s_status.motor_us[1] = m2;
            s_status.motor_us[2] = m3;
            s_status.motor_us[3] = m4;

            xSemaphoreGive(s_mutex);
        }

        // ── Periodic serial log ───────────────────────────────
        if (loop_count % FC_LOG_EVERY_N == 0) {
            ESP_LOGI(TAG, "─ Loop #%lu ────────────────────────────────",
                     (unsigned long)loop_count);
            ESP_LOGI(TAG, "  THR  : %d µs", sp.throttle);
            ESP_LOGI(TAG, "  OUTER  Roll : sp=%+6.1f°  meas=%+6.1f°  err=%+5.1f°  → %+6.1f°/s",
                     sp.roll,  imu.roll,  sp.roll  - imu.roll,  target_roll_rate);
            ESP_LOGI(TAG, "  OUTER  Pitch: sp=%+6.1f°  meas=%+6.1f°  err=%+5.1f°  → %+6.1f°/s",
                     sp.pitch, imu.pitch, sp.pitch - imu.pitch, target_pitch_rate);
            ESP_LOGI(TAG, "  INNER  Roll : sp=%+6.1f°/s  gyro=%+6.1f°/s  out=%+6.1fµs",
                     target_roll_rate,  imu.gyro_x, out_roll);
            ESP_LOGI(TAG, "  INNER  Pitch: sp=%+6.1f°/s  gyro=%+6.1f°/s  out=%+6.1fµs",
                     target_pitch_rate, imu.gyro_y, out_pitch);
            ESP_LOGI(TAG, "  INNER  Yaw  : sp=%+6.1f°/s  gyro=%+6.1f°/s  out=%+6.1fµs",
                     target_yaw_rate,   imu.gyro_z, out_yaw);
            ESP_LOGI(TAG, "  MOTORS: M1=%u  M2=%u  M3=%u  M4=%u",
                     m1, m2, m3, m4);
            ESP_LOGI(TAG, "────────────────────────────────────────────");
        }
    }
}

// ============================================================
// MCPWM hardware init
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

        ESP_LOGI(TAG, "ESC%d (M%d): GPIO%d, Group%d", i+1, i+1, s_esc_gpio[i], grp);
    }

    for (int grp = 0; grp < 2; grp++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(s_esc_timer[grp]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_esc_timer[grp], MCPWM_TIMER_START_NO_STOP));
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
        ESP_LOGE(TAG, "Mutex failed");
        return ESP_ERR_NO_MEM;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_armed = false;

    // ── Outer loop — P-only angle controllers ────────────────
    // ki=0, kd=0, integral_limit irrelevant (no integration), output clamped
    pid_init(&s_pid_angle_roll,
             FC_ANGLE_ROLL_KP,  0.0f, 0.0f,
             FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);

    pid_init(&s_pid_angle_pitch,
             FC_ANGLE_PITCH_KP, 0.0f, 0.0f,
             FC_OUTER_RATE_LIMIT, FC_OUTER_RATE_LIMIT);

    // ── Inner loop — full rate PID ────────────────────────────
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
        ESP_LOGE(TAG, "Task create failed");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;

    ESP_LOGI(TAG, "Flight control ready — cascade PID, Core %d, prio %d",
             FC_TASK_CORE, FC_TASK_PRIO);
    ESP_LOGI(TAG, "  Outer  Roll : Kp=%.2f  (angle → rate, limit ±%.0f°/s)",
             s_pid_angle_roll.kp, FC_OUTER_RATE_LIMIT);
    ESP_LOGI(TAG, "  Outer  Pitch: Kp=%.2f",
             s_pid_angle_pitch.kp);
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
    if (s_armed) { ESP_LOGW(TAG, "Already armed"); return; }
    if (!imu_is_healthy()) { ESP_LOGE(TAG, "ARM REFUSED — IMU unhealthy"); return; }
    if (!s_started)        { ESP_LOGE(TAG, "ARM REFUSED — not initialised"); return; }

    if (!setpoint_is_fresh()) {
        ESP_LOGW(TAG, "ARM WARNING — no UDP setpoint yet. "
                      "Motors will idle until packets arrive.");
    }

    esc_all_to_min();
    vTaskDelay(pdMS_TO_TICKS(1000));
    pids_reset_all();
    s_armed = true;
    imu_reset_yaw();

    ESP_LOGI(TAG, "ARMED — cascade PID active. Yaw reference reset.");
    ESP_LOGW(TAG, "SAFETY: Remove hands. Throttle > %d µs spins motors.",
             FC_THROTTLE_IDLE_US);
}

void flight_control_disarm(void)
{
    if (!s_armed) { ESP_LOGW(TAG, "Already disarmed"); return; }
    s_armed = false;
    esc_all_to_min();
    pids_reset_all();
    ESP_LOGI(TAG, "DISARMED — all motors at %d µs", FC_ESC_MIN_US);
}

bool flight_control_is_armed(void) { return s_armed; }

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
    // Float writes on LX7 are atomic — safe to update without mutex.
    // Clamp to a sensible range to prevent runaway from bad Blynk input.
    if (roll_kp  < 0.0f) roll_kp  = 0.0f;
    if (pitch_kp < 0.0f) pitch_kp = 0.0f;
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

    ESP_LOGI(TAG, "Inner gains updated — Roll Kp=%.2f Ki=%.2f Kd=%.2f",
             roll_kp, roll_ki, roll_kd);
    ESP_LOGI(TAG, "                      Pitch Kp=%.2f Ki=%.2f Kd=%.2f",
             pitch_kp, pitch_ki, pitch_kd);
    ESP_LOGI(TAG, "                      Yaw   Kp=%.2f Ki=%.2f Kd=%.2f",
             yaw_kp, yaw_ki, yaw_kd);
}

void flight_control_calibrate_escs(void)
{
    if (s_armed) { ESP_LOGE(TAG, "CALIBRATE REFUSED — disarm first"); return; }

    ESP_LOGW(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGW(TAG, "║   ESC CALIBRATION — REMOVE PROPELLERS    ║");
    ESP_LOGW(TAG, "╚══════════════════════════════════════════╝");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Step 1: MAX signal (2000 µs)");
    esc_write_all(FC_ESC_MAX_US, FC_ESC_MAX_US, FC_ESC_MAX_US, FC_ESC_MAX_US);

    ESP_LOGW(TAG, "Connect battery NOW. Listen for 3 beeps. Waiting 8 s...");
    vTaskDelay(pdMS_TO_TICKS(8000));

    ESP_LOGI(TAG, "Step 2: MIN signal (1000 µs)");
    esc_all_to_min();
    ESP_LOGI(TAG, "Listen for double-beep — calibration saved.");
}

void flight_control_motor_test(int motor_num, uint16_t pulse_us)
{
    if (s_armed) { ESP_LOGE(TAG, "Motor test refused — disarm first"); return; }
    if (motor_num < 1 || motor_num > 4) { ESP_LOGE(TAG, "Bad motor: %d", motor_num); return; }

    pulse_us = clamp_us(pulse_us, FC_ESC_MIN_US, FC_ESC_MAX_US);
    uint16_t v[4] = {FC_ESC_MIN_US, FC_ESC_MIN_US, FC_ESC_MIN_US, FC_ESC_MIN_US};
    v[motor_num - 1] = pulse_us;
    esc_write_all(v[0], v[1], v[2], v[3]);

    ESP_LOGI(TAG, "Motor %d (GPIO%d) → %d µs  (others at %d µs)",
             motor_num, s_esc_gpio[motor_num - 1], pulse_us, FC_ESC_MIN_US);
}