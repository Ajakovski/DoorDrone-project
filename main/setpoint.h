#pragma once

/*
 * setpoint.h — Shared flight setpoint structure
 *
 * The UDP listener task (Core 0) writes this.
 * The flight control task (Core 1) reads this.
 * Both access via setpoint_get() — mutex-protected, never block each other.
 *
 * ─── Wire packet format (12 bytes, big-endian) ─────────────────────────
 *  [0]    0xAB            magic byte 0
 *  [1]    0xCD            magic byte 1
 *  [2-3]  throttle        uint16  1000–2000 µs
 *  [4-5]  roll_raw        int16   value × 10  → ±45.0°  (range ±450)
 *  [6-7]  pitch_raw       int16   value × 10  → ±45.0°  (range ±450)
 *  [8-9]  yaw_rate_raw    int16   value × 10  → ±300.0°/s (range ±3000)
 *  [10]   flags           uint8   reserved — DO NOT use for arm/disarm yet
 *  [11]   checksum        uint8   XOR of bytes [0..10]
 * ───────────────────────────────────────────────────────────────────────
 *
 * UDP port: 4210
 * Staleness timeout: 500 ms — flight control task must check setpoint_is_fresh()
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// Configuration
// ============================================================
#define SETPOINT_UDP_PORT       4210
#define SETPOINT_TIMEOUT_MS     500     // stale if no packet received within this window

// ============================================================
// Shared Data Structure
// ============================================================
typedef struct {
    uint16_t throttle;      // µs,       1000–2000     (stick: bottom=1000, top=2000)
    float    roll;          // degrees,  −45.0 to +45.0 (target angle: right=positive)
    float    pitch;         // degrees,  −45.0 to +45.0 (target angle: nose-down=positive)
    float    yaw_rate;      // °/s,      −300.0 to +300.0 (CW from above = positive)
    uint8_t  flags;         // reserved for future use — do not interpret yet
    bool     fresh;         // true while packets arrive within SETPOINT_TIMEOUT_MS
    uint32_t packet_count;  // total valid packets received (wraps at UINT32_MAX)
} setpoint_t;

// ============================================================
// Public API
// ============================================================

/**
 * @brief Bind UDP socket and start the listener task on Core 0.
 *
 * Call after WiFi is connected and an IP address is assigned.
 * Calling a second time is a no-op.
 *
 * @return ESP_OK, or ESP_ERR_NO_MEM if task/mutex creation fails.
 */
esp_err_t setpoint_init(void);

/**
 * @brief Thread-safe snapshot of the current setpoint.
 *
 * Always safe to call from any task. Returns safe defaults
 * (throttle=1000, all angles=0) if no packet has ever been received.
 *
 * @param out Pointer to caller-allocated setpoint_t. Must not be NULL.
 */
void setpoint_get(setpoint_t *out);

/**
 * @brief Returns true if a valid packet arrived within SETPOINT_TIMEOUT_MS.
 *
 * The flight control task MUST call this and disarm if it returns false
 * while the drone is armed. This is the primary stick-loss detection.
 */
bool setpoint_is_fresh(void);

#ifdef __cplusplus
}
#endif