/*
 * setpoint.c — UDP Listener + Shared Setpoint State
 *
 * Binds a UDP socket on port 4210 (INADDR_ANY) and parses incoming
 * 12-byte packets into the shared setpoint_t struct.
 *
 * Task: Core 0, priority 4, 4 KB stack.
 * The IMU/flight tasks on Core 1 read via setpoint_get() — no contention.
 */

#include "setpoint.h"

#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "[SETPOINT]";

// ============================================================
// Packet Layout Constants
// ============================================================
#define MAGIC_0         0xAB
#define MAGIC_1         0xCD
#define PACKET_SIZE     12

// ============================================================
// Task Configuration
// ============================================================
#define UDP_TASK_STACK  4096
#define UDP_TASK_PRIO   4       // Low — networking tier, Core 0
#define UDP_TASK_CORE   0

// ============================================================
// Internal State
// ============================================================
static SemaphoreHandle_t s_mutex      = NULL;
static int64_t           s_last_rx_us = 0;
static bool              s_started    = false;

static setpoint_t s_setpoint = {
    .throttle     = 1000,
    .roll         = 0.0f,
    .pitch        = 0.0f,
    .yaw_rate     = 0.0f,
    .flags        = 0,
    .fresh        = false,
    .packet_count = 0,
};

// ============================================================
// Packet Parser
// Returns true and fills *out on success; returns false and logs on error.
// ============================================================
static bool parse_packet(const uint8_t *buf, int len, setpoint_t *out)
{
    if (len != PACKET_SIZE) {
        ESP_LOGW(TAG, "Wrong length: got %d, expected %d", len, PACKET_SIZE);
        return false;
    }

    if (buf[0] != MAGIC_0 || buf[1] != MAGIC_1) {
        ESP_LOGW(TAG, "Bad magic: 0x%02X 0x%02X", buf[0], buf[1]);
        return false;
    }

    // XOR checksum over bytes [0..10], compare to byte [11]
    uint8_t chk = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++) chk ^= buf[i];
    if (chk != buf[PACKET_SIZE - 1]) {
        ESP_LOGW(TAG, "Checksum fail: calc=0x%02X recv=0x%02X", chk, buf[PACKET_SIZE - 1]);
        return false;
    }

    // Extract big-endian fields
    uint16_t thr = ((uint16_t)buf[2] << 8) | buf[3];
    int16_t  rol = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    int16_t  pit = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    int16_t  yaw = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    uint8_t  flg = buf[10];

    // Hard range validation — reject anything physically impossible
    if (thr < 1000 || thr > 2000) { ESP_LOGW(TAG, "Throttle OOB: %u", thr);  return false; }
    if (rol < -450  || rol > 450)  { ESP_LOGW(TAG, "Roll OOB: %d",   rol);   return false; }
    if (pit < -450  || pit > 450)  { ESP_LOGW(TAG, "Pitch OOB: %d",  pit);   return false; }
    if (yaw < -3000 || yaw > 3000) { ESP_LOGW(TAG, "Yaw OOB: %d",    yaw);   return false; }

    out->throttle = thr;
    out->roll     = rol / 10.0f;    // tenths-of-degree → degrees
    out->pitch    = pit / 10.0f;
    out->yaw_rate = yaw / 10.0f;
    out->flags    = flg;

    return true;
}

// ============================================================
// UDP Listener Task — Core 0
// ============================================================
static void udp_listener_task(void *arg)
{
    ESP_LOGI(TAG, "UDP listener starting on port %d (Core %d)",
             SETPOINT_UDP_PORT, UDP_TASK_CORE);

    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // 200 ms receive timeout — allows periodic staleness checks without spin-waiting
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Address reuse — allows clean restart without TIME_WAIT issues
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(SETPOINT_UDP_PORT),
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d — port %d may be in use",
                 errno, SETPOINT_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket bound — waiting for packets on 0.0.0.0:%d", SETPOINT_UDP_PORT);

    // Receive buffer — slightly oversized to detect truncation
    uint8_t buf[PACKET_SIZE + 8];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);
    char sender_str[INET_ADDRSTRLEN];

    bool     first_packet   = true;
    uint32_t stale_log_tick = 0;     // Rate-limit stale log spam

    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&sender, &sender_len);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Normal 200 ms timeout — check for staleness
                int64_t now_us  = esp_timer_get_time();
                int64_t age_ms  = (s_last_rx_us > 0) ?
                                  (now_us - s_last_rx_us) / 1000LL : -1;

                if (age_ms > (int64_t)SETPOINT_TIMEOUT_MS) {
                    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        s_setpoint.fresh = false;
                        xSemaphoreGive(s_mutex);
                    }
                    // Log every ~2 seconds to avoid flooding serial
                    if (stale_log_tick % 10 == 0 && s_last_rx_us > 0) {
                        ESP_LOGW(TAG, "Setpoint STALE — %lld ms since last packet", age_ms);
                    }
                    stale_log_tick++;
                }
            } else {
                ESP_LOGE(TAG, "recvfrom error: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        // Successfully received a datagram — parse it
        setpoint_t parsed = {0};
        if (!parse_packet(buf, n, &parsed)) {
            continue;
        }

        // Log first valid packet's origin (once)
        if (first_packet) {
            inet_ntop(AF_INET, &sender.sin_addr, sender_str, sizeof(sender_str));
            ESP_LOGI(TAG, "First valid packet from %s:%d",
                     sender_str, ntohs(sender.sin_port));
            first_packet = false;
        }

        // Commit to shared state under mutex
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s_setpoint.throttle     = parsed.throttle;
            s_setpoint.roll         = parsed.roll;
            s_setpoint.pitch        = parsed.pitch;
            s_setpoint.yaw_rate     = parsed.yaw_rate;
            s_setpoint.flags        = parsed.flags;
            s_setpoint.fresh        = true;
            s_setpoint.packet_count++;
            s_last_rx_us = esp_timer_get_time();
            xSemaphoreGive(s_mutex);
        }

        stale_log_tick = 0;

        // Periodic status log — every 100 packets (~2 s at 50 Hz)
        if (s_setpoint.packet_count % 100 == 0) {
            ESP_LOGI(TAG, "PKT #%lu | THR=%u µs | R=%.1f° P=%.1f° Y=%.1f°/s | flags=0x%02X",
                     (unsigned long)s_setpoint.packet_count,
                     s_setpoint.throttle,
                     s_setpoint.roll,
                     s_setpoint.pitch,
                     s_setpoint.yaw_rate,
                     s_setpoint.flags);
        }
    }

    // Never reached, but clean up if it ever is
    close(sock);
    vTaskDelete(NULL);
}

// ============================================================
// Public API
// ============================================================

esp_err_t setpoint_init(void)
{
    if (s_started) {
        ESP_LOGW(TAG, "Already initialised — ignoring second call");
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed — out of heap?");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t rc = xTaskCreatePinnedToCore(
        udp_listener_task,
        "setpoint_udp",
        UDP_TASK_STACK,
        NULL,
        UDP_TASK_PRIO,
        NULL,
        UDP_TASK_CORE
    );

    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed");
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Setpoint subsystem ready — UDP port %d, Core %d, prio %d",
             SETPOINT_UDP_PORT, UDP_TASK_CORE, UDP_TASK_PRIO);
    return ESP_OK;
}

void setpoint_get(setpoint_t *out)
{
    if (!out) return;

    // If not yet initialised, return safe defaults without crashing
    if (!s_mutex) {
        out->throttle     = 1000;
        out->roll         = 0.0f;
        out->pitch        = 0.0f;
        out->yaw_rate     = 0.0f;
        out->flags        = 0;
        out->fresh        = false;
        out->packet_count = 0;
        return;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(out, &s_setpoint, sizeof(setpoint_t));
        xSemaphoreGive(s_mutex);
    }
}

bool setpoint_is_fresh(void)
{
    if (!s_mutex) return false;
    bool fresh = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        fresh = s_setpoint.fresh;
        xSemaphoreGive(s_mutex);
    }
    return fresh;
}