/*
 * main.c — Orchestration Layer  v1.6.0 — Emergency Landing
 *
 * Handles: WiFi, MQTT/Blynk telemetry, NVS, LED.
 * All flight control is delegated to flight_control.c.
 *
 * Blynk virtual pin map:
 *   V0  → ARM/DISARM (1=arm, 0=immediate disarm — pilot intent only)
 *   V2  → Roll  telemetry (°)       [read-only]
 *   V3  → Pitch telemetry (°)       [read-only]
 *   V4  → Yaw   telemetry (°)       [read-only]
 *   V5  → Outer loop Kp (roll=pitch, float 0.0–20.0)
 *   V6  → LED
 *   V7  → ESC calibration trigger
 *   V8  → Motor test select (1–4)
 *   V9  → Motor test pulse (µs)
 *
 * Flight control input:
 *   UDP port 4210 — 12-byte setpoint packets
 *   flags byte bit 1 (0x02) = emergency landing trigger from HTML RC controller
 *
 * Emergency landing is triggered by:
 *   - MQTT disconnect while armed  → flight_control_trigger_emergency()
 *   - UDP setpoint stale > 600ms   → handled inside flight_control.c
 *   - HTML RC emergency button     → UDP flags bit 1 (0x02), handled in flight_control.c
 *
 * Manual disarm via V0 is always immediate — it is a pilot command.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "cJSON.h"

#include "driver/gpio.h"

#include "imu.h"
#include "setpoint.h"
#include "flight_control.h"

// ============================================================
// Configuration
// ============================================================
#define WIFI_SSID               "Tenda_4A8498"
#define WIFI_PASS               "Delete07"
#define BLYNK_TEMPLATE_ID       "TMPL4w_6WYcly"
#define BLYNK_TEMPLATE_NAME     "LED tutorial"
#define BLYNK_AUTH_TOKEN        "BdkkgsrneObeqtF3FsumPZO6ou1da1hr"
#define BLYNK_SERVER            "blynk.cloud"
#define FIRMWARE_VERSION        "1.6.0"

#define MQTT_PORT               1883

#define BLYNK_PIN_ROLL          "ds/V2"
#define BLYNK_PIN_PITCH         "ds/V3"
#define BLYNK_PIN_YAW           "ds/V4"

#define IMU_BLYNK_HZ            2
#define IMU_BLYNK_PERIOD_MS     (1000 / IMU_BLYNK_HZ)

#define LED_GPIO                GPIO_NUM_8

// ============================================================
// Globals
// ============================================================
static const char *TAG      = "[MAIN]";
static const char *WIFI_TAG = "[WiFi]";
static const char *MQTT_TAG = "[MQTT]";

static EventGroupHandle_t       wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client     = NULL;
static TickType_t               mqtt_offline_ts = 0;

#define WIFI_CONNECTED_BIT  BIT0
#define MQTT_CONNECTED_BIT  BIT1

static uint8_t test_motor = 1;

// ============================================================
// LED
// ============================================================
static void gpio_init_led(void)
{
    gpio_config_t cfg = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LED_GPIO,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(LED_GPIO, 0);
}

// ============================================================
// Blynk IMU telemetry — 2 Hz, Core 0
// ============================================================
static void blynk_imu_publish_task(void *arg)
{
    xEventGroupWaitBits(wifi_event_group, MQTT_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    char buf[16];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if ((xEventGroupGetBits(wifi_event_group) & MQTT_CONNECTED_BIT) && mqtt_client) {
            imu_data_t imu;
            imu_get_data(&imu);
            if (imu.healthy && imu.calibrated) {
                snprintf(buf, sizeof(buf), "%.2f", imu.roll);
                esp_mqtt_client_publish(mqtt_client, BLYNK_PIN_ROLL, buf, 0, 0, 0);
                snprintf(buf, sizeof(buf), "%.2f", imu.pitch);
                esp_mqtt_client_publish(mqtt_client, BLYNK_PIN_PITCH, buf, 0, 0, 0);
                snprintf(buf, sizeof(buf), "%.2f", imu.yaw);
                esp_mqtt_client_publish(mqtt_client, BLYNK_PIN_YAW, buf, 0, 0, 0);
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_BLYNK_PERIOD_MS));
    }
}

// ============================================================
// Blynk virtual pin handlers
// ============================================================
static void handle_v0_arm(const char *v)
{
    if (atoi(v) == 1) {
        flight_control_arm();
    } else {
        // V0=0 is intentional pilot disarm — always immediate
        flight_control_disarm();
    }
}

static void handle_v5_outer_kp(const char *v)
{
    float kp = strtof(v, NULL);
    ESP_LOGI(TAG, "V5 → Outer Kp = %.2f", kp);
    flight_control_set_angle_gains(kp, kp);
}

static void handle_v6_led(const char *v)
{
    int val = atoi(v);
    if (val == 0 || val == 1) gpio_set_level(LED_GPIO, val);
}

static void handle_v7_calibrate(const char *v)
{
    if (atoi(v) == 1) flight_control_calibrate_escs();
}

static void handle_v8_motor_select(const char *v)
{
    int val = atoi(v);
    if (val >= 1 && val <= 4) {
        test_motor = (uint8_t)val;
        ESP_LOGI(TAG, "V8 → test motor %d", val);
    }
}

static void handle_v9_test_pulse(const char *v)
{
    int val = atoi(v);
    if (val >= 1000 && val <= 2000)
        flight_control_motor_test(test_motor, (uint16_t)val);
}

// ============================================================
// WiFi
// ============================================================
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));
    wifi_config_t wcfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ============================================================
// MQTT / Blynk
// ============================================================
static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t  event  = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "Connected");
            mqtt_offline_ts = 0;
            xEventGroupSetBits(wifi_event_group, MQTT_CONNECTED_BIT);
            esp_mqtt_client_subscribe(client, "downlink/#", 0);
            {
                cJSON *info = cJSON_CreateObject();
                cJSON_AddStringToObject(info, "tmpl",  BLYNK_TEMPLATE_ID);
                cJSON_AddStringToObject(info, "ver",   FIRMWARE_VERSION);
                cJSON_AddStringToObject(info, "build", __DATE__ " " __TIME__);
                cJSON_AddStringToObject(info, "type",  BLYNK_TEMPLATE_ID);
                cJSON_AddNumberToObject(info, "rxbuff", 512);
                char *s = cJSON_PrintUnformatted(info);
                esp_mqtt_client_publish(client, "info/mcu", s, 0, 1, 0);
                free(s);
                cJSON_Delete(info);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(MQTT_TAG, "Disconnected");
            xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);
            // MQTT loss while flying is a fault condition — trigger emergency landing,
            // NOT immediate disarm. The drone levels itself then descends safely.
            // Only ARMED_NORMAL needs this — emergency states handle themselves.
            if (flight_control_get_state() == FC_STATE_ARMED_NORMAL) {
                ESP_LOGE(TAG, "MQTT lost while armed — EMERGENCY LANDING");
                flight_control_trigger_emergency();
            }
            if (!mqtt_offline_ts) mqtt_offline_ts = xTaskGetTickCount();
            break;

        case MQTT_EVENT_DATA: {
            if (event->topic_len > 12 && memcmp(event->topic, "downlink/ds/", 12) == 0) {
                const char *pin = event->topic + 12;
                int plen        = event->topic_len - 12;

                char val[32];
                int n = (event->data_len < (int)sizeof(val) - 1) ?
                         event->data_len : (int)sizeof(val) - 1;
                memcpy(val, event->data, n);
                val[n] = '\0';

#define PM(s) ((plen==2 && !memcmp(pin,"V"s,2)) || (plen==1 && pin[0]==(s[0])))
                if      (PM("0")) handle_v0_arm(val);
                else if (PM("5")) handle_v5_outer_kp(val);
                else if (PM("6")) handle_v6_led(val);
                else if (PM("7")) handle_v7_calibrate(val);
                else if (PM("8")) handle_v8_motor_select(val);
                else if (PM("9")) handle_v9_test_pulse(val);
#undef PM
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(MQTT_TAG, "Error");
            break;

        default: break;
    }
}

static void mqtt_app_start(void)
{
    char uri[256];
    snprintf(uri, sizeof(uri), "mqtt://device:%s@%s:%d",
             BLYNK_AUTH_TOKEN, BLYNK_SERVER, MQTT_PORT);
    esp_mqtt_client_config_t cfg = { .broker.address.uri = uri };
    mqtt_client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                                    mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

static void nvs_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
}

// ============================================================
// app_main
// ============================================================
void app_main(void)
{
    ESP_LOGI(TAG, "══════════════════════════════════════════════");
    ESP_LOGI(TAG, "  ESP32-S3 Drone Controller  v%s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "  Phase 3: Cascade PID + Emergency Landing");
    ESP_LOGI(TAG, "══════════════════════════════════════════════");
    ESP_LOGW(TAG, "SAFETY: All motors start DISARMED");

    nvs_init();
    gpio_init_led();

    // IMU: BLOCKING ~3s — keep drone flat and still
    ESP_LOGI(TAG, "IMU init — keep drone stationary...");
    if (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU FAILED — halting");
        while (1) vTaskDelay(portMAX_DELAY);
    }

    // Flight control: MCPWM + cascade PID task (Core 1)
    if (flight_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "FLIGHT CONTROL INIT FAILED — halting");
        while (1) vTaskDelay(portMAX_DELAY);
    }

    wifi_init();
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    if (setpoint_init() != ESP_OK) {
        ESP_LOGE(TAG, "SETPOINT INIT FAILED");
    }

    mqtt_app_start();

    xTaskCreatePinnedToCore(blynk_imu_publish_task, "imu_blynk",
                            3072, NULL, 3, NULL, 0);

    // ── Monitoring loop — 5s interval ────────────────────────
    static const char *state_str[] = {
        "DISARMED",
        "ARMED",
        "EMERG-LEVEL",
        "EMERG-LAND"
    };

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        imu_data_t        imu;
        setpoint_t        sp;
        fc_status_t       fc;

        imu_get_data(&imu);
        setpoint_get(&sp);
        flight_control_get_status(&fc);

        fc_flight_state_t state = flight_control_get_state();
        const char *sstr = (state <= FC_STATE_EMERGENCY_LAND) ?
                           state_str[state] : "UNKNOWN";

        ESP_LOGI(TAG, "══ Health (5s) ══════════════════════════════════════");
        ESP_LOGI(TAG, "  IMU   : %s | R=%+5.1f°  P=%+5.1f°  Y=%+5.1f°  %.1f°C",
                 imu.healthy ? "OK  " : "FAIL",
                 imu.roll, imu.pitch, imu.yaw, imu.temp_c);
        ESP_LOGI(TAG, "  UDP   : %s | THR=%u  R=%+5.1f°  P=%+5.1f°  Y=%+5.1f°/s",
                 sp.fresh ? "FRESH" : "STALE",
                 sp.throttle, sp.roll, sp.pitch, sp.yaw_rate);
        ESP_LOGI(TAG, "  FC    : %s | loops=%lu | emerg_remain=%lu ms",
                 sstr,
                 (unsigned long)fc.loop_count,
                 (unsigned long)fc.emergency_phase_ms_remaining);

        if (state == FC_STATE_ARMED_NORMAL) {
            ESP_LOGI(TAG, "  OUTER  Roll : sp=%+5.1f°  meas=%+5.1f°  err=%+4.1f°  → %+6.1f°/s",
                     fc.angle_sp[0], fc.angle_meas[0],
                     fc.angle_err[0], fc.angle_pid_out[0]);
            ESP_LOGI(TAG, "  OUTER  Pitch: sp=%+5.1f°  meas=%+5.1f°  err=%+4.1f°  → %+6.1f°/s",
                     fc.angle_sp[1], fc.angle_meas[1],
                     fc.angle_err[1], fc.angle_pid_out[1]);
            ESP_LOGI(TAG, "  INNER  Roll : sp=%+5.1f°/s  gyro=%+5.1f°/s  out=%+5.1fµs",
                     fc.rate_sp[0], fc.rate_meas[0], fc.rate_pid_out[0]);
            ESP_LOGI(TAG, "  INNER  Pitch: sp=%+5.1f°/s  gyro=%+5.1f°/s  out=%+5.1fµs",
                     fc.rate_sp[1], fc.rate_meas[1], fc.rate_pid_out[1]);
            ESP_LOGI(TAG, "  INNER  Yaw  : sp=%+5.1f°/s  gyro=%+5.1f°/s  out=%+5.1fµs",
                     fc.rate_sp[2], fc.rate_meas[2], fc.rate_pid_out[2]);
            ESP_LOGI(TAG, "  MOTORS: M1=%u  M2=%u  M3=%u  M4=%u",
                     fc.motor_us[0], fc.motor_us[1],
                     fc.motor_us[2], fc.motor_us[3]);
        } else if (state == FC_STATE_EMERGENCY_LEVEL ||
                   state == FC_STATE_EMERGENCY_LAND) {
            ESP_LOGW(TAG, "  MOTORS: M1=%u  M2=%u  M3=%u  M4=%u",
                     fc.motor_us[0], fc.motor_us[1],
                     fc.motor_us[2], fc.motor_us[3]);
        }

        ESP_LOGI(TAG, "════════════════════════════════════════════════════");
    }
}