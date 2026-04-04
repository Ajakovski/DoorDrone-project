/*
 * ESP32-S3 WiFi-based Blynk MQTT Client
 * FIXED: Motor Control using MCPWM Groups 0 and 1
 *
 * V0 → ARM/DISARM switch (0 = disarmed, 1 = armed)
 * V1 → Throttle slider (1000-2000 µs PWM)
 * V2 → Roll  (°)  — published from IMU task (read-only from Blynk)
 * V3 → Pitch (°)  — published from IMU task (read-only from Blynk)
 * V4 → Yaw   (°)  — published from IMU task (read-only from Blynk)
 * V6 → LED control (0 = off, 1 = on)
 * V7 → ESC calibration trigger
 * V8 → Motor select for individual testing
 * V9 → Test pulse value
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
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
#include "driver/mcpwm_prelude.h"

#include "imu.h"   // MPU6050 driver

// ============================================================
// Configuration
// ============================================================
#define WIFI_SSID               "Tenda_4A8498"
#define WIFI_PASS               "Delete07"
#define BLYNK_TEMPLATE_ID       "TMPL4w_6WYcly"
#define BLYNK_TEMPLATE_NAME     "LED tutorial"
#define BLYNK_AUTH_TOKEN        "BdkkgsrneObeqtF3FsumPZO6ou1da1hr"
#define BLYNK_SERVER            "blynk.cloud"
#define FIRMWARE_VERSION        "1.2.0"   // Bumped: IMU added

#define USE_MQTTS               0

#if USE_MQTTS
    #define MQTT_PORT           8883
    extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_cert_pem_start");
#else
    #define MQTT_PORT           1883
#endif

// ============================================================
// Blynk Virtual Pin Topics (uplink: device → Blynk cloud)
// ============================================================
#define BLYNK_PIN_ROLL          "ds/V2"
#define BLYNK_PIN_PITCH         "ds/V3"
#define BLYNK_PIN_YAW           "ds/V4"

// IMU Blynk publish rate
#define IMU_BLYNK_PUBLISH_HZ    2                           // 2 updates/sec to Blynk
#define IMU_BLYNK_PERIOD_MS     (1000 / IMU_BLYNK_PUBLISH_HZ)

// ============================================================
// Hardware Pin Definitions
// ============================================================
#define LED_GPIO                GPIO_NUM_8

// ESC PWM Pins
#define ESC1_GPIO               GPIO_NUM_1
#define ESC2_GPIO               GPIO_NUM_5
#define ESC3_GPIO               GPIO_NUM_10
#define ESC4_GPIO               GPIO_NUM_11

// PWM Parameters for BLHeli ESCs
#define ESC_PWM_FREQ_HZ         50
#define ESC_PWM_PERIOD_US       20000
#define ESC_MIN_PULSE_US        1000
#define ESC_MAX_PULSE_US        2000
#define ESC_SAFE_PULSE_US       1000

// ============================================================
// System State
// ============================================================
typedef struct {
    bool armed;
    uint16_t throttle_us;
    bool led_state;
} system_state_t;

static system_state_t sys_state = {
    .armed        = false,
    .throttle_us  = ESC_SAFE_PULSE_US,
    .led_state    = false
};

// ============================================================
// Tags and Globals
// ============================================================
static const char *TAG      = "[MAIN]";
static const char *WIFI_TAG = "[WiFi]";
static const char *MQTT_TAG = "[MQTT]";
static const char *ESC_TAG  = "[ESC]";

static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client = NULL;

#define WIFI_CONNECTED_BIT      BIT0
#define MQTT_CONNECTED_BIT      BIT1

static TickType_t mqtt_offline_timestamp = 0;

// MCPWM handles
static mcpwm_timer_handle_t esc_timer[2] = {NULL};
static mcpwm_oper_handle_t  esc_oper[4]  = {NULL};
static mcpwm_cmpr_handle_t  esc_cmpr[4]  = {NULL};
static mcpwm_gen_handle_t   esc_gen[4]   = {NULL};

// ============================================================
// ESC PWM Initialization
// ============================================================
static void esc_pwm_init(void)
{
    ESP_LOGI(ESC_TAG, "Initializing ESC PWM outputs");
    ESP_LOGI(ESC_TAG, "ESP32-S3: MCPWM Group 0 (ESC1,2) + Group 1 (ESC3,4)");

    const int esc_gpios[4] = {ESC1_GPIO, ESC2_GPIO, ESC3_GPIO, ESC4_GPIO};

    for (int grp = 0; grp < 2; grp++) {
        mcpwm_timer_config_t timer_config = {
            .group_id      = grp,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,
            .period_ticks  = ESC_PWM_PERIOD_US,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &esc_timer[grp]));
    }

    for (int i = 0; i < 4; i++) {
        int group_id  = (i < 2) ? 0 : 1;
        int timer_idx = group_id;

        mcpwm_operator_config_t oper_config = { .group_id = group_id };
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &esc_oper[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(esc_oper[i], esc_timer[timer_idx]));

        mcpwm_comparator_config_t cmpr_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(esc_oper[i], &cmpr_config, &esc_cmpr[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_cmpr[i], ESC_SAFE_PULSE_US));

        mcpwm_generator_config_t gen_config = { .gen_gpio_num = esc_gpios[i] };
        ESP_ERROR_CHECK(mcpwm_new_generator(esc_oper[i], &gen_config, &esc_gen[i]));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(esc_gen[i],
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      MCPWM_TIMER_EVENT_EMPTY,
                                                      MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(esc_gen[i],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                        esc_cmpr[i],
                                                        MCPWM_GEN_ACTION_LOW)));

        ESP_LOGI(ESC_TAG, "ESC%d: GPIO%d, Group%d", i+1, esc_gpios[i], group_id);
    }

    for (int grp = 0; grp < 2; grp++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(esc_timer[grp]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(esc_timer[grp], MCPWM_TIMER_START_NO_STOP));
    }

    ESP_LOGI(ESC_TAG, "All 4 ESCs initialized at %d µs (DISARMED)", ESC_SAFE_PULSE_US);
}

// ============================================================
// ESC Control Functions
// ============================================================
static void esc_set_all_throttle(uint16_t pulse_us)
{
    if (pulse_us < ESC_MIN_PULSE_US) pulse_us = ESC_MIN_PULSE_US;
    if (pulse_us > ESC_MAX_PULSE_US) pulse_us = ESC_MAX_PULSE_US;

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_cmpr[i], pulse_us));
    }
    ESP_LOGI(ESC_TAG, "All ESCs set to %d µs", pulse_us);
}

static void esc_arm_motors(void)
{
    if (sys_state.armed) {
        ESP_LOGW(ESC_TAG, "Already armed");
        return;
    }

    // Safety: refuse to arm if IMU is not healthy
    if (!imu_is_healthy()) {
        ESP_LOGE(ESC_TAG, "ARM REFUSED — IMU not healthy. Check I2C wiring.");
        return;
    }

    ESP_LOGI(ESC_TAG, "Starting arming sequence");
    esc_set_all_throttle(ESC_SAFE_PULSE_US);
    vTaskDelay(pdMS_TO_TICKS(1000));

    sys_state.armed = true;

    // Reset yaw heading reference on every arm
    imu_reset_yaw();

    ESP_LOGI(ESC_TAG, "Motors ARMED — ready for throttle. Yaw reference set.");
}

static void esc_disarm_motors(void)
{
    if (!sys_state.armed) {
        ESP_LOGW(ESC_TAG, "Already disarmed");
        return;
    }

    ESP_LOGI(ESC_TAG, "DISARMING motors");
    sys_state.armed       = false;
    sys_state.throttle_us = ESC_SAFE_PULSE_US;
    esc_set_all_throttle(ESC_SAFE_PULSE_US);
    ESP_LOGI(ESC_TAG, "Motors DISARMED");
}

static void esc_calibration_manual_mode(void)
{
    ESP_LOGW(ESC_TAG, "╔════════════════════════════════════════╗");
    ESP_LOGW(ESC_TAG, "║   ESC CALIBRATION MODE - MANUAL        ║");
    ESP_LOGW(ESC_TAG, "║   REMOVE ALL PROPELLERS NOW            ║");
    ESP_LOGW(ESC_TAG, "╚════════════════════════════════════════╝");

    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(ESC_TAG, "Step 1: Setting MAX throttle (2000µs)");
    esc_set_all_throttle(ESC_MAX_PULSE_US);

    ESP_LOGW(ESC_TAG, "⚠  Connect battery NOW. Listen for 3 beeps.");
    ESP_LOGW(ESC_TAG, "   Waiting 8 seconds...");
    vTaskDelay(pdMS_TO_TICKS(8000));

    ESP_LOGI(ESC_TAG, "Step 2: Setting MIN throttle (1000µs)");
    esc_set_all_throttle(ESC_MIN_PULSE_US);

    ESP_LOGI(ESC_TAG, "✓ Listen for double-beep — calibration saved.");
    ESP_LOGI(ESC_TAG, "✓ ESCs calibrated. Arm normally to fly.");
}

static uint8_t test_motor_selected = 1;

static void esc_test_individual(int motor_num, uint16_t pulse_us)
{
    if (motor_num < 1 || motor_num > 4) {
        ESP_LOGE(ESC_TAG, "Invalid motor number: %d", motor_num);
        return;
    }
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_cmpr[i], ESC_MIN_PULSE_US));
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_cmpr[motor_num - 1], pulse_us));
    ESP_LOGI(ESC_TAG, "Motor %d set to %d µs", motor_num, pulse_us);
}

// ============================================================
// GPIO Initialization
// ============================================================
static void gpio_init_led(void)
{
    gpio_config_t io_conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(TAG, "LED initialized on GPIO%d", LED_GPIO);
}

// ============================================================
// Blynk IMU Publish Task — 2Hz
// Runs on Core 0 (same as WiFi/MQTT) but at low priority.
// Publishes Roll→V2, Pitch→V3, Yaw→V4 to Blynk cloud.
// ============================================================
static void blynk_imu_publish_task(void *arg)
{
    ESP_LOGI(TAG, "Blynk IMU publish task started (%dHz)", IMU_BLYNK_PUBLISH_HZ);

    // Wait until MQTT is up before publishing anything
    xEventGroupWaitBits(wifi_event_group, MQTT_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    char val_str[16];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        EventBits_t bits = xEventGroupGetBits(wifi_event_group);

        if ((bits & MQTT_CONNECTED_BIT) && mqtt_client) {
            imu_data_t imu;
            imu_get_data(&imu);

            if (imu.healthy && imu.calibrated) {
                snprintf(val_str, sizeof(val_str), "%.2f", imu.roll);
                esp_mqtt_client_publish(mqtt_client, BLYNK_PIN_ROLL, val_str, 0, 0, 0);

                snprintf(val_str, sizeof(val_str), "%.2f", imu.pitch);
                esp_mqtt_client_publish(mqtt_client, BLYNK_PIN_PITCH, val_str, 0, 0, 0);

                snprintf(val_str, sizeof(val_str), "%.2f", imu.yaw);
                esp_mqtt_client_publish(mqtt_client, BLYNK_PIN_YAW, val_str, 0, 0, 0);
            } else {
                ESP_LOGW(TAG, "IMU not ready for publish — healthy=%d calibrated=%d",
                         imu.healthy, imu.calibrated);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_BLYNK_PERIOD_MS));
    }
}

// ============================================================
// Blynk Virtual Pin Handlers
// ============================================================
static void handle_blynk_v0_arm(const char *value_str)
{
    int value = atoi(value_str);
    if      (value == 1) { ESP_LOGI(TAG, "V0 → ARM");   esc_arm_motors();   }
    else if (value == 0) { ESP_LOGI(TAG, "V0 → DISARM"); esc_disarm_motors(); }
    else                 { ESP_LOGW(TAG, "V0 invalid value: %s", value_str); }
}

static void handle_blynk_v1_throttle(const char *value_str)
{
    int value = atoi(value_str);
    if (value < ESC_MIN_PULSE_US || value > ESC_MAX_PULSE_US) {
        ESP_LOGW(TAG, "V1 value %d out of range [%d-%d]",
                 value, ESC_MIN_PULSE_US, ESC_MAX_PULSE_US);
        return;
    }
    sys_state.throttle_us = (uint16_t)value;
    if (sys_state.armed) {
        esc_set_all_throttle(sys_state.throttle_us);
        ESP_LOGI(TAG, "V1 → Throttle %d µs (ARMED)", sys_state.throttle_us);
    } else {
        ESP_LOGW(TAG, "V1 → Throttle %d µs received but DISARMED (ignored)",
                 sys_state.throttle_us);
    }
}

static void handle_blynk_v6_led(const char *value_str)
{
    int value = atoi(value_str);
    if (value == 0 || value == 1) {
        sys_state.led_state = (bool)value;
        gpio_set_level(LED_GPIO, value);
        ESP_LOGI(TAG, "V6 → LED GPIO%d = %d", LED_GPIO, value);
    } else {
        ESP_LOGW(TAG, "V6 invalid value: %s", value_str);
    }
}

static void handle_blynk_v7_calibrate(const char *value_str)
{
    if (atoi(value_str) == 1) {
        ESP_LOGW(TAG, "V7 → ESC CALIBRATION TRIGGERED");
        esc_calibration_manual_mode();
    }
}

static void handle_blynk_v8_motor_select(const char *value_str)
{
    int value = atoi(value_str);
    if (value >= 1 && value <= 4) {
        test_motor_selected = (uint8_t)value;
        ESP_LOGI(TAG, "V8 → Motor %d selected for testing", value);
    }
}

static void handle_blynk_v9_test_pulse(const char *value_str)
{
    int value = atoi(value_str);
    if (value >= 1000 && value <= 2000) {
        esc_test_individual(test_motor_selected, (uint16_t)value);
        ESP_LOGI(TAG, "V9 → Motor %d @ %d µs", test_motor_selected, value);
    }
}

// ============================================================
// WiFi Event Handler
// ============================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(WIFI_TAG, "Disconnected, retrying...");
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ============================================================
// WiFi Initialization
// ============================================================
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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ============================================================
// MQTT Event Handler
// ============================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event  = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT Connected");
            mqtt_offline_timestamp = 0;
            xEventGroupSetBits(wifi_event_group, MQTT_CONNECTED_BIT);

            esp_mqtt_client_subscribe(client, "downlink/#", 0);

            {
                cJSON *info = cJSON_CreateObject();
                cJSON_AddStringToObject(info, "tmpl",  BLYNK_TEMPLATE_ID);
                cJSON_AddStringToObject(info, "ver",   FIRMWARE_VERSION);
                cJSON_AddStringToObject(info, "build", __DATE__ " " __TIME__);
                cJSON_AddStringToObject(info, "type",  BLYNK_TEMPLATE_ID);
                cJSON_AddNumberToObject(info, "rxbuff", 512);
                char *info_str = cJSON_PrintUnformatted(info);
                esp_mqtt_client_publish(client, "info/mcu", info_str, 0, 1, 0);
                free(info_str);
                cJSON_Delete(info);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(MQTT_TAG, "MQTT Disconnected");
            xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);

            if (sys_state.armed) {
                ESP_LOGE(ESC_TAG, "MQTT lost while ARMED — emergency DISARM");
                esc_disarm_motors();
            }

            if (!mqtt_offline_timestamp) {
                mqtt_offline_timestamp = xTaskGetTickCount();
            }
            break;

        case MQTT_EVENT_DATA: {
            ESP_LOGI(MQTT_TAG, "Topic: %.*s", event->topic_len, event->topic);
            ESP_LOGI(MQTT_TAG, "Data:  %.*s", event->data_len,  event->data);

            if (event->topic_len > 13 && memcmp(event->topic, "downlink/ds/", 12) == 0) {
                const char *pin_start = event->topic + 12;
                int pin_len = event->topic_len - 12;

                char value_str[32];
                int copy_len = (event->data_len < (int)sizeof(value_str) - 1) ?
                                event->data_len : (int)sizeof(value_str) - 1;
                memcpy(value_str, event->data, copy_len);
                value_str[copy_len] = '\0';

                if ((pin_len == 2 && memcmp(pin_start, "V0", 2) == 0) ||
                    (pin_len == 1 && memcmp(pin_start, "0",  1) == 0)) {
                    handle_blynk_v0_arm(value_str);
                }
                else if ((pin_len == 2 && memcmp(pin_start, "V1", 2) == 0) ||
                         (pin_len == 1 && memcmp(pin_start, "1",  1) == 0)) {
                    handle_blynk_v1_throttle(value_str);
                }
                else if ((pin_len == 2 && memcmp(pin_start, "V6", 2) == 0) ||
                         (pin_len == 1 && memcmp(pin_start, "6",  1) == 0)) {
                    handle_blynk_v6_led(value_str);
                }
                else if ((pin_len == 2 && memcmp(pin_start, "V7", 2) == 0) ||
                         (pin_len == 1 && memcmp(pin_start, "7",  1) == 0)) {
                    handle_blynk_v7_calibrate(value_str);
                }
                else if ((pin_len == 2 && memcmp(pin_start, "V8", 2) == 0) ||
                         (pin_len == 1 && memcmp(pin_start, "8",  1) == 0)) {
                    handle_blynk_v8_motor_select(value_str);
                }
                else if ((pin_len == 2 && memcmp(pin_start, "V9", 2) == 0) ||
                         (pin_len == 1 && memcmp(pin_start, "9",  1) == 0)) {
                    handle_blynk_v9_test_pulse(value_str);
                }
                // V2, V3, V4 are read-only (IMU output) — no downlink handler needed
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(MQTT_TAG, "MQTT Error");
            break;

        default:
            break;
    }
}

// ============================================================
// MQTT Initialization
// ============================================================
static void mqtt_app_start(void)
{
    char uri[256];

    if (strncmp(BLYNK_TEMPLATE_ID, "TMPL", 4) != 0) {
        ESP_LOGE(MQTT_TAG, "Invalid BLYNK_TEMPLATE_ID");
        return;
    }

#if USE_MQTTS
    snprintf(uri, sizeof(uri), "mqtts://device:%s@%s:%d",
             BLYNK_AUTH_TOKEN, BLYNK_SERVER, MQTT_PORT);
#else
    snprintf(uri, sizeof(uri), "mqtt://device:%s@%s:%d",
             BLYNK_AUTH_TOKEN, BLYNK_SERVER, MQTT_PORT);
#endif

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
#if USE_MQTTS
        .broker.verification.certificate = (const char *)ca_cert_pem_start,
#endif
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                                    mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// ============================================================
// NVS Initialization
// ============================================================
static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// ============================================================
// Application Entry Point
// ============================================================
void app_main(void)
{
    volatile const char firmwareTag[] = "blnkinf\0"
        "mcu\0" FIRMWARE_VERSION "\0"
        "fw-type\0" BLYNK_TEMPLATE_ID "\0"
        "build\0" __DATE__ " " __TIME__ "\0"
        "\0";
    (void)firmwareTag;

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "ESP32-S3 Blynk Motor Control + IMU");
    ESP_LOGI(TAG, "Firmware: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGW(TAG, "SAFETY: Motors start DISARMED");

    // -- Subsystem init order matters --
    nvs_init();
    gpio_init_led();
    esc_pwm_init();

    // IMU init is BLOCKING (~3s for calibration).
    // Drone must be flat and still at this point.
    ESP_LOGI(TAG, "Starting IMU initialization — keep drone stationary...");
    esp_err_t imu_ret = imu_init();
    if (imu_ret != ESP_OK) {
        // Non-fatal for now, but log loudly.
        // TODO: In flight firmware, refuse to proceed without a healthy IMU.
        ESP_LOGE(TAG, "IMU INIT FAILED (%s) — attitude data unavailable!", 
                 esp_err_to_name(imu_ret));
    }

    wifi_init();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    mqtt_app_start();

    // Blynk IMU publish task — Core 0, low priority
    xTaskCreatePinnedToCore(
        blynk_imu_publish_task,
        "imu_blynk",
        3072,
        NULL,
        3,      // Low priority — best-effort publishing
        NULL,
        0       // Core 0, same as WiFi/MQTT
    );

    // Main monitoring loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Periodic health report
        imu_data_t imu;
        imu_get_data(&imu);

        ESP_LOGI(TAG, "--- System Status ---");
        ESP_LOGI(TAG, "  Armed:   %s", sys_state.armed ? "YES" : "no");
        ESP_LOGI(TAG, "  IMU:     %s | Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                 imu.healthy ? "OK" : "FAIL",
                 imu.roll, imu.pitch, imu.yaw);
        ESP_LOGI(TAG, "  IMU Temp: %.1f°C | Updates: %lu",
                 imu.temp_c, (unsigned long)imu.update_count);

        // Safety: if armed and IMU unhealthy, disarm
        if (sys_state.armed && !imu_is_healthy()) {
            ESP_LOGE(TAG, "CRITICAL: IMU unhealthy while ARMED — forcing DISARM");
            esc_disarm_motors();
        }
    }
}