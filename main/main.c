#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "driver/gpio.h"
#include "cJSON.h"

#include "esp_netif.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/dns.h"

#include "sdkconfig.h"


#define WIFI_SSID   "Tenda_4A8498"
#define WIFI_PASS   "Delete07"
#define BLYNK_AUTH  "BdkkgsrneObeqtF3FsumPZO6ou1da1hr"

#define LED_BLUE   48
#define LED_RED    47
#define LED_GREEN  21
#define LED_BUILTIN 38  // Built-in LED on most ESP32-S3 boards

static const char *TAG = "BLYNK_MQTT";

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static esp_mqtt_client_handle_t mqtt_client;

/* ================= WIFI ================= */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* ================= DNS CONFIGURATION ================= */

static void configure_dns(void)
{
    // Get the WiFi station interface
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "Failed to get netif handle");
        return;
    }

    // Configure primary DNS (Google)
    esp_netif_dns_info_t dns_main;
    inet_pton(AF_INET, "8.8.8.8", &dns_main.ip.u_addr.ip4);
    dns_main.ip.type = IPADDR_TYPE_V4;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_main);

    // Configure backup DNS (Cloudflare)
    esp_netif_dns_info_t dns_backup;
    inet_pton(AF_INET, "1.1.1.1", &dns_backup.ip.u_addr.ip4);
    dns_backup.ip.type = IPADDR_TYPE_V4;
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_backup);

    ESP_LOGI(TAG, "DNS servers configured: 8.8.8.8 (main), 1.1.1.1 (backup)");

    // Wait a bit for DNS to be ready
    vTaskDelay(pdMS_TO_TICKS(2000));
}

/* ================= GPIO ================= */

static void led_init(void)
{
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << LED_BLUE) |
            (1ULL << LED_RED) |
            (1ULL << LED_GREEN) |
            (1ULL << LED_BUILTIN)
    };
    gpio_config(&io);
    
    // Turn off all LEDs initially
    gpio_set_level(LED_BLUE, 0);
    gpio_set_level(LED_RED, 0);
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_BUILTIN, 0);
}

/* ================= BLYNK MQTT ================= */

static void blink_builtin_led(void)
{
    // Quick blink: ON for 100ms, then OFF
    gpio_set_level(LED_BUILTIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_BUILTIN, 0);
}

static void handle_blynk_message(const char *topic, const char *data, int data_len)
{
    // Create a null-terminated string
    char *json_str = malloc(data_len + 1);
    if (!json_str) {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON");
        return;
    }
    
    memcpy(json_str, data, data_len);
    json_str[data_len] = '\0';
    
    ESP_LOGI(TAG, "Received: %s", json_str);
    
    // Parse the value directly (Blynk sends simple values like "0" or "1")
    // For datastream messages, the format is just the raw value
    int value = atoi(json_str);
    
    // Blink built-in LED to indicate command received
    blink_builtin_led();
    
    // Extract pin name from topic: "downlink/ds/Blue RGB" -> "Blue RGB"
    const char *pin_name = strrchr(topic, '/');
    if (pin_name) {
        pin_name++; // Skip the '/'
        
        // Map datastream names to GPIO pins
        if (strcmp(pin_name, "Blue RGB") == 0 || 
            strcmp(pin_name, "v0") == 0 || 
            strcmp(pin_name, "V0") == 0) {
            gpio_set_level(LED_BLUE, value);
            ESP_LOGI(TAG, "✓ Blue LED -> %d", value);
        }
        else if (strcmp(pin_name, "Red RGB") == 0 || 
                 strcmp(pin_name, "v1") == 0 || 
                 strcmp(pin_name, "V1") == 0) {
            gpio_set_level(LED_RED, value);
            ESP_LOGI(TAG, "✓ Red LED -> %d", value);
        }
        else if (strcmp(pin_name, "Green RGB") == 0 || 
                 strcmp(pin_name, "v2") == 0 || 
                 strcmp(pin_name, "V2") == 0) {
            gpio_set_level(LED_GREEN, value);
            ESP_LOGI(TAG, "✓ Green LED -> %d", value);
        }
        else {
            ESP_LOGW(TAG, "Unknown pin: %s", pin_name);
        }
    }
    
    free(json_str);
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "✓ MQTT connected to Blynk!");
        esp_mqtt_client_subscribe(mqtt_client,
            "downlink/ds/#", 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Subscribed to topic");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT data received on topic: %.*s", event->topic_len, event->topic);
        
        // Create null-terminated topic string
        char *topic = malloc(event->topic_len + 1);
        if (topic) {
            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            handle_blynk_message(topic, event->data, event->data_len);
            free(topic);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error occurred");
        break;

    default:
        break;
    }
}


static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://blynk.cloud:1883",
            },
        },
        .credentials = {
            .username = "device",
            .authentication = {
                .password = BLYNK_AUTH,
            },
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started");
}

/* ================= DNS TEST ================= */

static bool test_dns_resolution(const char *hostname)
{
    ESP_LOGI(TAG, "Testing DNS resolution for: %s", hostname);
    
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    
    struct addrinfo *res = NULL;
    int err = getaddrinfo(hostname, "1883", &hints, &res);
    
    if (err == 0 && res != NULL) {
        struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
        char ip_str[16];
        inet_ntoa_r(addr->sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "✓ DNS resolved %s to: %s", hostname, ip_str);
        freeaddrinfo(res);
        return true;
    } else {
        ESP_LOGE(TAG, "✗ DNS resolution failed for %s: error=%d", hostname, err);
        return false;
    }
}

/* ================= MAIN ================= */

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize peripherals
    led_init();
    
    // Initialize WiFi
    wifi_init();

    // Wait for WiFi connection
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    xEventGroupWaitBits(wifi_event_group,
                       WIFI_CONNECTED_BIT,
                       false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "WiFi connected!");

    // Configure DNS servers
    configure_dns();

    // Test DNS resolution
    int retries = 3;
    bool dns_ok = false;
    
    for (int i = 0; i < retries && !dns_ok; i++) {
        if (i > 0) {
            ESP_LOGW(TAG, "Retry %d/%d", i + 1, retries);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        dns_ok = test_dns_resolution("blynk.cloud");
    }

    if (!dns_ok) {
        ESP_LOGE(TAG, "DNS resolution failed after %d retries. Check your network/router DNS settings.", retries);
        return;
    }

    // Start MQTT connection
    vTaskDelay(pdMS_TO_TICKS(1000));
    mqtt_start();
}