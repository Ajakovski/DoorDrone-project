/*
 * imu.c — MPU6050 Driver Implementation
 *
 * ESP-IDF 5.x new I2C master API.
 * Task pinned to Core 1 at 100Hz.
 */

#include "imu.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "[IMU]";

// ============================================================
// MPU6050 Register Map
// ============================================================
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A    // DLPF config
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B   // First byte of 14-byte burst read
#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_GYRO_XOUT_H     0x43
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

// ============================================================
// Scale Factors
// ============================================================
// Gyro  ±250°/s  → 131.0 LSB/(°/s)
// Accel ±2g      → 16384 LSB/g
// Temp  formula: °C = raw/340.0 + 36.53
#define GYRO_SCALE_DPS          (1.0f / 131.0f)
#define ACCEL_SCALE_G           (1.0f / 16384.0f)
#define TEMP_SCALE              (1.0f / 340.0f)
#define TEMP_OFFSET_C           36.53f

#define DEG_TO_RAD              (3.14159265f / 180.0f)
#define RAD_TO_DEG              (180.0f / 3.14159265f)

// ============================================================
// Task Parameters
// ============================================================
#define IMU_TASK_STACK_SIZE     4096
#define IMU_TASK_PRIORITY       10      // Above normal, Core 1 (WiFi is on Core 0)
#define IMU_TASK_CORE           1

// ============================================================
// Internal State
// ============================================================
static i2c_master_bus_handle_t  s_i2c_bus    = NULL;
static i2c_master_dev_handle_t  s_mpu_dev    = NULL;
static SemaphoreHandle_t        s_mutex      = NULL;

static imu_data_t   s_shared_data;
static int64_t      s_last_update_us = 0;

// Calibration offsets in raw LSB
static float s_gyro_off_x  = 0.0f;
static float s_gyro_off_y  = 0.0f;
static float s_gyro_off_z  = 0.0f;
static float s_accel_off_x = 0.0f;
static float s_accel_off_y = 0.0f;
static float s_accel_off_z = 0.0f;  // Removes the 1g component on Z

// ============================================================
// Low-Level I2C Helpers
// ============================================================
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_mpu_dev, buf, sizeof(buf), 10);
}

static esp_err_t mpu_read_burst(uint8_t start_reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_mpu_dev, &start_reg, 1, data, len, 10);
}

// ============================================================
// MPU6050 Hardware Configuration
// ============================================================
static esp_err_t mpu6050_configure(void)
{
    // WHO_AM_I verification
    uint8_t who = 0;
    esp_err_t ret = mpu_read_burst(MPU_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed — check wiring on GPIO%d (SDA) / GPIO%d (SCL)",
                 IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO);
        return ESP_ERR_NOT_FOUND;
    }
    if (who != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X, expected 0x68. Check AD0 pin (must be GND)", who);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MPU6050 found at 0x68 — WHO_AM_I=0x%02X", who);

    // Full device reset
    ret = mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(150));

    // Wake up, select X gyro as clock source (more stable than internal 8MHz RC)
    ret = mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(20));

    // DLPF_CFG = 3:
    //   Accel: 44Hz bandwidth, 4.9ms delay
    //   Gyro:  42Hz bandwidth, 4.8ms delay
    // Good vibration rejection while preserving attitude dynamics.
    ret = mpu_write_reg(MPU_REG_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;

    // Sample rate = Gyro output rate / (1 + SMPLRT_DIV)
    // With DLPF enabled, gyro output rate = 1kHz
    // SMPLRT_DIV = 9 → 1000 / (1+9) = 100Hz — matches our task rate
    ret = mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x09);
    if (ret != ESP_OK) return ret;

    // Gyro full-scale: ±250°/s (FS_SEL = 0) → 131 LSB/(°/s)
    // Sufficient for aggressive maneuvers; upgrade to ±500 if needed
    ret = mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // Accel full-scale: ±2g (AFS_SEL = 0) → 16384 LSB/g
    ret = mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 configured: 100Hz | DLPF=3 | Gyro±250°/s | Accel±2g | Clk=X-Gyro");
    return ESP_OK;
}

// ============================================================
// 14-Byte Burst Read: Accel(6) + Temp(2) + Gyro(6)
// ============================================================
static esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                   int16_t *temp,
                                   int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    esp_err_t ret = mpu_read_burst(MPU_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    *ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    *az   = (int16_t)((buf[4]  << 8) | buf[5]);
    *temp = (int16_t)((buf[6]  << 8) | buf[7]);
    *gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy   = (int16_t)((buf[10] << 8) | buf[11]);
    *gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

// ============================================================
// Boot Calibration
// ============================================================
static void mpu6050_calibrate(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BOOT CALIBRATION — DO NOT MOVE DRONE   ║");
    ESP_LOGI(TAG, "║  Collecting %3d samples (%.1fs)...      ║",
             IMU_CALIB_SAMPLES, IMU_CALIB_SAMPLES * IMU_TASK_PERIOD_MS / 1000.0f);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");

    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    int valid = 0;

    for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
        int16_t ax, ay, az, temp, gx, gy, gz;
        if (mpu6050_read_raw(&ax, &ay, &az, &temp, &gx, &gy, &gz) == ESP_OK) {
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            sum_ax += ax;
            sum_ay += ay;
            sum_az += az;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }

    if (valid < IMU_CALIB_SAMPLES / 2) {
        ESP_LOGE(TAG, "Calibration failed — only %d/%d valid reads. Check I2C wiring.",
                 valid, IMU_CALIB_SAMPLES);
        return;
    }

    s_gyro_off_x  = (float)(sum_gx / valid);
    s_gyro_off_y  = (float)(sum_gy / valid);
    s_gyro_off_z  = (float)(sum_gz / valid);
    s_accel_off_x = (float)(sum_ax / valid);
    s_accel_off_y = (float)(sum_ay / valid);
    s_accel_off_z = (float)(sum_az / valid) - 16384.0f;  // Remove nominal 1g on Z

    float max_gyro_off_lsb = fmaxf(fabsf(s_gyro_off_x),
                              fmaxf(fabsf(s_gyro_off_y), fabsf(s_gyro_off_z)));

    ESP_LOGI(TAG, "Calibration done (%d/%d samples used):", valid, IMU_CALIB_SAMPLES);
    ESP_LOGI(TAG, "  Gyro offsets  [LSB]: X=%.1f  Y=%.1f  Z=%.1f  (%.2f  %.2f  %.2f °/s)",
             s_gyro_off_x, s_gyro_off_y, s_gyro_off_z,
             s_gyro_off_x * GYRO_SCALE_DPS,
             s_gyro_off_y * GYRO_SCALE_DPS,
             s_gyro_off_z * GYRO_SCALE_DPS);
    ESP_LOGI(TAG, "  Accel offsets [LSB]: X=%.1f  Y=%.1f  Z=%.1f  (%.4f  %.4f  %.4f g)",
             s_accel_off_x, s_accel_off_y, s_accel_off_z,
             s_accel_off_x * ACCEL_SCALE_G,
             s_accel_off_y * ACCEL_SCALE_G,
             s_accel_off_z * ACCEL_SCALE_G);

    // Sanity check — warn if drone was likely moving
    // 1000 LSB ≈ 7.6 °/s — clearly not stationary
    if (max_gyro_off_lsb > 1000.0f) {
        ESP_LOGW(TAG, "WARNING: Gyro offsets are unusually large (%.1f LSB = %.1f°/s).",
                 max_gyro_off_lsb, max_gyro_off_lsb * GYRO_SCALE_DPS);
        ESP_LOGW(TAG, "         Was the drone stationary? Reboot for a clean calibration.");
    }
}

// ============================================================
// IMU Task — 100Hz, Core 1
// ============================================================
static void imu_task(void *arg)
{
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;

    int64_t prev_time_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        int16_t ax_r, ay_r, az_r, temp_r, gx_r, gy_r, gz_r;
        esp_err_t ret = mpu6050_read_raw(&ax_r, &ay_r, &az_r, &temp_r,
                                          &gx_r, &gy_r, &gz_r);

        if (ret == ESP_OK) {
            // Compute dt from real elapsed time (handles any jitter)
            int64_t now_us = esp_timer_get_time();
            float dt = (float)(now_us - prev_time_us) * 1e-6f;
            prev_time_us = now_us;

            // Guard against runaway dt on first iteration or after stall
            if (dt <= 0.0f || dt > 0.5f) {
                dt = (float)IMU_TASK_PERIOD_MS * 0.001f;
            }

            // Apply calibration offsets and convert to physical units
            float gx = (gx_r - s_gyro_off_x) * GYRO_SCALE_DPS;   // °/s
            float gy = (gy_r - s_gyro_off_y) * GYRO_SCALE_DPS;
            float gz = (gz_r - s_gyro_off_z) * GYRO_SCALE_DPS;

            float ax = (ax_r - s_accel_off_x) * ACCEL_SCALE_G;    // g
            float ay = (ay_r - s_accel_off_y) * ACCEL_SCALE_G;
            float az = (az_r - s_accel_off_z) * ACCEL_SCALE_G;

            float temp_c = (float)temp_r * TEMP_SCALE + TEMP_OFFSET_C;

            // --------------------------------------------------------
            // Accelerometer-derived roll and pitch
            // Assumes: X forward, Y left, Z up (standard flat mount)
            // If your mount differs, negate ax/ay/az as needed.
            // --------------------------------------------------------
            float roll_acc  = atan2f(ay, az) * RAD_TO_DEG;
            float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

            // --------------------------------------------------------
            // Complementary filter
            // CF_ALPHA = 0.98 → 98% trust in gyro integration (fast, no vibration)
            //                    2% correction from accelerometer (long-term drift)
            // --------------------------------------------------------
            roll  = IMU_CF_ALPHA * (roll  + gx * dt) + (1.0f - IMU_CF_ALPHA) * roll_acc;
            pitch = IMU_CF_ALPHA * (pitch + gy * dt) + (1.0f - IMU_CF_ALPHA) * pitch_acc;

            // Yaw: pure gyro integration — no accelerometer correction available
            // Drifts ~1-5°/min at room temperature. Acceptable for short flights.
            // NOTE: imu_reset_yaw() called on arm to start from a known reference.
            yaw += gz * dt;

            // Wrap yaw to [-180, +180]
            if      (yaw >  180.0f) yaw -= 360.0f;
            else if (yaw < -180.0f) yaw += 360.0f;

            // Thread-safe update of shared data
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_shared_data.roll         = roll;
                s_shared_data.pitch        = pitch;
                s_shared_data.yaw          = yaw;
                s_shared_data.gyro_x       = gx;
                s_shared_data.gyro_y       = gy;
                s_shared_data.gyro_z       = gz;
                s_shared_data.accel_x      = ax;
                s_shared_data.accel_y      = ay;
                s_shared_data.accel_z      = az;
                s_shared_data.temp_c       = temp_c;
                s_shared_data.healthy      = true;
                s_shared_data.calibrated   = true;
                s_shared_data.update_count++;
                xSemaphoreGive(s_mutex);
            }

            s_last_update_us = esp_timer_get_time();

        } else {
            ESP_LOGW(TAG, "I2C read error: %s", esp_err_to_name(ret));

            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                s_shared_data.healthy = false;
                xSemaphoreGive(s_mutex);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

// ============================================================
// Public API
// ============================================================

esp_err_t imu_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing MPU6050 on SDA=GPIO%d SCL=GPIO%d @ %dkHz addr=0x%02X",
             IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO,
             IMU_I2C_FREQ_HZ / 1000, IMU_MPU6050_ADDR);

    // --- I2C Bus ---
    i2c_master_bus_config_t bus_cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = I2C_NUM_0,
        .scl_io_num             = IMU_I2C_SCL_GPIO,
        .sda_io_num             = IMU_I2C_SDA_GPIO,
        .glitch_ignore_cnt      = 7,
        // HW-123 has 4.7kΩ pull-ups. Internal 45kΩ won't interfere — enable for robustness.
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- MPU6050 Device ---
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = IMU_MPU6050_ADDR,
        .scl_speed_hz    = IMU_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- Configure sensor registers ---
    ret = mpu6050_configure();
    if (ret != ESP_OK) return ret;

    // --- Shared data mutex ---
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed — out of heap?");
        return ESP_ERR_NO_MEM;
    }

    memset(&s_shared_data, 0, sizeof(s_shared_data));
    s_shared_data.healthy    = false;
    s_shared_data.calibrated = false;

    // --- Boot calibration (blocking ~3s) ---
    mpu6050_calibrate();

    // --- Start IMU task on Core 1 ---
    BaseType_t rc = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        NULL,
        IMU_TASK_CORE
    );
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "IMU task creation failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "IMU running — 100Hz on Core %d, priority %d",
             IMU_TASK_CORE, IMU_TASK_PRIORITY);
    return ESP_OK;
}

void imu_get_data(imu_data_t *out)
{
    if (!out) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(out, &s_shared_data, sizeof(imu_data_t));
        xSemaphoreGive(s_mutex);
    }
}

bool imu_is_healthy(void)
{
    int64_t now_us = esp_timer_get_time();
    return (now_us - s_last_update_us) < ((int64_t)IMU_HEALTH_TIMEOUT_MS * 1000LL);
}

void imu_reset_yaw(void)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_shared_data.yaw = 0.0f;
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "Yaw reset to 0° (heading reference established)");
}