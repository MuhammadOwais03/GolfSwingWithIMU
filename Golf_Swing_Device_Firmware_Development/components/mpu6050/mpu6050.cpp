

#include "mpu6050.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <vector>

static const char* TAG = "MPU6050";

void MPU6050::setup_master() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        }
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config error: %d", ret);
        return;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install error: %d", ret);
        return;
    }

    // Reset MPU6050
    uint8_t data[] = {0x6B, 0x80};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 reset error: %d", ret);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up MPU6050
    data[0] = 0x6B;
    data[1] = 0x00;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake-up error: %d", ret);
        return;
    }

    // Configure DLPF (44 Hz bandwidth, 8 kHz internal sample rate)
    data[0] = 0x1A;
    data[1] = 0x03;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DLPF config error: %d", ret);
        return;
    }

    // Configure accelerometer (±4g)
    data[0] = 0x1C;
    data[1] = 0x08;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accel config error: %d", ret);
        return;
    }

    // Configure gyroscope (±250°/s)
    data[0] = 0x1B;
    data[1] = 0x00;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Gyro config error: %d", ret);
        return;
    }

    // Set sample rate (100 Hz = 8000 / (1 + 79))
    data[0] = 0x19;
    data[1] = 79;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sample rate config error: %d", ret);
        return;
    }

    // Verify WHO_AM_I
    uint8_t who_am_i;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, 
        (uint8_t[]){0x75}, 1, &who_am_i, 1, pdMS_TO_TICKS(100));
    if (ret == ESP_OK && who_am_i == 0x68) {
        ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02x, initialized successfully", who_am_i);
    } else {
        ESP_LOGE(TAG, "MPU6050 WHO_AM_I error: 0x%02x, ret=%d", who_am_i, ret);
    }
}

void MPU6050::readRawData() {
    uint8_t data[14];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, 
        (uint8_t[]){0x3B}, 1, data, 14, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error: %d", ret);
        return;
    }

    int16_t accelX = (data[0] << 8) | data[1];
    int16_t accelY = (data[2] << 8) | data[3];
    int16_t accelZ = (data[4] << 8) | data[5];
    int16_t tempRaw = (data[6] << 8) | data[7];
    int16_t gyroX = (data[8] << 8) | data[9];
    int16_t gyroY = (data[10] << 8) | data[11];
    int16_t gyroZ = (data[12] << 8) | data[13];

    ESP_LOGI(TAG, "Raw Accel: X=%d, Y=%d, Z=%d", accelX, accelY, accelZ);
    ESP_LOGI(TAG, "Raw Gyro: X=%d, Y=%d, Z=%d", gyroX, gyroY, gyroZ);
    ESP_LOGI(TAG, "Raw Temp: %d", tempRaw);

    float accel_sensitivity = 8192.0f; // LSB/g at ±4g
    float gyro_sensitivity = 131.0f;   // LSB/(°/s) at ±250°/s
    float accel_x_ms2 = (accelX / accel_sensitivity) * 9.81f;
    float accel_y_ms2 = (accelY / accel_sensitivity) * 9.81f;
    float accel_z_ms2 = (accelZ / accel_sensitivity) * 9.81f;
    float gyro_x_rads = (gyroX / gyro_sensitivity) * (M_PI / 180.0f);
    float gyro_y_rads = (gyroY / gyro_sensitivity) * (M_PI / 180.0f);
    float gyro_z_rads = (gyroZ / gyro_sensitivity) * (M_PI / 180.0f);
    float temperature = (tempRaw / 340.0f) + 36.53f;

    ESP_LOGI(TAG, "Accel in m/s^2: X=%.6f, Y=%.6f, Z=%.6f", accel_x_ms2, accel_y_ms2, accel_z_ms2);
    ESP_LOGI(TAG, "Gyro in rad/s: X=%.6f, Y=%.6f, Z=%.6f", gyro_x_rads, gyro_y_rads, gyro_z_rads);
    ESP_LOGI(TAG, "Temp in degree Centigrade: %.2f C", temperature);

    std::vector<float> reading = {accel_x_ms2, accel_y_ms2, accel_z_ms2, 
                                   gyro_x_rads, gyro_y_rads, gyro_z_rads};

    this->readings.push_back(reading);

    // if (flash != nullptr) {
    //     flash->write_file(readings);
    // } else {
    //     ESP_LOGE(TAG, "Flash pointer is null");
    // }

    // flash->read_file();
}