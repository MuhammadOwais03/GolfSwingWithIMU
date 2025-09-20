// #include <stdio.h>
// #include "mpu6050.hpp"
// #include "esp_mac.h"
// #include "freertos/FreeRTOS.h"
// #include <math.h> 
// #include <vector>
// #include "flash.hpp"


// #define I2C_MASTER_FREQ_HZ 100000     // I2C master clock frequency
// #define ESP_SLAVE_ADDR 0x68        // slave address for MPU6050
// #define I2C_MASTER_NUM I2C_NUM_0    // I2C port number
// static FlashFile* flash = nullptr; // Removed because FlashFile is undefined and not used in this file
// // 

// void MPU6050::setup_master() {
//     // Initialize I2C
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = GPIO_NUM_21,
//         .scl_io_num = GPIO_NUM_22,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master = {
//             .clk_speed = I2C_MASTER_FREQ_HZ
//         },
//         .clk_flags = 0
//     };
//     esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
//     if (ret != ESP_OK) {
//         printf("I2C config error: %d\n", ret);
//         return;
//     }
//     ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
//     if (ret != ESP_OK) {
//         printf("I2C driver install error: %d\n", ret);
//         return;
//     }

//     // Reset MPU6050
//     uint8_t data[] = {0x6B, 0x80}; // Set DEVICE_RESET bit
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
//     if (ret != ESP_OK) {
//         printf("MPU6050 reset error: %d\n", ret);
//         return;
//     }
//     vTaskDelay(pdMS_TO_TICKS(100));

//     // Wake up MPU6050 (clear sleep bit)
//     data[0] = 0x6B; // PWR_MGMT_1
//     data[1] = 0x00; // Clear sleep bit
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
//     if (ret != ESP_OK) {
//         printf("MPU6050 wake-up error: %d\n", ret);
//         return;
//     }

//     // Configure accelerometer (±2g)
//     data[0] = 0x1C; // ACCEL_CONFIG
//     data[1] = 0x00; // ±2g range
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
//     if (ret != ESP_OK) {
//         printf("Accel config error: %d\n", ret);
//         return;
//     }

//     // Configure gyroscope (±250°/s)
//     data[0] = 0x1B; // GYRO_CONFIG
//     data[1] = 0x00; // ±250°/s range
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
//     if (ret != ESP_OK) {
//         printf("Gyro config error: %d\n", ret);
//         return;
//     }

//     // Set sample rate (125Hz = 1kHz / (1 + 7))
//     data[0] = 0x19; // SMPLRT_DIV
//     data[1] = 0x07;
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, data, 2, pdMS_TO_TICKS(100));
//     if (ret != ESP_OK) {
//         printf("Sample rate config error: %d\n", ret);
//         return;
//     }

//     // Verify WHO_AM_I (should return 0x68)
//     uint8_t who_am_i;
//     ret = i2c_master_write_read_device(I2C_MASTER_NUM, ESP_SLAVE_ADDR, 
//         (uint8_t[]){0x75}, 1, &who_am_i, 1, pdMS_TO_TICKS(100));
//     if (ret == ESP_OK && who_am_i == 0x68) {
//         printf("MPU6050 WHO_AM_I: 0x%02x, initialized successfully\n", who_am_i);
//     } else {
//         printf("MPU6050 WHO_AM_I error: 0x%02x, ret=%d\n", who_am_i, ret);
//     }


//     flash = new FlashFile("/storage", "sensor.csv", 5, 0, 0);





// void MPU6050::readRawData() {
//     uint8_t data[14]; // 6 accel + 2 temp + 6 gyro

//     // Step 1: Write start register (ACCEL_XOUT_H = 0x3B)
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, 0x3B, true);  // starting register address ax is 2 bytes (16 bit) long and so on
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);

//     // Step 2: Read 14 bytes (Accel + Temp + Gyro)
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);
//     i2c_master_read(cmd, data, 13, I2C_MASTER_ACK); // save 14 bytes means 14x8 bits in data array
//     i2c_master_read_byte(cmd, data + 13, I2C_MASTER_NACK); // last byte NACK
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);


//    // Sensitivities
//     float accel_sensitivity = 16384.0f; // LSB/g at ±2g
//     float gyro_sensitivity  = 131.0f;   // LSB/(°/s) at ±250°/s

//     // Parse Accel (convert to m/s^2)
//     float accelX = ((int16_t)(data[0] << 8 | data[1])) / accel_sensitivity * 9.81f;
//     float accelY = ((int16_t)(data[2] << 8 | data[3])) / accel_sensitivity * 9.81f;
//     float accelZ = ((int16_t)(data[4] << 8 | data[5])) / accel_sensitivity * 9.81f;

//     // Parse Temp
//     int16_t tempRaw = (data[6] << 8) | data[7];
//     float temperature = (tempRaw / 340.0f) + 36.53f;

//     // Parse Gyro (convert to rad/s)
//     float gyroX = ((int16_t)(data[8]  << 8 | data[9]))  / gyro_sensitivity * (M_PI / 180.0f);
//     float gyroY = ((int16_t)(data[10] << 8 | data[11])) / gyro_sensitivity * (M_PI / 180.0f);
//     float gyroZ = ((int16_t)(data[12] << 8 | data[13])) / gyro_sensitivity * (M_PI / 180.0f);

//     std::vector <float> readings = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};

//     flash->write_file(readings);


//     // Print results
//     printf("Accel in m/s^2: X=%f, Y=%f, Z=%f\n", accelX, accelY, accelZ);
//     printf("Gyro in rad/s: X=%f, Y=%f, Z=%f\n", gyroX, gyroY, gyroZ);
//     printf("Temp in degree Centigrade: %.2f C\n", temperature);
   
// }


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

    // Initialize flash (line 96)
    flash = new FlashFile("/spiffs", "/sensor.csv", 5, 0, 0);

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

    std::vector<float> readings = {accel_x_ms2, accel_y_ms2, accel_z_ms2, 
                                   gyro_x_rads, gyro_y_rads, gyro_z_rads};

    if (flash != nullptr) {
        flash->write_file(readings);
    } else {
        ESP_LOGE(TAG, "Flash pointer is null");
    }

    flash->read_file();
}