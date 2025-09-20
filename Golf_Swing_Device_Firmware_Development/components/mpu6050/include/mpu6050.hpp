// #include "driver/i2c.h"
// #include "esp_log.h"


// class MPU6050 {
// public:
//     void setup_master();
//     void readRawData();
// };

#pragma once
#include "driver/i2c.h"
#include "flash.hpp"
#include <string>
#include <vector>

#define I2C_MASTER_FREQ_HZ 100000
#define ESP_SLAVE_ADDR 0x68
#define I2C_MASTER_NUM I2C_NUM_0

class MPU6050 {
public:
    MPU6050() : flash(nullptr) {}
    ~MPU6050() { delete flash; } // Clean up flash pointer
    void setup_master();
    void readRawData();
private:
    FlashFile* flash;
};