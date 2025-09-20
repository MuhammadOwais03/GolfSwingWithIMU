#pragma once
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <stdio.h>
#include "esp_attr.h"
#include "mpu6050.hpp"
#include "blink.hpp"

class PushButton {
public:
    explicit PushButton(gpio_num_t pin, blink *blinker = nullptr, MPU6050 *mpu = nullptr);

private:
    gpio_num_t pin_;
    blink *blinker_;
    MPU6050 *mpu_;

    static void IRAM_ATTR button_isr_handler(void *arg);  // static ISR
    static void button_task(void *pvParameters);
};
