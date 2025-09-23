#pragma once

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <stdio.h>
#include <vector>


class Vibration {
public:
    Vibration() = default;
    void vibrational_setup();
    void read_vibration_data();
    std::vector<float> vib_readings;

};