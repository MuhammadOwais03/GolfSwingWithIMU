#include <stdio.h>
#include "vibration.hpp"

#define VIBRATION_GPIO GPIO_NUM_34

void Vibration::vibrational_setup()
{

     gpio_config_t vib_conf = {
        .pin_bit_mask = (1ULL << VIBRATION_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&vib_conf);

}

void Vibration::read_vibration_data()
{
    int level = gpio_get_level(VIBRATION_GPIO);
    printf("Vibration GPIO level: %d\n", level);
    vib_readings.push_back(static_cast<float>(level));
}
