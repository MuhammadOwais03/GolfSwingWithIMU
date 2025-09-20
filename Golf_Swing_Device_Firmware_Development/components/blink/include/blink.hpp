#pragma once


#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"



class blink {
    private:
        bool state = false;
    public:
        void blink_init(void);
        void blink_toggle(void);
        bool is_on() const { return state; }
};