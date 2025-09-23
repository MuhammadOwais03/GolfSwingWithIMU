#pragma once
#include "flash.hpp"
#include "driver/gpio.h"    


class ReadPython {
    public:
        ReadPython(FlashFile *flash = nullptr) : flash_(flash) {};
        void uart_init();
        void read_python_file();
        FlashFile *flash_;

};