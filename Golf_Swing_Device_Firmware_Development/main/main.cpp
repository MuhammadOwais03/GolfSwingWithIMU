#include <stdio.h>

#include "push_button.hpp"
#include "ReadPython.hpp"


static PushButton* button = nullptr;

extern "C" void app_main(void)
{
     static blink blinker;   
    static MPU6050 mpu;
    static FlashFile flash("/spiffs", "/sensor.csv", 5, 0, 0);
    static Vibration vib;
    static ReadPython reader(&flash);
    
    
    blinker.blink_init();
    mpu.setup_master();
    vib.vibrational_setup();
    reader.uart_init();
    
    printf("%d\n", GPIO_NUM_15);
    gpio_install_isr_service(0);
    
    button = new PushButton(GPIO_NUM_15, &blinker, &mpu, &flash, &vib); 

    while (1) {
        reader.read_python_file();
        vTaskDelay(pdMS_TO_TICKS(100));
    };


   
}