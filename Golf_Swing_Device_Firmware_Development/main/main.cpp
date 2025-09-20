#include <stdio.h>

#include "push_button.hpp"


static PushButton* button = nullptr;

extern "C" void app_main(void)
{
     static blink blinker;   
    static MPU6050 mpu;


    blinker.blink_init();
    mpu.setup_master();
    
    printf("%d\n", GPIO_NUM_15);
    gpio_install_isr_service(0);
    // PushButton button(GPIO_NUM_15, &blinker, &mpu); // Initialize button on GPIO 0
    // Create PushButton dynamically
    button = new PushButton(GPIO_NUM_15, &blinker, &mpu);

   
}