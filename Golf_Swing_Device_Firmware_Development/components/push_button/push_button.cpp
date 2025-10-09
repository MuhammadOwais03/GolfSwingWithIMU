#include "push_button.hpp"
#include "esp_rom_sys.h"   // for ets_printf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern "C" int ets_printf(const char *fmt, ...);

// Queue to notify button press
static QueueHandle_t button_evt_queue = nullptr;

PushButton::PushButton(gpio_num_t pin, blink* blinker, FlashFile *flash, Vibration *vib, ImuManager *imu) : pin_(pin), blinker_(blinker), flash_(flash), vib_(vib), imu_(imu)
{

    ets_printf("Configuring button on GPIO %d\n", this->pin_);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;   
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin_);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

   

    // Create event queue if not exists
    if (button_evt_queue == nullptr) {
        button_evt_queue = xQueueCreate(10, sizeof(gpio_num_t));
    }

    // Attach ISR handler
    gpio_isr_handler_add(pin_, button_isr_handler, this);

    // Start task to handle button events
    xTaskCreate(button_task, "button_task", 4096, this, 10, NULL);
}

// ISR handler (keep minimal!)
void IRAM_ATTR PushButton::button_isr_handler(void *arg) 
{
    PushButton *button = static_cast<PushButton *>(arg);
    gpio_num_t pin = button->pin_;
    ets_printf("ISR fired on GPIO %d\n", pin);
    // ets_printf("ISR fired, arg=%p, GPIO=%d\n", arg, button->pin_);

    // Send event to queue
    if (button_evt_queue) {
        xQueueSendFromISR(button_evt_queue, &pin, NULL);
    }
}




void PushButton::button_task(void *pvParameters)
{
    auto *button = static_cast<PushButton *>(pvParameters);
    gpio_num_t pin;
    TickType_t last_press = 0;
    TickType_t blinker_on_time = 0;
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);
    const TickType_t auto_off_delay = pdMS_TO_TICKS(8000); // 15 seconds
    bool write_done = true;
    

    for (;;) {
        if (xQueueReceive(button_evt_queue, &pin, pdMS_TO_TICKS(10))) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_press) > debounce_delay) {
                button->blinker_->blink_toggle();
                if (button->blinker_->is_on()) {
                    blinker_on_time = now; // Record when blinker turned on
                    button->imu_->readings.clear(); // Clear IMU readings when blinker turns on
                    button->vib_->vib_readings.clear(); // Clear vibration readings when blinker turns on
                }
                last_press = now;
            }
        }

        // Check if blinker is on and auto-off time has elapsed
        if (button->blinker_->is_on()) {
            TickType_t now = xTaskGetTickCount();
            write_done = false;
            if ((now - blinker_on_time) > auto_off_delay) {
                button->blinker_->blink_toggle(); // Turn off blinker
                blinker_on_time = 0; // Reset on time
            }
            
            button->imu_->loop();

            const RawAccelVec& accel = button->imu_->getRawLowAccelerationIng();
            const RawGyroVec& gyro = button->imu_->getRawGyroInMdps();
            button->imu_->readings.push_back({// converting to m/s2
                                                static_cast<float>(accel[0] ),
                                                static_cast<float>(accel[1] ),
                                                static_cast<float>(accel[2] ),
                                                gyro[0],
                                                gyro[1],
                                                gyro[2]
                                            });
                                            
            button->vib_->read_vibration_data();
            // vTaskDelay(pdMS_TO_TICKS(25)); // Read every 1s
        } else {
            // button->flash_->write_file(button->mpu_->readings, button->vib_->vib_readings);
            if (!write_done) {
                button->flash_->clear_file();
                button->flash_->write_file(button->imu_->readings, button->vib_->vib_readings);
                button->flash_->read_file();
                write_done = true;
            }            
            // ESP_LOGI("PushButton", "Readings size: %d", button->imu_->readings.size());
            vTaskDelay(pdMS_TO_TICKS(100)); // Read every 1s
            
        }
    }
}