#include "push_button.hpp"
#include "esp_rom_sys.h"   // for ets_printf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern "C" int ets_printf(const char *fmt, ...);

// Queue to notify button press
static QueueHandle_t button_evt_queue = nullptr;

PushButton::PushButton(gpio_num_t pin, blink* blinker,  MPU6050 *mpu, FlashFile *flash, Vibration *vib) : pin_(pin), blinker_(blinker), mpu_(mpu), flash_(flash), vib_(vib)
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
    const TickType_t auto_off_delay = pdMS_TO_TICKS(15000); // 15 seconds

    for (;;) {
        if (xQueueReceive(button_evt_queue, &pin, pdMS_TO_TICKS(10))) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_press) > debounce_delay) {
                button->blinker_->blink_toggle();
                if (button->blinker_->is_on()) {
                    blinker_on_time = now; // Record when blinker turned on
                }
                last_press = now;
            }
        }

        // Check if blinker is on and auto-off time has elapsed
        if (button->blinker_->is_on()) {
            TickType_t now = xTaskGetTickCount();
            if ((now - blinker_on_time) > auto_off_delay) {
                button->blinker_->blink_toggle(); // Turn off blinker
                blinker_on_time = 0; // Reset on time
            }
            button->flash_->clear_file();
            button->mpu_->readRawData();
            button->vib_->read_vibration_data();
            vTaskDelay(pdMS_TO_TICKS(1000)); // Read every 1s
        } else {
            button->flash_->write_file(button->mpu_->readings, button->vib_->vib_readings);
            button->mpu_->readings = {};
            button->flash_->read_file();
            vTaskDelay(pdMS_TO_TICKS(10)); // Short delay when LED is OFF
        }
    }
}