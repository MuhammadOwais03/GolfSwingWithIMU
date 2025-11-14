#include "push_button.hpp"
#include "esp_rom_sys.h"   // for ets_printf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"  // For reset

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
    ESP_LOGI("PushButton", "Button task started");

    // Timing constants
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);
    const TickType_t auto_off_delay = pdMS_TO_TICKS(8000);  // 8 seconds
    const TickType_t record_interval = pdMS_TO_TICKS(25);   // Read every 25ms during recording
    const TickType_t idle_interval = pdMS_TO_TICKS(100);    // Idle delay

    // State vars
    TickType_t last_press = 0;
    TickType_t blinker_on_time = 0;
    bool write_done = true;
    bool last_button_state = false;

    // TWDT safety
    esp_task_wdt_add(NULL);

    for (;;) {
        // Poll button state (active-low)
        int level = gpio_get_level(GPIO_NUM_4);
        bool is_pressed = (level == 0);

        // Detect press edge with debounce
        if (is_pressed && !last_button_state) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_press) > debounce_delay) {
                // Toggle blinker (assume this turns ON if OFF)
                button->blinker_->blink_toggle();
                last_press = now;

                if (button->blinker_->is_on()) {
                    // AFTER PUSH: Start recording mode
                    ESP_LOGI("PushButton", "Recording started - Blinker ON");
                    blinker_on_time = now;  // Start auto-off timer
                    button->imu_->readings.clear();
                    button->vib_->vib_readings.clear();
                    button->imu_->eulerAngles_.clear();
                }
            }
        }
        last_button_state = is_pressed;

        TickType_t now = xTaskGetTickCount();

        if (button->blinker_->is_on()) {
            // RECORDING MODE (after push)
            write_done = false;  // Flag to write on next OFF

            // Check auto-off
            if ((now - blinker_on_time) > auto_off_delay) {
                button->blinker_->blink_toggle();  // Turn OFF
                ESP_LOGI("PushButton", "Auto-off - Blinker OFF");
                blinker_on_time = 0;
                // Fall through to OFF logic (write will happen below)
            } else {
                // Read sensors and store
                button->imu_->loop();

                const RawAccelVec& accel = button->imu_->getRawLowAccelerationIng();
                const LinearAccelVec& linearAcceleration = button->imu_->getLinearLowAccelerationInMeterPerSec();
                const RawAccelVec& high_accel = button->imu_->getRawHighAccelerationIng();
                const RawGyroVec& gyro = button->imu_->getRawGyroInMdps();
                const EulerAngles &pitch = button->imu_->getPitch();
                const EulerAngles &roll = button->imu_->getRoll();
                const EulerAngles &yaw = button->imu_->getYaw();

                // Note: Fix potential bug - high_accel[1] and [2] were using [0]
                button->imu_->readings.push_back({
                    static_cast<float>(linearAcceleration[0]),
                    static_cast<float>(linearAcceleration[1]),
                    static_cast<float>(linearAcceleration[2]),
                    static_cast<float>(high_accel[0]),
                    static_cast<float>(high_accel[1]),  // Fixed: was high_accel[0]
                    static_cast<float>(high_accel[2]),  // Fixed: was high_accel[0]
                    gyro[0],
                    gyro[1],
                    gyro[2]
                });

                ESP_LOGI("PushButton", "Pitch: %.2f | Roll: %.2f | Yaw: %.2f", pitch, roll, yaw);
                button->imu_->eulerAngles_.push_back({pitch, roll, yaw});

                button->vib_->read_vibration_data();

                // Yield during recording
                vTaskDelay(1);
            }

            // TWDT reset during active recording
            esp_task_wdt_reset();

        } else {
            // IDLE MODE (before push or after auto-off)
            if (!write_done) {
                // Write to flash (only once per session)
                ESP_LOGI("PushButton", "Writing data to flash");
                esp_task_wdt_reset();
                button->flash_->clear_file();
                esp_task_wdt_reset();
                button->flash_->write_file(button->imu_->readings, button->vib_->vib_readings, button->imu_->eulerAngles_);
                esp_task_wdt_reset();
                button->flash_->read_file();  // Verify?
                esp_task_wdt_reset();
                write_done = true;
            }

            // Idle yield
            vTaskDelay(idle_interval);
        }

        // Periodic TWDT reset (idle or post-record)
        esp_task_wdt_reset();
    }

    esp_task_wdt_delete(NULL);  // Unreachable
}

