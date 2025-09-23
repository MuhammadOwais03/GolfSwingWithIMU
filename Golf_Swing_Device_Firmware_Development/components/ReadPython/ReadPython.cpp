#include <stdio.h>
#include "ReadPython.hpp"
#include <string>
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"


#define BUF_SIZE 1024


#define UART_PORT UART_NUM_0
#define TX_PIN    GPIO_NUM_1
#define RX_PIN    GPIO_NUM_3            // GPIO3 (default for UART0 RX)


void ReadPython::uart_init()
{
     const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,        // Default threshold
            .source_clk = UART_SCLK_DEFAULT,
            .flags = 0,                        // Default: no special flags
        };


    // Configure UART parameters
    uart_param_config(UART_PORT, &uart_config);

    // Set UART pins
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void ReadPython::read_python_file()
{

    uint8_t data[BUF_SIZE];

    int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';
            std::string cmd = std::string((char *)data);
            
            // Trim \r\n
            cmd.erase(cmd.find_last_not_of("\r\n") + 1);

            // ESP_LOGI("TAG", "Received command: %s", cmd.c_str());

            if (cmd == "READ_FILE") {
                FILE *f = fopen((this->flash_->get_BASENAME() + this->flash_->get_FILENAME()).c_str(), "r");
                if (!f) {
                    uart_write_bytes(UART_PORT, "ERROR: Cannot open file\n", 24);
                } else {
                    char buf[128];
                    while (fgets(buf, sizeof(buf), f)) {
                        uart_write_bytes(UART_PORT, buf, strlen(buf));
                    }
                    fclose(f);
                    uart_write_bytes(UART_PORT, "EOF\n", 4);
                }
            }
            else if (cmd == "CLEAR_FILE") {
                this->flash_->clear_file();
                uart_write_bytes(UART_PORT, "OK: File cleared\n", 17);
            }
            else if (cmd == "SIZE") {
                this->flash_->size_of_flash();
                char msg[64];
                snprintf(msg, sizeof(msg), "SIZE: total=%zu used=%zu\n", this->flash_->get_TOTAL(), this->flash_->get_USED());
                uart_write_bytes(UART_PORT, msg, strlen(msg));
            }
            else {
                uart_write_bytes(UART_PORT, "ERROR: Unknown command\n", 23);
            }
        }
}
