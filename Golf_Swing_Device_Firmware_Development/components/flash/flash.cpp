#include <cstdio>
#include <stdio.h>
#include "flash.hpp"
#include "esp_spiffs.h"
#include "esp_log.h"

static const char *TAG = "FlashFile";

// Constructor implementation
FlashFile::FlashFile(std::string basePath, std::string fileName, size_t maxFiles, size_t total, size_t used)
    : BASEPATH(basePath), FILENAME(fileName), MAXFILES(maxFiles), TOTAL(total), USED(used) {
    ESP_LOGI(TAG, "FlashFile initialized: basePath=%s, fileName=%s, maxFiles=%zu, total=%zu, used=%zu",
             BASEPATH.c_str(), FILENAME.c_str(), MAXFILES, TOTAL, USED);
    flash_config();
}

void FlashFile::flash_config() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = BASEPATH.c_str(),  // mount point
        .partition_label = nullptr,     // default partition
        .max_files = 5,          // how many files can be open
        .format_if_mount_failed = true  // format if mount fails
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
}

void FlashFile::size_of_flash()
{
    esp_err_t result;
    result = esp_spiffs_info(NULL, &this->TOTAL, &this->USED); // get info about the storage
    if (result != ESP_OK) {
        ESP_LOGE("OWAIS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGI("OWAIS", "Partition size: total: %zu, used: %zu", this->TOTAL, this->USED);
        if (this->USED > this->TOTAL) {
            ESP_LOGW("OWAIS", "SPIFFS reports used > total, filesystem may be corrupted");
        }
    }
}

void FlashFile::read_file()
{
    FILE *fptr = fopen((BASEPATH + FILENAME).c_str(), "r");
    if (fptr == NULL) {
        ESP_LOGE("OWAIS", "Failed to open file for reading");
        return;
    }

    fseek(fptr, 0, SEEK_END);
    long filesize = ftell(fptr);
    rewind(fptr);

    std::string content(filesize, '\0');
    fread(&content[0], 1, filesize, fptr);
    fclose(fptr);

    ESP_LOGI("OWAIS", "Full file content:\n%s", content.c_str());
}



void FlashFile::write_file(const std::vector<std::vector<float>>& readings, const std::vector<float>& vib_readings) {
    // Open file for appending (creates if not exists)
    FILE *f = fopen((BASEPATH + FILENAME).c_str(), "a");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    // Check if file is empty → write header
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    if (file_size == 0) {
        fprintf(f, "ax,ay,az,gx,gy,gz,vib\n");
    }

    // Sanity check: expect exactly 6 values
    if (readings.size() == 0) {
        // ESP_LOGE(TAG, "Expected something in readings readings, got %d", (int)readings.size());
        fclose(f);
        return;
    }

    // // Write readings as CSV row
    // fprintf(f, "%f,%f,%f,%f,%f,%f\n",
    //         readings[0], readings[1], readings[2],
    //         readings[3], readings[4], readings[5]);

    // Ensure vib_readings size matches readings size
    if (vib_readings.size() != readings.size()) {
        ESP_LOGW(TAG, "vib_readings size (%zu) does not match readings size (%zu)", vib_readings.size(), readings.size());
    }

    for (size_t i = 0; i < readings.size(); ++i) {
        const std::vector<float>& reading = readings[i];
        float vib = (i < vib_readings.size()) ? vib_readings[i] : 0.0f;

        // Write each reading as a CSV row with vib_reading at the end
        if (reading.size() == 6) {
            fprintf(f, "%f,%f,%f,%f,%f,%f,%f\n",
                    reading[0], reading[1], reading[2],
                    reading[3], reading[4], reading[5],
                    vib);
        } else {
            ESP_LOGW(TAG, "Skipping row with unexpected size: %zu", reading.size());
        }
    }

    

    fclose(f);
    ESP_LOGI(TAG, "Data written to %s%s", BASEPATH.c_str(), FILENAME.c_str());
}

// // This version only saves readings (no vib), as requested previously.
// void FlashFile::write_file_new(const std::vector<std::vector<float>> &readings)
// {
//     FILE *f = fopen((BASEPATH + FILENAME).c_str(), "a");
//     if (f == nullptr) {
//         ESP_LOGE(TAG, "Failed to open file for writing");
//         return;
//     }

//     // Check if file is empty → write header
//     fseek(f, 0, SEEK_END);
//     long file_size = ftell(f);
//     if (file_size == 0) {
//         fprintf(f, "ax,ay,az,gx,gy,gz\n");
//     }

//     for (const auto& reading : readings) {
//         if (reading.size() == 6) {
//             fprintf(f, "%f,%f,%f,%f,%f,%f\n",
//                     reading[0], reading[1], reading[2],
//                     reading[3], reading[4], reading[5]);
//         } else {
//             ESP_LOGW(TAG, "Skipping row with unexpected size: %zu", reading.size());
//         }
//     }

//     fclose(f);
//     ESP_LOGI(TAG, "Data written to %s%s", BASEPATH.c_str(), FILENAME.c_str());
// }

void FlashFile::clear_file() {
    FILE *f = fopen((BASEPATH + FILENAME).c_str(), "w");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for clearing");
        return;
    }
    fclose(f);
    ESP_LOGI(TAG, "File %s%s cleared", BASEPATH.c_str(), FILENAME.c_str());
}