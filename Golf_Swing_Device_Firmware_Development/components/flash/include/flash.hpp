#pragma once
#include <string>
#include <vector>

class FlashFile {
public: 
    // Constructor with initializer list
    explicit FlashFile(std::string basePath = "/storage",
                       std::string fileName = "/sensor.csv",
                       size_t maxFiles = 5,
                       size_t total = 0,
                       size_t used = 0);

    void flash_config();
    void size_of_flash();
    void read_file();
    void write_file(const std::vector<float>& readings); // example with floats

private:
    std::string BASEPATH;
    std::string FILENAME;
    int MAXFILES;
    size_t TOTAL;
    size_t USED;
};
