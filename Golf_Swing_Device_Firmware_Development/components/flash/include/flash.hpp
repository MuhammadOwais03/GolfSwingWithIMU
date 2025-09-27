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
    void clear_file();
    void write_file(const  std::vector<std::vector<float>>& readings, const std::vector<float>& vib_readings); 
    void write_file_new(const  std::vector<float>& accel_low, const  std::vector<float>& gyro, const std::vector<float>& vib_readings);
    std::string get_BASENAME() { return BASEPATH; };
    std::string get_FILENAME() { return FILENAME; };
    size_t get_TOTAL() { return TOTAL; };
    size_t get_USED() { return USED; };
 
private:
    std::string BASEPATH;
    std::string FILENAME;
    int MAXFILES;
    size_t TOTAL;
    size_t USED;
};
