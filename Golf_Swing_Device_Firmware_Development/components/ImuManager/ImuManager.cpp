#include "ImuManager.hpp"

const Character *ImuManager::TAG = "LSM6DSV320X_MANAGER";
FilterSettingMask ImuManager::filterSettingMask = {0};

ImuManager::ImuManager(gpio_num_t cs, gpio_num_t sck, gpio_num_t miso, gpio_num_t mosi, SpiMode mode)
    : cs_{cs}, sck_{sck}, miso_{miso}, mosi_{mosi}, mode_{mode},
      lowAccel_{3, 0.0f}, linearLowAccel_{3, 0.0f}, highAccel_{3, 0.0f}, dpsGyro_{3, 0.0f}, tempInC_{0.0f},
      quat_{4, 0.0f}, pitch_{0.0}, roll_{0.0}, yaw_{0.0},
      dataRawMotionOne{0}, dataRawMotionTwo{0}, dataRawMotionThree{0}, dataRawTemperature{0}, status{0},
      lowAccelStatus_{false}, highAccelStatus_{false}, gyroStatus_{false}, tempStatus_{false},
      devCtx{0}, spiHandle{0}, whoamI{0}, txBuffer{0}
{
    memset(dataRawMotionOne, 0, sizeof(dataRawMotionOne));
    memset(dataRawMotionTwo, 0, sizeof(dataRawMotionTwo));
    memset(dataRawMotionThree, 0, sizeof(dataRawMotionThree));
    memset(&status, 0, sizeof(status));
    memset(txBuffer, 0, sizeof(txBuffer));
    memset(&devCtx, 0, sizeof(devCtx));
    memset(&spiHandle, 0, sizeof(spiHandle));
}

void ImuManager::setup()
{
    if (!initSpi())
    {
        ESP_LOGE(TAG, "SPI initialization failed");
        return;
    }

    initImu320x(devCtx, &ImuManager::spiWrite, &ImuManager::spiRead, &ImuManager::spidelay, this);
    spidelay(BOOT_TIME);
    if (!whoAmI())
    {
        ESP_LOGE(TAG, "IMU identification failed");
        return;
    }

    if (!resetImu())
    {
        ESP_LOGE(TAG, "IMU reset failed");
        return;
    }

    setupImuDataRatesAndScales();
    setupImuFilter();
}

void ImuManager::loop()
{
    getImuDataStatus();
    updateLowAccelVec();
    updateHighAccelVec();
    updateGyroVec();
    updateQuaternionVec();
    updateLinearLowAccelVec();
    updatePitchVar();
    updateRollVar();
    updateYawVar();
    updateTemperatureVar();
}

Flag ImuManager::initSpi()
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi_,
        .miso_io_num = miso_,
        .sclk_io_num = sck_,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64};

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10 MHz
    devcfg.mode = mode_;
    devcfg.spics_io_num = cs_;
    devcfg.queue_size = 1;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spiHandle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI device addition failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SPI initialized");
    return true;
}

int32_t ImuManager::spiRead(void *handle, u8 reg, u8 *bufp, u16 len)
{
    auto *self = static_cast<ImuManager *>(handle);

    uint8_t out[1 + len];
    out[0] = reg | 0x80;
    memset(&out[1], 0x00, len);

    uint8_t in[1 + len];

    spi_transaction_t t = {};
    t.length = (1 + len) * 8;
    t.tx_buffer = out;
    t.rx_buffer = in;

    esp_err_t ret = spi_device_transmit(self->spiHandle, &t);
    if (ret != ESP_OK)
        return -1;

    memcpy(bufp, &in[1], len);
    return 0;
}

int32_t ImuManager::spiWrite(void *handle, u8 reg, const u8 *bufp, u16 len)
{
    auto *self = static_cast<ImuManager *>(handle);

    esp_err_t ret;
    spi_transaction_t t = {};

    uint8_t tx_data[1 + len];
    tx_data[0] = reg & 0x7F;
    memcpy(&tx_data[1], bufp, len);

    t.length = (1 + len) * 8;
    t.tx_buffer = tx_data;

    gpio_set_level(self->cs_, 0);
    ret = spi_device_transmit(self->spiHandle, &t);
    gpio_set_level(self->cs_, 1);

    return (ret == ESP_OK) ? 0 : -1;
}

void ImuManager::spidelay(u32 ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void ImuManager::initImu320x(stmdev_ctx_t &devCtx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void *handle)
{
    devCtx.write_reg = write;
    devCtx.read_reg = read;
    devCtx.mdelay = delay;
    devCtx.handle = handle;
}

Flag ImuManager::whoAmI()
{
    lsm6dsv320x_device_id_get(&devCtx, &whoamI);
    ESP_LOGI(TAG, "WHOAMI: 0x%02X", whoamI);

    if (whoamI != LSM6DSV320X_ID)
    {
        ESP_LOGE(TAG, "Wrong device ID, expected 0x%02X", LSM6DSV320X_ID);
        return false;
    }

    return true;
}

Flag ImuManager::resetImu()
{
    lsm6dsv320x_reset_t rst;
    lsm6dsv320x_reset_set(&devCtx, LSM6DSV320X_RESTORE_CTRL_REGS);
    do
    {
        lsm6dsv320x_reset_get(&devCtx, &rst);
    } while (rst != LSM6DSV320X_READY);

    return true;
}

void ImuManager::setupImuDataRatesAndScales()
{
    lsm6dsv320x_block_data_update_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_xl_data_rate_set(&devCtx, LSM6DSV320X_ODR_AT_480Hz);
    lsm6dsv320x_hg_xl_data_rate_set(&devCtx, LSM6DSV320X_HG_XL_ODR_AT_480Hz, 1);
    lsm6dsv320x_gy_data_rate_set(&devCtx, LSM6DSV320X_ODR_AT_480Hz);
    lsm6dsv320x_xl_full_scale_set(&devCtx, LSM6DSV320X_16g);
    lsm6dsv320x_hg_xl_full_scale_set(&devCtx, LSM6DSV320X_320g);
    lsm6dsv320x_gy_full_scale_set(&devCtx, LSM6DSV320X_2000dps);

    // SFLP initialization
    /* Configure FIFO */
    lsm6dsv320x_fifo_watermark_set(&devCtx, FIFO_WATERMARK);
    lsm6dsv320x_fifo_mode_set(&devCtx, LSM6DSV320X_STREAM_MODE);

    /* Enable SFLP Game Rotation only */
    lsm6dsv320x_sflp_data_rate_set(&devCtx, LSM6DSV320X_SFLP_120Hz);
    lsm6dsv320x_sflp_game_rotation_set(&devCtx, PROPERTY_ENABLE);

    // // lsm6dsv320x_fifo_sflp_raw_t sflp_cfg = { .game_rotation = 1 };
    // lsm6dsv320x_fifo_sflp_raw_t sflp_cfg = {
    //     .game_rotation = 1, // Enable quaternion FIFO
    //     .gravity = 0,       // FIFO for gravity vector (disable if unused)
    //     .gbias = 0          // FIFO for gbias output (disable if unused)
    // };
    // // lsm6dsv320x_fifo_sflp_batch_set(&devCtx, sflp_cfg);
    // lsm6dsv320x_fifo_sflp_batch_set(&devCtx, sflp_cfg);

    lsm6dsv320x_fifo_sflp_raw_t sflp_cfg = {1, 0, 0};
    lsm6dsv320x_fifo_sflp_batch_set(&devCtx, sflp_cfg);

     
    lsm6dsv320x_fifo_status_get(&devCtx, &fifo_status);
}

void ImuManager::setupImuFilter()
{
    filterSettingMask.drdy = PROPERTY_ENABLE;
    filterSettingMask.irq_xl = PROPERTY_ENABLE;
    filterSettingMask.irq_g = PROPERTY_ENABLE;
    lsm6dsv320x_filt_settling_mask_set(&devCtx, filterSettingMask);
    lsm6dsv320x_filt_gy_lp1_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_gy_lp1_bandwidth_set(&devCtx, LSM6DSV320X_GY_ULTRA_LIGHT);
    lsm6dsv320x_filt_xl_lp2_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_xl_lp2_bandwidth_set(&devCtx, LSM6DSV320X_XL_STRONG);
}

void ImuManager::getImuDataStatus()
{
    lsm6dsv320x_flag_data_ready_get(&devCtx, &status);
    lowAccelStatus_ = status.drdy_xl;
    highAccelStatus_ = status.drdy_hgxl;
    gyroStatus_ = status.drdy_gy;
    tempStatus_ = status.drdy_temp;
}

void ImuManager::updateLowAccelVec()
{
    if (isLowAccelReady())
    {
        memset(dataRawMotionOne, 0, sizeof(dataRawMotionOne));
        lsm6dsv320x_acceleration_raw_get(&devCtx, dataRawMotionOne);
        lowAccel_[0] = lsm6dsv320x_from_fs16_to_mg(dataRawMotionOne[0]) / 1000.0;
        lowAccel_[1] = lsm6dsv320x_from_fs16_to_mg(dataRawMotionOne[1]) / 1000.0;
        lowAccel_[2] = lsm6dsv320x_from_fs16_to_mg(dataRawMotionOne[2]) / 1000.0;
        // ESP_LOGI(TAG, "LOW Accel [g]: %.3f %.3f %.3f", lowAccel_[0], lowAccel_[1], lowAccel_[2]);
    }
}

void ImuManager::updateLinearLowAccelVec()
{
    // updateLowAccelVec();
    // updateQuaternionVec();
    Vector3 rawAccel;
    rawAccel.x = lowAccel_[0] * 9.81; //converting g to m/sec2
    rawAccel.y = lowAccel_[1] * 9.81; //converting g to m/sec2
    rawAccel.z = lowAccel_[2] * 9.81; //converting g to m/sec2

    Vector3 linAccel = computeLinearAccelerationQuat(rawAccel, quat_[0], quat_[1], quat_[2], quat_[3]);
    linearLowAccel_[0] = linAccel.x;
    linearLowAccel_[1] = linAccel.y;
    linearLowAccel_[2] = linAccel.z;

    // printf("Linear Acc [m/s²]: X=%.2f Y=%.2f Z=%.2f\n",
    //                        linearLowAccel_[0], linearLowAccel_[1], linearLowAccel_[2]);
    
}

// Gravity in sensor frame from quaternion (x, y, z, w)
Vector3 ImuManager::computeGravityFromQuat(float qx, float qy, float qz, float qw) {
    Vector3 g;

    // Rotate world gravity vector (0, 0, 9.81) by quaternion
    // g = q * (0,0,9.81) * q_conjugate
    // Simplified formula:
    g.x = 2.0f * (qx * qz - qw * qy) * 9.81f;
    g.y = 2.0f * (qy * qz + qw * qx) * 9.81f;
    g.z = (1.0f - 2.0f * (qx*qx + qy*qy)) * 9.81f;

    return g;
}

// Linear acceleration = rawAccel - gravity
Vector3 ImuManager::computeLinearAccelerationQuat(const Vector3& rawAccel, float qx, float qy, float qz, float qw) 
{
    Vector3 gravity = computeGravityFromQuat(qx, qy, qz, qw);

    Vector3 linAccel;
    linAccel.x = rawAccel.x - gravity.x;
    linAccel.y = rawAccel.y - gravity.y;
    linAccel.z = rawAccel.z - gravity.z;

    return linAccel;
}

void ImuManager::updateHighAccelVec()
{
    if (isHighAccelReady())
    {
        memset(dataRawMotionTwo, 0, sizeof(dataRawMotionTwo));
        lsm6dsv320x_hg_acceleration_raw_get(&devCtx, dataRawMotionTwo);
        highAccel_[0] = lsm6dsv320x_from_fs256_to_mg(dataRawMotionTwo[0]) / 1000.0;
        highAccel_[1] = lsm6dsv320x_from_fs256_to_mg(dataRawMotionTwo[1]) / 1000.0;
        highAccel_[2] = lsm6dsv320x_from_fs256_to_mg(dataRawMotionTwo[2]) / 1000.0;
        // ESP_LOGI(TAG, "HIGH Accel [g]: %.3f %.3f %.3f", highAccel_[0] , highAccel_[1] , highAccel_[2]);
    }
}

void ImuManager::updateGyroVec()
{
    if (isGyroReady())
    {
        memset(dataRawMotionThree, 0, sizeof(dataRawMotionThree));
        lsm6dsv320x_angular_rate_raw_get(&devCtx, dataRawMotionThree);
        dpsGyro_[0] = lsm6dsv320x_from_fs2000_to_mdps(dataRawMotionThree[0]) / 1000.0;
        dpsGyro_[1] = lsm6dsv320x_from_fs2000_to_mdps(dataRawMotionThree[1]) / 1000.0;
        dpsGyro_[2] = lsm6dsv320x_from_fs2000_to_mdps(dataRawMotionThree[2]) / 1000.0;
        // ESP_LOGI(TAG, "Gyro [mdps]: %.2f %.2f %.2f", this->dpsGyro_[0] , this->dpsGyro_[1] , this->dpsGyro_[2] );
    }
}

void ImuManager::updateTemperatureVar()
{
    if (isTemperatureReady())
    {
        memset(&dataRawTemperature, 0, sizeof(dataRawTemperature));
        lsm6dsv320x_temperature_raw_get(&devCtx, &dataRawTemperature);
        tempInC_ = lsm6dsv320x_from_lsb_to_celsius(dataRawTemperature);
        // ESP_LOGI(TAG, "Temp [°C]: %.2f", tempInC_);
    }
}

static float_t npy_half_to_float(uint16_t h)
{
    union
    {
        float_t ret;
        uint32_t retbits;
    } conv;
    conv.retbits = lsm6dsv320x_from_f16_to_f32(h);
    return conv.ret;
}

void ImuManager::updateQuaternionVec()
{
    int c = 5;
    static float q[4];
    while (c--) {
        lsm6dsv320x_fifo_out_raw_t f_data;
        lsm6dsv320x_fifo_out_raw_get(&devCtx, &f_data);

        // if (f_data.tag == LSM6DSV320X_SFLP_GAME_ROTATION_VECTOR_TAG)
        // {
        uint16_t *sflp = (uint16_t *)&f_data.data[2];
        

        if (f_data.data[0] == 0x00)
        {
            q[0] = npy_half_to_float(sflp[0]);
            q[1] = npy_half_to_float(sflp[1]);
        }
        else if (f_data.data[0] == 0x01)
        {
            q[2] = npy_half_to_float(sflp[0]);
            q[3] = npy_half_to_float(sflp[1]);
        }
     }
    quat_[0] = q[0];
    quat_[1] = q[1];
    quat_[2] = q[2];
    quat_[3] = q[3];

}

void ImuManager::updatePitchVar()
{
    // TODO: Implement pitch update logic
    const QuaternionVec &q = getQuaternions();
    
    // Calculate pitch (in radians)
    double sinp = 2.0 * (q[3] * q[1] - q[2] * q[0]);
    if (std::abs(sinp) >= 1)
        pitch_ = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch_ = std::asin(sinp);
    pitch_ = pitch_ * (180.0 / M_PI); // Convert to degrees
}

void ImuManager::updateRollVar()
{
    // TODO: Implement roll update logic
    const QuaternionVec &q = getQuaternions();
    // Calculate roll (in radians)
    double sinr_cosp = 2.0 * (q[3] * q[0] + q[1] * q[2]);
    double cosr_cosp = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);
    roll_ = roll_ * (180.0 / M_PI); // Convert to degrees
   
}

void ImuManager::updateYawVar()
{
    // TODO: Implement yaw update logic
    const QuaternionVec &q = getQuaternions();
    // Calculate yaw (in radians)
    double siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1]);
    double cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
    yaw_ = yaw_ * (180.0 / M_PI); // Convert to degrees
   
}