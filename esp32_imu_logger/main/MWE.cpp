
#include "MWE.hpp"


/* Tasks */

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

static void mpuTask(void*)
{   
    //Initialize non-SPI GPIOs
    gpio_set_direction((gpio_num_t)CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CS, 1);
    // Let MPU know which bus and address to use

    MPU.setBus(spi);
    spi_device_handle_t mpu_spi_handle;
    spi_device_handle_t mpu_spi_handle_hs;
    spi.addDevice(0, CLOCK_SPEED_HIGH, -1, &mpu_spi_handle_hs, mpu_spi_pre_transfer_callback, mpu_spi_post_transfer_callback);
    spi.addDevice(0, CLOCK_SPEED_LOW, -1, &mpu_spi_handle, mpu_spi_pre_transfer_callback, mpu_spi_post_transfer_callback);
    MPU.setAddr(mpu_spi_handle);
    MPU.setAddr_hs(mpu_spi_handle_hs);
    // Verify connection
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());

    // Self-Test
    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPU.selfTest(&retSelfTest)) {
        ESP_LOGE(TAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
             (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));

    // Calibrate
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    // ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

    // Configure
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    constexpr gpio_config_t kGPIOConfig{
        .pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE  //
    };
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(MPU.setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));
    
    uint32_t notificationValue = 0;  // n notifications. Increased from ISR; reset from this task
    
    mpud::raw_axes_t accelRaw;  // accel raw axes 
    mpud::raw_axes_t gyroRaw;  // gyro raw axes 
    mpud::float_axes_t accelG;   // accel axes in (g) gravity format
    mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    mpud::float_axes_t gyroRADS;  // gyro axes in (DPS) º/s format
    
    // Reading Loop
    while (true) {
        // Wait for notification from mpuISR
        xTaskNotifyWait(0, 0, &notificationValue, portMAX_DELAY);
        if (notificationValue > 1) { 
            ESP_LOGE(TAG, "Something went too slowly. ISR notfication value: %d", notificationValue);
            continue;
        }
        notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // now it's getting serious. Read new sensor data from registers
        MPU.motion(&accelRaw, &gyroRaw);
        // Convert
        accelG = mpud::accelGravity(accelRaw, kAccelFS);
        gyroDPS = mpud::gyroDegPerSec(gyroRaw, kGyroFS);
        gyroRADS = mpud::gyroRadPerSec(gyroRaw, kGyroFS);

        ESP_LOGI(TAG, "\t Acc.x: %+6.2f \t Acc.y: %+6.2f \t Acc.z: %+6.2f \t\t Gyr.x: %+7.2f \t Gyr.y: %+7.2f \t Gyr.z: %+7.2f", accelG.x, accelG.y, accelG.z, gyroDPS.x, gyroDPS.y, gyroDPS.z);
    }
    vTaskDelete(nullptr);
}

// # # # # # # # # # # # # # # # # # # 
// Main
// # # # # # # # # # # # # # # # # # # 

extern "C" void app_main()
{   
    // Initialize bus through either the Library API or esp-idf API
    spi.begin(MOSI, MISO, SCLK);
    
    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 2048, NULL, 6, &mpu_task_handle);   
}
