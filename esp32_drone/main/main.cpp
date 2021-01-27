// =========================================================================
// Released under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_real.cpp
 * A more 'elaborated' example. Shows how to:
 *  - Use either SPI or I2C in the same code
 *  - Use the MPU with interrupt signal
 *  - Read sensor data from FIFO
 *  - Perform Self-Test check
 *  - Calibrate sensor data output using offset registers
 *  - Calculate Tilt Angles
 * 
 * @note
 * To try this example: \n
 * Set the I2C/SPI pins in 'Bus configuration' and the interrupt pin in 'MPU configuration'.
 *
 * @todo Document the steps
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"


/* Bus configuration */

// This MACROS are defined in "skdconfig.h" and set through 'menuconfig'.
// Can use to check which protocol has been selected.

#include "SPIbus.hpp"
static SPI_t& spi                     = vspi;  // hspi or vspi
static constexpr int MOSI             = 23;
static constexpr int MISO             = 19;
static constexpr int SCLK             = 18;
static constexpr int CS               = 5;
static constexpr uint32_t CLOCK_SPEED_LOW = 1*1000*1000;  // 1MHz
static constexpr uint32_t CLOCK_SPEED_HIGH = 10*1000*1000;  // 10MHz


void mpu_spi_pre_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level((gpio_num_t)CS, 0);
}

void mpu_spi_post_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level((gpio_num_t)CS, 1);
}


/* MPU configuration */

static constexpr int kInterruptPin         = 34;  // GPIO_NUM
static constexpr uint16_t kSampleRate      = 50;  // Hz
static constexpr mpud::accel_fs_t kAccelFS = mpud::ACCEL_FS_4G;
static constexpr mpud::gyro_fs_t kGyroFS   = mpud::GYRO_FS_500DPS;
static constexpr mpud::dlpf_t kDLPF        = mpud::DLPF_98HZ;
static constexpr mpud::int_config_t kInterruptConfig{
    .level = mpud::INT_LVL_ACTIVE_HIGH,
    .drive = mpud::INT_DRV_PUSHPULL,
    .mode  = mpud::INT_MODE_PULSE50US,
    .clear = mpud::INT_CLEAR_STATUS_REG  //
};

// FIFO
constexpr uint16_t kFIFOPacketSize = 12;  // in Byte
constexpr uint16_t kFIFOSize = 512;  // in Byte

/*-*/

static const char* TAG = "MPU9250";

static void mpuISR(void*);
static void mpuTask(void*);
static void printTask(void*);

xQueueHandle data_queue; 

// Main
extern "C" void app_main()
{
    ESP_LOGI(TAG, "$ MPU Driver Example: Advanced\n");
    // Initialize bus through either the Library API or esp-idf API
    spi.begin(MOSI, MISO, SCLK);

    // Create a queue to store sensor readings 
    uint8_t data_frame[kFIFOSize];
    data_queue = xQueueCreate(10, sizeof(data_frame));
    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 2048, nullptr, 6, nullptr);
    // Create a task to print angles
    xTaskCreate(printTask, "printTask", 2 * 2048, nullptr, 0, nullptr);
}

/* Tasks */

static MPU_t MPU;

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
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

    // Configure
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    // Setup FIFO
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));

    // Setup Interrupt
    // set the number of data-ready interrupts of mpu to wait before actually doing something 
    // this way constantly checking the fifo count isn't necessary
    uint32_t n_interrupts_wait = (uint32_t)(kFIFOSize/kFIFOPacketSize*0.75);
    // uint32_t n_interrupts_wait = (uint32_t)(kFIFOSize/kFIFOPacketSize*0.75);
    printf("n_interrupts_wait: %d\n",n_interrupts_wait);

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

    // Ready to start reading
    ESP_ERROR_CHECK(MPU.resetFIFO());  // start clean
    
    uint32_t notificationValue = 0;  // n notifications. Increased from ISR; reset from this task
    // Reading Loop
    while (true) {
        // Wait for notification from mpuISR
        xTaskNotifyWait(0, 0, &notificationValue, portMAX_DELAY);
        if (notificationValue < (n_interrupts_wait-1)) { 
            // ESP_LOGW(TAG, "Task Notification value: %d", notificationValue);
            continue;
        }
        // ESP_LOGW(TAG, "Task Notification value: %d", notificationValue);
        // now it's getting serious. FIFO is almost as full as we want it to be to read from it
        // wait one more interrupt, then clear the task notification 
        notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Check FIFO count
        uint16_t fifocount = MPU.getFIFOCount();
        if (esp_err_t err = MPU.lastError()) {
            ESP_LOGE(TAG, "Error reading fifo count, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        // ESP_LOGI(TAG, "FIFO count: %d", fifocount);
        if ((fifocount % kFIFOPacketSize)) {
            ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
        }
        // Burst read data from FIFO
        uint8_t FIFOpacket[kFIFOSize];
        if (esp_err_t err = MPU.readFIFO_HS(fifocount, FIFOpacket)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        FIFOpacket[510]=fifocount & 0xff;
        FIFOpacket[511]=(fifocount >> 8);

        MPU.resetFIFO();
        // Format
        if(xQueueSendToBack(data_queue, (void*) FIFOpacket, 1000/portTICK_RATE_MS)!=pdTRUE){
            ESP_LOGE(TAG, "Writing to queue faled. \n");
        }
        else{
            // ESP_LOGI(TAG, "FIFOpacket written to queue.");
        }
    }
    vTaskDelete(nullptr);
}

static void printTask(void*)
{
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (true) {
        uint8_t FIFOpacket[kFIFOSize];
        mpud::raw_axes_t rawAccel;
        mpud::raw_axes_t rawGyro;
        mpud::float_axes_t accelG;   // accel axes in (g) gravity format
        mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
        if(xQueueReceive(data_queue, (void*) FIFOpacket, 10000/portTICK_PERIOD_MS)!=pdTRUE){
            ESP_LOGE(TAG, "Reading from queue faled. \n");
        }
        else{
            // ESP_LOGI(TAG, "Successfully read data package from queue.");
            uint16_t fifocount; 
            fifocount = ((uint16_t)FIFOpacket[511] << 8) | FIFOpacket[510]; 
            // ESP_LOGI(TAG, "fifocount read: %d", fifocount);
            for(uint16_t i = 0; i < fifocount;i+=kFIFOPacketSize){
                // ESP_LOGI(TAG, "Packet Nb.: %d", i);
                rawAccel.x = FIFOpacket[i] << 8 | FIFOpacket[i+1];
                rawAccel.y = FIFOpacket[i+2] << 8 | FIFOpacket[i+3];
                rawAccel.z = FIFOpacket[i+4] << 8 | FIFOpacket[i+5];
                rawGyro.x  = FIFOpacket[i+6] << 8 | FIFOpacket[i+7];
                rawGyro.y  = FIFOpacket[i+8] << 8 | FIFOpacket[i+9];
                rawGyro.z  = FIFOpacket[i+10] << 8 | FIFOpacket[i+11];
                accelG = mpud::accelGravity(rawAccel, mpud::ACCEL_FS_4G);
                gyroDPS = mpud::gyroDegPerSec(rawGyro, mpud::GYRO_FS_500DPS);
                ESP_LOGI(TAG, "\t Acc.x: %+6.2f \t Acc.y: %+6.2f \t Acc.z: %+6.2f \t\t Gyr.x: %+7.2f \t Gyr.y: %+7.2f \t Gyr.z: %+7.2f", accelG.x, accelG.y, accelG.z, gyroDPS.x, gyroDPS.y, gyroDPS.z);
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}