
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
#include "MadgwickAHRS.hpp"


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
// static void printTask(void*);
// static void get_orientationTask(void*);

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
    float dt_s = 1.0/kSampleRate;  // delta t in seconds (TODO: make more exact with reading the time between interrupts)
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

        // Format and print
        // ESP_LOGI(TAG, "\t Acc.x: %+6.2f \t Acc.y: %+6.2f \t Acc.z: %+6.2f \t\t Gyr.x: %+7.2f \t Gyr.y: %+7.2f \t Gyr.z: %+7.2f", accelG.x, accelG.y, accelG.z, gyroDPS.x, gyroDPS.y, gyroDPS.z);

        ESP_LOGI(TAG, "\t q0: %+6.2f \t q1: %+6.2f \t q2: %+6.2f \t q3: %+7.2f ", q0, q1, q2, q3);
        MadgwickAHRSupdateIMU(gyroDPS.x, gyroDPS.y, gyroDPS.z, accelG.x, accelG.y, accelG.z, dt_s);
        


    }
    vTaskDelete(nullptr);
}

/*
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
*/

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}


/* placeholder tasks
static void flightTask(void*)
{   
    GetOrientation();  // Get the current orientation (euler angles) of the quadcopter 
    GetControlls();  // Get the desired orientation of the quadcopter 
    PID();  // Controll the desired orientation 
    UpdateESC();  // Update the 
    
    // Helper functions: 
    GetState();  // Get the current state of the ESCs (i.e. current throttle for each ESC) 
    SingleReadIMU();  // Read Accel and Gyro Registers 
    

}
*/