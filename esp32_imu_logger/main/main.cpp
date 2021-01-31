#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstring>
using namespace std;

#include <sys/unistd.h>
#include <sys/stat.h>
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#include "main.hpp"

// Main
extern "C" void app_main()
{
    ESP_LOGI(TAG, "$ MPU Driver Example: Advanced\n");
    mount_sd_card();
    ESP_LOGI(TAG, "$ SD card initialized\n");
    // Initialize bus through either the Library API or esp-idf API
    spi.begin(MOSI, MISO, SCLK);

    // Create a queue to store sensor readings 
    uint8_t data_frame[kFIFOSize];
    data_queue = xQueueCreate(10, sizeof(data_frame));
    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 2048, nullptr, 6, &mpu_task_handle);
    // Create a task to print angles
    const char * filename = get_filename();
    xTaskCreate(printTask, "printTask", 2 * 2048, (void*) filename, 0, &print_task_handle);
}

/* Tasks */

static MPU_t MPU;

static void mpuTask(void*)
{   
    //Initialize non-SPI GPIOs
    gpio_set_direction((gpio_num_t)CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CS, 1);

    gpio_set_direction((gpio_num_t)LOG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LOG_PIN, 1);

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
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());

    // Self-Test
    mpud::selftest_t retSelfTest;
    while (esp_err_t err = MPU.selfTest(&retSelfTest)) {
        ESP_LOGE(TAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
            // TODO: Check why this happens and error-handle!!! 
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

static void printTask(void * fn)
{   
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    // TODO: Check options to assign file to random sector to increase longevity of sd card 
    const char* filename = (const char*) fn; 
    ESP_LOGI(TAG, "Opening file: %s", filename);
    char filepath[100];
    strcpy(filepath, MOUNT_POINT); 
    strcat(filepath, "/");
    strcat(filepath, filename);

    FILE* f = fopen(filepath, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    //
    // File created 
    // 
    while (true) {
        // Write data to file 
        // TODO: 
        // Bytes 0...507:   sensor reads from fifo (currently only ~0...360 are used -> OPTIMIZTION POTENTIAL)
        // Bytes 508, 509:  timestamp (of the last MPU-interrupt?!) 
        // Bytes 510, 511:  fifo byte count 
        // ----> 4 of these packets per fwrite
        uint8_t FIFOpacket[kFIFOSize];
        mpud::raw_axes_t rawAccel;
        mpud::raw_axes_t rawGyro;
        mpud::float_axes_t accelG;   // accel axes in (g) gravity format
        mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
        if(xQueueReceive(data_queue, (void*) FIFOpacket, 10000/portTICK_PERIOD_MS)!=pdTRUE){
            ESP_LOGE(TAG, "Reading from queue faled. \n");
        }
        else{
            ESP_LOGI(TAG, "Successfully read data package from queue.");
            uint16_t fifocount; 
            fifocount = ((uint16_t)FIFOpacket[511] << 8) | FIFOpacket[510]; 
            // ESP_LOGI(TAG, "fifocount read: %d", fifocount);
            gpio_set_level((gpio_num_t)LOG_PIN, 0);
            fwrite (FIFOpacket , sizeof(uint8_t), sizeof(FIFOpacket), f);
            for(uint16_t i = 0; i < fifocount;i+=kFIFOPacketSize){
                /*
                 ESP_LOGI(TAG, "Packet Nb.: %d", i);
                rawAccel.x = FIFOpacket[i] << 8 | FIFOpacket[i+1];
                rawAccel.y = FIFOpacket[i+2] << 8 | FIFOpacket[i+3];
                rawAccel.z = FIFOpacket[i+4] << 8 | FIFOpacket[i+5];
                rawGyro.x  = FIFOpacket[i+6] << 8 | FIFOpacket[i+7];
                rawGyro.y  = FIFOpacket[i+8] << 8 | FIFOpacket[i+9];
                rawGyro.z  = FIFOpacket[i+10] << 8 | FIFOpacket[i+11];
                accelG = mpud::accelGravity(rawAccel, mpud::ACCEL_FS_4G);
                gyroDPS = mpud::gyroDegPerSec(rawGyro, mpud::GYRO_FS_500DPS);
                ESP_LOGI(TAG, "\t Acc.x: %+6.2f \t Acc.y: %+6.2f \t Acc.z: %+6.2f \t\t Gyr.x: %+7.2f \t Gyr.y: %+7.2f \t Gyr.z: %+7.2f", accelG.x, accelG.y, accelG.z, gyroDPS.x, gyroDPS.y, gyroDPS.z);
                */
            }
            gpio_set_level((gpio_num_t)LOG_PIN, 1);
        }
        if(false){
            fclose(f);
            ESP_LOGI(TAG, "File closed. Deleting PrintTask...");
            vTaskDelete(print_task_handle);
        }
            
    }
}

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

static void mount_sd_card(void)
{
    //
    // Create filesystem and mount 
    // 
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_CMD, GPIO_PULLUP_ONLY);   // PIN_NUM_SD_CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_D0, GPIO_PULLUP_ONLY);    // PIN_NUM_SD_D0, needed in 4- and 1-line modes
    gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_D1, GPIO_PULLUP_ONLY);    // PIN_NUM_SD_D1, needed in 4-line mode only ****
    gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_D2, GPIO_PULLUP_ONLY);   // PIN_NUM_SD_D2, needed in 4-line mode only ****
    gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_D3, GPIO_PULLUP_ONLY);   // PIN_NUM_SD_D3, needed in 4- and 1-line modes

    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}
static const char* get_filename()
{
    return "tmp003.imu";
}