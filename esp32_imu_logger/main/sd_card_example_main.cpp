/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MPU.hpp"
#include "SPIbus.hpp"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

static const char *TAG = "example";

static constexpr int PIN_NUM_SD_CMD            = 15;
static constexpr int PIN_NUM_SD_D0             = 2;
static constexpr int PIN_NUM_SD_D1             = 4;
static constexpr int PIN_NUM_SD_D2             = 12;
static constexpr int PIN_NUM_SD_D3             = 13;


static constexpr int PIN_NUM_IMU_MOSI            = 23;
static constexpr int PIN_NUM_IMU_MISO            = 19;
static constexpr int PIN_NUM_IMU_SCLK            = 18;
static constexpr int PIN_NUM_IMU_CS              = 5;
static constexpr int PIN_NUM_IMU_INT             = 34;

static constexpr int LOG_PIN                     = 33;


#define MOUNT_POINT "/sdcard"


extern "C" void app_main(void)
{
    //Initialize GPIOs for debugging
    gpio_set_direction((gpio_num_t)LOG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LOG_PIN, 1);


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

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    // TODO: Check options to assign file to random sector to increase longevity of sd card 
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen(MOUNT_POINT"/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    uint8_t buffer512[512];

    for(uint16_t i=0; i>512; i++){
        buffer512[i] = 43;
    }

    uint8_t buffer1024[1024];

    for(uint16_t i=0; i>1024; i++){
        buffer1024[i] = 13;
    }

    for(uint8_t j=0; j<16; j++){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)LOG_PIN, 0);
        fwrite (buffer512 , sizeof(uint8_t), sizeof(buffer512), f);
        gpio_set_level((gpio_num_t)LOG_PIN, 1);
        vTaskDelay(23 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)LOG_PIN, 0);
        fwrite (buffer1024 , sizeof(uint8_t), sizeof(buffer1024), f);
        gpio_set_level((gpio_num_t)LOG_PIN, 1);
    }


    
    fclose(f);
    ESP_LOGI(TAG, "File written");

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(MOUNT_POINT"/foo.txt", &st) == 0) {
        // Delete it if it exists
        unlink(MOUNT_POINT"/foo.txt");
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file");
    if (rename(MOUNT_POINT"/hello.txt", MOUNT_POINT"/foo.txt") != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen(MOUNT_POINT"/foo.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");
#ifdef USE_SPI_MODE
    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
#endif
}
