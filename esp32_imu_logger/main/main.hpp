#ifndef MAIN_HPP
#define MAIN_HPP

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstring>
using namespace std;

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "sdkconfig.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#include "SPIbus.hpp"
static SPI_t& spi                     = vspi;  // hspi or vspi
static constexpr int MOSI             = 23;
static constexpr int MISO             = 19;
static constexpr int SCLK             = 18;
static constexpr int CS               = 5;
static constexpr uint32_t CLOCK_SPEED_LOW = 1*1000*1000;  // 1MHz
static constexpr uint32_t CLOCK_SPEED_HIGH = 10*1000*1000;  // 10MHz

static constexpr int LOG_PIN          = 33;
static constexpr int SYNC_PIN         = 17;


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
static constexpr uint16_t kSampleRate      = 100;  // Hz
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
constexpr uint8_t kFIFOReadsMax = (uint8_t)(kFIFOSize/kFIFOPacketSize); 

/*-*/

static const char* TAG = "App";

/* SD card configuation */

static constexpr int PIN_NUM_SD_CMD            = 15;
static constexpr int PIN_NUM_SD_D0             = 2;
static constexpr int PIN_NUM_SD_D1             = 4;
static constexpr int PIN_NUM_SD_D2             = 12;
static constexpr int PIN_NUM_SD_D3             = 13;

#define MOUNT_POINT "/sdcard"
static const char *pcTmpFileDefault = "tmp_000.imu";

xQueueHandle data_queue; 
xQueueHandle timestamp_queue; 
xQueueHandle mpu_ticks_queue;

struct DataFrame 
{
    uint8_t     SensorReads[kFIFOSize];
    uint64_t    Timestamps[kFIFOReadsMax];
    uint8_t     n_samples;
};

struct DataSample6Axis 
{
    uint8_t     SensorReads[kFIFOPacketSize];
    uint64_t    Timestamp;
};


// WIFI STUFF 
/* Signal Wi-Fi events on this event-group */

#define HOST_IP_ADDR "192.168.178.68"
#define PORT 3333
#define COMMAND_PORT 3334
#define TCP_PORT 65432

#define MULTICAST_TTL 1

#define MULTICAST_IPV4_ADDR "224.3.29.71"
#define UDP_PORT 10000

static const char *V4TAG = "mcast-ipv4";

const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t command_event_group; 
static EventGroupHandle_t status_event_group; 

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

// status_event_group bits: 
const uint8_t ucMPUReady_BIT          = BIT0;
const uint8_t ucMPUWriting_BIT        = BIT1;
const uint8_t ucFSReady_BIT           = BIT2;
const uint8_t ucRxCommandsReady_BIT   = BIT3;
const uint8_t ucFileSaved_BIT         = BIT4;
const uint8_t ucNOTUSED_BIT0          = BIT5;
const uint8_t ucNOTUSED_BIT1          = BIT6;
const uint8_t ucNOTUSED_BIT2          = BIT7;
// ... continue as needed 

// command_event_group bits: 
const uint8_t MPUReadyForMeasurement_BIT = BIT0;
const uint8_t StartMeasurement_BIT = BIT1;
const uint8_t IsMeasuring_BIT = BIT2;

static MPU_t MPU; 

/* Functions */

static void prvMountSDCard(void);


/* Tasks */

TaskHandle_t mpu_task_handle = NULL; 
TaskHandle_t udp_cmd_task_handle = NULL; 
TaskHandle_t xHandleWriteFileSD = NULL; 
TaskHandle_t xHandleTransmitFileTCP = NULL; 
TaskHandle_t xHandleMountSDCard = NULL; 

static void prvTransmitFileTCP(void*);

static void prvWriteFileSD(void*);

static void prvSimpleOtaExample(void*);


static void mpuISR(void*);

static void mpuTask(void*);

static void udp_rx_commands_task(void*);

static void udp_tx_sensor_data(void *pvParameters);

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

static void wifi_init_sta(void);

static void get_device_service_name(char *service_name, size_t max);

esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data);

static void provision_wifi();

static void mcast_example_task(void *pvParameters); 

static int socket_add_ipv4_multicast_group(int sock, bool assign_source_if);

static int create_multicast_ipv4_socket(void);

/* Tasks */

#endif