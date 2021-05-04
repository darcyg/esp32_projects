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

#include "ICM.hpp"
#include "icm/math.hpp"
#include "icm/types.hpp"


#include "SPIbus.hpp"
static SPI_t& spi                     = vspi;  // hspi or vspi
static constexpr int MOSI             = 23;
static constexpr int MISO             = 19;
static constexpr int SCLK             = 18;
static constexpr int CS               = 5;
static constexpr uint32_t SPI_CLOCK_SPEED = 8*1000*1000;  // 8MHz

static constexpr int LOG_PIN          = 33;
static constexpr int SYNC_PIN         = 17;


void icm_spi_pre_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level((gpio_num_t)CS, 0);
}

void icm_spi_post_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level((gpio_num_t)CS, 1);
}


/* ICM configuration */

static constexpr int kInterruptPin         = 17;  // GPIO_NUM
static constexpr uint16_t kSampleRate      = 50;  // Hz
static constexpr icm20601::accel_fs_t kAccelFS = icm20601::ACCEL_FS_4G;
static constexpr icm20601::gyro_fs_t kGyroFS   = icm20601::GYRO_FS_500DPS;
static constexpr icm20601::dlpf_t kDLPF        = icm20601::DLPF_98HZ;
static constexpr icm20601::int_config_t kInterruptConfig{
    .level = icm20601::INT_LVL_ACTIVE_HIGH,
    .drive = icm20601::INT_DRV_PUSHPULL,
    .mode  = icm20601::INT_MODE_PULSE50US,
    .clear = icm20601::INT_CLEAR_STATUS_REG  //
};

// FIFO
constexpr uint16_t kFIFOPacketSize = 12;  // in Byte
constexpr uint16_t kFIFOSize = 512;  // in Byte
constexpr uint8_t kFIFOReadsMax = (uint8_t)(kFIFOSize/kFIFOPacketSize); 

/* END ICM configuration */

static const char* TAG = "IMU Logger";

/* SD card configuation */

static constexpr int PIN_NUM_SD_CMD            = 15;
static constexpr int PIN_NUM_SD_D0             = 2;
static constexpr int PIN_NUM_SD_D1             = 4;

#define MOUNT_POINT "/sdcard"
static const char *pcTmpFileDefault = "tmp_000.imu";

xQueueHandle data_queue; 
xQueueHandle timestamp_queue; 
xQueueHandle icm_ticks_queue;

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

//#define HOST_IP_ADDR "192.168.178.28"
#define HOST_IP_ADDR "192.168.178.68"
#define PORT 3333
#define COMMAND_PORT 3334
#define TCP_PORT 65432

#define MULTICAST_TTL 1

#define MULTICAST_IPV4_ADDR "224.3.29.71"
#define UDP_PORT 10000

static const char *V4TAG = "mcast-ipv4";


static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_EVENT = BIT0;

#define DISABLE_BT_PROV 1

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

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

static EventGroupHandle_t status_event_group; 
// status_event_group bits: 
const uint8_t ucICMReady_BIT          = BIT0;
const uint8_t ucICMWriting_BIT        = BIT1;
const uint8_t ucFSReady_BIT           = BIT2;
const uint8_t ucRxCommandsReady_BIT   = BIT3;
const uint8_t ucFileSaved_BIT         = BIT4;

// ... continue as needed 

static EventGroupHandle_t command_event_group; 
// command_event_group bits: 
const uint8_t ICMReadyForMeasurement_BIT    = BIT0;
const uint8_t StartMeasurement_BIT          = BIT1;
const uint8_t IsMeasuring_BIT               = BIT2;
const uint8_t ucOTAUptateStart              = BIT3;


static ICM_t ICM; 

/* Functions */

static void prvMountSDCard(void);


/* Tasks */

TaskHandle_t icm_task_handle = NULL; 
TaskHandle_t udp_cmd_task_handle = NULL; 
TaskHandle_t xHandleWriteFileSD = NULL; 
TaskHandle_t xHandleTransmitFileTCP = NULL; 
TaskHandle_t xHandleMountSDCard = NULL; 
TaskHandle_t xHandleLowPrioTask = NULL; 

static void prvTransmitFileTCP(void*);

static void prvWriteFileSD(void*);

static void prvSimpleOtaExample(void*);

static void prvICMTask(void*);

static void prvLowPrioPrint(void*);


static void icmISR(void*);

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