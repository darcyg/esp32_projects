
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
// #include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "MadgwickAHRS.hpp"
#include "motor_control.hpp"


/* Bus configuration */

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
static constexpr uint16_t kSampleRate      = 500;  // Hz
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


// MOTOR OBJECT
MotorControl Motors; 

// WIFI STUFF 
/* Signal Wi-Fi events on this event-group */

#define HOST_IP_ADDR "192.168.178.68"
#define PORT 3333

const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

static void udp_send_sensor_data_task(void *pvParameters);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_init_sta(void);
static void get_device_service_name(char *service_name, size_t max);
esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data);
static void provision_wifi();

xQueueHandle data_queue; 

// Main
extern "C" void app_main()
{
    ESP_LOGI(TAG, "$ Drone Stuff going on!!!\n");

    // Initialize Motor ESCs 
    // Motors.setup();  // TODO: CAUSES TROUBLE WITH SPI!!!! 
    // Motors.calibrate(); 

    // provision WiFi and connect 
    provision_wifi();

    // Initialize bus through either the Library API or esp-idf API
    spi.begin(MOSI, MISO, SCLK);
    
    // Create a queue to store orientation readings 
    extern volatile float quaternion_frame[4];
    data_queue = xQueueCreate(5, sizeof(quaternion_frame));

    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 2048, nullptr, 6, nullptr);
    xTaskCreate(udp_send_sensor_data_task, "udp_sensor", 4096, NULL, 5, NULL);

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
    mpud::float_axes_t gyroRADS;  // gyro axes in (DPS) º/s format
    float dt_s = 1.0/kSampleRate;  // delta t in seconds (TODO: make more exact with reading the time between interrupts)
    
    float QuatPacket[4];
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
        // gyroDPS = mpud::gyroDegPerSec(gyroRaw, kGyroFS);
        gyroRADS = mpud::gyroRadPerSec(gyroRaw, kGyroFS);
        
        // TODO: CHECK AXIS ORIENTATION!!! 
        MadgwickAHRSupdateIMU(gyroRADS.x, -gyroRADS.y, -gyroRADS.z, accelG.x, -accelG.y, -accelG.z, dt_s);
        
        QuatPacket[0] = q0;
        QuatPacket[1] = q1;
        QuatPacket[2] = q2;
        QuatPacket[3] = q3;

        // send data to queue
        if(xQueueSendToBack(data_queue, (void*) QuatPacket, 1000/portTICK_RATE_MS)!=pdTRUE){
            ESP_LOGE(TAG, "Writing to queue faled. \n");
        }
    }
    vTaskDelete(nullptr);
}


static void udp_send_sensor_data_task(void *pvParameters)
{
    
    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        int addr_family = AF_INET;
        int ip_protocol = IPPROTO_IP;


        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
        uint8_t i = 0; 
        float QuatPackage[40];
        while (1) {
            float quaternion_frame[4];
            // Check sensor_data_queue and read package/s 
            if(xQueueReceive(data_queue, (void*) quaternion_frame, 10000/portTICK_PERIOD_MS)!=pdTRUE){
                ESP_LOGE(TAG, "Reading from queue faled. \n");
            }
            QuatPackage[(i*4)+0] = quaternion_frame[0];
            QuatPackage[(i*4)+1] = quaternion_frame[1];
            QuatPackage[(i*4)+2] = quaternion_frame[2];
            QuatPackage[(i*4)+3] = quaternion_frame[3];

            if (i==9){
                // Send package to host 
                int err = sendto(sock, QuatPackage, sizeof(QuatPackage), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                i = 0; 
            }
            else{
                i++;
            }
            
            
            /*
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }
            */
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
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

static void provision_wifi()
{
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    } 

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();

    /* Register our event handler for Wi-Fi, IP and Provisioning related events */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

     /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Configuration for the provisioning manager */
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };

    /* Initialize provisioning manager with the
     * configuration parameters set above */
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;
    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned) {
        ESP_LOGI(TAG, "Starting provisioning");

        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
         *     - device name when scheme is wifi_prov_scheme_ble
         */
        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));

        /* What is the security level that we want (0 or 1):
         *      - WIFI_PROV_SECURITY_0 is simply plain text communication.
         *      - WIFI_PROV_SECURITY_1 is secure communication which consists of secure handshake
         *          using X25519 key exchange and proof of possession (pop) and AES-CTR
         *          for encryption/decryption of messages.
         */
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

        /* Do we want a proof-of-possession (ignored if Security 0 is selected):
         *      - this should be a string with length > 0
         *      - NULL if not used
         */
        const char *pop = "abcd1234";

        /* What is the service key (could be NULL)
         * This translates to :
         *     - Wi-Fi password when scheme is wifi_prov_scheme_softap
         *     - simply ignored when scheme is wifi_prov_scheme_ble
         */
        const char *service_key = NULL;

        /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
         * set a custom 128 bit UUID which will be included in the BLE advertisement
         * and will correspond to the primary GATT service that provides provisioning
         * endpoints as GATT characteristics. Each GATT characteristic will be
         * formed using the primary service UUID as base, with different auto assigned
         * 12th and 13th bytes (assume counting starts from 0th byte). The client side
         * applications must identify the endpoints by reading the User Characteristic
         * Description descriptor (0x2901) for each characteristic, which contains the
         * endpoint name of the characteristic */
        uint8_t custom_service_uuid[] = {
            /* LSB <---------------------------------------
             * ---------------------------------------> MSB */
            0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
            0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
        };
        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        /* An optional endpoint that applications can create if they expect to
         * get some additional custom data during provisioning workflow.
         * The endpoint name can be anything of your choice.
         * This call must be made before starting the provisioning.
         */
        // wifi_prov_mgr_endpoint_create("custom-data");
        /* Start provisioning service */
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key));

        /* The handler for the optional endpoint created above.
         * This call must be made after starting the provisioning, and only if the endpoint
         * has already been created above.
         */
        // wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);

        /* Uncomment the following to wait for the provisioning to finish and then release
         * the resources of the manager. Since in this case de-initialization is triggered
         * by the default event loop handler, we don't need to call the following */
        // wifi_prov_mgr_wait();
        // wifi_prov_mgr_deinit();
    } else {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

        /* We don't need the manager as device is already provisioned,
         * so let's release it's resources */
        wifi_prov_mgr_deinit();

        /* Start Wi-Fi station */
        wifi_init_sta();
    }

    /* Wait for Wi-Fi connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);

}


/* Event handler for catching system events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG, "Received Wi-Fi credentials"
                         "\n\tSSID     : %s\n\tPassword : %s",
                         (const char *) wifi_sta_cfg->ssid,
                         (const char *) wifi_sta_cfg->password);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                         "\n\tPlease reset to factory and retry provisioning",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                         "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");
                break;
            case WIFI_PROV_END:
                /* De-initialize manager once provisioning is finished */
                wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
        esp_wifi_connect();
    }
}

static void wifi_init_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

