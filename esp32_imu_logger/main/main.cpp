
#include "main.hpp"


/* Tasks */

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

static void mpuTask(void*)
{   
    EventBits_t udp_cmd_bits;
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
    // ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));

    // Configure
    ESP_ERROR_CHECK(MPU.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(MPU.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(MPU.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(MPU.setDigitalLowPassFilter(kDLPF));

    // Setup FIFO
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    // # # # # # # # # # # 
    // Interrupt Setup
    // # # # # # # # # # # 
    // set the number of data-ready interrupts of mpu to wait before actually doing something 
    // this way constantly checking the fifo count isn't necessary
    uint32_t n_interrupts_wait = (uint32_t)(kFIFOSize/kFIFOPacketSize*0.75);
    ESP_LOGI(TAG, "n_interrupts_wait: %d\n",n_interrupts_wait);

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
    // # # # # # # # # # # 
    // END Interrupt Setup 
    // # # # # # # # # # # 

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
            ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", (kFIFOPacketSize*n_interrupts_wait), fifocount);
            // TODO: Check why this happens and error-handle!!! 
        }
        // Burst read data from FIFO
        uint8_t FIFOpacket[kFIFOSize];
        if (esp_err_t err = MPU.readFIFO_HS(fifocount, FIFOpacket)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        // write number of bytes read from the fifo to the data packet 
        // TODO: re-thingk data packet structure to also hold time stamp 
        FIFOpacket[510]=fifocount & 0xff;
        FIFOpacket[511]=(fifocount >> 8);

        MPU.resetFIFO();
        // Send data to queue only if measurement is started
        udp_cmd_bits = xEventGroupGetBits(command_event_group); 

        if(udp_cmd_bits&StartMeasurement_BIT){
            if(xQueueSendToBack(data_queue, (void*) FIFOpacket, 1000/portTICK_RATE_MS)!=pdTRUE){
                ESP_LOGE(TAG, "Writing to queue faled.");
            }
            else{
                xEventGroupSetBits(status_event_group, MPUWriting_BIT);
            }
        }
        else{
            xEventGroupClearBits(status_event_group, MPUWriting_BIT);
            // ESP_LOGW(TAG, "Measurement not started yet...");
        }            
    }
    vTaskDelete(nullptr);
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
        .format_if_mount_failed = true,
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
    // gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_D2, GPIO_PULLUP_ONLY);   // PIN_NUM_SD_D2, needed in 4-line mode only ****
    // gpio_set_pull_mode((gpio_num_t)PIN_NUM_SD_D3, GPIO_PULLUP_ONLY);   // PIN_NUM_SD_D3, needed in 4- and 1-line modes

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
    ESP_LOGI(TAG, "$ SD card initialized\n");
    sdmmc_card_print_info(stdout, card);
}

static const char* get_filename()
{
    return "tmp003.imu";
}

static void write_data_to_sd(void * fn)
{   
    EventBits_t status_bits;
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    // TODO: Check options to assign file to random sector to increase longevity of sd card 
    const char* filename = (const char*) fn; 
    ESP_LOGI(TAG, "Opening file: %s", filename);
    char filepath[100];
    strcpy(filepath, MOUNT_POINT); 
    strcat(filepath, "/");
    strcat(filepath, filename);

    FILE* active_file = fopen(filepath, "w");
    if (active_file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    //
    // File created 
    // 
    bool data_written = pdFALSE; 

    while (true) {
        // Try to read from queue only if data is being written or there are messages left in the queue
        status_bits = xEventGroupGetBits(command_event_group); 
        if((status_bits&MPUWriting_BIT) | uxQueueMessagesWaiting(data_queue)){
            data_written = pdTRUE;

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

            if(xQueueReceive(data_queue, (void*) FIFOpacket, 1000/portTICK_PERIOD_MS)!=pdTRUE){
                ESP_LOGE(TAG, "Reading from queue faled. \n");
            }
            else{
                ESP_LOGI(TAG, "Successfully read data package from queue.");
                uint16_t fifocount; 
                fifocount = ((uint16_t)FIFOpacket[511] << 8) | FIFOpacket[510]; 
                // ESP_LOGI(TAG, "fifocount read: %d", fifocount);
                gpio_set_level((gpio_num_t)LOG_PIN, 0);
                fwrite (FIFOpacket , sizeof(uint8_t), sizeof(FIFOpacket), active_file);
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
        }
        else{
            ESP_LOGW(TAG, "Not writing data. data_written: %i", data_written);
            // TODO: Read the remaining queue elements!!! 
            if(data_written){
                fclose(active_file);
                ESP_LOGI(TAG, "File closed. Deleting Task...");
                vTaskDelete(print_task_handle);
            }       
        }
        
    }
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
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

// # # # # # # # # # # 
// UDP STUFF
// # # # # # # # # # # 

static void udp_tx_sensor_data(void *pvParameters)
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

static void udp_rx_commands_task(void * fn)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    static const char *payload = "Ready!";

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(COMMAND_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
            ESP_LOGE(TAG, "Unable to set socket option: errno %d", errno);
        }


        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, COMMAND_PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            // ESP_LOGI(TAG, "Message sent");


            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                // ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                // break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                float f; 
                f = (float)atof(rx_buffer);
                if (strncmp(rx_buffer, "Start", 5) == 0) {
                    ESP_LOGI(TAG, "Received Start Command. Setting Event Bit.");
                    xEventGroupSetBits(command_event_group, StartMeasurement_BIT);
                }                
                else if (strncmp(rx_buffer, "Stop", 4) == 0)
                {
                    ESP_LOGI(TAG, "Received Stop Command. Clearing Event Bit.");
                    xEventGroupClearBits(command_event_group, StartMeasurement_BIT);
                }
                
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(udp_cmd_task_handle);
}

// # # # # # # # # # # 
// MULTICAST STUFF
// # # # # # # # # # # 

static void mcast_example_task(void *pvParameters)
{
    while (1) {
        
        gpio_set_direction((gpio_num_t)SYNC_PIN, GPIO_MODE_INPUT_OUTPUT);
        gpio_set_level((gpio_num_t)SYNC_PIN, 1);
        bool sync_status;

        int sock;

        sock = create_multicast_ipv4_socket();
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create IPv4 multicast socket");
        }

        if (sock < 0) {
            // Nothing to do!
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        // set destination multicast addresses for sending from these sockets
        struct sockaddr_in sdestv4 = {
            .sin_family = PF_INET,
            .sin_port = htons(UDP_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);
        // Loop waiting for UDP received, and sending UDP packets if we don't
        // see any.
        int err = 1;
        while (err > 0) {
            struct timeval tv = {
                .tv_sec = 2,
                .tv_usec = 0,
            };
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);

            int s = select(sock + 1, &rfds, NULL, NULL, &tv);
            if (s < 0) {
                ESP_LOGE(TAG, "Select failed: errno %d", errno);
                err = -1;
                break;
            }
            else if (s > 0) {
                if (FD_ISSET(sock, &rfds)) {
                    sync_status = (bool)gpio_get_level((gpio_num_t)SYNC_PIN);
                    gpio_set_level((gpio_num_t)SYNC_PIN, !sync_status);
                    // Incoming datagram received
                    char recvbuf[48];
                    char raddr_name[32] = { 0 };

                    struct sockaddr_in6 raddr; // Large enough for both IPv4 or IPv6
                    socklen_t socklen = sizeof(raddr);
                    int len = recvfrom(sock, recvbuf, sizeof(recvbuf)-1, 0,
                                       (struct sockaddr *)&raddr, &socklen);
                    if (len < 0) {
                        ESP_LOGE(TAG, "multicast recvfrom failed: errno %d", errno);
                        err = -1;
                        break;
                    }

                    // Get the sender's address as a string
                    if (raddr.sin6_family == PF_INET) {
                        inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr.s_addr,
                                    raddr_name, sizeof(raddr_name)-1);
                    }

                    ESP_LOGI(TAG, "received %d bytes from %s:", len, raddr_name);

                    recvbuf[len] = 0; // Null-terminate whatever we received and treat like a string...
                    
                    ESP_LOGI(TAG, "%s; %d", recvbuf, sync_status);

                }
            }
        /*    
            else { // s == 0
                // Timeout passed with no incoming data, so send something!
                static int send_count;
                const char sendfmt[] = "Multicast #%d sent by ESP32\n";
                char sendbuf[48];
                char addrbuf[32] = { 0 };
                int len = snprintf(sendbuf, sizeof(sendbuf), sendfmt, send_count++);
                if (len > sizeof(sendbuf)) {
                    ESP_LOGE(TAG, "Overflowed multicast sendfmt buffer!!");
                    send_count = 0;
                    err = -1;
                    break;
                }

                struct addrinfo hints = {
                    .ai_flags = AI_PASSIVE,
                    .ai_socktype = SOCK_DGRAM,
                };
                struct addrinfo *res;


                hints.ai_family = AF_INET; // For an IPv4 socket
                int err = getaddrinfo(CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR,
                                      NULL,
                                      &hints,
                                      &res);
                if (err < 0) {
                    ESP_LOGE(TAG, "getaddrinfo() failed for IPV4 destination address. error: %d", err);
                    break;
                }
                if (res == 0) {
                    ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
                    break;
                }
                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(UDP_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                ESP_LOGI(TAG, "Sending to IPV4 multicast address %s:%d...",  addrbuf, UDP_PORT);
                err = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                freeaddrinfo(res);
                if (err < 0) {
                    ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                    break;
                }
            }
        */
        }

        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}

static int socket_add_ipv4_multicast_group(int sock, bool assign_source_if)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface
    imreq.imr_interface.s_addr = IPADDR_ANY;
    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        ESP_LOGE(V4TAG, "Configured IPV4 multicast address '%s' is invalid.", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        return err;
    }
    ESP_LOGI(TAG, "Configured IPV4 Multicast address %s", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(V4TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
            return err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        return err;
    }
    return err;
}

static int create_multicast_ipv4_socket(void)
{
    struct sockaddr_in saddr = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(V4TAG, "Failed to create socket. Error %d", errno);
        return -1;
    }

    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to bind socket. Error %d", errno);
        close(sock);
        return -1;
    }


    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(V4TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
        close(sock);
        return -1;
    }

    // this is also a listening socket, so add it to the multicast
    // group for listening...
    err = socket_add_ipv4_multicast_group(sock, true);
    if (err < 0) {
        close(sock);
        return -1;
    }

    // All set, socket is configured for sending and receiving
    return sock;
}




// # # # # # # # # # # # # # # # # # # 
// Main
// # # # # # # # # # # # # # # # # # # 
extern "C" void app_main()
{   
    // first of all create event groups for inter-task communication 
    status_event_group = xEventGroupCreate(); 
    command_event_group = xEventGroupCreate(); 

    // set up file system 
    mount_sd_card();

    // provision WiFi and connect 
    provision_wifi();

    // Initialize bus through either the Library API or esp-idf API
    spi.begin(MOSI, MISO, SCLK);

    // Create a queue to store sensor readings 
    uint8_t data_frame[kFIFOSize];
    data_queue = xQueueCreate(10, sizeof(data_frame));
    
    // Create UDP Multicast task for syncing 
    xTaskCreate(&mcast_example_task, "mcast_task", 4096, NULL, 5, NULL);
    
    // Create UDP task to receive commands from host 
    xTaskCreate(udp_rx_commands_task, "udp_rx_cmd", 4096, NULL, 5, &udp_cmd_task_handle);

    // Create a task to setup mpu and read sensor data
    xTaskCreate(mpuTask, "mpuTask", 4 * 2048, nullptr, 6, &mpu_task_handle);   

    // TODO: restructure... 
    // Create task to write the file 
    const char * filename = get_filename();  // make filename accessible for several tasks 
    xTaskCreate(write_data_to_sd, "writeData", 2 * 2048, (void*) filename, 0, &print_task_handle);

}
