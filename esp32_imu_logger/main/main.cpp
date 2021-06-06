
#include "main.hpp"

/* DEBUG PIN CONFIG */ 
gpio_config_t io_conf = {
    .pin_bit_mask = (uint64_t) 0x1 << PIN_NUM_DEBUG_PIN,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

static IRAM_ATTR void icmISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    int64_t sys_ticks = esp_timer_get_time();
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    xQueueSendToBackFromISR(icm_ticks_queue, &sys_ticks, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

static void vICMTask(void*)
{   
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());
    EventBits_t udp_cmd_bits;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_IMU_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_IMU_SPI_CS, 1);

    // Let ICM know which bus and address to use
    ICM.setBus(spi);
    spi_device_handle_t icm_spi_handle;
    spi.addDevice(0, SPI_CLOCK_SPEED, -1, &icm_spi_handle, icm_spi_pre_transfer_callback, icm_spi_post_transfer_callback);
    ICM.setAddr(icm_spi_handle);
    // Verify connection
    while (esp_err_t err = ICM.testConnection()) {
        wifi_log_e(TAG, "Failed to connect to the ICM, error=%#X", err);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    wifi_log_i(TAG, "ICM connection successful!", "");

    // Initialize
    ESP_ERROR_CHECK(ICM.initialize());
    
    // TODO: Consider SelfTest

    // Configure
    ESP_ERROR_CHECK(ICM.setAccelFullScale(kAccelFS));
    ESP_ERROR_CHECK(ICM.setGyroFullScale(kGyroFS));
    ESP_ERROR_CHECK(ICM.setSampleRate(kSampleRate));
    ESP_ERROR_CHECK(ICM.setDigitalLowPassFilter(kDLPF));

    // Setup FIFO
    ESP_ERROR_CHECK(ICM.setFIFOConfig(icm20601::FIFO_CFG_ACCEL | icm20601::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(ICM.setFIFOEnabled(true));

    // # # # # # # # # # # 
    // Interrupt Setup
    // # # # # # # # # # # 
    // set the number of data-ready interrupts of ICM to wait before actually doing something 
    // this way constantly checking the fifo count isn't necessary
    uint32_t n_interrupts_wait = (uint32_t)(kFIFOSize/kFIFOPacketSize*0.85);

    constexpr gpio_config_t kGPIOConfig{
        .pin_bit_mask = (uint64_t) 0x1 << PIN_NUM_IMU_INTERUPT,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE  //
    };
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(PIN_NUM_IMU_INTERUPT, icmISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(ICM.setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(ICM.setInterruptEnabled(icm20601::INT_EN_RAWDATA_READY));
    // # # # # # # # # # # 
    // END Interrupt Setup 
    // # # # # # # # # # # 

    // Ready to start reading
    ESP_ERROR_CHECK(ICM.resetFIFO());  // start clean
    xQueueReset(icm_ticks_queue);

    uint32_t notificationValue = 0;  // n notifications. Increased from ISR; reset from this task
    // Reading Loop
    while (true) {
        // Wait for notification from icmISR
        xTaskNotifyWait(0, 0, &notificationValue, portMAX_DELAY);
        if (notificationValue < (n_interrupts_wait-1)) { 
            continue;
        }
        // now it's getting serious. FIFO is almost as full as we want it to be to read from it
        // wait one more interrupt, then clear the task notification 
        notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Check FIFO count
        uint16_t fifocount = ICM.getFIFOCount();
        if (esp_err_t err = ICM.lastError()) {
            wifi_log_e(TAG, "Error reading fifo count, %#X", err);
            ICM.resetFIFO();
            continue;
        }
        if ((fifocount % kFIFOPacketSize)) {
            wifi_log_e(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", (kFIFOPacketSize*n_interrupts_wait), fifocount);
            // TODO: Check why this happens and error-handle!!! 
        }
        // Push data into data_frame 
        // Burst read data from FIFO
        struct DataFrame data_frame; 

        gpio_set_level(PIN_NUM_DEBUG_PIN, 0);
        if (esp_err_t err = ICM.readFIFO(fifocount, data_frame.SensorReads)) {
            wifi_log_e(TAG, "Error reading sensor data, %#X", err);
            ICM.resetFIFO();
            continue;
        }
        // write number of bytes read from the fifo to the data packet 
        gpio_set_level(PIN_NUM_DEBUG_PIN, 1);
        data_frame.n_samples = (uint8_t)(fifocount/kFIFOPacketSize); 
        gpio_set_level(PIN_NUM_DEBUG_PIN, 0);
        // uint64_t TICKpacket[(int)(kFIFOSize/kFIFOPacketSize)];
        uint8_t i_tick = 0; 
        // TODO: assert uxQueueMessagesWaiting(icm_ticks_queue) == data_frame.n_samples
        
        gpio_set_level(PIN_NUM_DEBUG_PIN, 1);
        while(uxQueueMessagesWaiting(icm_ticks_queue)){
            xQueueReceive(icm_ticks_queue, &data_frame.Timestamps[i_tick], 1000/portTICK_PERIOD_MS);
            i_tick ++;
        }
        gpio_set_level(PIN_NUM_DEBUG_PIN, 0);

        ICM.resetFIFO();
        // Send data to queue only if measurement is started
        udp_cmd_bits = xEventGroupGetBits(command_event_group); 

        if(udp_cmd_bits&StartMeasurement_BIT){
        // if(true){
            if(xQueueSendToBack(data_queue, (void*) &data_frame, 1000/portTICK_RATE_MS)!=pdTRUE){
                wifi_log_e(TAG, "Writing to queue faled.", "");
            }
            else{
                xEventGroupSetBits(status_event_group, ucICMWriting_BIT);
            }
        }
        else{
            xEventGroupClearBits(status_event_group, ucICMWriting_BIT);
            // ESP_LOGW(TAG, "Measurement not started yet...");
        }
        gpio_set_level(PIN_NUM_DEBUG_PIN, 1);
    }
    vTaskDelete(nullptr);
}

esp_err_t vMountSDCard(void)
{
    char* TAG = "Mount SD Card";
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
    wifi_log_i(TAG, "Initializing SD card", "");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    wifi_log_i(TAG, "Using SDMMC peripheral", "");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(PIN_NUM_SDMMC_CMD, GPIO_PULLUP_ONLY);   // needed in 4- and 1- line modes
    gpio_set_pull_mode(PIN_NUM_SDMMC_D0, GPIO_PULLUP_ONLY);    // needed in 4- and 1-line modes

    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            wifi_log_e(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.", "");
        } else {
            wifi_log_e(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    // Card has been initialized, print its properties
    wifi_log_i(TAG, "$ SD card sucessfully initialized.", "");
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

static void vWriteFileSDTask(void*)
{   
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());
    EventBits_t xStatusBits;
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    // TODO: Check options to assign file to random sector to increase longevity of sd card 
    ESP_LOGI(TAG, "Opening file: %s", pcTmpFileDefault);
    char filepath[100];
    strcpy(filepath, MOUNT_POINT); 
    strcat(filepath, "/");
    strcat(filepath, pcTmpFileDefault);

    FILE* active_file = fopen(filepath, "wb");
    if (active_file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        vTaskDelete(nullptr);
    }
    //
    // File created 
    // 
    bool data_written = pdFALSE; 
    uint16_t n_packets = 0;
    while (true) {
        // Try to read from queue only if data is being written or there are messages left in the queue
        xStatusBits = xEventGroupGetBits(command_event_group); 
        if((xStatusBits & ucICMWriting_BIT) | uxQueueMessagesWaiting(data_queue)){
            if(!data_written){
                ESP_LOGI(TAG, "Writing data..."); 
            }
            data_written = pdTRUE;
            // Write data to file 
            struct DataFrame data_frame;
            if(xQueueReceive(data_queue, &data_frame, 1000/portTICK_PERIOD_MS)!=pdTRUE){
                ESP_LOGE(TAG, "Reading from queue faled. \n");
            }
            else{
                // ESP_LOGI(TAG, "Successfully read data package from queue. n_samples: %d", data_frame.n_samples);
                // gpio_set_level(PIN_NUM_DEBUG_PIN, 0);
                // fwrite (FIFOpacket , sizeof(uint8_t), sizeof(FIFOpacket), active_file);
                fwrite (&data_frame , sizeof(data_frame), 1, active_file);
                // gpio_set_level(PIN_NUM_DEBUG_PIN, 1);
                n_packets ++;
            }
        }
        else{
            // ESP_LOGW(TAG, "Now writing data. data_written: %i", data_written);
            if(data_written){
                fclose(active_file);
                ESP_LOGI(TAG, "File closed. Deleting Task...");
                ESP_LOGI(TAG, "Packets written: %d", n_packets);
                xEventGroupSetBits(status_event_group, ucFileSaved_BIT);
                // Create task to transmit file
                xTaskCreate(vTransmitFileTCPTask, "TransmitFile", 2 * 2048, NULL, 0, &xHandleTransmitFileTCP);
                vTaskDelete(xHandleWriteFileSDTask);
            }       
        }
        
    }
}


static void vStatusLEDTask(void*){

    gpio_set_direction(PIN_NUM_STATUS_LED,GPIO_MODE_OUTPUT);

    uint16_t duty_ms = pow(2, 12);
    uint16_t fade_ms = 2000;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .timer_num = LEDC_TIMER_0,            // timer index
        .freq_hz = 5000,                      // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
            .gpio_num   = PIN_NUM_STATUS_LED,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel    = LEDC_CHANNEL_0,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
    };
    ledc_channel_config(&ledc_channel);
    
    // Initialize fade service.
    ledc_fade_func_install(0);

    while(1) {
        // TODO: Check Status Event Group Bits 
        // TODO: 

        ledc_set_fade_with_time(ledc_channel.speed_mode, ledc_channel.channel, duty_ms, fade_ms);
        ledc_fade_start(ledc_channel.speed_mode,ledc_channel.channel, LEDC_FADE_WAIT_DONE);
        
        ledc_set_fade_with_time(ledc_channel.speed_mode,ledc_channel.channel, 0, fade_ms);
        ledc_fade_start(ledc_channel.speed_mode,ledc_channel.channel, LEDC_FADE_WAIT_DONE);

        /*
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty_ms);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        */
    }
}


static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void vBattStatusTask(void*){
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(adc_channel, atten);

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    uint8_t n_samples = 64;
    uint32_t percentage;
    uint32_t adc_reading;
    uint32_t voltage;
    uint8_t n = 5;
    uint32_t a[n] = {50};  

    //Continuously sample ADC1
    while (1) {        
        adc_reading = 0;
        //Multisampling
        for (uint8_t i=0; i<n_samples; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)adc_channel);
        }
        adc_reading /= n_samples;
        //Convert adc_reading to voltage in mV to percentage in %
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        percentage = bat_voltage_to_percentage(voltage);

        // average over n values
        for(int8_t i=n-2; i>=0; i--){
                a[i+1] = a[i]; //move all element to the right except last one
            }
        a[0] = percentage; 
        
        // round down to multiple of 5 
        uint16_t temp = 0;
        for(uint8_t i=0; i<n; i++){
                temp += a[i];
            }
        temp /= n;
        battery_percentage = temp / 5 * 5; 
        // wifi_log_i(TAG, "Battery Percentage: %d", battery_percentage);
        wifi_log_i(TAG, "Battery Voltage: %d", voltage);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static uint32_t bat_voltage_to_percentage(uint32_t voltage){
    uint32_t bat_100 = (uint32_t)(0.5*4200);  // in mV  (voltage divider)
    uint32_t bat_5 = (uint32_t)(0.5*3600);  // in mV  (voltage divider)
    uint32_t percentage;
    if (voltage < bat_5){
        return 0;
    }
    else if (voltage > bat_100){
        return 100; 
    }
    else{
        percentage = (voltage-bat_5)*100/(bat_100-bat_5); 
        return percentage;
    }
}

// # # # # # # # # # # 
// WIFI SETUP
// # # # # # # # # # # 

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
    } 
    else {
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
// TCP FILE TRANSMIT 
// # # # # # # # # # # 

static void vTransmitFileTCPTask(void*)
{
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());

    EventBits_t xStatusBits;
    
    char cFilepath[100];
    strcpy(cFilepath, MOUNT_POINT); 
    strcat(cFilepath, "/");
    strcat(cFilepath, pcTmpFileDefault);

    char rx_buffer[128];
    // char host_ip[] = host_ip;
    int addr_family = 0;
    int ip_protocol = 0;

    while(true)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(TCP_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
            ESP_LOGE(TAG, "Unable to set socket option: errno %d", errno);
        }
        
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, TCP_PORT);
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        bool file_transmitted = pdFALSE; 
        while(true)
        {
            xStatusBits = xEventGroupGetBits(status_event_group); 
            if((xStatusBits & ucFileSaved_BIT)){
                ESP_LOGI(TAG, "Opening file: %s", pcTmpFileDefault);
                unsigned long lSize;
                struct DataFrame pDataBuffer;
                FILE* pFile = fopen(cFilepath, "rb");
                if (pFile == NULL) {
                    ESP_LOGE(TAG, "Failed to open file for reading.");
                    break;
                }
                // Read file contents and trasmit
                ESP_LOGI(TAG, "Reading contents...");
                // obtain file size:
                fseek(pFile , 0 , SEEK_END);
                lSize = ftell(pFile);
                rewind (pFile);
                ESP_LOGI(TAG, "File Size: %lu bytes", lSize);
                ESP_LOGI(TAG, "DataFrame Size: %d", sizeof(pDataBuffer));
                ESP_LOGI(TAG, "Number of DataFrame structs written: %d", (int)(lSize/sizeof(pDataBuffer)));

                uint16_t i_frame = 0;
                int64_t t_0 = 0; 
                

                while(fread(&pDataBuffer, sizeof(pDataBuffer), 1, pFile)){
                    ESP_LOGI(TAG, "frame: %d: n_samples = %d ", i_frame, pDataBuffer.n_samples);
                    float tcp_packet[pDataBuffer.n_samples][7];
                    i_frame ++;
                    uint32_t ulTime_us; 
                    icm20601::raw_axes_t accelRaw;  // accel raw axes 
                    icm20601::raw_axes_t gyroRaw;  // gyro raw axes 
                    icm20601::float_axes_t accelG;   // accel axes in (g) gravity format
                    icm20601::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
                    for(uint8_t i=0; i <= pDataBuffer.n_samples-1; i++){
                        if(t_0 == 0){
                            t_0 = pDataBuffer.Timestamps[i];
                        }
                        ulTime_us = (uint32_t)(pDataBuffer.Timestamps[i] - t_0);
                        accelRaw.x = pDataBuffer.SensorReads[0+i*12] << 8 | pDataBuffer.SensorReads[1+i*12];
                        accelRaw.y = pDataBuffer.SensorReads[2+i*12] << 8 | pDataBuffer.SensorReads[3+i*12];
                        accelRaw.z = pDataBuffer.SensorReads[4+i*12] << 8 | pDataBuffer.SensorReads[5+i*12];
                        gyroRaw.x  = pDataBuffer.SensorReads[6+i*12] << 8 | pDataBuffer.SensorReads[7+i*12];
                        gyroRaw.y  = pDataBuffer.SensorReads[8+i*12] << 8 | pDataBuffer.SensorReads[9+i*12];
                        gyroRaw.z  = pDataBuffer.SensorReads[10+i*12] << 8 | pDataBuffer.SensorReads[11+i*12];
                        
                        accelG = icm20601::accelGravity(accelRaw, kAccelFS);
                        gyroDPS = icm20601::gyroDegPerSec(gyroRaw, kGyroFS);

                        tcp_packet[i][0] = (float)(ulTime_us/1000.0);
                        tcp_packet[i][1] = accelG.x;
                        tcp_packet[i][2] = accelG.y;
                        tcp_packet[i][3] = accelG.z;
                        tcp_packet[i][4] = gyroDPS.x;
                        tcp_packet[i][5] = gyroDPS.y;
                        tcp_packet[i][6] = gyroDPS.z;

                        // ESP_LOGI(TAG, "t: %d \t Acc.x: %+6.2f \t Acc.y: %+6.2f \t Acc.z: %+6.2f \t Gyr.x: %+7.2f \t Gyr.y: %+7.2f \t Gyr.z: %+7.2f", ulTime_us, accelG.x, accelG.y, accelG.z, gyroDPS.x, gyroDPS.y, gyroDPS.z);

                    }
                    // SEND DATA VIA TCP SOCKET
                    int err = send(sock, tcp_packet, sizeof(tcp_packet), 0);
                    if (err < 0) {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        break;
                    }

                    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
                    // Error occurred during receiving
                    if (len < 0) {
                        ESP_LOGE(TAG, "recv failed: errno %d", errno);
                        // TODO: ERROR HANDLE CORRECTLY! 
                        break;
                    }
                    // Data received
                    else {
                        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                        ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                        ESP_LOGI(TAG, "%s", rx_buffer);
                    }
                }
                    
                // DONE TRANSMITTING -> CLOSE FILE AND DELETE. IT'S IN A BETTER PLACE NOW... 
                ESP_LOGI(TAG, "File successfully transmitted.");
                fclose(pFile);
                if(remove(cFilepath) != 0){
                    ESP_LOGE(TAG, "Error deleting file");                    
                }
                else{
                    ESP_LOGW(TAG, "File successfully deleted. Deleting task.");
                    xTaskCreate(vWriteFileSDTask, "WriteFile", 2 * 2048, NULL, 0, &xHandleWriteFileSDTask);
                    file_transmitted = pdTRUE; 
                    break;
                }
            }
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            if(file_transmitted){break;}
        }
    }
    vTaskDelete(xHandleTransmitFileTCP);
}

// # # # # # # # # # # 
// UDP MULTICAST
// # # # # # # # # # # 

static void vMCastRCVCommandsTask(void *pvParameters)
{
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());
    while (1) {
        int sock;

        sock = create_multicast_ipv4_socket(MULTICAST_CMD_PORT);
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
            .sin_port = htons(MULTICAST_CMD_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);
        // Loop waiting for UDP received, and sending UDP packets if we don't
        // see any.
        int err = 1;
        while (err > 0) {
            struct timeval tv = {
                .tv_sec = 10,
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
                    else{
                        // Get the sender's address as a string
                        if (raddr.sin6_family == PF_INET) {
                            inet_ntoa_r(((struct sockaddr_in *)&raddr)->sin_addr.s_addr, raddr_name, sizeof(raddr_name)-1);
                        }
                        recvbuf[len] = 0; // Null-terminate whatever we received and treat like a string...
                        ESP_LOGI(TAG, "received %d bytes from %s: %s", len, raddr_name, recvbuf);
                        // # # # # # # # # # # # 
                        // INTERPRET COMMAND 
                        // # # # # # # # # # # # 
                        // Start measurement
                        if (strncmp(recvbuf, "rec_start", 9) == 0) {
                            ESP_LOGI(TAG, "Received Start Command. Setting Event Bit.");
                            xEventGroupSetBits(command_event_group, StartMeasurement_BIT);
                        }
                        // Stop measurement                
                        else if (strncmp(recvbuf, "rec_stop", 8) == 0)
                        {
                            ESP_LOGI(TAG, "Received Stop Command. Clearing Event Bit.");
                            xEventGroupClearBits(command_event_group, StartMeasurement_BIT);
                        }
                        // OTA sensor update
                        else if (strncmp(recvbuf, "update_sensor", 13) == 0)
                        {
                            if(xEventGroupGetBits(command_event_group)&IsMeasuring_BIT){
                                // TODO: change to wifi log
                                ESP_LOGI(TAG, "Update cannot be started. Measurement in progress...");
                            }
                            else{
                                ESP_LOGI(TAG, "Received OTA Update Command. Setting Event Bit.");
                                xEventGroupSetBits(command_event_group, ucOTAUptateStart);
                            }                                
                        }
                        // Set host ip from beacon
                        else if (strncmp(recvbuf, "host_beacon", 11) == 0)
                        {
                            if(!(xEventGroupGetBits(status_event_group)&ucHostFound_BIT)){
                                host_ip = &raddr_name[0];
                                xEventGroupSetBits(status_event_group, ucHostFound_BIT);
                            } 
                        }
                        // # # # # # # # # # # # 
                        // END INTERPRET COMMAND 
                        // # # # # # # # # # # # 
                    }
                }
            }
        
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
                int err = getaddrinfo(MULTICAST_IPV4_ADDR,
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
                ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(MULTICAST_CMD_PORT);
                inet_ntoa_r(((struct sockaddr_in *)res->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)-1);
                err = sendto(sock, sendbuf, len, 0, res->ai_addr, res->ai_addrlen);
                freeaddrinfo(res);
                if (err < 0) {
                    ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
                    break;
                }
            }

        }

        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}

static void vMCastSyncTask(void *pvParameters)
{
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());
    while (1) {
        
        gpio_set_direction(PIN_NUM_SYNC_PIN, GPIO_MODE_INPUT_OUTPUT);
        gpio_set_level(PIN_NUM_SYNC_PIN, 1);
        bool sync_status;

        int sock;

        sock = create_multicast_ipv4_socket(MULTICAST_SYNC_PORT);
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
            .sin_port = htons(MULTICAST_SYNC_PORT),
        };
        // We know this inet_aton will pass because we did it above already
        inet_aton(MULTICAST_IPV4_ADDR, &sdestv4.sin_addr.s_addr);
        // Loop waiting for UDP received
        int err = 1;
        while (err > 0) {
            struct timeval tv = {
                .tv_sec = 5,
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
                    sync_status = (bool)gpio_get_level(PIN_NUM_SYNC_PIN);
                    // Incoming datagram received
                    char recvbuf[48];
                    char raddr_name[32] = { 0 };
                    struct sockaddr_in6 raddr; 
                    socklen_t socklen = sizeof(raddr);
                    recvfrom(sock, recvbuf, sizeof(recvbuf)-1, 0, (struct sockaddr *)&raddr, &socklen);
                    gpio_set_level(PIN_NUM_SYNC_PIN, !sync_status);
                }
            }
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
        ESP_LOGE(TAG, "Configured IPV4 multicast address '%s' is invalid.", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        return err;
    }
    ESP_LOGI(TAG, "Configured IPV4 Multicast address %s", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV4_ADDR);
    }

    if (assign_source_if) {
        // Assign the IPv4 multicast source interface, via its IP
        // (only necessary if this socket is IPV4 only)
        err = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                         sizeof(struct in_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
            return err;
        }
    }

    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        return err;
    }
    return err;
}

static int create_multicast_ipv4_socket(int port)
{
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());
    struct sockaddr_in saddr = { 0 };
    int sock = -1;
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket. Error %d", errno);
        return -1;
    }

    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(port);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (err < 0) {
        ESP_LOGE(TAG, "Failed to bind socket. Error %d", errno);
        close(sock);
        return -1;
    }


    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
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


// # # # # # # # # # # 
// HTTP OTA FIRMWARE 
// # # # # # # # # # # 

static void vOTAUpdateTask(void*)
{
    char* TAG = pcTaskGetTaskName(xTaskGetCurrentTaskHandle());
    wifi_log_i(TAG, "Starting OTA task", "");

    esp_http_client_config_t config = {
        .url = "https://192.168.178.68:8070/sensor_firmware/firmware.bin",
        .cert_pem = (char *)server_cert_pem_start,
        .event_handler = _http_event_handler,
    };

    while (1){
        // ESP_LOGI(TAG, "Waiting...");
        if(xEventGroupGetBits(command_event_group)&ucOTAUptateStart){
            wifi_log_i(TAG, "Updating...", "");
            // vTaskSuspendAll();
            esp_err_t ret = esp_https_ota(&config);
            if (ret == ESP_OK) {
                wifi_log_w(TAG, "Firmware upgrade successful. Restarting...", "");
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                esp_restart();
            } else {
                wifi_log_e(TAG, "Firmware upgrade failed", "");
                xEventGroupClearBits(command_event_group, ucOTAUptateStart);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// # # # # # # # # # # # # # # # # # # 
// Main
// # # # # # # # # # # # # # # # # # # 

extern "C" void app_main()
{   

    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // Create event groups for inter-task communication 
    status_event_group = xEventGroupCreate(); 
    command_event_group = xEventGroupCreate(); 

    EventBits_t uxBits;

    // Create queues to store sensor readings and system ticks of the readings 
    data_queue = xQueueCreate(5, sizeof(struct DataFrame));
    icm_ticks_queue = xQueueCreate(kFIFOReadsMax, sizeof(int64_t));


    // Create status LED task
    xTaskCreate(vStatusLEDTask, "StatusLEDTask", 2048, NULL, 1, &xHandleStatusLEDTask); 

    // Create Battery Level 
    xTaskCreate(vBattStatusTask, "BattStatusTask", 4*2048, NULL, 1, &xHandleBattStatusTask); 

    // provision WiFi and connect 
    provision_wifi(); 

    // Create UDP Multicast task for receiving host_ip, commands  
    xTaskCreate(vMCastRCVCommandsTask, "MCastRCVCommandsTask", 4096, NULL, 3, &xHandleMCastRCVTask);

        // Wait for host before anything happens 
    ESP_LOGI(TAG, "Waiting for host...");
    uxBits = xEventGroupWaitBits(
            status_event_group,   /* The event group being tested. */
            ucHostFound_BIT, /* The bits within the event group to wait for. */
            pdFALSE,        /* Don't clear bit*/
            pdTRUE,       /* Wait for all bits */
            portMAX_DELAY);  /* Wait forever if necessary. */ 
    
    if(uxBits & ucHostFound_BIT){
        ESP_LOGI(TAG, "Host found: %s", host_ip);
    }
    
    // Create task for wifi-logging
    start_wifi_logger(host_ip, WIFI_LOG_PORT); 
    vTaskDelay(5000/portTICK_PERIOD_MS);
    
    // Set up file system 
    esp_err_t ret = vMountSDCard();
    if(ret == ESP_OK){
        xTaskCreate(vWriteFileSDTask, "WriteFileSDTask", 2 * 2048, NULL, 3, &xHandleWriteFileSDTask);
    }

    // Create OTA update task 
    xTaskCreate(vOTAUpdateTask, "OTAUpdateTask", 8192, NULL, 3, &xHandleOTAUpdateTask);

    
    // Create UDP Multicast task for syncing 
    xTaskCreate(vMCastSyncTask, "MCastSyncTask", 4096, NULL, 5, &xHandleMCastSyncTask);
    
    // Initialize SPI bus 
    spi.begin(PIN_NUM_IMU_SPI_MOSI, PIN_NUM_IMU_SPI_MISO, PIN_NUM_IMU_SPI_CLK);
    
    // Create a task to setup ICM and read sensor data
    xTaskCreate(vICMTask, "ICMTask", 4 * 2048, NULL, 6, &xHandleICMTask);   
}
