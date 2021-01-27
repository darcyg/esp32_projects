#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// RTOS header files 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"

// ESP-IDF header files 
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"

// Components 
#include "motor_control.hpp"
#include "SPIbus.hpp"
#include "MPU.hpp"
// #include "mpu/math.hpp"
// #include "mpu/types.hpp"



extern "C" {
    void app_main(void);
    }

MotorControl Motors; 


void setup()
{   
    Motors.setup();
}


void app_main(void)
{   
    setup();
    float throttle[4] = {0.0, 0.0, 0.0, 0.0};
    while (1)
    {   
        Motors.setThrottle(throttle);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        for(int ch=0; ch<4; ch++)
        {
            throttle[ch] = 100.0;
        }
        Motors.setThrottle(throttle);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        for(int ch=0; ch<4; ch++)
        {
            throttle[ch] = 10.0;
        }
    }
    
}