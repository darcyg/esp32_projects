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

void calibrate_throttle()
{
    float throttle_high[4] = {100.0, 100.0, 100.0, 100.0};
    float throttle_low[4] = {0.0, 0.0, 0.0, 0.0};
    Motors.setThrottle(throttle_high);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    Motors.setThrottle(throttle_low);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

void app_main(void)
{   
    setup();
    calibrate_throttle();
    float throttle[4];
    while (1)
    {   
        for(int t=30; t>=0; t-=1)
        {
            for(int ch=0; ch<4; ch++)
            {
            throttle[ch] = (float)t;
            }
            Motors.setThrottle(throttle);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }        
    }
}
