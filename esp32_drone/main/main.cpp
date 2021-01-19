#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "motor_control.hpp"
// #include "imu.h"

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