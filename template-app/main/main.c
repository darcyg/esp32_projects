#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */


#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define PWM_CH_1               (27)
#define PWM_CH_2               (14)
#define PWM_CH_3               (12)
#define PWM_CH_4               (13)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define DUTY_RESOLUTION        LEDC_TIMER_14_BIT
#define LEDC_TEST_CH_NUM       (4)

int get_abs_duty(float rel_duty, int duty_resolution)
{   
    int abs_duty; 
    int max = pow(2, duty_resolution);
    abs_duty = (int)(pow(2, duty_resolution)/100 * rel_duty);
    if(abs_duty > max)
    {
        abs_duty = max; 
    }
    else if(abs_duty < 0)
    {
        abs_duty = 0;
    }
    return abs_duty;
};

float throttle_to_duty(float throttle)
{
    // map throttle 0-100% to duty cycle 1/20 ... 2/20 (50Hz PWM for ESC controll)
    float duty;
    duty = throttle * 0.05 + 5;  // in percent
    return duty;
};

void app_main(void)
{   
    int ch;
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = DUTY_RESOLUTION, // resolution of PWM duty
        .freq_hz = 50,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = PWM_CH_1,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = PWM_CH_2,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = PWM_CH_3,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = PWM_CH_4,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    

    // set to 0 

    float duty; // in percent
    int duty_conv;
    float throttle = 0.0; 

    printf("Throttle: %.2f\n", throttle);
    duty = throttle_to_duty(throttle);
    duty_conv = get_abs_duty(duty, DUTY_RESOLUTION);
    printf("duty: %.2f \t duty_conv: %d\n", duty, duty_conv);
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) 
        {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, duty_conv);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // set to 100 
    throttle = 100.0; 

    printf("Throttle: %.2f\n", throttle);
    duty = throttle_to_duty(throttle);
    duty_conv = get_abs_duty(duty, DUTY_RESOLUTION);
    printf("duty: %.2f \t duty_conv: %d\n", duty, duty_conv);
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) 
        {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, duty_conv);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }

    vTaskDelay(pdMS_TO_TICKS(3000));

    // set to 0 
    throttle = 0.0; 

    printf("Throttle: %.2f\n", throttle);
    duty = throttle_to_duty(throttle);
    duty_conv = get_abs_duty(duty, DUTY_RESOLUTION);
    printf("duty: %.2f \t duty_conv: %d\n", duty, duty_conv);
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) 
        {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, duty_conv);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }

    vTaskDelay(pdMS_TO_TICKS(3000));
    


    while (1) {
        throttle += 1; 
        if(throttle > 100)
        {
            throttle = 0.0;
        }
        
        printf("Throttle: %.2f\n", throttle);
        duty = throttle_to_duty(throttle);
        duty_conv = get_abs_duty(duty, DUTY_RESOLUTION);
        printf("duty: %.2f \t duty_conv: %d\n", duty, duty_conv);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) 
        {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, duty_conv);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }

        vTaskDelay(pdMS_TO_TICKS(30));

        // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 16384);
        // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        // printf("Software fading: 100 to 0 percent duty\n");
        // for (int duty = 100; duty >= 0; duty--) {
        //     duty_conv = get_abs_duty((float) duty, DUTY_RESOLUTION);
        //     printf("%d...%d\n", duty, duty_conv);
        //     ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty_conv);
        //     ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        //     vTaskDelay(20 / portTICK_PERIOD_MS);
        // }
        // vTaskDelay(2000 / portTICK_PERIOD_MS);

        // printf("Software setting to 0 percent duty\n");
        // ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
        // ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}