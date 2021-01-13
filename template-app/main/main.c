/* LEDC (LED Controller) fade example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include <driver/adc.h>
#include "driver/gpio.h"
#include "esp_adc_cal.h"



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
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define DUTY_RESOLUTION        LEDC_TIMER_14_BIT

#define LEDC_TEST_CH_NUM       (1)

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   20          //Multisampling


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC1_CHANNEL_5;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

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

void app_main(void)
{   

    int ch;

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);


    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

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
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    float duty; // in percent
    int duty_conv;
    float throttle;
    printf("Change Poti to control LED...%d \n", width);
    while (1) {       
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) 
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);

        throttle = adc_reading/pow(2, 12) * 100; 
        printf("Throttle: %.2f\n", throttle);

        duty = throttle_to_duty(throttle);
        duty_conv = get_abs_duty(duty, DUTY_RESOLUTION);
        printf("duty: %.2f \t duty_conv: %d\n", duty, duty_conv);
        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty_conv);
        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);

        vTaskDelay(pdMS_TO_TICKS(100));

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