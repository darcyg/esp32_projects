#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "motor_control.hpp"

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

MotorControl::MotorControl(){}

void MotorControl::setup()
{   
int ch;
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    pwm_timer_cfg.duty_resolution = DUTY_RESOLUTION; // resolution of PWM duty
    pwm_timer_cfg.freq_hz = 50;                      // frequency of PWM signal
    pwm_timer_cfg.speed_mode = LEDC_HS_MODE;           // timer mode
    pwm_timer_cfg.timer_num = LEDC_HS_TIMER;            // timer index
    pwm_timer_cfg.clk_cfg = LEDC_AUTO_CLK;              // Auto select the source clock

    ledc_timer_config(&pwm_timer_cfg);
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
    for(ch = 0; ch < n_ch; ch++)
    {
        pwm_channel[ch].channel = LEDC_HS_CH0_CHANNEL;
        pwm_channel[ch].duty = 0;
        pwm_channel[ch].gpio_num = pwm_gpios[ch];
        pwm_channel[ch].hpoint = 0;
        pwm_channel[ch].timer_sel = LEDC_HS_TIMER;
    }
    
    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < n_ch; ch++) 
    {
        ledc_channel_config(&pwm_channel[ch]);
    }
}

esp_err_t MotorControl::calibrate()
{
    return ESP_OK;
}

esp_err_t MotorControl::setThrottle(float *throttle)
{   
    float duty; // in percent
    int duty_conv;

    for (int ch = 0; ch < n_ch; ch++) 
        {   
            duty = throttle_to_duty(throttle[ch]);
            duty_conv = get_abs_duty(duty, DUTY_RESOLUTION);
            ledc_set_duty(pwm_channel[ch].speed_mode, pwm_channel[ch].channel, duty_conv);
            ledc_update_duty(pwm_channel[ch].speed_mode, pwm_channel[ch].channel);
            printf("Channel %d set to throttle: %.1f%% \n", ch, throttle[ch]);
        }
    return ESP_OK;
}

float * MotorControl::getThrottle()
{
    return throttle;
}


int MotorControl::get_abs_duty(float rel_duty, int duty_resolution)
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

float MotorControl::throttle_to_duty(float throttle)
{
    // map throttle 0-100% to duty cycle 1/20 ... 2/20 (50Hz PWM for ESC controll)
    float duty;
    duty = throttle * 0.05 + 5;  // in percent
    return duty;
};
