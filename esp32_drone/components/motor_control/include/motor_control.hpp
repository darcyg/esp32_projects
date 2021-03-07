#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"


// configuration
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define PWM_CH_1               (27)
#define PWM_CH_2               (14)
#define PWM_CH_3               (12)
#define PWM_CH_4               (13)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define DUTY_RESOLUTION        LEDC_TIMER_14_BIT
#define PWM_CH_NUM             (4)

// class definition
class MotorControl {
    public:
    MotorControl();
    void setup();
    esp_err_t calibrate();
    esp_err_t setThrottle(float *throttle);
    float * getThrottle();

    private:
    ledc_timer_config_t pwm_timer_cfg;
    uint8_t n_ch = PWM_CH_NUM;
    float throttle[PWM_CH_NUM];
    ledc_channel_config_t pwm_channel[PWM_CH_NUM];
    uint8_t pwm_gpios[PWM_CH_NUM] = {PWM_CH_1, PWM_CH_2, PWM_CH_3, PWM_CH_4};

    int get_abs_duty(float rel_duty, int duty_resolution);
    float throttle_to_duty(float throttle);
 };

#endif