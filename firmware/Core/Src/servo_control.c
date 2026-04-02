#include "servo_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

static uint16_t angle_to_pwm(const Servo_t *servo, float angle_rad) {
    float angle_clamped = fmaxf(-M_PI_2, fminf(M_PI_2, angle_rad));
    float angle_deg = angle_clamped * 180.0f / M_PI;
    float normalized = (angle_deg + 90.0f) / 180.0f;
    uint16_t pwm_us = servo->pwm_min_us
        + (uint16_t)(normalized * (servo->pwm_max_us - servo->pwm_min_us));
    return pwm_us;
}

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    servo->htim = htim;
    servo->channel = channel;
    servo->current_angle_rad = 0.0f;
    servo->min_angle_rad = -M_PI_2;
    servo->max_angle_rad = M_PI_2;
    servo->pwm_min_us = SERVO_PWM_MIN;
    servo->pwm_max_us = SERVO_PWM_MAX;
    HAL_TIM_PWM_Start(htim, channel);
    Servo_SetAngle(servo, 0.0f);
}

void Servo_SetPulseRange(Servo_t *servo, uint16_t min_us, uint16_t max_us) {
    if (min_us >= max_us) {
        return;
    }

    servo->pwm_min_us = min_us;
    servo->pwm_max_us = max_us;
    Servo_SetAngle(servo, servo->current_angle_rad);
}

void Servo_SetAngle(Servo_t *servo, float angle_rad) {
    float angle_clamped = fmaxf(servo->min_angle_rad, fminf(servo->max_angle_rad, angle_rad));
    uint16_t pwm_us = angle_to_pwm(servo, angle_clamped);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pwm_us);
    servo->current_angle_rad = angle_clamped;
}

float Servo_GetAngle(Servo_t *servo) {
    return servo->current_angle_rad;
}
