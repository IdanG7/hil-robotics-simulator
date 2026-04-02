#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SERVO_PWM_MIN 1000
#define SERVO_PWM_MAX 2000
#define SERVO_PWM_CENTER 1500

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_angle_rad;
    float min_angle_rad;
    float max_angle_rad;
    uint16_t pwm_min_us;
    uint16_t pwm_max_us;
} Servo_t;

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetPulseRange(Servo_t *servo, uint16_t min_us, uint16_t max_us);
void Servo_SetAngle(Servo_t *servo, float angle_rad);
float Servo_GetAngle(Servo_t *servo);

#endif // SERVO_CONTROL_H
