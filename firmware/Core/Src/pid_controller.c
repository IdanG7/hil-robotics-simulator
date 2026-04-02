#include "pid_controller.h"
#include <math.h>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -M_PI_2;
    pid->output_max = M_PI_2;
}

void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetSetpoint(PID_Controller_t *pid, float setpoint) {
    pid->setpoint = setpoint;
}

float PID_Update(PID_Controller_t *pid, float measurement) {
    float error = pid->setpoint - measurement;
    float P = pid->Kp * error;
    pid->integral += error * pid->dt;
    float I = pid->Ki * pid->integral;
    float derivative = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * derivative;
    float output = P + I + D;

    if (output > pid->output_max) {
        output = pid->output_max;
        pid->integral -= error * pid->dt;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        pid->integral -= error * pid->dt;
    }

    pid->prev_error = error;
    return output;
}

void PID_Reset(PID_Controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
