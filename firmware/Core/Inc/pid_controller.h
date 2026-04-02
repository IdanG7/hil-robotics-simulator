#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float dt;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt);
void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd);
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);
float PID_Update(PID_Controller_t *pid, float measurement);
void PID_Reset(PID_Controller_t *pid);

#endif // PID_CONTROLLER_H
