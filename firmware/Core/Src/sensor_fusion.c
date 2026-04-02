// firmware/Core/Src/sensor_fusion.c
#include "sensor_fusion.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// MPU6050 sensitivity scales (default ±2g, ±250°/s)
#define ACCEL_SCALE (16384.0f)  // LSB/g
#define GYRO_SCALE  (131.0f)    // LSB/(°/s)
#define DEG_TO_RAD  (M_PI / 180.0f)

void SensorFusion_Init(SensorFusion_t *fusion, float alpha, float dt) {
    fusion->roll = 0.0f;
    fusion->pitch = 0.0f;
    fusion->alpha = alpha;
    fusion->dt = dt;
    fusion->initialized = false;
}

void SensorFusion_Update(SensorFusion_t *fusion,
                         int16_t accel_x, int16_t accel_y, int16_t accel_z,
                         int16_t gyro_x, int16_t gyro_y) {
    float ax = (float)accel_x / ACCEL_SCALE;
    float ay = (float)accel_y / ACCEL_SCALE;
    float az = (float)accel_z / ACCEL_SCALE;
    float gx = (float)gyro_x / GYRO_SCALE * DEG_TO_RAD;
    float gy = (float)gyro_y / GYRO_SCALE * DEG_TO_RAD;

    float accel_roll = atan2f(ay, az);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (!fusion->initialized) {
        fusion->roll = accel_roll;
        fusion->pitch = accel_pitch;
        fusion->initialized = true;
        return;
    }

    fusion->roll = fusion->alpha * (fusion->roll + gx * fusion->dt)
                 + (1.0f - fusion->alpha) * accel_roll;
    fusion->pitch = fusion->alpha * (fusion->pitch + gy * fusion->dt)
                  + (1.0f - fusion->alpha) * accel_pitch;
}

float SensorFusion_GetRoll(SensorFusion_t *fusion) {
    return fusion->roll;
}

float SensorFusion_GetPitch(SensorFusion_t *fusion) {
    return fusion->pitch;
}

void SensorFusion_Reset(SensorFusion_t *fusion) {
    fusion->roll = 0.0f;
    fusion->pitch = 0.0f;
    fusion->initialized = false;
}
