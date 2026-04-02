// firmware/Core/Inc/sensor_fusion.h
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float roll;         // Roll angle (radians)
    float pitch;        // Pitch angle (radians)
    float alpha;        // Filter coefficient (0.98 typical)
    float dt;           // Sample period (seconds)
    bool initialized;   // First sample flag
} SensorFusion_t;

void SensorFusion_Init(SensorFusion_t *fusion, float alpha, float dt);
void SensorFusion_Update(SensorFusion_t *fusion,
                         int16_t accel_x, int16_t accel_y, int16_t accel_z,
                         int16_t gyro_x, int16_t gyro_y);
float SensorFusion_GetRoll(SensorFusion_t *fusion);
float SensorFusion_GetPitch(SensorFusion_t *fusion);
void SensorFusion_Reset(SensorFusion_t *fusion);

#endif // SENSOR_FUSION_H
