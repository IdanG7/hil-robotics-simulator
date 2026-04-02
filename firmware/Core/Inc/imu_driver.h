#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR (0x68 << 1)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_Data_t;

bool IMU_Init(I2C_HandleTypeDef *hi2c);
bool IMU_ReadData(I2C_HandleTypeDef *hi2c, IMU_Data_t *data);
bool IMU_WhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *device_id);

#endif // IMU_DRIVER_H
