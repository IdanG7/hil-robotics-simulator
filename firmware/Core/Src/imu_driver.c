#include "imu_driver.h"

#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

bool IMU_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i;
    if (!IMU_WhoAmI(hi2c, &who_am_i) || who_am_i != 0x68) {
        return false;
    }
    uint8_t pwr_mgmt = 0x00;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR,
                                                  MPU6050_PWR_MGMT_1_REG, 1,
                                                  &pwr_mgmt, 1, 100);
    return (status == HAL_OK);
}

bool IMU_WhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *device_id) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR,
                                                 MPU6050_WHO_AM_I_REG, 1,
                                                 device_id, 1, 100);
    return (status == HAL_OK);
}

bool IMU_ReadData(I2C_HandleTypeDef *hi2c, IMU_Data_t *data) {
    uint8_t buffer[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR,
                                                 MPU6050_ACCEL_XOUT_H, 1,
                                                 buffer, 14, 100);
    if (status != HAL_OK) {
        return false;
    }
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
    return true;
}
