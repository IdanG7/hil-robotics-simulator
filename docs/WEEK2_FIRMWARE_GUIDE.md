# Week 2 Firmware Implementation Guide

**Target**: STM32F446RE NUCLEO board
**Language**: C with STM32 HAL
**Architecture**: Structured firmware with modular design

---

## Overview

This guide provides detailed implementation specifications for each firmware module in Week 2. All code follows embedded C best practices: no dynamic allocation, static buffers, defensive programming, and deterministic timing.

---

## Module 1: UART Handler (uart_handler.c/h)

### Purpose
Handle UART communication with DMA for efficient, non-blocking serial I/O.

### Header File (uart_handler.h)

```c
#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 128

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    volatile uint16_t rx_head;  // DMA write position
    volatile uint16_t rx_tail;  // Application read position
    volatile bool tx_busy;
} UART_Handle_t;

// Public API
void UART_Init(UART_Handle_t *handle, UART_HandleTypeDef *huart);
bool UART_BytesAvailable(UART_Handle_t *handle);
uint8_t UART_ReadByte(UART_Handle_t *handle);
bool UART_SendPacket(UART_Handle_t *handle, const uint8_t *data, uint16_t len);

// Callbacks (call from HAL interrupts)
void UART_RxCallback(UART_Handle_t *handle);
void UART_TxCallback(UART_Handle_t *handle);

#endif // UART_HANDLER_H
```

### Implementation (uart_handler.c)

```c
#include "uart_handler.h"
#include <string.h>

void UART_Init(UART_Handle_t *handle, UART_HandleTypeDef *huart) {
    handle->huart = huart;
    handle->rx_head = 0;
    handle->rx_tail = 0;
    handle->tx_busy = false;

    // Start DMA circular receive
    HAL_UART_Receive_DMA(huart, handle->rx_buffer, UART_RX_BUFFER_SIZE);
}

bool UART_BytesAvailable(UART_Handle_t *handle) {
    // Update rx_head from DMA counter
    handle->rx_head = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(handle->huart->hdmarx);

    return (handle->rx_head != handle->rx_tail);
}

uint8_t UART_ReadByte(UART_Handle_t *handle) {
    uint8_t byte = handle->rx_buffer[handle->rx_tail];
    handle->rx_tail = (handle->rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return byte;
}

bool UART_SendPacket(UART_Handle_t *handle, const uint8_t *data, uint16_t len) {
    if (handle->tx_busy || len > UART_TX_BUFFER_SIZE) {
        return false;
    }

    memcpy(handle->tx_buffer, data, len);
    handle->tx_busy = true;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(handle->huart, handle->tx_buffer, len);
    if (status != HAL_OK) {
        handle->tx_busy = false;
        return false;
    }

    return true;
}

void UART_RxCallback(UART_Handle_t *handle) {
    // Called from HAL_UART_RxCpltCallback
    // Nothing needed for circular DMA
}

void UART_TxCallback(UART_Handle_t *handle) {
    // Called from HAL_UART_TxCpltCallback
    handle->tx_busy = false;
}
```

### Integration with HAL Callbacks (stm32f4xx_it.c)

```c
// Add to stm32f4xx_it.c or main.c
extern UART_Handle_t uart_handle;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        UART_TxCallback(&uart_handle);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        UART_RxCallback(&uart_handle);
    }
}
```

---

## Module 2: Protocol Parser (protocol.c/h)

### Purpose
Encode/decode binary protocol packets with CRC-8 validation. Must match Python `protocol.py` exactly.

### Header File (protocol.h)

```c
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_HEADER 0xAA
#define MAX_PAYLOAD_SIZE 64

// Command types (must match Python)
typedef enum {
    CMD_SET_JOINT_ANGLES = 0x10,
    CMD_SET_JOINT_ANGLE_SINGLE = 0x11,
    CMD_GET_TELEMETRY = 0x20,
    CMD_SYSTEM_RESET = 0x30,
    CMD_CALIBRATE_IMU = 0x31,
    CMD_SET_PID_GAINS = 0x40,
    CMD_SET_MODE = 0x50
} CommandType_t;

// Telemetry types (must match Python)
typedef enum {
    TEL_FULL = 0x01,
    TEL_ANGLES_ONLY = 0x02,
    TEL_IMU_ONLY = 0x03,
    TEL_ERROR = 0xF0,
    TEL_ACK = 0xF1
} TelemetryType_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t data[MAX_PAYLOAD_SIZE];
} Packet_t;

// Public API
uint8_t Protocol_CRC8(const uint8_t *data, uint16_t len);
bool Protocol_DecodePacket(const uint8_t *raw_buffer, uint16_t len, Packet_t *packet);
uint16_t Protocol_EncodePacket(const Packet_t *packet, uint8_t *output_buffer);

#endif // PROTOCOL_H
```

### Implementation (protocol.c)

```c
#include "protocol.h"
#include <string.h>

uint8_t Protocol_CRC8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

bool Protocol_DecodePacket(const uint8_t *raw_buffer, uint16_t len, Packet_t *packet) {
    // Minimum packet: [HEADER][TYPE][LENGTH][CRC] = 4 bytes
    if (len < 4) {
        return false;
    }

    // Check header
    if (raw_buffer[0] != PACKET_HEADER) {
        return false;
    }

    uint8_t type = raw_buffer[1];
    uint8_t length = raw_buffer[2];

    // Check length
    if (length > MAX_PAYLOAD_SIZE || len != (4 + length)) {
        return false;
    }

    // Validate CRC (over TYPE + LENGTH + DATA)
    uint8_t crc_expected = Protocol_CRC8(&raw_buffer[1], 2 + length);
    uint8_t crc_received = raw_buffer[3 + length];

    if (crc_expected != crc_received) {
        return false;
    }

    // Populate packet structure
    packet->type = type;
    packet->length = length;
    if (length > 0) {
        memcpy(packet->data, &raw_buffer[3], length);
    }

    return true;
}

uint16_t Protocol_EncodePacket(const Packet_t *packet, uint8_t *output_buffer) {
    if (packet->length > MAX_PAYLOAD_SIZE) {
        return 0;
    }

    output_buffer[0] = PACKET_HEADER;
    output_buffer[1] = packet->type;
    output_buffer[2] = packet->length;

    if (packet->length > 0) {
        memcpy(&output_buffer[3], packet->data, packet->length);
    }

    // Calculate and append CRC
    uint8_t crc = Protocol_CRC8(&output_buffer[1], 2 + packet->length);
    output_buffer[3 + packet->length] = crc;

    return 4 + packet->length;
}
```

---

## Module 3: Servo Control (servo_control.c/h)

### Purpose
Control PWM servos with angle-to-PWM conversion and limit enforcement.

### Header File (servo_control.h)

```c
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SERVO_PWM_MIN 1000   // 1ms (0°)
#define SERVO_PWM_MAX 2000   // 2ms (180°)
#define SERVO_PWM_CENTER 1500 // 1.5ms (90°)

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_angle_rad;
    float min_angle_rad;  // -π/2
    float max_angle_rad;  //  π/2
} Servo_t;

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_t *servo, float angle_rad);
float Servo_GetAngle(Servo_t *servo);

#endif // SERVO_CONTROL_H
```

### Implementation (servo_control.c)

```c
#include "servo_control.h"
#include <math.h>

// Helper function
static uint16_t angle_to_pwm(float angle_rad) {
    // Clamp to [-π/2, π/2]
    float angle_clamped = fmaxf(-M_PI_2, fminf(M_PI_2, angle_rad));

    // Convert to degrees
    float angle_deg = angle_clamped * 180.0f / M_PI;

    // Map [-90, 90] → [1000, 2000]
    float normalized = (angle_deg + 90.0f) / 180.0f;  // [0, 1]
    uint16_t pwm_us = SERVO_PWM_MIN + (uint16_t)(normalized * (SERVO_PWM_MAX - SERVO_PWM_MIN));

    return pwm_us;
}

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    servo->htim = htim;
    servo->channel = channel;
    servo->current_angle_rad = 0.0f;
    servo->min_angle_rad = -M_PI_2;
    servo->max_angle_rad = M_PI_2;

    // Start PWM output
    HAL_TIM_PWM_Start(htim, channel);

    // Set to center position
    Servo_SetAngle(servo, 0.0f);
}

void Servo_SetAngle(Servo_t *servo, float angle_rad) {
    // Clamp angle
    float angle_clamped = fmaxf(servo->min_angle_rad, fminf(servo->max_angle_rad, angle_rad));

    // Convert to PWM
    uint16_t pwm_us = angle_to_pwm(angle_clamped);

    // Update timer compare register
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pwm_us);

    servo->current_angle_rad = angle_clamped;
}

float Servo_GetAngle(Servo_t *servo) {
    return servo->current_angle_rad;
}
```

---

## Module 4: PID Controller (pid_controller.c/h)

### Purpose
Implement standard discrete PID algorithm for servo position control.

### Header File (pid_controller.h)

```c
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
```

### Implementation (pid_controller.c)

```c
#include "pid_controller.h"
#include <math.h>

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
    // Calculate error
    float error = pid->setpoint - measurement;

    // Proportional term
    float P = pid->Kp * error;

    // Integral term (with anti-windup)
    pid->integral += error * pid->dt;
    float I = pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * derivative;

    // Calculate output
    float output = P + I + D;

    // Clamp output
    if (output > pid->output_max) {
        output = pid->output_max;
        pid->integral -= error * pid->dt;  // Anti-windup
    } else if (output < pid->output_min) {
        output = pid->output_min;
        pid->integral -= error * pid->dt;  // Anti-windup
    }

    // Save error for next iteration
    pid->prev_error = error;

    return output;
}

void PID_Reset(PID_Controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
```

---

## Module 5: IMU Driver (imu_driver.c/h)

### Purpose
Basic I2C driver for MPU6050 IMU (raw data only, no fusion).

### Header File (imu_driver.h)

```c
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR 0x68 << 1  // I2C address (shifted for HAL)

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
```

### Implementation (imu_driver.c)

```c
#include "imu_driver.h"

// MPU6050 Registers
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

bool IMU_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i;

    // Check WHO_AM_I (should return 0x68)
    if (!IMU_WhoAmI(hi2c, &who_am_i) || who_am_i != 0x68) {
        return false;
    }

    // Wake up MPU6050 (clear sleep bit)
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
    uint8_t buffer[14];  // 6 accel + 2 temp + 6 gyro

    // Read all sensor data starting from ACCEL_XOUT_H
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR,
                                                 MPU6050_ACCEL_XOUT_H, 1,
                                                 buffer, 14, 100);

    if (status != HAL_OK) {
        return false;
    }

    // Parse data (big-endian)
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    // Skip temperature (buffer[6-7])
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return true;
}
```

---

## Main Control Loop (main.c)

### Overview
Orchestrates all modules: UART communication, protocol handling, PID control, servo output, and telemetry.

### Key Variables (main.c)

```c
// Global handles
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;

// Custom handles
UART_Handle_t uart_handle;
Servo_t servo_shoulder;
Servo_t servo_elbow;
PID_Controller_t pid_shoulder;
PID_Controller_t pid_elbow;

// State variables
volatile bool control_loop_flag = false;  // Set by SysTick every 20ms
float target_shoulder = 0.0f;
float target_elbow = 0.0f;
```

### Main Loop Structure

```c
int main(void) {
    // HAL initialization
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();

    // Custom initialization
    UART_Init(&uart_handle, &huart2);
    Servo_Init(&servo_shoulder, &htim2, TIM_CHANNEL_1);
    Servo_Init(&servo_elbow, &htim3, TIM_CHANNEL_2);
    PID_Init(&pid_shoulder, 2.0f, 0.05f, 0.2f, 0.02f);  // 50Hz
    PID_Init(&pid_elbow, 2.0f, 0.05f, 0.2f, 0.02f);

    // IMU initialization (optional)
    bool imu_available = IMU_Init(&hi2c1);

    // Set initial targets
    PID_SetSetpoint(&pid_shoulder, 0.0f);
    PID_SetSetpoint(&pid_elbow, 0.0f);

    // Configure SysTick for 50Hz (20ms)
    HAL_SYSTICK_Config(SystemCoreClock / 50);

    // Main loop
    while (1) {
        // Process incoming UART commands
        Process_UART_Commands();

        // Run control loop at 50Hz
        if (control_loop_flag) {
            control_loop_flag = false;
            Run_Control_Loop();
        }
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
    control_loop_flag = true;  // Trigger control loop
}
```

### Command Processing Function

```c
void Process_UART_Commands(void) {
    static uint8_t packet_buffer[128];
    static uint16_t buffer_pos = 0;

    // Read available bytes
    while (UART_BytesAvailable(&uart_handle)) {
        uint8_t byte = UART_ReadByte(&uart_handle);

        // Look for header
        if (buffer_pos == 0 && byte != PACKET_HEADER) {
            continue;  // Discard non-header bytes
        }

        packet_buffer[buffer_pos++] = byte;

        // Check if we have enough for length field
        if (buffer_pos >= 3) {
            uint8_t expected_len = 4 + packet_buffer[2];

            if (buffer_pos >= expected_len) {
                // Try to decode packet
                Packet_t packet;
                if (Protocol_DecodePacket(packet_buffer, buffer_pos, &packet)) {
                    Handle_Command(&packet);
                } else {
                    // Send error telemetry
                    Send_Error_Telemetry();
                }
                buffer_pos = 0;  // Reset for next packet
            }
        }

        // Prevent buffer overflow
        if (buffer_pos >= sizeof(packet_buffer)) {
            buffer_pos = 0;
        }
    }
}

void Handle_Command(Packet_t *packet) {
    switch (packet->type) {
        case CMD_SET_JOINT_ANGLES: {
            // Parse two floats (8 bytes)
            if (packet->length == 8) {
                float shoulder, elbow;
                memcpy(&shoulder, &packet->data[0], 4);
                memcpy(&elbow, &packet->data[4], 4);

                // Update PID setpoints
                PID_SetSetpoint(&pid_shoulder, shoulder);
                PID_SetSetpoint(&pid_elbow, elbow);

                // Send ACK
                Send_ACK();
            }
            break;
        }

        case CMD_GET_TELEMETRY: {
            Send_Telemetry_Full();
            break;
        }

        case CMD_SET_PID_GAINS: {
            // Parse 6 floats (24 bytes)
            if (packet->length == 24) {
                float kp1, ki1, kd1, kp2, ki2, kd2;
                memcpy(&kp1, &packet->data[0], 4);
                memcpy(&ki1, &packet->data[4], 4);
                memcpy(&kd1, &packet->data[8], 4);
                memcpy(&kp2, &packet->data[12], 4);
                memcpy(&ki2, &packet->data[16], 4);
                memcpy(&kd2, &packet->data[20], 4);

                PID_SetGains(&pid_shoulder, kp1, ki1, kd1);
                PID_SetGains(&pid_elbow, kp2, ki2, kd2);

                Send_ACK();
            }
            break;
        }

        default:
            Send_Error_Telemetry();
            break;
    }
}
```

### Control Loop Function

```c
void Run_Control_Loop(void) {
    // Get current servo angles (feedback)
    float current_shoulder = Servo_GetAngle(&servo_shoulder);
    float current_elbow = Servo_GetAngle(&servo_elbow);

    // Run PID controllers
    float output_shoulder = PID_Update(&pid_shoulder, current_shoulder);
    float output_elbow = PID_Update(&pid_elbow, current_elbow);

    // Update servo positions
    Servo_SetAngle(&servo_shoulder, output_shoulder);
    Servo_SetAngle(&servo_elbow, output_elbow);

    // Optional: Send telemetry every N iterations
    static uint8_t telemetry_counter = 0;
    if (++telemetry_counter >= 10) {  // Every 200ms (50Hz / 10)
        Send_Telemetry_Angles();
        telemetry_counter = 0;
    }
}
```

### Telemetry Functions

```c
void Send_Telemetry_Angles(void) {
    Packet_t packet;
    packet.type = TEL_ANGLES_ONLY;
    packet.length = 8;  // 2 floats

    float shoulder = Servo_GetAngle(&servo_shoulder);
    float elbow = Servo_GetAngle(&servo_elbow);

    memcpy(&packet.data[0], &shoulder, 4);
    memcpy(&packet.data[4], &elbow, 4);

    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}

void Send_Telemetry_Full(void) {
    Packet_t packet;
    packet.type = TEL_FULL;
    packet.length = 52;  // uint32 + 10 floats + 6 int16

    // Timestamp (milliseconds)
    uint32_t timestamp = HAL_GetTick();
    memcpy(&packet.data[0], &timestamp, 4);

    // Joint angles (2 floats)
    float shoulder = Servo_GetAngle(&servo_shoulder);
    float elbow = Servo_GetAngle(&servo_elbow);
    memcpy(&packet.data[4], &shoulder, 4);
    memcpy(&packet.data[8], &elbow, 4);

    // Joint velocities (2 floats - set to 0 for now)
    float vel_shoulder = 0.0f;
    float vel_elbow = 0.0f;
    memcpy(&packet.data[12], &vel_shoulder, 4);
    memcpy(&packet.data[16], &vel_elbow, 4);

    // IMU data (6 floats = 3 accel + 3 gyro)
    IMU_Data_t imu_data;
    bool imu_read = IMU_ReadData(&hi2c1, &imu_data);

    if (imu_read) {
        float accel_x = imu_data.accel_x / 16384.0f;  // Convert to g
        float accel_y = imu_data.accel_y / 16384.0f;
        float accel_z = imu_data.accel_z / 16384.0f;
        memcpy(&packet.data[20], &accel_x, 4);
        memcpy(&packet.data[24], &accel_y, 4);
        memcpy(&packet.data[28], &accel_z, 4);

        float gyro_x = imu_data.gyro_x / 131.0f;  // Convert to °/s
        float gyro_y = imu_data.gyro_y / 131.0f;
        float gyro_z = imu_data.gyro_z / 131.0f;
        memcpy(&packet.data[32], &gyro_x, 4);
        memcpy(&packet.data[36], &gyro_y, 4);
        memcpy(&packet.data[40], &gyro_z, 4);
    } else {
        // Fill with zeros if IMU unavailable
        memset(&packet.data[20], 0, 24);
    }

    // IMU orientation (2 floats - placeholder for now)
    float roll = 0.0f;
    float pitch = 0.0f;
    memcpy(&packet.data[44], &roll, 4);
    memcpy(&packet.data[48], &pitch, 4);

    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}

void Send_ACK(void) {
    Packet_t packet;
    packet.type = TEL_ACK;
    packet.length = 0;

    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}

void Send_Error_Telemetry(void) {
    Packet_t packet;
    packet.type = TEL_ERROR;
    packet.length = 0;

    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}
```

---

## Build Configuration

### Compiler Flags (STM32CubeIDE)

**Optimization**: `-O2` (good balance of speed and debuggability)
**Warnings**: `-Wall -Wextra` (catch potential issues)
**Math**: `-lm` (link math library for `sqrt`, `sin`, `cos`)

### Memory Layout

**Flash**: 512KB total
- Bootloader/vector table: ~16KB
- Firmware code: ~20KB
- HAL drivers: ~10KB
- Remaining: ~466KB (plenty of headroom)

**RAM**: 128KB total
- Global variables: ~4KB
- Stack: ~2KB
- Heap: 0 (no dynamic allocation)
- Remaining: ~122KB

---

## Debugging Tips

### UART Debugging
- Use `printf` redirected to UART (ITM not needed)
- Add LED blink patterns for state indication
- Monitor PA2 (TX) with logic analyzer

### Timing Issues
- Check SysTick configuration (should be 20ms for 50Hz)
- Use GPIO toggle to measure control loop execution time
- Verify timer prescalers (180MHz / 180 = 1MHz, then / 20000 = 50Hz)

### PID Tuning
- Start with Kp=1.0, Ki=0, Kd=0
- Observe oscillation, reduce Kp by 30-40%
- Add Ki=0.05 for steady-state error elimination
- Add Kd=0.2 if overshoot is excessive

---

**Guide Status**: ✅ Complete
**Purpose**: Detailed implementation reference for Week 2 firmware
**Next**: Create step-by-step implementation plan with TDD approach
