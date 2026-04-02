/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_handler.h"
#include "protocol.h"
#include "servo_control.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include "sensor_fusion.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_SHOULDER_PWM_MIN_US 1000U
#define SERVO_SHOULDER_PWM_MAX_US 2000U
#define SERVO_ELBOW_PWM_MIN_US 1000U
#define SERVO_ELBOW_PWM_MAX_US 2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
UART_Handle_t uart_handle;
Servo_t servo_shoulder;
Servo_t servo_elbow;
PID_Controller_t pid_shoulder;
PID_Controller_t pid_elbow;
SensorFusion_t imu_fusion;

volatile bool control_loop_flag = false;
bool imu_available = false;

// Watchdog variables
volatile uint32_t last_command_time = 0;
volatile bool watchdog_triggered = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Process_UART_Commands(void);
void Handle_Command(Packet_t *packet);
void Run_Control_Loop(void);
void Send_Telemetry_Angles(void);
void Send_Telemetry_Full(void);
void Send_ACK(void);
void Send_Error_Telemetry(void);
void Check_Watchdog(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Startup blink - shows firmware is running
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(100);
  }

  // Initialize UART
  UART_Init(&uart_handle, &huart2);

  // Initialize PID controllers (50Hz = 0.02s)
  PID_Init(&pid_shoulder, 2.0f, 0.05f, 0.2f, 0.02f);
  PID_Init(&pid_elbow, 2.0f, 0.05f, 0.2f, 0.02f);

  // Initialize servos (starts PWM)
  // Both servos on TIM2: PA0=CH1 (shoulder), PA1=CH2 (elbow)
  Servo_Init(&servo_shoulder, &htim2, TIM_CHANNEL_1);
  Servo_Init(&servo_elbow, &htim2, TIM_CHANNEL_2);

  Servo_SetPulseRange(&servo_shoulder, SERVO_SHOULDER_PWM_MIN_US, SERVO_SHOULDER_PWM_MAX_US);
  Servo_SetPulseRange(&servo_elbow, SERVO_ELBOW_PWM_MIN_US, SERVO_ELBOW_PWM_MAX_US);


  // Initialize IMU (non-blocking, just sets flag)
  imu_available = IMU_Init(&hi2c1);

  // Initialize sensor fusion (alpha=0.98, dt=0.02s for 50Hz)
  SensorFusion_Init(&imu_fusion, 0.98f, 0.02f);

  // Initialize watchdog timer
  last_command_time = HAL_GetTick();

  // Set initial targets
  PID_SetSetpoint(&pid_shoulder, 0.0f);
  PID_SetSetpoint(&pid_elbow, 0.0f);

  // LED on = ready
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Process_UART_Commands();
    Check_Watchdog();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 179;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 179;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Process_UART_Commands(void) {
    static uint8_t packet_buffer[128];
    static uint16_t buffer_pos = 0;

    while (UART_BytesAvailable(&uart_handle)) {
        uint8_t byte = UART_ReadByte(&uart_handle);

        if (buffer_pos == 0 && byte != PACKET_HEADER) {
            continue;
        }

        packet_buffer[buffer_pos++] = byte;

        if (buffer_pos >= 3) {
            uint8_t expected_len = 4 + packet_buffer[2];
            if (buffer_pos >= expected_len) {
                Packet_t packet;
                if (Protocol_DecodePacket(packet_buffer, buffer_pos, &packet)) {
                    Handle_Command(&packet);
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                } else {
                    Send_Error_Telemetry();
                }
                buffer_pos = 0;
            }
        }

        if (buffer_pos >= sizeof(packet_buffer)) {
            buffer_pos = 0;
        }
    }
}

void Handle_Command(Packet_t *packet) {
    // Reset watchdog on any valid command
    last_command_time = HAL_GetTick();
    watchdog_triggered = false;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // LED on = active
    
    switch (packet->type) {
        case CMD_SET_JOINT_ANGLES: {
            if (packet->length == 8) {
                float shoulder, elbow;
                memcpy(&shoulder, &packet->data[0], 4);
                memcpy(&elbow, &packet->data[4], 4);
                // Directly set servo angles
                Servo_SetAngle(&servo_shoulder, shoulder);
                Servo_SetAngle(&servo_elbow, elbow);
                // Also update PID setpoints for future use
                PID_SetSetpoint(&pid_shoulder, shoulder);
                PID_SetSetpoint(&pid_elbow, elbow);
                Send_ACK();
            }
            break;
        }
        case CMD_GET_TELEMETRY: {
            Send_Telemetry_Full();
            break;
        }
        case CMD_SET_PID_GAINS: {
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

void Check_Watchdog(void) {
    uint32_t now = HAL_GetTick();

    if (now - last_command_time > WATCHDOG_TIMEOUT_MS) {
        if (!watchdog_triggered) {
            // First trigger: move to safe position
            Servo_SetAngle(&servo_shoulder, SERVO_SAFE_POSITION);
            Servo_SetAngle(&servo_elbow, SERVO_SAFE_POSITION);
            watchdog_triggered = true;

            // Blink LED rapidly to indicate watchdog
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
    }
}

void Run_Control_Loop(void) {
    float current_shoulder = Servo_GetAngle(&servo_shoulder);
    float current_elbow = Servo_GetAngle(&servo_elbow);

    float output_shoulder = PID_Update(&pid_shoulder, current_shoulder);
    float output_elbow = PID_Update(&pid_elbow, current_elbow);

    Servo_SetAngle(&servo_shoulder, output_shoulder);
    Servo_SetAngle(&servo_elbow, output_elbow);

    static uint8_t telemetry_counter = 0;
    if (++telemetry_counter >= 10) {
        Send_Telemetry_Angles();
        telemetry_counter = 0;
    }
}

void Send_Telemetry_Angles(void) {
    Packet_t packet;
    packet.type = TEL_ANGLES_ONLY;
    packet.length = 8;

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
    packet.length = 52;

    uint32_t timestamp = HAL_GetTick();
    memcpy(&packet.data[0], &timestamp, 4);

    float shoulder = 0.0f;
    float elbow = 0.0f;
    memcpy(&packet.data[4], &shoulder, 4);
    memcpy(&packet.data[8], &elbow, 4);

    float vel_shoulder = 0.0f;
    float vel_elbow = 0.0f;
    memcpy(&packet.data[12], &vel_shoulder, 4);
    memcpy(&packet.data[16], &vel_elbow, 4);

    IMU_Data_t imu_data;
    float roll = 0.0f;
    float pitch = 0.0f;
    if (imu_available && IMU_ReadData(&hi2c1, &imu_data)) {
        float accel_x = imu_data.accel_x / 16384.0f;
        float accel_y = imu_data.accel_y / 16384.0f;
        float accel_z = imu_data.accel_z / 16384.0f;
        memcpy(&packet.data[20], &accel_x, 4);
        memcpy(&packet.data[24], &accel_y, 4);
        memcpy(&packet.data[28], &accel_z, 4);

        float gyro_x = imu_data.gyro_x / 131.0f;
        float gyro_y = imu_data.gyro_y / 131.0f;
        float gyro_z = imu_data.gyro_z / 131.0f;
        memcpy(&packet.data[32], &gyro_x, 4);
        memcpy(&packet.data[36], &gyro_y, 4);
        memcpy(&packet.data[40], &gyro_z, 4);

        // Update sensor fusion with raw IMU data
        SensorFusion_Update(&imu_fusion,
                            imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                            imu_data.gyro_x, imu_data.gyro_y);
        roll = SensorFusion_GetRoll(&imu_fusion);
        pitch = SensorFusion_GetPitch(&imu_fusion);
    } else {
        memset(&packet.data[20], 0, 24);
    }

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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
