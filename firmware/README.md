# HIL Firmware - STM32 NUCLEO-F446RE

Real-time control firmware for 2-DOF robotic arm.

## Hardware Configuration

- **MCU**: STM32F446RET6 (ARM Cortex-M4, 180MHz)
- **USART2**: USB-Serial communication (PA2 TX, PA3 RX)
- **TIM2_CH1**: Servo 1 PWM (PA0)
- **TIM3_CH2**: Servo 2 PWM (PA1)
- **I2C1**: MPU6050 IMU (PB8 SCL, PB9 SDA)

## Build

Open project in STM32CubeIDE:
1. File → Open Projects from File System
2. Select `firmware/` directory
3. Build: Project → Build Project (Ctrl+B)

## Flash

Connect NUCLEO board via USB:
1. Build project
2. Run → Debug (F11) or Run (Ctrl+F11)

## Project Structure

```
firmware/
├── Core/
│   ├── Src/
│   │   ├── main.c              # Initialization, main loop
│   │   ├── uart_handler.c      # (to be added)
│   │   └── protocol.c          # (to be added)
│   └── Inc/
│       └── ...
└── README.md
```
