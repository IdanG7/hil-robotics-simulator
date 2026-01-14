# STM32CubeIDE Project Setup Instructions

**Task 10: STM32 Firmware Project Setup**

Follow these steps when you have STM32CubeIDE installed and ready to use.

---

## Step 1: Launch STM32CubeIDE and Create Project

Open STM32CubeIDE (Windows):

1. **File → New → STM32 Project**
2. **Board Selector** tab → Search "**NUCLEO-F446RE**"
3. Select the board → Click **Next**
4. **Project Name**: `hil_firmware`
5. **Location**: `D:\Projects\Robot\firmware`
6. **Targeted Language**: **C**
7. Click **Finish**
8. When prompted "Initialize all peripherals with their default mode?": Select **Yes**

---

## Step 2: Configure Peripherals in STM32CubeMX

The `.ioc` file should auto-open. If not, double-click `hil_firmware.ioc` in Project Explorer.

### USART2 (USB-Serial via ST-Link)

**Location**: Connectivity → USART2

**Configuration**:
- **Mode**: Asynchronous
- **Parameter Settings**:
  - **Baud Rate**: 115200 Bits/s
  - **Word Length**: 8 Bits
  - **Parity**: None
  - **Stop Bits**: 1
- **DMA Settings** (click "Add" button):
  - **USART2_RX**: DMA1 Stream 5, Direction: Peripheral to Memory, Mode: **Circular**
  - **USART2_TX**: DMA1 Stream 6, Direction: Memory to Peripheral, Mode: **Normal**

### TIM2 (Servo 1 PWM)

**Location**: Timers → TIM2

**Configuration**:
- **Clock Source**: Internal Clock
- **Channel 1**: **PWM Generation CH1**
- **Parameter Settings**:
  - **Prescaler**: `180-1` = **179** (gives 1MHz timer clock from 180MHz APB)
  - **Counter Period (AutoReload)**: `20000-1` = **19999** (gives 50Hz, 20ms period)
  - **Pulse (CCR1)**: **1500** (neutral position, 1.5ms pulse width)

### TIM3 (Servo 2 PWM)

**Location**: Timers → TIM3

**Configuration**:
- **Clock Source**: Internal Clock
- **Channel 2**: **PWM Generation CH2**
- **Parameter Settings**:
  - **Prescaler**: `180-1` = **179**
  - **Counter Period (AutoReload)**: `20000-1` = **19999**
  - **Pulse (CCR2)**: **1500**

### I2C1 (MPU6050 IMU)

**Location**: Connectivity → I2C1

**Configuration**:
- **Mode**: **I2C**
- **Parameter Settings**:
  - **I2C Speed Mode**: Standard Mode
  - **I2C Clock Speed**: **100 kHz**
- **GPIO Settings** (should auto-configure):
  - **PB8**: I2C1_SCL
  - **PB9**: I2C1_SDA

### GPIO (LED)

**Location**: System Core → GPIO

**Configuration**:
- Pin **PA5**: Set as **GPIO_Output** (this is the built-in LED on NUCLEO board)
  - User Label: "LED"
  - GPIO output level: Low
  - GPIO mode: Output Push Pull
  - GPIO Pull-up/Pull-down: No pull-up and no pull-down
  - Maximum output speed: Low

---

## Step 3: Generate Code

1. **Save the .ioc file** (Ctrl+S) or click **Project → Generate Code**
2. Wait for code generation to complete
3. When prompted "Do you want to open the associated perspective?": Select **Yes**

---

## Step 4: Verify Generated Files

Check that these files were created:
- `Core/Src/main.c`
- `Core/Inc/main.h`
- `Core/Src/stm32f4xx_it.c` (interrupt handlers)
- `.project` and `.cproject` (Eclipse project files)

---

## Step 5: Build Firmware to Verify Setup

1. **Project → Build Project** (or press **Ctrl+B**)
2. Check Console output

**Expected Result**: `Build Finished. 0 errors, X warnings (might have some warnings about unused variables)`

If build fails:
- Check that all peripherals are configured correctly
- Verify prescaler and counter period values
- Regenerate code from .ioc file

---

## Step 6: Verify Project Structure

Your `firmware/` directory should now contain:

```
firmware/
├── Core/
│   ├── Src/
│   │   ├── main.c
│   │   ├── stm32f4xx_it.c
│   │   └── ...
│   └── Inc/
│       ├── main.h
│       └── ...
├── Drivers/            (STM32 HAL library - gitignored)
├── .gitignore
├── README.md
├── .project
├── .cproject
└── hil_firmware.ioc
```

---

## Step 7: Initial Commit (After Completing Step 5)

**Note**: The user handles commits, not Claude. Use these commands when ready:

```bash
cd D:\Projects\Robot
git add firmware/.gitignore firmware/README.md
git add firmware/Core/Src/main.c firmware/Core/Inc/main.h
git add firmware/.project firmware/.cproject firmware/hil_firmware.ioc
git commit -m "feat(firmware): initialize STM32CubeIDE project

- Configure USART2 for serial communication (115200 baud, DMA)
- Configure TIM2/TIM3 for servo PWM (50Hz)
- Configure I2C1 for MPU6050 IMU
- Add build instructions and hardware configuration"
```

---

## Troubleshooting

**Build Error: "No rule to make target"**
- Regenerate code from .ioc file
- Clean build: Project → Clean

**UART Not Working**
- Verify PA2/PA3 pins are assigned to USART2_TX/RX
- Check that DMA is enabled and configured correctly

**PWM Not Outputting**
- Start timer in main.c: `HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);`
- Verify prescaler and counter period calculations

**I2C Communication Failing**
- Check pull-up resistors on SCL/SDA lines (usually 4.7kΩ)
- Verify MPU6050 address (0x68 or 0x69)

---

## Next Steps (Week 2)

After hardware arrives (Jan 14):
1. Flash firmware to board
2. Test LED blink (PA5)
3. Test UART communication (echo test)
4. Implement protocol handler
5. Test servo PWM output
6. Integrate MPU6050 driver

---

**Task 10 Status**: Setup instructions complete. Execute these steps when STM32CubeIDE is ready to use.
