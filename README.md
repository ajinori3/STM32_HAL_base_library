# STM32_HAL_base_library
For HAL and STM32MXCube user

---
# Developer Enviroment
+ MCU: STM32F103RCT(on STorM32 V1.32 board)
       STM32F405RG (little modify is needed)
+ IDE: Visual GDB(Using TrueSTUDIO as STM32MXCube IDE setting)
---
# How to use
1.  Open .ioc file with STM32CubeMX.
2.  Adjust settings to your environment.
    I recommend not to change Code Generator settings.
3.  Generate code.

4.  [Additional] If you want this library in non-STM32F1xx family, please modify 
#include <stm32f1xx_hal.h> in MPU6050.h 
to fit your MCU familly.

More detail, please read sample code or contact on twitter @hakumai_no_tomo.
