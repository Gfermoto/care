/*
C.A.R.E. STM32F407 Main Header File
*/

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// STM32 HAL includes - только при компиляции для STM32
#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#endif

// Standard C includes (only those directly used in header)
#include <math.h>

// Project includes
#ifdef USE_HAL_DRIVER
#include "care_radar.h"
#endif

// Function prototypes
void Error_Handler(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_UART2_Init(void);
void MX_CAN1_Init(void);
void MX_SPI1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
