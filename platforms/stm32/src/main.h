/*
C.A.R.E. STM32F407 Main Header File
*/

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// STM32 HAL includes
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

// Standard C includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Project includes
#include "care_radar.h"

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
