/*
C.A.R.E. STM32F407 HAL MSP (MCU Support Package) Implementation
*/

#ifdef USE_HAL_DRIVER
#include "main.h"
// External variables
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1;

// UART MSP Initialization
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(huart->Instance == USART2) {
        // Enable UART2 clock
        __HAL_RCC_USART2_CLK_ENABLE();

        // Enable GPIOA clock
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // Configure UART2 pins
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // Enable UART2 interrupt
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

// UART MSP De-initialization
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if(huart->Instance == USART2) {
        // Disable UART2 clock
        __HAL_RCC_USART2_CLK_DISABLE();

        // De-initialize UART2 pins
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

        // Disable UART2 interrupt
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}

// CAN MSP Initialization
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hcan->Instance == CAN1) {
        // Enable CAN1 clock
        __HAL_RCC_CAN1_CLK_ENABLE();

        // Enable GPIOB clock
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // Configure CAN1 pins
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // Enable CAN1 interrupts
        HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
        HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    }
}

// CAN MSP De-initialization
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
    if(hcan->Instance == CAN1) {
        // Disable CAN1 clock
        __HAL_RCC_CAN1_CLK_DISABLE();

        // De-initialize CAN1 pins
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

        // Disable CAN1 interrupts
        HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    }
}

// SPI MSP Initialization
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hspi->Instance == SPI1) {
        // Enable SPI1 clock
        __HAL_RCC_SPI1_CLK_ENABLE();

        // Enable GPIOA clock
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // Configure SPI1 pins
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // Enable SPI1 interrupt
        HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
    }
}

// SPI MSP De-initialization
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
    if(hspi->Instance == SPI1) {
        // Disable SPI1 clock
        __HAL_RCC_SPI1_CLK_DISABLE();

        // De-initialize SPI1 pins
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        // Disable SPI1 interrupt
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
    }
}

#endif // USE_HAL_DRIVER

// SystemClock_Config and Error_Handler are defined in main.c
