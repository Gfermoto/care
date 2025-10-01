/*
C.A.R.E. STM32F407 Interrupt Service Routines
*/

#include "stm32f4xx_it.h"
#ifdef USE_HAL_DRIVER
#include "main.h"
// External variables
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1;

// System tick handler
void SysTick_Handler(void) {
    HAL_IncTick();
}

// UART2 interrupt handler
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}

// CAN1 interrupt handler
void CAN1_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX1_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_SCE_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}

// SPI1 interrupt handler
void SPI1_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi1);
}

// Default exception handlers
void NMI_Handler(void) {
    // Non-maskable interrupt
}

void HardFault_Handler(void) {
    // Hard fault
    while (1) {
        // Error state
    }
}

void MemManage_Handler(void) {
    // Memory management fault
    while (1) {
        // Error state
    }
}

void BusFault_Handler(void) {
    // Bus fault
    while (1) {
        // Error state
    }
}

void UsageFault_Handler(void) {
    // Usage fault
    while (1) {
        // Error state
    }
}

void SVC_Handler(void) {
    // System service call
}

void DebugMon_Handler(void) {
    // Debug monitor
}

void PendSV_Handler(void) {
    // Pendable service
}

void WWDG_IRQHandler(void) {
    // Window watchdog
}

void PVD_IRQHandler(void) {
    // PVD through EXTI line detection
}

void TAMP_STAMP_IRQHandler(void) {
    // Tamper and TimeStamp through the EXTI line
}

void RTC_WKUP_IRQHandler(void) {
    // RTC Wakeup through the EXTI line
}

void FLASH_IRQHandler(void) {
    // Flash memory
}

void RCC_IRQHandler(void) {
    // RCC
}

void EXTI0_IRQHandler(void) {
    // EXTI Line0
}

void EXTI1_IRQHandler(void) {
    // EXTI Line1
}

void EXTI2_IRQHandler(void) {
    // EXTI Line2
}

void EXTI3_IRQHandler(void) {
    // EXTI Line3
}

void EXTI4_IRQHandler(void) {
    // EXTI Line4
}

void DMA1_Stream0_IRQHandler(void) {
    // DMA1 Stream0
}

void DMA1_Stream1_IRQHandler(void) {
    // DMA1 Stream1
}

void DMA1_Stream2_IRQHandler(void) {
    // DMA1 Stream2
}

void DMA1_Stream3_IRQHandler(void) {
    // DMA1 Stream3
}

void DMA1_Stream4_IRQHandler(void) {
    // DMA1 Stream4
}

void DMA1_Stream5_IRQHandler(void) {
    // DMA1 Stream5
}

void DMA1_Stream6_IRQHandler(void) {
    // DMA1 Stream6
}

void DMA1_Stream7_IRQHandler(void) {
    // DMA1 Stream7
}

void ADC_IRQHandler(void) {
    // ADC1, ADC2 and ADC3
}

// CAN1 handlers already defined above

void EXTI9_5_IRQHandler(void) {
    // EXTI Line[9:5]
}

void TIM1_BRK_TIM9_IRQHandler(void) {
    // TIM1 Break and TIM9
}

void TIM1_UP_TIM10_IRQHandler(void) {
    // TIM1 Update and TIM10
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    // TIM1 Trigger and Commutation and TIM11
}

void TIM1_CC_IRQHandler(void) {
    // TIM1 Capture Compare
}

void TIM2_IRQHandler(void) {
    // TIM2
}

void TIM3_IRQHandler(void) {
    // TIM3
}

void TIM4_IRQHandler(void) {
    // TIM4
}

void I2C1_EV_IRQHandler(void) {
    // I2C1 Event
}

void I2C1_ER_IRQHandler(void) {
    // I2C1 Error
}

void I2C2_EV_IRQHandler(void) {
    // I2C2 Event
}

void I2C2_ER_IRQHandler(void) {
    // I2C2 Error
}

// SPI1 handler already defined above

void SPI2_IRQHandler(void) {
    // SPI2
}

void USART1_IRQHandler(void) {
    // USART1
}

// USART2 handler already defined above

void USART3_IRQHandler(void) {
    // USART3
}

void EXTI15_10_IRQHandler(void) {
    // EXTI Line[15:10]
}

void RTC_Alarm_IRQHandler(void) {
    // RTC Alarm (A and B) through EXTI Line
}

void OTG_FS_WKUP_IRQHandler(void) {
    // USB OTG FS Wakeup through EXTI line
}

void TIM8_BRK_TIM12_IRQHandler(void) {
    // TIM8 Break and TIM12
}

void TIM8_UP_TIM13_IRQHandler(void) {
    // TIM8 Update and TIM13
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    // TIM8 Trigger and Commutation and TIM14
}

void TIM8_CC_IRQHandler(void) {
    // TIM8 Capture Compare
}

void DMA1_Stream8_IRQHandler(void) {
    // DMA1 Stream8
}

void FSMC_IRQHandler(void) {
    // FSMC
}

void SDIO_IRQHandler(void) {
    // SDIO
}

void TIM5_IRQHandler(void) {
    // TIM5
}

void SPI3_IRQHandler(void) {
    // SPI3
}

void UART4_IRQHandler(void) {
    // UART4
}

void UART5_IRQHandler(void) {
    // UART5
}

void TIM6_DAC_IRQHandler(void) {
    // TIM6 and DAC1&2 underrun errors
}

void TIM7_IRQHandler(void) {
    // TIM7
}

void DMA2_Stream0_IRQHandler(void) {
    // DMA2 Stream0
}

void DMA2_Stream1_IRQHandler(void) {
    // DMA2 Stream1
}

void DMA2_Stream2_IRQHandler(void) {
    // DMA2 Stream2
}

void DMA2_Stream3_IRQHandler(void) {
    // DMA2 Stream3
}

void DMA2_Stream4_IRQHandler(void) {
    // DMA2 Stream4
}

void ETH_IRQHandler(void) {
    // Ethernet
}

void ETH_WKUP_IRQHandler(void) {
    // Ethernet Wakeup through EXTI line
}

void CAN2_TX_IRQHandler(void) {
    // CAN2 TX
}

void CAN2_RX0_IRQHandler(void) {
    // CAN2 RX0
}

void CAN2_RX1_IRQHandler(void) {
    // CAN2 RX1
}

void CAN2_SCE_IRQHandler(void) {
    // CAN2 SCE
}

void OTG_FS_IRQHandler(void) {
    // USB OTG FS
}

void DMA2_Stream5_IRQHandler(void) {
    // DMA2 Stream5
}

void DMA2_Stream6_IRQHandler(void) {
    // DMA2 Stream6
}

void DMA2_Stream7_IRQHandler(void) {
    // DMA2 Stream7
}

void USART6_IRQHandler(void) {
    // USART6
}

void I2C3_EV_IRQHandler(void) {
    // I2C3 event
}

void I2C3_ER_IRQHandler(void) {
    // I2C3 error
}

void OTG_HS_EP1_OUT_IRQHandler(void) {
    // USB OTG HS End Point1 Out
}

void OTG_HS_EP1_IN_IRQHandler(void) {
    // USB OTG HS End Point1 In
}

void OTG_HS_WKUP_IRQHandler(void) {
    // USB OTG HS Wakeup through EXTI
}

void OTG_HS_IRQHandler(void) {
    // USB OTG HS
}

void DCMI_IRQHandler(void) {
    // DCMI
}

void HASH_RNG_IRQHandler(void) {
    // Hash and Rng
}

void FPU_IRQHandler(void) {
    // FPU
}

#endif // USE_HAL_DRIVER
