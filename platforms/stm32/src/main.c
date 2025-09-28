/*
C.A.R.E. - Collaborative Awareness Radar for Empathic interaction
STM32F407 Implementation for LD2450 Radar

Hardware Setup:
- LD2450 Radar: UART2 (PA2/PA3)
- CAN Transceiver: CAN1 (PB0/PB1) 
- MCP2515 (if needed): SPI1 (PA4/PA5/PA6/PA7)
- Status LED: PC13
- Error LED: PC14
*/

#include "main.h"
#include "care_radar.h"
// #include "LD2450.h"  // Пока отключено - библиотека не совместима с STM32
#include "string.h"
#include "stdio.h"

// Global handles
UART_HandleTypeDef huart2;
CAN_HandleTypeDef hcan1;
SPI_HandleTypeDef hspi1;

// Global configuration and safety data
CareConfig config;
SafetyZone safety_zone;
RadarTarget targets[3];

// Performance counters
static uint32_t radar_reads = 0;
static uint32_t can_sends = 0;
static uint32_t safety_checks = 0;

// LD2450 Radar instance - пока отключено
// LD2450 radar;

int main(void) {
    // Initialize HAL
    HAL_Init();
    
    // Configure system clock
    SystemClock_Config();
    
    // Initialize GPIO
    MX_GPIO_Init();
    
    // Initialize peripherals
    MX_UART2_Init();
    MX_CAN1_Init();
    MX_SPI1_Init();
    
    // Initialize components
    setupRadar();
    setupCAN();
    
    // Load default configuration
    loadConfig();
    
    // Initialize targets
    for (int i = 0; i < 3; i++) {
        targets[i].id = i;
        targets[i].valid = false;
        targets[i].timestamp = 0;
    }
    
    // Initialize safety zone
    safety_zone.min_distance = config.safety_distance;
    safety_zone.max_angle = config.safety_angle;
    safety_zone.emergency_stop = false;
    safety_zone.active_targets = 0;
    safety_zone.last_trigger = 0;
    
    printf("C.A.R.E. STM32F407 Radar Module Starting...\r\n");
    printf("Safety Distance: %d mm\r\n", config.safety_distance);
    printf("Safety Angle: %d degrees\r\n", config.safety_angle);
    printf("CAN Enabled: %s\r\n", config.can_enabled ? "Yes" : "No");
    printf("C.A.R.E. STM32F407 Ready!\r\n");
    
    // Main loop
    uint32_t last_radar_update = 0;
    uint32_t last_safety_check = 0;
    uint32_t last_can_send = 0;
    uint32_t last_stats_print = 0;
    
    while (1) {
        uint32_t current_time = HAL_GetTick();
        
        // Radar data handling (20 Hz)
        if (current_time - last_radar_update >= 50) {
            handleRadarData();
            last_radar_update = current_time;
        }
        
        // Safety zone checking (50 Hz)
        if (current_time - last_safety_check >= 20) {
            checkSafetyZone();
            last_safety_check = current_time;
        }
        
        // CAN data sending (10 Hz)
        if (current_time - last_can_send >= 100) {
            sendCANData();
            last_can_send = current_time;
        }
        
        // Performance stats (1 Hz)
        if (current_time - last_stats_print >= 1000) {
            printPerformanceStats();
            last_stats_print = current_time;
        }
        
        // Small delay to prevent CPU overload
        HAL_Delay(1);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    // Initializes the RCC Oscillators according to the specified parameters
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | 
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // Configure UART2 pins (PA2, PA3)
    GPIO_InitStruct.Pin = RADAR_UART_TX_PIN | RADAR_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(RADAR_UART_PORT, &GPIO_InitStruct);
    
    // Configure CAN pins (PB0, PB1)
    GPIO_InitStruct.Pin = CAN_TX_PIN | CAN_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);
    
    // Configure SPI pins (PA4, PA5, PA6, PA7)
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure MCP2515 CS pin (PA4)
    GPIO_InitStruct.Pin = MCP2515_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(MCP2515_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MCP2515_CS_PORT, MCP2515_CS_PIN, GPIO_PIN_SET);
    
    // Configure MCP2515 INT pin (PA5)
    GPIO_InitStruct.Pin = MCP2515_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MCP2515_INT_PORT, &GPIO_InitStruct);
    
    // Configure LED pins
    GPIO_InitStruct.Pin = STATUS_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ERROR_LED_PIN;
    HAL_GPIO_Init(ERROR_LED_PORT, &GPIO_InitStruct);
    
    // Initialize LEDs
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_RESET);
}

void MX_UART2_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = RADAR_UART_BAUDRATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

void MX_CAN1_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    
    // Configure CAN filter
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
}

void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

void setupRadar(void) {
    // Initialize LD2450 radar - пока отключено
    // radar.begin(&huart2, false);
    printf("LD2450 Radar initialized (STM32F407)\r\n");
}

void setupCAN(void) {
    printf("CAN interface initialized (STM32F407)\r\n");
}

void handleRadarData(void) {
    radar_reads++;
    
    // Get radar targets - пока отключено
    // for (int i = 0; i < radar.getSensorSupportedTargetCount(); i++) {
    //     if (i < 3) { // Limit to 3 targets
    //         targets[i].id = i;
    //         targets[i].x = radar.getTargetXCoordinate(i);
    //         targets[i].y = radar.getTargetYCoordinate(i);
    //         targets[i].speed = radar.getTargetSpeed(i);
    //         targets[i].distance = sqrt(targets[i].x * targets[i].x + targets[i].y * targets[i].y);
    //         targets[i].angle = atan2(targets[i].x, targets[i].y) * 180.0 / M_PI;
    //         targets[i].valid = radar.getTargetMovingDistance(i) > 0;
    //         targets[i].timestamp = HAL_GetTick();
    //     }
    // }
    
    // Заглушка для тестирования
    static uint32_t test_counter = 0;
    test_counter++;
    if (test_counter % 100 == 0) {
        printf("Radar data simulation: %lu\r\n", test_counter);
    }
}

void checkSafetyZone(void) {
    safety_checks++;
    
    safety_zone.emergency_stop = false;
    safety_zone.active_targets = 0;
    
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            if (targets[i].distance < safety_zone.min_distance) {
                if (fabs(targets[i].angle) < safety_zone.max_angle) {
                    safety_zone.emergency_stop = true;
                    safety_zone.active_targets++;
                    safety_zone.last_trigger = HAL_GetTick();
                    sendCANEmergencyStop();
                    printf("EMERGENCY STOP: Target %d too close! Distance: %dmm, Angle: %.2f deg\r\n",
                           targets[i].id, targets[i].distance, targets[i].angle);
                    break;
                }
            }
        }
    }
}

void sendCANEmergencyStop(void) {
    if (!config.can_enabled) return;
    
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];
    uint32_t TxMailbox;
    
    TxHeader.StdId = CAN_EMERGENCY_STOP_ID;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = CAN_EMERGENCY_STOP_DLC;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    TxData[0] = 0x01; // Emergency stop command
    
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
        printf("CAN Emergency Stop sent\r\n");
    } else {
        printf("Failed to send CAN Emergency Stop\r\n");
    }
}

void sendCANData(void) {
    if (!config.can_enabled) return;
    
    can_sends++;
    
    if (safety_zone.emergency_stop) {
        sendCANEmergencyStop();
    } else {
        // Send target data
        for (int i = 0; i < 3; i++) {
            if (targets[i].valid) {
                CAN_TxHeaderTypeDef TxHeader;
                uint8_t TxData[8];
                uint32_t TxMailbox;
                
                TxHeader.StdId = CAN_TARGET_DATA_BASE_ID + i;
                TxHeader.ExtId = 0x01;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.DLC = CAN_TARGET_DATA_DLC;
                TxHeader.TransmitGlobalTime = DISABLE;
                
                // Pack target data into CAN frame
                TxData[0] = (targets[i].x >> 8) & 0xFF;
                TxData[1] = targets[i].x & 0xFF;
                TxData[2] = (targets[i].y >> 8) & 0xFF;
                TxData[3] = targets[i].y & 0xFF;
                TxData[4] = (targets[i].distance >> 8) & 0xFF;
                TxData[5] = targets[i].distance & 0xFF;
                TxData[6] = (targets[i].speed >> 8) & 0xFF;
                TxData[7] = targets[i].speed & 0xFF;
                
                if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
                    printf("Failed to send CAN target data for ID %d\r\n", i);
                }
            }
        }
    }
}

void loadConfig(void) {
    // Default configuration
    strcpy(config.wifi_ssid, "CARE_Radar_STM32");
    strcpy(config.wifi_password, "care2024");
    config.safety_distance = 300; // mm
    config.safety_angle = 60;    // degrees
    config.can_enabled = true;
    config.web_enabled = false;
    config.update_frequency = 20; // Hz
    
    printf("Configuration loaded\r\n");
}

void printPerformanceStats(void) {
    printf("=== C.A.R.E. STM32F407 Performance Statistics ===\r\n");
    printf("Radar Reads: %lu\r\n", radar_reads);
    printf("CAN Sends: %lu\r\n", can_sends);
    printf("Safety Checks: %lu\r\n", safety_checks);
    printf("Emergency Stop: %s\r\n", safety_zone.emergency_stop ? "ACTIVE" : "INACTIVE");
    printf("Active Targets: %d\r\n", safety_zone.active_targets);
    printf("===============================================\r\n");
}

// HAL Callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Handle radar data reception
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // Handle received CAN message
        printf("CAN message received: ID=0x%03X, DLC=%d\r\n", RxHeader.StdId, RxHeader.DLC);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    // CAN transmission complete
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    // CAN transmission complete
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    // CAN transmission complete
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    printf("CAN Error occurred\r\n");
    setErrorLED(true);
}

// Utility functions
uint32_t getSystemTick(void) {
    return HAL_GetTick();
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void setStatusLED(bool state) {
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void setErrorLED(bool state) {
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void blinkLED(GPIO_TypeDef* port, uint16_t pin, uint32_t duration) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(duration);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void Error_Handler(void) {
    __disable_irq();
    setErrorLED(true);
    while (1) {
        // Error state - blink error LED
        blinkLED(ERROR_LED_PORT, ERROR_LED_PIN, 500);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    printf("Assert failed: file %s on line %lu\r\n", file, line);
    Error_Handler();
}
#endif
