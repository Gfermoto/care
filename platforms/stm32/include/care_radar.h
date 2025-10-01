#ifndef CARE_RADAR_H
#define CARE_RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// STM32 HAL includes - только при компиляции для STM32
#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"
#else
// Forward declarations для линтера и IDE
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;
typedef struct __CAN_HandleTypeDef CAN_HandleTypeDef;
typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;
typedef struct GPIO_TypeDef GPIO_TypeDef;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// UART Configuration for LD2450
#define RADAR_UART UART2
#define RADAR_UART_BAUDRATE 256000
#define RADAR_UART_TX_PIN GPIO_PIN_2
#define RADAR_UART_RX_PIN GPIO_PIN_3
#define RADAR_UART_PORT GPIOA

// CAN Configuration
#define CAN_INSTANCE CAN1
#define CAN_TX_PIN GPIO_PIN_0
#define CAN_RX_PIN GPIO_PIN_1
#define CAN_PORT GPIOB

// SPI Configuration for MCP2515 (if external CAN controller needed)
#define MCP2515_SPI SPI1
#define MCP2515_CS_PIN GPIO_PIN_4
#define MCP2515_CS_PORT GPIOA
#define MCP2515_INT_PIN GPIO_PIN_5
#define MCP2515_INT_PORT GPIOA

// LED Configuration
#define STATUS_LED_PIN GPIO_PIN_13
#define STATUS_LED_PORT GPIOC
#define ERROR_LED_PIN GPIO_PIN_14
#define ERROR_LED_PORT GPIOC

// Radar Target Data Structure
typedef struct {
    uint16_t id;
    int16_t x;          // X-coordinate in mm
    int16_t y;          // Y-coordinate in mm
    int16_t speed;     // Speed in mm/s
    uint16_t distance; // Distance from sensor in mm
    float angle;        // Angle in degrees
    bool valid;         // Is target valid
    uint32_t timestamp; // Timestamp of last update in milliseconds
} __attribute__((packed)) RadarTarget;

// Safety Zone Status Structure
typedef struct {
    float min_distance;    // Minimum safe distance in mm
    float max_angle;       // Maximum angle for safety zone in degrees
    bool emergency_stop;   // True if emergency stop is active
    uint8_t active_targets; // Number of targets in safety zone
    uint32_t last_trigger; // Timestamp of last emergency stop trigger in milliseconds
} __attribute__((packed)) SafetyZone;

// Configuration Structure
typedef struct {
    char wifi_ssid[32];
    char wifi_password[64];
    uint16_t safety_distance;
    uint16_t safety_angle;
    bool can_enabled;
    bool web_enabled;
    uint32_t update_frequency;
} CareConfig;

// Global variables
extern CareConfig config;
extern SafetyZone safety_zone;
extern RadarTarget targets[3];
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1;

// Function prototypes
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_UART2_Init(void);
void MX_CAN1_Init(void);
void MX_SPI1_Init(void);

void setupRadar(void);
void setupCAN(void);
void setupSPI(void);
void handleRadarData(void);
void checkSafetyZone(void);
void sendCANEmergencyStop(void);
void sendCANData(void);
void loadConfig(void);
void printPerformanceStats(void);

// HAL Callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

// Utility functions
uint32_t getSystemTick(void);
void delay_ms(uint32_t ms);
void setStatusLED(bool state);
void setErrorLED(bool state);
void blinkLED(GPIO_TypeDef* port, uint16_t pin, uint32_t duration);

// CAN Message IDs
#define CAN_EMERGENCY_STOP_ID 0x100
#define CAN_TARGET_DATA_BASE_ID 0x200
#define CAN_STATUS_ID 0x300

// CAN Data Length
#define CAN_EMERGENCY_STOP_DLC 1
#define CAN_TARGET_DATA_DLC 8
#define CAN_STATUS_DLC 4

#endif // CARE_RADAR_H
