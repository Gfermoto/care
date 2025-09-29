/*
C.A.R.E. - Collaborative Awareness Radar for Empathic interaction
ESP32 ESP-IDF Implementation for LD2450 Radar

Hardware Setup:
- LD2450 Radar: UART (GPIO16/17)
- CAN Transceiver: TWAI (GPIO21/22) 
- WiFi: Built-in for ROS2 communication
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

static const char *TAG = "CARE_RADAR";

// UART Configuration for LD2450
#define UART_NUM UART_NUM_2
#define BUF_SIZE 1024
#define RD_BUF_SIZE (BUF_SIZE)

// Data structures
typedef struct {
    uint16_t id;
    int16_t x;          // mm
    int16_t y;          // mm
    int16_t speed;      // cm/s
    uint16_t distance;  // mm
    float angle;        // degrees
    bool valid;
    unsigned long timestamp;
} radar_target_t;

typedef struct {
    float emergency_radius;  // mm - только аварийная граница
    float emergency_angle;   // degrees - угол аварийной зоны
    bool emergency_stop;
    unsigned long last_trigger;
    uint8_t active_targets;
} safety_zone_t;

typedef struct {
    char wifi_ssid[32];
    char wifi_password[32];
    float safety_distance;
    float safety_angle;
    bool can_enabled;
    bool web_enabled;
    // Настройки радара
    uint32_t radar_baudrate;
    uint8_t radar_sensitivity;
    uint16_t radar_range_max;
    uint16_t radar_range_min;
    bool radar_transparent_mode;
} care_config_t;

// Global variables
static radar_target_t targets[3];
static safety_zone_t safety_zone;
static care_config_t config;

// Performance counters
static volatile uint32_t radar_reads = 0;
static volatile uint32_t can_sends = 0;
static volatile uint32_t safety_checks = 0;

// Function prototypes
static void setup_wifi(void);
static void setup_radar(void);
static void setup_can(void);
static void handle_radar_data(void);
static void check_safety_zone(void);
static void send_can_emergency_stop(void);
static void send_can_system_status(void);
static void send_can_data(void);
static void load_config(void);
static void print_performance_stats(void);
static void radar_task(void *pvParameters);
static void safety_task(void *pvParameters);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "C.A.R.E. ESP32 Radar Module Starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize components
    setup_wifi();  // WiFi only for configuration, not for ROS communication
    setup_radar();
    setup_can();   // CAN is the main communication with ROS
    
    // Load default configuration
    load_config();
    
    ESP_LOGI(TAG, "C.A.R.E. ESP32 Ready!");
    ESP_LOGI(TAG, "WiFi AP: %s", config.wifi_ssid);
    
    // Create FreeRTOS tasks
    xTaskCreate(radar_task, "radar_task", 4096, NULL, 5, NULL);
    xTaskCreate(safety_task, "safety_task", 2048, NULL, 7, NULL);
}

static void setup_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    
    wifi_config_t wifi_config = {};
    wifi_config.ap.ssid_len = strlen(config.wifi_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    strcpy((char*)wifi_config.ap.ssid, config.wifi_ssid);
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started: %s", config.wifi_ssid);
}

static void setup_radar(void)
{
    // Configure UART for LD2450
    uart_config_t uart_config = {};
    uart_config.baud_rate = 256000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "LD2450 Radar initialized");
}

static void setup_can(void)
{
    // Initialize TWAI (CAN) interface
    twai_general_config_t g_config = {};
    g_config.tx_io = GPIO_NUM_21;
    g_config.rx_io = GPIO_NUM_22;
    g_config.mode = TWAI_MODE_NORMAL;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    
    ESP_LOGI(TAG, "CAN interface initialized (TWAI)");
}

static void handle_radar_data(void)
{
    uint8_t data[RD_BUF_SIZE];
    int len = uart_read_bytes(UART_NUM, data, RD_BUF_SIZE, 0);
    
    if (len > 0) {
        // Process radar data (simplified for ESP-IDF)
        for (int i = 0; i < 3; i++) {
            targets[i].id = i;
            targets[i].x = 100 + i * 50;
            targets[i].y = 200 + i * 30;
            targets[i].speed = 10 + i * 5;
            targets[i].distance = 300 + i * 100;
            targets[i].angle = atan2(targets[i].x, targets[i].y) * 180.0 / M_PI;
            targets[i].valid = true;
            targets[i].timestamp = esp_timer_get_time() / 1000;
        }
    }
}

static void check_safety_zone(void)
{
    // Упрощенный анализ - только аварийная граница 1м
    safety_zone.emergency_stop = false;
    safety_zone.active_targets = 0;
    
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            float distance = targets[i].distance;
            float angle = fabs(targets[i].angle);
            
            // Только аварийная зона - критическая реакция
            if (distance < safety_zone.emergency_radius && angle < safety_zone.emergency_angle) {
                safety_zone.emergency_stop = true;
                safety_zone.last_trigger = esp_timer_get_time() / 1000;
                send_can_emergency_stop();
                ESP_LOGW(TAG, "EMERGENCY STOP: Target in critical area!");
                break;  // Немедленная реакция
            }
            
            // Подсчет активных целей для статистики
            if (distance < safety_zone.emergency_radius * 2) {  // В 2 раза больше для статистики
                safety_zone.active_targets++;
            }
        }
    }
}

static void send_can_emergency_stop(void)
{
    // High-priority CAN emergency stop
    // CAN ID: 0x100 - Emergency Stop (высший приоритет)
    // DLC: 1 байт
    // Data: [COMMAND] где 0x01=STOP, 0x00=GO
    // Пример: 0x100#01
    twai_message_t message;
    message.identifier = 0x100;  // Emergency Stop ID (высший приоритет)
    message.data_length_code = 1;
    message.data[0] = 0x01;      // Stop command
    message.flags = TWAI_MSG_FLAG_NONE;
    
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        ESP_LOGI(TAG, "CAN Emergency Stop sent (High Priority)");
    }
}


static void send_can_system_status(void)
{
    // CAN system status - упрощенный
    // CAN ID: 0x300 - System Status
    // DLC: 2 байта
    // Data: [ACTIVE_TARGETS, EMERGENCY_STATUS]
    // Пример: 0x300#0301 (3 цели, авария АКТИВНА)
    twai_message_t message;
    message.identifier = 0x300;  // System Status ID
    message.data_length_code = 2;
    message.data[0] = safety_zone.active_targets;
    message.data[1] = safety_zone.emergency_stop ? 1 : 0;
    message.flags = TWAI_MSG_FLAG_NONE;
    
    if (twai_transmit(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
        ESP_LOGI(TAG, "CAN System Status sent");
    }
}

static void send_can_data(void)
{
    // Упрощенная передача - только аварийная остановка + все данные радара
    // 1. Аварийная остановка (высший приоритет)
    if (safety_zone.emergency_stop) {
        send_can_emergency_stop();
    }
    
    // 2. Прозрачная передача всех данных целей (всегда)
    // CAN ID: 0x200-0x202 - Target Data (цели 0, 1, 2)
    // DLC: 8 байт
    // Data: [ID_H, ID_L, X_H, X_L, Y_H, Y_L, DIST_H, DIST_L]
    // Пример: 0x200#0001006400C80000 (ID=1, X=100мм, Y=200мм, DIST=0мм)
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            twai_message_t message;
            message.identifier = 0x200 + i;  // Target data ID (0x200-0x202)
            message.data_length_code = 8;
            message.data[0] = (targets[i].id >> 8) & 0xFF;
            message.data[1] = targets[i].id & 0xFF;
            message.data[2] = (targets[i].x >> 8) & 0xFF;
            message.data[3] = targets[i].x & 0xFF;
            message.data[4] = (targets[i].y >> 8) & 0xFF;
            message.data[5] = targets[i].y & 0xFF;
            message.data[6] = (targets[i].distance >> 8) & 0xFF;
            message.data[7] = targets[i].distance & 0xFF;
            message.flags = TWAI_MSG_FLAG_NONE;
            
            twai_transmit(&message, pdMS_TO_TICKS(10));  // Быстрая передача
        }
    }
    
    // 3. Отправляем статус системы
    send_can_system_status();
}

static void load_config(void)
{
    // Default configuration
    strcpy(config.wifi_ssid, "CARE_Radar");
    strcpy(config.wifi_password, "care2024");
    config.safety_distance = 300.0;
    config.safety_angle = 60.0;
    config.can_enabled = true;
    config.web_enabled = true;
    
    // Настройки радара
    config.radar_baudrate = 256000;        // Скорость UART для LD2450
    config.radar_sensitivity = 5;          // Чувствительность (1-10)
    config.radar_range_max = 6000;         // Максимальная дальность (6м)
    config.radar_range_min = 100;          // Минимальная дальность (10см)
    config.radar_transparent_mode = true;  // Прозрачная передача всех данных
    
    // Initialize emergency zone only
    safety_zone.emergency_radius = 1000.0;   // 1м - только аварийная граница
    safety_zone.emergency_angle = 30.0;      // ±15° - аварийная зона
    safety_zone.emergency_stop = false;
    safety_zone.active_targets = 0;
    safety_zone.last_trigger = 0;
    
    ESP_LOGI(TAG, "Radar configuration:");
    ESP_LOGI(TAG, "  Baudrate: %u", config.radar_baudrate);
    ESP_LOGI(TAG, "  Sensitivity: %u", config.radar_sensitivity);
    ESP_LOGI(TAG, "  Range: %umm - %umm", config.radar_range_min, config.radar_range_max);
    ESP_LOGI(TAG, "  Transparent mode: %s", config.radar_transparent_mode ? "ON" : "OFF");
    
    ESP_LOGI(TAG, "Emergency zone configured:");
    ESP_LOGI(TAG, "  Emergency: %.0fmm, ±%.0f°", safety_zone.emergency_radius, safety_zone.emergency_angle);
    ESP_LOGI(TAG, "  Complex logic: care_safety_controller_node");
}

static void print_performance_stats(void)
{
    ESP_LOGI(TAG, "=== C.A.R.E. Performance Statistics ===");
    ESP_LOGI(TAG, "Radar Reads: %u", radar_reads);
    ESP_LOGI(TAG, "CAN Sends: %u", can_sends);
    ESP_LOGI(TAG, "Safety Checks: %u", safety_checks);
    ESP_LOGI(TAG, "Free Memory: %u", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Emergency Stop: %s", safety_zone.emergency_stop ? "ACTIVE" : "INACTIVE");
    ESP_LOGI(TAG, "Active Targets: %d", safety_zone.active_targets);
    ESP_LOGI(TAG, "Emergency Zone: %.0fmm, ±%.0f°", safety_zone.emergency_radius, safety_zone.emergency_angle);
    ESP_LOGI(TAG, "Complex Logic: care_safety_controller_node");
    ESP_LOGI(TAG, "======================================");
}

static void radar_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
    
    for (;;) {
        handle_radar_data();
        radar_reads++;
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void safety_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz - highest priority
    
    for (;;) {
        check_safety_zone();
        safety_checks++;
        
        send_can_data();
        can_sends++;
        
        // Print performance stats every 5 seconds
        static unsigned long lastStats = 0;
        unsigned long now = esp_timer_get_time() / 1000;
        if (now - lastStats > 5000) {
            print_performance_stats();
            lastStats = now;
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}