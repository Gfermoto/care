/*
C.A.R.E. Radar Module - High-Performance Header File
Collaborative Awareness Radar for Empathic interaction

This header defines the core data structures and function prototypes
for the high-performance C.A.R.E. radar safety system using FreeRTOS,
LittleFS, and optimized ESP32 features.
*/

#ifndef CARE_RADAR_H
#define CARE_RADAR_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <LD2450.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>

// Version information
#define CARE_VERSION "1.0.0-HP"
#define CARE_BUILD_DATE __DATE__ " " __TIME__
#define CARE_BUILD_TYPE "High-Performance"

// Hardware configuration
#define RADAR_UART_RX_PIN 16
#define RADAR_UART_TX_PIN 17
#define CAN_TX_PIN 21
#define CAN_RX_PIN 22
#define LED_PIN LED_BUILTIN

// FreeRTOS Configuration
#define FREERTOS_HZ 1000
#define MAX_TASK_NAME_LEN 16
#define TASK_STACK_SIZE 4096
#define QUEUE_SIZE 10
#define MUTEX_TIMEOUT portMAX_DELAY

// Task Priorities (Higher number = higher priority)
#define RADAR_TASK_PRIORITY 5
#define CAN_TASK_PRIORITY 6
#define SAFETY_TASK_PRIORITY 7  // Highest priority for safety
#define WEB_TASK_PRIORITY 3

// Safety thresholds
#define DEFAULT_MIN_DISTANCE 300.0  // mm
#define DEFAULT_MAX_ANGLE 60.0      // degrees
#define CAN_SEND_INTERVAL 100       // ms
#define SAFETY_CHECK_INTERVAL 20    // ms (50Hz)

// CAN message IDs
#define CAN_EMERGENCY_STOP_ID 0x100
#define CAN_TARGET_DATA_ID 0x200
#define CAN_STATUS_ID 0x300

// Performance optimization
#define USE_SPIRAM 1
#define USE_DMA 1
#define USE_ASYNC_WEB 1
#define USE_LITTLEFS 1
#define USE_FREERTOS 1

// Data structures with SPIRAM optimization
struct RadarTarget {
    uint16_t id;
    int16_t x;          // mm
    int16_t y;          // mm
    int16_t speed;      // cm/s
    uint16_t distance;  // mm
    float angle;        // degrees
    bool valid;
    unsigned long timestamp;
    uint8_t priority;  // FreeRTOS priority
} __attribute__((packed));

struct SafetyZone {
    float min_distance;  // mm
    float max_angle;     // degrees
    bool emergency_stop;
    unsigned long last_trigger;
    uint8_t active_targets;
} __attribute__((packed));

struct CareConfig {
    String wifi_ssid;
    String wifi_password;
    float safety_distance;
    float safety_angle;
    bool can_enabled;
    bool web_enabled;
    uint32_t can_bitrate;
    uint8_t radar_priority;
    uint8_t can_priority;
    uint8_t safety_priority;
    uint8_t web_priority;
    bool debug_enabled;
    bool performance_monitoring;
} __attribute__((packed));

// Global variables with SPIRAM optimization
extern RadarTarget targets[3] __attribute__((section(".ext_ram")));
extern SafetyZone safety_zone __attribute__((section(".ext_ram")));
extern CareConfig config __attribute__((section(".ext_ram")));

// FreeRTOS Task Handles
extern TaskHandle_t radarTaskHandle;
extern TaskHandle_t canTaskHandle;
extern TaskHandle_t webTaskHandle;
extern TaskHandle_t safetyTaskHandle;

// FreeRTOS Queues and Semaphores
extern QueueHandle_t radarDataQueue;
extern QueueHandle_t canDataQueue;
extern QueueHandle_t safetyQueue;
extern SemaphoreHandle_t configMutex;
extern SemaphoreHandle_t dataMutex;

// Async Web Server
extern AsyncWebServer server;

// Performance counters
extern volatile uint32_t radar_reads;
extern volatile uint32_t can_sends;
extern volatile uint32_t safety_checks;
extern volatile uint32_t web_requests;

// Function prototypes
void setupWiFi();
void setupRadar();
void setupCAN();
void setupLittleFS();
void setupWebServer();
void setupFreeRTOS();
void loadConfig();
void saveConfig();

// FreeRTOS Task Functions
void radarTask(void* parameter);
void canTask(void* parameter);
void webTask(void* parameter);
void safetyTask(void* parameter);

// Core Functions
void handleRadarData();
void checkSafetyZone();
void sendCANEmergencyStop();
void sendCANData();
void printPerformanceStats();
void updateLED();

// Web Server Handlers
void handleRoot(AsyncWebServerRequest* request);
void handleConfig(AsyncWebServerRequest* request);
void handleStatus(AsyncWebServerRequest* request);
void handleAPI(AsyncWebServerRequest* request);

// Utility functions
float calculateAngle(int16_t x, int16_t y);
float calculateDistance(int16_t x, int16_t y);
bool isInSafetyZone(const RadarTarget& target);
void resetSafetyZone();
String getStatusJSON();
String getTargetsJSON();
String getPerformanceJSON();

// Performance monitoring
void printTaskStats();
void printMemoryStats();
void printQueueStats();
void printMutexStats();

// LittleFS utilities
bool saveConfigToFile();
bool loadConfigFromFile();
bool saveTargetsToFile();
bool loadTargetsFromFile();

// CAN utilities
bool sendCANFrame(uint32_t id, const uint8_t* data, uint8_t len);
bool receiveCANFrame(uint32_t* id, uint8_t* data, uint8_t* len);
void packTargetData(const RadarTarget& target, uint8_t* data);
void unpackTargetData(const uint8_t* data, RadarTarget& target);

// Safety utilities
bool isEmergencyStopActive();
void triggerEmergencyStop();
void clearEmergencyStop();
uint8_t getActiveTargetCount();
float getClosestTargetDistance();

// Performance optimization
void optimizeMemoryUsage();
void optimizeTaskScheduling();
void optimizeCANPerformance();
void optimizeWebPerformance();

#endif // CARE_RADAR_H