/*
C.A.R.E. Radar Module - Header File
Collaborative Awareness Radar for Empathic interaction

This header defines the core data structures and function prototypes
for the C.A.R.E. radar safety system.
*/

#ifndef CARE_RADAR_H
#define CARE_RADAR_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LD2450.h>

// Version information
#define CARE_VERSION "1.0.0"
#define CARE_BUILD_DATE __DATE__ " " __TIME__

// Hardware configuration
#define RADAR_UART_RX_PIN 16
#define RADAR_UART_TX_PIN 17
#define CAN_TX_PIN 21
#define CAN_RX_PIN 22
#define LED_PIN LED_BUILTIN

// Safety thresholds
#define DEFAULT_MIN_DISTANCE 300.0  // mm
#define DEFAULT_MAX_ANGLE 60.0      // degrees
#define CAN_SEND_INTERVAL 100       // ms

// CAN message IDs
#define CAN_EMERGENCY_STOP_ID 0x100
#define CAN_TARGET_DATA_ID 0x200
#define CAN_STATUS_ID 0x300

// Data structures
struct RadarTarget {
    uint16_t id;
    int16_t x;          // mm
    int16_t y;          // mm
    int16_t speed;      // cm/s
    uint16_t distance;  // mm
    float angle;        // degrees
    bool valid;
    unsigned long timestamp;
};

struct SafetyZone {
    float min_distance;  // mm
    float max_angle;     // degrees
    bool emergency_stop;
    unsigned long last_trigger;
};

struct CareConfig {
    String wifi_ssid;
    String wifi_password;
    float safety_distance;
    float safety_angle;
    bool can_enabled;
    bool web_enabled;
    uint32_t can_bitrate;
};

// Global variables
extern RadarTarget targets[3];
extern SafetyZone safety_zone;
extern CareConfig config;
extern WebServer server;

// Function prototypes
void setupWiFi();
void setupRadar();
void setupCAN();
void setupWebServer();
void handleRadarData();
void checkSafetyZone();
void sendCANEmergencyStop();
void sendCANData();
void handleRoot();
void handleConfig();
void handleStatus();
void handleAPI();
void loadConfig();
void saveConfig();
void printStatus();
void updateLED();

// Utility functions
float calculateAngle(int16_t x, int16_t y);
float calculateDistance(int16_t x, int16_t y);
bool isInSafetyZone(const RadarTarget& target);
void resetSafetyZone();
String getStatusJSON();
String getTargetsJSON();

#endif // CARE_RADAR_H
