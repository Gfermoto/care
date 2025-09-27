/*
C.A.R.E. Radar Module - Header File (Raspberry Pi Pico)
Collaborative Awareness Radar for Empathic interaction

This header defines the core data structures and function prototypes
for the C.A.R.E. radar safety system on Raspberry Pi Pico.
*/

#ifndef CARE_RADAR_H
#define CARE_RADAR_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MCP2515.h>
#include <LD2450.h>

// Version information
#define CARE_VERSION "1.0.0"
#define CARE_BUILD_DATE __DATE__ " " __TIME__

// Hardware configuration
#define RADAR_UART_RX_PIN 0
#define RADAR_UART_TX_PIN 1
#define CAN_CS_PIN 17
#define CAN_INT_PIN 20
#define LED_PIN LED_BUILTIN

// SPI pins for MCP2515
#define CAN_MOSI_PIN 19
#define CAN_MISO_PIN 16
#define CAN_SCK_PIN 18

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
    float safety_distance;
    float safety_angle;
    bool can_enabled;
    uint32_t can_bitrate;
    bool debug_enabled;
};

// Global variables
extern RadarTarget targets[3];
extern SafetyZone safety_zone;
extern CareConfig config;
extern Adafruit_MCP2515 mcp;

// Function prototypes
void setupRadar();
void setupCAN();
void handleRadarData();
void checkSafetyZone();
void sendCANEmergencyStop();
void sendCANData();
void printStatus();
void updateLED();
void loadConfig();
void saveConfig();

// Utility functions
float calculateAngle(int16_t x, int16_t y);
float calculateDistance(int16_t x, int16_t y);
bool isInSafetyZone(const RadarTarget& target);
void resetSafetyZone();
String getStatusJSON();
String getTargetsJSON();

// CAN utility functions
void packTargetData(const RadarTarget& target, uint8_t* data);
void unpackTargetData(const uint8_t* data, RadarTarget& target);
bool sendCANFrame(uint32_t id, const uint8_t* data, uint8_t len);
bool receiveCANFrame(can_frame& frame);

#endif // CARE_RADAR_H
