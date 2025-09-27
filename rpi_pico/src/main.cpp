/*
C.A.R.E. - Collaborative Awareness Radar for Empathic interaction
Raspberry Pi Pico Implementation with LD2450 Radar and CAN Safety Interface

Hardware Setup:
- LD2450 Radar: UART (GPIO0/1)
- CAN Transceiver: Pico-CAN-B (SPI)
- USB: For ROS2 communication

Connections:
LD2450 => Pico
5V => VBUS
GND => GND
TX => GPIO0 (RX)
RX => GPIO1 (TX)

Pico-CAN-B => Pico
MOSI => GPIO19
MISO => GPIO16
SCK => GPIO18
CS => GPIO17
INT => GPIO20
VCC => 3.3V
GND => GND
CANH => CAN_H
CANL => CAN_L
*/

#include <Arduino.h>
#include <LD2450.h>
#include <SPI.h>
#include <Adafruit_MCP2515.h>

// Hardware Serial for LD2450
HardwareSerial radarSerial(0, 1);

// LD2450 Radar instance
LD2450 radar;

// CAN Transceiver (MCP2515)
Adafruit_MCP2515 mcp(17); // CS pin

// ROS2-like data structure
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
};

// Global variables
RadarTarget targets[3];
SafetyZone safety_zone = {300.0, 60.0, false}; // 30cm, ±60°
unsigned long last_can_send = 0;
const unsigned long CAN_SEND_INTERVAL = 100; // ms

// Function prototypes
void setupRadar();
void setupCAN();
void handleRadarData();
void checkSafetyZone();
void sendCANEmergencyStop();
void sendCANData();
void printStatus();

void setup() {
    Serial.begin(115200);
    Serial.println("C.A.R.E. Pico Radar Module Starting...");
    
    // Initialize components
    setupRadar();
    setupCAN();
    
    Serial.println("C.A.R.E. Pico Ready!");
    Serial.println("USB Serial: /dev/ttyACM0");
}

void loop() {
    // Read radar data
    if (radar.read() > 0) {
        handleRadarData();
        checkSafetyZone();
        printStatus();
    }
    
    // Send CAN data periodically
    if (millis() - last_can_send > CAN_SEND_INTERVAL) {
        sendCANData();
        last_can_send = millis();
    }
    
    delay(10);
}

void setupRadar() {
    // Initialize LD2450 on UART0
    radar.begin(radarSerial, false);
    Serial.println("LD2450 Radar initialized");
}

void setupCAN() {
    // Initialize SPI for MCP2515
    SPI.begin();
    
    // Initialize MCP2515
    if (!mcp.begin()) {
        Serial.println("Error initializing MCP2515!");
        while (1) delay(10);
    }
    
    // Set CAN speed to 500kbps
    mcp.setBitrate(CAN_500KBPS);
    mcp.setNormalMode();
    
    Serial.println("CAN interface initialized (MCP2515)");
}

void handleRadarData() {
    // Get radar targets
    for (int i = 0; i < radar.getSensorSupportedTargetCount(); i++) {
        const LD2450::RadarTarget result = radar.getTarget(i);
        
        targets[i].id = result.id;
        targets[i].x = result.x;
        targets[i].y = result.y;
        targets[i].speed = result.speed;
        targets[i].distance = result.distance;
        targets[i].angle = atan2(result.x, result.y) * 180.0 / PI;
        targets[i].valid = result.valid;
        targets[i].timestamp = millis();
    }
}

void checkSafetyZone() {
    safety_zone.emergency_stop = false;
    
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            // Check distance threshold
            if (targets[i].distance < safety_zone.min_distance) {
                // Check angle threshold
                if (abs(targets[i].angle) < safety_zone.max_angle) {
                    safety_zone.emergency_stop = true;
                    sendCANEmergencyStop();
                    Serial.println("EMERGENCY STOP: Target too close!");
                    break;
                }
            }
        }
    }
}

void sendCANEmergencyStop() {
    // Send emergency stop command via CAN
    // CAN ID: 0x100 (Emergency Stop)
    // Data: 0x01 (Stop command)
    
    can_frame frame;
    frame.can_id = 0x100;
    frame.can_dlc = 1;
    frame.data[0] = 0x01;
    
    mcp.sendMessage(&frame);
    Serial.println("CAN Emergency Stop sent");
}

void sendCANData() {
    // Send radar data via CAN for ROS2 integration
    if (safety_zone.emergency_stop) {
        sendCANEmergencyStop();
    } else {
        // Send target data
        for (int i = 0; i < 3; i++) {
            if (targets[i].valid) {
                can_frame frame;
                frame.can_id = 0x200 + i; // Base ID + target index
                frame.can_dlc = 8;
                
                // Pack target data into CAN frame
                frame.data[0] = (targets[i].x >> 8) & 0xFF;
                frame.data[1] = targets[i].x & 0xFF;
                frame.data[2] = (targets[i].y >> 8) & 0xFF;
                frame.data[3] = targets[i].y & 0xFF;
                frame.data[4] = (targets[i].distance >> 8) & 0xFF;
                frame.data[5] = targets[i].distance & 0xFF;
                frame.data[6] = (targets[i].speed >> 8) & 0xFF;
                frame.data[7] = targets[i].speed & 0xFF;
                
                mcp.sendMessage(&frame);
            }
        }
    }
}

void printStatus() {
    Serial.println("=== C.A.R.E. Status ===");
    Serial.println("Emergency Stop: " + String(safety_zone.emergency_stop ? "ACTIVE" : "INACTIVE"));
    Serial.println("Safety Zone: " + String(safety_zone.min_distance) + "mm, ±" + String(safety_zone.max_angle) + "°");
    
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            Serial.println("Target " + String(i+1) + ":");
            Serial.println("  X: " + String(targets[i].x) + "mm");
            Serial.println("  Y: " + String(targets[i].y) + "mm");
            Serial.println("  Distance: " + String(targets[i].distance) + "mm");
            Serial.println("  Speed: " + String(targets[i].speed) + "cm/s");
            Serial.println("  Angle: " + String(targets[i].angle) + "°");
        }
    }
    Serial.println("======================");
}
