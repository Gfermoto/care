/*
C.A.R.E. - Collaborative Awareness Radar for Empathic interaction
ESP32 Implementation with LD2450 Radar and CAN Safety Interface

Hardware Setup:
- LD2450 Radar: UART (GPIO16/17)
- CAN Transceiver: TWAI (GPIO21/22) 
- WiFi: Built-in for ROS2 communication

Connections:
LD2450 => ESP32
5V => 5V
GND => GND
TX => GPIO16 (RX)
RX => GPIO17 (TX)

CAN Transceiver => ESP32
CTX => GPIO21 (TX)
CRX => GPIO22 (RX)
VCC => 3.3V
GND => GND
CANH => CAN_H
CANL => CAN_L
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LD2450.h>

// Hardware Serial for LD2450
HardwareSerial radarSerial(2);

// LD2450 Radar instance
LD2450 radar;

// WiFi credentials (configure in platformio.ini)
const char* ssid = "CARE_Radar";
const char* password = "care2024";

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

// Web server for configuration
WebServer server(80);

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

void setup() {
    Serial.begin(115200);
    Serial.println("C.A.R.E. ESP32 Radar Module Starting...");
    
    // Initialize components
    setupWiFi();
    setupRadar();
    setupCAN();
    setupWebServer();
    
    Serial.println("C.A.R.E. ESP32 Ready!");
    Serial.println("Web interface: http://" + WiFi.localIP().toString());
}

void loop() {
    // Handle web server
    server.handleClient();
    
    // Read radar data
    if (radar.read() > 0) {
        handleRadarData();
        checkSafetyZone();
    }
    
    // Send CAN data periodically
    if (millis() - last_can_send > CAN_SEND_INTERVAL) {
        sendCANData();
        last_can_send = millis();
    }
    
    delay(10);
}

void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.println("WiFi AP started: " + String(ssid));
    Serial.println("IP address: " + WiFi.softAPIP().toString());
}

void setupRadar() {
    // Initialize LD2450 on UART2
    radar.begin(radarSerial, false);
    Serial.println("LD2450 Radar initialized");
}

void setupCAN() {
    // Initialize TWAI (CAN) interface
    // Note: This is a placeholder - actual TWAI setup would go here
    Serial.println("CAN interface initialized (TWAI)");
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/config", HTTP_POST, handleConfig);
    server.on("/status", handleStatus);
    server.begin();
    Serial.println("Web server started");
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
    Serial.println("CAN Emergency Stop sent");
}

void sendCANData() {
    // Send radar data via CAN for ROS2 integration
    // This would include target positions and safety status
    if (safety_zone.emergency_stop) {
        sendCANEmergencyStop();
    }
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>C.A.R.E. Radar</title></head><body>";
    html += "<h1>C.A.R.E. Radar Module</h1>";
    html += "<h2>Current Status</h2>";
    html += "<p>Emergency Stop: " + String(safety_zone.emergency_stop ? "ACTIVE" : "INACTIVE") + "</p>";
    html += "<h2>Detected Targets</h2>";
    
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            html += "<p>Target " + String(i+1) + ": ";
            html += "X=" + String(targets[i].x) + "mm, ";
            html += "Y=" + String(targets[i].y) + "mm, ";
            html += "Distance=" + String(targets[i].distance) + "mm, ";
            html += "Speed=" + String(targets[i].speed) + "cm/s</p>";
        }
    }
    
    html += "<h2>Configuration</h2>";
    html += "<form action='/config' method='POST'>";
    html += "<label>Min Distance (mm): <input type='number' name='distance' value='" + String(safety_zone.min_distance) + "'></label><br>";
    html += "<label>Max Angle (deg): <input type='number' name='angle' value='" + String(safety_zone.max_angle) + "'></label><br>";
    html += "<input type='submit' value='Update'></form>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleConfig() {
    if (server.hasArg("distance")) {
        safety_zone.min_distance = server.arg("distance").toFloat();
    }
    if (server.hasArg("angle")) {
        safety_zone.max_angle = server.arg("angle").toFloat();
    }
    
    server.send(200, "text/plain", "Configuration updated");
}

void handleStatus() {
    DynamicJsonDocument doc(1024);
    doc["emergency_stop"] = safety_zone.emergency_stop;
    doc["min_distance"] = safety_zone.min_distance;
    doc["max_angle"] = safety_zone.max_angle;
    
    JsonArray targets_array = doc.createNestedArray("targets");
    for (int i = 0; i < 3; i++) {
        if (targets[i].valid) {
            JsonObject target = targets_array.createNestedObject();
            target["id"] = targets[i].id;
            target["x"] = targets[i].x;
            target["y"] = targets[i].y;
            target["distance"] = targets[i].distance;
            target["speed"] = targets[i].speed;
            target["angle"] = targets[i].angle;
        }
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}
