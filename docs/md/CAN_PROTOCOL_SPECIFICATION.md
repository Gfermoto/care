# C.A.R.E. CAN Protocol Specification v2.0

## 🚌 **УПРОЩЕННЫЙ CAN ПРОТОКОЛ**

### **🎯 ПРИНЦИПЫ:**
- **Контроллеры = только датчики** (передают данные)
- **ROS2 = принимает решения** (включая emergency stop)
- **Масштабируемость** (множественные контроллеры)
- **Эффективность** (минимум сообщений)

## 📋 **CAN MESSAGE IDS**

### **Формула расчета ID:**
```c
// Данные целей: BASE_ID + (device_id * 10) + target_id
#define CAN_TARGET_DATA_BASE_ID 0x200

// Device 1: 0x200, 0x201, 0x202 (цели 0, 1, 2)
// Device 2: 0x210, 0x211, 0x212 (цели 0, 1, 2)  
// Device 3: 0x220, 0x221, 0x222 (цели 0, 1, 2)

// Статус устройства: BASE_ID + (device_id * 10)
#define CAN_STATUS_BASE_ID 0x300

// Device 1: 0x300
// Device 2: 0x310
// Device 3: 0x320

// Safety Commands (отправляются ТОЛЬКО ROS2 системой)
#define CAN_SLOWDOWN_COMMAND_ID 0x100    // Команда замедления
#define CAN_EMERGENCY_STOP_ID   0x101    // Команда экстренной остановки
```

## 📊 **MESSAGE FORMATS**

### **1. Target Data Message**
```c
// CAN ID: 0x200 + (device_id * 10) + target_id
// DLC: 8 bytes
// Frequency: 10-20 Hz per target

struct CanTargetMessage {
    int16_t x;          // Bytes 0-1: X coordinate in mm (signed)
    int16_t y;          // Bytes 2-3: Y coordinate in mm (signed)  
    uint16_t distance;  // Bytes 4-5: Distance in mm (unsigned)
    int16_t speed;      // Bytes 6-7: Speed in cm/s (signed)
} __attribute__((packed));

// Пример упаковки:
uint8_t pack_target_message(const RadarTarget* target, uint8_t* data) {
    data[0] = (target->x >> 8) & 0xFF;      // X high byte
    data[1] = target->x & 0xFF;             // X low byte
    data[2] = (target->y >> 8) & 0xFF;      // Y high byte
    data[3] = target->y & 0xFF;             // Y low byte
    data[4] = (target->distance >> 8) & 0xFF; // Distance high byte
    data[5] = target->distance & 0xFF;      // Distance low byte
    data[6] = (target->speed >> 8) & 0xFF;  // Speed high byte
    data[7] = target->speed & 0xFF;         // Speed low byte
    return 8; // DLC
}
```

### **2. Device Status Message**
```c
// CAN ID: 0x300 + (device_id * 10)
// DLC: 4 bytes
// Frequency: 1 Hz (heartbeat)

struct CanStatusMessage {
    uint8_t device_id;      // Byte 0: Device identifier
    uint8_t active_targets; // Byte 1: Number of active targets (0-3)
    uint8_t system_status;  // Byte 2: System status flags
    uint8_t battery_level;  // Byte 3: Battery/power level (0-100%)
} __attribute__((packed));

// System status flags (byte 2):
#define STATUS_OK           0x00  // All systems normal
#define STATUS_WARNING      0x01  // Non-critical issues
#define STATUS_ERROR        0x02  // Critical errors
#define STATUS_RADAR_FAULT  0x04  // LD2450 communication error
#define STATUS_CAN_FAULT    0x08  // CAN communication issues
#define STATUS_POWER_LOW    0x10  // Low power warning
```

### **3. Safety Command Messages (ROS2 → Controllers)**

#### **3.1. Slowdown Command (0x100)**
```c
// CAN ID: 0x100
// DLC: 4 bytes
// Frequency: On demand (medium priority)

struct CanSlowdownMessage {
    uint8_t command;        // Byte 0: Slowdown command
    uint8_t source_id;      // Byte 1: Source device ID  
    uint8_t speed_limit;    // Byte 2: Speed limit (0-100%)
    uint8_t duration;       // Byte 3: Duration in seconds (0=indefinite)
} __attribute__((packed));

// Slowdown commands:
#define SLOWDOWN_ACTIVATE   0x01  // Activate speed limitation
#define SLOWDOWN_CLEAR      0x00  // Clear slowdown state
```

#### **3.2. Emergency Stop Command (0x101)**
```c
// CAN ID: 0x101
// DLC: 2 bytes
// Frequency: On demand (highest priority)

struct CanEmergencyMessage {
    uint8_t command;        // Byte 0: Emergency command
    uint8_t source_id;      // Byte 1: Source device ID
} __attribute__((packed));

// Emergency commands:
#define EMERGENCY_STOP      0x01  // Immediate stop all operations
#define EMERGENCY_CLEAR     0x00  // Clear emergency state
#define EMERGENCY_RESET     0xFF  // Reset all controllers
```

## 🔄 **MESSAGE FLOW**

### **Normal Operation:**
```
1. ESP32/STM32 reads LD2450 data via UART
2. Processes and validates target data  
3. Sends CAN messages:
   - Target data: 10-20 Hz per active target
   - Status: 1 Hz heartbeat
4. ROS2 receives and processes all data
5. Makes safety decisions centrally
```

### **Emergency Scenario:**
```
1. ROS2 care_safety_controller_node detects danger
2. Sends CAN 0x100 (Emergency Stop) to ALL controllers
3. All ESP32/STM32 controllers receive and execute stop
4. Controllers confirm by setting status to ERROR/STOPPED
5. ROS2 monitors compliance and logs event
```

## ⚙️ **CONTROLLER CONFIGURATION**

### **Настройки для ESP32/STM32:**
```yaml
can_config:
  device_id: 1              # Unique device identifier (1-15)
  bitrate: 500000           # CAN bus speed
  target_base_id: 0x200     # Base ID for target messages
  status_base_id: 0x300     # Base ID for status messages
  
radar_config:
  uart_port: 2              # UART port number
  baud_rate: 256000         # LD2450 communication speed
  max_targets: 3            # Maximum targets to track
  update_rate: 10           # Data update frequency (Hz)
```

### **ID Calculation Examples:**
```c
// Device 1 (device_id = 1):
uint16_t target_ids[3] = {0x200, 0x201, 0x202};  // Targets 0, 1, 2
uint16_t status_id = 0x300;                       // Status

// Device 2 (device_id = 2):  
uint16_t target_ids[3] = {0x210, 0x211, 0x212};  // Targets 0, 1, 2
uint16_t status_id = 0x310;                       // Status

// Device 5 (device_id = 5):
uint16_t target_ids[3] = {0x250, 0x251, 0x252};  // Targets 0, 1, 2  
uint16_t status_id = 0x350;                       // Status
```

## 🛡️ **ERROR HANDLING**

### **Missing Messages:**
- **Target timeout**: If no target data for >500ms, mark as invalid
- **Status timeout**: If no status for >2s, mark device as offline
- **Emergency timeout**: If emergency not acknowledged in 100ms, escalate

### **Invalid Data:**
- **Range checks**: X/Y within ±6000mm, distance 0-8000mm
- **Consistency**: Distance should match sqrt(x² + y²) within tolerance
- **Speed limits**: Reasonable speed values (-200 to +200 cm/s)

## 📈 **PERFORMANCE SPECIFICATIONS**

### **Bandwidth Usage:**
```
Single Controller (3 targets active):
- Target data: 3 × 8 bytes × 10 Hz = 240 bytes/s
- Status data: 1 × 4 bytes × 1 Hz = 4 bytes/s  
- Total: ~244 bytes/s per controller

10 Controllers:
- Total bandwidth: ~2.44 KB/s
- CAN utilization: <1% at 500 kbps
```

### **Latency Requirements:**
- **Target data**: <10ms from sensor to CAN
- **Emergency stop**: <5ms from ROS2 to controllers
- **Status updates**: <1s acceptable delay

## ✅ **VALIDATION CHECKLIST**

- [ ] ID ranges don't conflict (0x200-0x2FF for targets)
- [ ] Emergency ID (0x100) reserved for ROS2 only
- [ ] Status messages provide adequate diagnostics
- [ ] Message sizes fit in CAN frame (≤8 bytes)
- [ ] Frequencies appropriate for real-time operation
- [ ] Error handling covers all failure modes
- [ ] Scalable to 15+ controllers without conflicts

**CAN Protocol v2.0 Complete!** 🚌✅
