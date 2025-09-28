# C.A.R.E. Radar Targets Array Message
# Массив всех обнаруженных целей от одного устройства

# Header information
std_msgs/Header header       # Timestamp and frame_id
uint8 device_id             # Source device ID (1-15)

# Target data
RadarTarget[] targets       # Array of detected targets (max 3 for LD2450)
uint8 target_count         # Number of valid targets in array

# Device status
uint8 device_status         # Device status flags (from CAN protocol)
uint8 radar_status          # LD2450 specific status
float32 update_rate         # Current update frequency (Hz)

# Safety summary
uint8 targets_in_safety_zone  # Number of targets in safety zone
bool emergency_detected       # Any target triggers emergency condition
float32 closest_distance      # Distance to closest target (mm)
