# C.A.R.E. System Status Message
# Общий статус системы и отдельных устройств

# Header
std_msgs/Header header

# System-wide status
uint8 SYSTEM_OK = 0
uint8 SYSTEM_WARNING = 1
uint8 SYSTEM_ERROR = 2
uint8 SYSTEM_EMERGENCY = 3

uint8 overall_status        # Overall system status
string status_message       # Human-readable status description

# Device statuses
DeviceInfo[] devices        # Array of all known devices
uint8 active_devices        # Number of active devices
uint8 offline_devices       # Number of offline devices

# Safety system status
bool safety_system_active   # Is safety controller running
bool emergency_stop_active  # Is emergency stop engaged
builtin_interfaces/Time last_safety_check  # Last safety evaluation

# Performance metrics
float32 system_uptime       # System uptime in seconds
float32 message_rate        # Average message processing rate (Hz)
uint32 total_targets_seen   # Total targets detected since startup
uint32 emergency_events     # Total emergency events triggered

# Resource usage
float32 cpu_usage           # System CPU usage (0.0-1.0)
float32 memory_usage        # System memory usage (0.0-1.0)
uint32 can_message_count    # Total CAN messages processed
