# C.A.R.E. Device Information Message
# Информация об устройстве в системе

# Device identification
uint8 device_id                 # Unique device ID (1-15)
string device_name              # Human-readable device name
string device_type              # Device type (ESP32, STM32, etc.)
string firmware_version         # Firmware version string
string hardware_revision        # Hardware revision

# Communication status
uint8 STATUS_ONLINE = 0
uint8 STATUS_WARNING = 1  
uint8 STATUS_ERROR = 2
uint8 STATUS_OFFLINE = 3

uint8 communication_status      # Current communication status
builtin_interfaces/Time last_seen  # Last message received time
float32 message_rate           # Current message rate (Hz)
uint32 messages_received       # Total messages received
uint32 messages_lost           # Total messages lost/timeout

# Device capabilities
uint8 max_targets              # Maximum targets device can track
float32 max_range              # Maximum detection range (mm)
float32 horizontal_fov         # Horizontal field of view (degrees)
float32 vertical_fov           # Vertical field of view (degrees)
float32 update_rate            # Maximum update rate (Hz)

# Current operational status
uint8 active_targets           # Current number of active targets
float32 current_range          # Current detection range (mm)
float32 current_update_rate    # Current update rate (Hz)
uint8 system_load              # Device system load (0-100%)

# Error information
uint8 error_count              # Number of errors since startup
string last_error_message      # Description of last error
builtin_interfaces/Time last_error_time  # Time of last error

# Power and health
uint8 battery_level            # Battery level (0-100%, 255 if N/A)
float32 temperature            # Device temperature (Celsius)
bool self_test_passed          # Last self-test result
