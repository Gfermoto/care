# C.A.R.E. Safety Zone Configuration Message
# Определение зон безопасности для системы

# Header
std_msgs/Header header

# Zone geometry
float32 min_distance        # Minimum safe distance in mm
float32 max_distance        # Maximum detection distance in mm
float32 horizontal_angle    # Horizontal angle coverage in degrees (±angle/2)
float32 vertical_angle      # Vertical angle coverage in degrees (±angle/2)

# Dual zone configuration
float32 slowdown_distance   # Distance for slowdown zone in mm (> critical_distance)
float32 critical_distance   # Distance for emergency stop zone in mm

# Zone center and orientation
geometry_msgs/Point center  # Zone center position
geometry_msgs/Quaternion orientation  # Zone orientation

# Safety parameters
float32 warning_distance    # Warning threshold distance in mm
float32 hysteresis         # Hysteresis band to prevent oscillation in mm

# Zone status
bool zone_active           # Is this zone currently active
uint8 violation_level      # Current violation level (0=none, 1=warning, 2=critical)
uint8 targets_in_zone      # Number of targets currently in zone

# Timing parameters
builtin_interfaces/Duration min_trigger_time  # Minimum time in zone to trigger
builtin_interfaces/Duration max_clear_time    # Maximum time to clear violation

# Zone identification
uint8 zone_id              # Unique zone identifier
string zone_name           # Human-readable zone name
uint8 priority             # Zone priority (higher = more critical)
