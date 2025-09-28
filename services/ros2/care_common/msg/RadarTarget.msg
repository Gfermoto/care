# C.A.R.E. Radar Target Message
# Консистентная структура цели для всех компонентов системы
# Соответствует CAN протоколу и структурам ESP32/STM32

# Target identification
uint16 id                    # Target ID (0-2 for LD2450)
uint8 device_id             # Source device ID (1-15)

# Position data (from LD2450)
int16 x                     # X coordinate in mm (signed, ±6000mm range)
int16 y                     # Y coordinate in mm (signed, ±6000mm range)
uint16 distance             # Distance from sensor in mm (0-8000mm range)
float32 angle               # Angle in degrees (calculated from x,y)

# Motion data
int16 speed                 # Speed in cm/s (signed, ±200 cm/s range)

# Validity and timing
bool valid                  # Target validity flag
builtin_interfaces/Time timestamp  # Detection timestamp
builtin_interfaces/Time last_seen   # Last update timestamp

# Quality indicators
float32 confidence          # Detection confidence (0.0-1.0)
uint8 tracking_age         # How long target has been tracked (frames)

# Safety classification
bool in_safety_zone        # Is target in configured safety zone
float32 safety_distance    # Distance to safety zone boundary (mm)
