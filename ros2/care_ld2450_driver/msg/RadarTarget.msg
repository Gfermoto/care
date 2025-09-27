# C.A.R.E. Radar Target Message
# Represents a single radar target detected by LD2450

uint16 id                    # Target ID (1-3)
int16 x                      # X position in mm
int16 y                      # Y position in mm
int16 speed                  # Speed in cm/s
uint16 distance              # Distance in mm
float32 angle                # Angle in degrees
bool valid                   # Target validity
builtin_interfaces/Time timestamp  # Detection timestamp
