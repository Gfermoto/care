# C.A.R.E. Safety Status Message
# Contains safety system status and emergency stop information

bool emergency_stop          # Emergency stop active
float32 min_distance         # Minimum safe distance in mm
float32 max_angle            # Maximum safe angle in degrees
uint8 active_targets        # Number of targets in safety zone
builtin_interfaces/Time timestamp  # Status timestamp
