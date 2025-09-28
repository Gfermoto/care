# C.A.R.E. Safety Command Message
# Команды безопасности для контроллеров

# Header
std_msgs/Header header

# Command types
uint8 COMMAND_SLOWDOWN = 1      # Activate slowdown zone
uint8 COMMAND_STOP = 2          # Emergency stop
uint8 COMMAND_RESUME = 3        # Resume operation
uint8 COMMAND_CLEAR_SLOWDOWN = 4 # Clear slowdown state
uint8 COMMAND_RESET = 5         # Reset emergency state
uint8 COMMAND_TEST = 6          # Test emergency systems
uint8 COMMAND_CONFIGURE = 7     # Update safety configuration

uint8 command_type              # Command to execute
uint8 target_device_id          # Target device (0 = all devices)
uint8 priority                  # Command priority (0-255, higher = more urgent)

# Command parameters
string command_data             # Additional command parameters (JSON format)
builtin_interfaces/Duration timeout  # Command timeout
uint32 sequence_number          # Command sequence for tracking

# Source information
uint8 source_node_id            # Node that issued command
string source_node_name         # Name of source node
builtin_interfaces/Time issued_time  # When command was issued

# Acknowledgment requirements
bool require_ack                # Does command require acknowledgment
builtin_interfaces/Duration ack_timeout  # Acknowledgment timeout
