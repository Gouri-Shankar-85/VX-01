# Header
std_msgs/Header header

# Current operating mode
uint8 mode

# Mode constants
uint8 MODE_GROUND=0       # Hexapod walking
uint8 MODE_AIR=1          # Drone flight
uint8 MODE_WATER=2        # Underwater
uint8 MODE_TRANSITION=3   # Switching between modes

# Transition info
uint8 from_mode
uint8 to_mode
float64 transition_progress  # 0.0 to 1.0

# Mode specific flags
bool hexapod_active
bool drone_active
bool underwater_active