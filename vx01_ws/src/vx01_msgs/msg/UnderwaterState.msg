# Header
std_msgs/Header header

# Water detection
bool is_underwater
float64 water_depth        # meters from surface

# Pressure sensor
float64 pressure           # hPa
float64 water_temperature  # celsius

# Buoyancy
float64 buoyancy_force     # Newtons
bool buoyancy_stable

# Seabed distance
float64 seabed_distance    # meters (from TFMini-S)

# Underwater locomotion
string locomotion_mode     # SWIM, WALK_SEABED, HOVER
float64[4] thruster_pwm    # BLDC thruster values