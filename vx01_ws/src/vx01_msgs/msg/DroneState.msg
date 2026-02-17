# Header
std_msgs/Header header

# Connection
bool connected
bool armed
string flight_mode      # STABILIZE, GUIDED, LOITER etc

# Battery
float64 battery_voltage
float64 battery_percentage

# Local position
float64 local_x
float64 local_y
float64 local_z

# Altitude
float64 altitude

# GPS
float64 latitude
float64 longitude
bool gps_fixed

# Motor state
float64[4] motor_thrusts  # 0.0 to 1.0
float64[4] arm_angles     # Arm servo angles (radians)