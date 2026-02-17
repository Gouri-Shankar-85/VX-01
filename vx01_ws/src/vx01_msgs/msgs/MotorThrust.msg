# Header
std_msgs/Header header

# Attitude commands
float64 roll      # radians
float64 pitch     # radians
float64 yaw       # radians

# Thrust
float64 thrust    # 0.0 to 1.0

# Individual motor thrusts (optional)
float64[4] motor_thrusts

# Mode
uint8 control_mode
uint8 ATTITUDE_THRUST=0
uint8 VELOCITY=1
uint8 POSITION=2