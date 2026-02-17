# Header
std_msgs/Header header

# Joint states (18 joints)
float64[18] joint_positions
float64[18] joint_velocities

# Body pose
geometry_msgs/Pose body_pose

# Gait info
string gait_mode        # STAND, WALK, TURN, STOP
int32 gait_phase        # Current phase (0-5)
bool is_walking

# Foot contact
bool[6] foot_contact    # True if foot on ground

# End effector positions
geometry_msgs/Point[6] foot_positions