# Standard Header (CRITICAL for TF/Time sync)
std_msgs/Header header

# Logic Data
string label       # Semantic Class (e.g., "gate", "buoy")
int32 id           # Unique tracking ID (e.g., Tracker ID 1)

# Standard Geometry
# Replaces x, y, z, theta_z. Allows direct use with TF2 and Planner
geometry_msgs/Pose pose

# Domain Specifics
bool has_orientation             # True if orientation was explicitly calculated
geometry_msgs/Vector3 size       # Dimensions (x, y, z)
float64 confidence               # 0.0 to 1.0
