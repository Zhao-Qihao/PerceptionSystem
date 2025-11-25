# kuafu fusion frame structure info
# detection fusion module

# header info
std_msgs/Header header

# whether the obstacle is valid
bool is_ok

# fused obstacles info
kf_msgs/FusionObstacle[] obstacles

# drivable info
kf_msgs/DrivableZone drivable_zone

# grid map info
nav_msgs/OccupancyGrid gridmap