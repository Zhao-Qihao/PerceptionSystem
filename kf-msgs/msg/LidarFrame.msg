# kuafu lidar frame output structure info
# lidar detection module

# header info
std_msgs/Header header

bool flag

# lidar obstacles info
kf_msgs/LidarObstacles obstacles

# drivable info
kf_msgs/DrivableZone drivable_zone

# grid map info
nav_msgs/OccupancyGrid gridmap