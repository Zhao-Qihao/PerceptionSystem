# kuafu lidar localization output structure info
# lidar localization module

# header info
std_msgs/Header header

# pose
# east-north coordinate
geometry_msgs/Pose pose

# overlap ratio of current cloud with respect to the cloudmap
float64 ratio_cloud_wrt_map

# position covariance
geometry_msgs/Point cov
