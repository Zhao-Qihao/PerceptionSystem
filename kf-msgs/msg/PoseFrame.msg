# kuafu pose output structure info
# localization module

# header info
std_msgs/Header Header

# local info
geometry_msgs/Point local_pos
float64 local_speed
float64 local_head

# global info
geometry_msgs/Point gps_pos
geometry_msgs/Point gps_vel
geometry_msgs/Point gps_acc
float64 gps_speed
geometry_msgs/Point gps_head
geometry_msgs/Point gps_headrate
geometry_msgs/Point gps_accuracy

# gps flag
uint8 gps_flag
uint8 gps_flag_idle = 0             # not start
uint8 gps_flag_aligning = 1
uint8 gps_flag_low_accuracy = 2
uint8 gps_flag_high_accuracy = 3
uint8 gps_flag_gps_not_valid = 6    # ex. in the tunnel
uint8 gps_flag_aligned = 7
