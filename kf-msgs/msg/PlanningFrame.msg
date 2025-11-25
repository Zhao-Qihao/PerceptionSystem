# kuafu planning output structure info
# planning module

# header info
std_msgs/Header header

# frame id
uint8 frame_num

float64 pose_ref_x        # x-axis of reference pose
float64 pose_ref_y        # y-axis of reference pose
float64 pose_ref_heading  # heading angle of reference pose

# array of trajectory points
kf_msgs/TrajPoint[] traj_points

# driving direction
uint8 orientation
uint8 orientation_forward = 0    
uint8 orientation_backward = 1   
uint8 orientation_side = 2      

# send to control, reflect acceleration
uint8 stop_level

uint8 aux_cmd                          # auxiliary operation command
uint8 aux_cmd_idle = 0                 # idle
uint8 aux_cmd_light_turn_left = 1      # left turn signal light
uint8 aux_cmd_light_turn_right = 2     # right turn signal light
uint8 aux_cmd_beep = 4                 # beep
uint8 aux_cmd_light_big_front = 16     # big front light
uint8 aux_cmd_light_small_front = 32   # small front light
uint8 aux_cmd_light_back = 64          # back light