# final obstacle structure info
# detection fusion module

# tracked num of frames
uint8 num_age

float64 a
float64 vel_x
float64 vel_y

# obstacle 3d position, [x,y,z]
# z is not used but reserved
geometry_msgs/Point position

# corner points
# z is not used but reserved
geometry_msgs/Point[] corners

# obstacle type
# fillin: what does it contain? accomplish it as direction_status in PlanningFrame.msg
uint8 type

# motion state
# fillin: what does it contain? accomplish it as direction_status in PlanningFrame.msg 0:static 1:dynamic
uint8 motion_type

# tracking id
uint64 track_id
