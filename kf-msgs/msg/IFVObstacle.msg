# kuafu ifv obstacle output structure info
# ifv module
# a single obstacle

# obstacle id
uint8 track_id

# obstacle 3d position, [x,y,z]
# z is not used but reserved
geometry_msgs/Point position

# corner points
# z is not used but reserved
geometry_msgs/Point[] corners

# box heading
float64 heading

# velocity
float64 velocity

# velocity heading
float64 velocity_heading

# number of observed age
uint8 num_age

# obstacle type
# fillin: what does it contain? accomplish it as direction_status in PlanningFrame.msg  0:Unkonw 1:Veh 2:Ped
uint8 type