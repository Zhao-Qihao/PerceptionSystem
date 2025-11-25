# kuafu lidar obstacle output structure info
# lidar detection module

# number of observed age
uint8 num_age

# acceleration
float64 a
# velocity in x direction
float64 vel_x
# velocity in y direction
float64 vel_y

# obstacle 3d position, [x,y,z]
# z is not used but reserved
geometry_msgs/Point position

# corner points
# z is not used but reserved
geometry_msgs/Point[] corners

# obstacle type
uint8 type
uint8 type_unknown=0    # others
uint8 type_veh=1        # ca
uint8 type_cyc=2        # bike
uint8 type_ped=3        # person

# motion state
uint8 motion_type
uint8 motion_type_unknown=0    # unkown motion type
uint8 motion_type_static=1     # static
uint8 motion_type_moving=2     # moving
uint8 motion_type_too_low=3    # height difference too small
uint8 motion_type_few_pts=4    # obstacle only with few points
uint8 motion_type_unmatch=5    # current obstacle has no correspondence within a range gating in previous frame
uint8 motion_type_bad_icp=6    # icp results low quality

# tracking id
uint8 track_id
