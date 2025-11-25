# kuafu ifv frame output structure info
# contains obstacles and traffic lights
# ifv module

# header info 
std_msgs/Header header

# ifv obstacles
kf_msgs/IFVObstacle[] obstacles

# ifv tracffic light detection
kf_msgs/TrafficLight left
kf_msgs/TrafficLight middle
kf_msgs/TrafficLight right