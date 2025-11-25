# kuafu traffic light info
# ifv module

# traffic light color
uint8 color
uint8 color_unknown = 0
uint8 color_red = 1
uint8 color_green = 2
uint8 color_yellow = 3

# traffic light direction
# using AND operation to represent combinations
uint8 direction
uint8 direction_unknown = 0
uint8 direction_straight = 1
uint8 direction_left = 2
uint8 direction_right = 4
uint8 direction_uturn = 8

# traffic light status
uint8 status
uint8 status_unknown = 0
uint8 status_on = 1
uint8 status_off = 2
uint8 status_blink = 3       # blink from green to red
uint8 status_long_blink = 4  # blink caution
