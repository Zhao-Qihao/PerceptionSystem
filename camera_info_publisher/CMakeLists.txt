cmake_minimum_required(VERSION 3.0.2)
project(camera_info_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
target_link_libraries(camera_info_publisher ${catkin_LIBRARIES})