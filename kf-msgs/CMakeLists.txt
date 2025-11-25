cmake_minimum_required(VERSION 2.8.3)
project(kf_msgs)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "DEBUG")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  message_generation
)

add_message_files(
  FILES
  DrivableZone.msg
  FusionFrame.msg
  FusionObstacle.msg
  FusionObstacles.msg
  IFVFrame.msg
  IFVObstacle.msg
  IFVObstacles.msg
  LidarFrame.msg
  LidarObstacle.msg
  LidarObstacles.msg
  LidarPoseFrame.msg
  NaviPoint.msg
  PlanningFrame.msg
  PoseFrame.msg
  TrafficLight.msg
  TrafficLights.msg
  TrajPoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY msg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)