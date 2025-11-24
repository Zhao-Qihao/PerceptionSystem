cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_preprocess)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.8 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_preprocess
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)


add_executable(pointcloud_preprocess src/pointcloud_preprocess.cpp)
target_link_libraries(pointcloud_preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)