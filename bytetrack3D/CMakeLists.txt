cmake_minimum_required(VERSION 2.8.3)
project(bytetrack3D)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  autoware_msgs
  OpenCV
  pluginlib
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(Boost_DIR "/usr/lib/x86_64-linux-gnu/cmake/boost")  # 根据实际路径调整
find_package(Boost REQUIRED)

set(YAML_CPP_DIR "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp")  # 替换为实际路径
find_package(yaml-cpp REQUIRED HINTS ${YAML_CPP_DIR})

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs
    pluginlib
)
#
# Core library
#
file(GLOB bytetrack3D_lib_src
  "lib/src/*.cpp"
  "lib/src/kalman_filter/*.cpp"
  "src/*.cpp"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  lib/include
  )

add_library(bytetrack3D_lib SHARED ${bytetrack3D_lib_src})
target_link_libraries(bytetrack3D_lib ${catkin_LIBRARIES} yaml-cpp)

target_include_directories(bytetrack3D_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/include>
  $<INSTALL_INTERFACE:include>
)

#
# ROS node
#
# add_library(${PROJECT_NAME} SHARED src/bytetrack3D.cpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/bytetrack3D_node.cpp)
target_link_libraries(${PROJECT_NAME}_node bytetrack3D_lib  ${catkin_LIBRARIES})


install(TARGETS bytetrack3D_node bytetrack3D_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)