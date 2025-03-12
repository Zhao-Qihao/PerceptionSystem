cmake_minimum_required(VERSION 3.0.2)
project(lidar_centerpoint)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_language(CUDA)
find_package(CUDA REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  pcl_ros
  roscpp
  tf
  tf_conversions
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib ${PROJECT_NAME}_cuda_lib
  CATKIN_DEPENDS  autoware_msgs   pcl_ros roscpp tf tf_conversions
)

# 手动设置 TensorRT 库路径
set(TENSORRT_ROOT /usr/local/TensorRT-8.4.3.1)
set(TENSORRT_INCLUDE_DIRS ${TENSORRT_ROOT}/include)
set(TENSORRT_LIB_DIRS ${TENSORRT_ROOT}/lib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  include
  include/autoware
  ${PCL_INCLUDE_DIRS} 
)

### centerpoint ###
add_library(${PROJECT_NAME}_lib SHARED
  lib/centerpoint_trt.cpp
  # lib/detection_class_remapper.cpp
  lib/utils.cpp
  # lib/ros_utils.cpp
  lib/network/network_trt.cpp
  lib/network/tensorrt_wrapper.cpp
  # lib/postprocess/non_maximum_suppression.cpp
  lib/preprocess/pointcloud_densification.cpp
  lib/preprocess/voxel_generator.cpp
  lib/tensorrt_common/tensorrt_common.cpp 
  lib/tensorrt_common/simple_profiler.cpp
)

cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
  lib/postprocess/circle_nms_kernel.cu
  lib/postprocess/postprocess_kernel.cu
  lib/network/scatter_kernel.cu
  lib/preprocess/preprocess_kernel.cu
)

target_link_libraries(${PROJECT_NAME}_lib
  ${TENSORRT_LIB_DIRS}/libnvinfer.so
  ${TENSORRT_LIB_DIRS}/libnvinfer_plugin.so
  ${TENSORRT_LIB_DIRS}/libnvonnxparser.so
  ${CUDA_LIBRARIES}
  ${CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
  ${CUDNN_LIBRARY}
  ${PROJECT_NAME}_cuda_lib
  ${catkin_LIBRARIES}
  stdc++fs 
)

# To suppress unknown-pragmas error. The root-cause is CUB library in CUDA 11.6.
# This issue was fixed by https://github.com/NVIDIA/cub/commit/7d608bf1dc14553e2fb219eabeed80b76621b6fe
target_include_directories(${PROJECT_NAME}_lib
  SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
)

## node ##
add_library(${PROJECT_NAME}_component SHARED
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_component
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_cuda_lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_component
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)