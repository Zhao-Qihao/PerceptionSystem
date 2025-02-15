cmake_minimum_required(VERSION 3.0.2)
project(center_point)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  autoware_msgs
  tf
  roslib
)

catkin_package(
)

include_directories(
  include/center_point
  ${catkin_INCLUDE_DIRS}
)


find_package(CUDA REQUIRED)
if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")
  set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
  set(CUDA_INSTALL_TARGET_DIR targets/aarch64-linux)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CMAKE_C_COMPILER /usr/bin/gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
  set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
endif()

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -Wextra -Wall -Wno-deprecated-declarations -O3")
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode arch=compute_87,code=compute_87)

set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_compile_options(-W)
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  3rdparty/libspconv/include
  include
  lib/include
)

link_directories(
  ${TENSORRT_LIB}
  3rdparty/libspconv/lib/x86_64
  /usr/lib/aarch64-linux-gnu
  /usr/aarch64-linux-gnu/lib/
)

file(GLOB_RECURSE SOURCE_FILES 
  lib/src/*.cu 
  lib/src/*.cpp
)

cuda_add_library(gpu SHARED ${SOURCE_FILES})

target_link_LIBraries(gpu
    libnvinfer.so
    libspconv.so   
)

add_executable(center_point src/center_point.cpp src/center_point_node.cpp)

target_link_libraries(center_point gpu libspconv.so libnvinfer.so  ${catkin_LIBRARIES} ${PCL_LIBRARIES})