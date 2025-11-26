cmake_minimum_required(VERSION 3.1)
project(tensorrt_yolotfl)

enable_language(CUDA)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # OpenCV
  cv_bridge
  image_transport
)

catkin_package(
)


set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wno-write-strings -Wall")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O0 -g -Wno-write-strings")
#set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -g")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")

add_compile_options(-D YOLOX_STANDALONE)

set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.3")
find_package(CUDA REQUIRED)
message("-- USING ACCELERATED CLUSTERING --")
message("Version: " ${CUDA_VERSION})
message("Runtime: " ${CUDA_CUDART_LIBRARY})
message("Library: " ${CUDA_CUDA_LIBRARY})
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/cuda/include
  include
  src
  extra/
  modules/
)
find_library(SYSTEM_LIBCURL NAMES curl PATHS /usr/lib/x86_64-linux-gnu)
# find_library(SYSTEM_LIBNGHTTP2 NAMES nghttp2 PATHS /usr/lib/x86_64-linux-gnu /usr/local/lib /lib/x86_64-linux-gnu)

link_directories(/usr/local/cuda/lib64)

# 指定TensorRT的路径
set(TensorRT_DIR "/usr/local/TensorRT-8.6.1.6")
# 添加TensorRT的include目录
include_directories(${TensorRT_DIR}/include)
# 添加TensorRT的lib目录到链接器路径
link_directories(${TensorRT_DIR}/lib)


# detector lib (cuda)
file(GLOB_RECURSE sources lib/tensorrt_common/tensorrt_common.cpp lib/tensorrt_common/simple_profiler.cpp lib/tensorrt_yolox/tensorrt_yolox.cpp lib/tensorrt_yolox/preprocess.cu)
add_library(detector SHARED ${sources})

target_link_libraries(detector
  ${catkin_LIBRARIES}
  nvinfer
  nvinfer_plugin
  nvparsers
  nvonnxparser
  cuda
  cudart
  "stdc++fs"
  ${Boost_LIBRARIES}
  ${SYSTEM_LIBCURL}
  # ${SYSTEM_LIBNGHTTP2}
  dl
)

target_include_directories(detector PRIVATE
  extra/
  modules/
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_TOOLKIT_ROOT_DIR}/include
)

# executable
add_executable(tensorrt_yolotfl_node src/yolox.cpp src/yolox_node.cpp src/yolo_config_parser.cpp ${sources})
# add_executable(tensorrt_yolotfl src/main_node.cpp src/yolo_config_parser.cpp ${sources})

target_include_directories(tensorrt_yolotfl_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(tensorrt_yolotfl_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  nvinfer
  nvinfer_plugin
  nvparsers
  nvonnxparser
  cuda
  cudart
  gflags
  ${Boost_LIBRARIES}
  ${SYSTEM_LIBCURL}
  # ${SYSTEM_LIBNGHTTP2}
  "stdc++fs"
  "stdc++"
  dl
)

# Install rules
install(TARGETS tensorrt_yolotfl_node detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
