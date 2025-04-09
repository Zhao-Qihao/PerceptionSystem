cmake_minimum_required(VERSION 3.0.2)
project(autoware_lidar_bevfusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_language(CUDA)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
# Find catkin and other dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  tf
  tf_conversions
)
find_package(PCL REQUIRED COMPONENTS common io)
add_definitions(-DTV_CUDA)  # 启用 TensorView 的 CUDA 支持

# Set CUDA-related options
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
option(CUDA_AVAIL "CUDA available" OFF)

# Check for CUDA availability
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.3")
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# Check for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)

# 手动指定 TensorRT 库路径（如果 find_library 未找到）
set(NVINFER /usr/local/TensorRT-10.7.0.23/lib/libnvinfer.so)
set(NVONNXPARSER /usr/local/TensorRT-10.7.0.23/lib/libnvonnxparser.so)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# Check for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so libcudnn.dylib
  PATHS $ENV{LD_LIBRARY_PATH} /usr/lib /usr/local/lib
  PATH_SUFFIXES lib lib64
  DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Check for spconv availability
option(SPCONV_AVAIL "spconv available" OFF)
find_package(cumm)
find_package(spconv)
if(${cumm_FOUND} AND ${spconv_FOUND})
  message("spconv is available!")
  set(SPCONV_AVAIL ON)
else()
  message("spconv is NOT Available")
  set(SPCONV_AVAIL OFF)
endif()

# Build only if all dependencies are available
if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL AND SPCONV_AVAIL)
  # Declare catkin package
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib ${PROJECT_NAME}_cuda_lib
    CATKIN_DEPENDS roscpp sensor_msgs
  )

  # Include directories
  include_directories(
    include
    include/autoware
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    /usr/local/TensorRT-10.7.0.23/include  # 添加 TensorRT 头文件路径
  )

  # 添加 TensorRT 库路径
  link_directories(/usr/local/TensorRT-10.7.0.23/lib)

  # Add compile options
  add_compile_options(-Wno-deprecated-declarations)

  # Debug flags for CUDA
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -g -G")
  endif()

  # Find Eigen3
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)

  # Add CUDA NVCC flags
  list(APPEND CUDA_NVCC_FLAGS
  "-gencode=arch=compute_89,code=sm_89"  # 添加对 SM 89 的支持
  "--expt-relaxed-constexpr"              # 原有标志保留
  "-diag-suppress 1675"
  "--extended-lambda"
  )

  # CUDA libraries
  cuda_add_library(bev_ops SHARED
    src/bev_ops/bev_pool_cuda.cu
  )

  add_library(autoware_tensorrt_plugins SHARED
    src/tensorrt_plugins/plugin_utils.cpp
    src/tensorrt_plugins/quick_cumsum_cuda_plugin.cpp
    src/tensorrt_plugins/quick_cumsum_cuda_plugin_creator.cpp
    src/tensorrt_plugins/get_indices_pairs_implicit_gemm_plugin_creator.cpp
    src/tensorrt_plugins/get_indices_pairs_implicit_gemm_plugin.cpp
    src/tensorrt_plugins/implicit_gemm_plugin_creator.cpp
    src/tensorrt_plugins/implicit_gemm_plugin.cpp
    src/tensorrt_plugins/plugin_registration.cpp
  )

  target_compile_definitions(autoware_tensorrt_plugins PRIVATE _GLIBCXX_USE_CXX11_ABI=1)
  target_link_libraries(autoware_tensorrt_plugins ${NVINFER} CUDA::cudart bev_ops spconv::spconv)

  cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
    lib/postprocess/circle_nms_kernel.cu
    lib/postprocess/postprocess_kernel.cu
    lib/preprocess/preprocess_kernel.cu
  )

  target_link_libraries(${PROJECT_NAME}_cuda_lib spconv::spconv)
  # target_include_directories(${PROJECT_NAME}_cuda_lib SYSTEM PUBLIC ${autoware_cuda_utils_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}_lib SHARED
    # lib/detection_class_remapper.cpp
    # lib/postprocess/non_maximum_suppression.cpp
    lib/preprocess/voxel_generator.cpp
    lib/preprocess/pointcloud_densification.cpp
    lib/preprocess/precomputed_features.cpp
    # lib/ros_utils.cpp
    lib/bevfusion_trt.cpp
    lib/tensorrt_commom/tensorrt_common.cpp
    lib/tensorrt_commom/profiler.cpp
  )

  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR})
  set(NVINFER_PLUGIN /usr/local/TensorRT-10.7.0.23/lib/libnvinfer_plugin.so)
  target_link_libraries(${PROJECT_NAME}_lib
    ${NVINFER}
    ${NVINFER_PLUGIN}
    /usr/local/TensorRT-10.7.0.23/lib/libnvonnxparser.so
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    /usr/local/cuda-12.3/targets/x86_64-linux/lib/libnvrtc.so
    ${PROJECT_NAME}_cuda_lib
    ${catkin_LIBRARIES}
    stdc++fs
  )

  add_executable(${PROJECT_NAME}_node src/lidar_bevfusion_node.cpp)
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
  

  install(TARGETS ${PROJECT_NAME}_cuda_lib
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(TARGETS autoware_tensorrt_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/plugins
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/plugins
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/plugins
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

else()
  message(WARNING "Dependencies not met, building without CUDA/TensorRT components")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib ${PROJECT_NAME}_cuda_lib
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

# 为目标设置 C++17 标准
target_compile_features(${PROJECT_NAME}_lib PRIVATE cxx_std_17)

# 为目标设置 C++17 标准
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)