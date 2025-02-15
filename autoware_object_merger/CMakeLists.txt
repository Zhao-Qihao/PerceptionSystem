# cmake_minimum_required(VERSION 2.8.3)
# project(autoware_object_merger)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   autoware_msgs
#   tf2_geometry_msgs
#   geometry_msgs
#   OpenCV
# )

# find_package(Eigen3 REQUIRED)

# include_directories(
#   SYSTEM
#   include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )

# add_library(${PROJECT_NAME}
#   src/association/data_association.cpp
#   # src/association/solver/mu_successive_shortest_path_wrapper.cpp
#   src/object_association_merger_node.cpp
# )

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   Eigen3::Eigen
#   ${OpenCV_LIBS}
# )

# add_executable(object_association_merger_node
#   src/object_association_merger_node.cpp
# )

# target_link_libraries(object_association_merger_node
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   Eigen3::Eigen
#   ${OpenCV_LIBS}
# )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp std_msgs autoware_msgs tf2_geometry_msgs
#   DEPENDS Eigen3 OpenCV
# )

# install(TARGETS ${PROJECT_NAME} object_association_merger_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )