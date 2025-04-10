#ifndef AUTOWARE__LIDAR_BEVFUSION__ROS_UTILS_HPP_
#define AUTOWARE__LIDAR_BEVFUSION__ROS_UTILS_HPP_

#include "autoware/lidar_bevfusion/preprocess/point_type.hpp"
#include "autoware/lidar_bevfusion/utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
// #include <autoware_perception_msgs/msg/detected_objects.hpp>
// #include <autoware_perception_msgs/msg/object_classification.hpp>
// #include <autoware_perception_msgs/msg/shape.hpp>
// #include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <string>
#include <vector>

// Define InputPointType
struct InputPointType
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

// Field checking macros
#define CHECK_OFFSET(structure1, structure2, field)             \
  static_assert(                                                \
    offsetof(structure1, field) == offsetof(structure2, field), \
    "Offset of " #field " in " #structure1 " does not match the one in " #structure2 ".")

#define CHECK_TYPE(structure1, structure2, field)                             \
  static_assert(                                                              \
    std::is_same_v<decltype(structure1::field), decltype(structure2::field)>, \
    "Type of " #field " in " #structure1 " and " #structure2 " have different types.")

#define CHECK_FIELD(structure1, structure2, field) \
  CHECK_OFFSET(structure1, structure2, field);     \
  CHECK_TYPE(structure1, structure2, field)

namespace autoware::lidar_bevfusion
{
using sensor_msgs::PointField;

// Check field consistency (optional)
CHECK_FIELD(InputPointType, InputPointType, x);
CHECK_FIELD(InputPointType, InputPointType, y);
CHECK_FIELD(InputPointType, InputPointType, z);
CHECK_FIELD(InputPointType, InputPointType, intensity);

// void box3DToDetectedObject(
//   const Box3D & box3d, const std::vector<std::string> & class_names,
//   autoware_perception_msgs::DetectedObject & obj);

uint8_t getSemanticType(const std::string & class_name);

}  // namespace autoware::lidar_bevfusion

#endif  // AUTOWARE__LIDAR_BEVFUSION__ROS_UTILS_HPP_