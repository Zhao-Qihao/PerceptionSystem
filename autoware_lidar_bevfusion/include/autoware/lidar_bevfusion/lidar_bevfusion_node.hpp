// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIDAR_BEVFUSION__LIDAR_BEVFUSION_NODE_HPP_
#define LIDAR_BEVFUSION__LIDAR_BEVFUSION_NODE_HPP_

#include "autoware/lidar_bevfusion/bevfusion_trt.hpp"
// #include "autoware/lidar_bevfusion/detection_class_remapper.hpp"
// #include "autoware/lidar_bevfusion/iou_bev_nms.hpp"
#include "autoware/lidar_bevfusion/utils.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <memory>
#include <string>
#include <vector>

namespace autoware::lidar_bevfusion
{

class LidarBEVFusionNode
{
public:
  using Matrix4f = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
  explicit LidarBEVFusionNode();

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg, const int64_t camera_id);
  void cameraInfoCallback(const sensor_msgs::CameraInfo& msg, const int64_t camera_id);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber cloud_sub_;
  std::vector<ros::Subscriber> image_subs_;
  std::vector<ros::Subscriber> camera_info_subs_;

  ros::Publisher objects_pub_;

  std::unique_ptr<BEVFusionTRT> detector_ptr_;
  // DetectionClassRemapper detection_class_remapper_;
  // IouBevNms iou_bev_nms_;

  std::vector<std::string> class_names_;
  std::string lidar_frame_;
  bool sensor_fusion_{false};
  float max_camera_lidar_delay_{0.0f};

  std::vector<sensor_msgs::ImageConstPtr> image_msgs_;
  std::vector<sensor_msgs::CameraInfo> camera_info_msgs_;
  std::vector<Matrix4f> lidar2camera_extrinsics_;
  std::vector<float> camera_masks_;

  bool images_available_{false};
  bool intrinsics_available_{false};
  bool extrinsics_available_{false};
  bool intrinsics_extrinsics_precomputed_{false};
};

}  // namespace autoware::lidar_bevfusion

#endif  // LIDAR_BEVFUSION__LIDAR_BEVFUSION_NODE_HPP_
