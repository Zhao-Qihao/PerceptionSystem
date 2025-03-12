// Copyright 2021 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_CENTERPOINT__NODE_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__NODE_HPP_

#include "autoware/lidar_centerpoint/centerpoint_trt.hpp"
// #include "autoware/lidar_centerpoint/detection_class_remapper.hpp"
// #include "autoware/lidar_centerpoint/postprocess/non_maximum_suppression.hpp"

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::lidar_centerpoint
{

class LidarCenterPointNode
{
public:
  LidarCenterPointNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber pointcloud_sub_;
  ros::Publisher objects_pub_;

  float score_threshold_;
  float circle_nms_dist_threshold_;
  std::vector<double> yaw_norm_thresholds_;
  std::string densification_world_frame_id_;
  int densification_num_past_frames_;
  std::string trt_precision_;
  int cloud_capacity_;
  std::string encoder_onnx_path_;
  std::string encoder_engine_path_;
  std::string head_onnx_path_;
  std::string head_engine_path_;
  std::vector<std::string> class_names_;
  bool has_twist_;
  int point_feature_size_;
  bool has_variance_;
  int max_voxel_size_;
  std::vector<double> point_cloud_range_;
  std::vector<double> voxel_size_;
  int downsample_factor_;
  int encoder_in_feature_size_;
  std::vector<int> allow_remapping_by_area_matrix_;
  std::vector<double> min_area_matrix_;
  std::vector<double> max_area_matrix_;

  std::unique_ptr<CenterPointTRT> detector_ptr_;
};

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__NODE_HPP_