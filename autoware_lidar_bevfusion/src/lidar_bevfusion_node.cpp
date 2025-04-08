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

#include "autoware/lidar_bevfusion/lidar_bevfusion_node.hpp"
#include "autoware/lidar_bevfusion/utils.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::lidar_bevfusion
{

LidarBEVFusionNode::LidarBEVFusionNode()
: nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
{
  // Modality
  nh_.param("sensor_fusion", sensor_fusion_, true);
  // Non network parameters
  nh_.param("max_camera_lidar_delay", max_camera_lidar_delay_, 0.0f);
  // TensorRT parameters
  std::string plugins_path;
  nh_.param("plugins_path", plugins_path, std::string(""));
  // Network parameters
  std::string onnx_path, engine_path, trt_precision;
  nh_.param("onnx_path", onnx_path, std::string(""));
  nh_.param("engine_path", engine_path, std::string(""));
  nh_.param("trt_precision", trt_precision, std::string(""));

  // Common parameters
  int out_size_factor;
  nh_.param("out_size_factor", out_size_factor, 0);

  // Lidar branch parameters
  int cloud_capacity, max_points_per_voxel;
  nh_.param("cloud_capacity", cloud_capacity, 0);
  nh_.param("max_points_per_voxel", max_points_per_voxel, 0);
  
  std::vector<int> voxels_num;
  nh_.param("voxels_num", voxels_num, std::vector<int>());
  
  std::vector<double> point_cloud_range_double, voxel_size_double;
  nh_.param("point_cloud_range", point_cloud_range_double, std::vector<double>());
  nh_.param("voxel_size", voxel_size_double, std::vector<double>());
  
  std::vector<float> point_cloud_range(point_cloud_range_double.begin(), point_cloud_range_double.end());
  std::vector<float> voxel_size(voxel_size_double.begin(), voxel_size_double.end());

  // Camera branch parameters
  std::vector<double> d_bound_double, x_bound_double, y_bound_double, z_bound_double;
  nh_.param("d_bound", d_bound_double, std::vector<double>());
  nh_.param("x_bound", x_bound_double, std::vector<double>());
  nh_.param("y_bound", y_bound_double, std::vector<double>());
  nh_.param("z_bound", z_bound_double, std::vector<double>());
  
  std::vector<float> d_bound(d_bound_double.begin(), d_bound_double.end());
  std::vector<float> x_bound(x_bound_double.begin(), x_bound_double.end());
  std::vector<float> y_bound(y_bound_double.begin(), y_bound_double.end());
  std::vector<float> z_bound(z_bound_double.begin(), z_bound_double.end());

  int num_cameras, raw_image_height, raw_image_width;
  nh_.param("num_cameras", num_cameras, 0);
  nh_.param("raw_image_height", raw_image_height, 0);
  nh_.param("raw_image_width", raw_image_width, 0);
  
  float img_aug_scale_x, img_aug_scale_y;
  nh_.param("img_aug_scale_x", img_aug_scale_x, 0.0f);
  nh_.param("img_aug_scale_y", img_aug_scale_y, 0.0f);
  
  int roi_height, roi_width, features_height, features_width;
  nh_.param("roi_height", roi_height, 0);
  nh_.param("roi_width", roi_width, 0);
  nh_.param("features_height", features_height, 0);
  nh_.param("features_width", features_width, 0);
  
  int num_depth_features;
  nh_.param("num_depth_features", num_depth_features, 0);

  // Head parameters
  int num_proposals;
  nh_.param("num_proposals", num_proposals, 0);
  nh_.param("class_names", class_names_, std::vector<std::string>());

  if (point_cloud_range.size() != 6) {
    ROS_ERROR("The size of point_cloud_range != 6");
    throw std::runtime_error("The size of point_cloud_range != 6");
  }
  if (voxel_size.size() != 3) {
    ROS_WARN_STREAM("The size of voxel_size != 3");
    throw std::runtime_error("The size of voxel_size != 3");
  }

  // pre-process
  std::string densification_world_frame_id;
  int densification_num_past_frames;
  nh_.param("densification_world_frame_id", densification_world_frame_id, std::string(""));
  nh_.param("densification_num_past_frames", densification_num_past_frames, 0);

  // post-process
  double circle_nms_dist_threshold_double;
  nh_.param("circle_nms_dist_threshold", circle_nms_dist_threshold_double, 0.0);
  float circle_nms_dist_threshold = static_cast<float>(circle_nms_dist_threshold_double);
  
  // {  // IoU NMS
  //   NMSParams p;
  //   double search_distance_2d, iou_threshold;
  //   nh_.param("iou_nms_search_distance_2d", search_distance_2d, 0.0);
  //   nh_.param("iou_nms_threshold", iou_threshold, 0.0);
  //   p.search_distance_2d_ = search_distance_2d;
  //   p.iou_threshold_ = iou_threshold;
  //   iou_bev_nms_.setParameters(p);
  // }
  
  std::vector<double> yaw_norm_thresholds;
  nh_.param("yaw_norm_thresholds", yaw_norm_thresholds, std::vector<double>());
  
  double score_threshold_double;
  nh_.param("score_threshold", score_threshold_double, 0.0);
  float score_threshold = static_cast<float>(score_threshold_double);

  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);

  // Common parameters
  out_size_factor = static_cast<std::int64_t>(out_size_factor);

  // Lidar branch parameters
  cloud_capacity = static_cast<std::int64_t>(cloud_capacity);
  max_points_per_voxel = static_cast<std::int64_t>(max_points_per_voxel);

  // Convert voxels_num from std::vector<int> to std::vector<std::int64_t>
  std::vector<std::int64_t> voxels_num_converted(voxels_num.begin(), voxels_num.end());

  // Camera branch parameters
  num_cameras = static_cast<std::int64_t>(num_cameras);
  raw_image_height = static_cast<std::int64_t>(raw_image_height);
  raw_image_width = static_cast<std::int64_t>(raw_image_width);

  roi_height = static_cast<std::int64_t>(roi_height);
  roi_width = static_cast<std::int64_t>(roi_width);
  features_height = static_cast<std::int64_t>(features_height);
  features_width = static_cast<std::int64_t>(features_width);
  num_depth_features = static_cast<std::int64_t>(num_depth_features);
  num_proposals = static_cast<std::int64_t>(num_proposals);

  // Ensure yaw_norm_thresholds is a vector of doubles
  std::vector<double> yaw_norm_thresholds_converted(yaw_norm_thresholds.begin(), yaw_norm_thresholds.end());

  // Call BEVFusionConfig constructor with converted types
  BEVFusionConfig config(
      sensor_fusion_, plugins_path, out_size_factor, cloud_capacity, max_points_per_voxel,
      voxels_num_converted, point_cloud_range, voxel_size, d_bound, x_bound, y_bound, z_bound,
      num_cameras, raw_image_height, raw_image_width, img_aug_scale_x, img_aug_scale_y, roi_height,
      roi_width, features_height, features_width, num_depth_features, num_proposals,
      circle_nms_dist_threshold, yaw_norm_thresholds_converted, score_threshold
  );

  std::vector<int64_t> allow_remapping_by_area_matrix;
  std::vector<double> min_area_matrix, max_area_matrix;
  // nh_.param("allow_remapping_by_area_matrix", allow_remapping_by_area_matrix, std::vector<int>());
  // nh_.param("min_area_matrix", min_area_matrix, std::vector<double>());
  // nh_.param("max_area_matrix", max_area_matrix, std::vector<double>());
  
  // detection_class_remapper_.setParameters(
  //   allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  auto trt_config =
    tensorrt_common::TrtCommonConfig(onnx_path, trt_precision, engine_path, 1ULL << 32U);
  detector_ptr_ = std::make_unique<BEVFusionTRT>(trt_config, densification_param, config);

  cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    "input/pointcloud", 1, &LidarBEVFusionNode::cloudCallback, this);

  objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
    "output/objects", 1);

  if (sensor_fusion_) {
    image_subs_.resize(num_cameras);
    camera_info_subs_.resize(num_cameras);
    image_msgs_.resize(num_cameras);
    camera_info_msgs_.resize(num_cameras);
    lidar2camera_extrinsics_.resize(num_cameras);

    for (int64_t camera_id = 0; camera_id < num_cameras; ++camera_id) {
      image_subs_[camera_id] = nh_.subscribe<sensor_msgs::Image>(
        "input/image" + std::to_string(camera_id), 1,
        boost::bind(&LidarBEVFusionNode::imageCallback, this, _1, camera_id));

        camera_info_subs_[camera_id] = nh_.subscribe<sensor_msgs::CameraInfo>(
          "input/camera_info" + std::to_string(camera_id), 1,
          [this, camera_id](const boost::shared_ptr<const sensor_msgs::CameraInfo>& msg) {
            cameraInfoCallback(*msg, camera_id);
          });
    }
  }

  bool build_only;
  nh_.param("build_only", build_only, false);
  if (build_only) {
    ROS_INFO("TensorRT engine was built. Shutting down the node.");
    ros::shutdown();
  }
}

void LidarBEVFusionNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  lidar_frame_ = pc_msg->header.frame_id;

  if (sensor_fusion_ && (!extrinsics_available_ || !images_available_ || !intrinsics_available_)) {
    return;
  }

  if (sensor_fusion_ && !intrinsics_extrinsics_precomputed_) {
    std::vector<sensor_msgs::CameraInfo> camera_info_msgs;
    std::vector<Matrix4f> lidar2camera_extrinsics;

    std::transform(
      camera_info_msgs_.begin(), camera_info_msgs_.end(), std::back_inserter(camera_info_msgs),
      [](const auto & opt) { return opt; });

    std::transform(
      lidar2camera_extrinsics_.begin(), lidar2camera_extrinsics_.end(),
      std::back_inserter(lidar2camera_extrinsics), [](const auto & opt) { return opt; });

    detector_ptr_->setIntrinsicsExtrinsics(camera_info_msgs, lidar2camera_extrinsics);
    intrinsics_extrinsics_precomputed_ = true;
  }

  double lidar_stamp = pc_msg->header.stamp.toSec();
  camera_masks_.resize(camera_info_msgs_.size());
  for (std::size_t i = 0; i < camera_masks_.size(); ++i) {
    camera_masks_[i] =
      (lidar_stamp - image_msgs_[i]->header.stamp.toSec()) < max_camera_lidar_delay_
        ? 1.0
        : 0.f;
  }

  std::vector<Box3D> det_boxes3d;
  std::unordered_map<std::string, double> proc_timing;
  bool is_success =
    detector_ptr_->detect(pc_msg, image_msgs_, camera_masks_, tf_buffer_, det_boxes3d, proc_timing);
  if (!is_success) {
    return;
  }

  std::vector<autoware_msgs::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_msgs::DetectedObject obj;
    // box3DToDetectedObject(box3d, class_names_, obj);
    raw_objects.emplace_back(obj);
  }

  autoware_msgs::DetectedObjectArray output_msg;
  output_msg.header = pc_msg->header;
  // output_msg.objects = iou_bev_nms_.apply(raw_objects);

  // detection_class_remapper_.mapClasses(output_msg);

  objects_pub_.publish(output_msg);

  // // add processing time for debug
  // if (debug_publisher_ptr_ && stop_watch_ptr_) {
  //   const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic", true);
  //   const double processing_time_ms = stop_watch_ptr_->toc("processing/total", true);
  //   const double pipeline_latency_ms =
  //     (ros::Time::now() - output_msg.header.stamp).toSec() * 1000.0;
  //   debug_publisher_ptr_->publish<tier4_debug_msgs::Float64Stamped>(
  //     "debug/cyclic_time_ms", cyclic_time_ms);
  //   debug_publisher_ptr_->publish<tier4_debug_msgs::Float64Stamped>(
  //     "debug/pipeline_latency_ms", pipeline_latency_ms);
  // }
}

void LidarBEVFusionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg, const int64_t camera_id)
{
  image_msgs_[camera_id] = msg;
  images_available_ = true;
}

void LidarBEVFusionNode::cameraInfoCallback(
  const sensor_msgs::CameraInfo& msg, const int64_t camera_id)
{
  camera_info_msgs_[camera_id] = msg;

  // Count the number of valid intrinsics
  size_t num_valid_intrinsics = std::count_if(
    camera_info_msgs_.begin(), camera_info_msgs_.end(),
    [](const sensor_msgs::CameraInfo& info) { return !info.K.empty(); });

  intrinsics_available_ = (num_valid_intrinsics == camera_info_msgs_.size());

  // If extrinsics are already available or lidar frame is not set, return early
  if (
    !lidar2camera_extrinsics_[camera_id].isZero() || lidar_frame_.empty() ||
    extrinsics_available_) {
    return;
  }

  try {
    // Lookup transform from camera frame to lidar frame
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped =
      tf_buffer_.lookupTransform(msg.header.frame_id, lidar_frame_, ros::Time(0));

    // Convert transform to Eigen matrix
    Eigen::Matrix4f lidar2camera_transform =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();

    // Ensure row-major storage
    Matrix4f lidar2camera_rowmajor_transform = lidar2camera_transform.eval();
    lidar2camera_extrinsics_[camera_id] = lidar2camera_rowmajor_transform;
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Count the number of valid extrinsics
  std::size_t num_valid_extrinsics = std::count_if(
    lidar2camera_extrinsics_.begin(), lidar2camera_extrinsics_.end(),
    [](const auto & opt) { return !opt.isZero(); });

  extrinsics_available_ = (num_valid_extrinsics == lidar2camera_extrinsics_.size());
}

}  // namespace autoware::lidar_bevfusion

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_bevfusion");
  autoware::lidar_bevfusion::LidarBEVFusionNode node;
  ros::spin();
  return 0;
}
