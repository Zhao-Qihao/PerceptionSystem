#include "autoware/lidar_centerpoint/node.hpp"

#include "pcl_ros/impl/transforms.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

namespace autoware::lidar_centerpoint
{
// 计算两个 DetectedObject 在 BEV 平面上的 IoU
float calculate_bev_iou(const autoware_msgs::DetectedObject& obj1, const autoware_msgs::DetectedObject& obj2) {
  // BEV平面的矩形表示为 [x_min, y_min, x_max, y_max]
  float obj1_x_min = obj1.pose.position.x - obj1.dimensions.x / 2.0f;
  float obj1_y_min = obj1.pose.position.y - obj1.dimensions.y / 2.0f;
  float obj1_x_max = obj1.pose.position.x + obj1.dimensions.x / 2.0f;
  float obj1_y_max = obj1.pose.position.y + obj1.dimensions.y / 2.0f;

  float obj2_x_min = obj2.pose.position.x - obj2.dimensions.x / 2.0f;
  float obj2_y_min = obj2.pose.position.y - obj2.dimensions.y / 2.0f;
  float obj2_x_max = obj2.pose.position.x + obj2.dimensions.x / 2.0f;
  float obj2_y_max = obj2.pose.position.y + obj2.dimensions.y / 2.0f;

  // 计算交集区域
  float inter_x_min = std::max(obj1_x_min, obj2_x_min);
  float inter_y_min = std::max(obj1_y_min, obj2_y_min);
  float inter_x_max = std::min(obj1_x_max, obj2_x_max);
  float inter_y_max = std::min(obj1_y_max, obj2_y_max);

  float inter_area = std::max(0.0f, inter_x_max - inter_x_min) * std::max(0.0f, inter_y_max - inter_y_min);

  // 计算并集区域
  float obj1_area = obj1.dimensions.x * obj1.dimensions.y;
  float obj2_area = obj2.dimensions.x * obj2.dimensions.y;
  float union_area = obj1_area + obj2_area - inter_area;

  // 防止除零错误
  if (union_area <= 0.0f) return 0.0f;

  return inter_area / union_area;
}

// BEV NMS实现
std::vector<autoware_msgs::DetectedObject> bev_nms(
  const std::vector<autoware_msgs::DetectedObject>& objects, float iou_threshold) {
  std::vector<autoware_msgs::DetectedObject> result;
  std::vector<std::pair<float, int>> score_index_pairs;

  // 按照置信度分数排序
  for (size_t i = 0; i < objects.size(); ++i) {
      score_index_pairs.emplace_back(std::make_pair(objects[i].score, i));
  }
  std::sort(score_index_pairs.begin(), score_index_pairs.end(),
            [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                return a.first > b.first;
            });

  std::vector<bool> removed(objects.size(), false);

  for (size_t i = 0; i < score_index_pairs.size(); ++i) {
      if (removed[score_index_pairs[i].second]) continue;

      result.push_back(objects[score_index_pairs[i].second]);

      for (size_t j = i + 1; j < score_index_pairs.size(); ++j) {
          if (removed[score_index_pairs[j].second]) continue;

          float iou = calculate_bev_iou(
              objects[score_index_pairs[i].second],
              objects[score_index_pairs[j].second]);

          if (iou > iou_threshold) {
              removed[score_index_pairs[j].second] = true;
          }
      }
  }

  return result;
}

LidarCenterPointNode::LidarCenterPointNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : tf_buffer_(), tf_listener_(tf_buffer_)
{
  // 读取参数
  
  private_nh.param("post_process_params/score_threshold", score_threshold_, 0.0f);
  private_nh.param("post_process_params/circle_nms_dist_threshold", circle_nms_dist_threshold_, 0.0f);
  private_nh.param("post_process_params/yaw_norm_thresholds", yaw_norm_thresholds_, std::vector<double>());
  private_nh.param("densification_params/world_frame_id", densification_world_frame_id_, std::string());
  private_nh.param("densification_params/num_past_frames", densification_num_past_frames_, 0);
  private_nh.param("trt_precision", trt_precision_, std::string());
  private_nh.param("cloud_capacity", cloud_capacity_, 0);
  private_nh.param("encoder_onnx_path", encoder_onnx_path_, std::string());
  private_nh.param("encoder_engine_path", encoder_engine_path_, std::string());
  private_nh.param("head_onnx_path", head_onnx_path_, std::string());
  private_nh.param("head_engine_path", head_engine_path_, std::string());
  private_nh.param("model_params/class_names", class_names_, std::vector<std::string>());
  private_nh.param("model_params/has_twist", has_twist_, false);
  private_nh.param("model_params/point_feature_size", point_feature_size_, 0);
  private_nh.param("model_params/has_variance", has_variance_, false);
  private_nh.param("model_params/max_voxel_size", max_voxel_size_, 0);
  private_nh.param("model_params/point_cloud_range", point_cloud_range_, std::vector<double>());
  private_nh.param("model_params/voxel_size", voxel_size_, std::vector<double>());
  private_nh.param("model_params/downsample_factor", downsample_factor_, 0);
  private_nh.param("model_params/encoder_in_feature_size", encoder_in_feature_size_, 0);
  private_nh.param("allow_remapping_by_area_matrix", allow_remapping_by_area_matrix_, std::vector<int>());
  private_nh.param("min_area_matrix", min_area_matrix_, std::vector<double>());
  private_nh.param("max_area_matrix", max_area_matrix_, std::vector<double>());
  std::cout << "encoder_onnx_path================================================: " << encoder_onnx_path_ << std::endl;
  bool build_only;
  private_nh.param("build_only", build_only, false);


  NetworkParam encoder_param(encoder_onnx_path_, encoder_engine_path_, trt_precision_);
  NetworkParam head_param(head_onnx_path_, head_engine_path_, trt_precision_);
  DensificationParam densification_param(
    densification_world_frame_id_, densification_num_past_frames_);

  if (point_cloud_range_.size() != 6) {
    ROS_WARN_STREAM("The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size_.size() != 3) {
    ROS_WARN_STREAM("The size of voxel_size != 3: use the default parameters.");
  }
  CenterPointConfig config(
    class_names_.size(), point_feature_size_, cloud_capacity_, max_voxel_size_, point_cloud_range_,
    voxel_size_, downsample_factor_, encoder_in_feature_size_, score_threshold_,
    circle_nms_dist_threshold_, yaw_norm_thresholds_, has_variance_);
  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);

  pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
    "input/pointcloud", 1,
    &LidarCenterPointNode::pointCloudCallback, this);
  objects_pub_ = nh.advertise<autoware_msgs::DetectedObjectArray>(
    "output/objects", 1);
  if (build_only) {
    ROS_INFO("TensorRT engine is built and shutdown node.");
    ros::shutdown();
  }
}

void LidarCenterPointNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg)
{ 
  ros::Time start_time = ros::Time::now();
  std::vector<Box3D> det_boxes3d;
  bool is_success = detector_ptr_->detect(*input_pointcloud_msg, tf_buffer_, det_boxes3d);
  if (!is_success) {
    return;
  }

  std::vector<autoware_msgs::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  std::cout << "det_boxes3d.size()=================================: " << det_boxes3d.size() << std::endl; 
  for (const auto & box3d : det_boxes3d) {
    autoware_msgs::DetectedObject object;
    object.header = input_pointcloud_msg->header;
    // box3DToDetectedObject(box3d, class_names_, has_twist_, has_variance_, obj);
    object.valid = true;
    object.pose_reliable = true;
    object.pose.position.x = box3d.x;
    object.pose.position.y = box3d.y;
    object.pose.position.z = box3d.z;
    object.dimensions.x = box3d.length;
    object.dimensions.y = box3d.width;
    object.dimensions.z = box3d.height;
    // mmdet3d yaw format to ros yaw format
    const float yaw = -box3d.yaw - M_PI / 2;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
    object.pose.orientation = q;
    object.score = box3d.score;
    const char* label_names[] = {"car", "truck", "bus", "bicycle", "pedestrian"};
    object.label = label_names[box3d.label];

    raw_objects.emplace_back(object);
  }
  // apply BEV NMS
  float nms_iou_threshold = 0.55;  // 设置IoU阈值
  std::vector<autoware_msgs::DetectedObject> filtered_objects = bev_nms(raw_objects, nms_iou_threshold);
  
  autoware_msgs::DetectedObjectArray output_msg;
  output_msg.header = input_pointcloud_msg->header;
  output_msg.objects = filtered_objects;
  // TODO: add NMS
  // output_msg.objects = iou_bev_nms_.apply(raw_objects);

  // detection_class_remapper_.mapClasses(output_msg);

  objects_pub_.publish(output_msg);
  ros::Time end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  ROS_INFO("Duration: %f", duration.toSec());
}

}  // namespace autoware::lidar_centerpoint

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_center_point");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  autoware::lidar_centerpoint::LidarCenterPointNode node(nh, private_nh);
  ros::spin();
  return 0;
}