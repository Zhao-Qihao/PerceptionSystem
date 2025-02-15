// Copyright 2020 Tier IV, Inc.
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

#define EIGEN_MPL2_ONLY

#include <autoware/object_merger/object_association_merger_node.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <chrono>
#include <unordered_map>
#include <utility>

using Label = autoware_perception_msgs::ObjectClassification;

namespace
{
bool isUnknownObjectOverlapped(
  const autoware_perception_msgs::DetectedObject & unknown_object,
  const autoware_perception_msgs::DetectedObject & known_object,
  const double precision_threshold, const double recall_threshold,
  const std::map<int, double> & distance_threshold_map,
  const std::map<int, double> & generalized_iou_threshold_map)
{
  const double   = generalized_iou_threshold_map.at(
    autoware::object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double distance_threshold = distance_threshold_map.at(
    autoware::object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double sq_distance_threshold = std::pow(distance_threshold, 2.0);
  const double sq_distance = autoware::universe_utils::calcSquaredDistance2d(
    unknown_object.kinematics.pose_with_covariance.pose,
    known_object.kinematics.pose_with_covariance.pose);
  if (sq_distance_threshold < sq_distance) return false;
  const auto precision =
    autoware::object_recognition_utils::get2dPrecision(unknown_object, known_object);
  const auto recall = autoware::object_recognition_utils::get2dRecall(unknown_object, known_object);
  const auto generalized_iou =
    autoware::object_recognition_utils::get2dGeneralizedIoU(unknown_object, known_object);
  return precision > precision_threshold || recall > recall_threshold ||
         generalized_iou > generalized_iou_threshold;
}
}  // namespace

namespace
{
std::map<int, double> convertListToClassMap(const std::vector<double> & distance_threshold_list)
{
  std::map<int /*class label*/, double /*distance_threshold*/> distance_threshold_map;
  int class_label = 0;
  for (const auto & distance_threshold : distance_threshold_list) {
    distance_threshold_map.insert(std::make_pair(class_label, distance_threshold));
    class_label++;
  }
  return distance_threshold_map;
}
}  // namespace

namespace autoware::object_merger
{
class ObjectAssociationMergerNode
{
public:
  explicit ObjectAssociationMergerNode(ros::NodeHandle nh)
  : nh_(nh), tf_listener_(tf_buffer_)
  {
    // Parameters
    nh_.param<std::string>("base_link_frame_id", base_link_frame_id_, "");
    nh_.param<int>("priority_mode", priority_mode_, 0);
    nh_.param<int>("sync_queue_size", sync_queue_size_, 10);
    nh_.param<bool>("remove_overlapped_unknown_objects", remove_overlapped_unknown_objects_, false);
    nh_.param<double>("precision_threshold_to_judge_overlapped", overlapped_judge_param_.precision_threshold, 0.5);
    nh_.param<double>("recall_threshold_to_judge_overlapped", overlapped_judge_param_.recall_threshold, 0.5);
    std::vector<double> generalized_iou_threshold_list;
    nh_.param<std::vector<double>>("generalized_iou_threshold", generalized_iou_threshold_list, std::vector<double>());
    overlapped_judge_param_.generalized_iou_threshold = convertListToClassMap(generalized_iou_threshold_list);

    std::vector<double> distance_threshold_list;
    nh_.param<std::vector<double>>("distance_threshold_list", distance_threshold_list, std::vector<double>());
    overlapped_judge_param_.distance_threshold_map = convertListToClassMap(distance_threshold_list);

    std::vector<int> can_assign_matrix;
    nh_.param<std::vector<int>>("can_assign_matrix", can_assign_matrix, std::vector<int>());
    std::vector<double> max_dist_matrix;
    nh_.param<std::vector<double>>("max_dist_matrix", max_dist_matrix, std::vector<double>());
    std::vector<double> max_rad_matrix;
    nh_.param<std::vector<double>>("max_rad_matrix", max_rad_matrix, std::vector<double>());
    std::vector<double> min_iou_matrix;
    nh_.param<std::vector<double>>("min_iou_matrix", min_iou_matrix, std::vector<double>());
    data_association_ = std::make_unique<autoware::object_merger::DataAssociation>(
      can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix);

    // Create publishers and subscribers
    object0_sub_.subscribe(nh_, "input/object0", 1);
    object1_sub_.subscribe(nh_, "input/object1", 1);
    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray, autoware_msgs::DetectedObjectArray> SyncPolicy;
    sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(sync_queue_size_), object0_sub_, object1_sub_);
    sync_ptr_->registerCallback(boost::bind(&ObjectAssociationMergerNode::objectsCallback, this, _1, _2));

    merged_object_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("output/object", 1);
  }

private:
  void objectsCallback(
    const autoware_msgs::DetectedObjectArray::ConstPtr & input_objects0_msg,
    const autoware_msgs::DetectedObjectArray::ConstPtr & input_objects1_msg)
  {
    // Guard
    if (merged_object_pub_.getNumSubscribers() < 1) {
      return;
    }

    /* transform to base_link coordinate */
    autoware_msgs::DetectedObjectArray transformed_objects0, transformed_objects1;
    transformed_objects0 = *input_objects0_msg;
    transformed_objects1 = *input_objects1_msg;

    // build output msg
    autoware_msgs::DetectedObjectArray output_msg;
    output_msg.header = input_objects0_msg->header;

    /* global nearest neighbor */
    std::unordered_map<int, int> direct_assignment, reverse_assignment;
    const auto & objects0 = transformed_objects0.objects;
    const auto & objects1 = transformed_objects1.objects;
    Eigen::MatrixXd score_matrix =
      data_association_->calcScoreMatrix(transformed_objects1, transformed_objects0);
    data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

    for (size_t object0_idx = 0; object0_idx < objects0.size(); ++object0_idx) {
      const auto & object0 = objects0.at(object0_idx);
      if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found and merge
        const auto & object1 = objects1.at(direct_assignment.at(object0_idx));
        switch (priority_mode_) {
          case PriorityMode::Object0:
            output_msg.objects.push_back(object0);
            break;
          case PriorityMode::Object1:
            output_msg.objects.push_back(object1);
            break;
          case PriorityMode::Confidence:
            if (object1.existence_probability <= object0.existence_probability)
              output_msg.objects.push_back(object0);
            else
              output_msg.objects.push_back(object1);
            break;
        }
      } else {  // not found
        output_msg.objects.push_back(object0);
      }
    }
    for (size_t object1_idx = 0; object1_idx < objects1.size(); ++object1_idx) {
      const auto & object1 = objects1.at(object1_idx);
      if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
      } else {                                                                 // not found
        output_msg.objects.push_back(object1);
      }
    }

    // Remove overlapped unknown object
    if (remove_overlapped_unknown_objects_) {
      std::vector<autoware_msgs::DetectedObject> unknown_objects, known_objects;
      unknown_objects.reserve(output_msg.objects.size());
      known_objects.reserve(output_msg.objects.size());
      for (const auto & object : output_msg.objects) {
        if (
          autoware::object_recognition_utils::getHighestProbLabel(object.classification) ==
          Label::UNKNOWN) {
          unknown_objects.push_back(object);
        } else {
          known_objects.push_back(object);
        }
      }
      output_msg.objects.clear();
      output_msg.objects = known_objects;
      for (const auto & unknown_object : unknown_objects) {
        bool is_overlapped = false;
        for (const auto & known_object : known_objects) {
          if (isUnknownObjectOverlapped(
                unknown_object, known_object, overlapped_judge_param_.precision_threshold,
                overlapped_judge_param_.recall_threshold,
                overlapped_judge_param_.distance_threshold_map,
                overlapped_judge_param_.generalized_iou_threshold)) {
            is_overlapped = true;
            break;
          }
        }
        if (!is_overlapped) {
          output_msg.objects.push_back(unknown_object);
        }
      }
    }

    // publish output msg
    merged_object_pub_.publish(output_msg);
  }

  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  tf::TransformBuffer tf_buffer_;
  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> object0_sub_;
  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> object1_sub_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray, autoware_msgs::DetectedObjectArray>>> sync_ptr_;
  ros::Publisher merged_object_pub_;
  std::string base_link_frame_id_;
  PriorityMode priority_mode_;
  int sync_queue_size_;
  bool remove_overlapped_unknown_objects_;
  OverlappedJudgeParam overlapped_judge_param_;
  std::unique_ptr<autoware::object_merger::DataAssociation> data_association_;
};

}  // namespace autoware::object_merger

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_association_merger_node");
  ros::NodeHandle nh("~");
  autoware::object_merger::ObjectAssociationMergerNode node(nh);
  ros::spin();
  return 0;
}