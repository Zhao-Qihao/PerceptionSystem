// Copyright 2023 TIER IV, Inc.
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

// Copyright 2024 AutoCore, Inc.
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

#include "bytetrack3D/bytetrack3D.hpp"
#include <bytetrack3D/bytetrack3D_node.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <uuid_msgs/UniqueID.h>

#include <utility>
#include <vector>

namespace bytetrack3D
{
ByteTrack3DNode::ByteTrack3DNode() : nh_("~")
{
  int track_buffer_length = nh_.param("track_buffer_length", 30);

  this->bytetrack3D_ = std::make_unique<bytetrack3D::ByteTrack3D>(track_buffer_length);

  detection_rect_sub_ = nh_.subscribe("in/objects", 1, &ByteTrack3DNode::on_rect, this);
  objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("out/objects", 1);
}

void ByteTrack3DNode::on_rect(
  const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
{
  // using Label = autoware_perception_msgs::ObjectClassification;
  const std::vector<std::string> labels = {"car", "truck", "bus", "bicycle", "motorcycle", "pedestrian", "unknown"};

  autoware_msgs::DetectedObjectArray out_objects;

  // Unpack detection results
  bytetrack3D::ObjectArray object_array;
  for (const auto & feat_obj : msg->objects) {
    Object obj;
    obj.x = feat_obj.pose.position.x;
    obj.y = feat_obj.pose.position.y;
    obj.z = feat_obj.pose.position.z;
    auto q = feat_obj.pose.orientation;
    obj.yaw =
      std::atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
    obj.l = feat_obj.dimensions.x;
    obj.w = feat_obj.dimensions.y;
    obj.h = feat_obj.dimensions.z;
    obj.score = feat_obj.score;
    
    // 查找 label 对应的索引，并赋值给 obj.type
    auto it = std::find(labels.begin(), labels.end(), feat_obj.label);
    if (it != labels.end()) {
      obj.type = std::distance(labels.begin(), it);  // 获取索引作为 int32_t 类型
    } else {
      obj.type = labels.size() - 1;  // 如果未找到，设置为 "unknown" 的索引
    }

    object_array.emplace_back(obj);
  }

  bytetrack3D::ObjectArray objects = bytetrack3D_->update_tracker(object_array);
  for (const auto & tracked_object : objects) {
    autoware_msgs::DetectedObject object;
    uuid_msgs::UniqueID uuid_msg;
    auto tracked_uuid = tracked_object.unique_id;
    // 正确复制UUID字节到uuid_msg
    std::memcpy(uuid_msg.uuid.data(), tracked_uuid.data, 16); // 固定16字节

    // 提取UUID前4字节为uint32_t
    object.id = *reinterpret_cast<const uint32_t*>(tracked_uuid.data);
    object.label = labels[tracked_object.type];
    object.score = 1.0f;

    object.pose.position.x = tracked_object.x;
    object.pose.position.y = tracked_object.y;
    object.pose.position.z = tracked_object.z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, tracked_object.yaw);
    object.pose.orientation = tf2::toMsg(q);
    object.dimensions.x = tracked_object.l;
    object.dimensions.y = tracked_object.w;
    object.dimensions.z = tracked_object.h;
    object.velocity.linear.x = tracked_object.vx;
    object.velocity.linear.y = tracked_object.vy;
    object.velocity.linear.z = tracked_object.vz;
    // object.kinematics.twist_with_covariance.twist.angular.z = tracked_object.vyaw;  // 在autoware_msg中并没有定义角速度,这里暂时不使用

    out_objects.objects.push_back(object);
  }

  out_objects.header = msg->header;
  objects_pub_.publish(out_objects);
}
}  // namespace bytetrack3D

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "bytetrack3d_node");

    try {
        ros::NodeHandle nh;
        bytetrack3D::ByteTrack3DNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return -1;
    }

    return 0;
}