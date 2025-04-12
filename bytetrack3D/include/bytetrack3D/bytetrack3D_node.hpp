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

#ifndef BYTETRACK3D__BYTETRACK3D_NODE_HPP_
#define BYTETRACK3D__BYTETRACK3D_NODE_HPP_

#include <bytetrack3D/bytetrack3D.hpp>
#include "ros/ros.h"

#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bytetrack3D
{
using LabelMap = std::map<int, std::string>;

class ByteTrack3DNode
{
public:
  ByteTrack3DNode();
  ~ByteTrack3DNode() = default;

private:
  void on_rect(const autoware_msgs::DetectedObjectArray::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber detection_rect_sub_;
  ros::Publisher objects_pub_;

  std::unique_ptr<bytetrack3D::ByteTrack3D> bytetrack3D_;
};

}  // namespace bytetrack3D

#endif  // BYTETRACK3D__BYTETRACK3D_NODE_HPP_
