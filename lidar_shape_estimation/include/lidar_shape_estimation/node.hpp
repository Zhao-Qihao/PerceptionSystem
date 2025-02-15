
#pragma once

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "lidar_shape_estimation/shape_estimator.hpp"
#include "autoware_msgs/DetectedObjectArray.h"

class ShapeEstimationNode
{
private:  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  void callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_msg);

private:
  ShapeEstimator estimator_;

public:
  ShapeEstimationNode();

  ~ShapeEstimationNode(){};
};
