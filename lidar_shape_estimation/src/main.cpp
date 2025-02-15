
#include "lidar_shape_estimation/node.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_shape_estimator");
  ShapeEstimationNode node;
  ros::spin();
  return 0;
}
