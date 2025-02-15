
#pragma once

#include "lidar_shape_estimation/model_interface.hpp"

class ConvexHullModel : public ShapeEstimationModelInterface
{
public:
  ConvexHullModel(){};

  ~ConvexHullModel(){};

  bool estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output) override;
};