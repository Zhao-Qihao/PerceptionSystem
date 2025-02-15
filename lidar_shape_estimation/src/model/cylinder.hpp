
#pragma once

#include "lidar_shape_estimation/model_interface.hpp"

class CylinderModel : public ShapeEstimationModelInterface
{
public:
  CylinderModel(){};

  ~CylinderModel(){};

  /*
   * if cluster size is 1, it can be calculated using the lower limit of the size of the object
   */
  bool estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output) override;
};