
#pragma once

#include "lidar_shape_estimation/model_interface.hpp"

class BoundingBoxModel : public ShapeEstimationModelInterface
{
private:
  double calcClosenessCriterion(const std::vector<double>& C_1, const std::vector<double>& C_2);

public:
  BoundingBoxModel(){};

  ~BoundingBoxModel(){};

  /*
   * minimum cluster size is 2.
   */
  bool estimate(const pcl::PointCloud<pcl::PointXYZ>& cluster, autoware_msgs::DetectedObject& output) override;
};