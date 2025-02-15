
#include "lidar_shape_estimation/shape_estimator.hpp"
#include "lidar_shape_estimation/model_interface.hpp"
#include "model/bounding_box.hpp"
#include "model/convex_hull.hpp"
#include "model/cylinder.hpp"
#include <memory>
#include <iostream>
#include <chrono>

ShapeEstimator::ShapeEstimator()
{
}

bool ShapeEstimator::getShapeAndPose(const std::string& label, const pcl::PointCloud<pcl::PointXYZ>& cluster,
                                     autoware_msgs::DetectedObject& output)
{
  if (cluster.empty())
    return false;
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  // if (label == "car" || label == "vehicle" || label == "truck" || label == "bus")
  // {
  //   model_ptr.reset(new BoundingBoxModel);
  // }
  // else if (label == "person")
  // {
  //   model_ptr.reset(new CylinderModel);
  // }
  // else if (label == "motorbike")
  // {
  //   model_ptr.reset(new BoundingBoxModel);
  // }
  // else if (label == "bicycle")
  // {
  //   model_ptr.reset(new BoundingBoxModel);
  // }
  // else
  // {
  //   //        model_ptr.reset(new CylinderModel);
  //   model_ptr.reset(new BoundingBoxModel);
  // };
  model_ptr.reset(new BoundingBoxModel);
  return model_ptr->estimate(cluster, output);
}
