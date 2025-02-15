
#include "lidar_shape_estimation/node.hpp"
#include "lidar_shape_estimation/shape_estimator.hpp"

ShapeEstimationNode::ShapeEstimationNode() : nh_(""), pnh_("~")
{
  sub_ = nh_.subscribe("input", 1, &ShapeEstimationNode::callback, this);
  pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("objects", 1, true);
}

void ShapeEstimationNode::callback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_msg)
{
  ros::Time start_time = ros::Time::now();
  // Guard
  if (pub_.getNumSubscribers() < 1)
    return;

  // Create output msg
  auto output_msg = *input_msg;

  // Estimate shape for each object and pack msg
  for (auto& object : output_msg.objects)
  {
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    // calculate distance
    double distance = sqrt((object.pose.position.x * object.pose.position.x) +
                          (object.pose.position.y * object.pose.position.y));

    if (distance < 10) {
      pcl::fromROSMsg(object.pointcloud, *cluster);
      // estimate shape and pose
      estimator_.getShapeAndPose(object.label, *cluster, object);
    }
  }

  // Publish
  pub_.publish(output_msg);
  ros::Time end_time = ros::Time::now();
  ros::Duration duration = end_time - start_time;
  ROS_INFO("clusters shape estimate cost time: %f", duration.toSec());
  return;
}
