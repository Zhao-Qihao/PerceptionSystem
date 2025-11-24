#ifndef PROJECT_LIDAR_OBJECT_FUSION_H
#define PROJECT_LIDAR_OBJECT_FUSION_H

#define __APP_NAME__ "lidar_object_fusion"

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>

#include <jsk_recognition_utils/geo/cube.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

class ROSRangeVisionFusionApp
{
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_fused_objects_;

  // detections0: euclidean detections
  // detections1: centerpoint detections
  message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
    *detections0_filter_subscriber_, *detections1_filter_subscriber_;

  bool processing_;
  double distanceThreshold_;
  double overlap_threshold_;

  size_t empty_frames_;

  typedef
  message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *detections_synchronizer_;

  void SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &detections0,
                                const autoware_msgs::DetectedObjectArray::ConstPtr &detections1);

  autoware_msgs::DetectedObjectArray
  FuseRangeVisionDetections(const autoware_msgs::DetectedObjectArray::ConstPtr &detections0,
                            const autoware_msgs::DetectedObjectArray::ConstPtr &detections1);

  bool IsObjectInRange(const autoware_msgs::DetectedObject &in_detection);


  autoware_msgs::DetectedObject TransformObject(const autoware_msgs::DetectedObject &in_detection,
                                                const tf::StampedTransform &in_transform);

  autoware_msgs::DetectedObject MergeObjects(const autoware_msgs::DetectedObject &in_object_a,
                                             const autoware_msgs::DetectedObject &in_object_b);

  double GetDistanceToObject(const autoware_msgs::DetectedObject &object0, const autoware_msgs::DetectedObject &object1);
  double get3DIoU(const autoware_msgs::DetectedObject &obj1, const autoware_msgs::DetectedObject &obj2);
  /*!
   * Reads the config params from the command line
   * @param in_private_handle
   */
  void InitializeROSIo(ros::NodeHandle &in_private_handle);
  void TransformRangeToVision(const autoware_msgs::DetectedObjectArray::ConstPtr &detections1,
                                                autoware_msgs::DetectedObjectArray &det1_in_det0,
                                                autoware_msgs::DetectedObjectArray &det1_out_det0);
                                      
  std::vector<int> hungarianAlgorithm(const std::vector<std::vector<float>>& costMatrix); 
  void nearestNeighborMatching(autoware_msgs::DetectedObjectArray& boxes1, autoware_msgs::DetectedObjectArray& boxes2,
                              float distanceThreshold, autoware_msgs::DetectedObjectArray& matchedBoxes);
  void PostRemoveProcess(autoware_msgs::DetectedObjectArray &matchedBoxes);
public:
  void Run();

  ROSRangeVisionFusionApp();
};


#endif //PROJECT_RANGE_VISION_FUSION_H
