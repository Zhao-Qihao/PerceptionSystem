#include "lidar_object_fusion/lidar_object_fusion.h"
#include <opencv2/opencv.hpp>

double ROSRangeVisionFusionApp::get3DIoU(const autoware_msgs::DetectedObject &obj1, const autoware_msgs::DetectedObject &obj2)
{
    // Extract 2D center and dimensions from 3D object
    cv::Point2f center1(obj1.pose.position.x, obj1.pose.position.y);
    cv::Point2f center2(obj2.pose.position.x, obj2.pose.position.y);
    float width1 = obj1.dimensions.x;
    float height1 = obj1.dimensions.y;
    float width2 = obj2.dimensions.x;
    float height2 = obj2.dimensions.y;

    // Extract orientation angle (assuming z-axis rotation)
    tf::Quaternion quat1(obj1.pose.orientation.x, obj1.pose.orientation.y, obj1.pose.orientation.z, obj1.pose.orientation.w);
    tf::Matrix3x3 mat1(quat1);
    double roll1, pitch1, yaw1;
    mat1.getRPY(roll1, pitch1, yaw1);

    tf::Quaternion quat2(obj2.pose.orientation.x, obj2.pose.orientation.y, obj2.pose.orientation.z, obj2.pose.orientation.w);
    tf::Matrix3x3 mat2(quat2);
    double roll2, pitch2, yaw2;
    mat2.getRPY(roll2, pitch2, yaw2);

    // Create rotated rectangles
    cv::RotatedRect rect1(center1, cv::Size2f(width1, height1), yaw1 * 180 / CV_PI);
    cv::RotatedRect rect2(center2, cv::Size2f(width2, height2), yaw2 * 180 / CV_PI);

    // Calculate intersection area
    std::vector<cv::Point2f> intersectionPoints;
    double intersectionArea = cv::rotatedRectangleIntersection(rect1, rect2, intersectionPoints);

    // Calculate union area
    double area1 = rect1.size.width * rect1.size.height;
    double area2 = rect2.size.width * rect2.size.height;
    double unionArea = area1 + area2 - intersectionArea;

    // Calculate IoU
    double iou = (unionArea > 0) ? intersectionArea / unionArea : 0;
    if (iou > 0)
    {
      std::cout << "3D IoU: " << iou << std::endl;
    }

    return iou;
}

double ROSRangeVisionFusionApp::GetDistanceToObject(const autoware_msgs::DetectedObject &object0, const autoware_msgs::DetectedObject &object1)
{ 
  float distance = sqrt(pow(object0.pose.position.x - object1.pose.position.x, 2) + pow(object0.pose.position.y - object1.pose.position.y, 2) +  
                pow(object0.pose.position.z - object1.pose.position.z, 2));
  return distance;
}

// 最近邻算法匹配，带有最大距离阈值
void ROSRangeVisionFusionApp::nearestNeighborMatching(autoware_msgs::DetectedObjectArray& boxes1, autoware_msgs::DetectedObjectArray& boxes2,
                              float distanceThreshold, autoware_msgs::DetectedObjectArray& matchedBoxes) {
    std::vector<bool> boxes2Matched(boxes2.objects.size(), false);  // 标记boxes2中的目标框是否已被匹配

    for (const auto& box1 : boxes1.objects) {
        int bestMatchIdx = -1;
        float bestCost = std::numeric_limits<float>::max();  // 用于存储最小匹配成本

        // 查找最匹配的目标框
        for (size_t i = 0; i < boxes2.objects.size(); ++i) {
            if (!boxes2Matched[i]) {
                float cost = GetDistanceToObject(box1, boxes2.objects[i]);  // 计算距离

                // 如果距离小于阈值，并且成本更小，则更新匹配
                if (cost <= distanceThreshold && cost < bestCost) {
                    bestCost = cost;
                    bestMatchIdx = i;
                }
            }
        }

        // 如果找到匹配的目标框，添加到结果集合
        if (bestMatchIdx != -1) {
            // boxes2.objects[bestMatchIdx].pose.orientation = box1.pose.orientation; // 这里经过测试后发现euclidean的方向比centerpoint的方向更准确
            matchedBoxes.objects.push_back(boxes2.objects[bestMatchIdx]);

            boxes2Matched[bestMatchIdx] = true;  // 标记为已匹配
        } else {
            // 如果没有匹配的目标框，可以选择保留boxes1中的目标框
            matchedBoxes.objects.push_back(box1);
        }
    }

    // 添加未匹配的boxes2中的目标框
    for (size_t i = 0; i < boxes2.objects.size(); ++i) {
        if (!boxes2Matched[i]) {
            matchedBoxes.objects.push_back(boxes2.objects[i]);
        }
    }
}

void ROSRangeVisionFusionApp::PostRemoveProcess(autoware_msgs::DetectedObjectArray &matchedBoxes)
{
    // 使用迭代器来安全地删除元素
    for (auto it = matchedBoxes.objects.begin(); it != matchedBoxes.objects.end(); ) {
        if (it->pose.position.x < 0.12 && it->pose.position.x > -11.33 && it->pose.position.y > -1.47 && it->pose.position.y < 1.47) {
            it = matchedBoxes.objects.erase(it); // 删除当前元素并更新迭代器
        } else {
            ++it; // 只有在不删除元素时才递增迭代器
        }
    }
}

autoware_msgs::DetectedObjectArray
ROSRangeVisionFusionApp::FuseRangeVisionDetections(
  const autoware_msgs::DetectedObjectArray::ConstPtr &detections0,
  const autoware_msgs::DetectedObjectArray::ConstPtr &detections1)
{
  // 存储匹配的目标框
  autoware_msgs::DetectedObjectArray matchedBoxes;
  matchedBoxes.header = detections1->header;

  // 解引用 ConstPtr 并创建非常量副本
  autoware_msgs::DetectedObjectArray boxes1 = *detections0;
  autoware_msgs::DetectedObjectArray boxes2 = *detections1;

  nearestNeighborMatching(boxes1, boxes2, distanceThreshold_, matchedBoxes);
  PostRemoveProcess(matchedBoxes);
  ROS_INFO("euclidean object num: %zu, centerpoint object num, %zu, num of matchedBoxes: %zu", detections0->objects.size(), detections1->objects.size(), matchedBoxes.objects.size());

  return matchedBoxes;
}

void
ROSRangeVisionFusionApp::SyncedDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &detections0,
  const autoware_msgs::DetectedObjectArray::ConstPtr &detections1)
{
  autoware_msgs::DetectedObjectArray fusion_objects;
  fusion_objects.objects.clear();

  if (empty_frames_ > 5)
  {
    ROS_INFO("[%s] Empty Detections. Make sure the vision and range detectors are running.", __APP_NAME__);
  }

  if (nullptr == detections0
      && nullptr == detections1)
  {
    empty_frames_++;
    return;
  }

  if (nullptr == detections0
      && nullptr != detections1
      && !detections1->objects.empty())
  {
    publisher_fused_objects_.publish(detections1);
    empty_frames_++;
    return;
  }
  if (nullptr == detections1
      && nullptr != detections0
      && !detections0->objects.empty())
  {
    publisher_fused_objects_.publish(detections0);
    empty_frames_++;
    return;
  }

  fusion_objects = FuseRangeVisionDetections(detections0, detections1);

  publisher_fused_objects_.publish(fusion_objects);
  empty_frames_ = 0;
}
  
void
ROSRangeVisionFusionApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string detected_objects_vision;
  std::string detected_objects_range, fused_topic_str = "/detection/fusion_tools/objects";
  std::string name_space_str = ros::this_node::getNamespace();
  bool sync_topics = true;

  ROS_INFO(
    "[%s] This node requires: Euclidean cluster and Centerpoint Detections being published.",
    __APP_NAME__);
  in_private_handle.param<std::string>("detected_objects_range", detected_objects_range,
                                       "/detection/lidar_detector/objects");
  ROS_INFO("[%s] detected_objects_range: %s", __APP_NAME__, detected_objects_range.c_str());

  in_private_handle.param<std::string>("detected_objects_vision", detected_objects_vision,
                                       "/casper_auto/detection/lidar_detector/objects");
  ROS_INFO("[%s] detected_objects_vision: %s", __APP_NAME__, detected_objects_vision.c_str());

  in_private_handle.param<double>("distanceThreshold", distanceThreshold_, 1.2);
  ROS_INFO("[%s] distanceThreshold: %f", __APP_NAME__, distanceThreshold_);

  in_private_handle.param<double>("overlap_threshold", overlap_threshold_, 0.3);
  ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold_);

  in_private_handle.param<bool>("sync_topics", sync_topics, true);
  ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics);

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      name_space_str.erase(name_space_str.begin());
    }
  }

  //generate subscribers and sychronizers
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_vision.c_str());
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_range.c_str());
  if (sync_topics)
  {
    detections0_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                    detected_objects_vision,
                                                                                                    1);
    detections1_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                   detected_objects_range,
                                                                                                   1);
    detections_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                     *detections0_filter_subscriber_,
                                                     *detections1_filter_subscriber_);
    detections_synchronizer_->registerCallback(
      boost::bind(&ROSRangeVisionFusionApp::SyncedDetectionsCallback, this, _1, _2));
  }

  publisher_fused_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(fused_topic_str, 1);

  ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, fused_topic_str.c_str());

}


void
ROSRangeVisionFusionApp::Run()
{
  ros::NodeHandle private_node_handle("~");
  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);
}

ROSRangeVisionFusionApp::ROSRangeVisionFusionApp()
{
  processing_ = false;
  overlap_threshold_ = 0.5;
  distanceThreshold_ = 1.2;
  empty_frames_ = 0;
}