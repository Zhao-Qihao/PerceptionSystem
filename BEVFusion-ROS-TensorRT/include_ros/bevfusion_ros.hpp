#ifndef bevfusion_ros_h
#define bevfusion_ros_h


#include "bevfusion_plugin.hpp"

#include <ros/ros.h>
// message_filters消息同步器
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h> // 时间接近

// 图像
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

// 点云
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>


class RosNode 
{ 
  std::string model_name_, precision_;
  ros::NodeHandle n_;
  ros::Publisher pub_img_;
  ros::Publisher pub_obj_;

  std::string topic_cloud_;
  std::string topic_img_f_, topic_img_fl_, topic_img_fr_;
  std::string topic_img_b_, topic_img_bl_, topic_img_br_;

    // 预分配的固定大小内存缓冲区（假设每张图像尺寸为 1920x1080x3 字节）
    std::vector<unsigned char> front_buffer;          // 前向摄像头
    std::vector<unsigned char> front_left_buffer;     // 前左
    std::vector<unsigned char> front_right_buffer;    // 前右
    std::vector<unsigned char> back_buffer;           // 后向
    std::vector<unsigned char> back_left_buffer;      // 后左
    std::vector<unsigned char> back_right_buffer;     // 后右

    // 存储图像数据的列表（复用缓冲区）
    std::vector<std::vector<unsigned char>> images_;  // 预分配容量为6
    size_t image_size;   // 图像尺寸,若不同camera的图像尺寸不同，则需要定义多个


  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_; 
  message_filters::Subscriber<sensor_msgs::Image> sub_f_img_; 
  message_filters::Subscriber<sensor_msgs::Image> sub_fl_img_; 
  message_filters::Subscriber<sensor_msgs::Image> sub_fr_img_; 
  message_filters::Subscriber<sensor_msgs::Image> sub_b_img_; 
  message_filters::Subscriber<sensor_msgs::Image> sub_bl_img_; 
  message_filters::Subscriber<sensor_msgs::Image> sub_br_img_; 

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, 
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	std::shared_ptr<Sync> sync_;

  std::shared_ptr<BEVFusionNode> bevfusion_node_;

  
 public:
  RosNode(const std::string model_name, const std::string  precision);
  ~RosNode(){};
  void getTopicName();
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, 
    const sensor_msgs::ImageConstPtr& msg_f_img,
    const sensor_msgs::ImageConstPtr& msg_fl_img,
    const sensor_msgs::ImageConstPtr& msg_fr_img,
    const sensor_msgs::ImageConstPtr& msg_b_img,
    const sensor_msgs::ImageConstPtr& msg_bl_img,
    const sensor_msgs::ImageConstPtr& msg_br_img);
  void pubDetected(std::vector<bevfusion::head::transbbox::BoundingBox>& boxes, const sensor_msgs::PointCloud2ConstPtr& msg_cloud);
};

#endif