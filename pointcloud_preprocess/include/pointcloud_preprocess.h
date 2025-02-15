#ifndef POINTCLOUD_PREPROCESS_NODE_H
#define POINTCLOUD_PREPROCESS_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class PointCloudPreprocessNode {
public:
    PointCloudPreprocessNode();
    ~PointCloudPreprocessNode();

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    bool isPointInCar(const pcl::PointXYZI& point);

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
    double clip_min_height_;
    double clip_max_height_;
    std::string input_topic_;
    std::string output_topic_;
    bool remove_self_point_;
};

#endif // POINTCLOUD_PREPROCESS_NODE_H