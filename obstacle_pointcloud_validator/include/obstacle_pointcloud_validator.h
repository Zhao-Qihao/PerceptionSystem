#ifndef OBSTACLE_POINTCLOUD_VALIDATOR_H
#define OBSTACLE_POINTCLOUD_VALIDATOR_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <vector>

struct PointsNumThresholdParam {
    std::vector<int> min_points_num;
    std::vector<int> max_points_num;
    std::vector<double> min_points_and_distance_ratio;
};

class ObstacleValidatorNode {
public:
    ObstacleValidatorNode();
    ~ObstacleValidatorNode();

    void init();
    void createRosPubSub();
    void SyncedCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input_objects,
                        const sensor_msgs::PointCloud2::ConstPtr &input_obstacle_pointcloud);

private:

    ros::NodeHandle nh_;
    std::string input_objects_topic_;
    std::string obstacle_pointcloud_topic_;
    std::string output_objects_topic_;

    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray>* object_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* obstacle_pointcloud_sub_;
    message_filters::Synchronizer<MySyncPolicy>* sync_;
    ros::Publisher object_pub_;
    PointsNumThresholdParam points_num_threshold_param_;

    size_t getThresholdPointCloud(const autoware_msgs::DetectedObject &object);
    size_t getMaxRadius(const autoware_msgs::DetectedObject &object);
};

#endif  // OBSTACLE_POINTCLOUD_VALIDATOR_H