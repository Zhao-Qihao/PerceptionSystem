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

    // 激光雷达到车辆边界的距离（由参数服务器读取）
    double lidar_to_car_left_;
    double lidar_to_car_right_;
    double lidar_to_car_front_;
    double lidar_to_car_back_;
};