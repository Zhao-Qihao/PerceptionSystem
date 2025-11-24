#include "pointcloud_preprocess.h"
#include <pcl/filters/crop_box.h>
#include <Eigen/Core>
#include <limits>

PointCloudPreprocessNode::PointCloudPreprocessNode()
    : nh_("~")
{
    // 获取参数
    nh_.param("clip_min_height", clip_min_height_, -3.25);
    nh_.param("clip_max_height", clip_max_height_, 0.5);
    nh_.param("input_topic", input_topic_, std::string("/input/pointcloud"));
    nh_.param("output_topic", output_topic_, std::string("/output/pointcloud"));
    nh_.param("remove_self_point", remove_self_point_, true);

    // 激光雷达到车辆边界的距离（从 YAML 读取，给出默认值）
    nh_.param("lidar_to_car_left",  lidar_to_car_left_,  0.6);
    nh_.param("lidar_to_car_right", lidar_to_car_right_, 0.7);
    nh_.param("lidar_to_car_front", lidar_to_car_front_, 0.3);
    nh_.param("lidar_to_car_back",  lidar_to_car_back_,  2.45);

    // 创建发布者和订阅者
    point_cloud_sub_ = nh_.subscribe(input_topic_, 1, &PointCloudPreprocessNode::cloudCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
}

PointCloudPreprocessNode::~PointCloudPreprocessNode()
{
}

void PointCloudPreprocessNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{   
    ros::Time start = ros::Time::now();
    // 将 ROS 点云消息转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 创建 PassThrough 过滤器
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clip_min_height_, clip_max_height_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pass.filter(*cloud_z_filtered);

    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;

    // 如果需要移除自身车辆部分的点云
    if (remove_self_point_) {
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud_z_filtered);

        // 使用激光雷达到车辆前后左右的距离定义车体包围盒
        // 坐标系假设：x 前方为正，y 左侧为正
        const float z_min = std::numeric_limits<float>::lowest();
        const float z_max = std::numeric_limits<float>::max();
        Eigen::Vector4f min_pt(
            -lidar_to_car_back_,   // 车尾在原点后方（x 负）
            -lidar_to_car_right_,  // 右侧在 y 负
            z_min,
            1.0f
        );
        Eigen::Vector4f max_pt(
            lidar_to_car_front_,   // 车头在 x 正
            lidar_to_car_left_,    // 左侧在 y 正
            z_max,
            1.0f
        );
        crop.setMin(min_pt);
        crop.setMax(max_pt);

        // 负向过滤：保留车体包围盒外部的点
        crop.setNegative(true);

        crop.filter(cloud_filtered);
    } else {
        cloud_filtered = *cloud_z_filtered;
    }

    // 将过滤后的 PCL 点云转换回 ROS 点云消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    output.header = cloud_msg->header;

    // 发布过滤后的点云
    point_cloud_pub_.publish(output);
    ROS_INFO("PointCloud Preprocess Time: %f ms", (ros::Time::now() - start).toSec()*1000);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_preprocess_node");
    PointCloudPreprocessNode node;
    ros::spin();
    return 0;
}