#include "pointcloud_preprocess.h"

// 定义车辆边界参数
const double CAR_LENGTH = 11.45;  // 车辆长度
const double CAR_WIDTH = 2.934;   // 车辆宽度
const double CAR_X_OFFSET = -5.725; // 车辆在点云坐标系中的X轴偏移
const double CAR_Y_OFFSET = 0.0; // 车辆在点云坐标系中的Y轴偏移

PointCloudPreprocessNode::PointCloudPreprocessNode()
    : nh_("~")
{
    // 获取参数
    nh_.param("clip_min_height", clip_min_height_, -3.25);
    nh_.param("clip_max_height", clip_max_height_, 0.5);
    nh_.param("input_topic", input_topic_, std::string("/input/pointcloud"));
    nh_.param("output_topic", output_topic_, std::string("/output/pointcloud"));
    nh_.param("remove_self_point", remove_self_point_, true);

    // 创建发布者和订阅者
    point_cloud_sub_ = nh_.subscribe(input_topic_, 1, &PointCloudPreprocessNode::cloudCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
}

PointCloudPreprocessNode::~PointCloudPreprocessNode()
{
}

bool PointCloudPreprocessNode::isPointInCar(const pcl::PointXYZI& point)
{
    // 检查点是否在车辆边界内
    double x = point.x - CAR_X_OFFSET;
    double y = point.y - CAR_Y_OFFSET;

    return (x >= -CAR_LENGTH / 2.0 && x <= CAR_LENGTH / 2.0 &&
            y >= -CAR_WIDTH / 2.0 && y <= CAR_WIDTH / 2.0);
}

void PointCloudPreprocessNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 将 ROS 点云消息转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // 创建 PassThrough 过滤器
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clip_min_height_, clip_max_height_);
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;
    pass.filter(cloud_filtered);

    // 如果需要移除自身车辆部分的点云
    if (remove_self_point_) {
        pcl::PointCloud<pcl::PointXYZI> cloud_without_car;
        for (const auto& point : cloud_filtered.points) {
            if (!isPointInCar(point)) {
                cloud_without_car.push_back(point);
            }
        }
        cloud_filtered = cloud_without_car;
    }

    // 将过滤后的 PCL 点云转换回 ROS 点云消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    output.header = cloud_msg->header;

    // 发布过滤后的点云
    point_cloud_pub_.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_preprocess_node");
    PointCloudPreprocessNode node;
    ros::spin();
    return 0;
}