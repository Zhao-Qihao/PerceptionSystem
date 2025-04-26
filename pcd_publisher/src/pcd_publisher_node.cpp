#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <vector>

// 读取 .bin 文件并解析为点云数据
std::vector<float> read_bin_file(const std::string& file_path) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        ROS_ERROR("Could not open file %s", file_path.c_str());
        exit(1);
    }

    std::vector<float> points;
    float value;
    while (file.read(reinterpret_cast<char*>(&value), sizeof(float))) {
        points.push_back(value);
    }

    return points;
}

// 创建 PointCloud2 消息
sensor_msgs::PointCloud2 create_point_cloud_msg(const std::vector<float>& points, const std::string& frame_id) {
    sensor_msgs::PointCloud2 cloud_msg;

    // 设置头部信息
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = ros::Time::now();

    // 设置点云数据
    cloud_msg.height = 1;  // 一行点
    cloud_msg.width = points.size() / 4;  // 每个点有4个属性 (x, y, z, intensity)
    cloud_msg.is_dense = true;

    // 设置点云字段
    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    // cloud_msg.fields[3].name = "ring";
    // cloud_msg.fields[3].offset = 16;
    // cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    // cloud_msg.fields[3].count = 1;

    // 设置点云数据
    cloud_msg.point_step = 16;  // 每个点的步长      kitti 的步长为16， nuscenes的步长为20
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
    memcpy(&cloud_msg.data[0], &points[0], cloud_msg.data.size());

    return cloud_msg;
}

// 发布点云数据
void publish_point_cloud() {
    ros::NodeHandle nh("~");  // 使用私有命名空间

    std::string file_path;
    std::string frame_id;
    int publish_rate;
    std::string output_topic;

    // 获取参数
    nh.getParam("file_path", file_path);
    nh.getParam("frame_id", frame_id);
    nh.getParam("publish_rate", publish_rate);
    nh.getParam("output_topic", output_topic);

    ROS_INFO("File path: %s", file_path.c_str());
    ROS_INFO("Frame ID: %s", frame_id.c_str());
    ROS_INFO("Publish rate: %d Hz", publish_rate);
    ROS_INFO("Output topic: %s", output_topic.c_str());

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

    std::vector<float> points = read_bin_file(file_path);
    sensor_msgs::PointCloud2 cloud_msg = create_point_cloud_msg(points, frame_id);

    ros::Rate rate(publish_rate);  // 设置发布频率

    while (ros::ok()) {
        // 更新时间戳
        cloud_msg.header.stamp = ros::Time::now();
        pub.publish(cloud_msg);
        ROS_INFO("Published single point cloud message.");
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_publisher");
    publish_point_cloud();
    return 0;
}