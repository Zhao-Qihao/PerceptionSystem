#include "obstacle_pointcloud_validator.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <cmath>

void ObstacleValidatorNode::createRosPubSub(){        
    ROS_INFO("Creating ROS publishers and subscribers");

    object_sub_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh_, input_objects_topic_, 1);
    obstacle_pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, obstacle_pointcloud_topic_, 1);
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *object_sub_, *obstacle_pointcloud_sub_);
    sync_->registerCallback(boost::bind(&ObstacleValidatorNode::SyncedCallback, this, _1, _2));
    object_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(output_objects_topic_, 1);
}

void ObstacleValidatorNode::init(){
    //get params
    nh_.param("input/detected_objects", input_objects_topic_, std::string("/detection/lidar_detector/objects"));
    nh_.param("input/obstacle_pointcloud", obstacle_pointcloud_topic_, std::string("/kitti/velo/pointcloud"));
    nh_.param("output/detected_objects", output_objects_topic_, std::string("/detection/lidar_filtered/objects"));


    nh_.param("min_points_num", points_num_threshold_param_.min_points_num,
        std::vector<int>{100, 150, 120, 180, 140, 5, 30, 20, 10, 5, 10});
    nh_.param("max_points_num", points_num_threshold_param_.max_points_num,
        std::vector<int>{500, 600, 500, 800, 600, 20, 100, 80, 50, 20, 50});
    nh_.param("min_points_and_distance_ratio",
        points_num_threshold_param_.min_points_and_distance_ratio,
        std::vector<double>{800.0, 900.0, 850.0, 1000.0, 900.0, 300.0, 700.0, 600.0, 500.0, 300.0, 400.0});
}

ObstacleValidatorNode::ObstacleValidatorNode()
    : nh_("~")
{
}

ObstacleValidatorNode::~ObstacleValidatorNode()
{
}

size_t ObstacleValidatorNode::getMaxRadius(const autoware_msgs::DetectedObject &object) {
    auto square_radius = (object.dimensions.x * 0.5f) * (object.dimensions.x * 0.5f) +
                         (object.dimensions.y * 0.5f) * (object.dimensions.y * 0.5f) +
                         (object.dimensions.z * 0.5f) * (object.dimensions.z * 0.5f);
    return static_cast<size_t>(std::sqrt(square_radius));  // 返回计算得到的半径值，并进行类型转换
}

size_t ObstacleValidatorNode::getThresholdPointCloud(const autoware_msgs::DetectedObject &object) {
  const auto object_label_id = object.label_id;
  const auto object_distance = std::hypot(object.pose.position.x, object.pose.position.y);
  if (object_distance == 0) {
      ROS_WARN("Object distance is zero, skipping threshold calculation.");
      return 0;  // 或者返回一个合理的默认值
  }
  size_t threshold_pc = std::clamp(
    static_cast<size_t>(std::lround(points_num_threshold_param_.min_points_and_distance_ratio[object_label_id] / object_distance)),
    static_cast<size_t>(points_num_threshold_param_.min_points_num[object_label_id]),
    static_cast<size_t>(points_num_threshold_param_.max_points_num[object_label_id]));
  return threshold_pc;
}

void ObstacleValidatorNode::SyncedCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input_objects, const sensor_msgs::PointCloud2::ConstPtr &input_obstacle_pointcloud){
    ROS_INFO("SyncedCallback");

    // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_obstacle_pointcloud, *cloud);

    // Build KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    // Filter objects based on point cloud count within the bounding box
    autoware_msgs::DetectedObjectArray filtered_objects;
    autoware_msgs::DetectedObjectArray removed_objects;
    filtered_objects.header = input_objects->header;
    removed_objects.header = input_objects->header;
    for (const auto &object : input_objects->objects) {
        // Get the center of the bounding box
        // ROS_INFO("object.label: %s", object.label.c_str());
        float center_x = object.pose.position.x;
        float center_y = object.pose.position.y;
        float center_z = object.pose.position.z;

        // Get the dimensions of the bounding box
        float size_x = object.dimensions.x;
        float size_y = object.dimensions.y;
        float size_z = object.dimensions.z;

        // Calculate the search radius
        auto search_radius = getMaxRadius(object);

        // Search for points within the search radius
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        pcl::PointXYZ search_point(center_x, center_y, center_z);
        kdtree->radiusSearch(search_point, search_radius, point_indices, point_distances);

        // Count points within the bounding box
        int points_in_box = 0;
        for (int index : point_indices) {
            pcl::PointXYZ point = cloud->points[index];
            if (point.x >= center_x - size_x / 2.0 && point.x <= center_x + size_x / 2.0 &&
                point.y >= center_y - size_y / 2.0 && point.y <= center_y + size_y / 2.0 &&
                point.z >= center_z - size_z / 2.0 && point.z <= center_z + size_z / 2.0) {
                points_in_box++;
            }
        }

        // Get the threshold point cloud count
        size_t threshold_num = getThresholdPointCloud(object);

        // Check if the point cloud count is within the threshold
        if (points_in_box >= threshold_num) {
            ROS_INFO("filtered object");
            filtered_objects.objects.push_back(object);
        } else {
            ROS_INFO("removed object");
            removed_objects.objects.push_back(object);
        }
    }

    // Publish the filtered objects
    object_pub_.publish(filtered_objects);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_pointcloud_validator");
  ROS_INFO("obstacle_pointcloud_validator_node");
  ObstacleValidatorNode app;
  app.init();
  app.createRosPubSub();
  ros::spin();
  return 0;
}
