#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "ray_ground_filter.hpp"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
double m_dRange_m;

std_msgs::Header m_velodyne_header;
RayGroundRemove m_RayGroundRemove;
PointCloudXYZI::Ptr pRemovedGroundCloud (new PointCloudXYZI);
PointCloudXYZI::Ptr pGroundCloud (new PointCloudXYZI);

void velodyne_callback (const sensor_msgs::PointCloud2ConstPtr &pInput);
void thresholding (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pCloudThresholded);

void velodyne_callback (const sensor_msgs::PointCloud2ConstPtr &pInput)
{
        // Container for input data
        PointCloudXYZI::Ptr pInputCloud (new PointCloudXYZI);
        pcl::fromROSMsg(*pInput, *pInputCloud);
        m_velodyne_header = pInput->header;

        // Thresholding
        PointCloudXYZI::Ptr pThresholdCloud (new PointCloudXYZI);
        thresholding(pInputCloud, pThresholdCloud);

        // Remove ground
        m_RayGroundRemove.groundRemove(pThresholdCloud, pRemovedGroundCloud, pGroundCloud);
        pRemovedGroundCloud->header.frame_id = m_velodyne_header.frame_id;
        pGroundCloud->header.frame_id = m_velodyne_header.frame_id;
}

void thresholding (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pCloudThresholded)
{
            // set a pointer cloud thresholded
            for (const auto& point : pInputCloud->points)
            {
                    pcl::PointXYZI p;
                    p.x = (double)point.x;
                    p.y = (double)point.y;
                    p.z = (double)point.z;
                    p.intensity = point.intensity;

                    double distance = sqrt(pow(p.x,2) + pow(p.y,2));
                    //if ((distance < m_dRange_m) && (distance > 2.0) && (fabs(p.y) < 5.0) && p.z < 0)
                    if ((distance < m_dRange_m) && (distance > 1.5) && (fabs(p.y) < m_dRange_m))
                            pCloudThresholded->push_back (p);
            }
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "Ray_Ground_Filter");
  ros::NodeHandle nh;
  ros::Subscriber sub_velodyne;
  ros::Publisher pub_rayGround;
  ros::Publisher pub_Ground;
  std::string input_topic;
  std::string output_topic;

  nh.param ("ray_ground_filter/general_max_slope", m_RayGroundRemove.general_max_slope_, 5.0);
  nh.param ("ray_ground_filter/clipping_height", m_RayGroundRemove.clipping_height_, 0.3);
  nh.param ("ray_ground_filter/threshold_range", m_dRange_m, 30.0);
  nh.param ("ray_ground_filter/input_topic", input_topic, std::string("/kitti/velo/pointcloud"));
  nh.param ("ray_ground_filter/output_topic", output_topic, std::string("/kitti/velo/pub_rayGround"));


//   for (size_t i = 0; i < input_topic.size(); ++i) {
        sub_velodyne = nh.subscribe (input_topic, 1, &velodyne_callback);
        pub_rayGround = nh.advertise<PointCloudXYZI>(output_topic, 1);
        pub_Ground = nh.advertise<PointCloudXYZI>("/Ground", 1);
//   }
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
          ros::spinOnce();
          loop_rate.sleep();
          pub_rayGround.publish(pRemovedGroundCloud);
          pub_Ground.publish(pGroundCloud);
  }

	return 0;
}

// #include <ros/ros.h>
// #include <cmath>
// #include <vector>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>

// #include <sensor_msgs/PointCloud2.h>
// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

// #include <Eigen/Dense>
// #include <tf/tf.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include "ray_ground_filter.hpp"

// typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
// double m_dRange_m;

// std_msgs::Header m_velodyne_header;
// RayGroundRemove m_RayGroundRemove;

// void velodyne_callback (const sensor_msgs::PointCloud2ConstPtr &pInput, const std::string& output_topic, ros::NodeHandle& nh);

// void thresholding (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pCloudThresholded);

// void velodyne_callback (const sensor_msgs::PointCloud2ConstPtr &pInput, const std::string& output_topic, ros::NodeHandle& nh)
// {
//     // Container for input data
//     PointCloudXYZI::Ptr pInputCloud (new PointCloudXYZI);
//     pcl::fromROSMsg(*pInput, *pInputCloud);
//     m_velodyne_header = pInput->header;

//     // Thresholding
//     PointCloudXYZI::Ptr pThresholdCloud (new PointCloudXYZI);
//     thresholding(pInputCloud, pThresholdCloud);

//     // Remove ground
//     PointCloudXYZI::Ptr pRemovedGroundCloud (new PointCloudXYZI);
//     PointCloudXYZI::Ptr pGroundCloud (new PointCloudXYZI);
//     m_RayGroundRemove.groundRemove(pThresholdCloud, pRemovedGroundCloud, pGroundCloud);
//     pRemovedGroundCloud->header.frame_id = m_velodyne_header.frame_id;
//     pGroundCloud->header.frame_id = m_velodyne_header.frame_id;

//     // Publish the results
//     ros::Publisher pub_rayGround = nh.advertise<PointCloudXYZI>(output_topic, 1);
//     pub_rayGround.publish(pRemovedGroundCloud);
// }

// void thresholding (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pCloudThresholded)
// {
//     // set a pointer cloud thresholded
//     for (const auto& point : pInputCloud->points)
//     {
//         pcl::PointXYZI p;
//         p.x = (double)point.x;
//         p.y = (double)point.y;
//         p.z = (double)point.z;
//         p.intensity = point.intensity;

//         double distance = sqrt(pow(p.x,2) + pow(p.y,2));
//         if ((distance < m_dRange_m) && (distance > 1.5) && (fabs(p.y) < m_dRange_m))
//             pCloudThresholded->push_back (p);
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init (argc, argv, "Ray_Ground_Filter");
//     ros::NodeHandle nh;
//     std::vector<std::string> input_topics;
//     std::vector<std::string> output_topics;

//     nh.param ("ray_ground_filter/general_max_slope", m_RayGroundRemove.general_max_slope_, 5.0);
//     nh.param ("ray_ground_filter/clipping_height", m_RayGroundRemove.clipping_height_, 0.3);
//     nh.param ("ray_ground_filter/threshold_range", m_dRange_m, 30.0);
//     nh.getParam ("ray_ground_filter/input_topic", input_topics);
//     nh.getParam ("ray_ground_filter/output_topic", output_topics);

//     if (input_topics.size() != output_topics.size()) {
//         ROS_ERROR("Number of input topics and output topics must be the same.");
//         return -1;
//     }

//     for (size_t i = 0; i < input_topics.size(); ++i) {
//         ros::Subscriber sub_velodyne = nh.subscribe (input_topics[i], 1, std::bind(&velodyne_callback, std::placeholders::_1, output_topics[i], std::ref(nh)));
//     }

//     ros::Rate loop_rate(50);

//     while(ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }