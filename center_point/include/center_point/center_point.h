#ifndef CENTER_POINT_H_H
#define CENTER_POINT_H_H

#include <iostream>
#include <ros/ros.h> 
#include <ros/console.h>
#include <pcl/point_cloud.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include "common.h"
#include "centerpoint.h"
#include <ros/package.h>
#define MAX_POINTS_NUM 400000
#define FEATURE_NUM 5

struct CenterPointNodeInitOptions
{

};

using PointT = pcl::PointXYZI;

class CenterPointNode
{
    public:

    CenterPointNode();
    ~CenterPointNode();

    void createRosPubSub();
    void init(const CenterPointNodeInitOptions = CenterPointNodeInitOptions());
    bool point2Array(const pcl::PointCloud<PointT>::Ptr cloud, int feature_num = FEATURE_NUM);
    bool removeLowScoreNu(const Bndbox &box);
    void pubDetectedObject(const std::vector<Bndbox> &boxes,  const std_msgs::Header& in_header);

    private:
    void pointsCallBack(const sensor_msgs::PointCloud2 &msg);
 
    private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher point_pub_;
    ros::Publisher pub_objects_;
    double test_vel = 0.f;
    float *points_data_ = NULL;
    std::shared_ptr<CenterPoint> center_point_ptr_;
    cudaStream_t stream_ = NULL;

    float *d_points_ = nullptr;
    
};

#endif