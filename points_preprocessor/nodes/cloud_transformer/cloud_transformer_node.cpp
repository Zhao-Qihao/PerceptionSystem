/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class CloudTransformerNode
{
private:

  ros::NodeHandle     node_handle_;
  ros::Subscriber     points_node_sub_;
  ros::Publisher      transformed_points_pub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>   *input_point_subscriber_, *target_point_subscriber_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> SyncPolicyT;
  message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

  std::string         input_point_topic_;
  std::string         target_point_topic_;
  std::string         target_frame_;
  std::string         output_point_topic_;

  tf::TransformListener *tf_listener_ptr_;

  bool                transform_ok_;

  void publish_cloud(const ros::Publisher& in_publisher,
                     const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
  {
    in_publisher.publish(in_cloud_msg);
  }
//note-tianyu-note

  void transformXYZIRCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& in_cloud,
                           pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& out_cloud,
                           const tf::StampedTransform& in_tf_stamped_transform)
  {
    Eigen::Matrix4f transform;
    pcl_ros::transformAsMatrix(in_tf_stamped_transform, transform);

    if (&in_cloud != &out_cloud)
    {
      out_cloud.header   = in_cloud.header;
      out_cloud.is_dense = in_cloud.is_dense;
      out_cloud.width    = in_cloud.width;
      out_cloud.height   = in_cloud.height;
      out_cloud.points.reserve (out_cloud.points.size ());
      out_cloud.points.assign (in_cloud.points.begin (), in_cloud.points.end ());
      out_cloud.sensor_orientation_ = in_cloud.sensor_orientation_;
      out_cloud.sensor_origin_      = in_cloud.sensor_origin_;
      }
    if (in_cloud.is_dense)
      {
      for (size_t i = 0; i < out_cloud.points.size (); ++i)
        {
        //out_cloud.points[i].getVector3fMap () = transform * in_cloud.points[i].getVector3fMap ();
        Eigen::Matrix<float, 3, 1> pt (in_cloud[i].x, in_cloud[i].y, in_cloud[i].z);
        out_cloud[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) +
                          transform (0, 1) * pt.coeffRef (1) +
                          transform (0, 2) * pt.coeffRef (2) +
                          transform (0, 3));
        out_cloud[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) +
                          transform (1, 1) * pt.coeffRef (1) +
                          transform (1, 2) * pt.coeffRef (2) +
                          transform (1, 3));
        out_cloud[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) +
                          transform (2, 1) * pt.coeffRef (1) +
                          transform (2, 2) * pt.coeffRef (2) +
                          transform (2, 3));
        }
      }
    else
    {
      // Dataset might contain NaNs and Infs, so check for them first,
      for (size_t i = 0; i < out_cloud.points.size (); ++i)
      {
        if (!pcl_isfinite (in_cloud.points[i].x) ||
                   !pcl_isfinite (in_cloud.points[i].y) ||
                   !pcl_isfinite (in_cloud.points[i].z))
          {continue;}
        //out_cloud.points[i].getVector3fMap () = transform * in_cloud.points[i].getVector3fMap ();
        Eigen::Matrix<float, 3, 1> pt (in_cloud[i].x, in_cloud[i].y, in_cloud[i].z);
        out_cloud[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) +
                          transform (0, 1) * pt.coeffRef (1) +
                          transform (0, 2) * pt.coeffRef (2) +
                          transform (0, 3));
        out_cloud[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) +
                          transform (1, 1) * pt.coeffRef (1) +
                          transform (1, 2) * pt.coeffRef (2) +
                          transform (1, 3));
        out_cloud[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) +
                          transform (2, 1) * pt.coeffRef (1) +
                          transform (2, 2) * pt.coeffRef (2) +
                          transform (2, 3));
      }
    }
  }
  //note-tianyu 去除掉半径内的点云(横向半径保留一部分)
  void RemoveCloseCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr& in_cloud_ptr,
                           pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr& out_cloud_ptr)
  {
      out_cloud_ptr->points.clear();
      for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
      {
          if(std::hypot(std::abs(in_cloud_ptr->points[i].x),std::abs(in_cloud_ptr->points[i].y))<2.7 
             && std::abs(in_cloud_ptr->points[i].y)<1.25)
            continue;

          if(!isnan(in_cloud_ptr->points[i].x) && !isnan(in_cloud_ptr->points[i].y) && !isnan(in_cloud_ptr->points[i].z))
          {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
          }
      }
  }
/*
  void CloudCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_sensor_cloud)
  {
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr transformed_cloud_ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

    bool do_transform = false;
    tf::StampedTransform transform;
    if (target_frame_ != in_sensor_cloud->header.frame_id)
    {
      try {
        tf_listener_ptr_->lookupTransform(target_frame_, in_sensor_cloud->header.frame_id, ros::Time(0),
                                          transform);
        do_transform = true;
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("cloud_transformer: %s NOT Transforming.", ex.what());
        do_transform = false;
        transform_ok_ = false;
      }
    }
    if (do_transform)
    {
      transformXYZIRCloud(*in_sensor_cloud, *transformed_cloud_ptr, transform);
      transformed_cloud_ptr->header.frame_id = target_frame_;
      if (!transform_ok_)
        {ROS_INFO("cloud_transformer: Correctly Transformed"); transform_ok_=true;}
    }
    else
      { pcl::copyPointCloud(*in_sensor_cloud, *transformed_cloud_ptr);}

    publish_cloud(transformed_points_pub_, transformed_cloud_ptr);
  }
*/



  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_sensor_cloud, const sensor_msgs::PointCloud2::ConstPtr &target_sensor_cloud)
  {
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr input_cloud_ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr transformed_cloud_ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr target_cloud_ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr merge_cloud_ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr filter_merge_cloud_ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

    pcl::fromROSMsg(*in_sensor_cloud, *input_cloud_ptr);
    pcl::fromROSMsg(*target_sensor_cloud, *target_cloud_ptr);

    bool do_transform = false;
    tf::StampedTransform transform;
    if (target_frame_ != in_sensor_cloud->header.frame_id)
    {
      try {
        tf_listener_ptr_->lookupTransform(target_frame_, input_cloud_ptr->header.frame_id, ros::Time(0),
                                          transform);
        do_transform = true;
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("cloud_transformer: %s NOT Transforming.", ex.what());
        do_transform = false;
        transform_ok_ = false;
      }
    }
    
    if (do_transform)
    {
      transformXYZIRCloud(*input_cloud_ptr, *transformed_cloud_ptr, transform);
      transformed_cloud_ptr->header.frame_id = target_frame_;
      if (!transform_ok_)
        {ROS_INFO("cloud_transformer: Correctly Transformed"); transform_ok_=true;}
    }
    else
      { pcl::copyPointCloud(*input_cloud_ptr, *transformed_cloud_ptr);}

    *merge_cloud_ptr = *target_cloud_ptr + *transformed_cloud_ptr;
    RemoveCloseCloud(merge_cloud_ptr, filter_merge_cloud_ptr);
    filter_merge_cloud_ptr->header.frame_id = target_frame_;

    publish_cloud(transformed_points_pub_, filter_merge_cloud_ptr);
  }

public:
  CloudTransformerNode(tf::TransformListener* in_tf_listener_ptr):node_handle_("~"), transform_ok_(false)
  {
    tf_listener_ptr_ = in_tf_listener_ptr;
  }
  void Run()
  {
    ROS_INFO("Initializing Cloud Transformer, please wait...");
    node_handle_.param<std::string>("input_point_topic", input_point_topic_, "/points_raw_front");
    ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

    node_handle_.param<std::string>("target_point_topic", target_point_topic_, "/points_raw_up");
    ROS_INFO("Input point_topic: %s", target_point_topic_.c_str());

    node_handle_.param<std::string>("target_frame", target_frame_, "velodyne");
    ROS_INFO("Target Frame in TF (target_frame) : %s", target_frame_.c_str());

    node_handle_.param<std::string>("output_point_topic", output_point_topic_, "/points_transformed");
    ROS_INFO("output_point_topic: %s", output_point_topic_.c_str());

    ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
    //points_node_sub_ = node_handle_.subscribe(input_point_topic_, 1, &CloudTransformerNode::CloudCallback, this);
    ROS_INFO("Subscribing to... %s", target_point_topic_.c_str());
    //points_node_sub_ = node_handle_.subscribe(target_point_topic_, 1, &CloudTransformerNode::TargetCloudCallback, this);

    //note-tianyu 同步两个激光雷达传感器信号
    input_point_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                        input_point_topic_, 10);

    target_point_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                            target_point_topic_, 10);

    cloud_synchronizer_ =
        new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
                                                      *input_point_subscriber_,
                                                      *target_point_subscriber_);
    cloud_synchronizer_->registerCallback(boost::bind(&CloudTransformerNode::CloudCallback, this, _1, _2));


    transformed_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(output_point_topic_, 2);

    ROS_INFO("Ready");

    ros::spin();

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_transformer");
  tf::TransformListener tf_listener;
  CloudTransformerNode app(&tf_listener);

  app.Run();

  return 0;

}
