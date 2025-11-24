#include "pointcloud_concatenate/pointcloud_concatenate.hpp"
#include "pointcloud_concatenate/parse_lidar.h"
#include <pcl_conversions/pcl_conversions.h>

// Constructor
PointcloudConcatenate::PointcloudConcatenate(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  nh_ = nh;  // Set nodehandle
  node_name_ = ros::this_node::getName();

  // Initialise variables / parameters to class variables
  handleParams();

  // Initialization tf2 listener

  tfBuffer.reset(new tf2_ros::Buffer);
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));
 
  // Initialise publishers and subscribers
  // Queues size of 1 to only keep the most recent message
  sub_cloud_in1 = nh_.subscribe("/l1_points", 1, &PointcloudConcatenate::subCallbackCloudIn1, this);
  sub_cloud_in2 = nh_.subscribe("/l2_points", 1, &PointcloudConcatenate::subCallbackCloudIn2, this);
  sub_cloud_in3 = nh_.subscribe("/l3_points", 1, &PointcloudConcatenate::subCallbackCloudIn3, this);
  sub_cloud_in4 = nh_.subscribe("cloud_in4", 1, &PointcloudConcatenate::subCallbackCloudIn4, this);
  sub_cloud_in5 = nh_.subscribe("cloud_in5", 1, &PointcloudConcatenate::subCallbackCloudIn5, this);
  sub_cloud_in6 = nh_.subscribe("cloud_in6", 1, &PointcloudConcatenate::subCallbackCloudIn6, this);
  pub_cloud_out = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
}

// Destructor
PointcloudConcatenate::~PointcloudConcatenate() {
  // Free up allocated memory
  ROS_INFO("Destructing PointcloudConcatenate...");
  // delete pointer_name;
}

void PointcloudConcatenate::handleParams() {
  // Handle parameters
  // Set parameters
  ROS_INFO("Loading parameters...");
  std::string param_name;

  // Target frame
  std::string parse_str;
  param_name = node_name_ + "/target_frame";
  ros::param::get(param_name, parse_str);
  if (!parse_str.length() > 0) {
    param_frame_target_ = "/rslidar";
    ROSPARAM_WARN(param_name, param_frame_target_);
  }
  param_frame_target_ = parse_str;


  // livox frame
  param_name = node_name_ + "/livox_frame";
  ros::param::get(param_name, parse_str);
  if (!parse_str.length() > 0) {
    param_frame_target_ = "livox_frame";
    ROSPARAM_WARN(param_name, param_frame_target_);
  }
  livox_target_ = parse_str;

  // robosense frame
  param_name = node_name_ + "/robosense_frame";
  ros::param::get(param_name, parse_str);
  if (!parse_str.length() > 0) {
    param_frame_target_ = "/rslidar";
    ROSPARAM_WARN(param_name, param_frame_target_);
  }
  robosense_target_ = parse_str;

  // livox_target_ = std::string("livox_frame");
  // robosense_target_ = std::string("/rslidar");

  // Number of pointclouds
  param_name = node_name_ + "/clouds";
  if (!ros::param::get(param_name, param_clouds_)) {
    param_clouds_ = 6;
    ROSPARAM_WARN(param_name, param_clouds_);
  }

  // Frequency to update/publish
  param_name = node_name_ + "/hz";
  if (!ros::param::get(param_name, param_hz_)) {
    param_hz_ = 10;
    ROSPARAM_WARN(param_name, param_hz_);
  }

  // use custom format or not for robosense lidar.
  param_name = node_name_ + "/use_custom";
  if (!ros::param::get(param_name, use_custom_)) {
    use_custom_ = true;
    ROSPARAM_WARN(param_name, use_custom_);
  }

  ROS_INFO("Parameters loaded.");
}

void PointcloudConcatenate::subCallbackCloudIn1(sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZI> tmp_xyz_cloud;
  pcl::fromROSMsg(msg, tmp_xyz_cloud);
//坐标变换
  Eigen::Matrix4f guess;
  // guess<< 0.993,0.049, 0.108,  3.252,
  //         -0.049,0.999, 0.001, 0.479,
  //         -0.107,-0.006,0.994, -1.398,
  //          0.0, 0.0, 0.0, 1.0;
    guess<< 0.9963969588,0.07394, 0.0415439,  2.57475996,
          -0.074984,0.99689, 0.024153, -0.166139,
          -0.0396289,-0.02718159,0.998844, -0.8108785,
           0.0, 0.0, 0.0, 1.0;
  transformPointCloud(tmp_xyz_cloud, tmp_xyz_cloud, guess);

  pcl::toROSMsg(tmp_xyz_cloud, cloud_in1);
  cloud_in1.header = msg.header;
  cloud_in1_received = true;
  cloud_in1_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn2(sensor_msgs::PointCloud2 msg) {
  // xyzi
  pcl::PointCloud<pcl::PointXYZI> tmp_xyz_cloud;
  pcl::fromROSMsg(msg, tmp_xyz_cloud);
//坐标变换
  Eigen::Matrix4f guess;
  // guess << 0.996,0.053,0.066,1.127,
  //          -0.053,0.999,0.004,0.561,
  //          -0.066,-0.008,0.998,-1.136,
  //          0.0,0.0,0.0,1.0;
    guess<< 0.993,0.095, 0.074,  0.596,
          -0.092,0.995, -0.040, 0.065,
          -0.078,-0.033,0.996, -0.206,
           0.0, 0.0, 0.0, 1.0;
  transformPointCloud(tmp_xyz_cloud, tmp_xyz_cloud, guess);

  pcl::toROSMsg(tmp_xyz_cloud, cloud_in2);
  cloud_in2.header = msg.header;
  cloud_in2_received = true;
  cloud_in2_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn3(sensor_msgs::PointCloud2 msg) {

  pcl::PointCloud<pcl::PointXYZI> tmp_xyz_cloud;
  pcl::fromROSMsg(msg, tmp_xyz_cloud);
//坐标变换
  Eigen::Matrix4f guess;
  // guess << -0.998,0.023,-0.057,-0.228,
  //          -0.022,-1.000,-0.015,1.404,
  //          -0.058,-0.0014,0.998,-0.349,
  //          0.0,0.0,0.0,1.0;
    guess << -0.989,-0.139,0.050,-0.751,
           0.145,-0.980,-0.136,-0.090,
           0.030,0.142,0.989,-0.725,
           0.0,0.0,0.0,1.0;
  transformPointCloud(tmp_xyz_cloud, tmp_xyz_cloud, guess);

  pcl::toROSMsg(tmp_xyz_cloud, cloud_in3);
  cloud_in3.header = msg.header;
  cloud_in3_received = true;
  cloud_in3_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn4(sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOut(new pcl::PointCloud<pcl::PointXYZI>());
  if(use_custom_){
    pcl::PointCloud<RsPointXYZIRT>::Ptr pointCloudInRS(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(msg, *pointCloudInRS);
    handlePCMsg<RsPointXYZIRT,pcl::PointXYZI>(pointCloudInRS,pointCloudOut);
  }else{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutTmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *pointCloudOutTmp);
    handlePCMsg<pcl::PointXYZI,pcl::PointXYZI>(pointCloudOutTmp,pointCloudOut);
  }
//坐标变换
  Eigen::Matrix4f guess;
  // guess<< -0.997,-0.018,0.072,0.068,
  //         0.069,0.125,0.990,0.741,
  //         -0.027,0.992,-0.124,0.063,
  //         0.0,0.0,0.0,1.0;
    guess<< 0.97133735, -0.2376466, -0.00527683, 0.37390488,
            0.2375424, 0.96961158, 0.05854047, 1.38535938,
            0.00879546, -0.05811602, 0.99827109, -1.3313624,
            0.0, 0.0, 0.0, 1.0;


//   Left transform:
// [[ 0.97133735 -0.2376466   0.00527683  0.37390488]
//  [ 0.2375424   0.96961158 -0.05854047  1.38535938]
//  [ 0.00879546  0.05811602  0.99827109 -1.3313624 ]
//  [ 0.          0.          0.          1.        ]]
  transformPointCloud(*pointCloudOut, *pointCloudOut, guess);

  pcl::toROSMsg(*pointCloudOut, cloud_in4);
  cloud_in4.header = msg.header;
  cloud_in4_received = true;
  cloud_in4_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn5(sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOut(new pcl::PointCloud<pcl::PointXYZI>());
  if(use_custom_){
    pcl::PointCloud<RsPointXYZIRT>::Ptr pointCloudInRS(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(msg, *pointCloudInRS);
    handlePCMsg<RsPointXYZIRT,pcl::PointXYZI>(pointCloudInRS,pointCloudOut);
  }else{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutTmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *pointCloudOutTmp);
    handlePCMsg<pcl::PointXYZI,pcl::PointXYZI>(pointCloudOutTmp,pointCloudOut);
  }

  pcl::toROSMsg(*pointCloudOut, cloud_in5);
  cloud_in5.header = msg.header;
  cloud_in5_received = true;
  cloud_in5_received_recent = true;
}

void PointcloudConcatenate::subCallbackCloudIn6(sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOut(new pcl::PointCloud<pcl::PointXYZI>());
  if(use_custom_){
    pcl::PointCloud<RsPointXYZIRT>::Ptr pointCloudInRS(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(msg, *pointCloudInRS);
    handlePCMsg<RsPointXYZIRT,pcl::PointXYZI>(pointCloudInRS,pointCloudOut);
  }else{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudOutTmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *pointCloudOutTmp);
    handlePCMsg<pcl::PointXYZI,pcl::PointXYZI>(pointCloudOutTmp,pointCloudOut);
  }
//坐标变换
  Eigen::Matrix4f guess;
  // guess << -0.491,0.869,-0.059,-0.102,
  //          0.061,-0.034,-0.998,-0.551,
  //          -0.869,-0.493,-0.037,-0.389,
  //          0.0,0.0,0.0,1.0;
    guess << 0.96916296, 0.24381839, 0.03571737, 0.37637011,
            -0.24562348, 0.96747562, 0.06049805, -1.1833214,
            0.01980515, -0.0674055, 0.99752908, -1.37309376,
             0.0, 0.0, 0.0, 1.0;

//   Right transform:
// [[ 0.96916296  0.24381839  0.03571737  0.37637011]
//  [-0.24562348  0.96747562  0.06049805 -1.1833214 ]
//  [-0.01980515 -0.0674055   0.99752908 -1.37309376]
//  [ 0.          0.          0.          1.        ]]
  transformPointCloud(*pointCloudOut, *pointCloudOut, guess);

  pcl::toROSMsg(*pointCloudOut, cloud_in6);
  cloud_in6.header = msg.header;
  cloud_in6_received = true;
  cloud_in6_received_recent = true;
}

double PointcloudConcatenate::getHz() {
  return param_hz_;
}

void PointcloudConcatenate::update() {
  // Is run periodically and handles calling the different methods

  if (pub_cloud_out.getNumSubscribers() > 0 && param_clouds_ >= 1) {
    // Initialise pointclouds
    sensor_msgs::PointCloud2 cloud_to_concat;
    cloud_out = cloud_to_concat; // Clear the output pointcloud
    
    // Track success of transforms
    bool success = true;

    // Sleep if no pointclouds have been received yet
    if ((!cloud_in1_received) && (!cloud_in2_received) && (!cloud_in3_received) && (!cloud_in4_received)
          && (!cloud_in5_received) && (!cloud_in6_received)) {
      ROS_WARN("No pointclouds received yet. Waiting 1 second...");

      // Set end time
      ros::Time end = ros::Time::now();
      end.sec += 1;
      // Sleep
      ros::Time::sleepUntil(end);

      return;
    }

    
    // Concatenate the first pointcloud
    if (param_clouds_ >= 1 && success && cloud_in1_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in1_received_recent) {
        ROS_WARN("Cloud 1 was not received since last update, reusing last received message...");
      }
      cloud_in1_received_recent = false;

      // Transform pointcloud to the target frame
      // Here we just assign the pointcloud directly to the output to ensure the secondary
      // data is inherited correctly.
      success = pcl_ros::transformPointCloud(livox_target_, cloud_in1, cloud_out, *tfBuffer);

      if (!success) {
        ROS_WARN("Transforming cloud 1 from %s to %s failed!", cloud_in1.header.frame_id.c_str(), param_frame_target_.c_str());
      }
    }

    // Concatenate the second pointcloud
    if (param_clouds_ >= 2 && success && cloud_in2_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in2_received_recent) {
        ROS_WARN("Cloud 2 was not received since last update, reusing last received message...");
      }
      cloud_in2_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(livox_target_, cloud_in2, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 2 from %s to %s failed!", cloud_in2.header.frame_id.c_str(), param_frame_target_.c_str());
      }
      
      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Concatenate the third pointcloud
    if (param_clouds_ >= 3 && success && cloud_in3_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in3_received_recent) {
        ROS_WARN("Cloud 3 was not received since last update, reusing last received message...");
      }
      cloud_in3_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(livox_target_, cloud_in3, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 3 from %s to %s failed!", cloud_in3.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Concatenate the fourth pointcloud
    if (param_clouds_ >= 4 && success && cloud_in4_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in4_received_recent) {
        ROS_WARN("Cloud 4 was not received since last update, reusing last received message...");
      }
      cloud_in4_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(robosense_target_, cloud_in4, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 4 from %s to %s failed!", cloud_in4.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Concatenate the fifth pointcloud
    if (param_clouds_ >= 5 && success && cloud_in5_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in5_received_recent) {
        ROS_WARN("Cloud 5 was not received since last update, reusing last received message...");
      }
      cloud_in5_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(robosense_target_, cloud_in5, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 5 from %s to %s failed!", cloud_in5.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }


    // Concatenate the sixth pointcloud
    if (param_clouds_ >= 6 && success && cloud_in6_received) {
      // Warn if cloud was not received since last update
      if (!cloud_in6_received_recent) {
        ROS_WARN("Cloud 6 was not received since last update, reusing last received message...");
      }
      cloud_in6_received_recent = false;

      // Transform pointcloud to the target frame
      success = pcl_ros::transformPointCloud(robosense_target_, cloud_in6, cloud_to_concat, *tfBuffer);
      if (!success) {
        ROS_WARN("Transforming cloud 6 from %s to %s failed!", cloud_in6.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      // Concatenate the pointcloud
      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    // Publish the concatenated pointcloud
    if (success) {
      publishPointcloud(cloud_out);
    }
  }
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::PointCloud2 cloud) {
  // Publishes the combined pointcloud

  // Update the timestamp
  cloud.header.stamp = ros::Time::now();
  // Publish
  pub_cloud_out.publish(cloud);
}
