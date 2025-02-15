#ifndef __TRACKER_H__
#define __TRACKER_H__

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
#include "kusv_msgs/DetectedObjectArray.h"

#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "cluster.hpp"
#include "obstacle_tracking.hpp"
#include "ray_ground_filter.hpp"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

typedef struct _rgb RGB;
struct _rgb
{
        uint8_t m_r;
        uint8_t m_g;
        uint8_t m_b;

        _rgb ()
        { }

        _rgb (uint8_t r, uint8_t g, uint8_t b)
        {
                m_r = r;
                m_g = g;
                m_b = b;
        }
};

class Tracker
{
        private:
	// Declare nodehandler
	ros::NodeHandle nh;

	// Declare publisher
	ros::Publisher pub_shape;  
	ros::Publisher pub_result;	
	ros::Publisher pub_detectedObject;
	ros::Publisher pub_Origin;

        // Declare subscriber
	ros::Subscriber sub_velodyne;

	// Attitude
	std_msgs::Header m_velodyne_header;
	std::vector<RGB> m_globalRGB;
	int m_maxIndexNumber = 0;

	// param
	double m_fMarkerDuration;
	double m_fLeafSize;
	double m_dRange_m;
	double m_dClusterMinSize;
	double m_dClusterErrRadius;
	double m_dClusterMaxSize;


	// transformation
	tf::TransformListener *m_pListener;
	Eigen::Matrix4f m_tf_Base2Local, m_tf_Local2Base;
	double m_tf_x = 80.0;
	double m_tf_y = -45.0;
	double m_tf_z = 23.0; 
	double m_tf_roll = 0.0; 
	double m_tf_pitch = 0.0;
	double m_tf_yaw = 0.0;

	std::vector<clusterPtr> m_OriginalClusters;
	ObstacleTracking m_ObstacleTracking;
	RayGroundRemove m_RayGroundRemove;

	visualization_msgs::Marker m_Origin;
	visualization_msgs::MarkerArray m_arrShapes;
	kusv_msgs::DetectedObjectArray m_arrObjects;

	public:
	Tracker();
	~Tracker();
	void mainLoop ();
	void velodyne_callback (const sensor_msgs::PointCloud2ConstPtr &pInput);
	void setParameter ();
	void initTransformation();
	void thresholding (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pCloudThresholded);
	void downsample (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
	void dbscan (const PointCloudXYZI::ConstPtr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
	void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, std::vector<clusterPtr>& pOriginalClusters, const PointCloudXYZI::Ptr pInputCloud);
	void generateColor(size_t indexNumber);
	void displayShape (const std::vector<clusterPtr> pVecClusters);
	void setDetectedObject (const std::vector<clusterPtr>& pVecClusters);
	void publish ();
};

#endif //__TRACKER_H__
