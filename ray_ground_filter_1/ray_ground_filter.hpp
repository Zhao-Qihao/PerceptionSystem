#ifndef __RAY_GROUND_FILTER_HPP__
#define __RAY_GROUND_FILTER_HPP__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

struct PointXYZRTColor
{
	pcl::PointXYZI point;

	float radius;       //cylindric coords on XY Plane
	float theta;        //angle deg on XY plane

	size_t radial_div;  //index of the radial divsion to which this point belongs to
	size_t concentric_div;//index of the concentric division to which this points belongs to

	size_t red;         //Red component  [0-255]
	size_t green;       //Green Component[0-255]
	size_t blue;        //Blue component [0-255]

	size_t original_index; //index of this point in the source pointcloud
};
typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;

class RayGroundRemove
{
        public:
        double sensor_height_ = 1.6;//meters
        double general_max_slope_ = 7.0;//degrees
        double local_max_slope_ = 10.0;//degrees
	double radial_divider_angle_ = 0.1;//distance in rads between dividers
        double concentric_divider_distance_ = 0.01;//distance in meters between concentric divisions
        double min_height_threshold_ = 0.05;//minimum height threshold regardless the slope, useful for close points
	double clipping_height_ = 0.2; //the points higher than this will be removed from the input cloud.
        double min_point_distance_ = 2.0;//minimum distance from the origin to consider a point as valid
        double reclass_distance_threshold_ = 0.2;//distance between points at which re classification will occur

        size_t radial_dividers_num_;
	size_t concentric_dividers_num_;

	std::vector<cv::Scalar> colors_;
	const size_t color_num_ = 60;//different number of color to generate

	public:
	RayGroundRemove();
	~RayGroundRemove();
  void groundRemove(const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pRemovedGroundCloud, PointCloudXYZI::Ptr& pGroundCloud);

	void ConvertXYZToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
			PointCloudXYZRTColor& out_organized_points,
			std::vector<pcl::PointIndices>& out_radial_divided_indices,
			std::vector<PointCloudXYZRTColor>& out_radial_ordered_clouds);

	void ClassifyPointCloud(std::vector<PointCloudXYZRTColor>& in_radial_ordered_clouds,
			pcl::PointIndices& out_ground_indices,
			pcl::PointIndices& out_no_ground_indices);

	void ExtractPointsIndices(const PointCloudXYZI::Ptr in_cloud_ptr,
			const pcl::PointIndices& in_indices,
			PointCloudXYZI::Ptr out_removed_indices_cloud_ptr);

	void ClipCloud(const PointCloudXYZI::Ptr in_cloud_ptr, 
			double in_clip_height, 
			PointCloudXYZI::Ptr out_clipped_cloud_ptr);
        void RemovePointsUpTo(const PointCloudXYZI::Ptr in_cloud_ptr,
            double in_min_distance,
            PointCloudXYZI::Ptr out_filtered_cloud_ptr);
};

#endif //__RAY_GROUND_FILTER_HPP__
