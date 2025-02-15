#include "ray_ground_filter.hpp"
#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;


RayGroundRemove::RayGroundRemove() { }

RayGroundRemove::~RayGroundRemove() { }


void RayGroundRemove::groundRemove(const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pRemovedGroundCloud, PointCloudXYZI::Ptr& pGroundCloud)
{
	// outlier filter
	pcl::RadiusOutlierRemoval<pcl::PointXYZI> rorfilter (true); // Initializing with true will allow us to extract the removed indices

	PointCloudXYZI::Ptr pFilteredCloud (new PointCloudXYZI);
	rorfilter.setInputCloud (pInputCloud);
	rorfilter.setRadiusSearch (0.1);
        rorfilter.setMinNeighborsInRadius (10);
	rorfilter.setNegative (true);
	rorfilter.filter (*pFilteredCloud);


	//PointCloudXYZI::Ptr pClipCloud (new PointCloudXYZI);
	ClipCloud(pFilteredCloud, clipping_height_, pFilteredCloud);
        RemovePointsUpTo(pFilteredCloud, min_point_distance_,pFilteredCloud);

	PointCloudXYZRTColor organized_points;
	std::vector<pcl::PointIndices> radial_division_indices;
	std::vector<pcl::PointIndices> closest_indices;
	std::vector<PointCloudXYZRTColor> radial_ordered_clouds;

	//radial_dividers_num_ = ceil(360.0 / radial_divider_angle_);
	radial_dividers_num_ = 3600;

	// Convert xyz to rtz color
	ConvertXYZToRTZColor(pFilteredCloud, organized_points, radial_division_indices, radial_ordered_clouds);

	// Classify ground
	pcl::PointIndices ground_indices, no_ground_indices;
	ClassifyPointCloud(radial_ordered_clouds, ground_indices, no_ground_indices);


	ExtractPointsIndices(pFilteredCloud, ground_indices, pRemovedGroundCloud);
  ExtractPointsIndices(pFilteredCloud, no_ground_indices, pGroundCloud);
}

void RayGroundRemove::ConvertXYZToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
		PointCloudXYZRTColor& out_organized_points,
		std::vector<pcl::PointIndices>& out_radial_divided_indices,
		std::vector<PointCloudXYZRTColor>& out_radial_ordered_clouds)
{
	out_organized_points.resize(in_cloud->points.size());
	out_radial_divided_indices.clear();
	out_radial_divided_indices.resize(radial_dividers_num_);
	out_radial_ordered_clouds.resize(radial_dividers_num_);


	for(size_t i=0; i< in_cloud->points.size(); i++)
	{
		PointXYZRTColor new_point;
		auto radius         = (float) sqrt(
				in_cloud->points[i].x*in_cloud->points[i].x
				+ in_cloud->points[i].y*in_cloud->points[i].y
				);
		auto theta          = (float) atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
		if (theta < 0){ theta+=360; }

		auto radial_div     = (size_t) floor(theta/radial_divider_angle_);
		auto concentric_div = (size_t) floor(fabs(radius/concentric_divider_distance_));

		new_point.point    = in_cloud->points[i];
		new_point.radius   = radius;
		new_point.theta    = theta;
		new_point.radial_div = radial_div;
		new_point.concentric_div = concentric_div;
		new_point.original_index = i;

		out_organized_points[i] = new_point;

		//radial divisions
		out_radial_divided_indices[radial_div].indices.push_back(i);

		out_radial_ordered_clouds[radial_div].push_back(new_point);

	}//end for

	//order radial points on each division
#pragma omp for
	for(size_t i=0; i< radial_dividers_num_; i++)
	{
		std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
				[](const PointXYZRTColor& a, const PointXYZRTColor& b){ return a.radius < b.radius; });
	}
}

void RayGroundRemove::ClassifyPointCloud(std::vector<PointCloudXYZRTColor>& in_radial_ordered_clouds,
		pcl::PointIndices& out_ground_indices,
		pcl::PointIndices& out_no_ground_indices)
{
	out_ground_indices.indices.clear();
	out_no_ground_indices.indices.clear();
#pragma omp for
	for (size_t i=0; i < in_radial_ordered_clouds.size(); i++)//sweep through each radial division
	{

		float prev_radius = 0.f;
		float prev_height = - sensor_height_;
		bool prev_ground = false;
		bool current_ground = false;
		for (size_t j=0; j < in_radial_ordered_clouds[i].size(); j++)//loop through each point in the radial div
		{
			float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
			float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
			float current_height = in_radial_ordered_clouds[i][j].point.z;
			float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

			//for points which are very close causing the height threshold to be tiny, set a minimum value
			if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
				height_threshold = min_height_threshold_; 


			//check current point height against the LOCAL threshold (previous point)
			if (current_height <= (prev_height + height_threshold)
					&& current_height >= (prev_height - height_threshold)
			   )
			{
				//Check again using general geometry (radius from origin) if previous points wasn't ground
				if (!prev_ground)
				{
					if(current_height <= (-sensor_height_ + general_height_threshold)
							&& current_height >= (-sensor_height_ - general_height_threshold))
					{
						current_ground = true;
					}
					else
						current_ground = false;
				}
				else
				{
					current_ground = true;
				}
			}
			else
			{
				//check if previous point is too far from previous one, if so classify again
				if (points_distance > reclass_distance_threshold_ &&
						(current_height <= (-sensor_height_ + height_threshold)
						 && current_height >= (-sensor_height_ - height_threshold))
				   )
				{
					current_ground = true;
				}
				else
					current_ground = false;
			}

			if (current_ground)
			{
				out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
				prev_ground=true;
			}
			else
			{
				out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
				prev_ground = false;
			}

			prev_radius = in_radial_ordered_clouds[i][j].radius;
			prev_height = in_radial_ordered_clouds[i][j].point.z;
		}
	}
}

void RayGroundRemove::ExtractPointsIndices(const PointCloudXYZI::Ptr in_cloud_ptr,
		const pcl::PointIndices& in_indices,
		PointCloudXYZI::Ptr out_removed_indices_cloud_ptr)
{
	pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
	extract_ground.setInputCloud(in_cloud_ptr);
	extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(in_indices));

	extract_ground.setNegative(true);//true removes the indices, false leaves only the indices
	extract_ground.filter(*out_removed_indices_cloud_ptr);
}

void RayGroundRemove::ClipCloud(const PointCloudXYZI::Ptr in_cloud_ptr, 
		double in_clip_height, 
		PointCloudXYZI::Ptr out_clipped_cloud_ptr)
{
	pcl::ExtractIndices<pcl::PointXYZI> extractor;
	extractor.setInputCloud (in_cloud_ptr);
	pcl::PointIndices indices;

	for (size_t i=0; i< in_cloud_ptr->points.size(); i++)
	{
		if (in_cloud_ptr->points[i].z > in_clip_height)
		{
			indices.indices.push_back(i);
		}
	}
	extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	extractor.setNegative(true);
	extractor.filter(*out_clipped_cloud_ptr);
}

void RayGroundRemove::RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    double in_min_distance,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud (in_cloud_ptr);
  pcl::PointIndices indices;

#pragma omp for
  for (size_t i=0; i< in_cloud_ptr->points.size(); i++)
  {
    if (sqrt(in_cloud_ptr->points[i].x*in_cloud_ptr->points[i].x +
          in_cloud_ptr->points[i].y*in_cloud_ptr->points[i].y)
        < in_min_distance)
    {
      indices.indices.push_back(i);
    }
  }
  extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  extractor.setNegative(true);//true removes the indices, false leaves only the indices
  extractor.filter(*out_filtered_cloud_ptr);
}
