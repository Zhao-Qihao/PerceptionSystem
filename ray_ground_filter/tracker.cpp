#include "tracker.hpp"
#include <iostream>
#include <string>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

Tracker::Tracker()
{
            // define publisher
            pub_result = nh.advertise<PointCloudXYZI> ("output", 1);
	// define publisher
        pub_result = nh.advertise<PointCloudXYZI> ("output", 1);
        pub_shape = nh.advertise<visualization_msgs::MarkerArray>("Shape", 1);
        pub_detectedObject = nh.advertise<kusv_msgs::DetectedObjectArray>("DetectedObject", 1000);
        pub_Origin = nh.advertise<visualization_msgs::Marker> ("Origin", 1);

        // define subsciber
        sub_velodyne = nh.subscribe ("input", 1, &Tracker::velodyne_callback, this);

                // set parameter
                setParameter ();

                // Coordinate
                double abs = sqrt(pow(m_tf_x, 2.0) + pow(m_tf_y, 2.0) + pow(m_tf_z, 2.0));
                m_tf_x /= abs;
                m_tf_y /= abs;
                m_tf_z /= abs;
}

Tracker::~Tracker() { }

void Tracker::mainLoop()
{
        ros::Rate loop_rate(50);

        while(ros::ok())
        {
                ros::spinOnce();
                loop_rate.sleep();
        }
}


void Tracker::velodyne_callback (const sensor_msgs::PointCloud2ConstPtr &pInput)
{
        // Container for input data
        PointCloudXYZI::Ptr pInputCloud (new PointCloudXYZI);
        pcl::fromROSMsg(*pInput, *pInputCloud);
        m_velodyne_header = pInput->header;

        // Thresholding
        PointCloudXYZI::Ptr pThresholdCloud (new PointCloudXYZI);
        thresholding(pInputCloud, pThresholdCloud);

        // downsample
        PointCloudXYZI::Ptr pDownsampledCloud (new PointCloudXYZI);
        downsample(pThresholdCloud, pDownsampledCloud, m_fLeafSize);

        // Remove ground
        PointCloudXYZI::Ptr pRemovedGroundCloud (new PointCloudXYZI);
        m_RayGroundRemove.groundRemove(pDownsampledCloud, pRemovedGroundCloud);

        // dbscan
        std::vector<pcl::PointIndices> vecClusterIndices;
        dbscan(pRemovedGroundCloud, vecClusterIndices);

        // Set cluster pointcloud from clusterIndices and coloring
        setCluster (vecClusterIndices, m_OriginalClusters, pRemovedGroundCloud);

        // Associate and kalman filter tracking
        m_ObstacleTracking.associate(m_OriginalClusters);

        // Display shape
        displayShape (m_ObstacleTracking.m_TrackingObjects);

        // publish
        publish();
}


void Tracker::setParameter()
{
            // set parameter
            nh.param ("lidar_detection/Marker_duration", m_fMarkerDuration, 0.1);
            nh.param ("lidar_detection/Voxel_leafsize", m_fLeafSize, 0.2);
            nh.param ("lidar_detection/threshold_range", m_dRange_m, 15.0);
            nh.param ("lidar_detection/cluster_err_range", m_dClusterErrRadius, 0.5);
            nh.param ("lidar_detection/general_max_slope", m_RayGroundRemove.general_max_slope_, 7.0);
            nh.param ("lidar_detection/clipping_height", m_RayGroundRemove.clipping_height_, 0.2);
            nh.param ("lidar_detection/cluster_min_size", m_dClusterMinSize, 15.0);
            nh.param ("lidar_detection/cluster_max_size", m_dClusterMaxSize, 50.0);
}


void Tracker::thresholding (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pCloudThresholded)
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
                    if ((distance < m_dRange_m) && (distance > 2.0) && (fabs(p.y) < 5.0))
                            pCloudThresholded->push_back (p);
            }
}


void Tracker::downsample (const PointCloudXYZI::ConstPtr& pInputCloud, PointCloudXYZI::Ptr& pDownsampledCloud, float f_paramLeafSize_m)
{
        pDownsampledCloud->clear();

        // Voxel length of the corner : fLeafSize
        pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
        voxelFilter.setInputCloud (pInputCloud);
        voxelFilter.setLeafSize(f_paramLeafSize_m, f_paramLeafSize_m, f_paramLeafSize_m);
        voxelFilter.filter (*pDownsampledCloud);
}


void Tracker::dbscan(const PointCloudXYZI::ConstPtr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices)
{
        //////////////////////////////////////////////////////////////////
        // DBSCAN

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr pKdtreeDownsampledCloud (new pcl::search::KdTree<pcl::PointXYZI>);
        pKdtreeDownsampledCloud->setInputCloud (pInputCloud);

        // DBSCAN object and parameter setting
        // Tolerance is the length from core point to query point
        // Min cluster size is the minimum number of points in the circle with the tolerance as radius
        // Max cluster size is the maximum number of points in the circle with the tolerance as radius
        // extract the index of each cluster to vecClusterIndices
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclideanCluster;
        euclideanCluster.setClusterTolerance (m_dClusterErrRadius);
        euclideanCluster.setMinClusterSize (m_dClusterMinSize);
        euclideanCluster.setMaxClusterSize (m_dClusterMaxSize);
        euclideanCluster.setSearchMethod (pKdtreeDownsampledCloud);
        euclideanCluster.setInputCloud (pInputCloud);
        euclideanCluster.extract (vecClusterIndices);
}


void Tracker::setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, std::vector<clusterPtr>& pOriginalClusters, const PointCloudXYZI::Ptr pInputCloud)
{
        pOriginalClusters.clear();

        int objectNumber = 0;
        for (const auto& clusterIndice : vecClusterIndices)
        {
                std::string label_ = "Obstacle ";
                std::string label = label_;
                label.append (std::to_string(objectNumber));

                // generate random color to globalRGB variable
                generateColor(vecClusterIndices.size());

                // pCluster is a local cluster
                clusterPtr pCluster (new Cluster());

                // Cloring and calculate the cluster center point and quaternion
                pCluster->SetCloud(pInputCloud, clusterIndice.indices, m_velodyne_header, objectNumber, m_globalRGB[objectNumber].m_r, m_globalRGB[objectNumber].m_g, m_globalRGB[objectNumber].m_b, label, true);

                pOriginalClusters.push_back(pCluster);

                objectNumber++;
        }
}


// generate random color
void Tracker::generateColor(size_t indexNumber)
{
        if (indexNumber > m_maxIndexNumber)
        {
                int addRGB = indexNumber - m_maxIndexNumber;
                m_maxIndexNumber = indexNumber;

                for (size_t i = 0; i < addRGB; i++)
                {
                        uint8_t r = 1024 * rand () % 255;
                        uint8_t g = 1024 * rand () % 255;
                        uint8_t b = 1024 * rand () % 255;
                        m_globalRGB.push_back(RGB(r, g, b));
                }
        }
}


void Tracker::displayShape (const std::vector<clusterPtr> pVecClusters)
{
        // origin
        m_Origin.header.frame_id = m_velodyne_header.frame_id;
        m_Origin.header.stamp = ros::Time::now();

        m_Origin.ns = "/origin";
        m_Origin.id = 0;

        m_Origin.type = visualization_msgs::Marker::SPHERE;

        m_Origin.action = visualization_msgs::Marker::ADD;

        m_Origin.pose.position.x = 0;
        m_Origin.pose.position.y = 0;
        m_Origin.pose.position.z = 0;
        m_Origin.pose.orientation.x = 0.0;
        m_Origin.pose.orientation.y = 0.0;
        m_Origin.pose.orientation.z = 0.0;
        m_Origin.pose.orientation.w = 1.0;

        m_Origin.scale.x = 0.5;
        m_Origin.scale.y = 0.5;
        m_Origin.scale.z = 0.5;

        m_Origin.color.r = 0.0f;
        m_Origin.color.g = 1.0f;
        m_Origin.color.b = 0.0f;
        m_Origin.color.a = 1.0;

        m_Origin.lifetime = ros::Duration();

        // tracking objects
        m_arrShapes.markers.clear();
        m_arrObjects.objects.clear();

        uint32_t objectNumber = 0;
        for (auto pCluster : pVecClusters)
        {
                bool isClusterValid = pCluster->m_valid_cluster;

                if (isClusterValid)
                {
                        visualization_msgs::Marker shape;
                        kusv_msgs::DetectedObject object;

                        shape.lifetime = ros::Duration(m_fMarkerDuration);
                        shape.header.frame_id = m_velodyne_header.frame_id;
                        shape.id = objectNumber;


                        //shape.color.b = (float)(*cluster_it)->m_b/255.0f;
                        shape.color.r = 0.0;
                        shape.color.g = 1.0;
                        shape.color.b = 0.0;
                        shape.color.a = 0.5;

                        shape.type = visualization_msgs::Marker::LINE_STRIP;
                        shape.action = visualization_msgs::Marker::ADD;
                        shape.ns = "/Polygon";
                        shape.scale.x = 0.08;

                        for (auto const &point: pCluster->m_polygon.polygon.points)
                        {
                                geometry_msgs:: Point tmpPoint;
                                tmpPoint.x = point.x;
                                tmpPoint.y = point.y;
                                tmpPoint.z = point.z;

                                object.convex_hull.push_back (tmpPoint);
                                shape.points.push_back (tmpPoint);
                        }

                        m_arrShapes.markers.push_back(shape);

//			shape.type = visualization_msgs::Marker::CUBE;
//			shape.ns = "/BoundingBox";
//			shape.points.clear();
//			shape.pose.position = pCluster->m_center.position;
//			shape.pose.orientation = pCluster->m_center.orientation;
//			object.pose = shape.pose;
//
//			m_arrShapes.markers.push_back(shape);

			shape.scale.x = 1.0;
			shape.scale.y = 1.0;
			shape.scale.z = 1.0;
			shape.points.clear();
			shape.pose.position = pCluster->m_center.position;
			shape.pose.orientation = pCluster->m_center.orientation;
			shape.color.r = shape.color.g = shape.color.b = 1.0;
			shape.color.a = 1.0;
			shape.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			shape.ns = "/Text";
			double distance = sqrt(pow(shape.pose.position.x, 2.0) + pow (shape.pose.position.y, 2.0));
			shape.text = std::to_string(distance);
			//shape.text = std::to_string(pCluster->m_id);

			object.pose = shape.pose;
			object.id = pCluster->m_id;
			object.label = shape.text;

			m_arrShapes.markers.push_back (shape);
			m_arrObjects.objects.push_back (object);
		}
		objectNumber++;
        }
}


void Tracker::publish ()
{
	// Accumulate all cluster to pAccumulationCloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulationCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pAccumulationCloud->header.frame_id = m_velodyne_header.frame_id;

	// accumulation for publish
	for (const auto& pCluster : m_OriginalClusters)
		*pAccumulationCloud += *(pCluster->GetCloud());

	// broadcast tf
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(m_tf_x, m_tf_y, m_tf_z) );
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0);
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "car", "velodyne"));

	// publish
	pub_result.publish (*pAccumulationCloud);
	pub_shape.publish (m_arrShapes);
	pub_detectedObject.publish (m_arrObjects);
        pub_Origin.publish(m_Origin);
}
