#include "center_point.h"
#include <memory>
#include "autoware_msgs/DetectedObjectArray.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
CenterPointNode::CenterPointNode()
:nh_("~")
{
  points_data_ = new float[MAX_POINTS_NUM * FEATURE_NUM];
  std::string package_path = ros::package::getPath("center_point");

  std::string model_file_path =  package_path + "/model/rpn_centerhead_sim.plan.8431";
  
  std::cout<<" the model path "<<model_file_path<<std::endl;
  bool verbose = true;
  
  checkCudaErrors(cudaStreamCreate(&stream_));
  checkCudaErrors(cudaMalloc((void **)&d_points_, MAX_POINTS_NUM * FEATURE_NUM * sizeof(float)));
  
   center_point_ptr_= std::make_shared<CenterPoint>(package_path, model_file_path, verbose);
   center_point_ptr_->prepare();

}

CenterPointNode::~CenterPointNode()
{
    delete[] points_data_;
    center_point_ptr_->perf_report();
    checkCudaErrors(cudaFree(d_points_));
    checkCudaErrors(cudaStreamDestroy(stream_));
};
   
void CenterPointNode::init(const CenterPointNodeInitOptions)
{
    nh_.getParam("vel", test_vel);
    
    std::cout<<" vel "<< test_vel<<std::endl;
}

bool CenterPointNode::point2Array(const pcl::PointCloud<PointT>::Ptr cloud, int feature_num){

    if(5!=feature_num)
    {
        ROS_ERROR_STREAM("the feature num is not 5 is " << feature_num);
        return false;
    }
    int i = 0;
    for(auto& point: cloud->points)
    {
        points_data_[i*feature_num] = point.x;
        points_data_[i*feature_num+1] = point.y;
        points_data_[i*feature_num+2] = point.z;
        points_data_[i*feature_num+3] = point.intensity;
        points_data_[i*feature_num+4] = 0.0;
        
        i++;
    }

    return true;
}

bool CenterPointNode::removeLowScoreNu(const Bndbox &box)
{
    // the threshold of every class
    // {"car", "truck", "construction_vehicle", "bus", "trailer", "barrier", "motorcycle", "bicycle", "pedestrian", "traffic_cone"}
    // const std::vector<float> thresholds = {0.4, 0.4, 0.4, 0.3, 0.4, 0.4, 0.5, 0.5, 0.1, 0.1};
    const std::vector<float> thresholds = {0.3, 0.2, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2};
    // const std::vector<float> thresholds = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    // const std::vector<float> thresholds = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1, 0.1};

    int id = static_cast<int>(box.id);
    float score = box.score;
    if (score < thresholds[id]) {
        return true;
    }
    else {
        // 处理默认情况
        return false;
    }
}

void CenterPointNode::pubDetectedObject(const std::vector<Bndbox> &boxes,  const std_msgs::Header& in_header)
{
  autoware_msgs::DetectedObjectArray objects;
  objects.header = in_header;
  for (auto &box : boxes)
  {
    if (removeLowScoreNu(box))
    {
        continue;
    }

    autoware_msgs::DetectedObject object;
    object.header = in_header;
    object.valid = true;

    // geometry_msgs/Pose pose in message
    object.pose_reliable = true;

    object.pose.position.x = box.x;
    object.pose.position.y = box.y;
    object.pose.position.z = box.z;

    // Trained this way
    float yaw = box.rt;
    //yaw += M_PI/2;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
    object.pose.orientation = q;

    // Again: Trained this way
    // geometry_msgs/Vector3 dimensions in message
    object.dimensions.x = box.w;
    object.dimensions.y = box.l;
    object.dimensions.z = box.h;
    object.score = box.score;

    const char* label_names[] = {"car", "truck", "construction_vehicle", "bus", "trailer", "barrier", "motorcycle", "bicycle", "pedestrian", "traffic_cone"};
    object.label = label_names[box.id];
    object.label_id = box.id;
    // if (object.label == "motorcycle" || object.label == "bicycle") {
    //     object.label = "cyclist";
    // }
    // else if(object.label == "construction_vehicle" || object.label == "trailer" || object.label == "barrier" || object.label == "traffic_cone") {
    //     object.label = "unknown";
    // } // only label 'car', 'truck', 'bus', 'pedestrian', 'cyclist' , 'unknown' are valid
    objects.objects.push_back(object);
  }
  pub_objects_.publish(objects);
}

void CenterPointNode::pointsCallBack(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *cloud);
    
    if(cloud->points.size() < 100 || cloud->points.size() > MAX_POINTS_NUM)
    {
        ROS_INFO("the num of point cloud is not right ");
        return ;
    }

    if(point2Array(cloud))
    {
        int length = cloud->points.size() * FEATURE_NUM;
        checkCudaErrors(cudaMemcpy(d_points_, points_data_, length, cudaMemcpyHostToDevice));
        center_point_ptr_->doinfer((void *)d_points_, cloud->points.size(), stream_);
        pubDetectedObject(center_point_ptr_->nms_pred_, msg.header);
    }
    std::cout<<" hello world "<<std::endl;
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud,output); //point cloud msg -> ROS msg
    // output.header.frame_id="velodyne";
    // point_pub_.publish(output); //publish
}

void CenterPointNode::createRosPubSub(){        
    std::cout<<" hello world "<<std::endl;
    point_sub_ = nh_.subscribe("/l5_points", 1, &CenterPointNode::pointsCallBack, this);
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/output/pointclouds", 1);    
    pub_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
}
