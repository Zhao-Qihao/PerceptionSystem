#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION  // 预处理器宏定义

#include "bevfusion_ros.hpp"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

RosNode::RosNode(const std::string model_name, const std::string  precision)
  : model_name_(model_name), precision_(precision)
{ 
  
  bevfusion_node_.reset(new BEVFusionNode(model_name_, precision_));
  getTopicName();
  pub_img_ = n_.advertise<sensor_msgs::Image>("/detection/bevfusion/bevfusion_image", 10);
  pub_obj_ = n_.advertise<autoware_msgs::DetectedObjectArray>("/detection/bevfusion/objects", 10);
  sub_cloud_.subscribe(n_, topic_cloud_, 10);
  sub_f_img_.subscribe(n_, topic_img_f_, 10);
  sub_b_img_.subscribe(n_, topic_img_b_, 10);

  sub_fl_img_.subscribe(n_,topic_img_fl_, 10);
  sub_fr_img_.subscribe(n_,topic_img_fr_, 10);
  
  sub_bl_img_.subscribe(n_,topic_img_bl_, 10);
  sub_br_img_.subscribe(n_,topic_img_br_, 10);
  
  sync_ = std::make_shared<Sync>( MySyncPolicy(10), sub_cloud_, 
    sub_f_img_, sub_fl_img_, sub_fr_img_,
    sub_b_img_ ,sub_bl_img_, sub_br_img_); 
  
  sync_->registerCallback(boost::bind(&RosNode::callback,this, _1, _2,_3, _4, _5, _6,_7)); // 绑定回调函数
  
  size_t img_size = 1600 * 900 * 3; // 计算所需内存大小
  front_buffer.resize(img_size);          // 确保容量足够
  front_left_buffer.resize(img_size);
  front_right_buffer.resize(img_size);
  back_buffer.resize(img_size);
  back_left_buffer.resize(img_size);
  back_right_buffer.resize(img_size);
  images_.reserve(6);
  }


void RosNode::getTopicName()
{
  n_.param<std::string>("topic_cloud", topic_cloud_, "/lidar_top");
  
  n_.param<std::string>("topic_img_f", topic_img_f_, "/cam_front/raw");
  n_.param<std::string>("topic_img_b", topic_img_b_, "/cam_back/raw");
  
  n_.param<std::string>("topic_img_fl", topic_img_fl_, "/cam_front_left/raw");
  n_.param<std::string>("topic_img_fr", topic_img_fr_, "/cam_front_right/raw");
  
  n_.param<std::string>("topic_img_bl", topic_img_bl_, "/cam_back_left/raw");
  n_.param<std::string>("topic_img_br", topic_img_br_, "/cam_back_right/raw");
}


void RosNode::callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, 
  const sensor_msgs::ImageConstPtr& msg_f_img,
  const sensor_msgs::ImageConstPtr& msg_fl_img,
  const sensor_msgs::ImageConstPtr& msg_fr_img,
  const sensor_msgs::ImageConstPtr& msg_b_img,
  const sensor_msgs::ImageConstPtr& msg_bl_img,
  const sensor_msgs::ImageConstPtr& msg_br_img)
{
  auto start = ros::Time::now();
  // 将 cv::Mat 对象 front_img 的原始像素数据 逐字节复制到预分配的 front_buffer 缓冲区 中。
  // 这里front_img.data为起始位置， front_img.data + front_size为结束位置，而 front_buffer.data()为目标起始位置。
  // 结束地址 = 起始地址 + (总像素数 × 每像素字节数) = front_img.data + front_size
  memcpy(front_buffer.data(), msg_f_img->data.data(), image_size);
  memcpy(front_left_buffer.data(), msg_fl_img->data.data(), image_size);
  memcpy(front_right_buffer.data(), msg_fr_img->data.data(), image_size);
  memcpy(back_buffer.data(), msg_b_img->data.data(), image_size);
  memcpy(back_left_buffer.data(), msg_bl_img->data.data(), image_size);
  memcpy(back_right_buffer.data(), msg_br_img->data.data(), image_size);
  // ros::Time memcpy = ros::Time::now();
  // std::cout << "memcpy time: " << (memcpy - start).toSec() * 1000 << " ms\n";

  // 将缓冲区加入 images_
  images_.clear();
  images_.push_back(front_buffer);
  images_.push_back(front_right_buffer);
  images_.push_back(front_left_buffer);
  images_.push_back(back_buffer);
  images_.push_back(back_right_buffer);
  images_.push_back(back_left_buffer);

  ros::Time image_end = ros::Time::now();
  std::cout << "image preprocessing time: " << (image_end - start).toSec() * 1000 << " ms\n";
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg_cloud, *cloud_ptr);
  int lidar_num = cloud_ptr->points.size();
  float lidar_arr[lidar_num * 5];
  for(size_t i = 0; i < cloud_ptr->points.size(); ++i )
  {
    long index = i * 5;
    lidar_arr[index]     = cloud_ptr->points[i].x;
    lidar_arr[index + 1] = cloud_ptr->points[i].y;
    lidar_arr[index + 2] = cloud_ptr->points[i].z;
    lidar_arr[index + 3] = cloud_ptr->points[i].intensity;
    // lidar_arr[index + 4] = cloud->points[i].time;
    lidar_arr[index + 4] = 0;
  }
  ros::Time lidar_end = ros::Time::now();
  std::cout << "lidar preprocessing time: " << (lidar_end - image_end).toSec() * 1000 << " ms\n";
  auto boxes = bevfusion_node_->Inference(images_, lidar_arr, cloud_ptr->points.size());
  pubDetected(boxes, msg_cloud);

  // cv::Mat img = cv::imread((pkg_path + "/configs/cuda-bevfusion.jpg").c_str());
  // cv::resize(img, img, cv::Size(img.size().width /2, img.size().height /2));
  // sensor_msgs::Image::Ptr msg_img_new; 
  // msg_img_new = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  // pub_img_.publish(msg_img_new);
  auto end = ros::Time::now();
  std::cout << "Inference time: " << (end - start).toSec() * 1000 << " ms\n";
}

void RosNode::pubDetected(std::vector<bevfusion::head::transbbox::BoundingBox>& boxes, const sensor_msgs::PointCloud2ConstPtr& msg_cloud) {
    autoware_msgs::DetectedObjectArray objects;
    objects.header = msg_cloud->header;

    for (const auto& box : boxes) {
        autoware_msgs::DetectedObject object;  // 每次循环重新创建对象
        object.header = msg_cloud->header;
        object.valid = true;

        // 坐标与尺寸转换
        object.pose.position.x = box.position.x;
        object.pose.position.y = box.position.y;
        object.pose.position.z = box.position.z;
        object.dimensions.x = box.size.w;
        object.dimensions.y = box.size.l;
        object.dimensions.z = box.size.h;

        // 方向转换
        const float yaw = box.z_rotation;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
        object.pose.orientation = q;

        // 速度与置信度
        object.velocity.linear.x = box.velocity.vx;
        object.velocity.linear.y = box.velocity.vy;
        object.score = box.score;

        // 类别标签
        static const char* label_names[] = {
            "car", "truck", "construction_vehicle", "bus", "trailer",
            "barrier", "motorcycle", "bicycle", "pedestrian", "traffic_cone"
        };
        if (box.id < sizeof(label_names)/sizeof(label_names[0])) {
            object.label = label_names[box.id];
        } else {
            object.label = "unknown";  // 处理越界情况
        }

        objects.objects.push_back(object);
    }

    // 发布消息
    pub_obj_.publish(objects);
}