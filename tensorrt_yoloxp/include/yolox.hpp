#ifndef YOLOX_HPP
#define YOLOX_HPP

#include <memory>
#include <string>
#include <vector>
#include "yolo_config_parser.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gflags/gflags.h>
#include "autoware_msgs/DetectedObjectArray.h"
#include "tensorrt_yolox/tensorrt_yolox.hpp"

class YOLOXNode {
public:
    YOLOXNode(ros::NodeHandle& nh, int argc, char* argv[]);
    void createRosSubPub();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void image2point_IPM(tensorrt_yolox::Object& object);
    void pubDetectedImage(const tensorrt_yolox::ObjectArrays& objects, const cv::Mat& image, const std_msgs::Header& header);
    void pubDetected2DBoxes(const tensorrt_yolox::ObjectArrays& detected_objects, const std_msgs::Header& header);
    void pubDetected3DBoxes(tensorrt_yolox::ObjectArrays& detected_objects, const std_msgs::Header& header);
    bool filter_dist_objects(autoware_msgs::DetectedObject& object);
    void giveDimentions(autoware_msgs::DetectedObject& object);
    bool getProfFlag() const;
    void printProfiling() const;

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_;

    ros::Publisher pub_bboxes_;
    ros::Publisher pub_bboxes_3d_;
    std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;
    bool init_;
    bool prof_;

    std::vector<std::string> class_name_;
    std::vector<std::vector<int>> colormap;
    std::string model_path;
    std::string precision;
    float nms_thresh;
    float thresh;
    int num_class;
    std::string rgbfile_path;
    std::string classname_path;
    std::string input_topic;
};

#endif // YOLOX_HPP