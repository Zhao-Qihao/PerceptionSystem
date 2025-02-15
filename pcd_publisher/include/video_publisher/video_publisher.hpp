#ifndef VIDEO_PUBLISHER_NODE_HPP
#define VIDEO_PUBLISHER_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisherNode {
public:
    VideoPublisherNode(ros::NodeHandle& nh, const std::string& video_path, const std::string& topic_name);
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    cv::VideoCapture cap_;
    std::string video_path_;
    std::string topic_name_;

    void publishFrame();
};

#endif // VIDEO_PUBLISHER_NODE_HPP