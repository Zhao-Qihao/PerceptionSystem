#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

class ImagePublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    cv::Mat image_;
    int frame_rate_;
    std::string frame_id_;

public:
    ImagePublisher(const std::string& image_path, const std::string& topic_name, int frame_rate, const std::string& frame_id)
        : nh_(), image_pub_(nh_.advertise<sensor_msgs::Image>(topic_name, 1)), frame_rate_(frame_rate)
    {
        image_ = cv::imread(image_path);
        if (image_.empty())
        {
            ROS_ERROR("Could not read image file: %s", image_path.c_str());
            exit(1);
        }
        frame_id_ = frame_id;
    }

    void run()
    {
        ros::Rate loop_rate(frame_rate_);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
        msg->header.frame_id = frame_id_;
        while (ros::ok())
        {
            msg->header.stamp = ros::Time::now();
            image_pub_.publish(msg);
            ROS_INFO("Published single image message.");
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string image_path;
    private_nh.param<std::string>("image_path", image_path, "/path/to/default/image.jpg");

    int frame_rate;
    private_nh.param("frame_rate", frame_rate, 30);

    std::string frame_id;
    private_nh.param<std::string>("frame_id", frame_id, "2image");
    ROS_INFO("Frame ID: %s", frame_id.c_str());

    std::string output_topic;
    private_nh.param<std::string>("output_topic", output_topic, "/camera/image_raw");

    ImagePublisher publisher(image_path, output_topic, frame_rate, frame_id);
    publisher.run();

    return 0;
}