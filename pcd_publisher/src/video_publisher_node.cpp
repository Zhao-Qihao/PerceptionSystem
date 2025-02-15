#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

class VideoPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    cv::VideoCapture cap_;

public:
    VideoPublisher(const std::string& video_path, const std::string& topic_name)
        : nh_(), image_pub_(nh_.advertise<sensor_msgs::Image>(topic_name, 1))
    {
        cap_.open(video_path);
        if (!cap_.isOpened())
        {
            ROS_ERROR("Could not open video file: %s", video_path.c_str());
            exit(1);
        }
    }

    void run(const int frame_rate)
    {
        cv::Mat frame;
        ros::Rate loop_rate(frame_rate); // 30 Hz

        while (ros::ok())
        {
            cap_ >> frame;
            if (frame.empty())
            {
                ROS_INFO("End of video file reached. Restarting...");
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // Restart the video
                continue;
            }

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub_.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string video_path;
    private_nh.param<std::string>("video_path", video_path, "/path/to/default/video.mp4");

    int frame_rate;
    private_nh.param("frame_rate", frame_rate, 30);

    std::string topic_name = "/camera/image_raw";
    private_nh.param<std::string>("topic_name", topic_name, "/camera/image_raw");

    VideoPublisher publisher(video_path, topic_name);
    publisher.run(frame_rate);

    return 0;
}