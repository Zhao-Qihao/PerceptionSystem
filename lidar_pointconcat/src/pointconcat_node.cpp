#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>  // concatenatePointCloud
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <boost/bind.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <algorithm>

class PointConcatNode {
public:
  PointConcatNode()
  : pnh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_) {
    if (!pnh_.getParam("input_topics", input_topics_) || input_topics_.empty()) {
      ROS_FATAL("Parameter '~input_topics' is required and must be a non-empty list.");
      ros::shutdown();
      return;
    }
    pnh_.param<std::string>("output_topic", output_topic_, std::string("/points_concat"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string(""));
    pnh_.param<double>("max_delay", max_delay_, 0.05);   // seconds
    pnh_.param<int>("queue_size", queue_size_, 5);
    pnh_.param<bool>("publish_on_input", publish_on_input_, true);

    last_msgs_.resize(input_topics_.size());
    has_msg_.assign(input_topics_.size(), false);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    subs_.resize(input_topics_.size());
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      subs_[i] = nh_.subscribe<sensor_msgs::PointCloud2>(
        input_topics_[i], queue_size_,
        boost::bind(&PointConcatNode::cloudCallback, this, _1, i));
      ROS_INFO("Subscribing to input_topics[%zu]=%s", i, input_topics_[i].c_str());
    }

    if (!publish_on_input_) {
      double publish_rate_hz = 0.0;
      pnh_.param<double>("publish_rate", publish_rate_hz, 10.0);
      if (publish_rate_hz <= 0.0) publish_rate_hz = 10.0;
      timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz),
                               &PointConcatNode::timerCallback, this);
    }

    ROS_INFO("lidar_pointconcat started. output_topic=%s output_frame=%s max_delay=%.3f",
             output_topic_.c_str(),
             output_frame_.empty() ? "(keep input frame)" : output_frame_.c_str(),
             max_delay_);
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, size_t idx) {
    sensor_msgs::PointCloud2Ptr stored(new sensor_msgs::PointCloud2);

    if (!output_frame_.empty() && msg->header.frame_id != output_frame_) {
      try {
        geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(output_frame_, msg->header.frame_id, msg->header.stamp,
                                     ros::Duration(0.05));
        tf2::doTransform(*msg, *stored, tf);
      } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF transform %s -> %s failed: %s",
                          msg->header.frame_id.c_str(), output_frame_.c_str(), ex.what());
        return;
      }
    } else {
      *stored = *msg;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_msgs_[idx] = stored;
      has_msg_[idx] = true;
    }

    if (publish_on_input_) {
      publishMerged();
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    publishMerged();
  }

  void publishMerged() {
    std::vector<sensor_msgs::PointCloud2ConstPtr> candidates;
    candidates.reserve(input_topics_.size());
    ros::Time newest_stamp(0);

    {
      std::lock_guard<std::mutex> lk(mutex_);
      for (size_t i = 0; i < last_msgs_.size(); ++i) {
        if (has_msg_[i] && last_msgs_[i]) {
          candidates.push_back(last_msgs_[i]);
          if (last_msgs_[i]->header.stamp > newest_stamp) {
            newest_stamp = last_msgs_[i]->header.stamp;
          }
        }
      }
    }

    if (candidates.empty()) return;

    // Filter by max_delay
    std::vector<sensor_msgs::PointCloud2ConstPtr> in_window;
    in_window.reserve(candidates.size());
    for (const auto& m : candidates) {
      if ((newest_stamp - m->header.stamp).toSec() <= max_delay_) {
        in_window.push_back(m);
      }
    }
    if (in_window.empty()) return;

    // If not transforming to a fixed frame, ensure all frames match
    std::string out_frame = output_frame_;
    if (out_frame.empty()) {
      out_frame = in_window.front()->header.frame_id;
      for (const auto& m : in_window) {
        if (m->header.frame_id != out_frame) {
          ROS_WARN_THROTTLE(1.0, "Frames differ but output_frame is empty. Skip publish. "
                                 "Set ~output_frame to a common TF frame.");
          return;
        }
      }
    }

    // Concatenate via PCLPointCloud2 to preserve fields efficiently
    pcl::PCLPointCloud2 merged;
    bool first = true;

    for (const auto& m : in_window) {
      pcl::PCLPointCloud2 pcl_in;
      pcl_conversions::toPCL(*m, pcl_in);
      if (first) {
        merged = pcl_in;
        first = false;
      } else {
        pcl::PCLPointCloud2 tmp;
        pcl::concatenatePointCloud(merged, pcl_in, tmp);
        merged = tmp;
      }
    }

    sensor_msgs::PointCloud2 out_msg;
    pcl_conversions::fromPCL(merged, out_msg);
    out_msg.header.stamp = newest_stamp;
    out_msg.header.frame_id = out_frame;

    pub_.publish(out_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  std::vector<ros::Subscriber> subs_;
  std::vector<std::string> input_topics_;
  std::vector<sensor_msgs::PointCloud2ConstPtr> last_msgs_;
  std::vector<bool> has_msg_;
  std::mutex mutex_;
  std::string output_topic_;
  std::string output_frame_;
  double max_delay_{0.05};
  int queue_size_{5};
  bool publish_on_input_{true};
  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointconcat_node");
  PointConcatNode node;
  ros::spin();
  return 0;
}