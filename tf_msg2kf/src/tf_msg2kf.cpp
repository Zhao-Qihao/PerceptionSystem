#include <ros/ros.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

#include <kf_msgs/LidarObstacles.h>
#include <kf_msgs/LidarObstacle.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include <algorithm>
#include <string>
#include <cmath>
#include <cctype>
#include <unordered_map>
#include <array>
#include <cstdint>

class TfMsg2Kf
{
public:
  TfMsg2Kf()
    : pnh_("~"),
      frame_counter_(0)
  {
    std::string input_topic;
    std::string output_array_topic;
    std::string output_single_topic;

    // 1) 订阅跟踪后的话题
    pnh_.param<std::string>("input_topic", input_topic,
                            std::string("/detection/tracking/object_tracker/objects"));
    // 2) 输出 kf_msgs/LidarObstacles
    pnh_.param<std::string>("output_array_topic", output_array_topic,
                            std::string("/lidar_obstacles"));

    pnh_.param<double>("static_speed_threshold", static_speed_threshold_, 0.5);
    pnh_.param<int>("max_lost_frames", max_lost_frames_, 30);

    used_ids_.fill(false);

    sub_ = nh_.subscribe(input_topic, 1, &TfMsg2Kf::objectsCallback, this);
    pub_array_ = nh_.advertise<kf_msgs::LidarObstacles>(output_array_topic, 1);

    ROS_INFO_STREAM("TfMsg2Kf node listening on "
                    << input_topic << ", publishing array to "
                    << output_array_topic);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_array_;

  double static_speed_threshold_;
  int max_lost_frames_;

  struct TrackInfo
  {
    uint8_t small_id;
    uint64_t last_seen_frame;
  };

  std::unordered_map<uint32_t, TrackInfo> id_map_;
  std::array<bool, 256> used_ids_;
  uint64_t frame_counter_;

  void objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
  {
    ++frame_counter_;

    kf_msgs::LidarObstacles out;
    out.header = msg->header;
    out.obstacles.reserve(msg->objects.size());

    for (const auto& obj : msg->objects)
    {
      uint8_t track_id = getOrAssignTrackId(obj.id);

      kf_msgs::LidarObstacle o;
      o.track_id = track_id;
      fillFromDetectedObject(obj, o);
      out.obstacles.push_back(o);
    }

    cleanupOldTracks();
    pub_array_.publish(out);
  }

  void fillFromDetectedObject(const autoware_msgs::DetectedObject& src,
                              kf_msgs::LidarObstacle& dst) const
  {
    // 加速度、速度
    dst.a     = src.acceleration.linear.x;
    dst.vel_x = src.velocity.linear.x;
    dst.vel_y = src.velocity.linear.y;

    // 位置
    dst.position = src.pose.position;

    // 角点：来自 convex_hull.polygon.points (Point32 -> Point)
    dst.corners.clear();
    const auto& poly = src.convex_hull.polygon;
    dst.corners.reserve(poly.points.size());
    for (const auto& p32 : poly.points)
    {
      geometry_msgs::Point p;
      p.x = p32.x;
      p.y = p32.y;
      p.z = p32.z;
      dst.corners.push_back(p);
    }

    // 类型
    dst.type = classifyType(src.label);

    // 运动状态
    dst.motion_type = classifyMotion(src);
  }

  uint8_t classifyType(const std::string& label) const
  {
    using kf_msgs::LidarObstacle;

    std::string lower = toLower(label);

    if (lower.find("car") != std::string::npos ||
        lower.find("veh") != std::string::npos ||
        lower.find("truck") != std::string::npos ||
        lower.find("bus") != std::string::npos)
    {
      return LidarObstacle::type_veh;
    }

    if (lower.find("ped") != std::string::npos ||
        lower.find("person") != std::string::npos)
    {
      return LidarObstacle::type_ped;
    }

    if (lower.find("bike") != std::string::npos ||
        lower.find("bicycle") != std::string::npos ||
        lower.find("cycl") != std::string::npos)
    {
      return LidarObstacle::type_cyc;
    }

    return LidarObstacle::type_unknown;
  }

  uint8_t classifyMotion(const autoware_msgs::DetectedObject& src) const
  {
    using kf_msgs::LidarObstacle;

    if (!src.velocity_reliable)
    {
      return LidarObstacle::motion_type_unknown;
    }

    const double vx = src.velocity.linear.x;
    const double vy = src.velocity.linear.y;
    const double speed = std::sqrt(vx * vx + vy * vy);

    if (speed < static_speed_threshold_)
    {
      return LidarObstacle::motion_type_static;
    }

    return LidarObstacle::motion_type_moving;
  }

  uint8_t getOrAssignTrackId(uint32_t original_id)
  {
    auto it = id_map_.find(original_id);
    if (it != id_map_.end())
    {
      it->second.last_seen_frame = frame_counter_;
      return it->second.small_id;
    }

    for (size_t i = 0; i < used_ids_.size(); ++i)
    {
      if (!used_ids_[i])
      {
        used_ids_[i] = true;
        TrackInfo info;
        info.small_id = static_cast<uint8_t>(i);
        info.last_seen_frame = frame_counter_;
        id_map_[original_id] = info;
        return info.small_id;
      }
    }

    auto victim_it = id_map_.begin();
    for (auto it2 = id_map_.begin(); it2 != id_map_.end(); ++it2)
    {
      if (it2->second.last_seen_frame < victim_it->second.last_seen_frame)
      {
        victim_it = it2;
      }
    }

    uint8_t reused_id = victim_it->second.small_id;
    id_map_.erase(victim_it);

    TrackInfo info;
    info.small_id = reused_id;
    info.last_seen_frame = frame_counter_;
    id_map_[original_id] = info;

    return reused_id;
  }

  void cleanupOldTracks()
  {
    if (max_lost_frames_ <= 0)
    {
      return;
    }

    for (auto it = id_map_.begin(); it != id_map_.end(); )
    {
      const TrackInfo& info = it->second;
      if (frame_counter_ > info.last_seen_frame &&
          frame_counter_ - info.last_seen_frame >
            static_cast<uint64_t>(max_lost_frames_))
      {
        used_ids_[info.small_id] = false;
        it = id_map_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  static std::string toLower(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_msg2kf_node");
  TfMsg2Kf node;
  ros::spin();
  return 0;
}