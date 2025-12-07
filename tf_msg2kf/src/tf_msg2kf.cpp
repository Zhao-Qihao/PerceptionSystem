#include <ros/ros.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

#include <kf_msgs/LidarObstacles.h>
#include <kf_msgs/LidarObstacle.h>
#include <kf_msgs/TrafficLights.h>
#include <kf_msgs/TrafficLight.h>

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

    std::string tl_input_topic;
    std::string tl_output_topic;

    pnh_.param<std::string>("input_topic", input_topic,
                            std::string("/detection/tracking/object_tracker/objects"));
    pnh_.param<std::string>("output_array_topic", output_array_topic,
                            std::string("/lidar_obstacles"));

    pnh_.param<std::string>("tl_input_topic", tl_input_topic,
                            std::string("/detection/traffic_light/results"));
    pnh_.param<std::string>("tl_output_topic", tl_output_topic,
                            std::string("/ifv_light"));

    pnh_.param<double>("static_speed_threshold", static_speed_threshold_, 0.5);
    pnh_.param<int>("max_lost_frames", max_lost_frames_, 30);

    used_ids_.fill(false);

    sub_ = nh_.subscribe(input_topic, 1, &TfMsg2Kf::objectsCallback, this);
    pub_array_ = nh_.advertise<kf_msgs::LidarObstacles>(output_array_topic, 1);

    tl_sub_ = nh_.subscribe(tl_input_topic, 1, &TfMsg2Kf::trafficLightsCallback, this);
    tl_pub_ = nh_.advertise<kf_msgs::TrafficLights>(tl_output_topic, 1);

    ROS_INFO_STREAM("TfMsg2Kf node listening on "
                    << input_topic << ", publishing array to "
                    << output_array_topic);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_array_;

  ros::Subscriber tl_sub_;
  ros::Publisher tl_pub_;

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

    ROS_INFO_STREAM("objectsCallback: frame=" << frame_counter_
                    << ", received objects size=" << msg->objects.size());

    for (const auto& obj : msg->objects)
    {
      uint8_t track_id = getOrAssignTrackId(obj.id);

      ROS_DEBUG_STREAM("objectsCallback: original_id=" << obj.id
                       << ", assigned_track_id=" << static_cast<int>(track_id)
                       << ", label=" << obj.label);

      kf_msgs::LidarObstacle o;
      o.track_id = track_id;
      fillFromDetectedObject(obj, o);
      out.obstacles.push_back(o);
    }

    ROS_INFO_STREAM("objectsCallback: publishing obstacles size=" 
                    << out.obstacles.size());

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

  void trafficLightsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
  {
    kf_msgs::TrafficLights out;
    out.header = msg->header;
    out.traffic_lights.reserve(msg->objects.size());

    for (const auto& obj : msg->objects)
    {
      // 这几类不发送：prohibit_* / countdown_* / script_*
      std::string lower = toLower(obj.label);
      if (lower.find("prohibit") != std::string::npos ||
          lower.find("countdown") != std::string::npos ||
          lower.find("script")   != std::string::npos)
      {
        continue;
      }

      kf_msgs::TrafficLight tl;
      fillTrafficLightFromDetectedObject(obj, tl);
      out.traffic_lights.push_back(tl);
    }

    tl_pub_.publish(out);
  }

  void fillTrafficLightFromDetectedObject(const autoware_msgs::DetectedObject& src,
                                          kf_msgs::TrafficLight& dst) const
  {
    dst.color     = classifyTrafficLightColor(src.label);
    dst.direction = classifyTrafficLightDirection(src.label);
    dst.status    = classifyTrafficLightStatus(src.label);
  }
  uint8_t classifyTrafficLightColor(const std::string& label) const
  {
    using kf_msgs::TrafficLight;

    std::string lower = toLower(label);

    // round_red / up_red / left_red / ... -> red
    if (lower.find("red") != std::string::npos)
    {
      return TrafficLight::color_red;
    }

    // *_green -> green
    if (lower.find("green") != std::string::npos)
    {
      return TrafficLight::color_green;
    }

    // *_yellow -> yellow
    if (lower.find("yellow") != std::string::npos)
    {
      return TrafficLight::color_yellow;
    }

    // black / unknown 等
    return TrafficLight::color_unknown;
  }

  uint8_t classifyTrafficLightDirection(const std::string& label) const
  {
    using kf_msgs::TrafficLight;

    std::string lower = toLower(label);

    // left_* 系列
    if (lower.find("left") != std::string::npos)
    {
      return TrafficLight::direction_left;
    }

    // right_* 系列
    if (lower.find("right") != std::string::npos)
    {
      return TrafficLight::direction_right;
    }

    // turn_around_* 系列
    if (lower.find("turn_around") != std::string::npos)
    {
      return TrafficLight::direction_uturn;
    }

    // round_* / up_* / down_* 系列，认为是直行
    if (lower.find("round") != std::string::npos ||
        lower.find("up") != std::string::npos ||
        lower.find("down") != std::string::npos)
    {
      return TrafficLight::direction_straight;
    }

    // prohibit_* / countdown_* / script_* / unknown 等没有明确方向
    return TrafficLight::direction_unknown;
  }

  uint8_t classifyTrafficLightStatus(const std::string& label) const
  {
    using kf_msgs::TrafficLight;

    std::string lower = toLower(label);

    // 最后一类是 unknown
    if (lower.find("unknown") != std::string::npos)
    {
      return TrafficLight::status_unknown;
    }

    // *_black 认为是灭灯
    if (lower.find("black") != std::string::npos)
    {
      return TrafficLight::status_off;
    }

    // 其它有颜色的（red/green/yellow）都当作点亮
    return TrafficLight::status_on;
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