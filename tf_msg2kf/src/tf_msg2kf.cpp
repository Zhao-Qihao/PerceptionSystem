#include <ros/ros.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

#include <kf_msgs/FusionObstacles.h>
#include <kf_msgs/FusionObstacle.h>
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
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

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
    pnh_.param<std::string>("target_frame", target_frame_,
                            std::string("base_link"));

    sub_ = nh_.subscribe(input_topic, 1, &TfMsg2Kf::objectsCallback, this);
    pub_array_ = nh_.advertise<kf_msgs::FusionObstacles>(output_array_topic, 1);

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
  tf::TransformListener tf_listener_;
  std::string target_frame_;

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

    autoware_msgs::DetectedObjectArray transformed = *msg;

    const std::string& source_frame = msg->header.frame_id;
    bool use_tf = true;

    if (source_frame.empty())
    {
      ROS_WARN_THROTTLE(1.0, "objectsCallback: empty frame_id, skip TF transform");
      use_tf = false;
    }
    else
    {
      try
      {
        // rslidar -> base_link（target_frame_）
        tf_listener_.waitForTransform(
            target_frame_, source_frame,
            msg->header.stamp, ros::Duration(0.1));
      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN_THROTTLE(1.0,
                          "objectsCallback: waitForTransform %s -> %s failed: %s; using original poses",
                          source_frame.c_str(), target_frame_.c_str(), ex.what());
        use_tf = false;
      }
    }

    if (use_tf)
    {
      for (size_t i = 0; i < transformed.objects.size(); ++i)
      {
        geometry_msgs::PoseStamped in_pose, out_pose;
        in_pose.header = msg->header;          // frame_id = rslidar
        in_pose.pose   = msg->objects[i].pose; // 原始 pose

        try
        {
          tf_listener_.transformPose(target_frame_, in_pose, out_pose);
          transformed.objects[i].pose = out_pose.pose;
        }
        catch (tf::TransformException& ex)
        {
          ROS_WARN_THROTTLE(1.0,
                            "objectsCallback: transformPose %s -> %s failed: %s; keep original pose for object %zu",
                            source_frame.c_str(), target_frame_.c_str(), ex.what(), i);
        }
      }
      // 把 header 中的坐标系也更新为目标坐标系
      transformed.header.frame_id = target_frame_;
    }

    kf_msgs::FusionObstacles out;
    out.header = transformed.header;                 // 现在是 base_link
    out.obstacles.reserve(transformed.objects.size());

    ROS_INFO_STREAM("objectsCallback: frame=" << frame_counter_
                    << ", received objects size=" << transformed.objects.size());

    for (const auto& obj : transformed.objects)
    {
      uint8_t track_id = obj.id;

      ROS_DEBUG_STREAM("objectsCallback: original_id=" << obj.id
                       << ", assigned_track_id=" << static_cast<int>(track_id)
                       << ", label=" << obj.label);

      kf_msgs::FusionObstacle o;
      o.track_id = track_id;
      fillFromDetectedObject(obj, o); // 这里用的就是 base_link 坐标系下的 pose
      out.obstacles.push_back(o);
    }

    ROS_INFO_STREAM("objectsCallback: publishing obstacles size="
                    << out.obstacles.size());

    cleanupOldTracks();
    pub_array_.publish(out);
  }

  void fillFromDetectedObject(const autoware_msgs::DetectedObject& src,
                              kf_msgs::FusionObstacle& dst) const
  {
    // 加速度、速度
    dst.a     = src.acceleration.linear.x;
    dst.vel_x = src.velocity.linear.x;
    dst.vel_y = src.velocity.linear.y;

    // 位置
    dst.position = src.pose.position;
    dst.position.z = 0.0;

    // 角点：由 position + dimensions + orientation 计算 BEV 底面 4 个角点
    dst.corners.clear();

    const double cx = src.pose.position.x;
    const double cy = src.pose.position.y;

    const double half_len = 0.5 * src.dimensions.x;  // 长度方向
    const double half_wid = 0.5 * src.dimensions.y;  // 宽度方向

    const auto& q = src.pose.orientation;
    const double qw = q.w;
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;

    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz));
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    dst.corners.reserve(4);

    // 局部坐标下的四个角 (以盒子中心为原点，x 为长度，y 为宽度)
    const double local_x[4] = {  half_len, -half_len, -half_len,  half_len };
    const double local_y[4] = {  half_wid,  half_wid, -half_wid, -half_wid };

    for (int i = 0; i < 4; ++i)
    {
      const double lx = local_x[i];
      const double ly = local_y[i];

      geometry_msgs::Point p;
      // 先在平面内按 yaw 旋转，再平移到中心
      p.x = cx + lx * cos_yaw - ly * sin_yaw;
      p.y = cy + lx * sin_yaw + ly * cos_yaw;
      p.z = 0.0;   // BEV 平面，z 固定为 0

      dst.corners.push_back(p);
    }

    // 类型
    dst.type = classifyType(src.label);

    // 运动状态
    dst.motion_type = classifyMotion(src);
  }

  uint8_t classifyType(const std::string& label) const
  {
    using kf_msgs::FusionObstacle;

    std::string lower = toLower(label);

    if (lower.find("car") != std::string::npos ||
        lower.find("veh") != std::string::npos ||
        lower.find("truck") != std::string::npos ||
        lower.find("bus") != std::string::npos)
    {
      return FusionObstacle::type_veh;
    }

    if (lower.find("ped") != std::string::npos ||
        lower.find("person") != std::string::npos)
    {
      return FusionObstacle::type_ped;
    }

    if (lower.find("bike") != std::string::npos ||
        lower.find("bicycle") != std::string::npos ||
        lower.find("cycl") != std::string::npos)
    {
      return FusionObstacle::type_cyc;
    }

    return FusionObstacle::type_unknown;
  }

  uint8_t classifyMotion(const autoware_msgs::DetectedObject& src) const
  {
    using kf_msgs::FusionObstacle;

    if (!src.velocity_reliable)
    {
      return FusionObstacle::motion_type_unknown;
    }

    const double vx = src.velocity.linear.x;
    const double vy = src.velocity.linear.y;
    const double speed = std::sqrt(vx * vx + vy * vy);

    if (speed < static_speed_threshold_)
    {
      return FusionObstacle::motion_type_static;
    }

    return FusionObstacle::motion_type_moving;
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