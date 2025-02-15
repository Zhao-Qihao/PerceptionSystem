
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "visualize_detected_objects.h"

VisualizeDetectedObjects::VisualizeDetectedObjects() : arrow_height_(0.5), label_height_(1.0)
{
  ros::NodeHandle private_nh_("~");

  ros_namespace_ = ros::this_node::getNamespace();

  if (ros_namespace_.substr(0, 2) == "//")
  {
    ros_namespace_.erase(ros_namespace_.begin());
  }

  std::string markers_out_topic = ros_namespace_ + "/objects_markers";

  std::string object_src_topic;
  private_nh_.param<std::string>("objects_src_topic", object_src_topic, "/objects");
  object_src_topic = ros_namespace_ + object_src_topic;

  ROS_INFO("[%s] objects_src_topic: %s", __APP_NAME__, object_src_topic.c_str());

  private_nh_.param<double>("object_speed_threshold", object_speed_threshold_, 0.1);
  ROS_INFO("[%s] object_speed_threshold: %.2f", __APP_NAME__, object_speed_threshold_);

  private_nh_.param<double>("arrow_speed_threshold", arrow_speed_threshold_, 0.25);
  ROS_INFO("[%s] arrow_speed_threshold: %.2f", __APP_NAME__, arrow_speed_threshold_);

  private_nh_.param<double>("marker_display_duration", marker_display_duration_, 0.1);
  ROS_INFO("[%s] marker_display_duration: %.2f", __APP_NAME__, marker_display_duration_);

  std::vector<double> color;
  private_nh_.param<std::vector<double>>("label_color", color, {255.,255.,255.,1.0});
  label_color_ = ParseColor(color);
  ROS_INFO("[%s] label_color: %s", __APP_NAME__, ColorToString(label_color_).c_str());

  private_nh_.param<std::vector<double>>("arrow_color", color, {0.,255.,0.,0.8});
  arrow_color_ = ParseColor(color);
  ROS_INFO("[%s] arrow_color: %s", __APP_NAME__, ColorToString(arrow_color_).c_str());

  private_nh_.param<std::vector<double>>("hull_color", color, {51.,204.,51.,0.8});
  hull_color_ = ParseColor(color);
  ROS_INFO("[%s] hull_color: %s", __APP_NAME__, ColorToString(hull_color_).c_str());

  private_nh_.param<std::vector<double>>("box_color", color, {51.,128.,204.,0.5});
  box_color_ = ParseColor(color);
  ROS_INFO("[%s] box_color: %s", __APP_NAME__, ColorToString(box_color_).c_str());

  private_nh_.param<std::vector<double>>("model_color", color, {190.,190.,190.,1});
  model_color_ = ParseColor(color);
  ROS_INFO("[%s] model_color: %s", __APP_NAME__, ColorToString(model_color_).c_str());

  private_nh_.param<std::vector<double>>("centroid_color", color, {77.,121.,255.,0.8});
  centroid_color_ = ParseColor(color);
  ROS_INFO("[%s] centroid_color: %s", __APP_NAME__, ColorToString(centroid_color_).c_str());

  subscriber_detected_objects_ =
    node_handle_.subscribe(object_src_topic, 1,
                           &VisualizeDetectedObjects::DetectedObjectsCallback, this);
  ROS_INFO("[%s] object_src_topic: %s", __APP_NAME__, object_src_topic.c_str());

  publisher_markers_ = node_handle_.advertise<visualization_msgs::MarkerArray>(
    markers_out_topic, 1);
  ROS_INFO("[%s] markers_out_topic: %s", __APP_NAME__, markers_out_topic.c_str());

}

std::string VisualizeDetectedObjects::ColorToString(const std_msgs::ColorRGBA &in_color)
{
  std::stringstream stream;

  stream << "{R:" << std::fixed << std::setprecision(1) << in_color.r*255 << ", ";
  stream << "G:" << std::fixed << std::setprecision(1) << in_color.g*255 << ", ";
  stream << "B:" << std::fixed << std::setprecision(1) << in_color.b*255 << ", ";
  stream << "A:" << std::fixed << std::setprecision(1) << in_color.a << "}";
  return stream.str();
}

float VisualizeDetectedObjects::CheckColor(double value)
{
  float final_value;
  if (value > 255.)
    final_value = 1.f;
  else if (value < 0)
    final_value = 0.f;
  else
    final_value = value/255.f;
  return final_value;
}

float VisualizeDetectedObjects::CheckAlpha(double value)
{
  float final_value;
  if (value > 1.)
    final_value = 1.f;
  else if (value < 0.1)
    final_value = 0.1f;
  else
    final_value = value;
  return final_value;
}

std_msgs::ColorRGBA VisualizeDetectedObjects::ParseColor(const std::vector<double> &in_color)
{
  std_msgs::ColorRGBA color;
  float r,g,b,a;
  if (in_color.size() == 4) //r,g,b,a
  {
    color.r = CheckColor(in_color[0]);
    color.g = CheckColor(in_color[1]);
    color.b = CheckColor(in_color[2]);
    color.a = CheckAlpha(in_color[3]);
  }
  return color;
}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::VisualizeSelf(const autoware_msgs::DetectedObjectArray &in_objects)
{  
  visualization_msgs::MarkerArray self_model_marker;
  visualization_msgs::Marker model;

  std::string self_vehicle_type = "truck";
  model.header = in_objects.header;
  model.lifetime = ros::Duration(marker_display_duration_);
  model.type = visualization_msgs::Marker::MESH_RESOURCE;
  model.action = visualization_msgs::Marker::ADD;
  model.ns = ros_namespace_ + "/model_markers";
  model.mesh_use_embedded_materials = false;
  model.color = model_color_;
  if (self_vehicle_type == "car")
  {
    model.mesh_resource = "package://detected_objects_visualizer/models/car.dae";
    model.pose.position.x = 0.0;
    model.pose.position.y = 0.0;
    model.pose.position.z = -1.73;
    model.pose.orientation.x = 0.0;
    model.pose.orientation.y = 0.0;
    model.pose.orientation.z = std::sqrt(0.5);
    model.pose.orientation.w = std::sqrt(0.5);
  }
  if (self_vehicle_type == "truck")
  {
  model.mesh_resource = "package://detected_objects_visualizer/models/truck.obj";
  model.pose.position.x = -2.85;
  model.pose.position.y = 0.0;
  model.pose.position.z = -3.23;
  tf::Quaternion tf_q = tf::createQuaternionFromRPY(-M_PI/2, M_PI, -M_PI/2);
  geometry_msgs::Quaternion q;
  tf::quaternionTFToMsg(tf_q, q);
  model.pose.orientation = q;
  }
  model.scale.x = 1.55;
  model.scale.y = 0.8;
  model.scale.z = 0.8;
  model.id = marker_id_++;
  self_model_marker.markers.push_back(model);

  return self_model_marker;
}

void VisualizeDetectedObjects::DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers, arrow_markers, centroid_markers, polygon_hulls, bounding_boxes,
                                  object_models;

  visualization_msgs::MarkerArray visualization_markers;
  // 获取自身车辆的模型标记
  // visualization_msgs::MarkerArray self_model_marker = VisualizeSelf(in_objects);

  marker_id_ = 0;

  label_markers = ObjectsToLabels(in_objects);
  arrow_markers = ObjectsToArrows(in_objects);   // velocity arrows
  // polygon_hulls = ObjectsToHulls(in_objects);    // polygon hulls
  bounding_boxes = ObjectsToBoxes(in_objects);
  object_models = ObjectsToModels(in_objects);  // dae models
  centroid_markers = ObjectsToCentroids(in_objects);

  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       label_markers.markers.begin(), label_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       arrow_markers.markers.begin(), arrow_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       polygon_hulls.markers.begin(), polygon_hulls.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       bounding_boxes.markers.begin(), bounding_boxes.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       object_models.markers.begin(), object_models.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       centroid_markers.markers.begin(), centroid_markers.markers.end());
    // 插入自身车辆的模型标记
  // visualization_markers.markers.insert(visualization_markers.markers.end(),
  //                                      self_model_marker.markers.begin(), self_model_marker.markers.end());

  publisher_markers_.publish(visualization_markers);

}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToCentroids(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray centroid_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
    {
      visualization_msgs::Marker centroid_marker;
      centroid_marker.lifetime = ros::Duration(marker_display_duration_);

      centroid_marker.header = in_objects.header;
      centroid_marker.type = visualization_msgs::Marker::SPHERE;
      centroid_marker.action = visualization_msgs::Marker::ADD;
      centroid_marker.pose = object.pose;
      centroid_marker.ns = ros_namespace_ + "/centroid_markers";

      centroid_marker.scale.x = 0.5;
      centroid_marker.scale.y = 0.5;
      centroid_marker.scale.z = 0.5;

      if (object.color.a == 0)
      {
        centroid_marker.color = centroid_color_;
      }
      else
      {
        centroid_marker.color = object.color;
      }
      centroid_marker.id = marker_id_++;
      centroid_markers.markers.push_back(centroid_marker);
    }
  }
  return centroid_markers;
}//ObjectsToCentroids

visualization_msgs::MarkerArray 
VisualizeDetectedObjects::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray object_boxes;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
        // (object.pose_reliable) &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      visualization_msgs::Marker box;

      box.lifetime = ros::Duration(marker_display_duration_);
      box.header = in_objects.header;
      box.type = visualization_msgs::Marker::LINE_LIST;
      box.action = visualization_msgs::Marker::ADD;
      box.ns = ros_namespace_ + "/box_markers";
      box.id = marker_id_++;
      box.scale.x = 0.12; // 设置线条宽度

      if (object.color.a == 0)
      {
        box.color = box_color_;
      }
      else
      {
        box.color = object.color;
      }

      // 计算包围盒的 8 个顶点
      double x = object.dimensions.x;
      double y = object.dimensions.y;
      double z = object.dimensions.z;
      tf2::Quaternion q(object.pose.orientation.x, object.pose.orientation.y, object.pose.orientation.z, object.pose.orientation.w);
      tf2::Matrix3x3 m(q);

      geometry_msgs::Point vertices[8];
      vertices[0].x = -x/2; vertices[0].y = -y/2; vertices[0].z = -z/2;
      vertices[1].x = -x/2; vertices[1].y = -y/2; vertices[1].z = z/2;
      vertices[2].x = -x/2; vertices[2].y = y/2; vertices[2].z = -z/2;
      vertices[3].x = -x/2; vertices[3].y = y/2; vertices[3].z = z/2;
      vertices[4].x = x/2; vertices[4].y = -y/2; vertices[4].z = -z/2;
      vertices[5].x = x/2; vertices[5].y = -y/2; vertices[5].z = z/2;
      vertices[6].x = x/2; vertices[6].y = y/2; vertices[6].z = -z/2;
      vertices[7].x = x/2; vertices[7].y = y/2; vertices[7].z = z/2;

      // 旋转并平移顶点
      for (int i = 0; i < 8; ++i)
      {
        tf2::Vector3 v(vertices[i].x, vertices[i].y, vertices[i].z);
        v = m * v;
        vertices[i].x = v.x() + object.pose.position.x;
        vertices[i].y = v.y() + object.pose.position.y;
        vertices[i].z = v.z() + object.pose.position.z;
      }

      // 定义 12 条线段
      box.points.push_back(vertices[0]); box.points.push_back(vertices[1]);
      box.points.push_back(vertices[1]); box.points.push_back(vertices[3]);
      box.points.push_back(vertices[3]); box.points.push_back(vertices[2]);
      box.points.push_back(vertices[2]); box.points.push_back(vertices[0]);

      box.points.push_back(vertices[4]); box.points.push_back(vertices[5]);
      box.points.push_back(vertices[5]); box.points.push_back(vertices[7]);
      box.points.push_back(vertices[7]); box.points.push_back(vertices[6]);
      box.points.push_back(vertices[6]); box.points.push_back(vertices[4]);

      box.points.push_back(vertices[0]); box.points.push_back(vertices[4]);
      box.points.push_back(vertices[1]); box.points.push_back(vertices[5]);
      box.points.push_back(vertices[2]); box.points.push_back(vertices[6]);
      box.points.push_back(vertices[3]); box.points.push_back(vertices[7]);

      object_boxes.markers.push_back(box);
    }
  }
  return object_boxes;
}//ObjectsToBoxes

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToModels(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray object_models;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
      object.label != "unknown" &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      visualization_msgs::Marker model;

      model.lifetime = ros::Duration(marker_display_duration_);
      model.header = in_objects.header;
      model.type = visualization_msgs::Marker::MESH_RESOURCE;
      model.action = visualization_msgs::Marker::ADD;
      model.ns = ros_namespace_ + "/model_markers";
      model.mesh_use_embedded_materials = false;
      model.color = model_color_;
      if(object.label == "car")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/car.dae";
      }
      else if (object.label == "person" || object.label == "pedestrian")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/person.dae";
      }
      else if (object.label == "cyclist")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/cyclist.obj";
      }
      else if (object.label == "bus")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/bus.obj";
      }
      else if(object.label == "truck")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/truck.obj";
      }
      // else
      // {
      //   model.mesh_resource = "package://detected_objects_visualizer/models/box.dae";
      // }
      model.pose = object.pose;
      model.pose.position.z-= object.dimensions.z/2;
      model.scale.x = 1;
      model.scale.y = 1;
      model.scale.z = 1;
      if (object.label == "person" || object.label == "pedestrian") {
        model.scale.x = 0.3;
        model.scale.y = 0.3;
        model.scale.z = 0.3;
        model.pose.position.z += object.dimensions.z/2;
      }
      if (object.label == "bus") {
        model.scale.x = 0.5;
        model.scale.y = 0.5;
        model.scale.z = 0.5;
      }
      if (object.label == "cyclist") {
        model.scale.x = 0.04;
        model.scale.y = 0.04;
        model.scale.z = 0.04;
        tf2::Quaternion current_orientation(
        object.pose.orientation.x,
        object.pose.orientation.y,
        object.pose.orientation.z,
        object.pose.orientation.w
        );
        // 绕Z轴顺时针旋转180度的四元数
        tf2::Quaternion rotation_90_degrees_clockwise(0, 0, -0.7071, 0.7071);
        rotation_90_degrees_clockwise.normalize(); // 确保四元数归一化
        // 新的姿态
        tf2::Quaternion new_orientation = current_orientation * rotation_90_degrees_clockwise;
        // 将 tf2::Quaternion 转换为 geometry_msgs::Quaternion
        geometry_msgs::Quaternion new_orientation_msg;
        new_orientation_msg.x = new_orientation.x();
        new_orientation_msg.y = new_orientation.y();
        new_orientation_msg.z = new_orientation.z();
        new_orientation_msg.w = new_orientation.w();
        model.pose.orientation = new_orientation_msg;
      }
      if (object.label == "car") {
        tf2::Quaternion current_orientation(
        object.pose.orientation.x,
        object.pose.orientation.y,
        object.pose.orientation.z,
        object.pose.orientation.w
        );
        // 绕Z轴顺时针旋转180度的四元数
        tf2::Quaternion rotation_180_degrees_clockwise(0, 0, 1, 0);
        rotation_180_degrees_clockwise.normalize(); // 确保四元数归一化
        // 新的姿态
        tf2::Quaternion new_orientation = current_orientation * rotation_180_degrees_clockwise;
        // 将 tf2::Quaternion 转换为 geometry_msgs::Quaternion
        geometry_msgs::Quaternion new_orientation_msg;
        new_orientation_msg.x = new_orientation.x();
        new_orientation_msg.y = new_orientation.y();
        new_orientation_msg.z = new_orientation.z();
        new_orientation_msg.w = new_orientation.w();
        model.pose.orientation = new_orientation_msg;
      }
      if (object.label == "truck") {
        tf2::Quaternion current_orientation(
        object.pose.orientation.x,
        object.pose.orientation.y,
        object.pose.orientation.z,
        object.pose.orientation.w
        );
        // 绕Z轴顺时针旋转180度的四元数
        tf2::Quaternion rotation_180_degrees_clockwise(0, 7071, 7071, 0);
        rotation_180_degrees_clockwise.normalize(); // 确保四元数归一化
        // 新的姿态
        tf2::Quaternion new_orientation = current_orientation * rotation_180_degrees_clockwise;
        // 将 tf2::Quaternion 转换为 geometry_msgs::Quaternion
        geometry_msgs::Quaternion new_orientation_msg;
        new_orientation_msg.x = new_orientation.x();
        new_orientation_msg.y = new_orientation.y();
        new_orientation_msg.z = new_orientation.z();
        new_orientation_msg.w = new_orientation.w();
        model.pose.orientation = new_orientation_msg;
        model.scale.x = 1.1;
        model.scale.y = 0.7;
        model.scale.z = 0.6;
      }
      model.id = marker_id_++;

      object_models.markers.push_back(model);
    }
  }
  return object_models;
}//ObjectsToModels

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToHulls(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray polygon_hulls;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty() && object.label == "unknown")
    {
      visualization_msgs::Marker hull;
      hull.lifetime = ros::Duration(marker_display_duration_);
      hull.header = in_objects.header;
      hull.type = visualization_msgs::Marker::LINE_STRIP;
      hull.action = visualization_msgs::Marker::ADD;
      hull.ns = ros_namespace_ + "/hull_markers";
      hull.id = marker_id_++;
      hull.scale.x = 0.2;

      for(auto const &point: object.convex_hull.polygon.points)
      {
        geometry_msgs::Point tmp_point;
        tmp_point.x = point.x;
        tmp_point.y = point.y;
        tmp_point.z = point.z;
        hull.points.push_back(tmp_point);
      }

      if (object.color.a == 0)
      {
        hull.color = hull_color_;
      }
      else
      {
        hull.color = object.color;
      }

      polygon_hulls.markers.push_back(hull);
    }
  }
  return polygon_hulls;
}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToArrows(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray arrow_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) && object.pose_reliable)
    {
      double velocity = object.velocity.linear.x;

      if (abs(velocity) >= arrow_speed_threshold_)
      {
        visualization_msgs::Marker arrow_marker;
        arrow_marker.lifetime = ros::Duration(marker_display_duration_);

        tf::Quaternion q(object.pose.orientation.x,
                         object.pose.orientation.y,
                         object.pose.orientation.z,
                         object.pose.orientation.w);
        double roll, pitch, yaw;

        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // in the case motion model fit opposite direction
        if (velocity < -0.1)
        {
          yaw += M_PI;
          // normalize angle
          while (yaw > M_PI)
            yaw -= 2. * M_PI;
          while (yaw < -M_PI)
            yaw += 2. * M_PI;
        }

        tf::Matrix3x3 obs_mat;
        tf::Quaternion q_tf;

        obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
        obs_mat.getRotation(q_tf);

        arrow_marker.header = in_objects.header;
        arrow_marker.ns = ros_namespace_ + "/arrow_markers";
        arrow_marker.action = visualization_msgs::Marker::ADD;
        arrow_marker.type = visualization_msgs::Marker::ARROW;

        // green
        if (object.color.a == 0)
        {
          arrow_marker.color = arrow_color_;
        }
        else
        {
          arrow_marker.color = object.color;
        }
        arrow_marker.id = marker_id_++;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        arrow_marker.pose.position.x = object.pose.position.x;
        arrow_marker.pose.position.y = object.pose.position.y;
        arrow_marker.pose.position.z = arrow_height_;

        arrow_marker.pose.orientation.x = q_tf.getX();
        arrow_marker.pose.orientation.y = q_tf.getY();
        arrow_marker.pose.orientation.z = q_tf.getZ();
        arrow_marker.pose.orientation.w = q_tf.getW();

        // Set the scale of the arrow -- 1x1x1 here means 1m on a side
        arrow_marker.scale.x = 3;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;

        arrow_markers.markers.push_back(arrow_marker);
      }//velocity threshold
    }//valid object
  }//end for
  return arrow_markers;
}//ObjectsToArrows

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToLabels(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
    {
      visualization_msgs::Marker label_marker;

      label_marker.lifetime = ros::Duration(marker_display_duration_);
      label_marker.header = in_objects.header;
      label_marker.ns = ros_namespace_ + "/label_markers";
      label_marker.action = visualization_msgs::Marker::ADD;
      label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label_marker.scale.x = 1.5;
      label_marker.scale.y = 1.5;
      label_marker.scale.z = 1.5;

      label_marker.color = label_color_;

      label_marker.id = marker_id_++;

      if(!object.label.empty() && object.label != "unknown")
        label_marker.text = object.label + " "; //Object Class if available

      std::stringstream distance_stream;
      distance_stream << std::fixed << std::setprecision(1)
                      << sqrt((object.pose.position.x * object.pose.position.x) +
                                (object.pose.position.y * object.pose.position.y));
      std::string distance_str = distance_stream.str() + " m";
      label_marker.text += distance_str;

      if (object.velocity_reliable)
      {
        double velocity = object.velocity.linear.x;
        if (velocity < -0.1)
        {
          velocity *= -1;
        }

        if (abs(velocity) < object_speed_threshold_)
        {
          velocity = 0.0;
        }

        tf::Quaternion q(object.pose.orientation.x, object.pose.orientation.y,
                         object.pose.orientation.z, object.pose.orientation.w);

        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // convert m/s to km/h
        std::stringstream kmh_velocity_stream;
        kmh_velocity_stream << std::fixed << std::setprecision(1) << (velocity * 3.6);
        std::string text = "\n<" + std::to_string(object.id) + "> " + kmh_velocity_stream.str() + " km/h";
        label_marker.text += text;
      }

      label_marker.pose.position.x = object.pose.position.x;
      label_marker.pose.position.y = object.pose.position.y;
      label_marker.pose.position.z = label_height_;
      label_marker.scale.z = 1.0;
      if (!label_marker.text.empty())
        label_markers.markers.push_back(label_marker);
    }
  }  // end in_objects.objects loop

  return label_markers;
}//ObjectsToLabels

bool VisualizeDetectedObjects::IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      std::isnan(in_object.pose.orientation.x) ||
      std::isnan(in_object.pose.orientation.y) ||
      std::isnan(in_object.pose.orientation.z) ||
      std::isnan(in_object.pose.orientation.w) ||
      std::isnan(in_object.pose.position.x) ||
      std::isnan(in_object.pose.position.y) ||
      std::isnan(in_object.pose.position.z) ||
      (in_object.pose.position.x == 0.) ||
      (in_object.pose.position.y == 0.) ||
      (in_object.dimensions.x <= 0.) ||
      (in_object.dimensions.y <= 0.) ||
      (in_object.dimensions.z <= 0.)
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid