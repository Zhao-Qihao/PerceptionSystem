// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/object_merger/association/data_association.hpp"
#include "autoware/object_merger/association/solver/gnn_solver.hpp"

#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

#include <algorithm>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace autoware::object_recognition_utils
{

struct ClassificationResult
{
  uint8_t label;
  double probability;
};

uint8_t getHighestProbLabel(const std::vector<ClassificationResult>& classification)
{
  if (classification.empty()) {
    throw std::invalid_argument("Classification results are empty.");
  }

  auto max_iter = std::max_element(
    classification.begin(), classification.end(),
    [](const ClassificationResult& a, const ClassificationResult& b) {
      return a.probability < b.probability;
    });

  return max_iter->label;
}

// Helper function to calculate the area of a polygon
double polygonArea(const std::vector<cv::Point2f>& polygon) {
    double area = 0.0;
    int j = polygon.size() - 1;
    for (int i = 0; i < polygon.size(); i++) {
        area += (polygon[j].x + polygon[i].x) * (polygon[j].y - polygon[i].y);
        j = i;
    }
    return std::abs(area / 2.0);
}

// Helper function to calculate the intersection area of two polygons
double intersectionArea(const std::vector<cv::Point2f>& poly1, const std::vector<cv::Point2f>& poly2) {
    cv::Rect_<float> rect1 = cv::boundingRect(poly1);
    cv::Rect_<float> rect2 = cv::boundingRect(poly2);
    cv::Rect_<float> intersectionRect = rect1 & rect2;
    if (intersectionRect.width <= 0 || intersectionRect.height <= 0) {
        return 0.0;
    }
    std::vector<cv::Point2f> intersectionPoints;
    cv::intersectConvexConvex(poly1, poly2, intersectionPoints);
    return polygonArea(intersectionPoints);
}

// Function to project a 3D bounding box to a 2D bounding box
std::vector<cv::Point2f> projectTo2D(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& dimensions) {
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double length = dimensions.x;
    double width = dimensions.y;

    std::vector<cv::Point2f> points;
    points.push_back(cv::Point2f(-length / 2, -width / 2));
    points.push_back(cv::Point2f(length / 2, -width / 2));
    points.push_back(cv::Point2f(length / 2, width / 2));
    points.push_back(cv::Point2f(-length / 2, width / 2));

    cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(0, 0), yaw * 180 / M_PI, 1.0);
    cv::Mat translatedPoints;
    for (auto& point : points) {
        cv::Mat pointMat = (cv::Mat_<float>(3, 1) << point.x, point.y, 1);
        pointMat = rotationMatrix * pointMat;
        point.x = pointMat.at<float>(0) + pose.position.x;
        point.y = pointMat.at<float>(1) + pose.position.y;
    }

    return points;
}

double get2DIoU(const autoware_msgs::DetectedObject & object0, const autoware_msgs::DetectedObject & object1, double min_union_iou_area) {
    // Project both objects to 2D
    std::vector<cv::Point2f> poly0 = projectTo2D(object0.pose, object0.dimensions);
    std::vector<cv::Point2f> poly1 = projectTo2D(object1.pose, object1.dimensions);

    // Calculate areas
    double area0 = polygonArea(poly0);
    double area1 = polygonArea(poly1);

    // Calculate intersection area
    double interArea = intersectionArea(poly0, poly1);

    // Calculate union area
    double unionArea = area0 + area1 - interArea;

    // Check if union area is less than the minimum threshold
    if (unionArea < min_union_iou_area) {
        return 0.0;
    }

    // Calculate IoU
    double iou = interArea / unionArea;

    return iou;
}

double calcDistance3d(const autoware_msgs::DetectedObject& object0, const autoware_msgs::DetectedObject& object1) {
    double dx = object0.pose.position.x - object1.pose.position.x;
    double dy = object0.pose.position.y - object1.pose.position.y;
    double dz = object0.pose.position.z - object1.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace autoware::object_recognition_utils


namespace autoware::object_merger
{
double normalizeRadian(double radian)
{
  // Normalize the radian to the range [-π, π]
  radian = std::fmod(radian + M_PI, 2.0 * M_PI);
  if (radian < 0.0) {
    radian += 2.0 * M_PI;
  }
  radian -= M_PI;
  return radian;
}


double getFormedYawAngle(
  const geometry_msgs::Quaternion & quat0, const geometry_msgs::Quaternion & quat1,
  const bool distinguish_front_or_back = true)
{
  tf2::Quaternion q0(quat0.x, quat0.y, quat0.z, quat0.w);
  tf2::Quaternion q1(quat1.x, quat1.y, quat1.z, quat1.w);

  tf2::Matrix3x3 m0(q0);
  tf2::Matrix3x3 m1(q1);

  double roll0, pitch0, yaw0;
  double roll1, pitch1, yaw1;

  m0.getRPY(roll0, pitch0, yaw0);
  m1.getRPY(roll1, pitch1, yaw1);

  const double angle_range = distinguish_front_or_back ? M_PI : M_PI_2;
  const double angle_step = distinguish_front_or_back ? 2.0 * M_PI : M_PI;

  // Fixed yaw0 to be in the range of +-90 or 180 degrees of yaw1
  double fixed_yaw0 = yaw0;
  while (angle_range <= yaw1 - fixed_yaw0) {
    fixed_yaw0 = fixed_yaw0 + angle_step;
  }
  while (angle_range <= fixed_yaw0 - yaw1) {
    fixed_yaw0 = fixed_yaw0 - angle_step;
  }
  return std::fabs(fixed_yaw0 - yaw1);
}

DataAssociation::DataAssociation(
  std::vector<int> can_assign_vector, std::vector<double> max_dist_vector,
  std::vector<double> max_rad_vector, std::vector<double> min_iou_vector)
: score_threshold_(0.01)
{
  {
    const int assign_label_num = static_cast<int>(std::sqrt(can_assign_vector.size()));
    Eigen::Map<Eigen::MatrixXi> can_assign_matrix_tmp(
      can_assign_vector.data(), assign_label_num, assign_label_num);
    can_assign_matrix_ = can_assign_matrix_tmp.transpose();
  }
  {
    const int max_dist_label_num = static_cast<int>(std::sqrt(max_dist_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_dist_matrix_tmp(
      max_dist_vector.data(), max_dist_label_num, max_dist_label_num);
    max_dist_matrix_ = max_dist_matrix_tmp.transpose();
  }
  {
    const int max_rad_label_num = static_cast<int>(std::sqrt(max_rad_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_rad_matrix_tmp(
      max_rad_vector.data(), max_rad_label_num, max_rad_label_num);
    max_rad_matrix_ = max_rad_matrix_tmp.transpose();
  }
  {
    const int min_iou_label_num = static_cast<int>(std::sqrt(min_iou_vector.size()));
    Eigen::Map<Eigen::MatrixXd> min_iou_matrix_tmp(
      min_iou_vector.data(), min_iou_label_num, min_iou_label_num);
    min_iou_matrix_ = min_iou_matrix_tmp.transpose();
  }

  gnn_solver_ptr_ = std::make_unique<autoware::object_merger::gnn_solver::MuSSP>();
}

void DataAssociation::assign(
  const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment)
{
  std::vector<std::vector<double>> score(src.rows());
  for (int row = 0; row < src.rows(); ++row) {
    score.at(row).resize(src.cols());
    for (int col = 0; col < src.cols(); ++col) {
      score.at(row).at(col) = src(row, col);
    }
  }
  // Solve
  gnn_solver_ptr_->maximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  for (auto itr = direct_assignment.begin(); itr != direct_assignment.end();) {
    if (src(itr->first, itr->second) < score_threshold_) {
      itr = direct_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
  for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end();) {
    if (src(itr->second, itr->first) < score_threshold_) {
      itr = reverse_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
}

Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const autoware_msgs::DetectedObjectArray & objects0,
  const autoware_msgs::DetectedObjectArray & objects1)
{
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(objects1.objects.size(), objects0.objects.size());
  for (size_t objects1_idx = 0; objects1_idx < objects1.objects.size(); ++objects1_idx) {
    const autoware_msgs::DetectedObject & object1 =
      objects1.objects.at(objects1_idx);

    for (size_t objects0_idx = 0; objects0_idx < objects0.objects.size(); ++objects0_idx) {
      const autoware_msgs::DetectedObject & object0 =
        objects0.objects.at(objects0_idx);

      double score = 0.0;
      if (true) {
        const double max_dist = 2.5;
        const double dist = autoware::object_recognition_utils::calcDistance3d(
          object0, object1);

        bool passed_gate = true;
        // dist gate
        {  // passed_gate is always true
          if (max_dist < dist) passed_gate = false;
        }
        // angle gate
        if (passed_gate) {
          const double max_rad = 3.150;
          const double angle = getFormedYawAngle(
            object0.pose.orientation,
            object1.pose.orientation, false);
          if (std::fabs(max_rad) < M_PI && std::fabs(max_rad) < std::fabs(angle))
            passed_gate = false;
        }
        // 2d iou gate
        if (passed_gate) {
          const double min_iou = 0.1;
          const double min_union_iou_area = 1e-2;
          const double iou =
            autoware::object_recognition_utils::get2DIoU(object0, object1, min_union_iou_area);
          if (iou < min_iou) passed_gate = false;
        }

        // all gate is passed
        if (passed_gate) {
          score = (max_dist - std::min(dist, max_dist)) / max_dist;
          if (score < score_threshold_) score = 0.0;
        }
      }
      score_matrix(objects1_idx, objects0_idx) = score;
    }
  }

  return score_matrix;
}

}  // namespace autoware::object_merger
