// Copyright 2021 The Autoware Foundation
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

#ifndef TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
#define TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_

#include "fake_test_node/fake_test_node.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace test_utils
{
using FakeNodeFixture = autoware::tools::testing::FakeTestNode;
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

inline void waitForMessage(
  const std::shared_ptr<rclcpp::Node> & node, FakeNodeFixture * fixture, const bool & received_flag,
  const std::chrono::duration<int> max_wait_time = std::chrono::seconds{10LL},
  const bool fail_on_timeout = true)
{
  const auto dt{std::chrono::milliseconds{30LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!received_flag) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(fixture->get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      if (fail_on_timeout) {
        throw std::runtime_error(std::string("Did not receive a message soon enough"));
      } else {
        break;
      }
    }
  }
}

inline geometry_msgs::msg::TransformStamped getDummyTransform()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  return transform_stamped;
}

inline TrajectoryPoint make_traj_point(const double px, const double py, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = px;
  p.pose.position.y = py;
  p.longitudinal_velocity_mps = vx;
  return p;
}

inline Trajectory generateCurvatureTrajectory(
  std_msgs::msg::Header header, double curvature, double arc_length, double velocity)
{
  Trajectory trajectory;
  trajectory.header = header;

  // Calculate radius from curvature
  double radius =
    1 / fabs(curvature);  // Radius is the reciprocal of the absolute value of curvature
  double arc_angle = arc_length / radius;  // Total angle of the arc

  // Calculate starting angle based on curvature
  double start_angle = (curvature > 0) ? (3 * M_PI / 2) : (M_PI / 2);
  start_angle -= arc_angle / 2;

  const int points = 20;  // Number of points in the trajectory

  // Generate points along the arc in reverse order
  for (int i = points; i >= 0; --i) {
    double angle = start_angle + (arc_angle * i / points);  // Current angle of the point on the arc
    double x = radius * cos(angle);                         // X coordinate
    double y = radius * sin(angle);                         // Y coordinate
    // Adjust y-coordinate based on curvature sign
    y += (curvature > 0) ? radius : -radius;

    trajectory.points.push_back(make_traj_point(x, y, velocity));  // Speed set as per argument
  }

  return trajectory;
}

// TODO(Horibe): modify the controller nodes so that they does not publish topics when data is not
// ready. then, remove this function.
template <typename T>
inline void spinWhile(T & node)
{
  for (size_t i = 0; i < 10; i++) {
    rclcpp::spin_some(node);
    const auto dt{std::chrono::milliseconds{100LL}};
    std::this_thread::sleep_for(dt);
  }
}

void writeTrajectoriesToFiles(
  const Trajectory & original_ref_trajectory, const Trajectory & resampled_ref_trajectory,
  const Trajectory & predicted_trajectory, const rclcpp::Time & stamp)
{
  std::ofstream output_file_orig_x("original_x.log");
  std::ofstream output_file_orig_y("original_y.log");
  std::ofstream output_file_resampled_x("resampled_x.log");
  std::ofstream output_file_resampled_y("resampled_y.log");
  std::ofstream output_file_predicted_x("predicted_x.log");
  std::ofstream output_file_predicted_y("predicted_y.log");
  std::ofstream output_file_time("time.log");

  auto original_ref_trajectory_it = original_ref_trajectory.points.begin();
  while (original_ref_trajectory_it != original_ref_trajectory.points.end()) {
    output_file_orig_x << original_ref_trajectory_it->pose.position.x << ",";
    output_file_orig_y << original_ref_trajectory_it->pose.position.y << ",";
    ++original_ref_trajectory_it;
  }
  output_file_orig_x << std::endl;
  output_file_orig_y << std::endl;

  auto ref_it = resampled_ref_trajectory.points.begin();
  while (ref_it != resampled_ref_trajectory.points.end()) {
    output_file_resampled_x << ref_it->pose.position.x << ",";
    output_file_resampled_y << ref_it->pose.position.y << ",";
    ++ref_it;
  }
  output_file_resampled_x << std::endl;
  output_file_resampled_y << std::endl;

  auto pred_it = predicted_trajectory.points.begin();
  while (pred_it != predicted_trajectory.points.end()) {
    output_file_predicted_x << pred_it->pose.position.x << ",";
    output_file_predicted_y << pred_it->pose.position.y << ",";
    ++pred_it;
  }
  output_file_predicted_x << std::endl;
  output_file_predicted_y << std::endl;

  output_file_time << stamp.seconds() << "." << stamp.nanoseconds() << std::endl;
}
}  // namespace test_utils

#endif  // TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
