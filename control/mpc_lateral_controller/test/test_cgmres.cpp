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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "mpc_lateral_controller/mpc.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

rclcpp::NodeOptions makeNodeOptions()
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("mpc_lateral_controller");
  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args", "--params-file", share_dir + "/param/lateral_controller_defaults.param.yaml",
     "--params-file", share_dir + "/test/test_vehicle_info.param.yaml"});

  return node_options;
}

TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

nav_msgs::msg::Odometry makeOdometry(const geometry_msgs::msg::Pose & pose, const double velocity)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = pose;
  odometry.twist.twist.linear.x = velocity;
  return odometry;
}
class MPCTest : public ::testing::Test
{
protected:
  MPCParam param;
  // Test inputs
  Trajectory dummy_straight_trajectory;
  Trajectory dummy_right_turn_trajectory;
  SteeringReport neutral_steer;
  Pose pose_zero;
  double default_velocity = 1.0;
  rclcpp::Logger logger = rclcpp::get_logger("mpc_test_logger");

  double ctrl_period = 0.03;

  TrajectoryFilteringParam trajectory_param;

  void initParam()
  {
    dummy_straight_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(1.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(2.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(3.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(4.0, 0.0, 1.0f));

    dummy_right_turn_trajectory.points.push_back(makePoint(-1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(2.0, -2.0, 1.0f));

    neutral_steer.steering_tire_angle = 0.0;
    pose_zero.position.x = 0.0;
    pose_zero.position.y = 0.0;
  }

  void initializeMPC(mpc_lateral_controller::MPC & mpc)
  {
    mpc.m_ctrl_period = ctrl_period;
    // Init trajectory
    const auto current_kinematics =
      makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
    mpc.setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    initParam();
  }

  void TearDown() override { rclcpp::shutdown(); }
};  // class MPCTest

/* cppcheck-suppress syntaxError */
TEST_F(MPCTest, InitializeAndCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  EXPECT_FALSE(mpc->hasVehicleModel());
  EXPECT_FALSE(mpc->hasQPSolver());
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, InitializeAndCalculateRightTurn)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  EXPECT_FALSE(mpc->hasVehicleModel());
  EXPECT_FALSE(mpc->hasQPSolver());

  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, OsqpCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);

  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  EXPECT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, OsqpCalculateRightTurn)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, KinematicsNoDelayCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init trajectory
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, KinematicsNoDelayCalculateRightTurn)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, DynamicCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);

  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, MultiSolveWithBuffer)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  mpc->m_input_buffer = {0.0, 0.0, 0.0};
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);

  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
}

TEST_F(MPCTest, FailureCases)
{
  auto node = rclcpp::Node("mpc_test_node", makeNodeOptions());
  auto mpc = std::make_unique<MPC>(node);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  // Calculate MPC with a pose too far from the trajectory
  Pose pose_far;
  constexpr double admissible_position_error = 5.0;
  pose_far.position.x = pose_zero.position.x - admissible_position_error - 1.0;
  pose_far.position.y = pose_zero.position.y - admissible_position_error - 1.0;
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_far, default_velocity);
  EXPECT_FALSE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));

  // Calculate MPC with a fast velocity to make the prediction go further than the reference path
  EXPECT_FALSE(mpc->calculateMPC(
    neutral_steer, makeOdometry(pose_far, default_velocity + 10.0), ctrl_cmd, pred_traj, diag));
}
}  // namespace autoware::motion::control::mpc_lateral_controller
