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
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "trajectory_follower_node/controller_node.hpp"
#include "trajectory_follower_test_utils.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <fstream>
#include <memory>
#include <vector>

using Controller = autoware::motion::control::trajectory_follower_node::Controller;
using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using VehicleOdometry = nav_msgs::msg::Odometry;
using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using geometry_msgs::msg::AccelWithCovarianceStamped;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

rclcpp::NodeOptions makeNodeOptions(const bool enable_keep_stopped_until_steer_convergence = false)
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("trajectory_follower_node");
  const auto longitudinal_share_dir =
    ament_index_cpp::get_package_share_directory("pid_longitudinal_controller");
  const auto lateral_share_dir =
    ament_index_cpp::get_package_share_directory("mpc_lateral_controller");
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("lateral_controller_mode", "mpc");
  node_options.append_parameter_override("longitudinal_controller_mode", "pid");
  node_options.append_parameter_override(
    "enable_keep_stopped_until_steer_convergence",
    enable_keep_stopped_until_steer_convergence);  // longitudinal
  node_options.arguments(
    {"--ros-args", "--params-file",
     lateral_share_dir + "/param/lateral_controller_cgmres.param.yaml", "--params-file",
     //  lateral_share_dir + "/param/lateral_controller_defaults.param.yaml", "--params-file",
     longitudinal_share_dir + "/param/longitudinal_controller_defaults.param.yaml", "--params-file",
     share_dir + "/test/test_vehicle_info.param.yaml", "--params-file",
     share_dir + "/test/test_nearest_search.param.yaml", "--params-file",
     share_dir + "/param/trajectory_follower_node.param.yaml"});

  return node_options;
}

std::shared_ptr<Controller> makeNode(const rclcpp::NodeOptions & node_options)
{
  std::shared_ptr<Controller> node = std::make_shared<Controller>(node_options);

  // Enable all logging in the node
  auto ret =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    std::cout << "Failed to set logging severity to DEBUG\n";
  }
  return node;
}

template <typename MessageType>
void save_message_to_rosbag(
  const std::string & bag_directory, const std::shared_ptr<MessageType> & message,
  const std::string & topic_name)
{
  auto writer = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  rosbag2_storage::StorageOptions storage_options{bag_directory, "sqlite3"};
  rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
  writer->open(storage_options, converter_options);

  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = rosidl_generator_traits::data_type<MessageType>();
  topic_metadata.serialization_format = "cdr";
  writer->create_topic(topic_metadata);

  rclcpp::SerializedMessage serialized_message;
  rclcpp::Serialization<MessageType> serializer;
  serializer.serialize_message(message.get(), &serialized_message);

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    &serialized_message.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* unused */) {});
  bag_message->topic_name = topic_name;

  rcutils_time_point_value_t now;
  auto ret = rcutils_system_time_now(&now);
  if (ret != RCUTILS_RET_OK) {
    std::cerr << "Failed to get current system time: " << rcutils_get_error_string().str
              << std::endl;
    return;  // Early return on error
  }
  bag_message->time_stamp = now;

  writer->write(bag_message);

  // Function continues after this point if needed.
  // For instance, you might want to log success, perform cleanup, or other actions.
  std::cout << "Successfully wrote message to rosbag: " << bag_directory << std::endl;

  // Note: Cleanup and resource deallocation (if necessary) happens automatically
  // as we are using smart pointers and RAII patterns.
}

class ControllerTester
{
public:
  explicit ControllerTester(FakeNodeFixture * _fnf, const rclcpp::NodeOptions & node_options)
  : fnf(_fnf), node(makeNode(node_options))
  {
  }

  FakeNodeFixture * fnf;
  std::shared_ptr<Controller> node;

  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;

  Trajectory::SharedPtr resampled_reference_trajectory;
  bool received_resampled_reference_trajectory = false;
  Trajectory::SharedPtr predicted_trajectory_in_frenet_coordinate;
  bool received_predicted_trajectory_in_frenet_coordinate = false;

  void publish_default_odom()
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_pub->publish(odom_msg);
  };

  void publish_odom_vx(const double vx)
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_msg.header.frame_id = "map";
    odom_msg.twist.twist.linear.x = vx;
    // std::cerr << "published odom position: \n x: " << odom_msg.pose.pose.position.x
    //           << " y: " << odom_msg.pose.pose.position.y
    //           << " at frame ID: " << odom_msg.header.frame_id << std::endl;
    odom_pub->publish(odom_msg);
  };

  void publish_default_steer()
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_pub->publish(steer_msg);
  };

  void publish_steer_angle(const double steer)
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_msg.steering_tire_angle = steer;
    steer_pub->publish(steer_msg);
  };

  void publish_default_acc()
  {
    AccelWithCovarianceStamped acc_msg;
    acc_msg.header.stamp = node->now();
    accel_pub->publish(acc_msg);
  };

  void publish_autonomous_operation_mode()
  {
    OperationModeState msg;
    msg.stamp = node->now();
    msg.mode = OperationModeState::AUTONOMOUS;
    operation_mode_pub->publish(msg);
  };

  void publish_default_traj()
  {
    Trajectory traj_msg;
    traj_msg.header.stamp = node->now();
    traj_msg.header.frame_id = "map";
    traj_pub->publish(traj_msg);
  };

  void send_default_transform()
  {
    // Dummy transform: ego is at (0.0, 0.0) in map frame
    geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
    transform.header.stamp = node->now();
    br->sendTransform(transform);

    // Spin for transform to be published
    test_utils::spinWhile(node);
  };

  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    fnf->create_publisher<Trajectory>("controller/input/reference_trajectory");

  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    fnf->create_publisher<VehicleOdometry>("controller/input/current_odometry");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    fnf->create_publisher<SteeringReport>("controller/input/current_steering");

  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr accel_pub =
    fnf->create_publisher<AccelWithCovarianceStamped>("controller/input/current_accel");

  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub =
    fnf->create_publisher<OperationModeState>("controller/input/current_operation_mode");

  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    fnf->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *fnf->get_fake_node(),
      [this](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr predicted_traj_sub =
    fnf->create_subscription<Trajectory>(
      "controller/debug/predicted_trajectory_in_frenet_coordinate", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        predicted_trajectory_in_frenet_coordinate = msg;
        received_predicted_trajectory_in_frenet_coordinate = true;
      });

  rclcpp::Subscription<Trajectory>::SharedPtr resampled_ref_traj_sub =
    fnf->create_subscription<Trajectory>(
      "controller/debug/resampled_reference_trajectory", *fnf->get_fake_node(),
      [this](const Trajectory::SharedPtr msg) {
        resampled_reference_trajectory = msg;
        received_resampled_reference_trajectory = true;
      });

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(fnf->get_fake_node());
};

// TEST_F(FakeNodeFixture, no_input)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   // No published data: expect a stopped command
//   test_utils::waitForMessage(
//     tester.node, this, tester.received_control_command, std::chrono::seconds{1LL}, false);
//   ASSERT_FALSE(tester.received_control_command);
// }

// TEST_F(FakeNodeFixture, empty_trajectory)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();

//   // Empty trajectory: expect a stopped command
//   tester.publish_default_traj();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();
//   tester.publish_default_steer();

//   test_utils::waitForMessage(
//     tester.node, this, tester.received_control_command, std::chrono::seconds{1LL}, false);
//   ASSERT_FALSE(tester.received_control_command);
// }

// lateral
// TEST_F(FakeNodeFixture, straight_trajectory)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(1.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(2.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   // following conditions will pass even if the MPC solution does not converge
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

TEST_F(FakeNodeFixture, right_turn)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.send_default_transform();
  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  // Right turning trajectory with a constant curvature of -0.5: expect right steering
  Trajectory traj_msg;
  std_msgs::msg::Header header;
  header.stamp = tester.node->now();
  header.frame_id = "map";
  traj_msg = test_utils::generateCurvatureTrajectory(header, -0.5, 4.0, 1.0);
  tester.traj_pub->publish(traj_msg);

  test_utils::waitForMessage(tester.node, this, tester.received_control_command);
  ASSERT_TRUE(tester.received_control_command);
  // ASSERT_TRUE(tester.received_resampled_reference_trajectory);

  // const auto save_directory = "/home/kyoichi-sugahara/workspace/log/reference_trajectory";
  // save_message_to_rosbag(
  //   save_directory, tester.resampled_reference_trajectory,
  //   "controller/debug/resampled_reference_trajectory");
  std::ofstream output_file("output.csv");
  output_file
    << "original reference trajectory,,,resampled_reference trajectory,,,predicted trajectory"
    << std::endl;
  output_file << "x,y,,x,y,,x,y" << std::endl;

  auto traj_msg_it = traj_msg.points.begin();
  auto ref_it = tester.resampled_reference_trajectory->points.begin();
  auto pred_it = tester.predicted_trajectory_in_frenet_coordinate->points.begin();

  while (traj_msg_it != traj_msg.points.end() ||
         ref_it != tester.resampled_reference_trajectory->points.end() ||
         pred_it != tester.predicted_trajectory_in_frenet_coordinate->points.end()) {
    if (traj_msg_it != traj_msg.points.end()) {
      output_file << traj_msg_it->pose.position.x << "," << traj_msg_it->pose.position.y << ",,";
      ++traj_msg_it;
    } else {
      output_file << ",,,";
    }

    if (ref_it != tester.resampled_reference_trajectory->points.end()) {
      output_file << ref_it->pose.position.x << "," << ref_it->pose.position.y << ",,";
      ++ref_it;
    } else {
      output_file << ",,,";
    }

    if (pred_it != tester.predicted_trajectory_in_frenet_coordinate->points.end()) {
      output_file << pred_it->pose.position.x << "," << pred_it->pose.position.y << std::endl;
      ++pred_it;
    } else {
      output_file << std::endl;
    }
  }

  output_file.close();

  std::cerr << "lat steer tire angle: " << tester.cmd_msg->lateral.steering_tire_angle << std::endl;
  std::cerr << "lat steer tire rotation rate: "
            << tester.cmd_msg->lateral.steering_tire_rotation_rate << std::endl;
  EXPECT_LT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_LT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, right_turn_convergence)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);
  Trajectory traj_msg;

  auto publishTrajectory = [&tester, &traj_msg]() {
    tester.send_default_transform();
    tester.publish_odom_vx(1.0);
    tester.publish_autonomous_operation_mode();
    tester.publish_default_steer();
    tester.publish_default_acc();

    // Right turning trajectory with a constant curvature of -0.5: expect right steering

    std_msgs::msg::Header header;
    header.stamp = tester.node->now();
    header.frame_id = "map";
    traj_msg = test_utils::generateCurvatureTrajectory(header, -0.5, 4.0, 1.0);
    tester.traj_pub->publish(traj_msg);
  };

  constexpr size_t iter_num = 10;
  for (size_t i = 0; i < iter_num; i++) {
    const auto start_time = std::chrono::system_clock::now();
    publishTrajectory();
    const auto after_publish_time = std::chrono::system_clock::now();

    test_utils::waitForMessage(tester.node, this, tester.received_control_command);
    const auto after_spin_time = std::chrono::system_clock::now();

    ASSERT_TRUE(tester.received_control_command);
    std::cerr << "lat steer tire angle: " << tester.cmd_msg->lateral.steering_tire_angle
              << std::endl;
    std::cerr << "lat steer tire rotation rate: "
              << tester.cmd_msg->lateral.steering_tire_rotation_rate << std::endl;
    EXPECT_LT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
    EXPECT_LT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
    const auto last_time = std::chrono::system_clock::now();
    const double publish_time_ms =
      std::chrono::duration_cast<std::chrono::nanoseconds>(after_publish_time - start_time)
        .count() *
      1.0e-6;
    const double spin_time_ms =
      std::chrono::duration_cast<std::chrono::nanoseconds>(after_spin_time - after_publish_time)
        .count() *
      1.0e-6;
    const double total_time_ms =
      std::chrono::duration_cast<std::chrono::nanoseconds>(last_time - start_time).count() * 1.0e-6;
    std::cerr << "Total time = " << total_time_ms << "ms\n\tpublish = " << publish_time_ms
              << "ms\n\tspin = " << spin_time_ms << "ms\n";
  }

  // ASSERT_TRUE(tester.received_resampled_reference_trajectory);

  // const auto save_directory = "/home/kyoichi-sugahara/workspace/log/reference_trajectory";
  // save_message_to_rosbag(
  //   save_directory, tester.resampled_reference_trajectory,
  //   "controller/debug/resampled_reference_trajectory");

  // std::cerr << "tester.cmd_msg's stamp: " << tester.cmd_msg->stamp.sec << "s "
  //           << tester.cmd_msg->stamp.nanosec << "ns" << std::endl;
  // std::cerr << "traj_msg.header.stamp: " << traj_msg.header.stamp.sec << "s "
  //           << traj_msg.header.stamp.nanosec << "ns" << std::endl;
  // EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

// TEST_F(FakeNodeFixture, left_turn)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Left turning trajectory: expect left steering
//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 1.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(1.0, 1.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(2.0, 2.0, 1.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   std::cerr << "lat steer tire angle: " << tester.cmd_msg->lateral.steering_tire_angle <<
//   std::endl; std::cerr << "lat steer tire rotation rate: "
//             << tester.cmd_msg->lateral.steering_tire_rotation_rate << std::endl;
//   EXPECT_GT(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
//   EXPECT_GT(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

// TEST_F(FakeNodeFixture, stopped)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();

//   const double steering_tire_angle = -0.5;
//   tester.publish_steer_angle(steering_tire_angle);

//   // Straight trajectory: expect no steering
//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(-1.0, 0.0, 0.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(1.0, 0.0, 0.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(2.0, 0.0, 0.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, steering_tire_angle);
//   EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
//   EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
// }

// longitudinal
// TEST_F(FakeNodeFixture, longitudinal_keep_velocity)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_odom_vx(1.0);
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish non stopping trajectory
//   Trajectory traj_msg;
//   traj_msg.header.stamp = tester.node->now();
//   traj_msg.header.frame_id = "map";
//   traj_msg.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj_msg.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj_msg);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 1.0);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.acceleration, 0.0);

//   // Generate another control message
//   tester.traj_pub->publish(traj_msg);
//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 1.0);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.acceleration, 0.0);
// }

// TEST_F(FakeNodeFixture, longitudinal_slow_down)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_acc();
//   tester.publish_default_steer();

//   const double odom_vx = 1.0;
//   tester.publish_odom_vx(odom_vx);

//   tester.publish_autonomous_operation_mode();

//   // Publish non stopping trajectory
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.5f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 0.5f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 0.5f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_LT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);

//   // Generate another control message
//   tester.traj_pub->publish(traj);
//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_LT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_accelerate)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   const double odom_vx = 0.5;
//   tester.publish_odom_vx(odom_vx);

//   tester.publish_autonomous_operation_mode();

//   // Publish non stopping trajectory
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_GT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);

//   // Generate another control message
//   tester.traj_pub->publish(traj);
//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);
//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_GT(tester.cmd_msg->longitudinal.speed, static_cast<float>(odom_vx));
//   EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_stopped)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish stopping trajectory
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 0.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 0.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 0.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_LT(
//     tester.cmd_msg->longitudinal.acceleration,
//     0.0f);  // when stopped negative acceleration to brake
// }

// TEST_F(FakeNodeFixture, longitudinal_reverse)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();

//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish reverse
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, -1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, -1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, -1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   EXPECT_LT(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_GT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_emergency)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_steer();
//   tester.publish_default_acc();

//   // Publish trajectory starting away from the current ego pose
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(10.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   // Emergencies (e.g., far from trajectory) produces braking command (0 vel, negative accel)
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 0.0f);
//   EXPECT_LT(tester.cmd_msg->longitudinal.acceleration, 0.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_not_check_steer_converged)
// {
//   const auto node_options = makeNodeOptions();
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();

//   // steering_tire_angle has to be larger than the threshold to check convergence.
//   const double steering_tire_angle = -0.5;
//   tester.publish_steer_angle(steering_tire_angle);

//   // Publish trajectory starting away from the current ego pose
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   // Not keep stopped state when the lateral control is not converged.
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 1.0f);
// }

// TEST_F(FakeNodeFixture, longitudinal_check_steer_converged)
// {
//   // set enable_keep_stopped_until_steer_convergence true
//   const auto node_options = makeNodeOptions(true);
//   ControllerTester tester(this, node_options);

//   tester.send_default_transform();
//   tester.publish_default_odom();
//   tester.publish_autonomous_operation_mode();
//   tester.publish_default_acc();

//   // steering_tire_angle has to be larger than the threshold to check convergence.
//   const double steering_tire_angle = -0.5;
//   tester.publish_steer_angle(steering_tire_angle);

//   // Publish trajectory starting away from the current ego pose
//   Trajectory traj;
//   traj.header.stamp = tester.node->now();
//   traj.header.frame_id = "map";
//   traj.points.push_back(test_utils::make_traj_point(0.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(50.0, 0.0, 1.0f));
//   traj.points.push_back(test_utils::make_traj_point(100.0, 0.0, 1.0f));
//   tester.traj_pub->publish(traj);

//   test_utils::waitForMessage(tester.node, this, tester.received_control_command);

//   ASSERT_TRUE(tester.received_control_command);
//   // Keep stopped state when the lateral control is not converged.
//   EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.speed, 0.0f);
// }
