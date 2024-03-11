cmake_minimum_required(VERSION 3.14)
project(external_controller_executor)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(LATERAL_CONTROLLER_PERFORMANCE_ANALYZER_NODE external_controller_executor_node)
ament_auto_add_library(${LATERAL_CONTROLLER_PERFORMANCE_ANALYZER_NODE} SHARED
  include/trajectory_follower_node/controller_node.hpp
  src/controller_node.cpp
)

rclcpp_components_register_node(${LATERAL_CONTROLLER_PERFORMANCE_ANALYZER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_node::Controller"
  EXECUTABLE ${LATERAL_CONTROLLER_PERFORMANCE_ANALYZER_NODE}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
  param
  launch
  config
)
