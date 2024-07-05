// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "map_loader/lanelet2_map_loader_node.hpp"

#include "lanelet2_local_projector.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <geography_utils/lanelet2_projector.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string>

using autoware_map_msgs::msg::LaneletMapBin;
using tier4_map_msgs::msg::MapProjectorInfo;

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);

  // subscription
  adaptor.init_sub(
    sub_map_projector_info_,
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { on_map_projector_info(msg); });

  declare_parameter<std::string>("lanelet2_map_path");
  declare_parameter<double>("center_line_resolution");
  declare_parameter<bool>("use_waypoints");
}

void Lanelet2MapLoaderNode::on_map_projector_info(
  const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  const auto lanelet2_filename = get_parameter("lanelet2_map_path").as_string();
  const auto center_line_resolution = get_parameter("center_line_resolution").as_double();
  const auto use_waypoints = get_parameter("use_waypoints").as_bool();

  // load map from file
  const auto map = load_map(lanelet2_filename, *msg);
  if (!map) {
    RCLCPP_ERROR(get_logger(), "Failed to load lanelet2_map. Not published.");
    return;
  }

  // overwrite centerline
  if (use_waypoints) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, center_line_resolution, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);
  }

  // create map bin msg
  const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());

  // create publisher and publish
  pub_map_bin_ =
    create_publisher<LaneletMapBin>("output/lanelet2_map", rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(map_bin_msg);
  RCLCPP_INFO(get_logger(), "Succeeded to load lanelet2_map. Map is published.");
}

lanelet::LaneletMapPtr Lanelet2MapLoaderNode::load_map(
  const std::string & lanelet2_filename,
  const tier4_map_msgs::msg::MapProjectorInfo & projector_info)
{
  lanelet::ErrorMessages errors{};
  if (projector_info.projector_type != tier4_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      geography_utils::get_lanelet2_projector(projector_info);
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, *projector, &errors);
    if (errors.empty()) {
      return map;
    }
  } else {
    const lanelet::projection::LocalProjector projector;
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    // overwrite local_x, local_y
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) {
        point.x() = point.attribute("local_x").asDouble().value();
      }
      if (point.hasAttribute("local_y")) {
        point.y() = point.attribute("local_y").asDouble().value();
      }
    }

    // realign lanelet borders using updated points
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }

    return map;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

LaneletMapBin Lanelet2MapLoaderNode::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{};
  std::string map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  LaneletMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.version_map_format = format_version;
  map_bin_msg.version_map = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapLoaderNode)
