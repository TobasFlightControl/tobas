// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/rotor_marker_publisher.hpp"

#include <ranges>

#include <QDebug>

#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_kdl_conversions/kdl_urdf.hpp>
#include <tobas_ros2_tools/register.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
RotorMarkerPublisher::RotorMarkerPublisher(rclcpp::Node::SharedPtr node, const uadf::Model& uadf)
  : node_(node), uadf_(uadf)
{
  markers_pub_ =
    ros2::createPublisher<visualization_msgs::msg::MarkerArray>(node, "visualization_marker_array", false, true);

  connect(&publish_markers_timer_, &QTimer::timeout, this, &self::publishTimerCb);
}

void RotorMarkerPublisher::updateInternalDataStructures()
{
  // Stop publishing markers.
  publish_markers_timer_.stop();

  // Hide all current markers.
  for (auto& marker : markers_.markers) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  publishMarkers();

  // Delete all markers.
  markers_.markers.clear();

  // Add markers for propeller links.
  for (const auto& [id, elem] : std::views::enumerate(uadf_.thrusts)) {
    const auto& [joint_name, thrust] = elem;
    const auto joint = uadf_.urdf->getJoint(joint_name);

    // Thrust line of action.
    constexpr double kArrowLength = 0.2;  // TODO: Reflect the expected maximum thrust in the arrow length.
    const auto arrow_start = kdl::Vector::Zero();
    const auto arrow_end = kdl::vectorUrdfToKdl(joint->axis) * kArrowLength;
    const auto arrow_scale = kdl::Vector(0.1, 0.2, 0.3) * kArrowLength;

    // Create the marker.
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = joint->child_link_name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.resize(2);
    kdl::pointKDLToMsg(arrow_start, marker.points.at(0));
    kdl::pointKDLToMsg(arrow_end, marker.points.at(1));
    kdl::vectorKDLToMsg(arrow_scale, marker.scale);

    // Color by rotation direction.
    marker.color.a = 1.0;
    switch (thrust.direction) {
      case uadf::Thrust::CW:
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        break;
      case uadf::Thrust::CCW:
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        break;
      default:
        throw;
    }

    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);  // Infinite lifetime.
    marker.frame_locked = true;                               // Keep fixed to the frame even if TF changes.

    // Add the marker.
    markers_.markers.push_back(marker);
  }

  // TODO: Also show markers for tilt joints and control surfaces.

  // Start publishing markers.
  publish_markers_timer_.start(100);
}

void RotorMarkerPublisher::publishMarkers()
{
  // Fill timestamps.
  const auto cur_time = node_->now();
  for (auto& marker : markers_.markers) {
    marker.header.stamp = cur_time;
  }

  // Publish markers.
  auto markers_ptr = make_unique<visualization_msgs::msg::MarkerArray>(markers_);
  markers_pub_->publish(std::move(markers_ptr));
}

void RotorMarkerPublisher::publishTimerCb()
{
  publishMarkers();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
