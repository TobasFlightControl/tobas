// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/rotor_marker_publisher.hpp"

#include <ranges>

#include <QDebug>

#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_kdl_conversions/kdl_urdf.hpp>

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
  // マーカの発行を停止
  publish_markers_timer_.stop();

  // 現在のマーカを全て非表示にする
  for (auto& marker : markers_.markers) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  publishMarkers();

  // 全てのマーカを削除
  markers_.markers.clear();

  // プロペラリンクのマーカを追加
  for (const auto& [id, elem] : std::views::enumerate(uadf_.thrusts)) {
    const auto& [joint_name, thrust] = elem;
    const auto joint = uadf_.urdf->getJoint(joint_name);

    // 推力の作用線
    const auto arrow_start = kdl::Vector::Zero();
    const auto arrow_end = kdl::vectorUrdfToKdl(joint->axis) * kArrowLength;
    const auto arrow_scale = kdl::Vector(0.1, 0.2, 0.3) * kArrowLength;

    // マーカを作成
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = joint->child_link_name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.points.resize(2);
    kdl::pointKDLToMsg(arrow_start, marker.points.at(0));
    kdl::pointKDLToMsg(arrow_end, marker.points.at(1));
    kdl::vectorKDLToMsg(arrow_scale, marker.scale);

    // 回転方向によって色分け
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

    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);  // 無限の生存期間
    marker.frame_locked = true;                               // TFが変化してもフレームに固定

    // マーカを追加
    markers_.markers.push_back(marker);
  }

  // TODO: チルトジョイントと操舵面のマーカも表示

  // マーカを発行開始
  publish_markers_timer_.start(100);
}

void RotorMarkerPublisher::publishMarkers()
{
  // Fill timestamps
  const auto cur_time = node_->now();
  for (auto& marker : markers_.markers) {
    marker.header.stamp = cur_time;
  }

  // Publish markers
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
