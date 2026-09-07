// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTimer>
#include <rclcpp/node.hpp>

#include <tobas_uadf/model.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace tobas
{
namespace gui
{
namespace sa
{
class RotorMarkerPublisher : public QObject
{
  Q_OBJECT

  using self = RotorMarkerPublisher;
  using super = QObject;

public:
  explicit RotorMarkerPublisher(rclcpp::Node::SharedPtr node, const uadf::Model& uadf);

  void updateInternalDataStructures();

  void publishMarkers();

private:
  const rclcpp::Node::SharedPtr node_;
  const uadf::Model& uadf_;

  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  QTimer publish_markers_timer_;

private Q_SLOTS:
  void publishTimerCb();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
