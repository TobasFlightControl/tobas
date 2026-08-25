// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./commands_publisher.hpp"

namespace tobas
{
namespace gui
{
namespace at
{
class JointTestWidget : public BaseWidget
{
  Q_OBJECT

  using self = JointTestWidget;
  using super = BaseWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit JointTestWidget(const RosQtBridge& bridge, const kdl::Tree& tree, const Drone& drone);

  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

  int numRegisteredChannels() const;

private:
  const kdl::Tree& tree_;
  const Drone& drone_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* zero_button_;
  QPushButton* home_button_;

  JointCommandsPublisherWidget* commands_publisher_;

  bool ros_initialized_ = false;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onZeroButtonClicked();
  void onHomeButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace at
}  // namespace gui
}  // namespace tobas
