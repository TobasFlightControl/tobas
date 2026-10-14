// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./commands_publisher.hpp"

namespace tobas::gui::at
{
class JointTestWidget : public BaseWidget
{
  Q_OBJECT

  using self = JointTestWidget;

public:
  explicit JointTestWidget(const rqt::RosQtBridge& _bridge, const kdl::Tree& _tree, const Drone& _drone);

  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

  int numRegisteredChannels() const;

private:
  const kdl::Tree& tree_;
  const Drone& drone_;

  qt::ToggleButton* start_stop_button_;
  QPushButton* zero_button_;
  QPushButton* home_button_;

  JointCommandsPublisherWidget* commands_publisher_;

  bool ros_initialized_ = false;
  bool running_ = false;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  void updateActionAvailability();

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onZeroButtonClicked();
  void onHomeButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
};
}  // namespace tobas::gui::at
