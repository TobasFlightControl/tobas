// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base_pose_commander.hpp"
#include "./joint_commander.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
class CommandersWidget : public QWidget
{
  Q_OBJECT

  using self = CommandersWidget;
  using super = QWidget;

public:
  explicit CommandersWidget(const RosQtBridge& bridge, const kdl::Tree& tree, const Drone& drone);

  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

  bool start(std::chrono::milliseconds timeout);
  void reset();

private:
  BasePoseCommanderWidget* base_pose_commander_;
  JointCommanderWidget* joint_commander_;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
