// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base_pose_commander.hpp"
#include "./joint_commander.hpp"

namespace tobas::gui::sim
{
class CommandersWidget : public QWidget
{
  Q_OBJECT

public:
  explicit CommandersWidget(const rqt::RosQtBridge& _bridge, const kdl::Tree& _tree, const Drone& _drone);

  void reset();
  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  BasePoseCommanderWidget* base_pose_commander_;
  JointCommanderWidget* joint_commander_;
};
}  // namespace tobas::gui::sim
