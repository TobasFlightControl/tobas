// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./joint_test/joint_test.hpp"
#include "./rotor_test/rotor_test.hpp"

namespace tobas::gui::at
{
class ActuatorTestWidget : public qt::VerticalTabWidget
{
  Q_OBJECT

public:
  explicit ActuatorTestWidget(const rqt::RosQtBridge& _bridge, const kdl::Tree& _tree, const Drone& _drone);

  void reset();
  void updateInternalDataStructures();
  void setProjectPath(const QString& _proj_path);
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  const Drone& drone_;

  RotorTestWidget* rotor_test_;
  JointTestWidget* joint_test_;

  BaseWidget* getWidget(int _index);
  const BaseWidget* getWidget(int _index) const;
};
}  // namespace tobas::gui::at
