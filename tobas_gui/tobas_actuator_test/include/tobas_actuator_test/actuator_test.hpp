// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./joint_test/joint_test.hpp"
#include "./rotor_test/rotor_test.hpp"

namespace tobas
{
namespace gui
{
namespace at
{
class ActuatorTestWidget : public qt::VerticalTabWidget
{
  Q_OBJECT

  using self = ActuatorTestWidget;
  using super = qt::VerticalTabWidget;

  // Without at least this much height, the `TabBar` text is clipped horizontally for some reason.
  static constexpr int kTabHeight = 35;
  static constexpr int kTabWidth = 70;

public:
  explicit ActuatorTestWidget(const RosQtBridge& bridge, const kdl::Tree& tree, const Drone& drone);

  void reset();
  void updateProject(const std::filesystem::path& proj_path);
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

private:
  const Drone& drone_;

  RotorTestWidget* rotor_test_;
  JointTestWidget* joint_test_;

  BaseWidget* getWidget(int index);
  const BaseWidget* getWidget(int index) const;

  void setTabsEnabled(bool enabled);
};
}  // namespace at
}  // namespace gui
}  // namespace tobas
