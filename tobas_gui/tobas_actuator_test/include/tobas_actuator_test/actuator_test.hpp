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

  static constexpr int kTabHeight = 35;  // これ以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;

public:
  explicit ActuatorTestWidget(
    rclcpp::Node::SharedPtr node,
    const RosQtBridge& bridge,
    const kdl::Tree& tree,
    const Drone& drone);

  void reset();
  void updateInternalDataStructures();

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
