// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/actuator_test.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace at
{
ActuatorTestWidget::ActuatorTestWidget(const rqt::RosQtBridge& bridge, const kdl::Tree& tree, const Drone& drone)
  : drone_(drone)
{
  setTabSize(kTabWidth, kTabHeight);
  enableWheelEvent(false);

  rotor_test_ = new RotorTestWidget(bridge, drone);
  addTab(rotor_test_, "Rotor Test");

  joint_test_ = new JointTestWidget(bridge, tree, drone);
  addTab(joint_test_, "Joint Test");

  setTabsEnabled(false);
}

void ActuatorTestWidget::reset()
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->reset();
  }
}

void ActuatorTestWidget::updateProject(const fs::path& proj_path)
{
  setTabsEnabled(false);

  rotor_test_->updateProject(proj_path);
  joint_test_->updateInternalDataStructures();

  // Enable test functions only when at least one channel is registered.
  setTabEnabled(rotor_test_, rotor_test_->numRegisteredChannels() > 0);
  setTabEnabled(joint_test_, joint_test_->numRegisteredChannels() > 0);

  // Adjust distortion caused by showing or hiding tabs.
  update();
}

void ActuatorTestWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  rotor_test_->initializeRosInterfaces(node, ns);
  joint_test_->initializeRosInterfaces(node, ns);

  setTabsEnabled(true);
}

void ActuatorTestWidget::clearRosInterfaces()
{
  rotor_test_->clearRosInterfaces();
  joint_test_->clearRosInterfaces();

  setTabsEnabled(false);
}

BaseWidget* ActuatorTestWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseWidget>(widget(index));
}

const BaseWidget* ActuatorTestWidget::getWidget(int index) const
{
  return qt::qConstPointerCast<BaseWidget>(widget(index));
}

void ActuatorTestWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->setEnabled(enabled);
  }
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
