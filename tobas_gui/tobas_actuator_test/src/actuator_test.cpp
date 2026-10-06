// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/actuator_test.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace tobas
{
namespace gui
{
namespace at
{
ActuatorTestWidget::ActuatorTestWidget(const rqt::RosQtBridge& bridge, const kdl::Tree& tree, const Drone& drone)
  : drone_(drone)
{
  // Without at least this much height, the `TabBar` text is clipped horizontally for some reason.
  setTabSize(35, 70);

  rotor_test_ = new RotorTestWidget(bridge, drone);
  addTab(rotor_test_, "Rotor Test");
  setTabEnabled(rotor_test_, false);

  joint_test_ = new JointTestWidget(bridge, tree, drone);
  addTab(joint_test_, "Joint Test");
  setTabEnabled(joint_test_, false);

  enableWheelEvent(false);
}

void ActuatorTestWidget::reset()
{
  for (int i = 0; i < count(); ++i) {
    getWidget(i)->reset();
  }
}

void ActuatorTestWidget::setProjectPath(const QString& proj_path)
{
  rotor_test_->setProjectPath(proj_path);
}

void ActuatorTestWidget::updateInternalDataStructures()
{
  rotor_test_->updateInternalDataStructures();
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
}

void ActuatorTestWidget::clearRosInterfaces()
{
  rotor_test_->clearRosInterfaces();
  joint_test_->clearRosInterfaces();
}

BaseWidget* ActuatorTestWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseWidget>(widget(index));
}

const BaseWidget* ActuatorTestWidget::getWidget(int index) const
{
  return qt::qConstPointerCast<BaseWidget>(widget(index));
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
