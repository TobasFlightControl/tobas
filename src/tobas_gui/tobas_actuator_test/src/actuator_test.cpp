// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/actuator_test.hpp"

#include <tobas_qt_tools/cast.hpp>

namespace tobas::gui::at
{
ActuatorTestWidget::ActuatorTestWidget(const rqt::RosQtBridge& _bridge, const kdl::Tree& _tree, const Drone& _drone)
  : drone_(_drone)
{
  // Without at least this much height, the `TabBar` text is clipped horizontally for some reason.
  setTabSize(35, 70);

  rotor_test_ = new RotorTestWidget(_bridge, _drone);
  addTab(rotor_test_, "Rotor Test");
  setTabEnabled(rotor_test_, false);

  joint_test_ = new JointTestWidget(_bridge, _tree, _drone);
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

void ActuatorTestWidget::setProjectPath(const QString& _proj_path)
{
  rotor_test_->setProjectPath(_proj_path);
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

void ActuatorTestWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  rotor_test_->initializeRosInterfaces(_node, _ns);
  joint_test_->initializeRosInterfaces(_node, _ns);
}

void ActuatorTestWidget::clearRosInterfaces()
{
  rotor_test_->clearRosInterfaces();
  joint_test_->clearRosInterfaces();
}

BaseWidget* ActuatorTestWidget::getWidget(int _index)
{
  return qt::qPointerCast<BaseWidget>(widget(_index));
}

const BaseWidget* ActuatorTestWidget::getWidget(int _index) const
{
  return qt::qConstPointerCast<BaseWidget>(widget(_index));
}
}  // namespace tobas::gui::at
