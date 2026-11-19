// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/actuator_test.hpp"

#include <QLabel>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/widget.hpp>

namespace tobas::gui::at
{
ActuatorTestWidget::ActuatorTestWidget(const rqt::RosQtBridge& _bridge, const kdl::Tree& _tree, const Drone& _drone)
  : drone_(_drone)
{
  setTabSize(32, 64);
  enableWheelEvent(false);

  rotor_test_ = new RotorTestWidget(_bridge, _drone);
  addTab(createTabPage(rotor_test_), "Rotor Test");
  setTabEnabled(rotor_test_->parentWidget(), false);

  joint_test_ = new JointTestWidget(_bridge, _tree, _drone);
  addTab(createTabPage(joint_test_), "Joint Test");
  setTabEnabled(joint_test_->parentWidget(), false);
}

QWidget* ActuatorTestWidget::createTabPage(BaseWidget* _content)
{
  const auto page = new qt::Widget();
  page->setAutoFillBackground(true);

  const auto title = new QLabel(_content->title());
  title->setFont(qt::DefaultFont(cmn::kTitlePointSize, QFont::Bold));

  _content->layout()->setContentsMargins(0, 0, 0, 0);

  const auto rows = new QVBoxLayout(page);
  rows->addWidget(title, 0, Qt::AlignTop);
  rows->addSpacing(16);
  rows->addWidget(_content, 1);

  return page;
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
  setTabEnabled(rotor_test_->parentWidget(), rotor_test_->numRegisteredChannels() > 0);
  setTabEnabled(joint_test_->parentWidget(), joint_test_->numRegisteredChannels() > 0);

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
  return widget(_index)->findChild<BaseWidget*>({}, Qt::FindDirectChildrenOnly);
}

const BaseWidget* ActuatorTestWidget::getWidget(int _index) const
{
  return widget(_index)->findChild<BaseWidget*>({}, Qt::FindDirectChildrenOnly);
}
}  // namespace tobas::gui::at
