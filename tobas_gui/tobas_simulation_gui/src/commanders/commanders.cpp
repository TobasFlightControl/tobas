// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/commanders/commanders.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas::gui::sim
{
CommandersWidget::CommandersWidget(const rqt::RosQtBridge& _bridge, const kdl::Tree& _tree, const Drone& _drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Commanders", cmn::kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  const auto scroll_rows = qt::createScrollableQVBoxLayout(rows);

  base_pose_commander_ = new BasePoseCommanderWidget(_bridge);
  scroll_rows->addWidget(base_pose_commander_);

  joint_commander_ = new JointCommanderWidget(_tree, _drone);
  scroll_rows->addWidget(joint_commander_);

  scroll_rows->addStretch();
}

void CommandersWidget::reset()
{
  base_pose_commander_->reset();
  joint_commander_->reset();
}

void CommandersWidget::updateInternalDataStructures()
{
  base_pose_commander_->updateInternalDataStructures();
  joint_commander_->updateInternalDataStructures();
}

void CommandersWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  base_pose_commander_->initializeRosInterfaces(_node, _ns);
  joint_commander_->initializeRosInterfaces(_node, _ns);
}

void CommandersWidget::clearRosInterfaces()
{
  base_pose_commander_->clearRosInterfaces();
  joint_commander_->clearRosInterfaces();
}
}  // namespace tobas::gui::sim
