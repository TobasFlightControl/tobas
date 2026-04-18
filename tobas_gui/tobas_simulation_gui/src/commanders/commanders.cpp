// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/commanders/commanders.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace sim
{
CommandersWidget::CommandersWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const kdl::Tree& tree,
  const Drone& drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Commanders", cmn::kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  const auto scroll_rows = qt::createScrollableQVBoxLayout(rows);

  base_pose_commander_ = new BasePoseCommanderWidget(node, bridge, drone);
  scroll_rows->addWidget(base_pose_commander_);

  joint_commander_ = new JointCommanderWidget(node, tree, drone);
  scroll_rows->addWidget(joint_commander_);

  scroll_rows->addStretch();
}

void CommandersWidget::updateInternalDataStructures()
{
  base_pose_commander_->updateInternalDataStructures();
  joint_commander_->updateInternalDataStructures();
}

bool CommandersWidget::start(ch::milliseconds timeout)
{
  if (!base_pose_commander_->start(timeout)) {
    return false;
  }

  if (!joint_commander_->start(timeout)) {
    return false;
  }

  return true;
}

void CommandersWidget::reset()
{
  base_pose_commander_->reset();
  joint_commander_->reset();
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
