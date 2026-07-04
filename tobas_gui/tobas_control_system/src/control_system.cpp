// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/control_system.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
ControlSystemWidget::ControlSystemWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge, const Drone& drone)
  : drone_(drone)
{
  // Components
  arm_state_banner_ = new ArmStateBanner(bridge);
  pose_viewer_ = new PoseViewerWidget(bridge);
  power_source_viewer_ = new PowerSourceViewerWidget(bridge, drone);
  cpu_viewer_ = new CpuViewerWidget(bridge);
  gnss_viewer_ = new GnssViewerWidget(bridge);
  rcin_viewer_ = new rcin::RCInputViewerWidget(bridge);
  rotors_viewer_ = new RotorsViewerWiddget(bridge, drone);
  console_ = new ConsoleWidget(bridge);
  status_viewer_ = new StatusViewerWidget(bridge);
  mission_planner_ = new MissionPlannerWidget(node, bridge);

  // Layout
  const auto cols3 = new QHBoxLayout();
  cols3->addWidget(qt::makeGroup("Power", power_source_viewer_), 2);
  cols3->addWidget(qt::makeGroup("CPU", cpu_viewer_), 2);
  cols3->addWidget(qt::makeGroup("GNSS", gnss_viewer_), 1);

  const auto rows2 = new QVBoxLayout();
  rows2->addLayout(cols3, 0);
  rows2->addWidget(qt::makeGroup("RC Input", rcin_viewer_), 2);
  rows2->addWidget(qt::makeGroup("Rotors", rotors_viewer_), 2);
  rows2->addWidget(qt::makeGroup("Console", console_), 3);

  const auto cols2 = new QHBoxLayout();
  cols2->addLayout(rows2, 3);
  cols2->addWidget(qt::makeGroup("Status", status_viewer_), 1);

  const auto rows1 = new QVBoxLayout();
  rows1->addWidget(qt::makeGroup("Pose", pose_viewer_), 2);
  rows1->addLayout(cols2, 3);

  const auto cols1 = new QHBoxLayout();
  cols1->addLayout(rows1, 2);
  cols1->addWidget(qt::makeGroup("Mission Planner", mission_planner_), 3);

  const auto rows0 = new QVBoxLayout();
  rows0->addWidget(arm_state_banner_);
  rows0->addLayout(cols1, 1);
  setLayout(rows0);
}

void ControlSystemWidget::reset()
{
  arm_state_banner_->reset();
  pose_viewer_->reset();
  power_source_viewer_->reset();
  cpu_viewer_->reset();
  gnss_viewer_->reset();
  rcin_viewer_->reset();
  rotors_viewer_->reset();
  console_->reset();
  status_viewer_->reset();
  mission_planner_->reset();
}

void ControlSystemWidget::updateInternalDataStructures()
{
  reset();

  const auto ns = '/' + drone_.name;

  power_source_viewer_->updateInternalDataStructures();
  rotors_viewer_->updateInternalDataStructures();
  mission_planner_->updateNamespace(ns);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
