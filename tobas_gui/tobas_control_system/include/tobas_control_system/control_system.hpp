// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./arm_state_banner.hpp"
#include "./console.hpp"
#include "./cpu_viewer.hpp"
#include "./gnss_viewer.hpp"
#include "./health_viewer/health_viewer.hpp"
#include "./mission_planner/mission_planner.hpp"
#include "./pose_viewer.hpp"
#include "./power_source_viewer/power_source_viewer.hpp"
#include "./rcin_viewer/rcin_viewer.hpp"
#include "./rotors_viewer/rotors_viewer.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class ControlSystemWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ControlSystemWidget(const rqt::RosQtBridge& bridge, const Drone& drone);

  void reset();
  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

private:
  const Drone& drone_;

  ArmStateBanner* arm_state_banner_;
  PoseViewerWidget* pose_viewer_;
  PowerSourceViewerWidget* power_source_viewer_;
  CpuViewerWidget* cpu_viewer_;
  GnssViewerWidget* gnss_viewer_;
  rcin::RCInputViewerWidget* rcin_viewer_;
  RotorsViewerWiddget* rotors_viewer_;
  ConsoleWidget* console_;
  HealthViewerWidget* health_viewer_;
  MissionPlannerWidget* mission_planner_;
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
