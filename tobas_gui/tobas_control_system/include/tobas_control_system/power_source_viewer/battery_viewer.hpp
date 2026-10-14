// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_qt_tools/widgets/progress_bar.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas::gui::ctrl
{
class BatteryViewerWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryViewerWidget;

public:
  explicit BatteryViewerWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone);

  void reset();
  void updateInternalDataStructures();

private:
  const Drone& drone_;
  ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  qt::ProgressBar* voltage_;
  qt::ProgressBar* current_;

  void updateVoltage(double _voltage);
  void updateCurrent(double _current);

private Q_SLOTS:
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& _battery);
};
}  // namespace tobas::gui::ctrl
