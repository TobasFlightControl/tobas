// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_qt_tools/widgets/progress_bar.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class BatteryViewerWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 30;

public:
  explicit BatteryViewerWidget(const RosQtBridge& bridge, const Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const Drone& drone_;
  ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  qt::ProgressBar* voltage_;
  qt::ProgressBar* current_;

  void updateVoltage(const double& voltage);
  void updateCurrent(const double& current);

private Q_SLOTS:
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
