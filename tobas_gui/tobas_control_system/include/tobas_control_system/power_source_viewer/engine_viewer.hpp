// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp>
#include <tobas_qt_tools/widgets/progress_bar.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_msgs/msg/engine_state.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class EngineViewerWidget : public QWidget
{
  Q_OBJECT

  using self = EngineViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 30;

  static constexpr double kMinOilTemp = 0.0;    // [degC]
  static constexpr double kMaxOilTemp = 130.0;  // [degC]

public:
  explicit EngineViewerWidget(const rqt::RosQtBridge& bridge, const Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const Drone& drone_;
  IcePropulsionSystemConfig::ConstSharedPtr iprop_;

  qt::ProgressBar* fuel_quantity_;
  qt::ProgressBar* oil_temp_;

  void updateFuelQuantity(const double& fuel_quantity);
  void updateOilTemperature(const double& oil_temp);

private Q_SLOTS:
  void engineStateCb(const tobas_msgs::msg::EngineState::ConstSharedPtr& engine_state);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
