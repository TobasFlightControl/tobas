// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./debug.hpp"
#include "./loop_type.hpp"
#include "./pose.hpp"
#include "./sbus.hpp"
#include "./world/world.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
class SimulationSettingsWidget : public QWidget
{
  Q_OBJECT

  using self = SimulationSettingsWidget;
  using super = QWidget;

public:
  explicit SimulationSettingsWidget(rclcpp::Node::SharedPtr node);

  LoopType loopType() const;

  std::filesystem::path worldPath() const;

  double x() const;      // [m]
  double y() const;      // [m]
  double z() const;      // [m]
  double roll() const;   // [rad]
  double pitch() const;  // [rad]
  double yaw() const;    // [rad]

  std::filesystem::path sbusDevicePath() const;

  bool userDebug() const;

private:
  LoopTypeWidget* type_;
  WorldWidget* world_;
  PoseWidget* pose_;
  SbusWidget* sbus_;
  DebugWidget* debug_;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
