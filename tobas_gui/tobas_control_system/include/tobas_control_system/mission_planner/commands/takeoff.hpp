// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_mission_items/mission_items.hpp>

#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/altitude_tolerance.hpp"
#include "../fields/takeoff_max_accel.hpp"
#include "../fields/takeoff_max_jerk.hpp"
#include "../fields/takeoff_max_speed.hpp"
#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class TakeoffWidget : public BaseCommandWidget
{
  Q_OBJECT

public:
  explicit TakeoffWidget();

  const char* name() const override;

  mission::Takeoff dump() const;
  void load(const mission::Takeoff& _src);

  double altitude() const;
  mission::AltitudeFrame altitudeFrame() const;
  double maxSpeed() const;
  double maxAccel() const;
  double maxJerk() const;
  double altitudeTolerance() const;

  void altitude(double _value);
  void altitudeFrame(mission::AltitudeFrame _value);
  void maxSpeed(double _value);
  void maxAccel(double _value);
  void maxJerk(double _value);
  void altitudeTolerance(double _value);

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::TakeoffMaxSpeedWidget* max_speed_;
  field::TakeoffMaxAccelWidget* max_accel_;
  field::TakeoffMaxJerkWidget* max_jerk_;
  field::AltitudeToleranceWidget* altitude_tolerance_;
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
