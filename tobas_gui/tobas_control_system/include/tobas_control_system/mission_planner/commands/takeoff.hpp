// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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

  using self = TakeoffWidget;
  using super = BaseCommandWidget;

public:
  explicit TakeoffWidget();

  const char* name() const override;

  double altitude() const;
  mission::AltitudeFrame altitudeFrame() const;
  double maxSpeed() const;
  double maxAccel() const;
  double maxJerk() const;
  double altitudeTolerance() const;

  void altitude(double value);
  void altitudeFrame(mission::AltitudeFrame value);
  void maxSpeed(double value);
  void maxAccel(double value);
  void maxJerk(double value);
  void altitudeTolerance(double value);

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
