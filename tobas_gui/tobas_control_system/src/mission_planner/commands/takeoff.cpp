// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/commands/takeoff.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
TakeoffWidget::TakeoffWidget()
{
  altitude_ = new field::AltitudeWidget();
  altitude_frame_ = new field::AltitudeFrameWidget();
  max_speed_ = new field::TakeoffMaxSpeedWidget();
  max_accel_ = new field::TakeoffMaxAccelWidget();
  max_jerk_ = new field::TakeoffMaxJerkWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();

  addField(altitude_);
  addField(altitude_frame_);
  addField(max_speed_, true);
  addField(max_accel_, true);
  addField(max_jerk_, true);
  addField(altitude_tolerance_);
}

const char* TakeoffWidget::name() const
{
  return "Takeoff";
}

mission::Takeoff TakeoffWidget::dump() const
{
  mission::Takeoff res;

  res.altitude = altitude();
  res.altitude_frame = altitudeFrame();
  res.max_speed = maxSpeed();
  res.max_accel = maxAccel();
  res.max_jerk = maxJerk();
  res.altitude_tolerance = altitudeTolerance();
  res.timeout = 0.;  // TODO

  return res;
}

void TakeoffWidget::load(const mission::Takeoff& src)
{
  altitude(src.altitude);
  altitudeFrame(src.altitude_frame);
  maxSpeed(src.max_speed);
  maxAccel(src.max_accel);
  maxJerk(src.max_jerk);
  altitudeTolerance(src.altitude_tolerance);
}

double TakeoffWidget::altitude() const
{
  return altitude_->getValue();
}

mission::AltitudeFrame TakeoffWidget::altitudeFrame() const
{
  return altitude_frame_->getValue();
}

double TakeoffWidget::maxSpeed() const
{
  return getValueOrDefault(max_speed_);
}

double TakeoffWidget::maxAccel() const
{
  return getValueOrDefault(max_accel_);
}

double TakeoffWidget::maxJerk() const
{
  return getValueOrDefault(max_jerk_);
}

double TakeoffWidget::altitudeTolerance() const
{
  return getValueOrDefault(altitude_tolerance_);
}

void TakeoffWidget::altitude(double value)
{
  altitude_->setValue(value);
}

void TakeoffWidget::altitudeFrame(mission::AltitudeFrame value)
{
  altitude_frame_->setValue(value);
}

void TakeoffWidget::maxSpeed(double value)
{
  setValueOrDefault(max_speed_, value);
}

void TakeoffWidget::maxAccel(double value)
{
  setValueOrDefault(max_accel_, value);
}

void TakeoffWidget::maxJerk(double value)
{
  setValueOrDefault(max_jerk_, value);
}

void TakeoffWidget::altitudeTolerance(double value)
{
  setValueOrDefault(altitude_tolerance_, value);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
