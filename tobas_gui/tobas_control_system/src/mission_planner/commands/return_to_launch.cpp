// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/commands/return_to_launch.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
ReturnToLaunchWidget::ReturnToLaunchWidget()
{
  min_alt_ = new field::RtlMinAltitudeWidget();
  max_hor_vel_ = new field::MaxHorizontalVelocityWidget();
  max_hor_acc_ = new field::MaxHorizontalAccelWidget();
  max_hor_jerk_ = new field::MaxHorizontalJerkWidget();
  max_ver_vel_ = new field::MaxVerticalVelocityWidget();
  max_ver_acc_ = new field::MaxVerticalAccelWidget();
  max_ver_jerk_ = new field::MaxVerticalJerkWidget();
  max_head_rate_ = new field::MaxHeadingRateWidget();
  max_head_acc_ = new field::MaxHeadingAccelWidget();
  acceptance_radius_ = new field::AcceptanceRadiusWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();

  addField(min_alt_, true);
  addField(max_hor_vel_, true);
  addField(max_hor_acc_, true);
  addField(max_hor_jerk_, true);
  addField(max_ver_vel_, true);
  addField(max_ver_acc_, true);
  addField(max_ver_jerk_, true);
  addField(max_head_rate_, true);
  addField(max_head_acc_, true);
  addField(acceptance_radius_);
  addField(altitude_tolerance_);
}

const char* ReturnToLaunchWidget::name() const
{
  return "Return to Launch";
}

mission::ReturnToLaunch ReturnToLaunchWidget::dump() const
{
  mission::ReturnToLaunch res;

  res.min_altitude = minAltitude();
  res.max_horizontal_velocity = maxHorizontalVelocity();
  res.max_horizontal_accel = maxHorizontalAccel();
  res.max_horizontal_jerk = maxHorizontalJerk();
  res.max_vertical_velocity = maxVerticalVelocity();
  res.max_vertical_accel = maxVerticalAccel();
  res.max_vertical_jerk = maxVerticalJerk();
  res.max_heading_rate = maxHeadingRate();
  res.max_heading_accel = maxHeadingAccel();
  res.acceptance_radius = acceptanceRadius();
  res.altitude_tolerance = altitudeTolerance();
  res.timeout = 0.;  // TODO

  return res;
}

void ReturnToLaunchWidget::load(const mission::ReturnToLaunch& src)
{
  minAltitude(src.min_altitude);
  maxHorizontalVelocity(src.max_horizontal_velocity);
  maxHorizontalAccel(src.max_horizontal_accel);
  maxHorizontalJerk(src.max_horizontal_jerk);
  maxVerticalVelocity(src.max_vertical_velocity);
  maxVerticalAccel(src.max_vertical_accel);
  maxVerticalJerk(src.max_vertical_jerk);
  maxHeadingRate(src.max_heading_rate);
  maxHeadingAccel(src.max_heading_accel);
  acceptanceRadius(src.acceptance_radius);
  altitudeTolerance(src.altitude_tolerance);
}

double ReturnToLaunchWidget::minAltitude() const
{
  return getValueOrDefault(min_alt_);
}

double ReturnToLaunchWidget::maxHorizontalVelocity() const
{
  return getValueOrDefault(max_hor_vel_);
}

double ReturnToLaunchWidget::maxHorizontalAccel() const
{
  return getValueOrDefault(max_hor_acc_);
}

double ReturnToLaunchWidget::maxVerticalVelocity() const
{
  return getValueOrDefault(max_ver_vel_);
}

double ReturnToLaunchWidget::maxHorizontalJerk() const
{
  return getValueOrDefault(max_hor_jerk_);
}

double ReturnToLaunchWidget::maxVerticalAccel() const
{
  return getValueOrDefault(max_ver_acc_);
}

double ReturnToLaunchWidget::maxVerticalJerk() const
{
  return getValueOrDefault(max_ver_jerk_);
}

double ReturnToLaunchWidget::maxHeadingRate() const
{
  return getValueOrDefault(max_head_rate_);
}

double ReturnToLaunchWidget::maxHeadingAccel() const
{
  return getValueOrDefault(max_head_acc_);
}

double ReturnToLaunchWidget::acceptanceRadius() const
{
  return getValueOrDefault(acceptance_radius_);
}

double ReturnToLaunchWidget::altitudeTolerance() const
{
  return getValueOrDefault(altitude_tolerance_);
}

void ReturnToLaunchWidget::minAltitude(double value)
{
  setValueOrDefault(min_alt_, value);
}

void ReturnToLaunchWidget::maxHorizontalVelocity(double value)
{
  setValueOrDefault(max_hor_vel_, value);
}

void ReturnToLaunchWidget::maxHorizontalAccel(double value)
{
  setValueOrDefault(max_hor_acc_, value);
}

void ReturnToLaunchWidget::maxHorizontalJerk(double value)
{
  setValueOrDefault(max_hor_jerk_, value);
}

void ReturnToLaunchWidget::maxVerticalVelocity(double value)
{
  setValueOrDefault(max_ver_vel_, value);
}

void ReturnToLaunchWidget::maxVerticalAccel(double value)
{
  setValueOrDefault(max_ver_acc_, value);
}

void ReturnToLaunchWidget::maxVerticalJerk(double value)
{
  setValueOrDefault(max_ver_jerk_, value);
}

void ReturnToLaunchWidget::maxHeadingRate(double value)
{
  setValueOrDefault(max_head_rate_, value);
}

void ReturnToLaunchWidget::maxHeadingAccel(double value)
{
  setValueOrDefault(max_head_acc_, value);
}

void ReturnToLaunchWidget::acceptanceRadius(double value)
{
  setValueOrDefault(acceptance_radius_, value);
}

void ReturnToLaunchWidget::altitudeTolerance(double value)
{
  setValueOrDefault(altitude_tolerance_, value);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
