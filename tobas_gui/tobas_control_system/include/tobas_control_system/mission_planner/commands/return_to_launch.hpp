// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_mission_items/mission_items.hpp>

#include "../fields/acceptance_radius.hpp"
#include "../fields/altitude_tolerance.hpp"
#include "../fields/max_heading_accel.hpp"
#include "../fields/max_heading_rate.hpp"
#include "../fields/max_horizontal_accel.hpp"
#include "../fields/max_horizontal_jerk.hpp"
#include "../fields/max_horizontal_velocity.hpp"
#include "../fields/max_vertical_accel.hpp"
#include "../fields/max_vertical_jerk.hpp"
#include "../fields/max_vertical_velocity.hpp"
#include "../fields/rtl_min_altitude.hpp"
#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class ReturnToLaunchWidget : public BaseCommandWidget
{
  Q_OBJECT

public:
  explicit ReturnToLaunchWidget();

  const char* name() const override;

  mission::ReturnToLaunch dump() const;
  void load(const mission::ReturnToLaunch& _src);

  double minAltitude() const;
  double maxHorizontalVelocity() const;
  double maxHorizontalAccel() const;
  double maxHorizontalJerk() const;
  double maxVerticalVelocity() const;
  double maxVerticalAccel() const;
  double maxVerticalJerk() const;
  double maxHeadingRate() const;
  double maxHeadingAccel() const;
  double acceptanceRadius() const;
  double altitudeTolerance() const;

  void minAltitude(double _value);
  void maxHorizontalVelocity(double _value);
  void maxHorizontalAccel(double _value);
  void maxHorizontalJerk(double _value);
  void maxVerticalVelocity(double _value);
  void maxVerticalAccel(double _value);
  void maxVerticalJerk(double _value);
  void maxHeadingRate(double _value);
  void maxHeadingAccel(double _value);
  void acceptanceRadius(double _value);
  void altitudeTolerance(double _value);

private:
  field::RtlMinAltitudeWidget* min_alt_;
  field::MaxHorizontalVelocityWidget* max_hor_vel_;
  field::MaxHorizontalAccelWidget* max_hor_acc_;
  field::MaxHorizontalJerkWidget* max_hor_jerk_;
  field::MaxVerticalVelocityWidget* max_ver_vel_;
  field::MaxVerticalAccelWidget* max_ver_acc_;
  field::MaxVerticalJerkWidget* max_ver_jerk_;
  field::MaxHeadingRateWidget* max_head_rate_;
  field::MaxHeadingAccelWidget* max_head_acc_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::AltitudeToleranceWidget* altitude_tolerance_;
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
