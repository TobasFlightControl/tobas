// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/commands/land.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
LandWidget::LandWidget()
{
  speed_ = new field::LandSpeedWidget();

  addField(speed_, true);
}

const char* LandWidget::name() const
{
  return "Land";
}

mission::Land LandWidget::dump() const
{
  mission::Land res;

  res.speed = speed();
  res.timeout = 0.;  // TODO

  return res;
}

void LandWidget::load(const mission::Land& src)
{
  speed(src.speed);
}

double LandWidget::speed() const
{
  return getValueOrDefault(speed_);
}

void LandWidget::speed(double value)
{
  setValueOrDefault(speed_, value);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
