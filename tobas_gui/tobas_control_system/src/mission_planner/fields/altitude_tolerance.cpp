// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/altitude_tolerance.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
AltitudeToleranceWidget::AltitudeToleranceWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#NAV_MC_ALT_RAD
  // Use the ArduPilot default value (`WPNAV_WP_RADIUS`) because the PX4 default of 0.8 m is often too small.
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setMinimum(0.05);
  spin_box_->setMaximum(200.);
  spin_box_->setValue(2.);
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* AltitudeToleranceWidget::label() const
{
  return "Altitude Tolerance";
}

double AltitudeToleranceWidget::getValue() const
{
  return spin_box_->value();
}

void AltitudeToleranceWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
