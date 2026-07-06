// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/takeoff_max_speed.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
TakeoffMaxSpeedWidget::TakeoffMaxSpeedWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_TKO_SPEED
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(1.0);
  spin_box_->setMaximum(5.0);
  spin_box_->setValue(1.5);
  spin_box_->setSuffix(" m/s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* TakeoffMaxSpeedWidget::label() const
{
  return "Maximum Speed";
}

double TakeoffMaxSpeedWidget::getValue() const
{
  return spin_box_->value();
}

void TakeoffMaxSpeedWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
