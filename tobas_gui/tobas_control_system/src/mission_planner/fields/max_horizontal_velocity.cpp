// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/max_horizontal_velocity.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
MaxHorizontalVelocityWidget::MaxHorizontalVelocityWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_XY_CRUISE
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(1.);  // 3 m/s -> 1 m/s
  spin_box_->setMaximum(20.);
  spin_box_->setValue(5.);
  spin_box_->setSuffix(" m/s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxHorizontalVelocityWidget::label() const
{
  return "Maximum Horizontal Velocity";
}

double MaxHorizontalVelocityWidget::getValue() const
{
  return spin_box_->value();
}

void MaxHorizontalVelocityWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
