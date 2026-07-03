// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/max_vertical_accel.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
MaxVerticalAccelWidget::MaxVerticalAccelWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_DOWN_MAX
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_UP_MAX
  // TODO: Use separate settings for ascent and descent.
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(2.);
  spin_box_->setMaximum(15.);
  spin_box_->setValue(3.);
  spin_box_->setSuffix(" m/s²");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxVerticalAccelWidget::label() const
{
  return "Maximum Vertical Acceleration";
}

double MaxVerticalAccelWidget::getValue() const
{
  return spin_box_->value();
}

void MaxVerticalAccelWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
