// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/max_horizontal_accel.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
MaxHorizontalAccelWidget::MaxHorizontalAccelWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_HOR_MAX
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(2.);
  spin_box_->setMaximum(15.);
  spin_box_->setValue(5.);
  spin_box_->setSuffix(" m/s²");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxHorizontalAccelWidget::label() const
{
  return "Maximum Horizontal Acceleration";
}

double MaxHorizontalAccelWidget::getValue() const
{
  return spin_box_->value();
}

void MaxHorizontalAccelWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
