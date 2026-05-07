// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/takeoff_max_jerk.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
TakeoffMaxJerkWidget::TakeoffMaxJerkWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_JERK_AUTO
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(1.);
  spin_box_->setMaximum(80.);
  spin_box_->setValue(4.);
  spin_box_->setSuffix(" m/s³");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* TakeoffMaxJerkWidget::label() const
{
  return "Maximum Jerk";
}

double TakeoffMaxJerkWidget::getValue() const
{
  return spin_box_->value();
}

void TakeoffMaxJerkWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
