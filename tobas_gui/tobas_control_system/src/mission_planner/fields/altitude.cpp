// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/altitude.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
AltitudeWidget::AltitudeWidget()
{
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setValue(5.);
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* AltitudeWidget::label() const
{
  return "Altitude";
}

double AltitudeWidget::getValue() const
{
  return spin_box_->value();
}

void AltitudeWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
