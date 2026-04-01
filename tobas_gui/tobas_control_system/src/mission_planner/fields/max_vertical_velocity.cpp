// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/fields/max_vertical_velocity.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
MaxVerticalVelocityWidget::MaxVerticalVelocityWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_Z_V_AUTO_DN
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_Z_V_AUTO_UP
  // TODO: 上昇と下降で設定を分ける
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(0.5);
  spin_box_->setMaximum(4.);
  spin_box_->setValue(1.5);
  spin_box_->setSuffix(" m/s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxVerticalVelocityWidget::label() const
{
  return "Maximum Vertical Velocity";
}

double MaxVerticalVelocityWidget::getValue() const
{
  return spin_box_->value();
}

void MaxVerticalVelocityWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
