// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"

#include <tobas_string_tools/core.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
ParamGetterWidget_DoubleSpinBox::ParamGetterWidget_DoubleSpinBox(
  const QString& _param_name,
  const QString& _description_text)
  : super(_param_name, _description_text)
{
  spin_box_ = new qt::DoubleSpinBox();
  rows_->addWidget(spin_box_);
  connect(spin_box_, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

double ParamGetterWidget_DoubleSpinBox::getValue() const
{
  return spin_box_->value();
}

bool ParamGetterWidget_DoubleSpinBox::setValue(double _src)
{
  if (_src < spin_box_->minimum() || spin_box_->maximum() < _src) {
    return false;
  }

  spin_box_->setValue(_src);
  return true;
}

void ParamGetterWidget_DoubleSpinBox::setDecimals(int _decimals)
{
  spin_box_->setDecimals(_decimals);
}

void ParamGetterWidget_DoubleSpinBox::setMinimum(double _minimum)
{
  spin_box_->setMinimum(_minimum);
}

void ParamGetterWidget_DoubleSpinBox::setMaximum(double _maximum)
{
  spin_box_->setMaximum(_maximum);
}

void ParamGetterWidget_DoubleSpinBox::setSingleStep(double _single_step)
{
  spin_box_->setSingleStep(_single_step);
}

void ParamGetterWidget_DoubleSpinBox::setSuffix(const QString& _suffix)
{
  spin_box_->setSuffix(QString::fromStdString(str::convertToSuperscript(_suffix.toStdString())));
}

void ParamGetterWidget_DoubleSpinBox::onValueChanged(double _value)
{
  Q_EMIT valueChanged(_value);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
