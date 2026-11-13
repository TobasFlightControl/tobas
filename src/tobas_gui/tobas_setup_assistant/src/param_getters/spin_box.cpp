// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/spin_box.hpp"

#include <tobas_string_tools/core.hpp>

namespace tobas::gui::sa
{
ParamGetterWidget_SpinBox::ParamGetterWidget_SpinBox(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  spin_box_ = new qt::SpinBox();
  rows_->addWidget(spin_box_);
  connect(spin_box_, qOverload<int>(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

int ParamGetterWidget_SpinBox::getValue() const
{
  return spin_box_->value();
}

bool ParamGetterWidget_SpinBox::setValue(int _src)
{
  if (_src < spin_box_->minimum() || spin_box_->maximum() < _src) {
    return false;
  }

  spin_box_->setValue(_src);
  return true;
}

void ParamGetterWidget_SpinBox::setMinimum(int _minimum)
{
  spin_box_->setMinimum(_minimum);
}

void ParamGetterWidget_SpinBox::setMaximum(int _maximum)
{
  spin_box_->setMaximum(_maximum);
}

void ParamGetterWidget_SpinBox::setSingleStep(int _single_step)
{
  spin_box_->setSingleStep(_single_step);
}

void ParamGetterWidget_SpinBox::setSuffix(const QString& _suffix)
{
  spin_box_->setSuffix(QString::fromStdString(str::convertToSuperscript(_suffix.toStdString())));
}

void ParamGetterWidget_SpinBox::onValueChanged(int _value)
{
  Q_EMIT valueChanged(_value);
}
}  // namespace tobas::gui::sa
