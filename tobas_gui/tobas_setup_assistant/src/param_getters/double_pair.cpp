// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/double_pair.hpp"

namespace tobas::gui::sa
{
ParamGetterWidget_DoublePair::ParamGetterWidget_DoublePair(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  first_ = new qt::DoubleSpinBox();
  cols->addWidget(first_);

  second_ = new qt::DoubleSpinBox();
  cols->addWidget(second_);

  connect(first_, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(second_, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

std::pair<double, double> ParamGetterWidget_DoublePair::getValue() const
{
  return { first(), second() };
}

bool ParamGetterWidget_DoublePair::setValue(const std::pair<double, double>& _src)
{
  first_->setValue(_src.first);
  second_->setValue(_src.second);
  return true;
}

void ParamGetterWidget_DoublePair::setDecimals(int _decimals)
{
  first_->setDecimals(_decimals);
  second_->setDecimals(_decimals);
}

void ParamGetterWidget_DoublePair::setMinimum(double _minimum)
{
  first_->setMinimum(_minimum);
  second_->setMinimum(_minimum);
}

void ParamGetterWidget_DoublePair::setMaximum(double _maximum)
{
  first_->setMaximum(_maximum);
  second_->setMaximum(_maximum);
}

void ParamGetterWidget_DoublePair::setSingleStep(double _single_step)
{
  first_->setSingleStep(_single_step);
  second_->setSingleStep(_single_step);
}

void ParamGetterWidget_DoublePair::setSuffix(const QString& _suffix)
{
  first_->setSuffix(_suffix);
  second_->setSuffix(_suffix);
}

double ParamGetterWidget_DoublePair::first() const
{
  return first_->value();
}

double ParamGetterWidget_DoublePair::second() const
{
  return second_->value();
}

void ParamGetterWidget_DoublePair::onValueChanged(double)
{
  Q_EMIT valueChanged({ first(), second() });
}
}  // namespace tobas::gui::sa
