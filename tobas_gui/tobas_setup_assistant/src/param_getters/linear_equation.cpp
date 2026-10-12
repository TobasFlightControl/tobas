// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/linear_equation.hpp"

#include <tobas_string_tools/core.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
ParamGetterWidget_LinearEquation::ParamGetterWidget_LinearEquation(
  const QString& _param_name,
  const QString& _left,
  const QString& _value,
  const QString& _description_text)
  : super(_param_name, _description_text)
{
  c0_ = new qt::DoubleSpinBox();
  c1_ = new qt::DoubleSpinBox();
  suffix_ = new QLabel();

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  cols->addWidget(new QLabel(_left + " = "));
  cols->addWidget(c1_);
  cols->addWidget(new QLabel(_value + " + "));
  cols->addWidget(c0_);
  cols->addWidget(suffix_);
  cols->addStretch();

  connect(c0_, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(c1_, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

ParamGetterWidget_LinearEquation::ValueType ParamGetterWidget_LinearEquation::getValue() const
{
  return { c0_->value(), c1_->value() };
}

bool ParamGetterWidget_LinearEquation::setValue(const ValueType& _src)
{
  c0_->setValue(_src.first);
  c1_->setValue(_src.second);

  return true;
}

void ParamGetterWidget_LinearEquation::setDecimals(int _decimals)
{
  c0_->setDecimals(_decimals);
  c1_->setDecimals(_decimals);
}

void ParamGetterWidget_LinearEquation::setSuffix(const QString& _suffix)
{
  suffix_->setText(QString::fromStdString(str::convertToSuperscript(_suffix.toStdString())));
}

void ParamGetterWidget_LinearEquation::onValueChanged()
{
  Q_EMIT valueChanged(getValue());
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
