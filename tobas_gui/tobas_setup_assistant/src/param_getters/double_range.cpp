// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/double_range.hpp"

namespace tobas::gui::sa
{
ParamGetterWidget_DoubleRange::ParamGetterWidget_DoubleRange(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  min_ = new qt::LabeledDoubleSpinBox("min");
  cols->addWidget(min_);

  max_ = new qt::LabeledDoubleSpinBox("max");
  cols->addWidget(max_);

  connect(min_, &qt::LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
  connect(max_, &qt::LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
}

ParamGetterWidget_DoubleRange::ValueType ParamGetterWidget_DoubleRange::getValue() const
{
  return { min(), max() };
}

bool ParamGetterWidget_DoubleRange::setValue(const ValueType& _src)
{
  min_->setValue(_src.lower);
  max_->setValue(_src.upper);
  return true;
}

void ParamGetterWidget_DoubleRange::setDecimals(int _decimals)
{
  min_->setDecimals(_decimals);
  max_->setDecimals(_decimals);
}

void ParamGetterWidget_DoubleRange::setMinimum(double _minimum)
{
  min_->setMinimum(_minimum);
  max_->setMinimum(_minimum);
}

void ParamGetterWidget_DoubleRange::setMaximum(double _maximum)
{
  min_->setMaximum(_maximum);
  max_->setMaximum(_maximum);
}

void ParamGetterWidget_DoubleRange::setSingleStep(double _single_step)
{
  min_->setSingleStep(_single_step);
  max_->setSingleStep(_single_step);
}

void ParamGetterWidget_DoubleRange::setSuffix(const QString& _suffix)
{
  min_->setSuffix(_suffix);
  max_->setSuffix(_suffix);
}

double ParamGetterWidget_DoubleRange::min() const
{
  return min_->getValue();
}

double ParamGetterWidget_DoubleRange::max() const
{
  return max_->getValue();
}

bool ParamGetterWidget_DoubleRange::isValid() const
{
  return min() <= max();
}

void ParamGetterWidget_DoubleRange::onValueChanged(double)
{
  Q_EMIT valueChanged({ min(), max() });
}
}  // namespace tobas::gui::sa
