// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/int_range.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
ParamGetterWidget_IntRange::ParamGetterWidget_IntRange(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  min_ = new IntGetter("min");
  cols->addWidget(min_);

  max_ = new IntGetter("max");
  cols->addWidget(max_);

  connect(min_, &IntGetter::valueChanged, this, &self::onValueChanged);
  connect(max_, &IntGetter::valueChanged, this, &self::onValueChanged);
}

ParamGetterWidget_IntRange::ValueType ParamGetterWidget_IntRange::getValue() const
{
  return { min(), max() };
}

bool ParamGetterWidget_IntRange::setValue(const ValueType& _src)
{
  min_->setValue(_src.lower);
  max_->setValue(_src.upper);
  return true;
}

void ParamGetterWidget_IntRange::setMinimum(int _minimum)
{
  min_->setMinimum(_minimum);
  max_->setMinimum(_minimum);
}

void ParamGetterWidget_IntRange::setMaximum(int _maximum)
{
  min_->setMaximum(_maximum);
  max_->setMaximum(_maximum);
}

void ParamGetterWidget_IntRange::setSingleStep(int _single_step)
{
  min_->setSingleStep(_single_step);
  max_->setSingleStep(_single_step);
}

void ParamGetterWidget_IntRange::setSuffix(const QString& _suffix)
{
  min_->setSuffix(_suffix);
  max_->setSuffix(_suffix);
}

int ParamGetterWidget_IntRange::min() const
{
  return min_->getValue();
}

int ParamGetterWidget_IntRange::max() const
{
  return max_->getValue();
}

bool ParamGetterWidget_IntRange::isValid() const
{
  return min() <= max();
}

void ParamGetterWidget_IntRange::onValueChanged(int)
{
  Q_EMIT valueChanged({ min(), max() });
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
