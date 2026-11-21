// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/utils/double_range.hpp"

#include <QHBoxLayout>

namespace tobas::gui::sa
{
DoubleRangeWidget::DoubleRangeWidget(QWidget* _parent) : super(_parent)
{
  min_ = new qt::LabeledDoubleSpinBox("min");
  max_ = new qt::LabeledDoubleSpinBox("max");

  const auto cols = new QHBoxLayout(this);
  cols->setContentsMargins(0, 0, 0, 0);
  cols->addWidget(min_);
  cols->addWidget(max_);

  connect(min_, &qt::LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
  connect(max_, &qt::LabeledDoubleSpinBox::valueChanged, this, &self::onValueChanged);
}

st::Range<double> DoubleRangeWidget::value() const
{
  // Keep reversed bounds readable so the setting page can validate user input.
  st::Range<double> result;
  result.set(min_->getValue(), max_->getValue());
  return result;
}

void DoubleRangeWidget::setValue(const st::Range<double>& _value)
{
  min_->setValue(_value.lower);
  max_->setValue(_value.upper);
}

bool DoubleRangeWidget::isValid() const
{
  return value().isValid();
}

void DoubleRangeWidget::setDecimals(int _decimals)
{
  min_->setDecimals(_decimals);
  max_->setDecimals(_decimals);
}

void DoubleRangeWidget::setMinimum(double _minimum)
{
  min_->setMinimum(_minimum);
  max_->setMinimum(_minimum);
}

void DoubleRangeWidget::setMaximum(double _maximum)
{
  min_->setMaximum(_maximum);
  max_->setMaximum(_maximum);
}

void DoubleRangeWidget::setRange(double _minimum, double _maximum)
{
  min_->setMinimum(_minimum);
  min_->setMaximum(_maximum);
  max_->setMinimum(_minimum);
  max_->setMaximum(_maximum);
}

void DoubleRangeWidget::setSingleStep(double _single_step)
{
  min_->setSingleStep(_single_step);
  max_->setSingleStep(_single_step);
}

void DoubleRangeWidget::setSuffix(const QString& _suffix)
{
  min_->setSuffix(_suffix);
  max_->setSuffix(_suffix);
}

void DoubleRangeWidget::onValueChanged()
{
  Q_EMIT valueChanged(value());
}
}  // namespace tobas::gui::sa
