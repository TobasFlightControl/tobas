// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/labeled_double_spin_box.hpp"

#include <QHBoxLayout>
#include <QLabel>

namespace tobas::qt
{
LabeledDoubleSpinBox::LabeledDoubleSpinBox(const QString& _label_text)
{
  const auto cols = new QHBoxLayout(this);

  const auto label = new QLabel(_label_text + ":");
  cols->addWidget(label);

  data_ = new qt::DoubleSpinBox();
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  cols->addWidget(data_);

  connect(data_, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

double LabeledDoubleSpinBox::getValue() const
{
  return data_->value();
}

bool LabeledDoubleSpinBox::setValue(double _value)
{
  if (_value < data_->minimum() || data_->maximum() < _value) {
    return false;
  }

  data_->setValue(_value);
  return true;
}

void LabeledDoubleSpinBox::setDecimals(int _decimals)
{
  data_->setDecimals(_decimals);
}

void LabeledDoubleSpinBox::setMinimum(double _minimum)
{
  data_->setMinimum(_minimum);
}

void LabeledDoubleSpinBox::setMaximum(double _maximum)
{
  data_->setMaximum(_maximum);
}

void LabeledDoubleSpinBox::setSingleStep(double _single_step)
{
  data_->setSingleStep(_single_step);
}

void LabeledDoubleSpinBox::setSuffix(const QString& _suffix)
{
  data_->setSuffix(_suffix);
}

void LabeledDoubleSpinBox::onValueChanged(double _value)
{
  Q_EMIT valueChanged(_value);
}
}  // namespace tobas::qt
