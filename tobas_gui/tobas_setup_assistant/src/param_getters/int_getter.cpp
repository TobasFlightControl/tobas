// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/int_getter.hpp"

#include <QHBoxLayout>
#include <QLabel>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_string_tools/core.hpp>

namespace tobas::gui::sa
{
IntGetter::IntGetter(const QString& _name)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto label = new QLabel(_name + ":");
  label->setFont(qt::DefaultFont(cmn::kBodyPSize));
  cols->addWidget(label);

  data_ = new qt::SpinBox();
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  cols->addWidget(data_);

  connect(data_, qOverload<int>(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

int IntGetter::getValue() const
{
  return data_->value();
}

bool IntGetter::setValue(int _value)
{
  if (_value < data_->minimum() || data_->maximum() < _value) {
    return false;
  }

  data_->setValue(_value);
  return true;
}

void IntGetter::setMinimum(int _minimum)
{
  data_->setMinimum(_minimum);
}

void IntGetter::setMaximum(int _maximum)
{
  data_->setMaximum(_maximum);
}

void IntGetter::setSingleStep(int _single_step)
{
  data_->setSingleStep(_single_step);
}

void IntGetter::setSuffix(const QString& _suffix)
{
  data_->setSuffix(QString::fromStdString(str::convertToSuperscript(_suffix.toStdString())));
}

void IntGetter::onValueChanged(int _value)
{
  Q_EMIT valueChanged(_value);
}
}  // namespace tobas::gui::sa
