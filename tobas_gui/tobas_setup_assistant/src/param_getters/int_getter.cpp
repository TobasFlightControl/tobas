// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/int_getter.hpp"

#include <QHBoxLayout>
#include <QLabel>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_string_tools/core.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
IntGetter::IntGetter(const QString& name)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto label = new QLabel(name + ":");
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

bool IntGetter::setValue(const int& value)
{
  if (value < data_->minimum() || data_->maximum() < value) {
    return false;
  }

  data_->setValue(value);
  return true;
}

void IntGetter::setMinimum(int minimum)
{
  data_->setMinimum(minimum);
}

void IntGetter::setMaximum(int maximum)
{
  data_->setMaximum(maximum);
}

void IntGetter::setSingleStep(int single_step)
{
  data_->setSingleStep(single_step);
}

void IntGetter::setSuffix(const QString& suffix)
{
  data_->setSuffix(QString::fromStdString(str::convertToSuperscript(suffix.toStdString())));
}

void IntGetter::onValueChanged(int value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
