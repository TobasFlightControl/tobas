// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/ratio.hpp"

namespace tobas::gui::sa
{
ParamGetterWidget_Ratio::ParamGetterWidget_Ratio(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  left_value_ = new qt::SpinBox();
  right_value_ = new qt::SpinBox();
  left_text_ = new QLabel();
  right_text_ = new QLabel();

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  cols->addWidget(left_text_);
  cols->addWidget(left_value_);
  cols->addWidget(new QLabel(" : "));
  cols->addWidget(right_value_);
  cols->addWidget(right_text_);
  cols->addStretch();

  connect(left_value_, qOverload<int>(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
  connect(right_value_, qOverload<int>(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

ParamGetterWidget_Ratio::ValueType ParamGetterWidget_Ratio::getValue() const
{
  return { left_value_->value(), right_value_->value() };
}

bool ParamGetterWidget_Ratio::setValue(const ValueType& _src)
{
  left_value_->setValue(_src.first);
  right_value_->setValue(_src.second);

  return true;
}

void ParamGetterWidget_Ratio::setMinimum(int _minimum)
{
  left_value_->setMinimum(_minimum);
  right_value_->setMinimum(_minimum);
}

void ParamGetterWidget_Ratio::setMaximum(int _maximum)
{
  left_value_->setMinimum(_maximum);
  right_value_->setMinimum(_maximum);
}

void ParamGetterWidget_Ratio::setLeftText(const QString& _text)
{
  left_text_->setText(_text);
}

void ParamGetterWidget_Ratio::setRightText(const QString& _text)
{
  right_text_->setText(_text);
}

void ParamGetterWidget_Ratio::onValueChanged()
{
  Q_EMIT valueChanged(getValue());
}
}  // namespace tobas::gui::sa
