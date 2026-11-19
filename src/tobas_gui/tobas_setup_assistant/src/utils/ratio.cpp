// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/utils/ratio.hpp"

#include <QHBoxLayout>

namespace tobas::gui::sa
{
RatioWidget::RatioWidget(QWidget* _parent) : super(_parent)
{
  left_value_ = new qt::SpinBox();
  right_value_ = new qt::SpinBox();
  left_text_ = new QLabel();
  right_text_ = new QLabel();

  const auto cols = new QHBoxLayout(this);
  cols->setContentsMargins(0, 0, 0, 0);
  cols->addWidget(left_text_);
  cols->addWidget(left_value_);
  cols->addWidget(new QLabel(" : "));
  cols->addWidget(right_value_);
  cols->addWidget(right_text_);
  cols->addStretch(1);

  setFocusProxy(left_value_);
  connect(left_value_, qOverload<int>(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
  connect(right_value_, qOverload<int>(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

std::pair<int, int> RatioWidget::value() const
{
  return { left_value_->value(), right_value_->value() };
}

void RatioWidget::setValue(const std::pair<int, int>& _value)
{
  left_value_->setValue(_value.first);
  right_value_->setValue(_value.second);
}

void RatioWidget::setMinimum(int _minimum)
{
  left_value_->setMinimum(_minimum);
  right_value_->setMinimum(_minimum);
}

void RatioWidget::setMaximum(int _maximum)
{
  left_value_->setMaximum(_maximum);
  right_value_->setMaximum(_maximum);
}

void RatioWidget::setLeftText(const QString& _text)
{
  left_text_->setText(_text);
}

void RatioWidget::setRightText(const QString& _text)
{
  right_text_->setText(_text);
}

void RatioWidget::onValueChanged()
{
  Q_EMIT valueChanged(value());
}
}  // namespace tobas::gui::sa
