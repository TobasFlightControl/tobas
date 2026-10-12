// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/slider_display.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include "tobas_qt_tools/font.hpp"

namespace tobas
{
namespace qt
{
namespace
{
constexpr int kTextPointSize = 9;
}  // namespace

IntSliderDisplay::IntSliderDisplay(QWidget* _parent) : super(_parent)
{
  const DefaultFont font(kTextPointSize, QFont::Bold);

  text_ = new QLabel();
  text_->setFont(font);

  value_ = new QLineEdit();
  value_->setAlignment(Qt::AlignRight);
  value_->setFont(font);
  value_->setReadOnly(true);
  value_->setFocusPolicy(Qt::NoFocus);

  slider_ = new Slider(Qt::Horizontal);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(text_);
  cols->addWidget(value_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(slider_);

  setLayout(rows);

  // Connection
  connect(slider_, &Slider::valueChanged, this, &self::onSliderValueChanged);
}

int IntSliderDisplay::getValue() const
{
  return slider_->value();
}

int IntSliderDisplay::getMinimum() const
{
  return slider_->minimum();
}

int IntSliderDisplay::getMaximum() const
{
  return slider_->maximum();
}

QString IntSliderDisplay::getText() const
{
  return text_->text();
}

QString IntSliderDisplay::getSuffix() const
{
  return suffix_;
}

void IntSliderDisplay::setValue(int _value, bool _block_signal)
{
  const QSignalBlocker block(slider_);
  slider_->setValue(_value);

  const auto value = slider_->value();
  value_->setText(QString::number(value) + suffix_);

  if (!_block_signal) {
    Q_EMIT valueChanged(value);
  }
}

void IntSliderDisplay::setMinimum(int _minimum)
{
  slider_->setMinimum(_minimum);
}

void IntSliderDisplay::setMaximum(int _maximum)
{
  slider_->setMaximum(_maximum);
}

void IntSliderDisplay::setRange(int _minimum, int _maximum)
{
  setMinimum(_minimum);
  setMaximum(_maximum);
}

void IntSliderDisplay::setText(const QString& _text)
{
  text_->setText(_text);
}

void IntSliderDisplay::setSuffix(const QString& _suffix)
{
  suffix_ = _suffix;
  updateValueText(getValue());
}

void IntSliderDisplay::updateValueText(int _value)
{
  value_->setText(QString::number(_value) + suffix_);
}

void IntSliderDisplay::onSliderValueChanged(int _value)
{
  updateValueText(_value);
  Q_EMIT valueChanged(_value);
}

DoubleSliderDisplay::DoubleSliderDisplay(QWidget* _parent) : super(_parent)
{
  const DefaultFont font(kTextPointSize, QFont::Bold);

  text_ = new QLabel();
  text_->setFont(font);

  value_ = new QLineEdit();
  value_->setAlignment(Qt::AlignRight);
  value_->setFont(font);
  value_->setReadOnly(true);
  value_->setFocusPolicy(Qt::NoFocus);

  slider_ = new DoubleSlider(Qt::Horizontal);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(text_);
  cols->addWidget(value_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(slider_);

  setLayout(rows);

  // Connection
  connect(slider_, &DoubleSlider::valueChanged, this, &self::onSliderValueChanged);
}

double DoubleSliderDisplay::getValue() const
{
  return slider_->value();
}

double DoubleSliderDisplay::getMinimum() const
{
  return slider_->minimum();
}

double DoubleSliderDisplay::getMaximum() const
{
  return slider_->maximum();
}

QString DoubleSliderDisplay::getText() const
{
  return text_->text();
}

QString DoubleSliderDisplay::getSuffix() const
{
  return suffix_;
}

int DoubleSliderDisplay::getDecimals() const
{
  return decimals_;
}

void DoubleSliderDisplay::setValue(double _value, bool _block_signal)
{
  const QSignalBlocker block(slider_);
  slider_->setValue(_value);

  const auto value = slider_->value();
  value_->setText(QString::number(value, 'f', decimals_) + suffix_);

  if (!_block_signal) {
    Q_EMIT valueChanged(value);
  }
}

void DoubleSliderDisplay::setMinimum(double _minimum)
{
  slider_->setMinimum(_minimum);
}

void DoubleSliderDisplay::setMaximum(double _maximum)
{
  slider_->setMaximum(_maximum);
}

void DoubleSliderDisplay::setRange(double _minimum, double _maximum)
{
  setMinimum(_minimum);
  setMaximum(_maximum);
}

void DoubleSliderDisplay::setText(const QString& _text)
{
  text_->setText(_text);
}

void DoubleSliderDisplay::setSuffix(const QString& _suffix)
{
  suffix_ = _suffix;
  updateValueText(getValue());
}

void DoubleSliderDisplay::setDecimals(int _decimals)
{
  decimals_ = _decimals;
  updateValueText(getValue());
}

void DoubleSliderDisplay::updateValueText(double _value)
{
  value_->setText(QString::number(_value, 'f', decimals_) + suffix_);
}

void DoubleSliderDisplay::onSliderValueChanged(double _value)
{
  updateValueText(_value);
  Q_EMIT valueChanged(_value);
}
}  // namespace qt
}  // namespace tobas
