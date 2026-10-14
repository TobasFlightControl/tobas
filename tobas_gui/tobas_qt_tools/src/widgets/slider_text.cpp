// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/slider_text.hpp"

#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QLabel>

namespace tobas
{
namespace qt
{
namespace
{
constexpr int kValueWidth = 100;
}  // namespace

IntSliderTextWidget::IntSliderTextWidget(int _minimum, int _maximum, QWidget* _parent) : super(_parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  cols->addWidget(new QLabel(QString::number(_minimum)));

  slider_ = new Slider(Qt::Horizontal);
  slider_->setRange(_minimum, _maximum);
  connect(slider_, &Slider::valueChanged, this, &self::onSliderValueChanged);
  cols->addWidget(slider_);

  cols->addWidget(new QLabel(QString::number(_maximum)));

  line_edit_ = new QLineEdit();
  line_edit_->setFixedWidth(kValueWidth);
  line_edit_->setAlignment(Qt::AlignRight);
  line_edit_->setValidator(new QIntValidator(_minimum, _maximum));
  connect(line_edit_, &QLineEdit::returnPressed, this, &self::onLineEditReturnPressed);
  cols->addWidget(line_edit_);
}

int IntSliderTextWidget::get() const
{
  return slider_->value();
}

void IntSliderTextWidget::set(int _value)
{
  setSliderValue(_value);
  setLineEditText(_value);
}

void IntSliderTextWidget::onSliderValueChanged()
{
  const auto value = slider_->value();
  setLineEditText(value);
  Q_EMIT valueChanged(value);
}

void IntSliderTextWidget::onLineEditReturnPressed()
{
  const auto value = line_edit_->text().toInt();
  setSliderValue(value);
  Q_EMIT valueChanged(value);
}

void IntSliderTextWidget::setSliderValue(int _value)
{
  const QSignalBlocker block(slider_);
  slider_->setValue(_value);
}

void IntSliderTextWidget::setLineEditText(int _value)
{
  const QSignalBlocker block(line_edit_);
  line_edit_->setText(QString::number(_value));
}

DoubleSliderTextWidget::DoubleSliderTextWidget(double _minimum, double _maximum, int _decimals, QWidget* _parent)
  : super(_parent), decimals_(_decimals)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  cols->addWidget(new QLabel(QString::number(_minimum, 'f', _decimals)));

  slider_ = new DoubleSlider(Qt::Horizontal);
  slider_->setRange(_minimum, _maximum);
  connect(slider_, &Slider::valueChanged, this, &self::onSliderValueChanged);
  cols->addWidget(slider_);

  cols->addWidget(new QLabel(QString::number(_maximum, 'f', _decimals)));

  line_edit_ = new QLineEdit();
  line_edit_->setFixedWidth(kValueWidth);
  line_edit_->setAlignment(Qt::AlignRight);
  line_edit_->setValidator(new QDoubleValidator(_minimum, _maximum, _decimals));
  connect(line_edit_, &QLineEdit::returnPressed, this, &self::onLineEditReturnPressed);
  cols->addWidget(line_edit_);
}

double DoubleSliderTextWidget::get() const
{
  return slider_->value();
}

void DoubleSliderTextWidget::set(double _value)
{
  setSliderValue(_value);
  setLineEditText(_value);
}

void DoubleSliderTextWidget::onSliderValueChanged()
{
  const auto value = slider_->value();
  setLineEditText(value);
  Q_EMIT valueChanged(value);
}

void DoubleSliderTextWidget::onLineEditReturnPressed()
{
  const auto value = line_edit_->text().toDouble();
  setSliderValue(value);
  Q_EMIT valueChanged(value);
}

void DoubleSliderTextWidget::setSliderValue(double _value)
{
  const QSignalBlocker block(slider_);
  slider_->setValue(_value);
}

void DoubleSliderTextWidget::setLineEditText(double _value)
{
  const QSignalBlocker block(line_edit_);
  line_edit_->setText(QString::number(_value, 'f', decimals_));
}
}  // namespace qt
}  // namespace tobas
