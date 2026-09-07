// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/slider_text.hpp"

#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QIntValidator>

namespace tobas
{
namespace qt
{
namespace
{
constexpr int kValueWidth = 100;
}  // namespace

IntSliderTextWidget::IntSliderTextWidget(int minimum, int maximum, QWidget* parent) : super(parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  cols->addWidget(new QLabel(QString::number(minimum)));

  slider_ = new Slider(Qt::Horizontal);
  slider_->setRange(minimum, maximum);
  connect(slider_, &Slider::valueChanged, this, &self::onSliderValueChanged);
  cols->addWidget(slider_);

  cols->addWidget(new QLabel(QString::number(maximum)));

  line_edit_ = new QLineEdit();
  line_edit_->setFixedWidth(kValueWidth);
  line_edit_->setAlignment(Qt::AlignRight);
  line_edit_->setValidator(new QIntValidator(minimum, maximum));
  connect(line_edit_, &QLineEdit::returnPressed, this, &self::onLineEditReturnPressed);
  cols->addWidget(line_edit_);
}

int IntSliderTextWidget::get() const
{
  return slider_->value();
}

void IntSliderTextWidget::set(int value)
{
  setSliderValue(value);
  setLineEditText(value);
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

void IntSliderTextWidget::setSliderValue(int value)
{
  const QSignalBlocker block(slider_);
  slider_->setValue(value);
}

void IntSliderTextWidget::setLineEditText(int value)
{
  const QSignalBlocker block(line_edit_);
  line_edit_->setText(QString::number(value));
}

DoubleSliderTextWidget::DoubleSliderTextWidget(double minimum, double maximum, int decimals, QWidget* parent)
  : super(parent), decimals_(decimals)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  cols->addWidget(new QLabel(QString::number(minimum, 'f', decimals)));

  slider_ = new DoubleSlider(Qt::Horizontal);
  slider_->setRange(minimum, maximum);
  connect(slider_, &Slider::valueChanged, this, &self::onSliderValueChanged);
  cols->addWidget(slider_);

  cols->addWidget(new QLabel(QString::number(maximum, 'f', decimals)));

  line_edit_ = new QLineEdit();
  line_edit_->setFixedWidth(kValueWidth);
  line_edit_->setAlignment(Qt::AlignRight);
  line_edit_->setValidator(new QDoubleValidator(minimum, maximum, decimals));
  connect(line_edit_, &QLineEdit::returnPressed, this, &self::onLineEditReturnPressed);
  cols->addWidget(line_edit_);
}

double DoubleSliderTextWidget::get() const
{
  return slider_->value();
}

void DoubleSliderTextWidget::set(double value)
{
  setSliderValue(value);
  setLineEditText(value);
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

void DoubleSliderTextWidget::setSliderValue(double value)
{
  const QSignalBlocker block(slider_);
  slider_->setValue(value);
}

void DoubleSliderTextWidget::setLineEditText(double value)
{
  const QSignalBlocker block(line_edit_);
  line_edit_->setText(QString::number(value, 'f', decimals_));
}
}  // namespace qt
}  // namespace tobas
