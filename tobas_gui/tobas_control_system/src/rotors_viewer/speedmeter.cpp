// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rotors_viewer/speedmeter.hpp"

#include <cmath>

#include <QVBoxLayout>

#include <tobas_qt_tools/color.hpp>

namespace tobas::gui::ctrl
{
SpeedmeterWidget::SpeedmeterWidget()
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

  top_text_ = new QLabel();
  top_text_->setAlignment(Qt::AlignCenter);

  gauge_ = new QwtThermo();
  gauge_->setOrientation(Qt::Vertical);
  gauge_->setScale(0.0, 100.0);
  gauge_->setPipeWidth(50);
  gauge_->setFillBrush(qt::color::cyan500());

  bottom_text_ = new qt::FramedLabel();

  const auto rows = new QVBoxLayout();
  rows->addWidget(top_text_, 0);
  rows->addWidget(gauge_, 1);
  rows->addWidget(bottom_text_, 0);
  setLayout(rows);

  setAutoFillBackground(true);
  setBackgroundColor("transparent");
}

QString SpeedmeterWidget::getBackgroundColor() const
{
  return palette().color(QPalette::Window).name(QColor::HexArgb);
}

double SpeedmeterWidget::getMaximumValue() const
{
  return gauge_->upperBound();
}

double SpeedmeterWidget::getMinimumValue() const
{
  return gauge_->lowerBound();
}

double SpeedmeterWidget::getValue() const
{
  return value_;
}

QString SpeedmeterWidget::getTopText() const
{
  return top_text_->text();
}

QString SpeedmeterWidget::getBottomText() const
{
  return bottom_text_->text();
}

void SpeedmeterWidget::setBackgroundColor(const QString& _text)
{
  auto background_palette = palette();
  background_palette.setColor(QPalette::Window, QColor(_text));
  setPalette(background_palette);
}

void SpeedmeterWidget::setMaximumValue(double _value)
{
  gauge_->setUpperBound(_value);
}

void SpeedmeterWidget::setMinimumValue(double _value)
{
  gauge_->setLowerBound(_value);
  updateGaugeValue();
}

void SpeedmeterWidget::setValue(double _value)
{
  value_ = _value;
  updateGaugeValue();
}

void SpeedmeterWidget::setTopText(const QString& _text)
{
  top_text_->setText(_text);
}

void SpeedmeterWidget::setBottomText(const QString& _text)
{
  bottom_text_->setText(_text);
}

void SpeedmeterWidget::updateGaugeValue()
{
  gauge_->setValue(value_);
}
}  // namespace tobas::gui::ctrl
