// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/position_bar_widget.hpp"

#include <QPaintEvent>
#include <QPainter>

#include <tobas_math/core.hpp>

#include "tobas_qt_tools/font.hpp"

namespace tobas
{
namespace qt
{
PositionBarWidget::PositionBarWidget(QWidget* _parent) : super(_parent)
{
}

PositionBarWidget::PositionBarWidget(double _minimum, double _maximum, QWidget* _parent)
  : super(_parent), minimum_(_minimum), maximum_(_maximum)
{
}

double PositionBarWidget::getMinimum() const
{
  return minimum_;
}

double PositionBarWidget::getMaximum() const
{
  return maximum_;
}

int PositionBarWidget::getLineWidth() const
{
  return line_width_;
}

int PositionBarWidget::getTextPSize() const
{
  return text_psize_;
}

std::optional<QString> PositionBarWidget::getCenterText() const
{
  return center_text_;
}

std::optional<QString> PositionBarWidget::getLowerText() const
{
  return lower_text_;
}

std::optional<QString> PositionBarWidget::getUpperText() const
{
  return upper_text_;
}

std::optional<double> PositionBarWidget::getValue() const
{
  return value_;
}

std::optional<double> PositionBarWidget::getLower() const
{
  return lower_;
}

std::optional<double> PositionBarWidget::getUpper() const
{
  return upper_;
}

std::optional<double> PositionBarWidget::getMiddle() const
{
  if (lower_ && upper_) {
    return (*lower_ + *upper_) / 2;
  }
  else {
    return std::nullopt;
  }
}

std::optional<double> PositionBarWidget::getRange() const
{
  if (lower_ && upper_) {
    return *upper_ - *lower_;
  }
  else {
    return std::nullopt;
  }
}

void PositionBarWidget::setMinimum(double _minimum)
{
  minimum_ = _minimum;
  update();
}

void PositionBarWidget::setMaximum(double _maximum)
{
  maximum_ = _maximum;
  update();
}

void PositionBarWidget::setLineWidth(int _line_width)
{
  line_width_ = _line_width;
  update();
}

void PositionBarWidget::setTextPSize(int _text_psize)
{
  text_psize_ = _text_psize;
  update();
}

void PositionBarWidget::setFillColor(const QColor& _color)
{
  fill_color_ = _color;
  update();
}

void PositionBarWidget::setLimitLineColor(const QColor& _color)
{
  limit_line_color_ = _color;
  update();
}

void PositionBarWidget::setValueLineColor(const QColor& _color)
{
  value_line_color_ = _color;
  update();
}

void PositionBarWidget::setCenterText(const QString& _text)
{
  center_text_ = _text;
  update();
}

void PositionBarWidget::setLowerText(const QString& _text)
{
  lower_text_ = _text;
  update();
}

void PositionBarWidget::setUpperText(const QString& _text)
{
  upper_text_ = _text;
  update();
}

void PositionBarWidget::setValue(double _value)
{
  value_ = _value;
  update();
}

void PositionBarWidget::setLower(double _lower)
{
  lower_ = _lower;
  update();
}

void PositionBarWidget::setUpper(double _upper)
{
  upper_ = _upper;
  update();
}

void PositionBarWidget::clear()
{
  center_text_.reset();
  lower_text_.reset();
  upper_text_.reset();
  value_.reset();
  lower_.reset();
  upper_.reset();
  update();
}

void PositionBarWidget::updateRangeFromValue()
{
  if (!value_) {
    return;
  }

  if (!lower_ || value_.value() < lower_.value()) {
    lower_ = value_;
  }
  if (!upper_ || value_.value() > upper_.value()) {
    upper_ = value_;
  }

  update();
}

void PositionBarWidget::paintEvent(QPaintEvent* _event)
{
  // `QPainter` can only be defined inside `paintEvent`.
  QPainter painter(this);

  // Draw the background.
  painter.fillRect(_event->rect(), Qt::white);

  // Draw the frame.
  painter.save();
  painter.setPen(Qt::black);
  painter.drawRect(0, 0, width(), height());
  painter.restore();

  // Fill the value range.
  if (lower_ && upper_) {
    painter.save();
    drawRange(painter, lower_.value(), upper_.value());
    painter.restore();
  }

  // Show the value position.
  if (value_) {
    painter.save();
    drawValue(painter, value_.value());
    painter.restore();
  }

  // Show text.
  if (center_text_) {
    painter.save();
    drawCenterText(painter, center_text_.value());
    painter.restore();
  }
  if (lower_text_) {
    painter.save();
    drawLowerText(painter, lower_text_.value());
    painter.restore();
  }
  if (upper_text_) {
    painter.save();
    drawUpperText(painter, upper_text_.value());
    painter.restore();
  }
}

void HPositionBarWidget::drawRange(QPainter& _painter, double _lower, double _upper)
{
  // Compute the bar position.
  const int lower_pos = math::remap<double>(_lower, getMinimum(), getMaximum(), 0, width());
  const int upper_pos = math::remap<double>(_upper, getMinimum(), getMaximum(), 0, width());

  // Fill between the minimum and maximum values.
  _painter.setBrush(fill_color_);
  _painter.drawRect(lower_pos, 0, upper_pos - lower_pos, height());

  // Draw black lines at the minimum and maximum value positions.
  _painter.setPen(QPen(limit_line_color_, getLineWidth()));
  _painter.drawLine(lower_pos, 0, lower_pos, height());
  _painter.drawLine(upper_pos, 0, upper_pos, height());
}

void HPositionBarWidget::drawValue(QPainter& _painter, double _value)
{
  // Compute the bar position.
  const int value_pos = math::remap<double>(_value, getMinimum(), getMaximum(), 0, width());

  // Draw a red line at the current value position.
  _painter.setPen(QPen(value_line_color_, getLineWidth()));
  _painter.drawLine(value_pos, 0, value_pos, height());
}

void HPositionBarWidget::drawCenterText(QPainter& _painter, const QString& _text)
{
  drawTextCommon(_painter);
  _painter.drawText(QRect(0, 0, width(), height()), Qt::AlignHCenter | Qt::AlignVCenter, _text);
}

void HPositionBarWidget::drawLowerText(QPainter& _painter, const QString& _text)
{
  drawTextCommon(_painter);
  _painter.drawText(QRect(0, 0, width(), height()), Qt::AlignLeft | Qt::AlignVCenter, "     " + _text);
}

void HPositionBarWidget::drawUpperText(QPainter& _painter, const QString& _text)
{
  drawTextCommon(_painter);
  _painter.drawText(QRect(0, 0, width(), height()), Qt::AlignRight | Qt::AlignVCenter, _text + "     ");
}

void HPositionBarWidget::drawTextCommon(QPainter& _painter)
{
  _painter.setPen(Qt::gray);
  _painter.setFont(DefaultFont(getTextPSize()));
}

void VPositionBarWidget::drawRange(QPainter& _painter, double _lower, double _upper)
{
  // Compute the bar position.
  const int lower_pos = math::remap<double>(_lower, getMinimum(), getMaximum(), 0, height());
  const int upper_pos = math::remap<double>(_upper, getMinimum(), getMaximum(), 0, height());

  // Fill between the minimum and maximum values.
  _painter.setBrush(fill_color_);
  _painter.drawRect(0, lower_pos, width(), upper_pos - lower_pos);

  // Draw black lines at the minimum and maximum value positions.
  _painter.setPen(QPen(limit_line_color_, getLineWidth()));
  _painter.drawLine(0, lower_pos, width(), lower_pos);
  _painter.drawLine(0, upper_pos, width(), upper_pos);
}

void VPositionBarWidget::drawValue(QPainter& _painter, double _value)
{
  // Compute the bar position.
  const int value_pos = math::remap<double>(_value, getMinimum(), getMaximum(), 0, height());

  // Draw a red line at the current value position.
  _painter.setPen(QPen(value_line_color_, getLineWidth()));
  _painter.drawLine(0, value_pos, width(), value_pos);
}

void VPositionBarWidget::drawCenterText(QPainter& _painter, const QString& _text)
{
  drawTextCommon(_painter);
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  _painter.drawText(text_rect, Qt::AlignHCenter | Qt::AlignVCenter, _text);
}

void VPositionBarWidget::drawLowerText(QPainter& _painter, const QString& _text)
{
  drawTextCommon(_painter);
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  _painter.drawText(text_rect, Qt::AlignLeft | Qt::AlignVCenter, "     " + _text);
}

void VPositionBarWidget::drawUpperText(QPainter& _painter, const QString& _text)
{
  drawTextCommon(_painter);
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  _painter.drawText(text_rect, Qt::AlignRight | Qt::AlignVCenter, _text + "     ");
}

void VPositionBarWidget::drawTextCommon(QPainter& _painter)
{
  // Set the font.
  _painter.setPen(Qt::gray);
  _painter.setFont(DefaultFont(getTextPSize()));

  // Set painter rotation and translation.
  _painter.translate(width() / 2, height() / 2);
  _painter.rotate(90);
}
}  // namespace qt
}  // namespace tobas
