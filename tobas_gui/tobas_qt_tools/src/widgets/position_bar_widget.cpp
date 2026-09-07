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
PositionBarWidget::PositionBarWidget(QWidget* parent) : super(parent)
{
}

PositionBarWidget::PositionBarWidget(double minimum, double maximum, QWidget* parent)
  : super(parent), minimum_(minimum), maximum_(maximum)
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

void PositionBarWidget::setMinimum(double minimum)
{
  minimum_ = minimum;
  update();
}

void PositionBarWidget::setMaximum(double maximum)
{
  maximum_ = maximum;
  update();
}

void PositionBarWidget::setLineWidth(int line_width)
{
  line_width_ = line_width;
  update();
}

void PositionBarWidget::setTextPSize(int text_psize)
{
  text_psize_ = text_psize;
  update();
}

void PositionBarWidget::setFillColor(const QColor& color)
{
  fill_color_ = color;
  update();
}

void PositionBarWidget::setLimitLineColor(const QColor& color)
{
  limit_line_color_ = color;
  update();
}

void PositionBarWidget::setValueLineColor(const QColor& color)
{
  value_line_color_ = color;
  update();
}

void PositionBarWidget::setCenterText(const QString& text)
{
  center_text_ = text;
  update();
}

void PositionBarWidget::setLowerText(const QString& text)
{
  lower_text_ = text;
  update();
}

void PositionBarWidget::setUpperText(const QString& text)
{
  upper_text_ = text;
  update();
}

void PositionBarWidget::setValue(double value)
{
  value_ = value;
  update();
}

void PositionBarWidget::setLower(double lower)
{
  lower_ = lower;
  update();
}

void PositionBarWidget::setUpper(double upper)
{
  upper_ = upper;
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

void PositionBarWidget::paintEvent(QPaintEvent* event)
{
  // `QPainter` can only be defined inside `paintEvent`.
  QPainter painter(this);

  // Draw the background.
  painter.fillRect(event->rect(), Qt::white);

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

void HPositionBarWidget::drawRange(QPainter& painter, double lower, double upper)
{
  // Compute the bar position.
  const int lower_pos = math::remap<double>(lower, getMinimum(), getMaximum(), 0, width());
  const int upper_pos = math::remap<double>(upper, getMinimum(), getMaximum(), 0, width());

  // Fill between the minimum and maximum values.
  painter.setBrush(fill_color_);
  painter.drawRect(lower_pos, 0, upper_pos - lower_pos, height());

  // Draw black lines at the minimum and maximum value positions.
  painter.setPen(QPen(limit_line_color_, getLineWidth()));
  painter.drawLine(lower_pos, 0, lower_pos, height());
  painter.drawLine(upper_pos, 0, upper_pos, height());
}

void HPositionBarWidget::drawValue(QPainter& painter, double value)
{
  // Compute the bar position.
  const int value_pos = math::remap<double>(value, getMinimum(), getMaximum(), 0, width());

  // Draw a red line at the current value position.
  painter.setPen(QPen(value_line_color_, getLineWidth()));
  painter.drawLine(value_pos, 0, value_pos, height());
}

void HPositionBarWidget::drawCenterText(QPainter& painter, const QString& text)
{
  drawTextCommon(painter);
  painter.drawText(QRect(0, 0, width(), height()), Qt::AlignHCenter | Qt::AlignVCenter, text);
}

void HPositionBarWidget::drawLowerText(QPainter& painter, const QString& text)
{
  drawTextCommon(painter);
  painter.drawText(QRect(0, 0, width(), height()), Qt::AlignLeft | Qt::AlignVCenter, "     " + text);
}

void HPositionBarWidget::drawUpperText(QPainter& painter, const QString& text)
{
  drawTextCommon(painter);
  painter.drawText(QRect(0, 0, width(), height()), Qt::AlignRight | Qt::AlignVCenter, text + "     ");
}

void HPositionBarWidget::drawTextCommon(QPainter& painter)
{
  painter.setPen(Qt::gray);
  painter.setFont(DefaultFont(getTextPSize()));
}

void VPositionBarWidget::drawRange(QPainter& painter, double lower, double upper)
{
  // Compute the bar position.
  const int lower_pos = math::remap<double>(lower, getMinimum(), getMaximum(), 0, height());
  const int upper_pos = math::remap<double>(upper, getMinimum(), getMaximum(), 0, height());

  // Fill between the minimum and maximum values.
  painter.setBrush(fill_color_);
  painter.drawRect(0, lower_pos, width(), upper_pos - lower_pos);

  // Draw black lines at the minimum and maximum value positions.
  painter.setPen(QPen(limit_line_color_, getLineWidth()));
  painter.drawLine(0, lower_pos, width(), lower_pos);
  painter.drawLine(0, upper_pos, width(), upper_pos);
}

void VPositionBarWidget::drawValue(QPainter& painter, double value)
{
  // Compute the bar position.
  const int value_pos = math::remap<double>(value, getMinimum(), getMaximum(), 0, height());

  // Draw a red line at the current value position.
  painter.setPen(QPen(value_line_color_, getLineWidth()));
  painter.drawLine(0, value_pos, width(), value_pos);
}

void VPositionBarWidget::drawCenterText(QPainter& painter, const QString& text)
{
  drawTextCommon(painter);
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  painter.drawText(text_rect, Qt::AlignHCenter | Qt::AlignVCenter, text);
}

void VPositionBarWidget::drawLowerText(QPainter& painter, const QString& text)
{
  drawTextCommon(painter);
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  painter.drawText(text_rect, Qt::AlignLeft | Qt::AlignVCenter, "     " + text);
}

void VPositionBarWidget::drawUpperText(QPainter& painter, const QString& text)
{
  drawTextCommon(painter);
  QRect text_rect(-height() / 2, -width() / 2, height(), width());
  painter.drawText(text_rect, Qt::AlignRight | Qt::AlignVCenter, text + "     ");
}

void VPositionBarWidget::drawTextCommon(QPainter& painter)
{
  // Set the font.
  painter.setPen(Qt::gray);
  painter.setFont(DefaultFont(getTextPSize()));

  // Set painter rotation and translation.
  painter.translate(width() / 2, height() / 2);
  painter.rotate(90);
}
}  // namespace qt
}  // namespace tobas
