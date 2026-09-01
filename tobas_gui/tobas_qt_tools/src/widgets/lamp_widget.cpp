// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/lamp_widget.hpp"

namespace tobas
{
namespace qt
{
LampWidget::LampWidget(QWidget* parent) : super(parent)
{
  draw();
}

LampWidget::LampWidget(const QString& text, QWidget* parent) : super(text, parent)
{
  draw();
}

void LampWidget::setColor(const RGBColor& color)
{
  // Do nothing if the color does not change.
  if (color == c_) {
    return;
  }

  // Update to the new color.
  c_ = color;

  // Draw.
  draw();
}

void LampWidget::draw()
{
  const auto qss = QStringLiteral("QLabel {"
                                  "border: 2px solid lightgray;"
                                  "border-radius: %1px;"
                                  "background-color:"
                                  "QLinearGradient("
                                  "y1: 0, y2: 1,"
                                  "stop: 0 WHITE,"
                                  "stop: 0.2 #%2%3%4,"
                                  "stop: 0.8 #%5%6%7,"
                                  "stop: 1 #%8%9%10"
                                  ");"
                                  "}");

  const auto radius = sizeHint().height() / 2;
  const auto grad = c_.mean(RGBColor::White());
  setStyleSheet(qss.arg(radius)
                  .arg(grad.r, 2, 16, QLatin1Char('0'))
                  .arg(grad.g, 2, 16, QLatin1Char('0'))
                  .arg(grad.b, 2, 16, QLatin1Char('0'))
                  .arg(c_.r, 2, 16, QLatin1Char('0'))
                  .arg(c_.g, 2, 16, QLatin1Char('0'))
                  .arg(c_.b, 2, 16, QLatin1Char('0'))
                  .arg(c_.r, 2, 16, QLatin1Char('0'))
                  .arg(c_.g, 2, 16, QLatin1Char('0'))
                  .arg(c_.b, 2, 16, QLatin1Char('0')));
}
}  // namespace qt
}  // namespace tobas
