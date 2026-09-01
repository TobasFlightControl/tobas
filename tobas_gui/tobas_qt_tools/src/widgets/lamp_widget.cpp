// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/lamp_widget.hpp"

#include <format>

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
  constexpr char QSS[] = "QLabel {{"
                         "border: 2px solid lightgray;"
                         "border-radius: {}px;"
                         "background-color:"
                         "QLinearGradient("
                         "y1: 0, y2: 1,"
                         "stop: 0 WHITE,"
                         "stop: 0.2 #{:02X}{:02X}{:02X},"
                         "stop: 0.8 #{:02X}{:02X}{:02X},"
                         "stop: 1 #{:02X}{:02X}{:02X}"
                         ");"
                         "}}";

  const auto radius = sizeHint().height() / 2;
  const auto grad = c_.mean(RGBColor::White());
  const auto qss = std::format(QSS, radius, grad.r, grad.g, grad.b, c_.r, c_.g, c_.b, c_.r, c_.g, c_.b);
  setStyleSheet(QString::fromStdString(qss));
}
}  // namespace qt
}  // namespace tobas
