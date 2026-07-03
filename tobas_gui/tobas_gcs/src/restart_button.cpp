// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/restart_button.hpp"

#include <QGraphicsDropShadowEffect>

namespace tobas
{
namespace gui
{
namespace gcs
{
RestartButton::RestartButton(int radius) : super("↻")
{
  setCheckable(true);

  const auto font_size = radius * 3 / 2;
  const auto diameter = radius * 2;

  // Set the button appearance with a style sheet.
  QString qss = R"(
    QPushButton {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #66FF66, stop:1 #009900);
      border: 2px solid #004400;
      border-radius: %1px;
      color: white;
      font-size: %2px;
      width: %3px;
      height: %3px;
    }
    QPushButton:hover {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #99FF99, stop:1 #00CC00);
    }
    QPushButton:pressed, QPushButton:checked {
      background-color: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 #006600, stop:1 #003300);
      padding-top: 2px;
    }
    QPushButton:disabled {
      background-color: qlineargradient(
        spread:pad,
        x1:0.5,
        y1:0,
        x2:0.5,
        y2:1,
        stop:0 rgba(102,255,102,0.5),
        stop:1 rgba(0,153,0,0.5));
      border-color: rgba(0,0,0,0.5);
      color: rgba(255,255,255,0.5);
    }
  )";
  qss = qss.arg(radius).arg(font_size).arg(diameter);
  setStyleSheet(qss);
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
