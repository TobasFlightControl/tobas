// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/color.hpp"

namespace tobas
{
namespace qt
{
QString toCssColor(Qt::GlobalColor c, bool with_alpha)
{
  if (c == Qt::transparent) {
    return "transparent";  // Prioritize readability.
  }

  QColor q(c);

  if (with_alpha) {
    return q.name(QColor::HexArgb);  // #AARRGGBB
  }
  else {
    return q.name(QColor::HexRgb);  // #RRGGBB
  }
}

namespace color
{
namespace
{
QColor grayByPercentage(int percentage)
{
  // Add half of the divisor to round to the nearest integer.
  const auto value = (percentage * 255 + 50) / 100;
  return QColor(value, value, value);
}
}  // namespace

QColor gray10()
{
  return grayByPercentage(10);
}

QColor gray20()
{
  return grayByPercentage(20);
}

QColor gray30()
{
  return grayByPercentage(30);
}

QColor gray40()
{
  return grayByPercentage(40);
}

QColor gray50()
{
  return grayByPercentage(50);
}

QColor gray60()
{
  return grayByPercentage(60);
}

QColor gray70()
{
  return grayByPercentage(70);
}

QColor gray80()
{
  return grayByPercentage(80);
}

QColor gray90()
{
  return grayByPercentage(90);
}

QColor lightRed()
{
  return QColor(245, 205, 205);
}

QColor lightGreen()
{
  return QColor(210, 245, 210);
}

QColor lightYellow()
{
  return QColor(255, 240, 190);
}

QColor lightBlue()
{
  return QColor(220, 236, 255);
}

QColor steelBlue()
{
  return QColor(70, 130, 180);
}
}  // namespace color
}  // namespace qt
}  // namespace tobas
