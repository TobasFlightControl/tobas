// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/label.hpp"

#include "tobas_qt_tools/font.hpp"

#include <QColor>

namespace tobas
{
namespace qt
{
namespace
{
QString toCssColor(Qt::GlobalColor c)
{
  if (c == Qt::transparent) {
    return "transparent";
  }
  else {
    return QColor(c).name(QColor::HexRgb);
  }
}
}  // namespace

Label::Label(const QString& text, int point_size, int weight, bool italic, QWidget* parent) : super(text, parent)
{
  setFont(DefaultFont(point_size, weight, italic));
}

void Label::setTextColor(const QString& color)
{
  setStyleSheet("color: " + color + ";");
}

void Label::setTextColor(Qt::GlobalColor color)
{
  setTextColor(toCssColor(color));
}
}  // namespace qt
}  // namespace tobas
