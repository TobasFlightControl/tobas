// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/label.hpp"

#include "tobas_qt_tools/font.hpp"

#include <QColor>

namespace tobas::qt
{
namespace
{
QString toCssColor(Qt::GlobalColor _c)
{
  if (_c == Qt::transparent) {
    return "transparent";
  }
  else {
    return QColor(_c).name(QColor::HexRgb);
  }
}
}  // namespace

Label::Label(const QString& _text, int _point_size, QFont::Weight _weight, bool _italic, QWidget* _parent)
  : super(_text, _parent)
{
  setFont(DefaultFont(_point_size, _weight, _italic));
}

void Label::setTextColor(const QString& _color)
{
  setStyleSheet("color: " + _color + ";");
}

void Label::setTextColor(Qt::GlobalColor _color)
{
  setTextColor(toCssColor(_color));
}
}  // namespace tobas::qt
