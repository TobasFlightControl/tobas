// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/widget.hpp"

#include <QDebug>
#include <QPainter>

namespace tobas
{
namespace qt
{
QPoint Widget::getCenter() const
{
  return QPoint(width() / 2, height() / 2);
}

int Widget::calcMaxTextPointSize(const QString& _text, const QPoint& _center) const
{
  if (_text.isEmpty()) {
    qWarning() << "Text is empty.";
    return -1;
  }

  int point_size = 1;
  auto font = this->font();

  while (true) {
    font.setPointSize(point_size++);
    QFontMetrics fm(font);

    const auto text_width = fm.horizontalAdvance(_text);
    const auto text_height = fm.height();

    constexpr double kMargin = 0.1;
    if (_center.x() - text_width / 2 < width() * kMargin || width() * (1 - kMargin) < _center.x() + text_width / 2) {
      break;
    }
    if (_center.y() - text_height / 2 < height() * kMargin || height() * (1 - kMargin) < _center.y() + text_height / 2) {
      break;
    }
  }

  return point_size;
}

void Widget::setBackgroundColor(QPalette::ColorRole _cr)
{
  setAutoFillBackground(true);
  auto pal = palette();
  pal.setColor(QPalette::Window, pal.color(_cr));
}

void Widget::drawText(QPainter& _painter, const QString& _text, const QPoint& _center, const QFont& _font)
{
  _painter.save();

  _painter.setFont(_font);
  QFontMetrics fm(_font);

  const auto text_width = fm.horizontalAdvance(_text);
  const auto text_height = fm.height();
  const auto x = _center.x() - text_width / 2;
  const auto y = _center.y() + text_height / 2 - fm.descent();  // Baseline correction.
  _painter.drawText(x, y, _text);

  _painter.restore();
}

void Widget::drawText(QPainter& _painter, const QString& _text, const QPoint& _center, int _point_size)
{
  auto font = this->font();
  font.setPointSize(_point_size);
  drawText(_painter, _text, _center, font);
}

void Widget::drawMaximumText(QPainter& _painter, const QString& _text, const QPoint& _center)
{
  if (_text.isEmpty()) {
    return;
  }

  const auto point_size = calcMaxTextPointSize(_text, _center);
  drawText(_painter, _text, _center, point_size);
}

void Widget::drawMaximumText(QPainter& _painter, const QString& _text)
{
  drawMaximumText(_painter, _text, getCenter());
}
}  // namespace qt
}  // namespace tobas
