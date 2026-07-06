// SPDX-License-Identifier: GPL-3.-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rcin_viewer/stick_panel.hpp"

#include <cmath>

#include <QPainter>
#include <QPalette>

#include <tobas_qt_tools/color.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace rcin
{
StickPanel::StickPanel(const QString& title, const QString& x_label, const QString& y_label)
  : title_(title), x_label_(x_label), y_label_(y_label)
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

void StickPanel::reset()
{
  setValues(0.0, 0.0, false, false);
}

void StickPanel::setValues(double x, double y, bool ok, bool enabled)
{
  x_ = x;
  y_ = y;
  ok_ = ok;
  enabled_ = enabled;
  update();
}

void StickPanel::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const auto frame = rect().adjusted(0, 0, -1, -1);
  painter.setPen(QPen(qt::color::gray70(), 1));
  painter.setBrush(qt::color::gray90());
  painter.drawRect(frame);

  painter.setPen(palette().color(QPalette::WindowText));
  painter.drawText(QRect(0, 4, width(), 18), Qt::AlignHCenter | Qt::AlignVCenter, title_);

  const auto field = fieldRect();
  painter.setPen(QPen(qt::color::gray80(), 1));
  painter.setBrush(Qt::white);
  painter.drawRect(field);

  painter.setPen(QPen(qt::color::gray80(), 1));
  painter.drawLine(field.center().x(), field.top(), field.center().x(), field.bottom());
  painter.drawLine(field.left(), field.center().y(), field.right(), field.center().y());

  constexpr int kMarkerRadius = 7;
  const auto marker_x = field.center().x() + x_ * (field.width() - kMarkerRadius * 2) / 2;
  const auto marker_y = field.center().y() - y_ * (field.height() - kMarkerRadius * 2) / 2;
  const QPointF marker_pos(marker_x, marker_y);

  painter.setPen(QPen(enabled_ ? qt::color::steelBlue() : qt::color::gray50(), 1));
  painter.setBrush(ok_ ? (enabled_ ? qt::color::lightBlue() : qt::color::gray90()) : Qt::lightGray);
  painter.drawEllipse(marker_pos, kMarkerRadius, kMarkerRadius);

  const auto text_y = height() - 22;
  painter.setPen(qt::color::gray20());
  painter.drawText(
    QRect(12, text_y, width() / 2 - 12, 18), Qt::AlignLeft | Qt::AlignVCenter, x_label_ + ": " + axisText(x_));
  painter.drawText(
    QRect(width() / 2, text_y, width() / 2 - 12, 18), Qt::AlignRight | Qt::AlignVCenter, y_label_ + ": " + axisText(y_));
}

QRect StickPanel::fieldRect() const
{
  return rect().adjusted(12, 28, -12, -30);
}

QString StickPanel::axisText(double value) const
{
  if (!ok_) {
    return "---";
  }

  return QString::number(std::lround(value * 100.0));
}
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
