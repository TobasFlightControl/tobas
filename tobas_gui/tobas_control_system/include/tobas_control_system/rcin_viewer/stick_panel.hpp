// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QRect>
#include <QString>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace rcin
{
class StickPanel : public QWidget
{
  using super = QWidget;

public:
  explicit StickPanel(const QString& title, const QString& x_label, const QString& y_label);

  void reset();
  void setValues(double x, double y, bool ok, bool enabled);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  QString title_;
  QString x_label_;
  QString y_label_;

  double x_ = 0.0;
  double y_ = 0.0;
  bool ok_ = false;
  bool enabled_ = false;

  QRect fieldRect() const;
  QString axisText(double value) const;
};
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
