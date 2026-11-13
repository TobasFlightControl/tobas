// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QRect>
#include <QString>
#include <QWidget>

namespace tobas::gui::ctrl::rcin
{
class StickPanel : public QWidget
{
public:
  explicit StickPanel(const QString& _title, const QString& _x_label, const QString& _y_label);

  void reset();
  void setValues(double _x, double _y, bool _ok, bool _enabled);

protected:
  void paintEvent(QPaintEvent* _event) override;

private:
  QString title_;
  QString x_label_;
  QString y_label_;

  double x_ = 0.0;
  double y_ = 0.0;
  bool ok_ = false;
  bool enabled_ = false;

  QRect fieldRect() const;
  QString axisText(double _value) const;
};
}  // namespace tobas::gui::ctrl::rcin
