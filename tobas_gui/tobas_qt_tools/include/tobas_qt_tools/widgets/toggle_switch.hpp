// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./widget.hpp"

namespace tobas::qt
{
class ToggleSwitch : public qt::Widget
{
  Q_OBJECT

  using super = qt::Widget;

Q_SIGNALS:
  void toggled(bool _checked);

public:
  explicit ToggleSwitch(QWidget* _parent = nullptr);

  bool isChecked() const;
  const QString& getText() const;
  int getTextPointSize() const;
  const QColor& getOnColor() const;
  const QColor& getOffColor() const;

  void setChecked(bool _checked);
  void setText(const QString& _text);
  void setTextPointSize(int _point_size);
  void setOnColor(const QColor& _color);
  void setOffColor(const QColor& _color);

  void ignoreMousePressEvent(bool _ignore);

  QPoint getTextCenter() const;
  int calcMaxTextPointSize() const;

protected:
  void paintEvent(QPaintEvent* _event) override;
  void mousePressEvent(QMouseEvent* _event) override;
  void resizeEvent(QResizeEvent* _event) override;

private:
  bool checked_ = false;  ///< ON/OFF state.
  bool ignore_mouse_press_event_ = false;
  QString text_ = "";
  int text_psize_ = 0;
  QColor on_color_ = Qt::green;
  QColor off_color_ = Qt::gray;

  void drawBackground(QPainter& _painter);
  void drawSwitch(QPainter& _painter);
  void drawText(QPainter& _painter);
};
}  // namespace tobas::qt
