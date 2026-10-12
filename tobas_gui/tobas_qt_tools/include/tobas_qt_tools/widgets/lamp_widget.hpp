// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

#include "tobas_qt_tools/rgb_color.hpp"

namespace tobas
{
namespace qt
{
class LampWidget : public QLabel
{
  Q_OBJECT

  using super = QLabel;

public:
  explicit LampWidget(QWidget* _parent = nullptr);
  explicit LampWidget(const QString& _text, QWidget* _parent = nullptr);

  void setColor(const RGBColor& _color);

private:
  RGBColor c_ = RGBColor::Black();

  void draw();
};
}  // namespace qt
}  // namespace tobas
