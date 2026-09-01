// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./slider.hpp"

namespace tobas
{
namespace qt
{
class DoubleSlider : public Slider
{
  Q_OBJECT

  using self = DoubleSlider;
  using super = Slider;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit DoubleSlider(Qt::Orientation orientation, QWidget* parent = nullptr);

  double minimum() const;
  void setMinimum(double minimum);

  double maximum() const;
  void setMaximum(double maximum);

  double value() const;
  void setValue(double value);

  void setRange(double minimum, double maximum);

private Q_SLOTS:
  void onSliderValueChanged(int slider_value);

private:
  double min_ = 0.0;
  double max_ = 1.0;

  double valueFromSlider(int slider_value) const;
};
}  // namespace qt
}  // namespace tobas
