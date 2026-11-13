// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./slider.hpp"

namespace tobas::qt
{
class DoubleSlider : public Slider
{
  Q_OBJECT

  using self = DoubleSlider;
  using super = Slider;

Q_SIGNALS:
  void valueChanged(double _value);

public:
  explicit DoubleSlider(Qt::Orientation _orientation, QWidget* _parent = nullptr);

  double minimum() const;
  void setMinimum(double _minimum);

  double maximum() const;
  void setMaximum(double _maximum);

  double value() const;
  void setValue(double _value);

  void setRange(double _minimum, double _maximum);

private Q_SLOTS:
  void onSliderValueChanged(int _slider_value);

private:
  double min_ = 0.0;
  double max_ = 1.0;

  double valueFromSlider(int _slider_value) const;
};
}  // namespace tobas::qt
