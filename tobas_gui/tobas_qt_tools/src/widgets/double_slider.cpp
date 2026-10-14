// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/double_slider.hpp"

#include <tobas_math/core.hpp>

namespace tobas::qt
{
namespace
{
constexpr int kRange = 10000;
}  // namespace

DoubleSlider::DoubleSlider(Qt::Orientation _orientation, QWidget* _parent) : super(_orientation, _parent)
{
  super::setRange(0, kRange);
  super::setValue(kRange / 2);

  connect(this, &super::valueChanged, this, &self::onSliderValueChanged);
}

double DoubleSlider::minimum() const
{
  return min_;
}

void DoubleSlider::setMinimum(double _minimum)
{
  min_ = _minimum;
}

double DoubleSlider::maximum() const
{
  return max_;
}

void DoubleSlider::setMaximum(double _maximum)
{
  max_ = _maximum;
}

double DoubleSlider::value() const
{
  const auto slider_value = super::value();
  return valueFromSlider(slider_value);
}

void DoubleSlider::setValue(double _value)
{
  const int slider_value = math::remap<double>(_value, min_, max_, 0, kRange);
  super::setValue(slider_value);
}

void DoubleSlider::setRange(double _minimum, double _maximum)
{
  setMinimum(_minimum);
  setMaximum(_maximum);
}

void DoubleSlider::onSliderValueChanged(int _slider_value)
{
  const auto value = valueFromSlider(_slider_value);
  Q_EMIT valueChanged(value);
}

double DoubleSlider::valueFromSlider(int _slider_value) const
{
  return math::remap<double>(_slider_value, 0, kRange, min_, max_);
}
}  // namespace tobas::qt
