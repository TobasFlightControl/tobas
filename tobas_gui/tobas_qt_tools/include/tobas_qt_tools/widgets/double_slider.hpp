#pragma once

#include "./slider.hpp"

namespace qt
{
class DoubleSlider : public Slider
{
  Q_OBJECT

  using self = DoubleSlider;
  using super = Slider;

  static constexpr int kRange = 10000;
  static constexpr double kDefaultMinimum = 0.;
  static constexpr double kDefaultMaximum = 1.;

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
  double min_ = kDefaultMinimum;
  double max_ = kDefaultMaximum;

  double valueFromSlider(int slider_value) const;
};
}  // namespace qt
