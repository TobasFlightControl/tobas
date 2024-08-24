#include <QWheelEvent>

#include <tobas_math/core.hpp>

#include "tobas_qt_tools/widgets/slider.hpp"

namespace qt
{
void Slider::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

DoubleSlider::DoubleSlider(Qt::Orientation orientation, QWidget* parent) : super(orientation, parent)
{
  setRange(0, kRange);
  setValue(kRange / 2);

  connect(this, SIGNAL(super::valueChanged(int)), this, SLOT(onSliderValueChanged(int)));
}

double DoubleSlider::minimum() const
{
  return min_;
}

void DoubleSlider::setMinimum(double minimum)
{
  min_ = minimum;
}

double DoubleSlider::maximum() const
{
  return max_;
}

void DoubleSlider::setMaximum(double maximum)
{
  max_ = maximum;
}

double DoubleSlider::value() const
{
  const auto slider_value = super::value();
  return valueFromSlider(slider_value);
}

void DoubleSlider::setValue(double value)
{
  const int slider_value = math::remap<double>(value, min_, max_, 0, kRange);
  super::setValue(slider_value);
}

void DoubleSlider::setRange(double minimum, double maximum)
{
  setMinimum(minimum);
  setMaximum(maximum);
}

void DoubleSlider::onSliderValueChanged(int slider_value)
{
  const auto value = valueFromSlider(slider_value);
  Q_EMIT valueChanged(value);
}

double DoubleSlider::valueFromSlider(int slider_value) const
{
  return math::remap<double>(slider_value, 0, kRange, min_, max_);
}
}  // namespace qt
