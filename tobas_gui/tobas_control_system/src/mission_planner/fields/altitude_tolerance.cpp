#include <QHBoxLayout>

#include "tobas_control_system/mission_planner/fields/altitude_tolerance.hpp"

namespace gui
{
namespace gcs
{
namespace field
{
AltitudeToleranceWidget::AltitudeToleranceWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(2);
  spinbox_->setMinimum(1e-2);
  spinbox_->setValue(1.);
  spinbox_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spinbox_);

  connect(spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* AltitudeToleranceWidget::label() const
{
  return "Altitude Tolerance";
}

double AltitudeToleranceWidget::value() const
{
  return spinbox_->value();
}

void AltitudeToleranceWidget::setValue(double value)
{
  spinbox_->setValue(value);
}
}  // namespace field
}  // namespace gcs
}  // namespace gui
