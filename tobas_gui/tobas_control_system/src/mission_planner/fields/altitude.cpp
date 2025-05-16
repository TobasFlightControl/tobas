#include "tobas_control_system/mission_planner/fields/altitude.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace gcs
{
namespace field
{
AltitudeWidget::AltitudeWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(2);
  spinbox_->setValue(5.);
  spinbox_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spinbox_);

  connect(spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* AltitudeWidget::label() const
{
  return "Altitude";
}

double AltitudeWidget::value() const
{
  return spinbox_->value();
}

void AltitudeWidget::setValue(double value)
{
  spinbox_->setValue(value);
}
}  // namespace field
}  // namespace gcs
}  // namespace gui
