#include <QHBoxLayout>

#include "tobas_mission_planner/fields/altitude_tolerance.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
AltitudeToleranceWidget::AltitudeToleranceWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(3);
  spinbox_->setMinimum(1e-3);
  spinbox_->setValue(0.1);
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
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
