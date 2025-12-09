#include "tobas_control_system/mission_planner/fields/altitude_tolerance.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
AltitudeToleranceWidget::AltitudeToleranceWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#NAV_MC_ALT_RAD
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setMinimum(0.05);
  spin_box_->setMaximum(200.);
  spin_box_->setValue(0.8);
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* AltitudeToleranceWidget::label() const
{
  return "Altitude Tolerance";
}

double AltitudeToleranceWidget::value() const
{
  return spin_box_->value();
}

void AltitudeToleranceWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
