#include "tobas_control_system/mission_planner/fields/land_speed.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
LandSpeedWidget::LandSpeedWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_LAND_SPEED
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(0.6);
  spin_box_->setMaximum(2.);
  spin_box_->setValue(0.7);
  spin_box_->setSuffix(" m/s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* LandSpeedWidget::label() const
{
  return "Descending Speed";
}

double LandSpeedWidget::value() const
{
  return spin_box_->value();
}

void LandSpeedWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
