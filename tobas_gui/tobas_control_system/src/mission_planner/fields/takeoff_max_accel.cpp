#include "tobas_control_system/mission_planner/fields/takeoff_max_accel.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
TakeoffMaxAccelWidget::TakeoffMaxAccelWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_ACC_UP_MAX
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(1.);
  spin_box_->setMaximum(15.);
  spin_box_->setValue(4.);
  spin_box_->setSuffix(" m/s²");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* TakeoffMaxAccelWidget::label() const
{
  return "Maximum Accel";
}

double TakeoffMaxAccelWidget::getValue() const
{
  return spin_box_->value();
}

void TakeoffMaxAccelWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
