#include "tobas_control_system/mission_planner/fields/acceptance_radius.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
AcceptanceRadiusWidget::AcceptanceRadiusWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#NAV_ACC_RAD
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setMinimum(0.05);
  spin_box_->setMaximum(200.);
  spin_box_->setValue(10.);
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* AcceptanceRadiusWidget::label() const
{
  return "Acceptance Radius";
}

double AcceptanceRadiusWidget::getValue() const
{
  return spin_box_->value();
}

void AcceptanceRadiusWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
