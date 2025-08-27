#include "tobas_control_system/mission_planner/fields/acceptance_radius.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
AcceptanceRadiusWidget::AcceptanceRadiusWidget()
{
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setMinimum(1e-2);
  spin_box_->setValue(1.);
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* AcceptanceRadiusWidget::label() const
{
  return "Acceptance Radius";
}

double AcceptanceRadiusWidget::value() const
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
