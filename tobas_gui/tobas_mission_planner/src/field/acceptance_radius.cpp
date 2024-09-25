#include <QHBoxLayout>

#include "tobas_mission_planner/fields/acceptance_radius.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
AcceptanceRadiusWidget::AcceptanceRadiusWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(3);
  spinbox_->setMinimum(1e-3);
  spinbox_->setValue(1.0);
  spinbox_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spinbox_);

  connect(spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* AcceptanceRadiusWidget::label() const
{
  return "Acceptance Radius";
}

double AcceptanceRadiusWidget::value() const
{
  return spinbox_->value();
}
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
