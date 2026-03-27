#include "tobas_control_system/mission_planner/fields/latitude.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
LatitudeWidget::LatitudeWidget()
{
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(9);
  spin_box_->setMinimum(-90.);
  spin_box_->setMaximum(90.);
  spin_box_->setValue(0.);
  spin_box_->setSuffix(" deg");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* LatitudeWidget::label() const
{
  return "Latitude";
}

double LatitudeWidget::getValue() const
{
  return spin_box_->value();
}

void LatitudeWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
