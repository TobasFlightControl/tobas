#include "tobas_control_system/mission_planner/fields/latitude.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace gcs
{
namespace field
{
LatitudeWidget::LatitudeWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(9);
  spinbox_->setMinimum(-90.);
  spinbox_->setMaximum(90.);
  spinbox_->setValue(0.);
  spinbox_->setSuffix(" deg");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spinbox_);

  connect(spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* LatitudeWidget::label() const
{
  return "Latitude";
}

double LatitudeWidget::value() const
{
  return spinbox_->value();
}

void LatitudeWidget::setValue(double value)
{
  spinbox_->setValue(value);
}
}  // namespace field
}  // namespace gcs
}  // namespace gui
