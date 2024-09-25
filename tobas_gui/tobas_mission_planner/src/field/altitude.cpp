#include <QHBoxLayout>

#include "tobas_mission_planner/fields/altitude.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
AltitudeWidget::AltitudeWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(3);
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
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
